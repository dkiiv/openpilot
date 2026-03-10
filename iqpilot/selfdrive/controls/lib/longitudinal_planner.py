"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
from datetime import datetime

from cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.iqpilot.selfdrive.controls.lib.iq_dynamic.controller import IQDynamicController
from openpilot.iqpilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.iqpilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.iqpilot.selfdrive.controls.lib.slc_vcruise import SLCVCruise
from openpilot.iqpilot.selfdrive.selfdrived.events import IQEvents
from openpilot.iqpilot.models.helpers import get_active_bundle

IQDynamicState = custom.IQPlan.IQDynamicControl.IQDynamicControlState
LongitudinalPlanSource = custom.IQPlan.LongitudinalPlanSource
SpeedLimitAssistState = custom.IQPlan.SpeedLimit.AssistState
SpeedLimitSource = custom.IQPlan.SpeedLimit.Source


class LongitudinalPlannerIQ:
  def __init__(self, CP: structs.CarParams, CP_IQ: structs.IQCarParams, mpc):
    self.events_iq = IQEvents()
    self.iq_dynamic = IQDynamicController(CP, mpc)
    self.scc = SmartCruiseControl()
    self.slc = SLCVCruise()
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()

    self.output_v_target = 0.
    self.output_a_target = 0.
    self.speed_limit_last = 0.
    self.speed_limit_final_last = 0.
    self.speed_limit_source = SpeedLimitSource.none

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.iq_dynamic.active():
      return experimental_mode

    return experimental_mode and self.iq_dynamic.mode() == "blended"

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    # IQ.Pilot custom Speed Limit Controller
    now = datetime.now()
    if hasattr(sm, "alive"):
      time_validated = sm.alive.get('clocks', False) and getattr(sm['clocks'], 'timeValid', False)
    else:
      clocks = sm.get('clocks', None) if isinstance(sm, dict) else None
      time_validated = bool(getattr(clocks, 'timeValid', False))
    slc_v_cruise = self.slc.update(long_enabled, now, time_validated, v_cruise, v_ego, sm)
    # Prefer confirmed controller output for UI/planner rendering.
    # Fall back to active (policy-resolved) target/source when confirmed is unavailable.
    display_speed_limit = self.slc.slc_target if self.slc.slc_target > 0 else self.slc.slc_active_target
    display_source = self.slc.slc_source if self.slc.slc_source != "None" else self.slc.slc_active_source

    self.speed_limit_last = display_speed_limit if display_speed_limit > 0 else self.speed_limit_last
    self.speed_limit_final_last = display_speed_limit + self.slc.slc_offset if display_speed_limit > 0 else self.speed_limit_final_last
    source_map = {
      "Dashboard": SpeedLimitSource.car,
      "Map Data": SpeedLimitSource.map,
      "Mapbox": SpeedLimitSource.map,
      "None": SpeedLimitSource.none,
    }
    self.speed_limit_source = source_map.get(display_source, SpeedLimitSource.none)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (slc_v_cruise, a_ego),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    return self.output_v_target, self.output_a_target

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_iq.clear()
    self.iq_dynamic.update(sm)
    self.e2e_alerts_helper.update(sm, self.events_iq)

  def publish_longitudinal_plan_iq(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    def fill_plan(plan_msg) -> None:
      plan_msg.longitudinalPlanSource = self.source
      plan_msg.vTarget = float(self.output_v_target)
      plan_msg.aTarget = float(self.output_a_target)
      plan_msg.events = self.events_iq.to_msg()

      # IQ.Dynamic control state
      iq_dynamic = plan_msg.iqDynamic
      iq_dynamic.state = IQDynamicState.blended if self.iq_dynamic.mode() == 'blended' else IQDynamicState.acc
      iq_dynamic.enabled = self.iq_dynamic.enabled()
      iq_dynamic.active = self.iq_dynamic.active()

      # Smart Cruise Control
      smartCruiseControl = plan_msg.smartCruiseControl
      # Vision Control
      sccVision = smartCruiseControl.vision
      sccVision.state = self.scc.vision.state
      sccVision.vTarget = float(self.scc.vision.output_v_target)
      sccVision.aTarget = float(self.scc.vision.output_a_target)
      sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
      sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
      sccVision.enabled = self.scc.vision.is_enabled
      sccVision.active = self.scc.vision.is_active
      # Map Control
      sccMap = smartCruiseControl.map
      sccMap.state = self.scc.map.state
      sccMap.vTarget = float(self.scc.map.output_v_target)
      sccMap.aTarget = float(self.scc.map.output_a_target)
      sccMap.enabled = self.scc.map.is_enabled
      sccMap.active = self.scc.map.is_active

      # Speed Limit
      speedLimit = plan_msg.speedLimit
      resolver = speedLimit.resolver
      speed_limit = float(self.slc.slc_target if self.slc.slc_target > 0 else self.slc.slc_active_target)
      speed_limit_offset = float(self.slc.slc_offset)
      speed_limit_final = speed_limit + speed_limit_offset if speed_limit > 0 else 0.
      speed_limit_valid = speed_limit > 0.
      speed_limit_last_valid = self.speed_limit_last > 0.

      resolver.speedLimit = speed_limit
      resolver.speedLimitLast = float(self.speed_limit_last)
      resolver.speedLimitFinal = float(speed_limit_final)
      resolver.speedLimitFinalLast = float(self.speed_limit_final_last)
      resolver.speedLimitValid = speed_limit_valid
      resolver.speedLimitLastValid = speed_limit_last_valid
      resolver.speedLimitOffset = speed_limit_offset
      resolver.distToSpeedLimit = 0.
      resolver.source = self.speed_limit_source

      assist = speedLimit.assist
      assist.enabled = bool(self.slc.slc_target > 0 or self.slc.slc_unconfirmed > 0)
      assist.active = self.source == LongitudinalPlanSource.speedLimitAssist and self.slc.slc_target > 0
      if not assist.enabled:
        assist.state = SpeedLimitAssistState.disabled
      elif self.slc.slc_unconfirmed > 0:
        assist.state = SpeedLimitAssistState.preActive
      elif assist.active:
        assist.state = SpeedLimitAssistState.active
      else:
        assist.state = SpeedLimitAssistState.inactive
      assist.vTarget = float(self.output_v_target if assist.active else 255.)
      assist.aTarget = float(self.output_a_target if assist.active else 0.)

      # E2E Alerts
      e2eAlerts = plan_msg.e2eAlerts
      e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
      e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    valid = sm.all_checks(service_list=['carState', 'controlsState'])

    plan_iq_send = messaging.new_message('iqPlan')
    plan_iq_send.valid = valid
    fill_plan(plan_iq_send.iqPlan)
    pm.send('iqPlan', plan_iq_send)

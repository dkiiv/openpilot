"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
import sys
import os
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.lateral import apply_std_curvature_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mlbcan, mqbcan, pqcan, mebcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags
from opendbc.car.volkswagen.mebutils import LongControlJerk, LongControlLimit, LatControlCurvature

iqpilot_path = os.path.join(os.path.dirname(__file__), '..', '..', '..')
sys.path.insert(0, iqpilot_path)
try:
  from openpilot.common.params import Params
except ImportError:
  pass

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_IQ):
    super().__init__(dbc_names, CP, CP_IQ)
    self._params = Params()
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MLB:
      self.CCS = mlbcan
    elif CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO):
      self.CCS = mebcan
    else:
      self.CCS = mqbcan

    self.accel = 0
    self.apply_torque_last = 0
    self.apply_curvature_last = 0.
    self.steering_power_last = 0
    self.long_jerk_control = LongControlJerk(dt=(DT_CTRL * self.CCP.ACC_CONTROL_STEP)) if self.CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO) else None
    self.long_limit_control = LongControlLimit(dt=(DT_CTRL * self.CCP.ACC_CONTROL_STEP)) if self.CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO) else None
    self.gra_acc_counter_last = None
    self.klr_counter_last = None
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0
    self.accel_last = 0
    self.accel_diff = 0
    self.long_deviation = 0
    self.long_jerklimit = 0
    self.HCA_Status = 3
    self.leadDistanceBars = 0
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0
    self.speed_limit_last = 0
    self.speed_limit_changed_timer = 0
    self.blinkerActive = None
    self.hide_ea_error = False
    self.eps_timer_workaround = bool(CP.flags & VolkswagenFlags.MLB)
    self.hca_frame_timer_resetting = 0
    self.hca_frame_low_torque = 0
    self.long_override_counter = 0
    self.long_disabled_counter = 0
    self.LateralController = (
      LatControlCurvature(self.CCP.CURVATURE_PID, self.CCP.CURVATURE_LIMITS.CURVATURE_MAX, 1 / (DT_CTRL * self.CCP.STEER_STEP))
      if (CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO))
      else None
    )

  def update(self, CC, CC_IQ, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []
    output_torque = 0
    apply_torque = 0
    pqhca5or7Toggle = self._params.get_bool("pqhca5or7Toggle")

    CS.force_rhd_for_bsm = getattr(CC, "forceRHDForBSM", False)
    CS.enable_predicative_speed_limit = getattr(CC.cruiseControl, "speedLimitPredicative", False)
    CS.enable_pred_react_to_speed_limits = getattr(CC.cruiseControl, "speedLimitPredReactToSL", False)
    CS.enable_pred_react_to_curves = getattr(CC.cruiseControl, "speedLimitPredReactToCurves", False)

    if self.frame % self.CCP.STEER_STEP == 0:
      if self.CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO):
        if CC.latActive:
          hca_enabled = True
          if CC.curvatureControllerActive:
            apply_curvature = self.LateralController.update(CS.out, CC, actuators.curvature)
            apply_curvature = apply_curvature + (CS.out.steeringCurvature - (CC.currentCurvature - CC.rollCompensation))
          else:
            apply_curvature = actuators.curvature + (CS.out.steeringCurvature - CC.currentCurvature)
          apply_curvature = apply_std_curvature_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw, CS.out.steeringCurvature,
                                                       CS.out.steeringPressed, self.CCP.STEER_STEP, CC.latActive, self.CCP.CURVATURE_LIMITS)

          min_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MIN)
          max_power = min(self.steering_power_last + self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MAX)
          target_power_driver = int(np.interp(CS.out.steeringTorque, [self.CCP.STEER_DRIVER_ALLOWANCE, self.CCP.STEER_DRIVER_MAX],
                                                                     [self.CCP.STEERING_POWER_MAX, self.CCP.STEERING_POWER_MIN]))
          target_power = int(np.interp(CS.out.vEgo, [0., 0.5], [self.CCP.STEERING_POWER_MIN, target_power_driver]))
          steering_power = min(max(target_power, min_power), max_power)
        else:
          if self.LateralController is not None:
            self.LateralController.reset()
          if self.steering_power_last > 0:
            hca_enabled = True
            apply_curvature = np.clip(CS.out.steeringCurvature, -self.CCP.CURVATURE_LIMITS.CURVATURE_MAX, self.CCP.CURVATURE_LIMITS.CURVATURE_MAX)
            steering_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, 0)
          else:
            hca_enabled = False
            apply_curvature = 0.
            steering_power = 0

        can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_curvature, hca_enabled, steering_power))
        self.apply_curvature_last = apply_curvature
        self.steering_power_last = steering_power
      else:
        if CC.latActive:
          new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
          apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)
          self.hca_frame_timer_running += self.CCP.STEER_STEP
          if self.apply_torque_last == apply_torque:
            self.hca_frame_same_torque += self.CCP.STEER_STEP
            if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
              apply_torque -= (1, -1)[apply_torque < 0]
              self.hca_frame_same_torque = 0
          else:
            self.hca_frame_same_torque = 0
          hca_enabled = abs(apply_torque) > 0
          if self.eps_timer_workaround and self.hca_frame_timer_running >= self.CCP.STEER_TIME_BM / DT_CTRL:
            if abs(apply_torque) <= self.CCP.STEER_LOW_TORQUE:
              self.hca_frame_low_torque += self.CCP.STEER_STEP
              if self.hca_frame_low_torque >= self.CCP.STEER_TIME_LOW_TORQUE / DT_CTRL:
                hca_enabled = False
            else:
              self.hca_frame_low_torque = 0
              if self.hca_frame_timer_resetting > 0:
                apply_torque = 0
        else:
          self.hca_frame_low_torque = 0
          hca_enabled = False
          apply_torque = 0

        if hca_enabled:
          output_torque = apply_torque
          self.hca_frame_timer_resetting = 0
        else:
          output_torque = 0
          self.hca_frame_timer_resetting += self.CCP.STEER_STEP
          if self.hca_frame_timer_resetting >= self.CCP.STEER_TIME_RESET / DT_CTRL or not self.eps_timer_workaround:
            self.hca_frame_timer_running = 0
            apply_torque = 0

        if hca_enabled and abs(apply_torque) > 0:
          if pqhca5or7Toggle and (self.CP.flags & (VolkswagenFlags.PQ | VolkswagenFlags.MLB)):
            self.HCA_Status = 7
          else:
            self.HCA_Status = 5
        else:
          self.HCA_Status = 3

        self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
        self.apply_torque_last = apply_torque
        can_sends.append(self.CCS.create_hca_steering_control(self.packer_pt, self.CAN.pt, output_torque, self.HCA_Status))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))

    if self.CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO) and self.CP.flags & VolkswagenFlags.STOCK_KLR_PRESENT:
      if CS.klr_stock_values:
        klr_send_ready = CS.klr_stock_values["COUNTER"] != self.klr_counter_last
        if klr_send_ready:
          can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.cam, CC.latActive, CS.klr_stock_values))
          can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.pt, CC.latActive, CS.klr_stock_values))
        self.klr_counter_last = CS.klr_stock_values["COUNTER"]

    if self.CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO):
      if self.frame % 2 == 0:
        blinker_active = CS.left_blinker_active or CS.right_blinker_active
        left_blinker = CC.leftBlinker if not blinker_active else False
        right_blinker = CC.rightBlinker if not blinker_active else False
        can_sends.append(self.CCS.create_blinker_control(self.packer_pt, self.CAN.pt, CS.ea_hud_stock_values, CS.ea_control_stock_values,
                                                         left_blinker, right_blinker, self.hide_ea_error))

    if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl:
      stopping = actuators.longControlState == LongCtrlState.stopping
      if self.CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO):
        starting = actuators.longControlState == LongCtrlState.starting and CS.out.vEgo <= self.CP.vEgoStarting
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.enabled else 0)

        long_override = CC.cruiseControl.override or CS.out.gasPressed
        self.long_override_counter = min(self.long_override_counter + 1, 5) if long_override else 0
        long_override_begin = long_override and self.long_override_counter < 5

        self.long_disabled_counter = min(self.long_disabled_counter + 1, 5) if not CC.enabled else 0
        long_disabling = not CC.enabled and self.long_disabled_counter < 5

        critical_state = hud_control.visualAlert == VisualAlert.fcw
        if CC.longComfortMode and self.long_jerk_control is not None and self.long_limit_control is not None:
          self.long_jerk_control.update(CC.enabled, long_override, hud_control.leadDistance, hud_control.leadVisible, accel, critical_state)
          self.long_limit_control.update(CC.enabled, CS.out.vEgoRaw, hud_control.setSpeed, hud_control.leadDistance, hud_control.leadVisible, critical_state)

        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled, long_override)
        acc_hold_type = self.CCS.acc_hold_type(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled, starting, stopping,
                                               CS.esp_hold_confirmation, long_override, long_override_begin, long_disabling)
        can_sends.extend(self.CCS.create_acc_accel_control(
          self.packer_pt, self.CAN.pt, self.CP, CS.acc_type, CC.enabled,
          self.long_jerk_control.get_jerk_up() if CC.longComfortMode and self.long_jerk_control is not None else 4.0,
          self.long_jerk_control.get_jerk_down() if CC.longComfortMode and self.long_jerk_control is not None else 4.0,
          self.long_limit_control.get_upper_limit() if CC.longComfortMode and self.long_limit_control is not None else 0.,
          self.long_limit_control.get_lower_limit() if CC.longComfortMode and self.long_limit_control is not None else 0.,
          accel, acc_control, acc_hold_type, stopping, starting, CS.esp_hold_confirmation,
          CS.out.vEgoRaw * CV.MS_TO_KPH, long_override, CS.travel_assist_available,
        ))
        self.accel_last = accel
      else:
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CC.longActive, CC.cruiseControl.override, CS.out.accFaulted)
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
        if stopping and accel < 0:
          self.accel = -2.0
        else:
          self.accel = accel
        starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
        self.accel_diff = (0.0019 * (accel - self.accel_last)) + (1 - 0.0019) * self.accel_diff
        self.long_jerklimit = (0.01 * (np.clip(abs(accel), 0.7, 2))) + (1 - 0.01) * self.long_jerklimit
        self.long_deviation = np.interp(abs(accel - self.accel_diff), [0, 0.3, 1.0], [0.02, 0.04, 0.08])
        self.accel_last = accel
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, self.accel, acc_control, stopping, starting,
                                                           CS.esp_hold_confirmation, self.long_deviation, self.long_jerklimit))

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw) or CS.out.steerFaultTemporary:
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      if self.CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO):
        disable_alerts = getattr(CC, "disableCarSteerAlerts", False)
        sound_alert = self.CCP.LDW_SOUNDS["Chime"] if hud_alert != 0 and not disable_alerts else self.CCP.LDW_SOUNDS["None"]
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive, CS.out.steeringPressed,
                                                         hud_alert, hud_control, sound_alert))
      else:
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                         CS.out.steeringPressed, hud_alert, hud_control))

    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      self.distance_bar_frame = self.frame

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      if self.CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO):
        fcw_alert = hud_control.visualAlert == VisualAlert.fcw
        show_distance_bars = self.frame - self.distance_bar_frame < 400
        gap = max(8, CS.out.vEgo * hud_control.leadFollowTime)
        distance = max(8, hud_control.leadDistance) if hud_control.leadDistance != 0 else 0
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled,
                                                       CC.cruiseControl.override or CS.out.gasPressed)

        sl_predicative_active = CC.cruiseControl.speedLimitPredicative and CS.out.cruiseState.speedLimitPredicative != 0
        if CC.cruiseControl.speedLimit and CS.out.cruiseState.speedLimit != 0 and self.speed_limit_last != CS.out.cruiseState.speedLimit:
          self.speed_limit_changed_timer = self.frame
        self.speed_limit_last = CS.out.cruiseState.speedLimit
        sl_active = self.frame - self.speed_limit_changed_timer < 400
        speed_limit = CS.out.cruiseState.speedLimitPredicative if sl_predicative_active else (CS.out.cruiseState.speedLimit if sl_active else 0)

        acc_hud_event = self.CCS.acc_hud_event(acc_hud_status, CS.esp_hold_confirmation, sl_predicative_active,
                                               CS.speed_limit_predicative_type, sl_active)

        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, hud_control.setSpeed * CV.MS_TO_KPH,
                                                         hud_control.leadVisible, hud_control.leadDistanceBars + 1, show_distance_bars,
                                                         CS.esp_hold_confirmation, distance, gap, fcw_alert, acc_hud_event, speed_limit))
      else:
        leadDistance = min(8, hud_control.leadDistance) if hud_control.leadDistance != 0 else 0
        fcw_alert = hud_control.visualAlert == VisualAlert.fcw
        self.leadDistanceBars = min(3, hud_control.leadDistanceBars)
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CS.out.gasPressed, CC.longActive, CC.cruiseControl.override)
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH
        can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                         leadDistance, self.leadDistanceBars, fcw_alert, hud_control.leadVisible))

    if self.CP.flags & VolkswagenFlags.PQ:
      if self.frame % 2 == 0:
        self.blinkerActive = CS.leftBlinkerUpdate or CS.rightBlinkerUpdate
        leftBlinker = CC.leftBlinker if not self.blinkerActive else False
        rightBlinker = CC.rightBlinker if not self.blinkerActive else False
        can_sends.append(self.CCS.create_blinker_control(self.packer_pt, self.CAN.pt, leftBlinker, rightBlinker))

    if self.CP.openpilotLongitudinalControl and (self.CP.flags & VolkswagenFlags.PQ):
      if self.frame % 2 == 0:
        can_sends.append(self.CCS.filter_motor2(self.packer_pt, self.CAN.ext, CS.motor2_stock))

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if self.CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO):
      main_cruise_latching = not bool(CS.gra_stock_values["GRA_Typ_Hauptschalter"])
      stock_cancel_pressed = bool(CS.gra_stock_values["GRA_Abbrechen"] if main_cruise_latching else CS.gra_stock_values["GRA_Hauptschalter"])
    elif self.CP.flags & VolkswagenFlags.MLB:
      stock_cancel_pressed = bool(CS.gra_stock_values["LS_Abbrechen"])
    else:
      stock_cancel_pressed = bool(CS.gra_stock_values["GRA_Abbrechen"])

    cancel_cmd = stock_cancel_pressed or CC.cruiseControl.cancel
    if gra_send_ready and (cancel_cmd or CC.cruiseControl.resume):
      bus_send = self.CAN.aux if self.CP.flags & VolkswagenFlags.PQ else self.CAN.ext
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, bus_send, CS.gra_stock_values,
                                                           cancel=cancel_cmd, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.curvature = float(self.apply_curvature_last)
    new_actuators.accel = self.accel_last

    self.lead_distance_bars_last = hud_control.leadDistanceBars
    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends

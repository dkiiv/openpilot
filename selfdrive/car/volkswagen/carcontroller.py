from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_std_steer_angle_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.volkswagen import mqbcan, pqcan
from openpilot.selfdrive.car.volkswagen.values import CANBUS, CarControllerParams, VolkswagenFlags

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.CCS = pqcan if CP.flags & VolkswagenFlags.PQ else mqbcan
    self.packer_pt = CANPacker(dbc_name)

    self.apply_angle_last = 0
    self.gra_acc_counter_last = None
    self.frame = 0
    self.PLA_status = 0
    self.PLA_ESP_status = 0
    self.PLA_entryCounter = 0
    self.CSsteeringAngleDegLast = 0
    self.CSLH3_SignLast = 0

  def update(self, CC, CS, ext_bus, now_nanos, frogpilot_variables):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      # PLA_status definitions:
      #  8 = standby
      #  6 = active
      #  4 = activatable, entry request signal
      if CC.latActive:
        self.PLA_status = 6 if self.PLA_entryCounter >= 11 else 4
        self.PLA_ESP_status = 6 if self.PLA_entryCounter >= 32 else 4
        self.PLA_entryCounter += 1 if self.PLA_entryCounter <= 32 else self.PLA_entryCounter
        if CS.LH2_steeringState != 64 and self.PLA_entryCounter >= 30:
          self.PLA_entryCounter = 0
      else:
        self.PLA_status = 8
        self.PLA_ESP_status = 8
        self.PLA_entryCounter = 0

      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo, CarControllerParams) \
        if CC.latActive and self.PLA_status == 6 else self.CSsteeringAngleDegLast

      self.apply_angle_last = apply_angle
      self.CSsteeringAngleDegLast = CS.out.steeringAngleDeg
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.br, apply_angle, self.PLA_status, self.PLA_ESP_status, self.CSLH3_SignLast))
      self.CSLH3_SignLast = CS.LH_3_Sign

    # **** Gate_Komf Spammer ************************************************ #
    if self.PLA_entryCounter <= 30 and CC.latActive:
      can_sends.append(self.CCS.create_gk_spam(self.packer_pt, CANBUS.br, CS.Gate_Komf_stock))


    # **** Acceleration Controls ******************************************** #

    if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl:
      acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive, CC.cruiseControl.override)
      if frogpilot_variables.sport_plus:
        accel = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX_PLUS) if CC.longActive else 0
      else:
        accel = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0
      stopping = actuators.longControlState == LongCtrlState.stopping
      starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)

      can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, accel,
                                                         acc_control, stopping, starting, CS.esp_hold_confirmation))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.enabled,
                                                       CS.out.steeringPressed, hud_alert, hud_control, CC.latActive))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      lead_distance = 0
      if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
        lead_distance = 512 if CS.upscale_lead_car_signal else 8
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, acc_control, CC.cruiseControl.override)
      # FIXME: follow the recent displayed-speed updates, also use mph_kmh toggle to fix display rounding problem?
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, set_speed,
                                                       lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, ext_bus, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))


    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.eps_timer_soft_disable_alert = False

    CS.Gate_Komf_stock = 0
    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frameLast = self.frame


    return new_actuators, can_sends, self.eps_timer_soft_disable_alert

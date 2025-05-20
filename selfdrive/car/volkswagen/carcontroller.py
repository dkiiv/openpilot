from cereal import car
import cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.selfdrive.car import DT_CTRL, apply_driver_steer_torque_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.volkswagen import mqbcan, pqcan
from openpilot.selfdrive.car.volkswagen.values import CANBUS, CarControllerParams, VolkswagenFlags
from openpilot.selfdrive.controls.lib.drive_helpers import VOLKSWAGEN_V_CRUISE_MIN

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState
ButtonType = car.CarState.ButtonEvent.Type

def limit_jerk(accel, prev_accel, max_jerk, dt):
  max_delta_accel = max_jerk * dt
  delta_accel = max(-max_delta_accel, min(accel - prev_accel, max_delta_accel))
  return prev_accel + delta_accel

def EPB_handler(CS, self, ACS_Sta_ADR, ACS_Sollbeschl, vEgo, stopping):
  if (ACS_Sta_ADR == 1 and ACS_Sollbeschl < 0) and \
    ((CS.MOB_Standby and vEgo <= (18 * CV.KPH_TO_MS)) or self.EPB_enable):
      if not self.EPB_enable:  # First frame of EPB entry
          self.EPB_counter = 0
          self.EPB_brake = 0
          self.EPB_enable = 1
          self.EPB_enable_history = [True] * len(self.EPB_enable_history)
          self.EPB_brake_last = ACS_Sollbeschl
      else:
          self.EPB_brake = limit_jerk(-4, self.EPB_brake_last, 0.7, 0.02) if stopping else ACS_Sollbeschl
          self.EPB_brake_last = self.EPB_brake
      self.EPB_counter += 1
  else:
      if self.EPB_enable and self.EPB_counter < 10:  # Keep EPB_enable active for 10 frames
          self.EPB_counter += 1
      else:
          self.EPB_brake = 0
          self.EPB_enable = 0

  if CS.out.gasPressed or CS.out.brakePressed or CS.gra_stock_values["GRA_Abbrechen"]:
    if self.EPB_enable:
      self.ACC_anz_blind = 1
    self.EPB_brake = 0
    self.EPB_enable = 0
    self.EPB_enable_history = [False] * len(self.EPB_enable_history)

  if self.ACC_anz_blind and self.ACC_anz_blind_counter < 150:
    self.ACC_anz_blind_counter += 1
  else:
    self.ACC_anz_blind = 0
    self.ACC_anz_blind_counter = 0

  # Update EPB historical states and calculate EPB_active
  self.EPB_active = int((self.EPB_enable_history[(len(self.EPB_enable_history) - 2)] and not self.EPB_enable) or self.EPB_enable)
  self.EPB_enable_history = self.EPB_enable_history[1:] + [self.EPB_enable]

  return self.EPB_enable, self.EPB_brake, self.EPB_active

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    super().__init__(dbc_name, CP, VM)
    self.CCP = CarControllerParams(CP)
    self.CCS = pqcan if CP.flags & VolkswagenFlags.PQ else mqbcan
    self.packer_pt = CANPacker(dbc_name)
    self.ext_bus = CANBUS.pt if CP.networkLocation == car.CarParams.NetworkLocation.fwdCamera else CANBUS.cam

    self.apply_steer_last = 0
    self.gra_acc_counter_last = None
    self.bremse8_counter_last = None
    self.bremse11_counter_last = None
    self.acc_sys_counter_last = None
    self.acc_anz_counter_last = None
    self.ACC_anz_blind = 0
    self.ACC_anz_blind_counter = 0
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0
    self.last_button_frame = 0
    self.accel_last = 0
    self.frame = 0
    self.EPB_brake = 0
    self.EPB_brake_last = 0
    self.EPB_enable = 0
    self.EPB_enable_history = [False] * 25  # 0.5s history
    self.EPB_active = 0
    self.EPB_counter = 0
    self.accel_diff = 0
    self.long_deviation = 0
    self.long_jerklimit = 0
    self.stopped = 0
    self.stopping = 0

    self.sm = messaging.SubMaster(['longitudinalPlanSP'])
    self.param_s = Params()
    self.last_speed_limit_sign_tap_prev = False
    self.speed_limit = 0.
    self.speed_limit_offset = 0
    self.timer = 0
    self.final_speed_kph = 0
    self.init_speed = 0
    self.current_speed = 0
    self.v_set_dis = 0
    self.v_set_dis_prev = 0
    self.v_cruise_min = 0
    self.button_type = 0
    self.button_select = 0
    self.button_count = 0
    self.target_speed = 0
    self.t_interval = 7
    self.slc_active_stock = False
    self.sl_force_active_timer = 0
    self.v_tsc_state = 0
    self.slc_state = 0
    self.m_tsc_state = 0
    self.cruise_button = None
    self.last_cruise_button = None
    self.speed_diff = 0
    self.v_tsc = 0
    self.m_tsc = 0
    self.steady_speed = 0
    self.acc_type = -1
    self.send_count = 0

  def update(self, CC, CS, now_nanos):
    if not self.CP.pcmCruiseSpeed:
      self.sm.update(0)

      if self.sm.updated['longitudinalPlanSP']:
        self.v_tsc_state = self.sm['longitudinalPlanSP'].visionTurnControllerState
        self.slc_state = self.sm['longitudinalPlanSP'].speedLimitControlState
        self.m_tsc_state = self.sm['longitudinalPlanSP'].turnSpeedControlState
        self.speed_limit = self.sm['longitudinalPlanSP'].speedLimit
        self.speed_limit_offset = self.sm['longitudinalPlanSP'].speedLimitOffset
        self.v_tsc = self.sm['longitudinalPlanSP'].visionTurnSpeed
        self.m_tsc = self.sm['longitudinalPlanSP'].turnSpeed

      self.v_cruise_min = VOLKSWAGEN_V_CRUISE_MIN[CS.params_list.is_metric] * (CV.KPH_TO_MPH if not CS.params_list.is_metric else 1)
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    if not self.CP.pcmCruiseSpeed:
      if not self.last_speed_limit_sign_tap_prev and CS.params_list.last_speed_limit_sign_tap:
        self.sl_force_active_timer = self.frame
        self.param_s.put_bool_nonblocking("LastSpeedLimitSignTap", False)
      self.last_speed_limit_sign_tap_prev = CS.params_list.last_speed_limit_sign_tap

      sl_force_active = CS.params_list.speed_limit_control_enabled and (self.frame < (self.sl_force_active_timer * DT_CTRL + 2.0))
      sl_inactive = not sl_force_active and (not CS.params_list.speed_limit_control_enabled or (True if self.slc_state == 0 else False))
      sl_temp_inactive = not sl_force_active and (CS.params_list.speed_limit_control_enabled and (True if self.slc_state == 1 else False))
      slc_active = not sl_inactive and not sl_temp_inactive

      self.slc_active_stock = slc_active

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      # Logic to avoid HCA state 4 "refused":
      #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
      #   * Don't steer at standstill
      #   * Don't send > 3.00 Newton-meters torque
      #   * Don't send the same torque for > 6 seconds
      #   * Don't send uninterrupted steering for > 360 seconds
      # MQB racks reset the uninterrupted steering timer after a single frame
      # of HCA disabled; this is done whenever output happens to be zero.

      if CC.latActive:
        new_steer = int(round(actuators.steer * self.CCP.STEER_MAX))
        apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.CCP)
        self.hca_frame_timer_running += self.CCP.STEER_STEP
        if self.apply_steer_last == apply_steer:
          self.hca_frame_same_torque += self.CCP.STEER_STEP
          if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
            apply_steer -= (1, -1)[apply_steer < 0]
            self.hca_frame_same_torque = 0
        else:
          self.hca_frame_same_torque = 0
        hca_enabled = abs(apply_steer) > 0
      else:
        hca_enabled = False
        apply_steer = 0

      if not hca_enabled:
        self.hca_frame_timer_running = 0

      self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
      self.apply_steer_last = apply_steer
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.pt, apply_steer, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = clip(apply_steer * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX)
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, CANBUS.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #

    if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl:
      acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive, CC.cruiseControl.override)
      stopping = actuators.longControlState == LongCtrlState.stopping
      starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
      accel = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0
                                                                            # SMA to EMA conversion: alpha = 2 / (n + 1)    n = SMA-sample
      self.accel_diff = (0.0019 * (accel - self.accel_last)) + (1 - 0.0019) * self.accel_diff         # 1000 SMA equivalence
      self.long_jerklimit = (0.01 * (clip(abs(accel), 0.7, 2))) + (1 - 0.01) * self.long_jerklimit    # set jerk limit based on accel
      self.long_deviation = clip(CS.out.vEgo/40, 0, 0.13) * interp(abs(accel - self.accel_diff), [0, .2, 1.], [0.0, 0.0, 0.0])

      if self.CCS == pqcan and CC.longActive and actuators.accel <= 0 and CS.out.vEgoRaw <= 5:
        if not self.EPB_enable:  # first frame of EPB entry
          self.EPB_counter = 0
          self.EPB_brake = 0
          self.EPB_brake_last = accel - (CS.aEgoBremse / 2)
          self.EPB_enable = 1
        else:
          self.EPB_brake = limit_jerk(accel, self.EPB_brake_last, 0.7, 0.02)
          self.EPB_brake_last = self.EPB_brake
      else:
        acc_control = 0 if acc_control != 6 and self.EPB_enable else acc_control  # Pulse ACC status to 0 for one frame
        self.EPB_enable = 0
        self.EPB_brake = 0

        # Increment the EPB counter when EPB is enabled
        # Keep ACC status 0 for first 9 frames of EPB
      if self.EPB_enable:
        self.EPB_counter = min(self.EPB_counter + 1, 10)
        if self.EPB_counter <= 9:
          acc_control = 0
      else:
        self.EPB_counter = 0

      self.accel_last = accel
      if self.CCS == pqcan:
        can_sends.append(self.CCS.create_epb_control(self.packer_pt, CANBUS.br, self.EPB_brake, self.EPB_enable))
      can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, accel,
                                                         acc_control, stopping, starting, CS.esp_hold_confirmation,
                                                         self.long_deviation, self.long_jerklimit))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

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
    if self.CP.openpilotLongitudinalControl:
      if CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last:
        can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.ext_bus, CS.gra_stock_values, self.CP.openpilotLongitudinalControl,
                                                            cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))
      if not (CC.cruiseControl.cancel or CC.cruiseControl.resume) and CS.out.cruiseState.enabled:
        if not self.CP.pcmCruiseSpeed:
          self.cruise_button = self.get_cruise_buttons(CS, CC.vCruise)
          if self.cruise_button is not None:
            if self.acc_type == -1:
              if self.button_count >= 2 and self.v_set_dis_prev != self.v_set_dis:
                self.acc_type = 1 if abs(self.v_set_dis - self.v_set_dis_prev) >= 10 and self.last_cruise_button in (1, 2) else \
                                0 if abs(self.v_set_dis - self.v_set_dis_prev) < 10 and self.last_cruise_button not in (1, 2) else 1
              if self.send_count >= 10 and self.v_set_dis_prev == self.v_set_dis:
                self.cruise_button = 3 if self.cruise_button == 1 else 4
            if self.acc_type == 0:
              self.cruise_button = 1 if self.cruise_button == 1 else 2  # accel, decel
            elif self.acc_type == 1:
              self.cruise_button = 3 if self.cruise_button == 1 else 4  # resume, set
            if self.frame % self.CCP.BTN_STEP == 0:
              can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, CS.gra_stock_values, self.CP.openpilotLongitudinalControl,
                                                                  frame=(self.frame // self.CCP.BTN_STEP), buttons=self.cruise_button,
                                                                  custom_stock_long=True))
              self.send_count += 1
          else:
            self.send_count = 0
          self.last_cruise_button = self.cruise_button

    # **** Blinding Motor_2 for PQ radar ************ #
    if VolkswagenFlags.PQ and self.ext_bus == CANBUS.cam and self.CP.openpilotLongitudinalControl:
      if self.frame % 2 or CS.motor2_stock != getattr(self, 'motor2_last', CS.motor2_stock):  # 50hz / 20ms
        can_sends.append(self.CCS.create_motor2_control(self.packer_pt, CANBUS.cam, CS.motor2_stock))
      self.motor2_last = CS.motor2_stock

    # *** Below here is for OEM+ behavior modification of OEM ACC *** #
    # Modify Motor_2, Bremse_8, Bremse_11
    if VolkswagenFlags.PQ and not self.CP.openpilotLongitudinalControl:
      self.stopping = CS.acc_sys_stock["ACS_Anhaltewunsch"] and (CS.out.vEgoRaw <= 2 or self.stopping)
      self.stopped = self.EPB_enable and (CS.out.vEgoRaw == 0 or (self.stopping and self.stopped))

      if CS.acc_sys_stock["COUNTER"] != self.acc_sys_counter_last:
        EPB_handler(CS, self, CS.acc_sys_stock["ACS_Sta_ADR"], CS.acc_sys_stock["ACS_Sollbeschl"], CS.out.vEgoRaw, self.stopping)
        can_sends.append(self.CCS.filter_ACC_System(self.packer_pt, CANBUS.pt, CS.acc_sys_stock, self.EPB_active))
        can_sends.append(self.CCS.create_epb_control(self.packer_pt, CANBUS.br, self.EPB_brake, self.EPB_enable))
        can_sends.append(self.CCS.filter_epb1(self.packer_pt, CANBUS.cam, self.stopped))  # in custom module, filter the gateway fwd EPB msg
      if CS.acc_anz_stock["COUNTER"] != self.acc_anz_counter_last:
        can_sends.append(self.CCS.filter_ACC_Anzeige(self.packer_pt, CANBUS.pt, CS.acc_anz_stock, self.ACC_anz_blind))
      if self.frame % 2 or CS.motor2_stock != getattr(self, 'motor2_last', CS.motor2_stock):  # 50hz / 20ms
        can_sends.append(self.CCS.filter_motor2(self.packer_pt, CANBUS.cam, CS.motor2_stock, self.EPB_enable_history[0]))
        if CS.motor2_stock["GRA_Status"] in (1, 2) and self.motor2_last["GRA_Status"] == 0:
          self.EPB_enable_history = [False] * len(self.EPB_enable_history)  # disable filter when ECM enters cruise state
      if CS.bremse8_stock["COUNTER"] != self.bremse8_counter_last:
        can_sends.append(self.CCS.filter_bremse8(self.packer_pt, CANBUS.cam, CS.bremse8_stock, self.EPB_enable_history[0]))
      if CS.bremse11_stock["COUNTER"] != self.bremse11_counter_last:
        can_sends.append(self.CCS.filter_bremse11(self.packer_pt, CANBUS.cam, CS.bremse11_stock, self.stopped))
      if CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last:
        can_sends.append(self.CCS.filter_GRA_Neu(self.packer_pt, CANBUS.cam, CS.gra_stock_values, resume = self.stopped and (self.frame % 100 < 50)))

      self.motor2_last = CS.motor2_stock
      self.acc_sys_counter_last = CS.acc_sys_stock["COUNTER"]
      self.acc_anz_counter_last = CS.acc_anz_stock["COUNTER"]
      self.bremse8_counter_last = CS.bremse8_stock["COUNTER"]
      self.bremse11_counter_last = CS.bremse11_stock["COUNTER"]

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.v_set_dis_prev = self.v_set_dis
    self.frame += 1
    return new_actuators, can_sends

  # multikyd methods, sunnyhaibin logic
  def get_cruise_buttons_status(self, CS):
    if not CS.out.cruiseState.enabled:
      for be in CS.out.buttonEvents:
        if be.type in (ButtonType.accelCruise, ButtonType.resumeCruise,
                       ButtonType.decelCruise, ButtonType.setCruise) and be.pressed:
          self.timer = 40
        elif be.type == ButtonType.gapAdjustCruise and be.pressed:
          self.timer = 300
    elif self.timer:
      self.timer -= 1
    else:
      return 1
    return 0

  def get_target_speed(self, v_cruise_kph_prev):
    v_cruise_kph = v_cruise_kph_prev
    if self.slc_state > 1:
      v_cruise_kph = (self.speed_limit + self.speed_limit_offset) * CV.MS_TO_KPH
      if not self.slc_active_stock:
        v_cruise_kph = v_cruise_kph_prev
    return v_cruise_kph

  def get_button_type(self, button_type):
    self.type_status = "type_" + str(button_type)
    self.button_picker = getattr(self, self.type_status, lambda: "default")
    return self.button_picker()

  def reset_button(self):
    if self.button_type != 3:
      self.button_type = 0

  def type_default(self):
    self.button_type = 0
    return None

  def type_0(self):
    self.button_count = 0
    self.target_speed = self.init_speed
    self.speed_diff = self.target_speed - self.v_set_dis
    if self.target_speed > self.v_set_dis:
      self.button_type = 1
    elif self.target_speed < self.v_set_dis and self.v_set_dis > self.v_cruise_min:
      self.button_type = 2
    return None

  def type_1(self):
    cruise_button = 1
    self.button_count += 1
    if self.target_speed <= self.v_set_dis:
      self.button_count = 0
      self.button_type = 3
    elif self.button_count > 5:
      self.button_count = 0
      self.button_type = 3
    return cruise_button

  def type_2(self):
    cruise_button = 2
    self.button_count += 1
    if self.target_speed >= self.v_set_dis or self.v_set_dis <= self.v_cruise_min:
      self.button_count = 0
      self.button_type = 3
    elif self.button_count > 5:
      self.button_count = 0
      self.button_type = 3
    return cruise_button

  def type_3(self):
    cruise_button = None
    self.button_count += 1
    if self.button_count > self.t_interval:
      self.button_type = 0
    return cruise_button

  def get_curve_speed(self, target_speed_kph, v_cruise_kph_prev):
    if self.v_tsc_state != 0:
      vision_v_cruise_kph = self.v_tsc * CV.MS_TO_KPH
      if int(vision_v_cruise_kph) == int(v_cruise_kph_prev):
        vision_v_cruise_kph = 255
    else:
      vision_v_cruise_kph = 255
    if self.m_tsc_state > 1:
      map_v_cruise_kph = self.m_tsc * CV.MS_TO_KPH
      if int(map_v_cruise_kph) == 0.0:
        map_v_cruise_kph = 255
    else:
      map_v_cruise_kph = 255
    curve_speed = self.curve_speed_hysteresis(min(vision_v_cruise_kph, map_v_cruise_kph) + 2 * CV.MPH_TO_KPH)
    return min(target_speed_kph, curve_speed)

  def get_button_control(self, CS, final_speed, v_cruise_kph_prev):
    self.init_speed = round(min(final_speed, v_cruise_kph_prev) * (CV.KPH_TO_MPH if not CS.params_list.is_metric else 1))
    self.v_set_dis = round(CS.out.cruiseState.speed * (CV.MS_TO_MPH if not CS.params_list.is_metric else CV.MS_TO_KPH))
    cruise_button = self.get_button_type(self.button_type)
    return cruise_button

  def curve_speed_hysteresis(self, cur_speed: float, hyst=(0.75 * CV.MPH_TO_KPH)):
    if cur_speed > self.steady_speed:
      self.steady_speed = cur_speed
    elif cur_speed < self.steady_speed - hyst:
      self.steady_speed = cur_speed
    return self.steady_speed

  def get_cruise_buttons(self, CS, v_cruise_kph_prev):
    cruise_button = None
    if not self.get_cruise_buttons_status(CS):
      pass
    elif CS.out.cruiseState.enabled:
      set_speed_kph = self.get_target_speed(v_cruise_kph_prev)
      if self.slc_state > 1:
        target_speed_kph = set_speed_kph
      else:
        target_speed_kph = min(v_cruise_kph_prev, set_speed_kph)
      if self.v_tsc_state != 0 or self.m_tsc_state > 1:
        self.final_speed_kph = self.get_curve_speed(target_speed_kph, v_cruise_kph_prev)
      else:
        self.final_speed_kph = target_speed_kph

      cruise_button = self.get_button_control(CS, self.final_speed_kph, v_cruise_kph_prev)  # MPH/KPH based button presses
    return cruise_button

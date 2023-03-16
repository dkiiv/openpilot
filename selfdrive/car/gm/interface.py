#!/usr/bin/env python3
from cereal import car
from math import fabs, erf, atan, cos
from panda import Panda

from common.numpy_fast import interp, sign, clip
from common.conversions import Conversions as CV
from selfdrive.car import STD_CARGO_KG, create_button_event, scale_tire_stiffness, get_safety_config, create_mads_event
from selfdrive.car.gm.values import CAR, CruiseButtons, CarControllerParams, EV_CAR, CAMERA_ACC_CAR
from selfdrive.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType, FRICTION_THRESHOLD
from selfdrive.controls.lib.drive_helpers import apply_center_deadzone

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
GearShifter = car.CarState.GearShifter
TransmissionType = car.CarParams.TransmissionType
NetworkLocation = car.CarParams.NetworkLocation
BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise,
                CruiseButtons.MAIN: ButtonType.altButton3, CruiseButtons.CANCEL: ButtonType.cancel}

FRICTION_THRESHOLD_LAT_JERK = 2.0

def get_steer_feedforward_erf1(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  x = ANGLE_COEF * (angle + ANGLE_OFFSET) * (40.23 / (max(0.05,speed + SPEED_OFFSET))**SPEED_COEF)
  sigmoid_factor = (SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT)
  sigmoid = erf(x)
  sigmoid *= sigmoid_factor * sigmoid_factor
  sigmoid *= (40.23 / (max(0.05,speed + SPEED_OFFSET2))**SPEED_COEF2)
  linear = ANGLE_COEF2 * (angle + ANGLE_OFFSET)
  return sigmoid + linear

class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def get_steer_feedforward_acadia(desired_angle, v_ego):
    ANGLE_COEF = 5.00000000
    ANGLE_COEF2 = 1.90844451
    ANGLE_COEF3 = 0.03401073
    SPEED_OFFSET = 13.72019138
    SIGMOID_COEF_RIGHT = 0.00100000
    SIGMOID_COEF_LEFT = 0.00101873
    SPEED_COEF = 0.36844505
    x = ANGLE_COEF * (desired_angle) / max(0.01,v_ego)
    sigmoid = x / (1. + fabs(x))
    return ((SIGMOID_COEF_RIGHT if desired_angle > 0. else SIGMOID_COEF_LEFT) * sigmoid) * (0.01 + v_ego + SPEED_OFFSET) ** ANGLE_COEF2 + ANGLE_COEF3 * (desired_angle * SPEED_COEF - atan(desired_angle * SPEED_COEF))

  def get_steer_feedforward_function(self):
    if self.CP.carFingerprint == CAR.ACADIA:
      return self.get_steer_feedforward_acadia
    else:
      return CarInterfaceBase.get_steer_feedforward_default
    
  @staticmethod
  def torque_from_lateral_accel_volt(lateral_accel_value, torque_params, lateral_accel_error, lateral_accel_deadzone, friction_compensation, v_ego, g_lat_accel, lateral_jerk_desired):
    ANGLE_COEF = 0.15461558
    ANGLE_COEF2 = 0.22491234
    ANGLE_OFFSET = 0.0#-0.01173257
    SPEED_OFFSET = 2.37065180
    SIGMOID_COEF_RIGHT = 0.14917052
    SIGMOID_COEF_LEFT = 0.13559770
    SPEED_COEF = 0.49912791
    SPEED_COEF2 = 0.37766423
    SPEED_OFFSET2 = -0.36618369

    ff = get_steer_feedforward_erf1(lateral_accel_value, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)
    
    # now lateral jerk component
    if sign(lateral_accel_value) == sign(lateral_jerk_desired):
      ANGLE_COEF = 2.87742908
      ANGLE_COEF2 = 3.0171422
      SPEED_OFFSET = 0.32037237
      SIGMOID_COEF_1 = 0.13778183
      SIGMOID_COEF_2 = 0.78604993
      SPEED_COEF = 0.11084122
      SPEED_COEF2 = 1.84607376

      x = ANGLE_COEF * (lateral_jerk_desired) * (40.23 / (max(1.0,v_ego + SPEED_OFFSET))**SPEED_COEF)
      sigmoid1 = x / (1. + fabs(x))
      sigmoid1 *= SIGMOID_COEF_1
      
      x = ANGLE_COEF2 * (lateral_jerk_desired) * (40.23 / (max(1.0,v_ego + SPEED_OFFSET))**SPEED_COEF2)
      sigmoid2 = x / (1. + fabs(x))
      sigmoid2 *= SIGMOID_COEF_2 / (fabs(v_ego)+1)

      friction = sigmoid1 + sigmoid2
    else:
      ANGLE_COEF = 2.06223884
      ANGLE_COEF2 = 0.17299524
      ANGLE_OFFSET = 10.08002596
      SPEED_OFFSET = -0.11092718
      SIGMOID_COEF_1 = 0.17167717
      SIGMOID_COEF_2 = 1.99966155
      SPEED_COEF = 1.04884478
      SPEED_COEF2 = 2.00000000
      
      x = ANGLE_COEF * (lateral_jerk_desired) * (40.23 / (max(1.0,v_ego + SPEED_OFFSET))**SPEED_COEF)
      sigmoid1 = x / (1. + fabs(x))
      sigmoid1 *= SIGMOID_COEF_1
      
      x = ANGLE_COEF2 * (lateral_jerk_desired) * (40.23 / (max(1.0,v_ego + SPEED_OFFSET))**SPEED_COEF2)
      sigmoid2 = x / (1. + fabs(x))
      sigmoid2 *= SIGMOID_COEF_2
      
      max_speed = ANGLE_OFFSET
      speed_norm = 0.5 * cos(clip(v_ego / max_speed, 0., 1.) * 3.14) + 0.5
      
      friction = (1-speed_norm) * sigmoid1 + speed_norm * sigmoid2
    
    return ff + friction + g_lat_accel * 0.5
  
  @staticmethod
  def torque_from_lateral_accel_bolt_euv(lateral_accel_value, torque_params, lateral_accel_error, lateral_accel_deadzone, friction_compensation, v_ego, g_lat_accel, lateral_jerk_desired):
    ANGLE_COEF = 0.12038141
    ANGLE_COEF2 = 0.20606792
    ANGLE_OFFSET = 0.0#0720524
    SPEED_OFFSET = -0.63391381
    SIGMOID_COEF_RIGHT = 0.12397734
    SIGMOID_COEF_LEFT = 0.11149019
    SPEED_COEF = 0.30227209
    SPEED_COEF2 = 0.20079583
    SPEED_OFFSET2 = -1.43326564

    ff = get_steer_feedforward_erf1(lateral_accel_value, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)
    friction = interp(
      lateral_jerk_desired,
      [-FRICTION_THRESHOLD_LAT_JERK, FRICTION_THRESHOLD_LAT_JERK],
      [-torque_params.friction, torque_params.friction]
    )
    return ff + friction + g_lat_accel * 0.6
  
  @staticmethod
  def torque_from_lateral_accel_bolt(lateral_accel_value, torque_params, lateral_accel_error, lateral_accel_deadzone, friction_compensation, v_ego, g_lat_accel, lateral_jerk_desired):
    ANGLE_COEF = 0.18708832
    ANGLE_COEF2 = 0.28818528
    SPEED_OFFSET = 20.00000000
    SIGMOID_COEF_RIGHT = 0.36997215
    SIGMOID_COEF_LEFT = 0.43181054
    SPEED_COEF = 0.34143006
    x = ANGLE_COEF * lateral_accel_value * (40.23 / (max(0.2,v_ego + SPEED_OFFSET))**SPEED_COEF)
    sigmoid = erf(x)
    out = ((SIGMOID_COEF_RIGHT if lateral_accel_value < 0. else SIGMOID_COEF_LEFT) * sigmoid) + ANGLE_COEF2 * lateral_accel_value
    friction = interp(
      lateral_jerk_desired,
      [-FRICTION_THRESHOLD_LAT_JERK, FRICTION_THRESHOLD_LAT_JERK],
      [-torque_params.friction, torque_params.friction]
    )
    return out + friction + g_lat_accel * 0.6
  
  @staticmethod
  def torque_from_lateral_accel_silverado(lateral_accel_value, torque_params, lateral_accel_error, lateral_accel_deadzone, friction_compensation, v_ego, g_lat_accel, lateral_jerk_desired):
    ANGLE_COEF = 4.99999984
    ANGLE_COEF2 = 0.18643417
    ANGLE_OFFSET = 0.0#0113549
    SPEED_OFFSET = 14.66635487
    SIGMOID_COEF_RIGHT = 0.16275759
    SIGMOID_COEF_LEFT = 0.14734260
    SPEED_COEF = 1.29285146
    SPEED_COEF2 = 0.53967972
    SPEED_OFFSET2 = -0.43658794

    ff = get_steer_feedforward_erf1(lateral_accel_value, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)
    friction = interp(
      lateral_jerk_desired,
      [-FRICTION_THRESHOLD_LAT_JERK, FRICTION_THRESHOLD_LAT_JERK],
      [-torque_params.friction, torque_params.friction]
    )
    return ff + friction + g_lat_accel * 0.6

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    if self.CP.carFingerprint == CAR.BOLT_EUV:
      return self.torque_from_lateral_accel_bolt_euv
    elif self.CP.carFingerprint == CAR.VOLT:
      return self.torque_from_lateral_accel_volt
    elif self.CP.carFingerprint == CAR.SILVERADO:
      return self.torque_from_lateral_accel_silverado
    else:
      return self.torque_from_lateral_accel_linear

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long):
    ret.carName = "gm"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.gm)]
    ret.autoResumeSng = False

    if candidate in EV_CAR:
      ret.transmissionType = TransmissionType.direct
    else:
      ret.transmissionType = TransmissionType.automatic

    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.15]

    ret.longitudinalTuning.kpBP = [5., 35.]
    ret.longitudinalTuning.kiBP = [0.]

    if candidate in CAMERA_ACC_CAR:
      ret.experimentalLongitudinalAvailable = True
      ret.networkLocation = NetworkLocation.fwdCamera
      ret.radarUnavailable = True  # no radar
      ret.pcmCruise = True
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_CAM
      ret.minEnableSpeed = 5 * CV.KPH_TO_MS
      ret.minSteerSpeed = 10 * CV.KPH_TO_MS

      # Tuning for experimental long
      ret.longitudinalTuning.kpV = [2.0, 1.5]
      ret.longitudinalTuning.kiV = [0.72]
      ret.stopAccel = -2.0
      ret.stoppingDecelRate = 2.0  # reach brake quickly after enabling
      ret.vEgoStopping = 0.25
      ret.vEgoStarting = 0.25
      ret.longitudinalActuatorDelayUpperBound = 0.5

      if experimental_long:
        ret.pcmCruise = False
        ret.openpilotLongitudinalControl = True
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_CAM_LONG

    else:  # ASCM, OBD-II harness
      ret.openpilotLongitudinalControl = True
      ret.networkLocation = NetworkLocation.gateway
      ret.radarUnavailable = False
      ret.pcmCruise = False  # stock non-adaptive cruise control is kept off
      # supports stop and go, but initial engage must (conservatively) be above 18mph
      ret.minEnableSpeed = 18 * CV.MPH_TO_MS
      ret.minSteerSpeed = 6.7 * CV.MPH_TO_MS

    # Tuning
    ret.longitudinalTuning.kpV = [2.4, 1.5]
    ret.longitudinalTuning.kiV = [0.36]

    # These cars have been put into dashcam only due to both a lack of users and test coverage.
    # These cars likely still work fine. Once a user confirms each car works and a test route is
    # added to selfdrive/car/tests/routes.py, we can remove it from this list.
    ret.dashcamOnly = candidate in {CAR.CADILLAC_ATS, CAR.HOLDEN_ASTRA, CAR.MALIBU, CAR.BUICK_REGAL, CAR.EQUINOX}

    # Start with a baseline tuning for all GM vehicles. Override tuning as needed in each model section below.
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.00]]
    ret.lateralTuning.pid.kf = 0.00004   # full torque for 20 deg at 80mph means 0.00007818594
    ret.steerActuatorDelay = 0.1  # Default delay, not measured yet
    tire_stiffness_factor = 0.444  # not optimized yet

    ret.steerLimitTimer = 0.4
    ret.radarTimeStep = 0.0667  # GM radar runs at 15Hz instead of standard 20Hz

    if candidate == CAR.VOLT:
      ret.minEnableSpeed = -1
      ret.mass = 1607. + STD_CARGO_KG
      ret.wheelbase = 2.69
      ret.steerRatio = 17.7  # Stock 15.7, LiveParameters
      tire_stiffness_factor = 0.469  # Stock Michelin Energy Saver A/S, LiveParameters
      ret.centerToFront = ret.wheelbase * 0.45  # Volt Gen 1, TODO corner weigh

      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.steerActuatorDelay = 0.2
      
      ret.longitudinalTuning.kpBP = [5., 15., 35.]
      ret.longitudinalTuning.kpV = [0.7, .9, 0.8]
      ret.longitudinalTuning.kiBP = [5., 15., 35.]
      ret.longitudinalTuning.kiV = [0.07, 0.13, 0.13]
      ret.longitudinalTuning.kdBP = [5., 25.]
      ret.longitudinalTuning.kdV = [0.3, 0.0]
      ret.stoppingDecelRate = 0.2 # brake_travel/s while trying to stop
      ret.longitudinalActuatorDelayLowerBound = 0.41
      ret.longitudinalActuatorDelayUpperBound = 0.41

    elif candidate == CAR.MALIBU:
      ret.mass = 1496. + STD_CARGO_KG
      ret.wheelbase = 2.83
      ret.steerRatio = 15.8
      ret.centerToFront = ret.wheelbase * 0.4  # wild guess

    elif candidate == CAR.HOLDEN_ASTRA:
      ret.mass = 1363. + STD_CARGO_KG
      ret.wheelbase = 2.662
      # Remaining parameters copied from Volt for now
      ret.centerToFront = ret.wheelbase * 0.4
      ret.steerRatio = 15.7

    elif candidate == CAR.ACADIA:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.mass = 4353. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.86
      ret.steerRatio = 16.0 #14.4  # end to end is 13.46
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4
      ret.steerActuatorDelay = 0.24
        
      ret.lateralTuning.pid.kpBP = [i * CV.MPH_TO_MS for i in [0., 80.]]
      ret.lateralTuning.pid.kpV = [0., 0.16]
      ret.lateralTuning.pid.kiBP = [0., 35.]
      ret.lateralTuning.pid.kiV = [0.008, 0.012]
      ret.lateralTuning.pid.kdBP = [0.]
      ret.lateralTuning.pid.kdV = [0.6]
      ret.lateralTuning.pid.kf = 1. # get_steer_feedforward_acadia()

      ret.longitudinalTuning.kdBP = [5., 25.]
      ret.longitudinalTuning.kdV = [0.8, 0.4]
      ret.longitudinalTuning.kiBP = [5., 35.]
      ret.longitudinalTuning.kiV = [0.31, 0.34]
      ret.longitudinalActuatorDelayUpperBound = 0.5  # large delay to initially start braking

    elif candidate == CAR.BUICK_REGAL:
      ret.mass = 3779. * CV.LB_TO_KG + STD_CARGO_KG  # (3849+3708)/2
      ret.wheelbase = 2.83  # 111.4 inches in meters
      ret.steerRatio = 14.4  # guess for tourx
      ret.centerToFront = ret.wheelbase * 0.4  # guess for tourx

    elif candidate == CAR.CADILLAC_ATS:
      ret.mass = 1601. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 15.3
      ret.centerToFront = ret.wheelbase * 0.5

    elif candidate == CAR.ESCALADE:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.mass = 5653. * CV.LB_TO_KG + STD_CARGO_KG  # (5552+5815)/2
      ret.wheelbase = 2.95  # 116 inches in meters
      ret.steerRatio = 17.3
      ret.centerToFront = ret.wheelbase * 0.5
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.ESCALADE_ESV:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.mass = 2739. + STD_CARGO_KG
      ret.wheelbase = 3.302
      ret.steerRatio = 17.3
      ret.centerToFront = ret.wheelbase * 0.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[10., 41.0], [10., 41.0]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.13, 0.24], [0.01, 0.02]]
      ret.lateralTuning.pid.kf = 0.000045
      tire_stiffness_factor = 1.0

    elif candidate == CAR.BOLT_EUV:
      ret.mass = 1669. + STD_CARGO_KG
      ret.wheelbase = 2.63779
      ret.steerRatio = 16.8
      ret.centerToFront = ret.wheelbase * 0.4
      tire_stiffness_factor = 1.0
      ret.steerActuatorDelay = 0.12
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.SILVERADO:
      ret.minEnableSpeed = -1.
      ret.minSteerSpeed = -1 * CV.MPH_TO_MS
      ret.mass = 2400. + STD_CARGO_KG
      ret.wheelbase = 3.745
      ret.steerRatio = 16.3
      ret.centerToFront = ret.wheelbase * .49
      ret.steerActuatorDelay = 0.11
      tire_stiffness_factor = 1.0
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.EQUINOX:
      ret.mass = 3500. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 14.4
      ret.centerToFront = ret.wheelbase * 0.4
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam, self.cp_loopback)

    buttonEvents = []

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons and self.CS.prev_cruise_buttons != CruiseButtons.INIT:
      buttonEvents.append(create_button_event(self.CS.cruise_buttons, self.CS.prev_cruise_buttons, BUTTONS_DICT, CruiseButtons.UNPRESS))
      # Handle ACCButtons changing buttons mid-press
      if self.CS.cruise_buttons != CruiseButtons.UNPRESS and self.CS.prev_cruise_buttons != CruiseButtons.UNPRESS:
        buttonEvents.append(create_button_event(CruiseButtons.UNPRESS, self.CS.prev_cruise_buttons, BUTTONS_DICT, CruiseButtons.UNPRESS))

    self.CS.mads_enabled = False if not self.CS.control_initialized else ret.cruiseState.available

    if not self.CP.pcmCruise:
      if any(b.type == ButtonType.accelCruise and b.pressed for b in buttonEvents):
        self.CS.accEnabled = True

    self.CS.accEnabled, buttonEvents = self.get_sp_v_cruise_non_pcm_state(ret.cruiseState.available, self.CS.accEnabled,
                                                                          buttonEvents, c.vCruise)

    if ret.cruiseState.available:
      if self.enable_mads:
        if not self.CS.prev_mads_enabled and self.CS.mads_enabled:
          self.CS.madsEnabled = True
        if self.CS.prev_lkas_enabled != 1 and self.CS.lkas_enabled == 1:
          self.CS.madsEnabled = not self.CS.madsEnabled
        self.CS.madsEnabled = self.get_acc_mads(ret.cruiseState.enabled, self.CS.accEnabled, self.CS.madsEnabled)
    else:
      self.CS.madsEnabled = False

    if not self.CP.pcmCruise or (self.CP.pcmCruise and self.CP.minEnableSpeed > 0):
      if any(b.type == ButtonType.cancel for b in buttonEvents):
        self.CS.madsEnabled, self.CS.accEnabled = self.get_sp_cancel_cruise_state(self.CS.madsEnabled)
    if self.get_sp_pedal_disengage(ret):
      self.CS.madsEnabled, self.CS.accEnabled = self.get_sp_cancel_cruise_state(self.CS.madsEnabled)
      ret.cruiseState.enabled = False if self.CP.pcmCruise else self.CS.accEnabled

    if self.CP.pcmCruise and self.CP.minEnableSpeed > 0:
      if ret.gasPressed and not ret.cruiseState.enabled:
        self.CS.accEnabled = False
      self.CS.accEnabled = ret.cruiseState.enabled or self.CS.accEnabled

    ret, self.CS = self.get_sp_common_state(ret, self.CS, gap_button=bool(self.CS.gap_dist_button))

    # MADS BUTTON
    if self.CS.out.madsEnabled != self.CS.madsEnabled:
      if self.mads_event_lock:
        buttonEvents.append(create_mads_event(self.mads_event_lock))
        self.mads_event_lock = False
    else:
      if not self.mads_event_lock:
        buttonEvents.append(create_mads_event(self.mads_event_lock))
        self.mads_event_lock = True

    ret.buttonEvents = buttonEvents

    # The ECM allows enabling on falling edge of set, but only rising edge of resume
    events = self.create_common_events(ret, extra_gears=[GearShifter.sport, GearShifter.low,
                                                         GearShifter.eco, GearShifter.manumatic],
                                       pcm_enable=False, enable_buttons=(ButtonType.decelCruise,))
    #if not self.CP.pcmCruise:
    #  if any(b.type == ButtonType.accelCruise and b.pressed for b in ret.buttonEvents):
    #    events.add(EventName.buttonEnable)

    events, ret = self.create_sp_events(self.CS, ret, events, enable_pressed=self.CS.accEnabled,
                                        enable_buttons=(ButtonType.decelCruise,))

    # Enabling at a standstill with brake is allowed
    # TODO: verify 17 Volt can enable for the first time at a stop and allow for all GMs
    below_min_enable_speed = ret.vEgo < self.CP.minEnableSpeed or self.CS.moving_backward
    if below_min_enable_speed and not (ret.standstill and ret.brake >= 20 and
                                       self.CP.networkLocation == NetworkLocation.fwdCamera):
      events.add(EventName.belowEngageSpeed)
    if ret.cruiseState.standstill:
      events.add(EventName.resumeRequired)
    if ret.vEgo < self.CP.minSteerSpeed and self.CS.madsEnabled and ret.vEgo > 0.05:
      events.add(EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)

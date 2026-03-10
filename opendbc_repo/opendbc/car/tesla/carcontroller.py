import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from openpilot.iqpilot.selfdrive.tuning.eps_kompensator import apply_tesla_eps_kompensator
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.iqpilot.car.tesla.coop_steering import CoopSteeringCarController


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  # A Model 3 at 40 m/s using the Model Y limits sees a <0.3% difference in max angle (from curvature factor)
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")

def acc_stateControl(self, c):
  # Hold state = 13 for 9 frames on rising edge
  if c.cruiseControl.cancel and not self._prev_cancel:
    self._cancel_frame_count = 9
  self._prev_cancel = c.cruiseControl.cancel

  if self._cancel_frame_count > 0 and not c.longActive:
    state = 13
    self._cancel_frame_count -= 1
  else:
    self._cancel_frame_count = 0
    state = 4  # ACC_ON

  return state

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_IQ):
    CarControllerBase.__init__(self, dbc_names, CP, CP_IQ)
    self.coop_steer = CoopSteeringCarController()
    self.apply_angle_last = 0
    self._prev_cancel = False
    self._cancel_frame_count = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(CP, self.packer)

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

  def update(self, CC, CC_IQ, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # Tesla EPS enforces disabling steering on heavy lateral override force.
    # When enabling in a tight curve, we wait until user reduces steering force to start steering.
    # Canceling is done on rising edge and is handled generically with CC.cruiseControl.cancel
    lat_active = CC.latActive and CS.hands_on_level < 3

    if self.frame % CarControllerParams.STEER_STEP == 0:
      # Proprietary Tesla EPS Kompensator + lateral accel constrained angle limiting
      self.apply_angle_last = apply_tesla_eps_kompensator(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                          CS.out.steeringAngleDeg, lat_active, CarControllerParams, self.VM)

      can_sends.append(self.tesla_can.create_steering_control(*self.coop_steer.update(self.apply_angle_last, lat_active, self.CP_IQ, CS, self.VM)))

    if self.frame % 10 == 0:
      can_sends.append(self.tesla_can.create_steering_allowed())

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        state = acc_stateControl(CC)  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))

    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if CC.cruiseControl.cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.accel = self.coop_steer.coop_apply_angle_last_sat  # debug
    new_actuators.curvature = float(self.coop_steer.debug_angle_desired_limited)  # debug
    new_actuators.torque = float(self.coop_steer.override_angle_accu)  # debug

    self.frame += 1
    return new_actuators, can_sends

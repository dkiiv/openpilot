"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""

from openpilot.common.params import Params


class LatControlTorqueExtOverride:
  def __init__(self, CP):
    self.CP = CP
    self.params = Params()
    self.enforce_torque_control_toggle = self.params.get_bool("EnforceTorqueControl")
    self.torque_override_enabled = self.params.get_bool("TorqueParamsOverrideEnabled")
    self.frame = -1

  def update_override_torque_params(self, torque_params) -> bool:
    self.frame += 1
    if self.frame % 300 == 0:
      self.enforce_torque_control_toggle = self.params.get_bool("EnforceTorqueControl")
      self.torque_override_enabled = self.params.get_bool("TorqueParamsOverrideEnabled")

      if not self.enforce_torque_control_toggle or not self.torque_override_enabled:
        return False

      torque_params.latAccelFactor = float(self.params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
      torque_params.friction = float(self.params.get("TorqueParamsOverrideFriction", return_default=True))
      return True

    return False

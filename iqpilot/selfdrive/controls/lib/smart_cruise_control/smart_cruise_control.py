"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
import cereal.messaging as messaging
from openpilot.iqpilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import SmartCruiseControlVision
from openpilot.iqpilot.selfdrive.controls.lib.smart_cruise_control.map_controller import SmartCruiseControlMap


class SmartCruiseControl:
  def __init__(self):
    self.vision = SmartCruiseControlVision()
    self.map = SmartCruiseControlMap()

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, v_cruise: float) -> None:
    self.map.update(long_enabled, long_override, v_ego, a_ego, v_cruise)
    self.vision.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

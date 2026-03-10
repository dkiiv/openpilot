"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
from cereal import car

from openpilot.common.constants import CV
from openpilot.common.params import Params


class BlinkerPauseLateral:
  def __init__(self):
    self.params = Params()

    self.enabled = self.params.get_bool("BlinkerPauseLateralControl")
    self.is_metric = self.params.get_bool("IsMetric")
    self.min_speed = 0

  def get_params(self) -> None:
    self.enabled = self.params.get_bool("BlinkerPauseLateralControl")
    self.is_metric = self.params.get_bool("IsMetric")
    self.min_speed = self.params.get("BlinkerMinLateralControlSpeed")

  def update(self, CS: car.CarState) -> bool:
    if not self.enabled:
      return False

    one_blinker = CS.leftBlinker != CS.rightBlinker
    speed_factor = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
    min_speed_ms = self.min_speed * speed_factor

    return bool(one_blinker and CS.vEgo < min_speed_ms)

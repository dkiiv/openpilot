"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
from types import SimpleNamespace

from opendbc.car.volkswagen import mlbcan, mebcan, mqbcan, pqcan
from opendbc.car.volkswagen.carcontroller import CarController, LongCtrlState
from opendbc.car.volkswagen.values import HOLD_MAX_FRAMES, VolkswagenFlags


class FakeActuators(SimpleNamespace):
  def as_builder(self):
    return SimpleNamespace(torque=0.0, torqueOutputCan=0, curvature=0.0, accel=self.accel)


def simulate_standstill_receivers(stopping, starting, esp_hold, esp_starting_override=None, esp_stopping_override=None):
  acc06_starting = starting
  acc06_stopping = stopping
  acc07_starting = esp_starting_override if esp_starting_override is not None else starting
  acc07_stopping = esp_stopping_override if esp_stopping_override is not None else stopping
  split_release_hold = acc06_starting and not acc06_stopping and not acc07_starting and acc07_stopping
  tsk_faulted = acc06_starting and acc06_stopping
  esp_faulted = acc07_starting and acc07_stopping
  acc07_hold_type = 4 if starting else 3 if esp_hold else 1 if stopping else 0
  if acc07_hold_type == 4 and acc07_stopping and not split_release_hold:
    esp_faulted = True
  if acc07_hold_type == 1 and acc07_starting:
    esp_faulted = True
  if acc07_hold_type == 3 and acc07_starting:
    esp_faulted = True
  return tsk_faulted, esp_faulted


def build_controller(can_stack):
  controller = object.__new__(CarController)
  controller.CCS = can_stack
  controller.CCP = SimpleNamespace(
    ACCEL_MIN=-3.5,
    ACCEL_MAX=2.0,
    STEER_MAX=300,
    STEER_STEP=5,
    ACC_CONTROL_STEP=2,
    LDW_STEP=99,
    ACC_HUD_STEP=99,
  )
  controller.CAN = SimpleNamespace(pt=0, aux=1, ext=2, cam=3)
  controller.CP = SimpleNamespace(flags=0, openpilotLongitudinalControl=True, pcmCruise=False, vEgoStopping=0.3)
  controller.CP_IQ = SimpleNamespace()
  controller._params = SimpleNamespace(get_bool=lambda _: False)
  controller.packer_pt = None
  controller.frame = 2
  controller.accel = 0.0
  controller.accel_last = 0.0
  controller.accel_diff = 0.0
  controller.long_deviation = 0.0
  controller.long_jerklimit = 0.0
  controller.apply_torque_last = 0
  controller.apply_curvature_last = 0.0
  controller.gra_acc_counter_last = None
  controller.lead_distance_bars_last = None
  controller.distance_bar_frame = 0
  controller.long_override_counter = 0
  controller.long_disabled_counter = 0
  controller.esp_hold_frames = 2
  controller.hill_hold_accel = 0.0
  controller.detected_uphill = False
  controller.prev_impulse_count = 12
  controller.prev_starting_hold = False
  controller.prev_starting_no_hold = False
  controller.long_jerk_control = None
  controller.long_limit_control = None
  return controller


def build_state(**kwargs):
  gra_stock_values = {"COUNTER": 0, "GRA_Abbrechen": 0, "LS_Abbrechen": 0, "GRA_Typ_Hauptschalter": 0, "GRA_Hauptschalter": 0}
  gra_stock_values.update(kwargs.pop("gra_stock_values", {}))
  out = SimpleNamespace(
    brakePressed=kwargs.pop("brakePressed", False),
    standstill=kwargs.pop("standstill", True),
    cruiseState=SimpleNamespace(available=kwargs.pop("cruise_available", True)),
    vEgo=kwargs.pop("vEgo", 0.0),
    accFaulted=kwargs.pop("accFaulted", False),
  )
  state = SimpleNamespace(
    acc_type=kwargs.pop("acc_type", 1),
    esp_hold_confirmation=kwargs.pop("esp_hold_confirmation", True),
    esp_hold_uphill=kwargs.pop("esp_hold_uphill", True),
    esp_hold_torque_nm=kwargs.pop("esp_hold_torque_nm", 80.0),
    actual_torque_nm=kwargs.pop("actual_torque_nm", 0.0),
    wheel_impulse_count=kwargs.pop("wheel_impulse_count", 12),
    gra_stock_values=gra_stock_values,
    travel_assist_available=kwargs.pop("travel_assist_available", False),
    leftBlinkerUpdate=kwargs.pop("leftBlinkerUpdate", False),
    rightBlinkerUpdate=kwargs.pop("rightBlinkerUpdate", False),
    motor2_stock=kwargs.pop("motor2_stock", {}),
    out=out,
  )
  for key, value in kwargs.items():
    setattr(state, key, value)
  return state


def build_car_control(accel=-0.2):
  return SimpleNamespace(
    actuators=FakeActuators(accel=accel, longControlState=LongCtrlState.pid),
    hudControl=SimpleNamespace(
      visualAlert=0,
      leadDistanceBars=0,
      leadDistance=0.0,
      leadVisible=False,
      setSpeed=0.0,
      leadFollowTime=0.0,
      leftLaneDepart=False,
      leftLaneVisible=False,
      rightLaneDepart=False,
      rightLaneVisible=False,
    ),
    longActive=True,
    cruiseControl=SimpleNamespace(override=False, cancel=False, resume=False),
    leftBlinker=False,
    rightBlinker=False,
    latActive=False,
  )


def test_mqb_update_sends_hold_overrides(monkeypatch):
  controller = build_controller(mqbcan)
  state = build_state()
  control = build_car_control()
  captured = {}

  def fake_accel_control(*args, **kwargs):
    captured["args"] = args
    captured["kwargs"] = kwargs
    return []

  monkeypatch.setattr(mqbcan, "create_acc_accel_control", fake_accel_control)

  controller.update(control, SimpleNamespace(), state, 0)

  assert captured["args"][2] == 1
  assert captured["args"][5] is False
  assert captured["args"][6] is True
  assert captured["kwargs"]["esp_starting_override"] is False
  assert captured["kwargs"]["esp_stopping_override"] is True
  tsk_faulted, esp_faulted = simulate_standstill_receivers(
    captured["args"][5], captured["args"][6], state.esp_hold_confirmation,
    captured["kwargs"]["esp_starting_override"], captured["kwargs"]["esp_stopping_override"],
  )
  assert not tsk_faulted
  assert not esp_faulted


def test_pq_update_has_no_mqb_hold_overrides(monkeypatch):
  controller = build_controller(pqcan)
  controller.CP.flags = VolkswagenFlags.PQ
  controller.frame = 1
  controller.CCP.ACC_CONTROL_STEP = 1
  state = build_state()
  control = build_car_control()
  captured = {}

  def fake_accel_control(*args, **kwargs):
    captured["args"] = args
    captured["kwargs"] = kwargs
    return []

  monkeypatch.setattr(pqcan, "create_acc_accel_control", fake_accel_control)

  controller.update(control, SimpleNamespace(), state, 0)

  assert captured["kwargs"] == {}
  assert captured["args"][2] == 1
  assert captured["args"][5] is False
  assert captured["args"][6] is True
  tsk_faulted, esp_faulted = simulate_standstill_receivers(
    captured["args"][5], captured["args"][6], state.esp_hold_confirmation,
  )
  assert not tsk_faulted
  assert not esp_faulted


def test_mqb_acc_fts_extends_hold():
  controller = build_controller(mqbcan)
  state = build_state()

  long_active, accel, starting, stopping, start_override, stop_override = controller.mqbextendedStandstill(
    state, SimpleNamespace(), True, -0.2, False, True,
  )

  assert long_active is True
  assert accel > -0.2
  assert starting is True
  assert stopping is False
  assert start_override is False
  assert stop_override is True
  assert controller.esp_hold_frames == 3
  assert controller.hill_hold_accel > 0.0
  tsk_faulted, esp_faulted = simulate_standstill_receivers(
    stopping, starting, state.esp_hold_confirmation, start_override, stop_override,
  )
  assert not tsk_faulted
  assert not esp_faulted


def test_mqb_acc_basic_is_unchanged():
  controller = build_controller(mqbcan)
  state = build_state(acc_type=0)

  long_active, accel, starting, stopping, start_override, stop_override = controller.mqbextendedStandstill(
    state, SimpleNamespace(), True, -0.2, False, True,
  )

  assert long_active is True
  assert accel == -0.2
  assert starting is False
  assert stopping is True
  assert start_override is None
  assert stop_override is None
  assert controller.esp_hold_frames == 2
  assert controller.hill_hold_accel == 0.0
  tsk_faulted, esp_faulted = simulate_standstill_receivers(
    stopping, starting, state.esp_hold_confirmation, start_override, stop_override,
  )
  assert not tsk_faulted
  assert not esp_faulted


def test_mqb_fts_releases_after_hold_limit():
  controller = build_controller(mqbcan)
  controller.esp_hold_frames = HOLD_MAX_FRAMES + 1
  state = build_state()

  long_active, accel, starting, stopping, start_override, stop_override = controller.mqbextendedStandstill(
    state, SimpleNamespace(), True, -0.2, False, True,
  )

  assert long_active is False
  assert accel == -0.2
  assert starting is False
  assert stopping is True
  assert start_override is None
  assert stop_override is None


def test_pq_is_unchanged():
  controller = build_controller(pqcan)
  state = build_state()

  long_active, accel, starting, stopping, start_override, stop_override = controller.mqbextendedStandstill(
    state, SimpleNamespace(), True, -0.2, False, True,
  )

  assert long_active is True
  assert accel == -0.2
  assert starting is False
  assert stopping is True
  assert start_override is None
  assert stop_override is None
  tsk_faulted, esp_faulted = simulate_standstill_receivers(
    stopping, starting, state.esp_hold_confirmation, start_override, stop_override,
  )
  assert not tsk_faulted
  assert not esp_faulted


def test_mlb_is_unchanged():
  controller = build_controller(mlbcan)
  state = build_state()

  long_active, accel, starting, stopping, start_override, stop_override = controller.mqbextendedStandstill(
    state, SimpleNamespace(), True, -0.2, False, True,
  )

  assert long_active is True
  assert accel == -0.2
  assert starting is False
  assert stopping is True
  assert start_override is None
  assert stop_override is None
  tsk_faulted, esp_faulted = simulate_standstill_receivers(
    stopping, starting, state.esp_hold_confirmation, start_override, stop_override,
  )
  assert not tsk_faulted
  assert not esp_faulted


def test_meb_is_unchanged():
  controller = build_controller(mebcan)
  state = build_state()

  long_active, accel, starting, stopping, start_override, stop_override = controller.mqbextendedStandstill(
    state, SimpleNamespace(), True, -0.2, False, True,
  )

  assert long_active is True
  assert accel == -0.2
  assert starting is False
  assert stopping is True
  assert start_override is None
  assert stop_override is None
  tsk_faulted, esp_faulted = simulate_standstill_receivers(
    stopping, starting, state.esp_hold_confirmation, start_override, stop_override,
  )
  assert not tsk_faulted
  assert not esp_faulted

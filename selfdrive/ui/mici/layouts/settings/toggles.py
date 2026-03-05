import pyray as rl
from collections.abc import Callable
from cereal import log

from openpilot.system.ui.widgets.scroller import Scroller
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl, BigMultiToggle
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import NavWidget
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants
STOCK_ACC_OPTION = "Stock ACC"
IQ_STANDARD_OPTION = "IQ.Standard"
IQ_DYNAMIC_OPTION = "IQ.Dynamic"
IQ_PILOT_OPTION = "IQ.Pilot"
PERSONALITY_RELAXED = "Relaxed"
PERSONALITY_STANDARD = "Standard"
PERSONALITY_AGGRESSIVE = "Aggressive"
PERSONALITY_OPTIONS = [PERSONALITY_RELAXED, PERSONALITY_STANDARD, PERSONALITY_AGGRESSIVE]
PERSONALITY_OPTION_TO_PARAM = {
  PERSONALITY_RELAXED: PERSONALITY_TO_INT["relaxed"],
  PERSONALITY_STANDARD: PERSONALITY_TO_INT["standard"],
  PERSONALITY_AGGRESSIVE: PERSONALITY_TO_INT["aggressive"],
}
PERSONALITY_PARAM_TO_OPTION = {v: k for k, v in PERSONALITY_OPTION_TO_PARAM.items()}


class TogglesLayoutMici(NavWidget):
  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)
    # Keep IQ.Pilot enabled by default; the UI no longer exposes this toggle.
    ui_state.params.put_bool("OpenpilotEnabledToggle", True)

    self._personality_toggle = BigMultiToggle("driving personality", PERSONALITY_OPTIONS, select_callback=self._on_personality_selection)
    aol_toggle = BigParamControl("always on lateral (AOL)", "AolEnabled")
    self._longitudinal_control_selector = BigMultiToggle(
      "longitudinal control",
      [STOCK_ACC_OPTION, IQ_STANDARD_OPTION, IQ_DYNAMIC_OPTION, IQ_PILOT_OPTION],
      select_callback=self._on_longitudinal_control_selection,
    )
    # Keep selector typography compact enough for long labels.
    self._longitudinal_control_selector._label.set_font_size(26)
    self._longitudinal_control_selector._sub_label.set_font_size(24)
    is_metric_toggle = BigParamControl("use metric units", "IsMetric")
    ldw_toggle = BigParamControl("lane departure warnings", "IsLdwEnabled")
    always_on_dm_toggle = BigParamControl("always-on driver monitor", "AlwaysOnDM")
    record_front = BigParamControl("record & upload driver camera", "RecordFront", toggle_callback=restart_needed_callback)

    self._scroller = Scroller([
      self._longitudinal_control_selector,
      self._personality_toggle,
      aol_toggle,
      is_metric_toggle,
      ldw_toggle,
      always_on_dm_toggle,
      record_front,
    ], snap_items=False)

    # Toggle lists
    self._refresh_toggles = (
      ("IsMetric", is_metric_toggle),
      ("IsLdwEnabled", ldw_toggle),
      ("AlwaysOnDM", always_on_dm_toggle),
      ("RecordFront", record_front),
      ("AolEnabled", aol_toggle),
    )

    aol_toggle.set_enabled(lambda: ui_state.is_offroad())
    record_front.set_enabled(False if ui_state.params.get_bool("RecordFrontLock") else (lambda: not ui_state.engaged))

    if ui_state.params.get_bool("ShowDebugInfo"):
      gui_app.set_show_touches(True)
      gui_app.set_show_fps(True)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _update_state(self):
    super()._update_state()

    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._personality_toggle.set_value(PERSONALITY_PARAM_TO_OPTION.get(personality, PERSONALITY_STANDARD))
      ui_state.personality = personality

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    mode_value = self._get_longitudinal_control_option()
    alpha_available = bool(ui_state.CP is not None and ui_state.CP.alphaLongitudinalAvailable)

    self._longitudinal_control_selector.set_visible(True)
    self._personality_toggle.set_visible(True)

    if not alpha_available and mode_value != STOCK_ACC_OPTION:
      self._apply_longitudinal_control_selection(STOCK_ACC_OPTION)
      mode_value = STOCK_ACC_OPTION

    selector_enabled = alpha_available and not ui_state.engaged
    self._longitudinal_control_selector.set_enabled(selector_enabled)
    personality_enabled = alpha_available and mode_value in (IQ_PILOT_OPTION, IQ_DYNAMIC_OPTION)
    self._personality_toggle.set_enabled(personality_enabled)

    self._longitudinal_control_selector.set_value(mode_value)
    self._personality_toggle.set_value(
      PERSONALITY_PARAM_TO_OPTION.get(ui_state.params.get("LongitudinalPersonality", return_default=True), PERSONALITY_STANDARD)
    )

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))

  def _on_longitudinal_control_selection(self, value: str):
    alpha_available = bool(ui_state.CP is not None and ui_state.CP.alphaLongitudinalAvailable)
    if not alpha_available and value != STOCK_ACC_OPTION:
      self._longitudinal_control_selector.set_value(STOCK_ACC_OPTION)
      return

    if value == self._get_longitudinal_control_option():
      return

    previous_alpha = ui_state.params.get_bool("AlphaLongitudinalEnabled")
    self._apply_longitudinal_control_selection(value)
    if previous_alpha != ui_state.params.get_bool("AlphaLongitudinalEnabled"):
      restart_needed_callback(ui_state.params.get_bool("AlphaLongitudinalEnabled"))
      ui_state.params.put_bool("OnroadCycleRequested", True)
    self._update_toggles()

  def _on_personality_selection(self, value: str):
    ui_state.params.put("LongitudinalPersonality", PERSONALITY_OPTION_TO_PARAM[value])

  def _get_longitudinal_control_option(self) -> str:
    if not ui_state.params.get_bool("AlphaLongitudinalEnabled"):
      return STOCK_ACC_OPTION
    if not ui_state.params.get_bool("ExperimentalMode"):
      return IQ_STANDARD_OPTION
    return IQ_DYNAMIC_OPTION if ui_state.params.get_bool("IQDynamicMode") else IQ_PILOT_OPTION

  def _apply_longitudinal_control_selection(self, value: str) -> None:
    if value == IQ_PILOT_OPTION:
      ui_state.params.put_bool("AlphaLongitudinalEnabled", True)
      ui_state.params.put_bool("ExperimentalMode", True)
      ui_state.params.put_bool("IQDynamicMode", False)
    elif value == IQ_DYNAMIC_OPTION:
      ui_state.params.put_bool("AlphaLongitudinalEnabled", True)
      ui_state.params.put_bool("ExperimentalMode", True)
      ui_state.params.put_bool("IQDynamicMode", True)
    elif value == IQ_STANDARD_OPTION:
      ui_state.params.put_bool("AlphaLongitudinalEnabled", True)
      ui_state.params.put_bool("ExperimentalMode", False)
      ui_state.params.put_bool("IQDynamicMode", False)
      ui_state.params.put("LongitudinalPersonality", PERSONALITY_TO_INT["relaxed"])
    else:
      ui_state.params.put_bool("AlphaLongitudinalEnabled", False)
      ui_state.params.put_bool("ExperimentalMode", False)
      ui_state.params.put_bool("IQDynamicMode", False)

  def _render(self, rect: rl.Rectangle):
    self._scroller.render(rect)

"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
from enum import IntEnum

from openpilot.selfdrive.ui.mici.layouts.settings import settings as OP
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from .models import ModelsLayoutMici

ICON_SIZE = 70

OP.PanelType = IntEnum(
  "PanelType",
  [es.name for es in OP.PanelType] + [
    "MODELS",
  ],
  start=0,
)


class IQMiciSettingsLayout(OP.SettingsLayout):
  def __init__(self):
    OP.SettingsLayout.__init__(self)

    models_btn = BigButton("models", "", "../../iqpilot/selfdrive/assets/offroad/icon_models.png")
    models_btn.set_click_callback(lambda: self._set_current_panel(OP.PanelType.MODELS))

    self._panels.update({
      OP.PanelType.MODELS: OP.PanelInfo("models", ModelsLayoutMici(back_callback=lambda: self._set_current_panel(None))),
    })

    items = self._scroller._items.copy()

    items.insert(1, models_btn)
    self._scroller._items.clear()
    for item in items:
      self._scroller.add_widget(item)


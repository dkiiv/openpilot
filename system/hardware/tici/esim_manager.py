import threading
from dataclasses import dataclass
from enum import Enum
from queue import Queue, Empty
from typing import Callable

from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE
from openpilot.system.hardware.base import LPAError, LPAProfileNotFoundError, Profile


class EsimOperationState(Enum):
  IDLE = "idle"
  SCANNING = "scanning"
  DOWNLOADING = "downloading"
  SWITCHING = "switching"
  RENAMING = "renaming"
  DELETING = "deleting"
  REBOOTING_MODEM = "rebooting modem"
  COMPLETED = "completed"
  FAILED = "failed"


@dataclass
class EsimUiState:
  state: EsimOperationState = EsimOperationState.IDLE
  message: str = ""
  profiles: list[Profile] | None = None
  busy: bool = False


class EsimManager:
  def __init__(self):
    self._params = Params()
    self._lock = threading.Lock()
    self._callbacks: list[Callable[[EsimUiState], None]] = []
    self._state = EsimUiState()

    self._ops: Queue[Callable[[], None]] = Queue()
    self._worker = threading.Thread(target=self._worker_loop, daemon=True)
    self._worker.start()

  def is_supported(self) -> bool:
    raw_flag = self._params.get("EnableEsimProvisioning")
    enabled = True if raw_flag is None else self._params.get_bool("EnableEsimProvisioning")
    if not enabled:
      return False
    if HARDWARE.get_device_type() not in ("tici", "tizi", "mici"):
      return False
    try:
      return HARDWARE.get_sim_lpa() is not None
    except Exception:
      return False

  def add_callback(self, cb: Callable[[EsimUiState], None]) -> None:
    with self._lock:
      self._callbacks.append(cb)
      state = self._copy_state_locked()
    cb(state)

  def remove_callback(self, cb: Callable[[EsimUiState], None]) -> None:
    with self._lock:
      self._callbacks = [c for c in self._callbacks if c is not cb]

  def get_state(self) -> EsimUiState:
    with self._lock:
      return self._copy_state_locked()

  def refresh_profiles(self) -> None:
    self._enqueue(self._refresh_profiles)

  def add_profile(self, activation_code: str, nickname: str | None = None) -> None:
    def _op() -> None:
      self._set_state(EsimOperationState.DOWNLOADING, "Downloading profile...", busy=True)
      lpa = HARDWARE.get_sim_lpa()
      lpa.download_profile(activation_code, nickname=nickname)
      self._set_state(EsimOperationState.REBOOTING_MODEM, "Reconnecting modem...", busy=True)
      self._refresh_profiles()
      self._set_state(EsimOperationState.COMPLETED, "Profile added", busy=False)
    self._enqueue(_op)

  def switch_profile(self, iccid: str) -> None:
    def _op() -> None:
      self._set_state(EsimOperationState.SWITCHING, "Switching profile...", busy=True)
      lpa = HARDWARE.get_sim_lpa()
      lpa.switch_profile(iccid)
      self._set_state(EsimOperationState.REBOOTING_MODEM, "Reconnecting modem...", busy=True)
      self._refresh_profiles()
      self._set_state(EsimOperationState.COMPLETED, "Profile switched", busy=False)
    self._enqueue(_op)

  def rename_profile(self, iccid: str, nickname: str) -> None:
    def _op() -> None:
      self._set_state(EsimOperationState.RENAMING, "Renaming profile...", busy=True)
      lpa = HARDWARE.get_sim_lpa()
      lpa.nickname_profile(iccid, nickname)
      self._refresh_profiles()
      self._set_state(EsimOperationState.COMPLETED, "Profile renamed", busy=False)
    self._enqueue(_op)

  def delete_profile(self, iccid: str) -> None:
    def _op() -> None:
      self._set_state(EsimOperationState.DELETING, "Deleting profile...", busy=True)
      lpa = HARDWARE.get_sim_lpa()
      lpa.delete_profile(iccid)
      self._set_state(EsimOperationState.REBOOTING_MODEM, "Reconnecting modem...", busy=True)
      self._refresh_profiles()
      self._set_state(EsimOperationState.COMPLETED, "Profile deleted", busy=False)
    self._enqueue(_op)

  def set_scanning_state(self, scanning: bool) -> None:
    if scanning:
      self._set_state(EsimOperationState.SCANNING, "Point camera at an eSIM QR code", busy=True)
    else:
      self._set_state(EsimOperationState.IDLE, "", busy=False)

  def _enqueue(self, fn: Callable[[], None]) -> None:
    if not self.is_supported():
      self._set_state(EsimOperationState.FAILED, "eSIM provisioning is unavailable on this device", busy=False)
      return
    self._ops.put(fn)

  def _worker_loop(self) -> None:
    while True:
      try:
        op = self._ops.get(timeout=0.2)
      except Empty:
        continue
      try:
        op()
      except Exception as e:
        self._set_state(EsimOperationState.FAILED, self._map_error(e), busy=False)
      finally:
        self._ops.task_done()

  def _refresh_profiles(self) -> None:
    profiles = HARDWARE.get_sim_lpa().list_profiles()
    self._set_profiles(profiles)

  def _set_profiles(self, profiles: list[Profile]) -> None:
    with self._lock:
      self._state.profiles = profiles
      state = self._copy_state_locked()
      callbacks = list(self._callbacks)
    for cb in callbacks:
      cb(state)

  def _set_state(self, state: EsimOperationState, message: str, busy: bool) -> None:
    with self._lock:
      self._state.state = state
      self._state.message = message
      self._state.busy = busy
      snapshot = self._copy_state_locked()
      callbacks = list(self._callbacks)
    for cb in callbacks:
      cb(snapshot)

  def _copy_state_locked(self) -> EsimUiState:
    profiles = list(self._state.profiles) if self._state.profiles is not None else None
    return EsimUiState(
      state=self._state.state,
      message=self._state.message,
      profiles=profiles,
      busy=self._state.busy,
    )

  @staticmethod
  def _map_error(error: Exception) -> str:
    if isinstance(error, LPAProfileNotFoundError):
      return "Profile not found"
    if isinstance(error, LPAError):
      message = str(error)
      lower = message.lower()
      if "is euicc: no" in lower or "reports no euicc support" in lower:
        return "Insert the original comma SIM to enable eSIM provisioning on this device"
      if "certificate verify failed" in lower or "ssl" in lower or "tls" in lower:
        return "TLS validation failed while contacting SM-DP+"
      if "system time is not set" in lower:
        return "Device time is invalid; connect to network and retry"
      if "timed out" in lower or "timeout" in lower:
        return "Modem timed out while provisioning eSIM"
      if "cannot delete active profile" in lower:
        return "Cannot delete active profile"
      return message
    return str(error)


_ESIM_MANAGER: EsimManager | None = None
_ESIM_MANAGER_LOCK = threading.Lock()


def get_esim_manager() -> EsimManager:
  global _ESIM_MANAGER
  with _ESIM_MANAGER_LOCK:
    if _ESIM_MANAGER is None:
      _ESIM_MANAGER = EsimManager()
    return _ESIM_MANAGER

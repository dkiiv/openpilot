#!/usr/bin/env python3
"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
import importlib
import os
import sys
from pathlib import Path
from types import ModuleType


class ProprietaryModuleMissing(ImportError):
  pass


def _iter_proprietary_python_roots() -> list[Path]:
  roots: list[Path] = []

  env_root = Path(os.environ.get("IQPILOT_PROPRIETARY_ROOT", ""))
  if str(env_root):
    roots.append(env_root)

  repo_root = Path(__file__).resolve().parents[1]
  # Primary runtime install path on device.
  roots.append(repo_root / ".iqpilot")
  # Dev/build artifact bundle roots.
  roots.append(repo_root / "artifacts" / "iqpilot_model_selector_private")
  roots.append(repo_root / "artifacts" / "iqpilot_navd_private")
  roots.append(repo_root / "artifacts" / "iqpilot_hephaestusd_private")

  return [root / "python" for root in roots]


def _ensure_private_path() -> None:
  for python_root in _iter_proprietary_python_roots():
    if (python_root / "iqpilot_private").exists():
      python_root_str = str(python_root)
      if python_root_str not in sys.path:
        sys.path.insert(0, python_root_str)
      return


def _is_private_module_missing(error: ModuleNotFoundError, private_module_name: str) -> bool:
  missing = error.name or ""
  root = private_module_name.split(".", 1)[0]
  return missing == private_module_name or missing == root or missing.startswith(f"{private_module_name}.")


def _publish_module_symbols(public_module: ModuleType, private_module: ModuleType) -> None:
  skip = {
    "__name__",
    "__package__",
    "__loader__",
    "__spec__",
    "__file__",
    "__cached__",
    "__builtins__",
  }

  for key, value in private_module.__dict__.items():
    if key in skip:
      continue
    public_module.__dict__[key] = value

  public_module.__dict__["__private_module__"] = private_module.__name__
  if "__all__" not in public_module.__dict__:
    public_module.__dict__["__all__"] = [k for k in private_module.__dict__ if not k.startswith("_")]


def load_private_module(public_module_name: str, private_module_name: str) -> ModuleType:
  public_module = sys.modules[public_module_name]
  _ensure_private_path()

  try:
    private_module = importlib.import_module(private_module_name)
  except ModuleNotFoundError as error:
    if _is_private_module_missing(error, private_module_name):
      raise ProprietaryModuleMissing(
        f"missing proprietary module '{private_module_name}'. "
        "install the IQ Pilot private proprietary bundle into IQPILOT_PROPRIETARY_ROOT"
      ) from error
    raise

  _publish_module_symbols(public_module, private_module)
  return private_module

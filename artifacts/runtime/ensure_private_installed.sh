#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." >/dev/null && pwd )"
INSTALL_ROOT="${ROOT_DIR}/.iqpilot"
VERIFY_SCRIPT="${ROOT_DIR}/artifacts/runtime/verify_proprietary_bundle.py"

manifest_hash() {
  local manifest_path="$1"
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "${manifest_path}" | awk '{print $1}'
  else
    shasum -a 256 "${manifest_path}" | awk '{print $1}'
  fi
}

NAVD_BUNDLE="${ROOT_DIR}/artifacts/iqpilot_navd_private"
HEPHA_BUNDLE="${ROOT_DIR}/artifacts/iqpilot_hephaestusd_private"
MODEL_SELECTOR_BUNDLE="${ROOT_DIR}/artifacts/iqpilot_model_selector_private"
NAVD_STATE_FILE="${INSTALL_ROOT}/.installed_navd_manifest.sha256"
HEPHA_STATE_FILE="${INSTALL_ROOT}/.installed_hephaestusd_manifest.sha256"
MODEL_SELECTOR_STATE_FILE="${INSTALL_ROOT}/.installed_model_selector_manifest.sha256"
NAVD_SENTINEL_BASE="${INSTALL_ROOT}/python/iqpilot_private/navd/navd"
HEPHA_SENTINEL_BASE="${INSTALL_ROOT}/python/iqpilot_private/konn3kt/hephaestus/hephaestusd"
MODEL_SELECTOR_SENTINEL_BASE="${INSTALL_ROOT}/python/iqpilot_private/models/manager"

NAVD_HASH=""
HEPHA_HASH=""
MODEL_SELECTOR_HASH=""
NEED_INSTALL=0
HAVE_BUNDLE=0

module_present() {
  local base_path="$1"
  if [ -f "${base_path}.pyc" ]; then
    return 0
  fi
  if compgen -G "${base_path}".*.so >/dev/null; then
    return 0
  fi
  return 1
}

if [ -f "${NAVD_BUNDLE}/manifest.json" ]; then
  HAVE_BUNDLE=1
  NAVD_HASH="$(manifest_hash "${NAVD_BUNDLE}/manifest.json")"
  NAVD_DST_HASH=""
  if [ -f "${NAVD_STATE_FILE}" ]; then
    NAVD_DST_HASH="$(cat "${NAVD_STATE_FILE}" 2>/dev/null || true)"
  fi
  if ! module_present "${NAVD_SENTINEL_BASE}" || [ "${NAVD_HASH}" != "${NAVD_DST_HASH}" ]; then
    NEED_INSTALL=1
  fi
fi

if [ -f "${HEPHA_BUNDLE}/manifest.json" ]; then
  HAVE_BUNDLE=1
  HEPHA_HASH="$(manifest_hash "${HEPHA_BUNDLE}/manifest.json")"
  HEPHA_DST_HASH=""
  if [ -f "${HEPHA_STATE_FILE}" ]; then
    HEPHA_DST_HASH="$(cat "${HEPHA_STATE_FILE}" 2>/dev/null || true)"
  fi
  if ! module_present "${HEPHA_SENTINEL_BASE}" || [ "${HEPHA_HASH}" != "${HEPHA_DST_HASH}" ]; then
    NEED_INSTALL=1
  fi
fi

if [ -f "${MODEL_SELECTOR_BUNDLE}/manifest.json" ]; then
  HAVE_BUNDLE=1
  MODEL_SELECTOR_HASH="$(manifest_hash "${MODEL_SELECTOR_BUNDLE}/manifest.json")"
  MODEL_SELECTOR_DST_HASH=""
  if [ -f "${MODEL_SELECTOR_STATE_FILE}" ]; then
    MODEL_SELECTOR_DST_HASH="$(cat "${MODEL_SELECTOR_STATE_FILE}" 2>/dev/null || true)"
  fi
  if ! module_present "${MODEL_SELECTOR_SENTINEL_BASE}" || [ "${MODEL_SELECTOR_HASH}" != "${MODEL_SELECTOR_DST_HASH}" ]; then
    NEED_INSTALL=1
  fi
fi

if [ "${HAVE_BUNDLE}" -eq 0 ]; then
  exit 0
fi

if [ "${NEED_INSTALL}" -eq 0 ]; then
  exit 0
fi

TMP_ROOT="${INSTALL_ROOT}.tmp.$$"
echo "Installing bundled private artifacts..."
rm -rf "${TMP_ROOT}"
mkdir -p "${TMP_ROOT}"

if [ -d "${INSTALL_ROOT}" ]; then
  cp -a "${INSTALL_ROOT}"/. "${TMP_ROOT}/"
fi

if [ -f "${NAVD_BUNDLE}/manifest.json" ]; then
  python3 "${VERIFY_SCRIPT}" "${NAVD_BUNDLE}"
  cp -a "${NAVD_BUNDLE}"/. "${TMP_ROOT}/"
fi

if [ -f "${HEPHA_BUNDLE}/manifest.json" ]; then
  python3 "${VERIFY_SCRIPT}" "${HEPHA_BUNDLE}"
  cp -a "${HEPHA_BUNDLE}"/. "${TMP_ROOT}/"
fi

if [ -f "${MODEL_SELECTOR_BUNDLE}/manifest.json" ]; then
  python3 "${VERIFY_SCRIPT}" "${MODEL_SELECTOR_BUNDLE}"
  cp -a "${MODEL_SELECTOR_BUNDLE}"/. "${TMP_ROOT}/"
fi

if [ -d "${INSTALL_ROOT}" ]; then
  rm -rf "${INSTALL_ROOT}.bak"
  mv "${INSTALL_ROOT}" "${INSTALL_ROOT}.bak"
fi

mv "${TMP_ROOT}" "${INSTALL_ROOT}"

if [ -d "${INSTALL_ROOT}.bak" ]; then
  rm -rf "${INSTALL_ROOT}.bak"
fi

if [ -n "${NAVD_HASH}" ]; then
  printf '%s\n' "${NAVD_HASH}" > "${NAVD_STATE_FILE}"
fi
if [ -n "${HEPHA_HASH}" ]; then
  printf '%s\n' "${HEPHA_HASH}" > "${HEPHA_STATE_FILE}"
fi
if [ -n "${MODEL_SELECTOR_HASH}" ]; then
  printf '%s\n' "${MODEL_SELECTOR_HASH}" > "${MODEL_SELECTOR_STATE_FILE}"
fi

echo "Private artifacts installed."

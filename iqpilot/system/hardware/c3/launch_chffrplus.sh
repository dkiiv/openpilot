#!/usr/bin/env bash

IQ_C3_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
DIR="$( cd "$IQ_C3_DIR/../../../.." >/dev/null 2>&1 && pwd )"

source "$IQ_C3_DIR/launch_env.sh"

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0


  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$IQ_C3_DIR/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find ${DIR}/.git -newer ${DIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $DIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $DIR
          cd $DIR

          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        else
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi

  # Install/update proprietary runtime bundles.
  if [ -f "$DIR/artifacts/runtime/ensure_private_installed.sh" ]; then
    bash "$DIR/artifacts/runtime/ensure_private_installed.sh" || true
  elif [ -x "$DIR/scripts/iqpilot/ensure_navd_private_installed.sh" ]; then
    # Backward-compat fallback for older trees.
    "$DIR/scripts/iqpilot/ensure_navd_private_installed.sh" || true
  fi

  ln -sfn $(pwd) /data/pythonpath
  export IQPILOT_PROPRIETARY_ROOT="${IQPILOT_PROPRIETARY_ROOT:-$PWD/.iqpilot}"
  export IQPILOT_SOURCE_ROOT="${IQPILOT_SOURCE_ROOT:-$PWD/openpilot}"
  if [ -d "$IQPILOT_PROPRIETARY_ROOT/python" ]; then
    export PYTHONPATH="$IQPILOT_PROPRIETARY_ROOT/python:$PWD"
  else
    export PYTHONPATH="$PWD"
  fi

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  # start manager
  cd $DIR/system/manager
  if [ ! -f $DIR/prebuilt ]; then
    ./build.py
  fi

  # Install Hephaestus systemd service (starts independently after build completes)
  if [ -f "$DIR/system/hephaestusd.service" ]; then
    if [ ! -f /etc/systemd/system/hephaestusd.service ] || \
       ! cmp -s "$DIR/system/hephaestusd.service" /etc/systemd/system/hephaestusd.service; then
      sudo mount -o remount,rw /
      sudo cp "$DIR/system/hephaestusd.service" /etc/systemd/system/hephaestusd.service
      sudo systemctl daemon-reload
      sudo systemctl enable hephaestusd.service
      sudo mount -o remount,ro /
      sudo systemctl restart hephaestusd.service
    elif ! systemctl is-active --quiet hephaestusd.service; then
      sudo systemctl start hephaestusd.service
    fi
  fi

  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch

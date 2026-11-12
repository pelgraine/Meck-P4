#!/usr/bin/env bash
# build-variant.sh — build one Meck-P4 release variant as a merged .bin
#
# Usage:
#   tools/ci/build-variant.sh <variant>
#
#   variant    the name of a file in tools/ci/variants/ without ".conf",
#              e.g. "amoled-LR2021-ble-garmin" or "tft-nokeyboard-ble".
#
# Output:
#   release/meck-p4-<version>-<variant>-merged.bin
#   (<version> is MECK_FIRMWARE_VERSION in components/meshcore/meck.h)
#
# How it works:
#   1. Copies tools/ci/sdkconfig.base to sdkconfig. The base is a complete
#      settings file for the AMOLED, LR2021, keyboard, Bluetooth, Garmin
#      build.
#   2. Appends the variant's changes (tools/ci/variants/<variant>.conf).
#   3. Runs "idf.py reconfigure" and checks that every change took effect,
#      so a build never goes out with the wrong board, radio or connection.
#   4. Runs tools/build-release.sh to build and merge.
#
# Used by .github/workflows/build-firmware.yml. Also runs on a Mac after
# ". $IDF_PATH/export.sh"; note that it REPLACES the sdkconfig in the repo
# folder.

set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: tools/ci/build-variant.sh <variant>" >&2
    exit 1
fi
VARIANT="$1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

BASE="tools/ci/sdkconfig.base"
CONF="tools/ci/variants/${VARIANT}.conf"
if [[ ! -f "$CONF" ]]; then
    echo "ERROR: no such variant: $CONF" >&2
    exit 1
fi

VERSION="$(sed -n 's/^#define MECK_FIRMWARE_VERSION "\(.*\)".*/\1/p' components/meshcore/meck.h)"
if [[ -z "$VERSION" ]]; then
    echo "ERROR: MECK_FIRMWARE_VERSION not found in components/meshcore/meck.h" >&2
    exit 1
fi

echo "==> Meck-P4 ${VERSION}, variant ${VARIANT}"

# ---- Settings: base + this variant's changes ---------------------------------
cp "$BASE" sdkconfig
{
    echo
    echo "# --- variant: ${VARIANT} ---"
    cat "$CONF"
} >> sdkconfig

idf.py reconfigure

# ---- Check every change took effect -------------------------------------------
FAIL=0
while IFS= read -r line; do
    case "$line" in
        CONFIG_*=*)
            if ! grep -qxF "$line" sdkconfig; then
                echo "ERROR: expected setting is missing: $line" >&2
                FAIL=1
            fi
            ;;
        "# CONFIG_"*" is not set")
            sym="${line#\# }"
            sym="${sym% is not set}"
            if grep -q "^${sym}=" sdkconfig; then
                echo "ERROR: setting should be off but is on: $sym" >&2
                FAIL=1
            fi
            ;;
    esac
done < "$CONF"
if ! grep -qxF "CONFIG_EXAMPLE_BUILD_LVGL_9_UI=y" sdkconfig; then
    echo "ERROR: the lvgl_9_ui example (Meck) is no longer the one selected" >&2
    FAIL=1
fi

echo "==> Key settings in this build:"
grep -E '^(# )?CONFIG_(SCREEN_TYPE_(HI8561|RM69A10)|MECK_RADIO_(SX1262|LR2021)|BOARD_TYPE_T_DISPLAY_P4(_KEYBOARD)?|MECK_C6_ESP_HOSTED_TEST|MECK_BLE_GARMIN|BT_ENABLED|BT_NIMBLE_ENABLED|BT_NIMBLE_MEM_ALLOC_MODE_EXTERNAL|BT_NIMBLE_NVS_PERSIST|BT_NIMBLE_TRANSPORT_UART|ESP_HOSTED_ENABLE_BT_NIMBLE|ESP_HOST_WIFI_ENABLED|ESP_WIFI_REMOTE_ENABLED|ESP_HOSTED_ENABLED)[= ]' sdkconfig | sed 's/^/    /' || true

if [[ $FAIL -ne 0 ]]; then
    echo "ERROR: settings check failed for ${VARIANT}; not building." >&2
    exit 1
fi

# ---- Build and merge ------------------------------------------------------------
ALLOW_DIRTY=1 bash tools/build-release.sh "${VERSION}-${VARIANT}-merged"

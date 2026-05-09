#!/usr/bin/env bash
# build-release.sh — produce a single flashable Meck firmware bin
#
# Usage:
#   tools/build-release.sh [version]
#
#   version    optional, e.g. "0.1" or "0.1-rc1". Defaults to the short
#              git rev (or "dev" if not in a git checkout).
#
# Output:
#   release/meck-p4-<version>.bin     single merged image
#   release/meck-p4-<version>.sha256  checksum
#
# Flashing the produced file:
#   esptool.py --chip esp32p4 -p PORT write_flash 0x0 meck-p4-<version>.bin
#
# Requirements:
#   - idf.py on PATH (run ". $HOME/esp/esp-idf/export.sh" first)
#   - esptool.py on PATH (ships with esp-idf)
#   - python3
#
# Behaviour:
#   - Refuses to build if the working tree has uncommitted changes
#     (override with ALLOW_DIRTY=1).
#   - Reads flash params (offsets, mode, freq, size) from
#     build/flash_project_args so the merge always matches sdkconfig.
#   - Sanity-checks the merged bin for absolute paths and warns if any
#     leak through.

set -euo pipefail

# Resolve project root from the script's location so it works regardless
# of where it's run from.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

# ---- Determine version string ------------------------------------------------
if [[ $# -ge 1 ]]; then
    VERSION="$1"
elif command -v git >/dev/null 2>&1 && git rev-parse --git-dir >/dev/null 2>&1; then
    VERSION="$(git describe --tags --dirty --always 2>/dev/null || git rev-parse --short HEAD)"
else
    VERSION="dev"
fi
OUTPUT_NAME="meck-p4-${VERSION}"

# ---- Refuse dirty trees unless explicitly allowed ----------------------------
if [[ "${ALLOW_DIRTY:-0}" != "1" ]]; then
    if command -v git >/dev/null 2>&1 && git rev-parse --git-dir >/dev/null 2>&1; then
        if ! git diff --quiet || ! git diff --cached --quiet; then
            echo "ERROR: working tree has uncommitted changes." >&2
            echo "Commit, stash, or set ALLOW_DIRTY=1 to override." >&2
            exit 1
        fi
    fi
fi

# ---- Build -------------------------------------------------------------------
echo "==> Building Meck v${VERSION}"
idf.py build

# ---- Read merge args from build/flash_project_args ---------------------------
# Format on disk is space-separated tokens: --flash_mode dio --flash_freq 80m
# --flash_size 16MB <offset> <file> [<offset> <file>...]. We pull the binary
# pairs out in the same order esptool would consume them.
ARGS_FILE="build/flash_project_args"
if [[ ! -f "$ARGS_FILE" ]]; then
    echo "ERROR: $ARGS_FILE not found — was this built with idf.py?" >&2
    exit 1
fi

# Extract flash mode/freq/size tokens (preserve them as-is for merge_bin).
FLASH_MODE=$(awk '/--flash_mode/{getline; print; exit}' < <(tr ' ' '\n' < "$ARGS_FILE"))
FLASH_FREQ=$(awk '/--flash_freq/{getline; print; exit}' < <(tr ' ' '\n' < "$ARGS_FILE"))
FLASH_SIZE=$(awk '/--flash_size/{getline; print; exit}' < <(tr ' ' '\n' < "$ARGS_FILE"))

# Pull <offset> <file> pairs. flash_project_args lists them on a single line
# after the flag tokens. We strip the flag tokens then reassemble pairs.
PAIRS=$(python3 <<'PY'
import re, pathlib
text = pathlib.Path("build/flash_project_args").read_text().strip()
toks = text.split()
out = []
i = 0
while i < len(toks):
    t = toks[i]
    if t.startswith("--"):
        # skip flag and its value
        i += 2
        continue
    # offset followed by filename
    if re.match(r"^0x[0-9a-fA-F]+$", t) and i + 1 < len(toks):
        out.append(t)
        out.append(toks[i+1])
        i += 2
        continue
    i += 1
print(" ".join(out))
PY
)

if [[ -z "$PAIRS" ]]; then
    echo "ERROR: failed to parse offset/file pairs from $ARGS_FILE" >&2
    cat "$ARGS_FILE" >&2
    exit 1
fi

# ---- Merge -------------------------------------------------------------------
mkdir -p release
OUTPUT_BIN="release/${OUTPUT_NAME}.bin"

echo "==> Merging into ${OUTPUT_BIN}"
echo "    flash_mode=${FLASH_MODE} flash_freq=${FLASH_FREQ} flash_size=${FLASH_SIZE}"
echo "    pairs: ${PAIRS}"

# shellcheck disable=SC2086  # PAIRS is intentionally word-split
esptool.py --chip esp32p4 merge_bin \
    -o "$OUTPUT_BIN" \
    --flash_mode "$FLASH_MODE" \
    --flash_freq "$FLASH_FREQ" \
    --flash_size "$FLASH_SIZE" \
    $PAIRS

# ---- Sanity checks -----------------------------------------------------------
echo "==> Sanity checks"

# Absolute path leak check. We look for common host-path roots; add to this
# list if you build on a different OS / location.
LEAK_PATTERNS=(
    "/Users/"
    "/home/"
    "/private/var/folders/"
    "elizabethseabrooke"
)
LEAK_FOUND=0
for pat in "${LEAK_PATTERNS[@]}"; do
    matches=$(strings "$OUTPUT_BIN" | grep -F "$pat" | head -5 || true)
    if [[ -n "$matches" ]]; then
        if [[ $LEAK_FOUND -eq 0 ]]; then
            echo "WARNING: host paths leaked into the binary." >&2
            echo "         Enable CONFIG_COMPILER_HIDE_PATHS_MACROS in sdkconfig" >&2
            echo "         (Compiler options → Replace ESP-IDF and project paths)." >&2
            LEAK_FOUND=1
        fi
        echo "  pattern '$pat':" >&2
        echo "$matches" | sed 's/^/    /' >&2
    fi
done
if [[ $LEAK_FOUND -eq 0 ]]; then
    echo "    no host paths detected"
fi

# ---- Checksum ----------------------------------------------------------------
if command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$OUTPUT_BIN" > "${OUTPUT_BIN%.bin}.sha256"
elif command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$OUTPUT_BIN" > "${OUTPUT_BIN%.bin}.sha256"
fi

# ---- Summary -----------------------------------------------------------------
SIZE=$(wc -c < "$OUTPUT_BIN" | tr -d ' ')
echo
echo "==> Done"
echo "    File:    $OUTPUT_BIN"
echo "    Size:    $SIZE bytes ($(echo "scale=2; $SIZE/1024/1024" | bc) MB)"
echo "    SHA256:  $(cut -d' ' -f1 < "${OUTPUT_BIN%.bin}.sha256")"
echo
echo "Flash with:"
echo "    esptool.py --chip esp32p4 -p PORT write_flash 0x0 $(basename "$OUTPUT_BIN")"

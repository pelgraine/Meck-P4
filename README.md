# Meck-P4 — Meshcore for the LilyGo T-Display P4

A port of [Meck](https://github.com/pelgraine/Meck) (a MeshCore fork) to the
LilyGo T-Display P4. Targets the ESP32-P4 main MCU; the onboard ESP32-C6 is
not currently used by Meck on this device.

This repository is a fork of LilyGo's [T-Display-P4](https://github.com/Xinyuan-LilyGO/T-Display-P4)
example tree, with a `meshcore` ESP-IDF component added that brings up the
SX1262 LoRa radio and provides an LVGL-based standalone UI.

**Status:** Early but functional. Boots straight to the Meck home screen,
joins the MeshCore mesh, sends and receives channel messages, persists
history across reboots. See the [Discord channel](https://discord.com/channels/1495203904898728149/1500323702859104457)
for the latest progress.

## Hardware

LilyGo T-Display P4 (TFT version). The AMOLED variant has not been tested
but should work after adjusting the display init in
`main/examples/lvgl_9_ui/main.cpp`.

## Toolchain

The build uses ESP-IDF, not PlatformIO. (Meck for the T-Deck Pro and T5S3
uses PlatformIO; this is the difference.) You will need:

- **ESP-IDF v5.4.1** — install via Espressif's [official instructions](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32p4/get-started/index.html).
  The P4 target requires v5.4 or later.
- **Python 3.11+** (comes with ESP-IDF)
- A USB-C cable and the LilyGo board

Once ESP-IDF is installed and `. $IDF_PATH/export.sh` is sourced in your
shell, `idf.py` will be on your PATH.

## Build and flash

```
git clone https://github.com/pelgraine/Meck-P4
cd Meck-P4

# Set the target the first time
idf.py set-target esp32p4

# Build
idf.py build

# Flash and monitor
idf.py flash monitor
```

Press `Ctrl-]` to exit the monitor.

If `idf.py flash` can't find your device, specify the port explicitly:

```
idf.py -p /dev/cu.usbmodemXXXX flash monitor       # macOS
idf.py -p /dev/ttyACM0 flash monitor                # Linux
idf.py -p COM5 flash monitor                        # Windows
```

## SD card

An SD card formatted FAT32 is recommended but not required. With one
inserted, channel message history persists across reboots; without one,
the device works but loses history on reboot. The device creates the
`/sdcard/meshcore/` directory tree automatically.

## Default radio settings

Meck-P4 boots on Australia Narrow (916.575 MHz / SF7 / BW 62.5 kHz / CR 4/8 /
sync 0x1424) with a TX power of 22 dBm. Change these in Settings on the
device, or edit the defaults in `components/meshcore/variant.h` before
building.

## Repository layout

- `components/meshcore/` — the Meck radio, mesh, persistence, and UI code
- `main/examples/lvgl_9_ui/` — LilyGo's display + LVGL bring-up, lightly
  modified to hand off to Meck after init
- `components/cpp_bus_driver/` — LilyGo's hardware driver collection
  (BQ27220 fuel gauge, SX1262 radio, XL9535 IO expander, etc.)
- everything else — straight from the upstream LilyGo example tree

The Meck-specific work lives almost entirely in `components/meshcore/`. If
you want to hack on the firmware, that's the directory to look at first.
Of particular note:

- `MeckUI.cpp` — LVGL screens, settings, navigation
- `MeckMesh.h` — protocol-side hooks: receive, send, advert handling, ring
  buffers
- `MeckDataStore.h` — NVS and SD persistence
- `target.cpp` — radio attach, deferred-config queue, battery accessors

## Known limitations

- AMOLED variant untested
- ESP32-C6 (WiFi/BLE coprocessor) is present on the board but not yet
  used; companion-mode features from upstream Meck haven't been ported
- GPS module is initialised by LilyGo's stack but Meck doesn't yet read
  from it
- Direct messages don't yet persist to SD (channel messages do)
- Power management is minimal — expect short battery life until sleep is
  wired up

Most of these are queued for future work. If you want to chip away at
any of them, PRs welcome.

## Differences from upstream Meck (T-Deck Pro / T5S3 builds)

The P4 build is structurally a different beast: ESP-IDF instead of
PlatformIO, MIPI DSI display instead of e-paper, capacitive touch + virtual
keyboard instead of physical keys, RISC-V instead of Xtensa. The protocol
layer is shared MeshCore code, but the integration glue (UI, drivers,
persistence) is largely new.

Some Meck features from the T-Deck Pro / T5S3 builds aren't present yet:
e-book reader, audiobook player, alarm clock, web reader, IRC, voice notes,
SMS/phone, OTA updates, font picker, dark mode toggle (the P4 UI is
dark-by-default and stays that way for now). Most are reachable; some
(audio) require hardware that the P4 lacks.

## Contributing

Open an issue first for anything substantial — it's faster to agree on
direction before code than to rework after the fact. Style follows the
existing files (concise, embedded-style C++; no dynamic allocation outside
init; no retroactive reformatting of unchanged code).

For minor fixes, just open a PR.

## License

MIT for Meck-specific code. The wider project links libraries with mixed
licensing including GPL-3.0; the combined firmware binary is effectively
GPL-3.0 when distributed. See the upstream Meck README for the full
dependency license matrix.

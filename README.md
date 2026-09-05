# Meck-P4 — MeshCore for the LilyGo T-Display P4

A port of [Meck](https://github.com/pelgraine/Meck) (a MeshCore fork) to the
LilyGo T-Display P4. Targets the ESP32-P4 main MCU; as of v0.4 the onboard
ESP32-C6 is used for WiFi companion connectivity to the MeshCore app (BLE is
not yet enabled). Built on top of LilyGo’s
[T-Display-P4](https://github.com/Xinyuan-LilyGO/T-Display-P4) example tree
with a `meshcore` ESP-IDF component added on top.

[Check out the Meck-P4 discussion channel on the MeshCore Discord](https://discord.com/channels/1495203904898728149/1500323702859104457)

<img width="260" height="460" alt="IMG_3095" src="https://github.com/user-attachments/assets/f2a3eeb8-d23a-49aa-a725-da6429b20d9e" />

### Contents

- [Supported Devices](#supported-devices)
- [SD Card Requirements](#sd-card-requirements)
- [Flashing Firmware](#flashing-firmware)
  - [First-Time Flash (Merged Firmware)](#first-time-flash-merged-firmware)
  - [Building from Source](#building-from-source)
- [Home Screen](#home-screen)
- [Timezones](#timezones)
- [Touch Navigation](#touch-navigation)
- [Screen-Off Power Saving](#screen-off-power-saving)
- [Screen Orientation](#screen-orientation)
- [Virtual Keyboard](#virtual-keyboard)
- [T-Display P4 Keyboard (K270)](#t-display-p4-keyboard-k270)
  - [Connecting it](#connecting-it)
  - [Batteries](#batteries)
  - [Keyboard backlight](#keyboard-backlight)
  - [Typing](#typing)
  - [Navigating without touching the screen](#navigating-without-touching-the-screen)
  - [The onboard radios](#the-onboard-radios)
- [CardKB](#cardkb)
- [Channel Messages](#channel-messages)
- [Canned Messages](#canned-messages)
- [Channel Picker](#channel-picker)
- [Contacts](#contacts)
- [Direct Messages](#direct-messages)
- [Repeater Admin](#repeater-admin)
- [Room Servers](#room-servers)
- [Region Scope](#region-scope)
- [Notification Sounds](#notification-sounds)
- [Per-Contact Path Editor](#per-contact-path-editor)
- [Trace Route](#trace-route)
- [Path View](#path-view)
- [Position Adverts and Share Position](#position-adverts-and-share-position)
- [Private Channels](#private-channels)
- [Discover](#discover)
- [Audio Player](#audio-player)
- [Reader](#reader)
- [Notes](#notes)
- [Web Reader](#web-reader)
- [Maps](#maps)
- [Games](#games)
- [Config Export](#config-export)
- [Config Import](#config-import)
- [Debug Logs](#debug-logs)
- [Settings](#settings)
- [WiFi Companion](#wifi-companion)
- [GPS](#gps)
- [Battery](#battery)
- [Clock Sync](#clock-sync)
- [Persistence](#persistence)
- [Default Radio Settings](#default-radio-settings)
- [Repository Layout](#repository-layout)
- [Differences from upstream Meck](#differences-from-upstream-meck-t-deck-pro--t5s3-builds)
- [Contributing](#contributing)
- [Road-Map / To-Do](#road-map--to-do)
- [License](#license)

-----

## Supported Devices

Meck-P4 runs on the LilyGo T-Display P4 in **both** of its display variants, and both are verified working. The two boards are otherwise identical; they differ only in the screen and its touch controller. Select the panel at build time with `CONFIG_SCREEN_TYPE_HI8561` (TFT) or `CONFIG_SCREEN_TYPE_RM69A10` (AMOLED).

|Variant   |Panel model    |Display                          |Display driver|Touch driver       |
|----------|---------------|---------------------------------|--------------|-------------------|
|**TFT**   |H0405S002T002  |a-Si TFT, 4.05", 540 × 1168 px   |HI8561        |HI8561 (integrated)|
|**AMOLED**|H0410S001AMT001|a-Si AMOLED, 4.1", 568 × 1232 px |RM69A10       |GT9895             |

Both panels are 10-point capacitive touch, and the UI is rendered rotated to portrait. Everything outside the screen is common to both boards: an **ESP32-P4** (RISC-V dual-core) with 16 MB flash and 32 MB PSRAM; an onboard **ESP32-C6** (WiFi 6 / BLE 5.3) coprocessor over SDIO that provides WiFi companion connectivity to the MeshCore app as of v0.4 (BLE not yet enabled); an **SX1262** LoRa radio on the HPD16A module; a **BQ27220** fuel gauge (1000 mAh) with an **LGS4056H** charger; an **L76K** GPS on UART1; a **PCF8563** RTC (initialised but not yet used); an **ES8311** audio codec (NS4150B amplifier plus an electret microphone); an **ICM20948** IMU; an **AW86224** haptic motor; an **OV2710** MIPI camera; and an **XL9535** IO expander.

LilyGo also makes an **LR2021** radio variant of the board, and Meck-P4 supports it with its own firmware build (the standard builds are for the SX1262). On sub-GHz it behaves exactly like the SX1262 boards, and it adds **2.4 GHz LoRa**: the Radio Preset picker gains three 2.4 GHz presets (**2.4GHz Sydney**, **2.4GHz (2450)** and **2.4GHz Wide (2450)**), and on 2.4 GHz frequencies TX power is limited to **10 or 12 dBm**, the LR2021's 2.4 GHz ceiling. **2.4 GHz currently works on the internal antenna only** — the external antenna path is sub-GHz only for now. Sub-GHz works on Internal or External as usual on either radio.

-----

## SD Card Requirements

An SD card formatted **FAT32** is recommended but not strictly required.
With one inserted, every saved setting (radio prefs, channels, contacts,
identity) is mirrored to `/sdcard/meshcore/` automatically alongside the
NVS write, and channel message history persists across reboots. Without an
SD card the device still works — NVS holds settings in flash — but message
history is lost on reboot and you have no fallback if NVS is wiped.

If you’ve previously saved settings to SD and then erase NVS (factory
reset, fresh flash), the device automatically restores everything from the
SD backup on first boot.

-----

## Flashing Firmware

Download the latest firmware from the
[Releases](https://github.com/pelgraine/Meck-P4/releases) page. The release
file is a **merged binary** containing the bootloader, partition table,
and application combined into a single image — flash it at address `0x0`.

### First-Time Flash (Merged Firmware)

**Using the MeshCore Flasher (web-based):**

1. Go to <https://flasher.meshcore.io>
1. Scroll to the bottom and select **Custom Firmware**
1. Select the `meck-p4-X.Y.bin` file you downloaded
1. Click **Flash**, choose your device in the popup, and click **Connect**

Ensure you use the **right-side USB-C port** (the data port), not the high-speed charger port, to flash.
<img width="377" height="284" alt="correct port usage for flashing" src="https://github.com/user-attachments/assets/803b96bd-c93b-4699-a0d5-a680d3c304a9" />

If the flasher has completed successfully and your screen hasn't changed after flashing for the first time, unplug the device, turn it off and then on again.

**Using esptool.py:**

```
pip install esptool
esptool.py --chip esp32p4 -p /dev/cu.usbmodemXXXX write_flash 0x0 meck-p4-0.1.bin
```

(Replace the port with whatever your device shows up as. On macOS this
will be `/dev/cu.usbmodem*`, on Linux `/dev/ttyACM0`, on Windows a COM
port like `COM3`.)

If you’ve previously had something else on the device, run
`esptool.py --chip esp32p4 -p PORT erase_flash` first to clear NVS so Meck
starts with clean defaults.

### Building from Source

Meck-P4 uses ESP-IDF, not PlatformIO. (Meck for the T-Deck Pro and T5S3
uses PlatformIO; this is the difference.) You will need:

- **ESP-IDF v5.4.1 or later** — install via Espressif’s
  [official instructions](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32p4/get-started/index.html).
  The P4 target requires v5.4 or later.
- **Python 3.11+** (comes with ESP-IDF)
- A USB-C cable and the LilyGo board

Once ESP-IDF is installed and `. $IDF_PATH/export.sh` is sourced in your
shell, `idf.py` will be on your PATH. Then:

```
git clone https://github.com/pelgraine/Meck-P4
cd Meck-P4

# Set the target the first time
idf.py set-target esp32p4

# Build, flash, and monitor
idf.py flash monitor
```

Press `Ctrl-]` to exit the monitor. If `idf.py flash` can’t find your
device, specify the port explicitly with `-p /dev/cu.usbmodemXXXX` (or
your platform’s equivalent).

To produce a single merged release image (for sharing or publishing):

```
tools/build-release.sh 0.1
```

This generates `release/meck-p4-0.1.bin` along with a SHA-256 checksum.

-----

## Home Screen

The home screen is a horizontal eight-page layout. Swipe left or right to
navigate between pages. The Home page (page 0) shows node name, unread
message count, battery percentage and clock in the top-right corner, and
a ten-tile navigation grid (2 columns x 5 rows) linking to Messages,
Contacts, Settings, Reader, Notes, Discover, Trace, Maps, Audio, and Web.
A sixth row — **Voice** and **Games** — is hidden by default: swipe down on
the grid to reveal it and swipe up to hide it again. Voice opens the voice
message screens (experimental); Games opens the Game Boy emulator. See
[Games](#games).

|Page           |Purpose                                                                                            |
|---------------|---------------------------------------------------------------------------------------------------|
|0 Home         |Node name, unread message count, clock + battery, ten-tile navigation grid                         |
|1 Recent Heard |Live list of nodes whose adverts have been received                                                |
|2 Radio Details|Current frequency, bandwidth, spreading factor, coding rate, TX power, sync word                   |
|3 Advert       |Long-press to send a manual advert                                                                 |
|4 GPS          |Fix status, satellites, position, altitude, sentence rate. Long-press the tile to toggle GPS on/off|
|5 Battery      |Voltage, charge percent, current, chip temperature, remaining mAh                                  |
|6 Timezones    |World clock: Home plus two configurable zones. See [Timezones](#timezones)                         |
|7 Shutdown     |Long-press to power down                                                                           |

Paging wraps at both ends: swiping backwards from the Home page brings up
the Timezones page, and swiping forwards from Shutdown returns to the
Home page. The keyboard arrow keys follow the same cycle.

-----

## Timezones

The Timezones home page is a three-row world clock, ported from Meck
Watch. Swipe to it like any other home page -- it sits between Battery and
Shutdown, or one backwards swipe from the Home page thanks to the
wraparound.

Each row shows:

- the zone label and its UTC offset -- **Home** (your device's UTC offset,
  the same one set in Settings), **Zone 1**, and **Zone 2**
- the current local time in that zone
- a yellow **+1D** or **-1D** marker when that zone is on a different
  calendar day to Home
- a triplet of city codes for the offset (e.g. `PER/BEI/HKG` for UTC+8),
  chosen to avoid daylight-saving cities where a well-known non-DST city
  exists, so the labels stay correct year-round

**Changing a zone:** tap a row to open its offset picker. The big UTC
readout and the city line update live as you tap **-** and **+**
(offsets run UTC-12 to UTC+14); **Confirm** saves, and the back chevron
cancels. Editing the **Home** row changes the device's own UTC offset --
exactly the same setting as Settings > UTC Offset, kept in sync both
ways. Zone 1 and Zone 2 persist across reboots.

The rows appear once the device knows the time; until then the page shows
**Clock not set**. The page follows the same clock as the status bar, so
if the bar shows a time, the Timezones page shows the zones.

With the K270 keyboard, the page is fully drivable without touch: Up/Down
cycle a selection ring through the three rows, Enter opens the ringed
row's picker, and inside the picker Up/Down adjust the offset, Enter
confirms, and Esc cancels.

-----

## Touch Navigation

The T-Display P4 has no physical keyboard. All interaction is via touch
gestures. Text entry uses an on-screen virtual keyboard that appears when
needed.

|Gesture       |Description                                                                                                   |
|--------------|--------------------------------------------------------------------------------------------------------------|
|**Tap**       |Touch and release quickly. Opens tiles, selects items, advances pages.                                        |
|**Swipe**     |Touch, drag, release. Direction determines action (scroll, page turn, switch tile/filter). Home paging wraps at both ends. Swipe down on the home grid to reveal the hidden Voice and Games tiles, up to hide them.|
|**Long press**|Touch and hold. Context-dependent: send advert, toggle GPS, delete contacts, retry failed messages, view incoming message path, power off.|

-----

## Screen-Off Power Saving

The Auto Off timer (Settings > Auto Off, tap to cycle: Never / 1 / 2 / 5 / 10 / 30 minutes) puts the device into a low-power screen-off state after the set period of inactivity. When the timer fires the display is switched off, and on the AMOLED build the display pipeline behind it is shut down as well, releasing the driver lock that otherwise pins the CPU at full speed — with the screen off the CPU then spends most of its time (85-88% of it in testing) at 40 MHz instead of 360 MHz. On the TFT build the screen switches off but its display driver still holds the CPU at full speed. Either way the radio stays active and continues receiving messages in the background.

On keyboard (K270) builds, every keypress counts as activity and resets
the idle timer, so the screen never blanks mid-typing or while you are
navigating by keyboard.

> **To wake the device, press the P4 boot button on the side of the T-Display P4.** Touch wake is not yet supported. The P4 boot button is the third button from the top on the right edge, between the C6 Reset and P4 Reset buttons (labelled “BOOT” under the “P4” heading on the case). If the screen stays black after a period of inactivity, this is normal – press the boot button to bring it back.

<img width="300" height="600" alt="40BA63C0-4825-4C39-BA60-AAA020A8F475" src="https://github.com/user-attachments/assets/a5eebc32-ac66-459a-80e4-13d4081b1456" />

On the AMOLED build, waking rebuilds the display pipeline in about 130 ms and deliberately skips the panel's hardware reset — testing showed a reset added about 170 ms to every wake without fixing anything, so it now runs only at boot. This is not light sleep: the PM config keeps `light_sleep_enable=false`, and the saving comes from dynamic frequency scaling stepping the CPU between 360 MHz and 40 MHz as the display workload comes and goes.

-----

## Screen Orientation

Meck-P4 can run in **portrait** (the default) or **landscape**. The orientation is set from **Settings > Orientation**, which toggles between the two.

Changing it applies live. The display rotation is switched (0 degrees for portrait, 90 degrees for landscape), every screen is torn down and rebuilt at the new logical resolution, and you are returned to the home screen. The choice persists via NVS and is re-applied on the next boot before any screen is drawn.

The main screens adapt to the orientation: the home navigation grid is 2 columns by 5 rows in portrait and 5 columns by 2 rows in landscape, and the on-screen keyboard height scales with the panel so it stays usable either way. A few secondary screens (the audio player, maps, and the reader) are still laid out at the fixed portrait dimensions and do not yet re-flow in landscape. As of v0.8 the audio player's Now Playing screen scrolls in landscape, so its transport, volume and sleep-timer controls are reachable there. The Game Boy emulator lays itself out for either orientation.

-----

## Virtual Keyboard

Text entry (node name, message compose, channel name, channel secret) uses an on-screen virtual keyboard. The keyboard appears automatically when you tap a field that needs input, and dismisses on Send / Enter / Back.

### Theme

Two themes are available, switched from **Settings → KB Theme**:

- **Dark** (default) — light keys on a dark background. Easier on the eyes at night and matches the rest of the Meck UI.
- **Light** — dark keys on a light background, in case you prefer the higher contrast in bright daylight.

The choice persists via NVS and applies live to every keyboard instance the moment you change it. No reboot needed.

### Layout

Four physical layouts are available, cycled from **Settings → KB Layout**:

|Layout              |Description                                                                                                                                      |
|--------------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
|**QWERTY** (default)|Standard English layout                                                                                                                          |
|**AZERTY**          |French layout. A↔Q and W↔Z swap from QWERTY; M moves from row 3 (after L) to the right of row 2 (after L), giving row 2 ten cells and row 3 nine.|
|**QWERTZ**          |German layout. Y↔Z swap; same shape as QWERTY everywhere else.                                                                                   |
|**ЙЦУКЕН** (Cyrillic)|Standard Russian arrangement, also commonly used by Bulgarian users. The top three rows carry twelve letters each, with ё on the third row. The control keys keep their Latin labels (ABC / abc / 1#) so case and number switching still work. Ukrainian and Serbian letter sets are not included. Shown as "ЙЦУКЕН" in Settings.|

The same physical layout is used for both upper and lower case, with Shift toggling between them. The layout choice persists via NVS.

### Accents and diacritics

Long-pressing any vowel or accented base letter pops up a horizontal strip of accented variants. Tap one to insert it; tap anywhere outside the popover to dismiss. The popover is disabled in symbol/number mode (no diacritics on digits or punctuation).

Variants available, both lower and upper case:

|Base |Variants |
|-----|---------|
|a / A|á à â    |
|c / C|č ç      |
|e / E|é è ê ë ě|
|i / I|í ï      |
|o / O|ô        |
|r / R|ř        |
|s / S|š        |
|u / U|ù û ü    |
|y / Y|ý        |
|z / Z|ž        |

The set covers French and Czech in full; other languages with overlapping accents (Spanish, Italian, German umlaut variants, Portuguese, Slovak) are partially served by the same table.

### Symbol row

Tap the **`#+=`** key on the bottom row to switch the keyboard into symbol mode. Symbols include the usual punctuation plus `_`, `,`, and `:` — these were moved off the main letter rows so the `1#` shift key (which doubles as a row-mode toggle) doesn’t fight them for placement.

Tap **`abc`** to switch back to letters.

### Emoji

The two message-composer keyboards (channel and DM compose) have an **emoji key** just to the right of the space bar. Tap it to open a scrollable picker; tap an emoji to insert it and the picker closes, or tap outside the picker to dismiss without inserting.

Emoji render in colour in the picker, inline in messages, and in node and contact names, drawn from a set of Twemoji images baked into the firmware: 195 emoji plus the AU and EE flags as of v0.7.3, which added 93 from the Meck Watch's set (including the everyday red heart, which was previously dropped). Codepoints outside that set fall back to the normal text font. The set is generated from `tools/p4_emoji_registry.txt` by `tools/bake_p4_emoji.py`; sequences joined with ZWJ (for example the pirate flag) are not supported.

-----

## T-Display P4 Keyboard (K270)

The **LilyGo T-Display-P4-Keyboard (K270)** is a clamshell expansion board that
turns the P4 into a small handheld terminal: a physical QWERTY keyboard, a
battery compartment for 21700 cells, and two radios of its own. Meck-P4 has full
support for it, selected at build time.

Support is a **board type**, not a runtime option. In `idf.py menuconfig`, under
the Meck options, set **Board type** to `t_display_p4_keyboard` (the default is
`t_display_p4`). That defines `CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD`, which
gates the driver, the keyboard-backlight setting, and the keyboard-battery
column on the Battery tile. A plain `t_display_p4` build ignores an attached
keyboard entirely.

Electrically, the keyboard hangs off the **1x4 "P2" connector** as a bit-banged
I2C bus on **SDA = GPIO 46, SCL = GPIO 45**, carrying two chips: an **XL9555**
I/O expander at `0x20` and a **TCA8418** keypad scanner at `0x34`. The TCA8418's
reset line is on expander pin IO6 rather than a P4 GPIO, so Meck pulses it by
hand before starting the scanner. The keypad is polled from a 30 ms LVGL timer
rather than driven from the TCA8418 interrupt line, which keeps the driver off
GPIO 47/48 and avoids registering a second GPIO ISR service alongside the
radio's. The scan window is the full 10 x 7 matrix.

Detection is a normal probe at boot, and absence is not an error. With a
keyboard attached you get:

```
[P4KBD] keyboard detected (XL9555 0x20 + TCA8418 0x34)
MeckUI: P4 keyboard poll timer started
```

Without one:

```
[P4KBD] no XL9555 at 0x20 - keyboard not attached
MeckUI: no P4 keyboard found, using the on-screen keyboard
```

In the second case everything behaves exactly as on a plain T-Display P4 — the
on-screen keyboard is the only input. You can flash a keyboard build to a bare
board safely.

### Connecting it

**Follow this order every time. Do not hot-plug the keyboard.**

1. Switch the **P4 off** at its own power switch.
2. Switch the **keyboard off** at its power switch.
3. Insert the 21700 cell or cells into the keyboard.
4. Attach the P4 to the keyboard.
5. Switch the **keyboard on**.

Attaching or detaching with either side powered risks the board. The keyboard
carries its own power path and its own cells, and the two sides share a rail
once mated, so both switches must be off before they are joined or separated.

On the firmware side, Meck acquires **LDO channel 4 at 3.3 V** before it touches
the keyboard bus. That rail powers the I/O domain the bit-banged bus sits in;
without it the bus is marginal and reads fail intermittently, which is
indistinguishable from "no key pressed" — dropped reads show up as keystrokes
that need repeating. This is handled automatically, but it is the reason a
keyboard build powers a rail a plain build leaves alone.

### Batteries

The keyboard takes **one or two 21700 cells**. Two are wired as a single pack;
you do not have to fit both.

There is only **one fuel gauge in the system** — the P4's BQ27220 — and the
keyboard's selector switch puts exactly one battery on the rail at a time. The
gauge therefore always reads whichever cell is currently selected, and it has no
way to tell you which one that is. Meck handles this by asking you to *declare*
the switch position rather than pretending to sense it.

On the **Battery** tile you get two columns:

- the left column is the P4's internal 1000 mAh cell
- the right column, headed **KBD** in blue, is the keyboard pack

Only one column is live at a time. The declared source is drawn in white and
carries the readings; the other is grey and inert, and says either
`Pack not selected` or `Not in circuit (source: kbd)`. **Tap the blue KBD
header** to switch which side is declared live — the header stays tappable in
both states, so it is always the way back. The choice persists in NVS, and the
battery percentage in the top-right of the home screen follows it too.

**Capacity is a declaration as well.** With the keyboard pack selected, the
capacity line reads `Cap: 1 x 5000mAh (tap to change)`; **tap it** to toggle
between **5,000 mAh** (one cell) and **10,000 mAh** (two). The line is inert
while the internal cell is the declared source. This also persists.

**Read the keyboard-pack numbers as estimates, and understand what they rest
on.** The BQ27220's own state-of-charge comes from a coulomb counter referenced
against the internal cell's 1000 mAh design capacity; it is meaningless for a
21700 pack. So while the pack is selected, Meck ignores the chip's SoC and
derives everything from the measured voltage instead:

- **Chg** is a voltage-based percentage, shown as `~NN% est.` — but not the raw reading. The measured voltage sags under load and is pushed up by the charger while plugged in, so Meck first compensates for the estimated resistance of the pack circuit and then smooths the result over a few seconds. This is what stops the percentage reading chronically low with the backlight on, or leaping when the charger is plugged or pulled.
- **Vrest** is the compensated voltage the percentage is derived from. A well-tuned compensation means Vrest barely moves when you plug or unplug the charger; the raw **V** line above it will still jump, and that difference is the sag being corrected.
- **Rem** is simply *declared capacity x percentage*, shown as
  `~N/M est.` — arithmetic against the number you declared, not a measurement
- **TTE** is estimated remaining divided by present discharge current, shown
  only while actually discharging (5 mA or more) and `--` when idle or charging

The 5,000 mAh figure is a **nominal assumption for a typical 21700 cell**, not
something read from the cells. If yours are a different capacity the percentage
still tracks voltage correctly, but the mAh and time-to-empty figures scale off
whatever you declared. Voltage and current are direct gauge readings and are
unaffected by any of this.

Tapping **any** battery label anywhere in the UI toggles that reading between
percentage and voltage — that is separate from, and works alongside, the KBD
source toggle.

> **If the pack stops taking charge** (plugged in on a port that normally charges it, selector on the keyboard, but the current reads 0 mA), power-cycle the keyboard: switch the keyboard **off** at its power switch, leave it off for a minute, switch it back **on**, then press the P4 **reset** button. The charge path can latch out after an odd power event on the shared rail, and a reflash is not the fix — the keyboard power cycle is.

### Keyboard backlight

The key legends are lit by an SY7200A LED driver on the keyboard, wired to its
enable/dimming pin. Meck drives it with LEDC PWM at 20 kHz (timer 1, channel 1,
10-bit) rather than a plain high level, because full drive measured about
**+1 A** of extra battery draw.

**The LilyGo key toggles the backlight on and off.** It is the key marked with
the LilyGo logo (the Windows-key position); pressing it produces no character,
it only switches the light. The backlight always comes up **off** after a boot.

**Brightness is set in Settings -> Keyboard Backlight**, a slider from **5% to
100%**, defaulting to **25%**. Dragging applies the new level live — visible
only while the backlight is actually on — and the value is saved when you
release the slider. 25% measured about **398 mA** of pack draw with the
backlight lit, which is why it is the default rather than something brighter.
The setting only appears on keyboard builds.

### Typing

Keystrokes go to **whatever text field is focused on the current screen**. Meck
walks the active screen for a focused textarea rather than consulting a fixed
list, so every text field in the firmware is covered — settings, channel and
region editors, WiFi, trace and path editors, repeater admin, the web URL and
search bars, and the Notes editor.

Beyond plain characters:

|Key|Action|
|---|---|
|**Enter**|Commits the field — the same action as the on-screen keyboard's OK button, so each screen's existing save/send handler runs|
|**Esc**|Cancels the field, as the on-screen keyboard's close button does. With no field focused, Esc is the universal Back key — see [Navigating without touching the screen](#navigating-without-touching-the-screen)|
|**Backspace**|Deletes the character before the cursor|
|**Left / Right**|Move the cursor within the field. With no field focused, they navigate — see the navigation section below|
|**Mic / Record key**|Toggles the canned-messages overlay on the channel and room message screens — see [Canned Messages](#canned-messages)|
|**Shift**|One-shot upper case for the next letter|
|**Caps**|Caps Lock toggle. Shift and Caps together cancel, as on a normal keyboard|
|**Fn**|One-shot symbol layer for the next key|
|**LilyGo key**|Toggles the keyboard backlight (see above)|
|**Alt, Ctrl, F1-F11**|Produce nothing in Meck|

**Type-to-compose.** On the channel and room message screens, pressing any
printable key when nothing is focused opens the composer and starts typing into
it, so you do not have to tap the field first. Arrows, Enter and Esc are left
alone, so you can still move around the message list without the composer
springing open.

**The symbol layer.** Fn plus a key gives the symbol printed on it. A few
positions in LilyGo's vendor key map carry no printable symbol — Fn with Z, X, C
or V among them — and those combinations produce nothing. One deliberate Meck
change: **Fn+H types a comma**, because the vendor map carried no comma anywhere
and duplicated the apostrophe instead.

### Navigating without touching the screen

With no text field focused, the keyboard drives the whole UI:

**Esc is Back, everywhere.** On any screen, Esc presses that screen's Back
(or an open overlay's Cancel or Close) button, running exactly the handler
a tap would. On the home screen it does nothing (there is nowhere back to
go).

**Up/Down scroll or select.** On most screens they page the main list up
and down, the same movement a vertical swipe produces. On the Settings
screen, the channel picker, the Canned Messages sub-screen, and the
Repeater Admin Settings and command lists they instead move a thin white
selection ring row by row, scrolling just enough to keep the ringed row in
view, and **Enter** activates the ringed row -- tapping it without touching
the screen. On the channel and DM message screens the ring moves bubble by
bubble, with the compose box as the last stop (a first **Up** starts on
the newest message, a first **Down** on the compose box): **Enter** on a
bubble opens its [Path View](#path-view) -- or the Retry dialog for a
failed send -- exactly as a touch long-press does, and **Enter** on the
compose box starts typing. While a Path View, Retry dialog or Repeater
Admin prompt is open, Up/Down ring its buttons, Enter presses the ringed
one and Esc closes it. The ring only appears once you use the keys, and
disappears when you leave the screen or touch it.

**Left/Right** cycle the contact filter on the Contacts screen, and page
the home screens (wrapping at the ends, like swipes).

**On the home screen:**

|Key|Action|
|---|---|
|**Left / Right**|Page through the home screens, wrapping at both ends|
|**Up / Down** (Home page)|Cycle a selection ring around the navigation tiles|
|**Up / Down** (Timezones page)|Cycle the ring through the three zone rows|
|**Enter**|Open the ringed tile or zone row. On the Advert page, send an advert; on the GPS page, toggle GPS power (the same actions as the touch long-press)|
|**M / C / S / E / N / F / G / R / P / B**|Open Messages, Contacts, Settings, Reader, Notes, Discover (F), Maps (G), Trace (R), Audio (P), or Web (B) directly — the same key map as the Meck T-Deck builds|

The letter shortcuts are lowercase; a shifted letter is treated as a
deliberate uppercase character and does nothing on the home screen.

### The onboard radios

The keyboard carries two radios of its own: a **CC1101** and an **nRF24L01+**.
**Neither is available in Meck.** There is no driver for either anywhere in the
firmware — the only trace of them in the tree is a block of pin definitions in
LilyGo's vendor header, and they are reachable only through LilyGo's own
standalone example builds (`radiolib_cc1101_send_receive` and
`radiolib_nrf24l01_send_receive`), which are separate applications rather than
part of Meck.

Meck's mesh traffic goes over the P4's own **SX1262** on the HPD16A module, and
that is the only radio it uses.

-----

## CardKB

Meck-P4 has optional support for the **M5Stack CardKB**, a small I2C QWERTY keyboard, as an alternative to the on-screen keyboard for typing messages. Support is gated behind the `MECK_CARDKB` build flag, so it is a build-from-source option rather than a touch-only default.

The CardKB connects to the board's **P1 connector** (the 1x4 header), driven as a software-I2C bus on SDA = GPIO 48 and SCL = GPIO 47, because both hardware I2C controllers are already used by other peripherals. The bus runs at 10 kHz and the keyboard is expected at I2C address 0x5F. It is probed for at boot; if one answers, a poll timer starts and `MeckUI: CardKB poll timer started` is logged. If none is found, the device runs normally on the on-screen keyboard.

When a CardKB is present and a message composer (channel or DM/room) is open:

- Typing writes straight into the message field. The first keypress hides the on-screen keyboard so the message list reclaims its space, while the field keeps focus so you can keep typing.
- **Enter** sends the message (the same action as the on-screen keyboard's OK button) and **Esc** cancels.
- **Backspace** deletes the character before the cursor, and **Left / Right** move the cursor within the field.

CardKB input is currently wired to the message composers only. Whole-UI navigation with the arrow keys is not implemented for the CardKB — the K270 keyboard build has it (see [Navigating without touching the screen](#navigating-without-touching-the-screen)).

-----

## Channel Messages

Tap **Messages** from the home grid to open the channel messages screen.

|Gesture                     |Action                                        |
|----------------------------|----------------------------------------------|
|Swipe up / down             |Scroll messages                               |
|Swipe left / right          |Open channel picker                           |
|Tap compose area            |Open virtual keyboard to compose a new message|
|Tap **Send** on the keyboard|Send the current message                      |
|Tap **Back**                |Return to home                                |

Channel message history is persisted to per-channel files on the SD card,
so messages survive reboots when an SD card is present.

The view opens on the newest 100 messages and pages back through the whole stored history (up to 500 per channel): an **Earlier messages (N more)** row at the top adds the next 100, up to 300 on screen at once — beyond that the window slides and a **Newer messages (N more)** row at the bottom pages forward again. The message you were reading stays put while you page, a new arrival while you are paged back no longer jumps the view to the bottom, and both rows are keyboard stops on the K270 build.

**Per-message metadata:**

- **Incoming messages** display a small hop-count badge showing how many repeaters the packet passed through to reach you. Direct receptions show 0 hops.
- The footer under each bubble — `21:57 · 4 hops` — continues with the sender's **path-hash mode** and, where recognised, the **region scope** the message was sent under: `21:57 · 4 hops · 2-byte · au-nsw`. Byte mode comes from the packet itself, so it shows for stored history too; route-direct messages and your own sends show this device's mode. A scoped message whose region is not recognised shows *(reg unknown)*; an unscoped message shows nothing. Region is worked out when a message arrives and kept for the session only — history reloaded from SD after a reboot shows byte mode but no region. See [Region Scope](#region-scope).
- **Outgoing messages** show a send status that updates as repeater echoes arrive:
  - **Sending…** – the message has been transmitted but no repeater echo has been heard yet.
  - **✓ Heard N Repeats** – one or more repeaters relayed the message back. The count shows how many echoes were received, confirming the message propagated through the mesh.
  - **✕ Failed** – 18 seconds elapsed with no repeater echo. The message was transmitted but no repeater confirmed receipt. This typically means no repeater is in range, or the channel’s radio parameters don’t match the repeater’s.

**Long-press outgoing messages:** long-press any outgoing message to see which repeaters acknowledged it. If the message failed, a **Retry Send** option re-queues it with a fresh timestamp. The recipient may see a duplicate if the original arrives late via a slow path. Long-pressing an incoming message opens its [Path View](#path-view). With the K270 keyboard, ring a bubble with Up/Down and press Enter for the same result.

-----

## Canned Messages

Canned messages are up to five pre-written messages you can fire off with
a couple of presses instead of typing -- ported from Meck Watch. They work
in both channel views and room server views.

### Setting them up

Go to **Settings > Canned Messages**. You get five slots, each showing a
preview of its message or **(empty)**. Tap a slot to edit it: the editor
opens pre-filled with the current text (up to 133 characters, the normal
message cap). **Confirm** saves; saving with the text cleared empties the
slot; **Cancel** (or Esc on the keyboard) discards the edit. The slots
persist across reboots and firmware updates.

### Sending one

*Requires the K270 keyboard build -- the trigger is a keyboard key.*

From a channel or room server message view, press the **microphone key**
(the record key on the K270). An overlay lists your non-empty slots.
**Tap a message and it sends immediately** -- no confirmation step, straight
through the same send path as the Send button, including the delivery
status and repeater-echo tracking. The overlay then closes.

If every slot is empty, the mic key shows a brief **"No canned messages"**
toast instead -- go and fill a slot in Settings.

To close the overlay without sending: press the mic key again, press
**Esc**, or tap the back chevron.

**Keyboard-only operation:** while the overlay is open, Up/Down move the
selection ring through the messages and Enter sends the ringed one, so a
canned reply is mic key, arrow, Enter -- three presses, no touch.

The mic key does nothing outside the channel and room message screens, and
nothing in a DM view.

-----

## Channel Picker

Swiping left or right on the channel messages screen opens the channel
picker. All your channels and the DM inbox are shown in a single view with
unread message badges.

|Gesture      |Action                |
|-------------|----------------------|
|Tap a channel|Switch to that channel|
|Tap **Back** |Return to messages    |

The Public and #test channels are configured by default. New channels
can be added via the channel picker’s Add Channel button (with a
Confirm button and virtual keyboard) or through Settings → Channels.

A channel's stored history can also be cleared from here: **long-press a channel row** — or ring it and press **X** on the keyboard — for a **Delete message history?** confirmation showing how many stored messages will be removed from this device (the channel itself is kept). Delete clears that channel's messages from memory, its unread count, and its SD history file; Cancel does nothing.

-----

## Contacts

Tap **Contacts** from the home grid to open the contacts list. All known
mesh contacts are shown sorted by most recently heard, with their type
prefix (colour-coded: C / R / RS / S) and a 4-byte public-key prefix to
disambiguate near-key-collisions.

The list is windowed so it opens quickly however many contacts you hold: the first **60** of the sorted, filtered set are built, ending in a **Show more (N remaining)** row that adds the next 60. Changing filter — a swipe, a chip tap, or Left/Right on the keyboard — returns the list to its first 60, while an extended window is kept when you leave Contacts and come back. A grey count at the right end of the filter-chip row shows how full the store is — `stored/2000` on **All** (the P4 holds up to 2,000 contacts), `matched/stored` on any other filter — refreshed on open, on a filter change, and on Show more.

**Filter chip bar** at the top of the list:

- **All** — every contact
- **Chat** — chat nodes only
- **Repeater** — repeaters only
- **Room** — room servers only
- **Sensor** — sensor nodes only
- **Fav** — only contacts you’ve marked as favourite

|Gesture                 |Action                                                     |
|------------------------|-----------------------------------------------------------|
|Swipe up / down         |Scroll through contacts                                    |
|Swipe left / right      |Cycle filter (or tap a filter chip directly)               |
|Tap a contact           |Open contact detail screen                                 |
|**Long press a contact**|Toggle favourite (a star appears, contact rises to the top)|
|Tap **Back**            |Return to home                                             |

The contact detail screen shows public key prefix, type, flags, and last
advert time. It includes a **red Hold button** in the top-right that
deletes the contact when long-pressed (single tap is intentionally unbound
to prevent accidental loss).

**Auto-add policies** can be configured in **Settings → Contacts**:

- **Auto All** — every advert heard adds a contact. Selecting it also
  switches all four per-type toggles on, so it always adds everything
  even if the toggles were previously off (this also applies when a
  companion app switches the device into auto mode)
- **Custom** — per-type toggles (chat, repeater, room, sensor) decide which
  advert types to auto-add
- **Manual Only** — disables all auto-add

An **Overwrite oldest when full** toggle decides what happens when the
contacts table reaches its 2,000-entry limit.

To delete **every** contact at once — favourites and custom paths included, along with the direct-message history, while channel messages are kept — use **Settings > Experimental Features > Delete all contacts**. A confirmation card states exactly what goes, and the device restarts when the purge finishes.

To add a contact that hasn’t broadcast an advert recently (so it’s not in your auto-add list), use the **Discover** screen below to send an active discovery probe and add the node from the response. This is the easiest way to pick up a nearby repeater you’ve just brought online or one whose advert your device missed.

The contact detail screen branches by contact type:

- **Chat contacts** get a cyan **Send DM** button (see [Direct Messages](#direct-messages)) and a teal **Edit Path** button (see [Per-Contact Path Editor](#per-contact-path-editor)).
- **Repeater contacts** get a cyan **Admin** button (see [Repeater Admin](#repeater-admin)) and the **Edit Path** button.
- **Room server contacts** get the same **Admin** button (see [Room Servers](#room-servers)) and the **Edit Path** button.

-----

## Direct Messages

Tap a chat contact and press **Send DM** on its detail screen to open the DM conversation. The view follows the standard chat layout — keyboard along the bottom half of the screen, message bubbles scrolling above it.

Per-contact DM history is held in a 20-message ring buffer in PSRAM (lazy-allocated, so contacts you’ve never DM’d cost nothing) and persisted to `/sdcard/meshcore/dms/`, so messages survive reboots when an SD card is present.

**ACK tracking:** outgoing bubbles update from “Sending…” through to “Delivered” or “Failed” as the ACK round-trip completes. A direct path produces a near-instant delivered state; a flooded send waits the path-length-derived timeout before either confirming or marking failed.

**Reading received DMs:** the **Channel Picker** (swipe left/right on Messages) shows a **DM Inbox** row alongside your channels, with a per-contact unread badge. Tap a contact’s row to jump into that conversation.

-----

## Repeater Admin

Tap a Repeater contact to open its contact detail screen. Tap the cyan **Admin** button to open the login screen.

The login screen shows the contact name in the title bar, a password field with reveal-while-typing (the last character shows for 1.5s before being masked — easier than blind-typing symbols and numbers), a full Show/Hide toggle, a Remember Password checkbox, and a routing-mode badge showing **Flood** vs **Direct** (picked from the contact’s known path at entry).

On successful login, the admin home shows a persistent banner across the top — **green for admin**, **yellow for guest** — with the contact name and session role. Below the banner, a scrollable menu:

|Menu item      |What it does                                                                                                                                                                                              |
|---------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|**Status**     |Full RepeaterStats view: battery, clock-at-login, uptime, TX/RX airtime, last RSSI/SNR, noise floor, packet counts, duplicates, errors, queue length, debug flags. A Refresh button re-issues the request.|
|**Send Advert**|Single big button that triggers the repeater to broadcast an advertisement. Status line goes yellow during in-flight, green on success with the repeater’s response text, red on send failure.            |
|**Cmd Line**   |Free-form 100-character text input + Send button. Virtual keyboard slides up on focus. Scrollback shows command in cyan and response in white (or red on failure). Trimmed at 50 entries.                 |
|**Settings**   |Twelve command categories ported from the Meck Watch's repeater admin: Clock & Adverts, Neighbours, Get Config, Set Config, Bridge, GPS, Sensors, Regions, Powersaving & Power, Logs & Stats, Reboot & Power Off, Firmware & Device Info. Guest sessions see Neighbours and Firmware & Device Info.|

**Settings categories.** Each category opens a list of commands covering the MeshCore v1.17.1 repeater CLI, including the newer `cad`, `radio.fem.rxgain` / `radio.fem.txgain`, `extra.sf` and `pwrmgt.bootreason` keys. The cue at the right of each row says what a tap does: **send** sends the command straight away; **...** prompts for a parameter (the on-screen keyboard opens, or type on the K270; Enter sends, Esc cancels); **on/off** offers a two-button picker; **confirm** asks first, which guards anything destructive -- reboot, `start ota`, power off, `log erase`, `clear stats`, the admin password and the private key. Neighbours and Firmware Info keep their dedicated screens.

The repeater's reply is shown verbatim on a response panel: a green **Reply:** for a normal answer, or a red **Repeater returned an error:** when the text reads as one -- which is what a repeater without GPS, sensors, logging or power management, or one that does not know a newer key, sends back. If nothing arrives within the mesh's own round-trip estimate (plus five seconds) the panel reports **Timed out. No reply from repeater.**; the commands that never reply (reboot, `clkreboot`, `start ota`, power off) report **Command sent. No reply expected.** instead. Commands that only work on the repeater's own serial console (`log` status, `stats-*`, `erase`) are deliberately not listed; the free-form Cmd Line remains for anything else.

**Single-session policy:** logging into a new repeater tears down the prior session. **Remember Password** persists the entered password for that contact, so subsequent visits skip the prompt.

-----

## Room Servers

Tap a Room-type contact in the contacts list (filter to **Room** to find them quickly) to open the same admin login flow as repeaters. After login, the room’s post timeline appears as a scrollable bubble list:

- Bubbles are left-aligned, with the original **author name** above each bubble and a **timestamp and hop count footer** below
- Author resolution walks your contacts looking for a matching 4-byte pubkey prefix; unknown authors render as `Unknown <hex>`
- A composer at the bottom of the screen lets you post back to the room

**Persistence:** post history is saved per-room to `/sdcard/meshcore/posts/` and reloaded into PSRAM on boot, so you don’t need to re-login to see what’s already been received.

**Live re-render:** posts arriving while you’re sitting on the room view land in the bubble list without leaving the screen — no need to refresh.

-----

## Region Scope

Regions limit how far your flood messages propagate through the mesh. When you set a region, outgoing messages are tagged with a transport code that repeaters use to decide whether to forward them. Messages sent without a region reach all repeaters via the default wildcard, same as always.

Meck-P4 does not pre-set any region on a fresh flash. Region names are determined by your local mesh community. Common patterns follow ISO 3166 country/subdivision codes (e.g. `au` for Australia, `au-nsw` for New South Wales, `gb-eng` for England, `us-ca` for California), but communities may also use custom names for their area. Region names must be lowercase alphanumeric characters and hyphens only, max 30 characters. Check with your local group for the names in use; there is currently no discover-regions feature.

**Device-wide default region:** Set in Settings > Default Region. This applies to all channels and DMs unless a channel has its own override.

**Per-channel region:** In Settings > Channels, tap a channel, then tap Region Scope to edit. This overrides the device default for that specific channel only. Empty = use the device default.

**What shows on messages:** as of v0.7.3.5 the scope travels the other way too — each incoming channel message's footer names the region it was sent under, when the device can recognise it. A scope can only be recognised, never read back: a scoped packet carries a one-way code, so the device tries every scope it knows against it — your device default (**Settings > Default Region**), each channel's own scope (**Settings > Channels**, tap the channel, then **Region Scope**), and the shared 28-name list all the Meck firmwares carry (`au`, the Australian states and territories, and the NSW / VIC / ACT / TAS / QLD / WA sub-regions). A scoped message from outside that set shows *(reg unknown)*; an unscoped message shows nothing; your own sends show the scope they went out under. Recognition happens on arrival and lasts the session only — history reloaded from SD shows no region — and [Path View](#path-view) gains a **Region:** line above the route.

**Repeater region management:** Repeaters running MeshCore v1.10+ support region management via CLI commands. From the Repeater Admin screen, log in and use the Cmd Line to send region commands directly. All remote-capable region commands work through this interface:

|Command                                     |Description                             |
|--------------------------------------------|----------------------------------------|
|`region put <name> [parent]`                |Create a new region                     |
|`region remove <name>`                      |Remove a region                         |
|`region allowf <name>`                      |Allow flooding for a region             |
|`region denyf <name>`                       |Block flooding for a region             |
|`region get <name>`                         |Show info for a region                  |
|`region home` / `region home <name>`        |View or set the repeater’s home region  |
|`region default` / `region default <name>`  |View or set the repeater’s default scope|
|`region save`                               |Persist region changes to flash         |
|`region list allowed` / `region list denied`|View regions (firmware 1.12+)           |
|`region load <name> [F]`                    |Single-line region load                 |

The interactive multi-line `region load` (without parameters) is not supported because serial CLI commands are not currently available on the P4. Use individual `region put` and `region allowf` commands instead. See the [MeshCore CLI documentation](https://docs.meshcore.io/cli_commands/#region-management-v110) for full details and examples.

-----

## Notification Sounds

Meck-P4 supports per-channel notification tones. When a new message arrives on a channel that has a tone assigned and notifications are not muted, the tone plays through the ES8311 audio codec at a fixed 80% volume. The previous volume level is restored automatically when the tone finishes. Tones are skipped if the audio player is already playing music or an audiobook.

### Bundled tones

Seven notification tones are embedded in the firmware and copied to `/sdcard/audio/tones/` on first boot:

- Bell-01, Ding-01, High-Trill-01, Low-Ding-02, Low-Ding-03, Mid-Trill-01, Soft-Notif

Users can add their own MP3 files (44.1 kHz) to the same folder. The tone picker scans the folder each time it opens.

### Setting a tone

Open Settings > Channels, tap a channel, then tap Notification Tone. A scrollable picker lists all available tones plus “None (silent)” at the top. Tapping a tone selects it and plays a preview. The currently assigned tone is highlighted with a cyan border. Tone assignments persist across reboots (stored in `/sdcard/meshcore/notif_sounds.cfg`).

### Notification preferences

Each channel also has a notification preference, editable from the same channel detail screen:

|Preference       |Behaviour                                                                               |
|-----------------|----------------------------------------------------------------------------------------|
|**All** (default)|Notification tone plays on every new message                                            |
|**Mentions**     |Tone plays on every new message (mention-only filtering is planned for a future release)|
|**None**         |Channel is muted; no tone plays and no unread badge increments                          |

### Channels settings sub-screen

The Channels sub-screen (Settings > Channels) provides a centralised view of all your channels. Each row shows the channel name, region scope tag, and notification preference. Tap any channel to open its detail screen where you can edit the region scope, notification preference, notification tone, or delete the channel. An “Add Channel” button at the bottom lets you create new hashtag channels without leaving the settings flow. Channel adding and deleting also remain available in the channel picker for convenience.

-----

## Per-Contact Path Editor

A teal **Edit Path** button on the contact detail screen opens an editor for the contact’s outgoing route.

|Control                |Effect                                                                               |
|-----------------------|-------------------------------------------------------------------------------------|
|**Path Size** dropdown |1-byte or 2-byte path hash. Match the path hash mode the network is using.           |
|**Hex hops** text field|Comma-separated hex bytes — one byte per hop for 1-byte mode, two per hop for 2-byte.|
|**Save**               |Stores the entered path for this contact. Empty path = saved as a 0-hop direct path. |
|**Reset to Flood**     |Clears the stored path so the next send floods.                                      |

Useful when you’ve manually picked a working route via Trace Path and want to lock it in, or when a contact’s auto-acquired path has gone stale and you want to revert to flooding while a new path settles.

-----

## Trace Route

A standalone Trace Path screen — tap the **Trace** tile on the home grid — lets you probe a specific route hop by hop. Enter the path as comma-separated hex bytes (the same format as the Per-Contact Path Editor), pick the Path Size (1-byte or 2-byte), and tap **Run Trace**.

The result list below shows the per-hop SNR as 3-bar icons as TRACE replies come in, with a 30-second pending timeout.

Useful for diagnosing where in a chain a route is breaking down — if you get replies for the first two hops but the third never comes back, that’s where the link is failing.

-----

## Path View

Long-pressing an incoming channel message opens **Path View**, which displays the routing path that message took through the mesh: the hop count and hash size, then each hop's hash and, where that repeater is in your contacts, its name. Routes of up to **16 hops** are shown in full at any path-hash size (1, 2 or 3 bytes; a 3-byte hash is printed in full). The panel grows to fit the route -- in portrait all 16 hops fit without scrolling -- and scrolls beyond that. **Copy Path** pastes the text into the compose box and **Reply** starts a reply to the sender. Long-pressing one of your own messages shows the **Heard by** list instead: the repeaters that echoed it (up to eight), or the Retry dialog if the send failed.

With the K270 keyboard, ring a bubble with **Up/Down** and press **Enter** to open the same view; Up/Down then ring its buttons and **Esc** closes it (see [Navigating without touching the screen](#navigating-without-touching-the-screen)).

-----

## Position Adverts and Share Position

Meck-P4 can include your GPS position in outgoing adverts and share your current position with contacts.

### Position settings

Open **Settings > Position** to configure position behaviour:

|Setting                            |Description                                                                                                |
|-----------------------------------|-----------------------------------------------------------------------------------------------------------|
|**Latitude** (tap to edit)         |Manual latitude entry in decimal degrees                                                                   |
|**Longitude** (tap to edit)        |Manual longitude entry in decimal degrees                                                                  |
|**Share Position** (tap to cycle)  |Controls how position is shared: Off / Manual / Auto-GPS. In Auto-GPS mode, position is refreshed from the L76K GPS snapshot every 15 minutes and saved to prefs automatically.|
|**Copy Position**                  |Copies the current lat/lon to the clipboard                                                                |

When a position is set (either manually or via GPS), it is encoded into your outgoing adverts so other nodes on the mesh can see your location on their Maps screen.

### Share Position button

The **+** button on the channel messages screen includes a **Share Position** option that sends your current position as a message to the active channel or DM conversation.

-----

## Private Channels

Channels in Meck-P4 can be public or private:

- **Public channels** are prefixed with `#` (e.g. `#test`, `#sydney`). The channel secret is derived via SHA-256 from the name, matching the standard MeshCore convention. Anyone who knows the name can join.
- **Private channels** have no `#` prefix. A random 16-byte secret is generated when the channel is created, so only users who have been invited can participate.

### Sharing private channels

Open **Settings > Channels**, tap the channel you want to share, then tap **Share Channel**. This opens a contact picker and sends the channel name and secret as a DM to the selected contact. The recipient sees a pending invite notification that they can accept (tap) or dismiss (long-press).

### Creating channels

When adding a channel (via the channel picker or Settings > Channels), type a name starting with `#` for a public channel or without `#` for a private channel. The secret is handled automatically.

-----

## Discover

Tap **Discover** from the home grid to open the active-discovery screen. Unlike passive advert reception (which depends on a node spontaneously broadcasting and can take up to 12 hours per node), Discover sends a zero-hop **DISCOVER_REQ** control packet on the air and any repeater or room server within radio range responds within a second or two with its public key, type, and the SNR it measured when receiving your probe.

This is the same mechanism the MeshCore mobile app uses for its “scan” feature, and it’s how you find a node that hasn’t adverted recently or that your device wasn’t listening for at advert time.

### What you see

The screen has a status line, a Rescan button, and a scrollable list of result rows. The status line reads `Scanning... N found` while the 30-second window is open and `Scan done: N found` after it closes.

Each result row shows the node type (R for repeater, S for room server), the name, the 4-byte pub-key prefix, and a signal indicator:

- **Live entries** (heard during this scan) display SNR in dB, colour-graded green (strong) / yellow (marginal) / red (weak). These are direct neighbours one radio hop away.
- **Cached entries** (pre-seeded from your recent-heard ring at scan start) display a hop count in grey. These come from earlier advert reception and may or may not be reachable right now.

A `[+]` marker next to a row means the node is already in your contacts.

### Adding a contact from Discover

Tap any row to add that node to your contacts list. Three cases:

1. **Already in contacts.** Nothing changes (the `[+]` marker confirms this).
1. **Known to your recent-heard ring but not yet a contact** (e.g. auto-add was off when the advert came in, or you’d previously deleted it). Meck imports the cached advert blob and the contact appears with full name, location, and feature flags.
1. **Heard via DISCOVER_RESP only**, with no cached advert blob on hand. The contact is added with a placeholder name like `Rptr 19855E54` (the pub-key prefix) and the type from the response. When that node next sends a flood advert (or you send a manual advert to prompt one), the full name and any location fields populate automatically.

Case 3 is the main reason Discover is useful for picking up *new* repeaters: even without ever having heard their advert, you can request their identity and have them in your contacts within seconds.

### Scan behaviour

The scan window is 30 seconds. During that time:

- A single DISCOVER_REQ goes out at the start (zero-hop, ROUTE_TYPE_DIRECT, ~12 bytes on the wire).
- The type filter is set to **repeaters + rooms** (chat clients and sensors are not asked to respond).
- The list pre-seeds with up to 32 repeaters/rooms from your recent-heard ring so the screen has content immediately while live responses arrive.
- Any flood advert that lands during the window is also captured as a secondary signal, providing fallback for older repeater firmware that doesn’t yet implement DISCOVER_RESP.
- A random tag is generated per scan; late responses to previous scans are ignored.

Tap **Rescan** to start a fresh scan. Tap **Back** to return to the home screen.

-----

## Audio Player

Tap **Audio** from the home grid to open the audio player. Plays WAV and MP3 files from the SD card under `/sdcard/audio/`. Two top-level subtrees give the player different defaults:

|Subtree                    |Defaults                                                                                         |
|---------------------------|-------------------------------------------------------------------------------------------------|
|`/sdcard/audio/music/`     |Standard music playback. Always starts from the beginning (no resume bookmark).                 |
|`/sdcard/audio/audiobooks/`|Audiobook mode. Resume bookmark on, sleep timer available, position tracked through the playlist.|

Inside each, organise however you like (typically Artist / Album / track, or Author / Book / chapter). The audio browser shows breadcrumbs and lets you tap a track to play, with transport controls (-30s, play/pause, +30s), volume, and a progress bar on the Now Playing screen.

Playback continues when you leave the player. While a track is playing, a **`>>` indicator** appears in the header, including in the Audio and Reader file browsers; tap it to jump straight back to the Now Playing screen for the current track without restarting it. Audiobooks resume from where you left off across reboots; music always starts from the beginning.

**Cover art** displays when a **256x256 `cover.png`** is placed alongside your tracks. Other filenames (`folder.png`, `front.png`, `album.png`) are also recognised, case-insensitive. Larger PNGs are read by the file scanner but fail to allocate at decode time — see the audio player guide for the working recipe and a downscale command if you have higher-resolution covers on hand.

**Audio format requirements:** MP3 files must be at 44.1 kHz; WAV files must be 16-bit PCM (format code 0x0001) at 44.1 kHz, mono or stereo. Files outside this window fail the format check. See the audio player guide for `ffprobe` checks and `ffmpeg` conversion commands.

**Watchdog crash on first play:** if a file with large embedded album art crashes the device during playback startup, you’ve hit the libhelix-mp3 sync-word scan issue documented in the audio player guide. The firmware has a defensive ID3v2-skip patch that should prevent this, but the durable fix is to clean your files at the source with the `tools/mp3_clean.py` script before copying them to the SD card.

For full setup instructions including the `mp3_clean.py` script usage, SD card layout, troubleshooting, and developer notes, see:

**[Audio player guide](https://github.com/pelgraine/Meck-P4/blob/main/information/Meck%20Docs/audioplayerguide.md)**

-----

## Reader

Tap **Reader** from the home grid to open the text reader. It lists files from the SD card under `/sdcard/books`, with the same folder navigation, breadcrumb, and up-one-level button as the audio browser, so you can organise books into subfolders however you like.

The reader opens **plain-text (`.txt`)** and **EPUB (`.epub`)** files. The file list shows folders, `.txt`, and `.epub` items. Tapping an EPUB converts it to plain text the first time it is opened: a *Converting … to txt* screen is shown while the book is decoded, the result is cached as a `.txt` in a hidden `.epub_cache` subfolder, and the reader opens that. Re-opening the same EPUB later loads straight from the cached text, so the conversion only runs once. You only ever see and tap the `.epub` itself; the cached text stays hidden from the browser.

Tap a file to start reading. In the reading view, tap the **left third** of the screen to turn back a page and the **right two-thirds** to turn forward. Back returns to the file list; from the `/sdcard/books` root, Back returns to the home screen.

**Progress** shows as a percentage at the bottom of the reading view. It is based on your byte position through the file (the start of the current page divided by the file size), so it tracks how far through the whole file you are rather than counting pages.

**Resume bookmark.** A **green play icon** beside a file in the list means that file has a saved reading position. Re-opening it resumes at the page you were on, so you can leave a book part-read and pick it up later. Your position is saved as you turn pages and when you leave the book.

**Font size.** The reader honours the font-size preference in **Settings**: the same setting that scales the rest of the UI also sizes the body text of your `.txt` files.

-----

## Notes

Tap **Notes** from the home grid to open the notes app. Notes are plain UTF-8
files stored under `/sdcard/notes`, so they live on the removable card, survive
a firmware reflash, and can be read or edited on a computer. The notes folder is
created automatically on first run.

Both **`.md`** and **`.txt`** files are listed, and **new notes are created as
`.md`**. Nothing about the storage format changed with markdown support: a `.md`
note is ordinary markdown text, so it opens correctly in any editor or viewer
off-device.

The browser works like the reader's: folder navigation with an up-one-level
button and a breadcrumb, rooted at `/sdcard/notes`. A green **+ New Note** row
sits at the top of every folder, so you can start a note even in an empty
folder. New notes are created in the folder you are currently viewing, named by
date and time when the clock is synced (`note_YYYYMMDD_HHMM.md`, falling back to
seconds precision on a same-minute collision) or sequentially (`note_NNN.md`)
when it is not.

**Reading.** The two extensions open differently. A **`.md`** note is loaded
whole and rendered as markdown into a single scrolling view — swipe to scroll,
with no page turns, tap zones or progress percentage. A **`.txt`** note keeps
the original paged reader: tap the left third of the screen to turn back a page
and the right two-thirds to turn forward, with a progress percentage and a
saved-position bookmark so you can resume where you left off. Either way an
**Edit** button in the top-right reopens the note in the editor.

**What the markdown renderer supports.** It is a deliberate subset, chosen so
notes stay readable as plain text:

- `#`, `##` and `###` headings at the start of a line
- `- ` bullets, rendered as a bullet dot
- leading spaces, preserved as indentation
- inline `**bold**` and `*italic*` runs

Anything else renders literally. Where a run is marked both bold and italic,
bold wins — there is no bold-italic face on the device. Heading and body sizes
follow your **Settings -> Font Size** preference (Classic / Larger / Extra
Large), picked fresh each time the view is drawn.

**The editor and its toolbar.** Tapping **+ New Note**, or **Edit** on an
existing one, opens the editor with the on-screen keyboard (which honours your
KB Theme and KB Layout settings), or accepts the physical keyboard if one is
attached. A seven-button formatting toolbar sits on its own row under the
header:

|Button|Action|
|---|---|
|**B**|Inserts `**` at the cursor|
|**I**|Inserts `*` at the cursor|
|**H**|Cycles the current line's heading: none -> `#` -> `##` -> `###` -> none|
|**Bullet**|Toggles `- ` on the current line|
|**Right arrow**|Indents the current line by two spaces|
|**Left arrow**|Outdents the current line by two spaces|
|**Eye**|Toggles a live preview of the note rendered as markdown|

**B** and **I** insert a single marker per tap, so the flow is: tap the button
to open, type the word, tap the button again to close. They do not insert a pair
or move the cursor for you.

Focus stays in the text field across a toolbar tap, so you can format and keep
typing without tapping back into the note. The insertion point is a **blinking
white I-beam**. The one exception is the preview eye, which deliberately drops
focus while the preview is up so a physical keyboard cannot edit text you cannot
see; focus returns when you toggle the preview off. Tap **Save** to write your
changes back to the card.

Notes are bounded by a 16 KB editor buffer, which is what makes loading a `.md`
note whole for rendering safe.

**Rename and delete.** Long-press a note in the browser to open an action menu
with **Rename**, **Delete**, and **Cancel**. Delete asks for confirmation first.
Rename edits the name only — the original extension is preserved and re-added on
save, so renaming a `.md` note cannot accidentally turn it into a `.txt`. Both
operations move or remove the note's resume bookmark alongside the file.

-----

## Web Reader

Tap **Web** from the home grid to open the web reader, a lightweight reader-mode browser that runs over the onboard ESP32-C6. It is aimed at light, text-heavy pages, not the full modern web.

> **Work in progress.** The web reader ships in v0.6 with known limitations (listed below). An amber notice appears for a few seconds each time you open the Web tile as a reminder.

### How it works

The web reader drives the ESP32-C6 directly over ESP-AT, building its HTTP and TLS requests by hand on top of the C6's AT command set. This is separate from the WiFi companion transport: it shares the same C6 WiFi link but uses its own request path. The C6 must be connected to WiFi (see [WiFi Companion](#wifi-companion)) for the web reader to fetch anything.

### Using it

The landing menu has four entries: **Enter URL**, a **DuckDuckGo Lite** search, **Bookmarks**, and **History**.

- **Reader view.** Fetched pages are parsed to readable text and shown in a scrolling view with a status line. Headings render in dark red. Link markers show as a muted grey `[N]` and are followed from the **Links** panel rather than by tapping the number.
- **Links.** The **Links** button opens a panel listing the page's links so you can follow one.
- **Forms.** When a page has forms, an orange **Forms (N)** button appears. It opens a picker (duplicate forms are collapsed by action and field set), and selecting one opens a fill modal with a labelled field per text or password input. Submitting builds the `action?name=value&…` query and loads it through the normal page path, so history and status stay correct. GET forms are supported; POST is not yet.
- **Bookmarks** can be added from the reader view with a confirmation toast, and revisited from the landing menu along with your browsing **History**.

HTTPS is supported and confirmed working (verified against DuckDuckGo Lite). Common search field names such as `q` or `search` are shown as "Search" in both the page text and the fill modal.

### Known limitations in v0.6

- **Throughput and size.** The AT link runs at roughly 1.5 KB/s, and a single fetch is capped (about 32 KB of captured page text, plus a receive-time ceiling), so large pages truncate. When a page hits the cap, the reader prepends a "Page too large to load fully." notice. Light, text-heavy pages such as DuckDuckGo Lite work well; large portals (the Wikipedia portal is the worst case seen) do not.
- **No redirect following.** 301 and 302 responses show the server's short "document has moved" body instead of following it. This affects apex-to-www hops (for example `wikipedia.org` to `www.wikipedia.org`) and any http-to-https hop. The workaround is to follow the link by hand.
- **URL parsing.** A URL of the form `host?query` with no path slash (for example `https://example.com?q=foo`) folds the query into the hostname. Use a form with a path (`https://example.com/search?q=foo`), which parses correctly.
- **No TLS certificate verification.** HTTPS connections are not certificate-checked, so they are not protected against a man-in-the-middle.
- **Bot-flagged requests.** The request uses a simple `Meck-P4` user agent, which Cloudflare flags, so Cloudflare-gated sites do not load yet.
- **No cookies, Referer, POST, gzip, or chunked transfer.** So no logins, no session-cookie or referrer-gated sites, and no compressed or chunked responses.

### Not in v0.6

The **IRC client** (Stage 6) from the upstream Meck web reader is not started yet, and its placeholder has been removed for now. Several of the limitations above (redirect following, browser-like request headers, a cookie jar, POST and login support, certificate verification) are planned follow-up work.

-----

## Maps
<img width="250" height="500" alt="IMG_3025-EDIT" src="https://github.com/user-attachments/assets/de354952-763b-4279-8897-fd3f2825d6a2" />

Tap the **Maps** tile on the home grid to open an offline slippy-tile map of your area. The map renders OSM PNG tiles from `/sdcard/tiles/{z}/{x}/{y}.png` via LVGL’s image widget plus LV_USE_LODEPNG. Pan and zoom by touch. A GPS dot follows your fix when GPS is enabled; markers overlay contact positions filtered by type (repeaters by default), with a filter modal for switching to other types.

### Getting map tiles

You need to provide your own map tiles. There are a couple of ways to get them:

- **Pre-downloaded tile bundles** (a good way to support MeshCore development):
  - <https://buymeacoffee.com/ripplebiz/e/342543> (Europe)
  - <https://buymeacoffee.com/ripplebiz/e/342542> (US)
- **Roll your own** with a Python downloader script that fetches the areas you want:
  - <https://github.com/fistulareffigy/MTD-Script>
  - <https://github.com/TheBestJohn/MTD-Script> — a modified fork with parallel downloads and additional error handling

### Where the tiles go

Once you’ve downloaded them, copy the `tiles/` folder to the **root of your SD card** so the path on the device is `/sdcard/tiles/{z}/{x}/{y}.png`.

### SD card sizing

Meck-P4 works with SD cards up to 1 TB. Very large tile folders (tens of GB) can make the maps screen feel sluggish -- tile lookup walks the FAT directory structure, and on huge cards the seek overhead becomes noticeable. If you only need tiles for your local area, downloading just the zoom levels and bounding box you actually care about keeps things snappier than dumping the whole continent on the card.

-----

## Games

Tap **Games** (swipe down on the home grid to reveal the hidden row) to open the games menu. As of v0.8 it holds one entry, a **Game Boy / Game Boy Color emulator**, built on the [Peanut-GB](https://github.com/deltabeard/Peanut-GB) core. It runs original Game Boy (`.gb`) and Game Boy Color (`.gbc`) ROMs at full speed with sound. Game Boy Advance (`.gba`) is a different machine entirely and is not supported.

### ROMs

Put ROM files in a folder named **`roms`** at the top level of the SD card (`/sdcard/roms/`). The browser lists `.gb` and `.gbc` files in that folder (no subfolders), up to 64 of them, and skips the `._` metadata files macOS leaves on FAT cards. Original Game Boy titles are shown with the colourisation a real Game Boy Color would apply to them.

One game is bundled: **[µCity](https://github.com/AntonioND/ucity)** (GPL-3.0, by Antonio Niño Díaz), a city-building game written for the Game Boy Color. It is copied to `/sdcard/roms/ucity.gbc` the first time you open the games menu with an SD card inserted, so there is something to play before you copy anything over. If you are looking for more, [Halo Combat Devolved](https://sofaswordsman.itch.io/halo-combat-devolved) is a free 2 MB Game Boy Color homebrew that runs well on the P4 (it is not bundled).

### Controls

With the **K270 keyboard** detected at boot, the keys are the controls and nothing is drawn over the game:

|Key           |Game Boy button|
|--------------|---------------|
|Arrow keys    |D-pad          |
|**K**         |A              |
|**J**         |B              |
|**Enter**     |Start          |
|**Space**     |Select         |
|**Mic key**   |Mute / unmute  |
|**Esc**       |Quit to the ROM list|

Without a keyboard, **on-screen controls** are drawn: a d-pad, A and B, SELECT and START, and a MUTE button. Several can be held at once — a direction and A together, for example — because the touch controller's fingers are read directly rather than through the single-point UI pointer. Held controls light up. In portrait the game sits at the top with the controls below; in landscape the game is centred with the d-pad on the left and the buttons on the right. The **back chevron** top-left quits to the ROM list in either mode.

In-game button meanings are the game's own. µCity's manual (in its repository) covers its controls; on the Game Boy convention, A confirms menu choices, so use **K** (or the on-screen A) on its start menu.

### Saves and sound

Games that save to cartridge RAM keep their saves: the emulator writes a **`.sav` file next to the ROM** when you quit (`ucity.gbc` → `ucity.sav`) and loads it on the next launch. The format is the standard raw cart-RAM dump every desktop emulator uses, so saves can be copied between the P4 and a PC in either direction. Saves are written on quit, not continuously — power off mid-game and that session's progress is lost. Real-time-clock state (Pokémon Crystal's day/night cycle) is not yet persisted; the in-game clock restarts each boot.

Sound plays through the speaker at the **volume set in the audio player**. Launching a game stops any audiobook or music that is playing; the audiobook's resume position is kept. Mute (mic key or the MUTE button) silences the game without changing the saved volume and is cleared when you quit.

### Limitations in v0.8

- **2 MB games and memory.** The P4's PSRAM is heavily used by the mesh message history, and a 2 MB ROM needs 2 MB of it in one contiguous block. The emulator reserves that block on the first game you launch after boot and keeps it for the rest of the session, so launching *any* game early guarantees the big ones work later. If a 2 MB game refuses to launch late in a long session (serial shows `ROM alloc failed` with the largest free block), reboot and launch it first. A structural fix (allocating message rings only for channels that exist) is on the road-map.
- The games menu and ROM list are navigated by touch (or Esc to go back on the keyboard); keyboard-only row selection is not yet wired.
- One ROM folder, no subfolders; 64 ROMs maximum.
- Single frame buffer: fast horizontal motion may show a brief tear line.

-----

## Config Export

Tap **Export Config** in Settings to open a modal with four section checkboxes:

|Section       |What it includes                                                                                   |
|--------------|---------------------------------------------------------------------------------------------------|
|**Identity**  |Your Ed25519 keypair (public + private key). Check this to back up or transfer your mesh identity. |
|**Channels**  |All channel names, secrets, and scope settings.                                                    |
|**Contacts**  |Your full contact list with public keys, types, names, and path data.                              |
|**Radio**     |Frequency, bandwidth, spreading factor, coding rate, TX power, path hash mode.                    |

All four are checked by default. A warning label appears when Identity is checked, since the exported file will contain your private key in plain text.

Tap **Export** to write the file to the SD card root as a MeshCore-app-compatible JSON file (named `meck_config_<timestamp>.json`). The result appears as "OK: filename" in green or "Export failed" in red on the Backup to SD status line.

The exported JSON follows the same format used by the MeshCore companion apps, so you can use it to transfer your identity and settings between devices or to back up your configuration before a factory reset.

-----

## Config Import

Config import from JSON is supported via a boot-time file detection mechanism. To import a configuration:

1. Export your config from the MeshCore companion app (iOS/Android) as a JSON file.
2. Rename it to `import.json` and place it on the SD card at `/sdcard/meshcore/import.json`.
3. Reboot the device. Meck detects the file during `meck_app_init()`, parses it via cJSON, and applies the identity, channels, contacts, and radio settings it contains.
4. After a successful import, the file is moved to `/sdcard/meshcore/import.history/import-YYYYMMDD-HHMMSS.json` so it won't be imported again on subsequent boots.
5. If the file is malformed (bad JSON, missing keys, wrong-length hex), Meck prints a warning to serial and leaves `import.json` in place for you to fix and try again.

This is the primary way to transfer your mesh identity from another device or from the MeshCore companion app to Meck-P4.

-----

## Debug Logs

The Debug Logs sub-screen (Settings > Debug Logs) captures all firmware `printf` output to an SD card log file for troubleshooting. This is useful when diagnosing radio issues, packet handling problems, or unexpected behaviour that's hard to reproduce while tethered to a serial console.

### How it works

When you tap **Start**, Meck opens a new log file at `/sdcard/meshcore/logs/log_<unix_timestamp>.txt` and redirects all `printf` output from the firmware to that file. Every Meck source file uses a macro layer (`meck_log.h`) that routes `printf` calls through `meck_debug_log_printf()`, which writes to the SD file under a mutex when logging is active, or falls through to the UART console when it's not.

While logging is active, **no output goes to the USB serial console**. Serial output resumes the moment you tap **Stop**.

### Controls

|Button     |Action                                                                                                                                                                    |
|-----------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|**Start**  |Opens a new log file and begins capturing. Disabled while a session is already active.                                                                                    |
|**Stop**   |Closes the current log file and restores serial output. Disabled when no session is active.                                                                               |
|**Export** |Copies the most recent log file to the SD card root as `/sdcard/meck_debug_<unix_timestamp>.txt` for easy access when the card is mounted on a computer. Only available after Stop.|

A status label shows the current state: "Idle" when no session exists, "Recording -> filename" while active, "Stopped -> filename" after stopping, or "Exported -> filename" after an export.

### File locations

Log sessions are stored in `/sdcard/meshcore/logs/` with filenames based on the device's UTC clock at the time Start is tapped (e.g. `log_1748230800.txt`). If the clock hasn't synced yet, the filename will be `log_0.txt`.

The Export button copies the most recent session to the SD card root (e.g. `/sdcard/meck_debug_1748230800.txt`) so you don't have to navigate into the `meshcore/logs/` directory when pulling the file from the card. The original log file in `meshcore/logs/` is preserved.

-----

## Settings

Tap the **Settings** tile on the home grid to open the settings screen.

|Setting             |Edit Method                                                                                                                                                                                                                     |
|--------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|**Node Name**       |Tap to open virtual keyboard, type, **Confirm** to save                                                                                                                                                                         |
|**Frequency**       |Tap to open numeric editor, type exact value (e.g. 916.575), **Confirm** to save and apply                                                                                                                                      |
|**Bandwidth**       |Tap to open numeric editor, type value in kHz (e.g. 62.5), **Confirm** to save and apply                                                                                                                                        |
|**Spreading Factor**|Tap to open numeric editor, type value (5-12), **Confirm** to save and apply                                                                                                                                                    |
|**Coding Rate**     |Tap to cycle: 4/5, 4/6, 4/7, 4/8                                                                                                                                                                                                |
|**Radio Preset**    |Tap to open preset picker — 17 community presets covering AU, US, EU, CN regions, plus three 2.4 GHz presets on the LR2021 build. Selecting a preset populates the Frequency, Bandwidth, SF, and CR fields above. Shows “Custom” when the current values don’t match any preset.|
|**TX Power**        |Tap to cycle: 10 / 14 / 17 / 20 / 22 dBm. The LR2021 build adds 12 to the ladder, and on 2.4 GHz frequencies the cycle is limited to 10 / 12                                                                                                                                                                                        |
|**Path Hash Mode**  |Tap to cycle: 1-byte / 2-byte / 3-byte (default 2-byte matches the AU mesh)                                                                                                                                                     |
|**Default Region**  |Tap to open text editor, enter region name (e.g. `au-nsw`), **Confirm** to save. Empty = unscoped. See [Region Scope](#region-scope).                                                                                           |
|**UTC Offset**      |Tap to adjust (-12 to +14)                                                                                                                                                                                                      |
|**Font Size**       |Tap to cycle: Classic / Larger / Extra Large. Rescales every label live, without a reboot or screen rebuild. Also sizes the reader's body text and the Notes markdown view.|
|**Home Color**      |Tap to cycle: Plain / Multi                                                                                                                                                                                                     |
|**Canned Messages >>**|Opens the Canned Messages sub-screen — five editable message slots sent from the compose screens via the keyboard mic key. See [Canned Messages](#canned-messages).|
|**Antenna**         |Tap to toggle: Internal / External. Selects the SKY13453 LoRa antenna port and applies live. On 2.4 GHz frequencies keep this on Internal — the external path is sub-GHz only for now.                                                                                                                                    |
|**Position >>**     |Opens the Position sub-screen (latitude, longitude, share position mode, copy position). See [Position Adverts and Share Position](#position-adverts-and-share-position).|
|**Contacts >>**     |Opens the Contacts sub-screen (auto-add policies, type toggles)                                                                                                                                                                 |
|**Channels >>**     |Opens the Channels sub-screen (per-channel region scope, notification preferences, notification tones, add/delete channels)                                                                                                     |
|**Backup to SD**    |Force-write of every NVS blob to the SD card. Tap shows OK (count) or Failed                                                                                                                                                    |
|**Export Config**   |Opens a modal with four section checkboxes (Identity, Channels, Contacts, Radio). Tap Export to write a MeshCore-app-compatible JSON file to SD. See [Config Export](#config-export).                                               |
|**Debug Logs >>**   |Opens the Debug Logs sub-screen. Captures all firmware printf output to an SD log file for troubleshooting. See [Debug Logs](#debug-logs).                                                                                          |
|**Brightness**      |Slider: 12% to 100% -- applies live                                                                                                                                                                                              |
|**Keyboard Backlight**|Slider: 5% to 100%, default 25% -- applies live, saved on release. Keyboard builds only. The LilyGo key on the keyboard toggles the light on and off; this sets the level it comes on at. See [T-Display P4 Keyboard (K270)](#t-display-p4-keyboard-k270).|
|**Auto Off**        |Tap to cycle: Never / 1 / 2 / 5 / 10 / 30 minutes — when idle the screen switches off, and on the AMOLED build the display pipeline shuts down too, letting the CPU drop to 40 MHz. Wake with the **boot button** (touch wake is not yet supported). See [Screen-Off Power Saving](#screen-off-power-saving)            |
|**Orientation**     |Tap to toggle: Portrait (default) / Landscape. Applies live, rebuilding every screen at the new rotation, and persists across reboots. See [Screen Orientation](#screen-orientation).|
|**KB Theme**        |Tap to toggle between Dark (default) and Light virtual keyboard themes. See [Virtual Keyboard](#virtual-keyboard) for details.                                                                                                  |
|**KB Layout**       |Tap to cycle: QWERTY / AZERTY / QWERTZ / ЙЦУКЕН (Cyrillic). Layout switches apply live to every keyboard instance.                                                                                                            |
|**Experimental Features >>**|Opens the Experimental Features sub-screen — kept a level away from the main list on purpose. Currently holds one action, **Delete all contacts**: after a confirmation card, every contact is deleted (favourites and custom paths included) along with the direct-message history; channel messages are kept, and the device restarts when the purge finishes.|
|**Identity**        |Read-only display of your public key                                                                                                                                                                                            |

All settings persist via NVS with an SD card mirror.

-----

## WiFi Companion

As of v0.4, Meck-P4 can act as a WiFi companion radio for the MeshCore app.
The onboard ESP32-C6 provides the WiFi link (over SDIO using AT commands),
and the MeshCore app connects to the device over your local network on TCP
port 5000.

### Connecting to WiFi

WiFi is configured from the Settings screen:

1. Open **Settings** from the home grid.
2. Tap the **WiFi Companion** row to open the WiFi settings.
3. Enter your network **SSID** and confirm.
4. Enter your network **password** and confirm.
5. Toggle **WiFi on**.
6. Wait a few seconds for the **IP address** field to populate.

The device stores the credentials and reconnects automatically on later
boots. The assigned IP address is also shown on the home screen and on the
settings page, and refreshes live.

When WiFi is active, the screen dims to zero brightness instead of entering
light sleep, because light sleep would drop the SDIO bus and the TCP
connection. Touch or the boot button wakes the screen.

### Connecting the MeshCore app

1. Note the IP address shown on the device after it joins your network.
2. In the MeshCore app, add a companion device over WiFi (TCP) using that IP
   address and port **5000**.

For instructions on using the MeshCore companion app itself (messaging,
contacts, channels, and so on), see <https://meshcore.io/>.

-----

## GPS

The L76K GPS module is driven via UART1. Fix status, satellites, position,
altitude, and NMEA sentence rate populate continuously and are displayed
on the GPS tile (tile 4).

|Gesture                |Action            |
|-----------------------|------------------|
|Swipe to GPS tile      |View live fix data|
|**Long press the tile**|Toggle GPS on/off |

When toggled off, the L76K is placed into standby (saves around 25 mA at
the module while preserving the almanac for fast re-acquisition), the
parser stops, and the tile shows a clear OFF state. The choice persists
across reboots.

First cold-start fix typically takes 12–13 minutes outdoors with clear
sky; subsequent fixes after standby are much faster.

-----

## Battery

The BQ27220 fuel gauge reads voltage, current, state of charge, and chip
temperature. Cell design capacity is configured at boot as **1000 mAh** per [LilyGo wiki FAQ 9.9](https://wiki.lilygo.cc/get_started/en/Display/T-Display-P4/T-Display-P4.html), which is the canonical reference for the T-Display P4 battery setup. The wiki additionally instructs running one full **charge → natural discharge to power-off → recharge** cycle on first use so the gauge can learn its real Full Charge Capacity (FCC). After that single calibration cycle, the gauge’s coulomb counter references against the correct capacity and percentage readings are accurate.

If a stale FCC from previous firmware is detected (FCC > 1000 mAh), Meck logs a warning at boot prompting the calibration cycle and caps displayed FCC at the design value to keep the UI sane until the gauge re-learns. Direct readings (voltage, current) are unaffected by FCC state.

The Battery tile shows:

- **Voltage** with a voltage-curve charge percent estimate
- **Charge%** as reported by the BQ27220, recomputed against `min(FCC, design_capacity)` so a stale internal FCC can’t skew the displayed percentage
- **Current** in mA, with `idle` / `charging` / `discharging` label
- **Chip temp** — the BQ27220’s die temperature, **not** the cell
  temperature. The cell’s NTC is wired to the LGS4056H charge IC for
  over-temp protection, not to the gauge, so the gauge can’t read the cell
  directly. Expect 35–45°C while the device is active, dropping toward
  ambient when idle.
- **Remaining mAh / Full mAh**
- **Time empty** estimate when discharging

Tap any battery label anywhere in the UI to switch that reading between
percentage and voltage.

On keyboard builds this tile gains a second column for the keyboard's 21700
pack, with its own source and capacity toggles — see
[Batteries](#batteries) under the keyboard section, which also explains why
the pack's percentage and mAh figures are estimates.

-----

## Clock Sync

Meck-P4 has no hardware RTC backup yet (PCF8563 is initialised but not
read on boot or written on shutdown), so the clock starts unset on every
reboot. Once synced, the clock is used for message timestamps and the
status bar display.

The clock is automatically synced from any of these sources:

1. **MeshCore advert timestamps** — adverts received from other nodes
   include a timestamp field. The first plausible advert timestamp after
   boot becomes the clock source. This works for any device on a healthy
   mesh — no GPS or companion app needed.
1. **GPS RMC sentences** — once the L76K acquires a satellite fix, the
   parsed UTC time is pushed into the soft RTC at the GPS sentence rate.

The plausibility window is generous (rejects advert timestamps before
2025-01-01 or after 2032-01-01) so legitimate adverts always pass while
obviously broken peers don’t poison the clock.

The [Timezones](#timezones) home page reads the same clock as the status
bar, so the two always agree on whether the time is known.

-----

## Persistence

Meck-P4 uses **NVS-primary, SD-mirror** persistence:

- Every save (prefs, channels, contacts, identity) writes to NVS first
  for speed, then to `/sdcard/meshcore/` as a backup.
- On boot the device reads from NVS (fast). If NVS is empty (fresh flash,
  factory erase) it transparently restores from the SD backup and writes
  it back to NVS.
- Channel message history is written per-channel to
  `/sdcard/meshcore/messages/` so the last several hundred messages per
  channel survive reboots.
- A manual **Backup to SD** button in Settings force-writes every NVS blob
  to the card — useful if you suspect an automatic write was missed.

-----

## Default Radio Settings

Meck-P4 boots on **Australia Narrow**: 916.575 MHz / SF7 / BW 62.5 kHz /
CR 4/8 / sync word 0x1424 / TX 22 dBm. Change these via Settings on the
device (either by selecting a radio preset or by editing Frequency,
Bandwidth, Spreading Factor, and Coding Rate individually), or edit the
defaults in `components/meshcore/variant.h` before building if you want a
different region’s defaults baked in.

The radio preset picker covers 17 presets across AU, US, EU, and CN
regions. Selecting a preset populates all four radio parameter fields;
you can then customise individual values (e.g. changing only the Coding
Rate) and the preset row will show “Custom”.

-----

## Repository Layout

- `components/meshcore/` — Meck radio, mesh, persistence, and UI code
- `components/codec2/` — Codec2 voice codec (1200bps mode), ESP-IDF component wrapper around drowe67/codec2
- `main/examples/lvgl_9_ui/` — LilyGo’s display + LVGL bring-up, lightly
  modified to hand off to Meck after init
- `components/cpp_bus_driver/` — LilyGo’s hardware driver collection
  (BQ27220 fuel gauge, SX1262 radio, XL9555 IO expander, L76K GPS, and
  friends)
- `tools/build-release.sh` — single-command release-image builder
- everything else — straight from the upstream LilyGo example tree

The Meck-specific work lives almost entirely in `components/meshcore/`. If
you want to hack on the firmware, that’s the directory to look at first.
Files of particular note:

- `MeckUI.cpp` — LVGL screens, settings, navigation
- `MeckMesh.h` — protocol-side hooks: receive, send, advert handling, ring
  buffers, contact mutation, channel migration, active-discovery state
- `MeckDataStore.h` — NVS and SD persistence
- `MeckVoice.h` — voice over LoRa: Codec2 encode/decode, VE3 protocol, session management (not yet enabled)
- `MeckPicture.h` — picture over LoRa infrastructure (not yet enabled)
- `MeckAudio.cpp` / `MeckAudio.h` — audio backend wrapping `chmorgan/esp-audio-player` for WAV + MP3 playback
- `MeckAudioUI.cpp` / `MeckAudioUI.h` — audio browser and Now Playing screens
- `MeckGBC.cpp` / `MeckGBC.h` — Game Boy / Game Boy Color emulator: games menu, ROM browser, render, input, saves, sound
- `peanut_gb.h` — vendored Peanut-GB core (tvecera gbc-rtc-fix branch) with two marked Meck patches for upstream bugs
- `minigb_apu.c` / `minigb_apu.h` — vendored Game Boy APU (sound) with the sample rate patched for the ES8311
- `ucity_rom.gbc` — the bundled µCity ROM, embedded and copied to SD on first use
- `NotifSounds.h` — per-channel notification tone config, SD scanning, and playback request queue
- `BundledSounds.h` — 7 default notification MP3s embedded as byte arrays, copied to SD on first boot
- `es8311.cpp` — codec write-fn / clock reconfig / volume control routed through LilyGo’s `Cpp_Bus_Driver::Es8311`
- `meck_app.cpp` — lifecycle: NVS init, identity, prefs, mesh task spawn
- `target.cpp` — radio attach, deferred-config queue, battery accessors,
  antenna selection
- `meck.h` — the public API surface main.cpp uses

-----

## Differences from upstream Meck (T-Deck Pro / T5S3 builds)

The P4 build is structurally a different beast: ESP-IDF instead of
PlatformIO, MIPI DSI display instead of e-paper, capacitive touch + virtual
keyboard instead of physical keys, RISC-V instead of Xtensa. The protocol
layer is shared MeshCore code, but the integration glue (UI, drivers,
persistence) is largely new.

Several upstream Meck features aren’t yet present in Meck-P4 — see the
Road-Map below for the full list.

-----

## Contributing

Open an issue first for anything substantial — it’s faster to agree on
direction before code than to rework after the fact. Style follows the
existing files (concise, embedded-style C++; no dynamic allocation outside
init; no retroactive reformatting of unchanged code).

For minor fixes, just open a PR.

-----

## Road-Map / To-Do

There are a number of fairly major features still in the pipeline, with
no particular timeframes attached.

**Done:**

- [x] Core port: ESP-IDF component structure, LVGL UI bring-up, SX1262
  radio attach
- [x] Channel messaging — send and receive on Public, #test, #sydney
- [x] Per-message metadata — hop count for incoming, ACK count for
  outgoing
- [x] Standalone home screen with seven-tile horizontal tileview
- [x] Channel picker with unread badges
- [x] Channel message history persisted to SD
- [x] Contacts list with type-prefix pills, 4-byte pubkey display, filter
  chip bar (All / Chat / Rptr / Room / Sens / Fav)
- [x] Long-press to favourite, contact detail screen with red Hold-to-
  delete button
- [x] Contacts auto-add policies (Auto All / Custom / Manual Only) with
  per-type toggles
- [x] **Discover** — active zero-hop DISCOVER_REQ/RESP scan with SNR
  readout, list of nearby repeaters/rooms, tap-to-add for nodes not
  yet in your contacts
- [x] **Virtual keyboard** — Dark / Light theme, four layouts
  (QWERTY / AZERTY / QWERTZ / Cyrillic ЙЦУКЕН), long-press accent
  popover for French and Czech diacritics
- [x] **Audio player** — WAV + MP3 playback from SD with music /
  audiobook subtrees, transport controls, volume, and a tappable
  now-playing indicator that returns to the current track without
  restarting it. Audiobook resume bookmarks; music starts fresh each
  time. See [Audio Player](#audio-player). Cover-art rendering above
  256x256 still incomplete (see audio guide for status).
- [x] Settings screen with node name, radio preset, TX power, path hash
  mode, UTC offset, home color, brightness, auto screen-off, KB theme,
  KB layout
- [x] 17-preset radio picker
- [x] NVS-primary, SD-mirror persistence for prefs / channels / contacts /
  identity
- [x] Self-healing channel-secret migration on boot
- [x] Manual “Backup to SD” trigger
- [x] BQ27220 battery readout with correct design capacity per LilyGo
  wiki FAQ 9.9, FCC clamp + recalibration warning, accurate under-load
  remaining-mAh from the chip’s coulomb counter
- [x] L76K GPS with live fix data and long-press on/off toggle
- [x] Clock sync from MeshCore advert timestamps
- [x] Clock sync from GPS RMC sentences
- [x] Adjustable screen brightness (eight-step ladder)
- [x] Auto screen-off with boot-button wake (Never / 1 / 2 / 5 / 10 / 30 min) — tears down the MIPI-DSI bus to reduce CPU usage from ~94% to ~57% CPU_MAX
- [x] Tools script for one-command merged release builds
- [x] **Direct messaging** — DM compose, DM conversation view with ACK tracking, DM Inbox in the channel picker with per-contact unread badges, per-contact persistence to SD
- [x] **Roomserver access** — login via Admin button on Room contacts, post timeline as left-aligned bubbles with author + timestamp + hops, live re-render, composer, per-room persistence to SD
- [x] **Repeater admin** — login (admin + guest sessions), Status / Send Advert / Cmd Line / Settings menu with Remember Password; as of v0.7.3 Settings holds twelve table-driven command categories covering the MeshCore v1.17.1 CLI, with parameter prompts, on/off pickers, confirmations, verbatim replies with error and timeout reporting. See [Repeater Admin](#repeater-admin).
- [x] **Trace route** — standalone Trace Path screen with manual hex hop entry and per-hop SNR results
- [x] **Per-contact path editor** — Edit Path button on contact detail with Save / Reset to Flood, supports 1-byte and 2-byte path hash modes
- [x] **Map screen** — slippy-tile viewer over `/sdcard/tiles/{z}/{x}/{y}.png` with pan, zoom, GPS dot, contact markers, filter modal
- [x] **Config export to SD** — Settings → Export Config writes a MeshCore-app-compatible JSON file with selectable sections
- [x] **Debug logs to SD** — Settings → Debug Logs → Start redirects printf to a per-session log file
- [x] **Custom radio parameters** — editable Frequency, Bandwidth, Spreading Factor (text edit with confirm button) and Coding Rate (tap to cycle) in Settings. Radio Preset row shows “Custom” when values diverge from any preset.
- [x] **Region scope** — device-wide default region in Settings, per-channel scope via Settings → Channels. Scope key derived via SHA-256, matching upstream Meck v1.7+ / MeshCore v1.15+ protocol. Repeater region management is available in Repeater Admin under Settings > Regions, and via the free-form Cmd Line.
- [x] **Channels settings sub-screen** — Settings → Channels with per-channel region scope, notification preferences (All / Mentions / None), notification tone picker, add and delete channels
- [x] **Notification sounds** — 7 bundled MP3 tones copied to SD on first boot, per-channel tone assignment via picker, automatic playback at 80% volume on new messages (skips if audio player is active), tone config persisted to SD
- [x] **Position adverts** — encode GPS position into outgoing adverts so other nodes see your location on their Maps screen
- [x] **Share position** — + button on channel/DM compose to send current position as a message
- [x] **Position settings sub-screen** — Settings > Position with manual lat/lon entry, share position mode cycle (Off / Manual / Auto-GPS), copy position button, GPS auto-update every 15 minutes
- [x] **Path view** — long-press an incoming message to see its route through the mesh; as of v0.7.3 up to 16 hops at any hash size, in a panel that grows and scrolls, reachable from the K270 keyboard. See [Path View](#path-view).
- [x] **Private channels** — channels without a # prefix generate a random secret; share via DM with contact picker, pending invite accept/dismiss
- [x] **Voice over LoRa infrastructure** — Codec2 1200bps encode/decode, ES8311 mic capture, VE3 protocol, recording/playback/send UI (infrastructure complete, not yet enabled)
- [x] **Picture over LoRa infrastructure** — chunked image transfer protocol (infrastructure complete, not yet enabled)
- [x] Updated splash screen and progress bar
- [x] Home screen expanded to nine tiles (Voice and Camera placeholders added)
- [x] Maps and Trace tile colours swapped to avoid two adjacent red tiles
- [x] MAX_GROUP_CHANNELS bumped from 8 to 12
- [x] AMOLED variant verification
- [x] **ESP32-C6 WiFi companion** — connect the MeshCore app over WiFi (TCP port 5000), with on-device SSID/password configuration. See [WiFi Companion](#wifi-companion).
- [x] **EPUB reading** — the Reader opens `.epub` files via on-the-fly epub-to-txt conversion, cached so the conversion runs only once. See [Reader](#reader).
- [x] **Notes app** — create, edit, read, rename, and delete plain-text notes under `/sdcard/notes`, reusing the reader's paging and resume bookmark. See [Notes](#notes).
- [x] **Web reader** — reader-mode browser over the ESP32-C6 (plain HTTP and HTTPS, on-screen browser, GET form fill, bookmarks, history). Ships as a work in progress; the IRC client is not yet ported. See [Web Reader](#web-reader).
- [x] **Cyrillic keyboard** — ЙЦУКЕН layout added to the KB Layout cycle (Russian / Bulgarian).
- [x] **M5Stack CardKB support** — optional physical I2C keyboard for the message composers, gated behind the `MECK_CARDKB` build flag. See [CardKB](#cardkb).
- [x] **Screen orientation** — Portrait / Landscape toggle in Settings with a live rebuild. See [Screen Orientation](#screen-orientation).
- [x] **Keyboard bubble ring** (v0.7.3) — on the message screens Up/Down ring the bubbles and the compose box, Enter opens Path View / Retry or starts typing, Esc closes; the Path View, Retry and Repeater Admin overlays are keyboard-driven too. See [Navigating without touching the screen](#navigating-without-touching-the-screen).
- [x] **Emoji set expanded** (v0.7.3) — 195 emoji plus the AU and EE flags, adding the Meck Watch's set (including the red heart); regenerable with `tools/bake_p4_emoji.py`. See [Emoji](#emoji).
- [x] **SX1262 receive handling aligned with MeshCore v1.17.x** (v0.7.3) — the receiver now sees a packet from its preamble and a stalled preamble or header no longer holds up sending until the next packet (upstream PRs #3036 and #2977, re-implemented for the P4's own radio driver).
- [x] **MeshCore core sync** (v0.7.3) — malformed `PAYLOAD_TYPE_PATH` packets are rejected (MeshCore v1.17.0).
- [x] **Game Boy / Game Boy Color emulator** (v0.8) — Peanut-GB core, full speed with sound, keyboard or multi-touch on-screen controls, `.sav` battery saves, µCity bundled. See [Games](#games).
- [x] **Home grid hidden row** (v0.8) — Voice and Games tiles revealed by swiping down on the grid; touch paging wrap-around now works (it had never fired by touch); KBD battery gauge readable at larger font sizes.
- [x] **Display memory fixes** (v0.8) — the landscape rotation buffer is allocated once instead of per frame (map screen no longer freezes under tile-decode pressure); the screen-off path reserves the framebuffer's memory so waking the screen cannot fail for lack of it.
- [x] **Audio player in landscape** (v0.8) — Now Playing scrolls so all controls are reachable.
- [x] **LR2021 radio support** (v0.7.4) — LilyGo's LR2021 variant of the T-Display P4 runs its own build of the firmware, verified working, adding 2.4 GHz LoRa alongside sub-GHz (2.4 GHz on the internal antenna only for now). See [Supported Devices](#supported-devices).

**Pending:**

- [ ] Ethernet companion over the P4's onboard IP101GRI PHY, following upstream MeshCore's Ethernet support

- [ ] Voice over LoRa — enable Codec2 voice messaging end-to-end (infrastructure is complete, UI and protocol are in place, pending final integration and testing)
- [ ] Picture over LoRa — enable image transfer end-to-end (infrastructure is complete, pending final integration)
- [ ] Voice tile is live but experimental (it warns on entry); Camera stays disabled
- [ ] Games follow-ups — allocate message rings only for channels that exist so 2 MB ROMs load at any point in a session; persist MBC3 real-time-clock state (Pokémon Crystal's clock); keyboard row selection in the games menu and ROM list; Snake and Minesweeper as further Games entries; optional haptic tick on the touch buttons
- [ ] ESP32-C6 BLE companion firmware (WiFi companion is complete as of v0.4)
- [ ] Mentions-only notification filtering — the "Mentions" preference currently behaves the same as "All"; filtering to trigger only on @nodename is planned
- [ ] Serial CLI commands on the P4 — local serial settings require a serial terminal, which is not yet implemented. Remote CLI via Repeater Admin works normally
- [ ] M4B audiobook files not supported (MP3 and WAV only)
- [ ] Audio player pause button can take a second or two to refresh after a track change or when returning to the Now Playing screen; it no longer disappears for the rest of the session
- [ ] Audio cover-art rendering at >256x256 — pre-flight succeeds but the LVGL heap
  can’t allocate decoded framebuffers for larger sizes; needs decode-time
  downscale or a streaming decoder. 256x256 size png file works.
- [ ] Web reader follow-ups — redirect following, browser-like request headers, a cookie jar, POST and login support, and TLS certificate verification (the web reader itself ships in v0.6; see [Web Reader](#web-reader))
- [ ] IRC client — port the upstream Meck web reader's IRC client (Stage 6)
- [ ] PCF8563 hardware RTC integration — read on boot, write on shutdown
  so time survives power-off
- [ ] Light sleep actually engaging — light sleep is disabled in the PM config; screen-off power saving currently comes from dynamic frequency scaling (on the AMOLED build the screen-off path shuts the display pipeline down so the CPU can reach 40 MHz).
- [ ] Touch wake from screen-off — currently boot-button only
- [ ] OTA firmware updates over WiFi via the ESP32-C6
- [ ] GPS cold-boot acquisition speed-up — EASY (predicted ephemeris)
  doesn’t appear to be persisting across reboots as intended

-----

## License

MIT for Meck-specific code. The wider project links libraries with mixed
licensing including GPL-3.0 (GxEPD2, esp32-audioI2S) and LGPL-2.1
(Codec2); the combined firmware binary is effectively GPL-3.0 when
distributed. See the upstream Meck README for the full dependency
license matrix.

The Game Boy emulator adds [Peanut-GB](https://github.com/deltabeard/Peanut-GB) (MIT) and [minigb_apu](https://github.com/deltabeard/minigb_apu) (MIT), both vendored under `components/meshcore/`, and bundles the [µCity](https://github.com/AntonioND/ucity) ROM by Antonio Niño Díaz under GPL-3.0; its source is available at that repository. No commercial Game Boy software is included.
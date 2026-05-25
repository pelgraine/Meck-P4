# Meck-P4 — MeshCore for the LilyGo T-Display P4

A port of [Meck](https://github.com/pelgraine/Meck) (a MeshCore fork) to the
LilyGo T-Display P4. Targets the ESP32-P4 main MCU; the onboard ESP32-C6 is
not currently used by Meck on this device while the firmware is still at pre-release stage. Built on top of LilyGo’s
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
- [Touch Navigation](#touch-navigation)
- [Screen-Off Power Saving](#screen-off-power-saving)
- [Virtual Keyboard](#virtual-keyboard)
- [Channel Messages](#channel-messages)
- [Channel Picker](#channel-picker)
- [Contacts](#contacts)
- [Direct Messages](#direct-messages)
- [Repeater Admin](#repeater-admin)
- [Room Servers](#room-servers)
- [Region Scope](#region-scope)
- [Notification Sounds](#notification-sounds)
- [Per-Contact Path Editor](#per-contact-path-editor)
- [Trace Route](#trace-route)
- [Discover](#discover)
- [Audio Player](#audio-player)
- [Maps](#maps)
- [Settings](#settings)
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

Meck-P4 currently targets the LilyGo T-Display P4 (TFT version). The AMOLED
variant has not been tested but should work after adjusting the display
init in `main/examples/lvgl_9_ui/main.cpp`.

|Device                |Display                            |Input                                    |LoRa  |Battery                     |GPS         |RTC                                   |
|----------------------|-----------------------------------|-----------------------------------------|------|----------------------------|------------|--------------------------------------|
|**T-Display P4** (TFT)|4.05” punch-hole TFT LCD (1232×568)|GT911 capacitive touch + virtual keyboard|SX1262|BQ27220 fuel gauge, 1000 mAh|L76K (UART1)|PCF8563 (initialised but not yet used)|

The T-Display P4 uses the ESP32-P4 (RISC-V dual-core) with 16 MB flash and
32 MB PSRAM. The onboard ESP32-C6 (WiFi 6 / BLE 5.3 coprocessor) is present
but not yet used by Meck.

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

The home screen is a horizontal seven-tile layout. Swipe left or right to
navigate between tiles. The Home tile (tile 0) shows node name, unread
message count, battery percentage and clock in the top-right corner, and
a ten-tile navigation grid (2 columns × 5 rows) linking to Messages,
Contacts, Settings, Reader, Notes, Discover, Trace, Maps, Audio, and Web.

|Tile           |Purpose                                                                                            |
|---------------|---------------------------------------------------------------------------------------------------|
|0 Home         |Node name, unread message count, clock + battery, ten-tile navigation grid                         |
|1 Recent Heard |Live list of nodes whose adverts have been received                                                |
|2 Radio Details|Current frequency, bandwidth, spreading factor, coding rate, TX power, sync word                   |
|3 Advert       |Long-press to send a manual advert                                                                 |
|4 GPS          |Fix status, satellites, position, altitude, sentence rate. Long-press the tile to toggle GPS on/off|
|5 Battery      |Voltage, charge percent, current, chip temperature, remaining mAh                                  |
|6 Shutdown     |Long-press to power down                                                                           |

-----

## Touch Navigation

The T-Display P4 has no physical keyboard. All interaction is via touch
gestures. Text entry uses an on-screen virtual keyboard that appears when
needed.

|Gesture       |Description                                                                                                   |
|--------------|--------------------------------------------------------------------------------------------------------------|
|**Tap**       |Touch and release quickly. Opens tiles, selects items, advances pages.                                        |
|**Swipe**     |Touch, drag, release. Direction determines action (scroll, page turn, switch tile/filter).                    |
|**Long press**|Touch and hold. Context-dependent: send advert, toggle GPS, delete contacts, retry failed messages, power off.|

-----

## Screen-Off Power Saving

The Auto Off timer (Settings > Auto Off, tap to cycle: Never / 1 / 2 / 5 / 10 / 30 minutes) puts the device into a low-power screen-off state after the set period of inactivity. When the timer fires, the MIPI-DSI display bus is torn down and CPU usage drops from ~94% to ~57%. The radio stays active and continues receiving messages in the background.

> **To wake the device, press the P4 boot button on the side of the T-Display P4.** Touch wake is not yet supported. The P4 boot button is the third button from the top on the right edge, between the C6 Reset and P4 Reset buttons (labelled “BOOT” under the “P4” heading on the case). If the screen stays black after a period of inactivity, this is normal – press the boot button to bring it back.

This is not true light sleep. The PM config sets `light_sleep_enable=true`, but some PM locks are still preventing light sleep from actually engaging. The visible power saving comes from dynamic frequency scaling running more aggressively when the screen-on workload disappears. With low to moderate usage at 35% brightness, screen-off idle can add roughly an hour to an hour and a half of additional run time.

-----

## Virtual Keyboard

Text entry (node name, message compose, channel name, channel secret) uses an on-screen virtual keyboard. The keyboard appears automatically when you tap a field that needs input, and dismisses on Send / Enter / Back.

### Theme

Two themes are available, switched from **Settings → KB Theme**:

- **Dark** (default) — light keys on a dark background. Easier on the eyes at night and matches the rest of the Meck UI.
- **Light** — dark keys on a light background, in case you prefer the higher contrast in bright daylight.

The choice persists via NVS and applies live to every keyboard instance the moment you change it. No reboot needed.

### Layout

Three physical layouts are available, cycled from **Settings → KB Layout**:

|Layout              |Description                                                                                                                                      |
|--------------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
|**QWERTY** (default)|Standard English layout                                                                                                                          |
|**AZERTY**          |French layout. A↔Q and W↔Z swap from QWERTY; M moves from row 3 (after L) to the right of row 2 (after L), giving row 2 ten cells and row 3 nine.|
|**QWERTZ**          |German layout. Y↔Z swap; same shape as QWERTY everywhere else.                                                                                   |

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

**Per-message metadata:**

- **Incoming messages** display a small hop-count badge showing how many repeaters the packet passed through to reach you. Direct receptions show 0 hops.
- **Outgoing messages** show a send status that updates as repeater echoes arrive:
  - **Sending…** – the message has been transmitted but no repeater echo has been heard yet.
  - **✓ Heard N Repeats** – one or more repeaters relayed the message back. The count shows how many echoes were received, confirming the message propagated through the mesh.
  - **✕ Failed** – 18 seconds elapsed with no repeater echo. The message was transmitted but no repeater confirmed receipt. This typically means no repeater is in range, or the channel’s radio parameters don’t match the repeater’s.

**Retry send:** long-press any failed outgoing message (channel or DM) to open a retry prompt. Tap **Retry Send** to re-queue the message with a fresh timestamp. The recipient may see a duplicate if the original arrives late via a slow path.

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

-----

## Contacts

Tap **Contacts** from the home grid to open the contacts list. All known
mesh contacts are shown sorted by most recently heard, with their type
prefix (colour-coded: C / R / RS / S) and a 4-byte public-key prefix to
disambiguate near-key-collisions.

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

- **Auto All** — every advert heard adds a contact
- **Custom** — per-type toggles (chat, repeater, room, sensor) decide which
  advert types to auto-add
- **Manual Only** — disables all auto-add

An **Overwrite oldest when full** toggle decides what happens when the
contacts table reaches its 2,000-entry limit.

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
|**Settings**   |Scrollable menu list of setting categories — Position, Sync Clock, Admin Password, Guest Password, Change Identity Key, Manage Regions, Neighbours, Repeat Settings. Guest sessions see only Neighbours.  |

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
|`/sdcard/audio/music/`     |Standard music playback. Resume bookmark off by default.                                         |
|`/sdcard/audio/audiobooks/`|Audiobook mode. Resume bookmark on, sleep timer available, position tracked through the playlist.|

Inside each, organise however you like (typically Artist / Album / track, or Author / Book / chapter). The audio browser shows breadcrumbs and lets you tap a track to play, with transport controls (-30s, play/pause, +30s), volume, and a progress bar on the Now Playing screen.

**Cover art** displays when a **256x256 `cover.png`** is placed alongside your tracks. Other filenames (`folder.png`, `front.png`, `album.png`) are also recognised, case-insensitive. Larger PNGs are read by the file scanner but fail to allocate at decode time — see the audio player guide for the working recipe and a downscale command if you have higher-resolution covers on hand.

**Audio format requirements:** MP3 files must be at 44.1 kHz; WAV files must be 16-bit PCM (format code 0x0001) at 44.1 kHz, mono or stereo. Files outside this window fail the format check. See the audio player guide for `ffprobe` checks and `ffmpeg` conversion commands.

**Watchdog crash on first play:** if a file with large embedded album art crashes the device during playback startup, you’ve hit the libhelix-mp3 sync-word scan issue documented in the audio player guide. The firmware has a defensive ID3v2-skip patch that should prevent this, but the durable fix is to clean your files at the source with the `tools/mp3_clean.py` script before copying them to the SD card.

For full setup instructions including the `mp3_clean.py` script usage, SD card layout, troubleshooting, and developer notes, see:

**[Audio player guide](https://github.com/pelgraine/Meck-P4/blob/main/information/Meck%20Docs/audioplayerguide.md)**

-----

## Maps

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

Meck-P4 works with SD cards up to 1 TB. Very large tile folders (tens of GB) can make the maps screen feel sluggish — tile lookup walks the FAT directory structure, and on huge cards the seek overhead becomes noticeable. If you only need tiles for your local area, downloading just the zoom levels and bounding box you actually care about keeps things snappier than dumping the whole continent on the card.

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
|**Radio Preset**    |Tap to open preset picker — 17 community presets covering AU, US, EU, CN regions. Selecting a preset populates the Frequency, Bandwidth, SF, and CR fields above. Shows “Custom” when the current values don’t match any preset.|
|**TX Power**        |Tap to cycle: 10 / 14 / 17 / 20 / 22 dBm                                                                                                                                                                                        |
|**Path Hash Mode**  |Tap to cycle: 1-byte / 2-byte / 3-byte (default 2-byte matches the AU mesh)                                                                                                                                                     |
|**Default Region**  |Tap to open text editor, enter region name (e.g. `au-nsw`), **Confirm** to save. Empty = unscoped. See [Region Scope](#region-scope).                                                                                           |
|**UTC Offset**      |Tap to adjust (-12 to +14)                                                                                                                                                                                                      |
|**Home Color**      |Tap to cycle: Plain / Multi                                                                                                                                                                                                     |
|**Contacts >>**     |Opens the Contacts sub-screen (auto-add policies, type toggles)                                                                                                                                                                 |
|**Channels >>**     |Opens the Channels sub-screen (per-channel region scope, notification preferences, notification tones, add/delete channels)                                                                                                     |
|**Backup to SD**    |Force-write of every NVS blob to the SD card. Tap shows OK (count) or Failed                                                                                                                                                    |
|**Brightness**      |Slider: 12% to 100% — applies live                                                                                                                                                                                              |
|**Auto Off**        |Tap to cycle: Never / 1 / 2 / 5 / 10 / 30 minutes — when idle, the screen tears down the MIPI-DSI bus and CPU usage drops from ~94% to ~57% CPU_MAX. Wake with the **boot button** (touch wake is not yet supported)            |
|**KB Theme**        |Tap to toggle between Dark (default) and Light virtual keyboard themes. See [Virtual Keyboard](#virtual-keyboard) for details.                                                                                                  |
|**KB Layout**       |Tap to cycle: QWERTY / AZERTY / QWERTZ. Layout switches apply live to every keyboard instance.                                                                                                                                  |
|**Identity**        |Read-only display of your public key                                                                                                                                                                                            |

All settings persist via NVS with an SD card mirror.

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
- `MeckAudio.cpp` / `MeckAudio.h` — audio backend wrapping `chmorgan/esp-audio-player` for WAV + MP3 playback
- `MeckAudioUI.cpp` / `MeckAudioUI.h` — audio browser and Now Playing screens
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
- [x] **Virtual keyboard** — Dark / Light theme, three layouts
  (QWERTY / AZERTY / QWERTZ), long-press accent popover for French
  and Czech diacritics
- [x] **Audio player** — WAV + MP3 playback from SD with music /
  audiobook subtrees, transport controls, volume, bookmarks. See
  [Audio Player](#audio-player). Cover-art rendering still incomplete
  in v0.2 (see audio guide for status).
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
- [x] **Repeater admin** — login (admin + guest sessions), Status / Send Advert / Cmd Line / Settings menu with Remember Password
- [x] **Trace route** — standalone Trace Path screen with manual hex hop entry and per-hop SNR results
- [x] **Per-contact path editor** — Edit Path button on contact detail with Save / Reset to Flood, supports 1-byte and 2-byte path hash modes
- [x] **Map screen** — slippy-tile viewer over `/sdcard/tiles/{z}/{x}/{y}.png` with pan, zoom, GPS dot, contact markers, filter modal
- [x] **Config export to SD** — Settings → Export Config writes a MeshCore-app-compatible JSON file with selectable sections
- [x] **Debug logs to SD** — Settings → Debug Logs → Start redirects printf to a per-session log file
- [x] **Custom radio parameters** — editable Frequency, Bandwidth, Spreading Factor (text edit with confirm button) and Coding Rate (tap to cycle) in Settings. Radio Preset row shows “Custom” when values diverge from any preset.
- [x] **Region scope** — device-wide default region in Settings, per-channel scope via Settings → Channels. Scope key derived via SHA-256, matching upstream Meck v1.7+ / MeshCore v1.15+ protocol. Repeater region management supported via CLI commands in the Repeater Admin screen.
- [x] **Channels settings sub-screen** — Settings → Channels with per-channel region scope, notification preferences (All / Mentions / None), notification tone picker, add and delete channels
- [x] **Notification sounds** — 7 bundled MP3 tones copied to SD on first boot, per-channel tone assignment via picker, automatic playback at 80% volume on new messages (skips if audio player is active), tone config persisted to SD

**Pending:**

- [ ] Notes app
- [ ] Audio cover-art rendering at >256x256 — pre-flight succeeds but the LVGL heap
  can’t allocate decoded framebuffers for larger sizes; needs decode-time
  downscale or a streaming decoder. 256x256 works.
- [ ] Web browser & IRC client
- [ ] PCF8563 hardware RTC integration — read on boot, write on shutdown
  so time survives power-off
- [ ] ESP32-C6 BLE companion firmware — make the device usable as a
  Bluetooth companion to the iOS / Android MeshCore apps
- [ ] AMOLED variant verification
- [ ] Light sleep actually engaging — the screen-off path releases the
  dsi_phy NO_LIGHT_SLEEP PM lock, but other PM locks still prevent
  automatic light sleep entry. Power saving in v0.3.5 comes from
  dynamic frequency scaling instead.
- [ ] Touch wake from screen-off — currently boot-button only
- [ ] OTA firmware updates over WiFi via the ESP32-C6
- [ ] GPS cold-boot acquisition speed-up — EASY (predicted ephemeris)
  doesn’t appear to be persisting across reboots as intended;
  targeted for v0.3.6

-----

## License

MIT for Meck-specific code. The wider project links libraries with mixed
licensing including GPL-3.0; the combined firmware binary is effectively
GPL-3.0 when distributed. See the upstream Meck README for the full
dependency license matrix.
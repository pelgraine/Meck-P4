# Meck-P4 audio player guide

How to put music and audiobooks on your Meck-P4 (LilyGo T-Display P4 running Meck firmware) and have them play reliably with cover art displayed.

This guide exists because the path from "I have a folder of MP3s" to "they play on the device with cover art" has three sharp edges that aren't obvious until you hit them. We hit all three; this is how to avoid them.

## TL;DR

1. Put MP3s under `/sdcard/audio/music/<Artist>/<Album>/`.
2. MP3s must be at **44.1 kHz**. Tracks at other sample rates won't play; see [Sample rate](#sample-rate) for conversion.
3. Run `tools/mp3_clean.py` over them before copying to the SD card.
4. For cover art, save a **256x256 `cover.png`** in the album folder. Larger PNGs are still not supported; see [Cover art](#cover-art).
5. WAV files do play, but only as **16-bit PCM at 44.1 kHz**. See [WAV files](#wav-files) for conversion.

## Directory layout on the SD card

The audio browser expects this tree:

```
/sdcard/audio/
├── music/
│   └── <Artist>/
│       └── <Album>/
│           ├── 01 Track Name.mp3
│           ├── 02 Another.mp3
│           └── cover.png            (256x256)
└── audiobooks/
    └── <Author>/
        └── <Book Title>/
            ├── Chapter 01.mp3
            └── cover.png            (256x256)
```

The firmware creates `/sdcard/audio`, `/sdcard/audio/music`, and `/sdcard/audio/audiobooks` on first boot if they don't exist. You're free to organise inside however you like, but the two top-level distinction matters: the player applies different defaults to `audiobooks/` content (sleep timer enabled, bookmark resume on by default, position-tracking through the playlist).

Folder and file names with spaces, ampersands, accented characters, and so on are fine. The firmware uses POSIX paths verbatim from the FAT directory entries.

## Step 1: clean your MP3s before copying

This is the single most important step. Skip it and the first track you tap will look like it hangs for five seconds and then trigger a watchdog reboot.

### Why

`libhelix-mp3` (the decoder Meck uses) finds the start of each MP3 frame by scanning byte-by-byte for an 11-bit sync pattern (`0xFFE`). When an MP3 has embedded album art in an ID3v2 `APIC` frame, that art often runs to several megabytes. Embedded JPEGs contain markers (`0xFFE0`, `0xFFE1`, `0xFFD8`) that look like sync-word hits, so the decoder has to scan through the entire art payload before it finds the real first frame. On the ESP32-P4 this takes long enough to starve the IDLE task and fire the watchdog.

Two layers of defence:

1. **Firmware patch (already in Meck-P4):** `audio_mp3.cpp` reads the 10-byte ID3v2 header at the start of each file and seeks past the tag before handing data to libhelix. This means even uncleaned MP3s eventually play, but the patch lives in a managed component that `idf.py fullclean` will overwrite.
2. **Source-side cleanup (recommended):** strip oversized art from the files themselves with `mp3_clean.py`. This is the durable fix and also recovers a lot of SD card space.

You want both. The script does the heavy lifting; the firmware patch is the safety net.

### Installing the script's dependencies

The script needs `mutagen` (mandatory) and `Pillow` (optional, for PNG cover export).

```bash
# Simplest: install for your user without a venv
pip3 install --user --break-system-packages mutagen Pillow
```

If you'd rather use a virtual environment:

```bash
python3 -m venv ~/venv-mp3
source ~/venv-mp3/bin/activate
pip install mutagen Pillow
```

`Pillow` is optional. Without it the script will still strip oversized art, but exported covers will come out in whatever format was embedded (usually JPG, which the firmware can't display). Install Pillow if you want covers to appear on the device.

Note: do not `brew install mutagen`. That installs a Go-language file sync tool with the same name, unrelated to the Python library.

### Running the script

The script lives at `tools/mp3_clean.py`. Always do a dry run first to see what it would change:

```bash
cd ~/T-Display-P4/tools
python3 mp3_clean.py --dry-run /Volumes/SE32G/audio/music
```

Replace `/Volumes/SE32G` with your SD card's actual mount path. On macOS, `ls /Volumes/` shows what's mounted. Card readers usually present the card under the volume label set when the card was formatted; the name in your boot log (`Name: SE32G` in the SDHC info block) matches.

A dry run prints what would be dropped without modifying anything:

```
DRY RUN — no changes will be written: 47 mp3 files under /Volumes/SE32G/audio/music
Drop threshold: 64.0 KB
  would drop 2.3 MB from Arcane/Rain Dance/Arcane - Rain Dance - 01 Nebula.mp3
  would drop 2.3 MB from Arcane/Rain Dance/Arcane - Rain Dance - 02 Paradox.mp3
  ...
```

Once you're happy with what it'll do, drop `--dry-run` and add `--export-cover` to save the artwork as a PNG sidecar:

```bash
python3 mp3_clean.py --export-cover /Volumes/SE32G/audio/music
```

This walks the directory, drops every `APIC` frame larger than 64 KB from each MP3, and writes the largest dropped image as `cover.png` in each album folder. The first track in a folder wins; existing covers are not overwritten.

### Options worth knowing

| Flag | Effect |
|------|--------|
| `--dry-run` | Report only, don't modify any files |
| `--export-cover` | Save dropped art as `cover.png` in the album folder |
| `--max-art-bytes N` | Drop APIC frames larger than N bytes. Default 65536 (64 KB). Use `0` to drop all art |
| `--png` / `--no-png` | Convert exported covers to PNG (default) or keep source format |

The 64 KB threshold is the operative one. Below that, art is small enough that libhelix's sync-word scan completes in well under a second and the watchdog never fires. Above that, the scan time grows roughly linearly with the embedded image size.

### Sample rate

Separate to the embedded-art problem, the MP3 decode path on Meck only handles **44.1 kHz** audio. Tracks ripped or downloaded at other rates (48 kHz is the most common one to trip you up — Bandcamp lossless, anything re-exported from a DAW, some podcast feeds) will fail to play with an error in the serial log.

Check the sample rate of a file with ffprobe:

```bash
ffprobe -i "yourfile.mp3" 2>&1 | grep "Audio:"
# Audio: mp3, 48000 Hz, stereo, fltp, 192 kb/s    ← bad
# Audio: mp3, 44100 Hz, stereo, fltp, 192 kb/s    ← good
```

Re-encode in place if needed:

```bash
ffmpeg -i input.mp3 -ar 44100 output.mp3
```

For a whole album folder:

```bash
cd "/path/to/Album"
mkdir converted
for f in *.mp3; do ffmpeg -i "$f" -ar 44100 "converted/$f"; done
```

Verify by re-running `ffprobe` on a converted file and confirming `44100 Hz`. Once happy, move the contents of `converted/` up and delete the originals.

`mp3_clean.py` doesn't touch sample rate — it only strips oversized art. Run the sample-rate conversion first, then `mp3_clean.py` over the results.

## WAV files

WAV playback works, but the player has a narrower compatibility window than MP3. Files must be:

- **Format code 0x0001** (linear PCM). Not 0xFFFE (`WAVE_FORMAT_EXTENSIBLE`), which is what most DAWs export by default for 24-bit, 32-bit, or multichannel sessions.
- **16-bit samples.** 24-bit and 32-bit PCM either produce silence or distortion.
- **44.1 kHz sample rate.** Same constraint as MP3.
- **Mono or stereo.** Multichannel files fail the format check.

Symptom of a non-compliant WAV: the player logs `unknown file type` and refuses to start. The chmorgan `esp-audio-player` library only recognises format code 0x0001 (and in newer revisions 0x0003 IEEE float). Anything in the `EXTENSIBLE` wrapper, even if the underlying samples are plain PCM, gives up before reading the GUID subfield.

Inspect a WAV's actual format with ffprobe:

```bash
ffprobe -i "yourfile.wav" 2>&1 | grep "Audio:"
# Audio: pcm_s24le, 48000 Hz, stereo, s32, 2304 kb/s    ← won't play
# Audio: pcm_s16le, 44100 Hz, stereo, s16, 1411 kb/s    ← will play
```

Re-encode a single file:

```bash
ffmpeg -i "input.wav" -acodec pcm_s16le -ar 44100 "output.wav"
```

For a whole album folder:

```bash
cd "/path/to/Album"
mkdir converted
for f in *.wav; do ffmpeg -i "$f" -acodec pcm_s16le -ar 44100 "converted/$f"; done
```

Verify with ffprobe and check the output line shows `pcm_s16le, 44100 Hz`. Once you're happy, move the contents of `converted/` up to the album folder and delete the originals.

## Cover art

Cover art now displays on the player screen, but **only for PNG files at 256x256 pixels**. Larger sizes (the typical "cover.jpg embedded by your tagging tool" output of 1400x1400 or larger) fail at decode time — see [Cover art does not display at full resolution](#cover-art-does-not-display-at-full-resolution) below.

To get a working cover:

1. In each album folder, place a file named `cover.png` sized 256x256 pixels.
2. Other filenames (`folder.png`, `front.png`, `album.png`) are also recognised, case-insensitive. `cover.png` is the convention used in this guide.
3. JPG is recognised by the file scanner but the LVGL JPG decoder is disabled in this build (see [Symptom: boot loop with "Stack protection fault" in `tjpgd`](#symptom-boot-loop-with-stack-protection-fault-in-tjpgdlv_tjpgdcdecoder_info)). Stick to PNG.

If you have a higher-resolution cover handy and want to downscale it:

```bash
ffmpeg -i source.jpg -vf "scale=256:256" cover.png
```

Or via ImageMagick:

```bash
magick source.jpg -resize 256x256 cover.png
```

If you ran `mp3_clean.py --export-cover` earlier, Pillow will have already saved a downscaled-to-600px PNG. That's still too large for the current decoder — re-resize down to 256x256 before placing it in the album folder.

## Step 2: copy to the SD card

Drag the cleaned `music/` and `audiobooks/` folders into `/Volumes/<your-sd>/audio/`. Eject the card properly, slot it into the device, power on.

The audio browser is at the home screen's audio icon. Tap a track to play.

A successful first-play log looks like:

```
MeckAudioUI: opening file '/sdcard/audio/music/Arcane/Rain Dance/.../01 Nebula.mp3'
MeckAudioUI: playlist size=8, idx=0
I (38625) MeckAudio: playing '...01 Nebula.mp3' from 0s
```

## Known issues

### Cover art does not display at full resolution

Cover art displays correctly when the file is a 256x256 PNG (see [Cover art](#cover-art) for the working recipe). Larger PNGs are read by the file scanner — including dimensions and decoder pre-flight — but fail to allocate at decode time. A 1400x1400 PNG needs ~4 MB contiguous for its decoded framebuffer, a 2500x2500 PNG needs ~12 MB, and LVGL's heap can't reliably provide that. The log shows `lv_realloc: couldn't reallocate memory` when this happens.

The proper fix (downscaling at decode time via `lv_image_set_scale_to_fit` plus a streaming decoder, or restricting the cover slot to a reliably-allocatable size) is on the roadmap. In the meantime, supply your covers pre-scaled to 256x256.

`mp3_clean.py --export-cover` saves cover art alongside your MP3s ahead of time, downscaled to 600px on its longest side when Pillow is installed. 600px is still too large for the current decoder — resize further to 256x256 before the file lands in the album folder. The script will continue exporting at 600px so the source data is preserved for the eventual fix.

## Troubleshooting

### Symptom: tapping a track produces ~5 seconds of silence, then the device reboots

Backtrace shows `task_wdt fired on IDLE1` and the audio task stuck inside `decode_mp3` / `MP3FindSyncWord`.

**Cause:** the MP3 has a large embedded album-art `APIC` frame and libhelix is scanning through it byte-by-byte looking for sync words.

**Fix:** run `mp3_clean.py` over the file. The ID3v2-skip patch in `audio_mp3.cpp` should also prevent this even on uncleaned files, but if you have a build where that patch was overwritten (after `idf.py fullclean` or a managed-component bump) you'll see this symptom return until the script is run.

### Symptom: boot loop with "Stack protection fault" in `tjpgd/lv_tjpgd.c:decoder_info`

The crash backtrace mentions `decoder_info` at `lv_tjpgd.c:82-93`, in task `main`, with the stack pointer below the stack bounds.

**Cause:** `CONFIG_LV_USE_TJPGD=y` is enabled in `sdkconfig`. TJPGD allocates a ~3 KB workspace on the caller's stack. ESP-IDF's main task only has 3584 bytes of stack to start with. The first time anything during boot calls `lv_image_decoder_get_info` (typically LilyGo's startup progress bar or a screen-load path), TJPGD's stack frame blows the limit.

**Fix:** disable TJPGD.

```bash
cd ~/T-Display-P4
sed -i.bak 's/^CONFIG_LV_USE_TJPGD=y/# CONFIG_LV_USE_TJPGD is not set/' sdkconfig
idf.py build flash monitor
```

Or via `idf.py menuconfig`: search (`/`) for `TJPGD`, toggle the checkbox off, save, exit.

If you really need JPG support, the alternatives are increasing `CONFIG_ESP_MAIN_TASK_STACK_SIZE` to 8192 or higher, or switching to `CONFIG_LV_USE_LIBJPEG_TURBO` (which allocates from heap rather than the caller's stack).

### Symptom: WAV tap shows "unknown file type" and the track refuses to start

The serial log shows the audio task accepting the open but then rejecting the format. Inspect the file:

```bash
ffprobe -i "yourfile.wav" 2>&1 | grep "Audio:"
```

If the codec field is anything other than `pcm_s16le` and the sample rate is anything other than `44100 Hz`, the file is outside the compatibility window. See [WAV files](#wav-files) for the conversion command.

The most common case is `pcm_s24le` or `pcm_s32le` at 48 kHz, which is what DAWs export by default. Even if you ignore the bit depth, the `EXTENSIBLE` format-code wrapper alone is enough to fail the check.

### Symptom: MP3 tap shows an error in the serial log and the track refuses to start

Past the obvious "the file is corrupt" check, the most common cause is a non-44.1-kHz sample rate. Inspect with `ffprobe` and convert if needed — see [Sample rate](#sample-rate).

### Symptom: contacts save fails with `errno=17` after enabling audio

This isn't audio-related, but you may see it in logs alongside playback messages. ESP-IDF's FATFS doesn't implement POSIX rename-overwrite semantics; the fix lives in `MeckDataStore.h` (unlink-before-rename). If you see `rename(.tmp, .bin) failed (errno=17)`, you're on an old `MeckDataStore.h` from before that fix.

## For developers: the firmware-side picture

The cover art display path (present in v0.2 but incomplete; see [Known issues](#known-issues)):

1. `meck_audio_ui_show_player` is called when a track is opened.
2. It calls `load_album_art(g_current_file)`.
3. That helper strips the filename off the path, `opendir`s the parent, looks for `cover.{png,jpg}` / `folder.{png,jpg}` / `front.{png,jpg}` / `album.{png,jpg}` (case-insensitive).
4. On match, it builds an LVGL FS path `A:/sdcard/...` and pre-flights with `lv_image_decoder_get_info`. If no decoder claims the source (no LODEPNG/TJPGD/etc), it leaves the placeholder music-note icon visible.
5. On decoder success, it lazy-creates an `lv_image` widget inside the existing 280x280 `obj_cover_slot` and sets the src. The widget is reused across track changes; only the src is swapped.

Step 4 passes for any reasonable PNG (the header parse is cheap), but step 5 fails for covers larger than roughly 600x600 because LODEPNG's decoded framebuffer can't be allocated from LVGL's heap. The v0.3+ fix needs either source pre-scaling (read dims via the pre-flight, scale at decode) or a streaming decoder.

The MP3 decode path:

1. `MeckAudio` task pumps data via chmorgan's `esp-audio-player` library, which wraps libhelix-mp3.
2. Our patch in `audio_mp3.cpp` runs once per file open: reads the 10-byte ID3v2 header, parses the synchsafe size from bytes 6-9, honours the footer flag (bit 4 of flags byte 5), and seeks past the tag body before libhelix sees any of it.
3. The es8311 codec receives PCM via `meck_es8311_write_data` with a vTaskDelay yield every call as belt-and-braces against IDLE-task starvation on long decodes.

The relevant sdkconfig settings for the audio + cover workflow:

```
CONFIG_LV_USE_FS_STDIO=y
CONFIG_LV_FS_STDIO_LETTER=65
CONFIG_LV_USE_LODEPNG=y
# CONFIG_LV_USE_TJPGD is not set         <-- keep this off
# CONFIG_LV_USE_LIBJPEG_TURBO is not set
```

If you want JPG support and you're prepared to deal with the consequences, the right knob is `CONFIG_LV_USE_LIBJPEG_TURBO=y` plus the libjpeg-turbo managed component, not `CONFIG_LV_USE_TJPGD=y`.

The chmorgan audio-player patch is fragile: `idf.py fullclean` or a managed-component version bump will wipe it. The durable fix is to vendor `chmorgan__esp-audio-player` into `components/` so it sits under version control alongside the rest of the firmware. The script-side cleanup via `mp3_clean.py` is also a durable workaround independent of the firmware patch.

## Acknowledgements

The `mp3_clean.py` script depends on [`mutagen`](https://mutagen.readthedocs.io/) and optionally [`Pillow`](https://pillow.readthedocs.io/). The audio backend uses [`chmorgan/esp-audio-player`](https://github.com/chmorgan/esp-audio-player) wrapping [`libhelix-mp3`](https://github.com/ultraembedded/libhelix-mp3). LVGL filesystem and decoder support is from the [LVGL ESP-IDF component](https://github.com/lvgl/lvgl_esp32_drivers).
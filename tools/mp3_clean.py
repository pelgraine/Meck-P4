#!/usr/bin/env python3
"""
mp3_clean.py — strip oversized embedded album art from MP3 files for use
with the Meck-P4 audio player.

Background:
    libhelix-mp3 (used by chmorgan/esp-audio-player) scans byte-by-byte
    for the 11-bit MPEG sync pattern 0xFFE. JPEG bytes inside ID3v2 APIC
    frames are full of false matches, each triggering a failed MP3Decode.
    On files with multi-MB embedded art the scan starves IDLE on the
    audio core and fires the FreeRTOS task watchdog.

    The Meck firmware patch in audio_mp3.cpp skips the ID3v2 tag entirely
    on the first decode call, which fixes playback. But large tags still
    waste flash on the SD card, and downstream tools that don't have the
    patch (your laptop, other devices) still see the slow scan. So this
    script is a complement: clean up the files at the source.

Behaviour:
    Walks a directory tree, finds every .mp3 file, inspects the ID3v2
    APIC (Attached Picture) frames, and:
      - Drops APIC frames whose image data exceeds --max-art-bytes
        (default 64 KB — large enough for a decent 300×300 cover JPEG,
        small enough to keep playback startup snappy)
      - Optionally exports the dropped art alongside the album folder as
        cover.jpg so you can keep it for future use
      - Leaves the rest of the ID3v2 tag (title, artist, album, year,
        track number, etc) untouched
      - Files with no APIC frames or with already-small APIC frames are
        skipped entirely — no rewrite, no mtime change

Usage:
    ./mp3_clean.py /path/to/music/library
    ./mp3_clean.py --dry-run /path/to/music/library
    ./mp3_clean.py --max-art-bytes 32768 /path/to/music/library
    ./mp3_clean.py --export-cover /path/to/music/library

Dependencies:
    pip install mutagen

    mutagen is the standard Python library for reading/writing audio
    metadata. It handles ID3v2.2/2.3/2.4, synchsafe integers, unsynchronisation,
    and all the other places hand-rolled parsers fall over.

Tested against ID3v2.3 and ID3v2.4 tags. ID3v2.2 is rare in modern files
and mutagen handles it too if it shows up.
"""

import argparse
import sys
from pathlib import Path

try:
    from mutagen.id3 import ID3, APIC, ID3NoHeaderError
    from mutagen.mp3 import MP3, HeaderNotFoundError
except ImportError:
    sys.stderr.write(
        "error: mutagen not installed. Run:\n"
        "    pip install mutagen\n"
        "or, in your venv-tiles environment:\n"
        "    source ~/venv-tiles/bin/activate && pip install mutagen\n"
    )
    sys.exit(2)

# Pillow is optional. If present, exported covers are converted to PNG
# (which works with LVGL's LODEPNG decoder out of the box, unlike JPG
# which requires the TJPGD decoder — and TJPGD blows ESP-IDF's main
# task stack on first call, so PNG is the safer default). If Pillow
# is absent, fall back to writing the raw image bytes as-is with the
# format-appropriate extension.
try:
    from PIL import Image
    from io import BytesIO
    HAVE_PILLOW = True
except ImportError:
    HAVE_PILLOW = False


def human_bytes(n):
    """Format byte count for log output."""
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024:
            return f"{n:.1f} {unit}" if unit != "B" else f"{n} B"
        n /= 1024
    return f"{n:.1f} TB"


def pick_largest_apic(tags):
    """Return (key, APIC_frame) of the largest APIC, or (None, None)."""
    apic_frames = [(k, v) for k, v in tags.items() if k.startswith("APIC")]
    if not apic_frames:
        return None, None
    return max(apic_frames, key=lambda kv: len(kv[1].data))


def cover_path_for(mp3_path: Path, prefer_png: bool) -> Path:
    """Where we'd export cover art — alongside the mp3 file."""
    ext = ".png" if prefer_png else ".jpg"
    parent = mp3_path.parent
    siblings = list(parent.glob("*.mp3"))
    if len(siblings) > 1:
        # Album folder — use generic cover.{png,jpg}
        return parent / f"cover{ext}"
    else:
        # Standalone file — use track-specific name
        return parent / f"{mp3_path.stem}.cover{ext}"


def extension_for(mime: str) -> str:
    """Pick a file extension based on the APIC mime type."""
    if not mime:
        return ".jpg"
    mime = mime.lower()
    if "png" in mime:
        return ".png"
    if "jpeg" in mime or "jpg" in mime:
        return ".jpg"
    if "gif" in mime:
        return ".gif"
    return ".bin"  # unknown, but at least preserve it


def clean_one_file(mp3_path: Path, max_art_bytes: int,
                   dry_run: bool, export_cover: bool, prefer_png: bool):
    """Process one .mp3 file. Returns a tuple (status, before, after) where
    status is 'cleaned', 'skipped-small', 'skipped-no-art', 'error'."""
    try:
        tags = ID3(mp3_path)
    except ID3NoHeaderError:
        return ("skipped-no-art", 0, 0)
    except Exception as e:
        sys.stderr.write(f"  ERROR reading tags from {mp3_path}: {e}\n")
        return ("error", 0, 0)

    key, apic = pick_largest_apic(tags)
    if apic is None:
        return ("skipped-no-art", 0, 0)

    art_size = len(apic.data)
    if art_size <= max_art_bytes:
        return ("skipped-small", art_size, art_size)

    # We're going to drop this APIC frame. Optionally save it first.
    if export_cover and not dry_run:
        # Prefer PNG output if requested and Pillow is available, since
        # the firmware decodes PNG (LODEPNG) but enabling TJPGD for JPG
        # blows the main task stack on boot. If we can't convert (no
        # Pillow), fall back to writing the raw bytes with the source
        # extension and leave conversion to the user.
        want_png = prefer_png and HAVE_PILLOW
        cover_target = cover_path_for(mp3_path, prefer_png=want_png)

        # If user asked for PNG but Pillow is missing, downgrade to the
        # source format silently rather than writing the wrong bytes
        # under a .png name.
        if not want_png:
            src_ext = extension_for(apic.mime)
            if cover_target.suffix != src_ext:
                cover_target = cover_target.with_suffix(src_ext)

        if not cover_target.exists():
            try:
                if want_png:
                    # Decode source via Pillow, re-encode as PNG.
                    img = Image.open(BytesIO(apic.data))
                    # Optionally downscale to keep decode fast on-device.
                    # Anything above 600px wastes RAM on a 280-px slot.
                    max_dim = 600
                    if max(img.size) > max_dim:
                        img.thumbnail((max_dim, max_dim), Image.LANCZOS)
                    img.save(cover_target, "PNG", optimize=True)
                else:
                    cover_target.write_bytes(apic.data)
            except Exception as e:
                sys.stderr.write(f"  WARN couldn't write {cover_target}: {e}\n")
        # If a cover already exists in the folder, don't overwrite — the
        # first track in the folder wins. Subsequent tracks drop art
        # silently.

    if dry_run:
        return ("cleaned", art_size, 0)

    # Drop ALL APIC frames, not just the largest — if one is oversized,
    # the others tend to be redundant copies (front cover, back cover,
    # icon, etc) and they all contribute to the sync-word scan time.
    apic_keys = [k for k in list(tags.keys()) if k.startswith("APIC")]
    for k in apic_keys:
        del tags[k]

    try:
        tags.save(mp3_path)
    except Exception as e:
        sys.stderr.write(f"  ERROR saving {mp3_path}: {e}\n")
        return ("error", art_size, art_size)

    # Read back the new file size of the APIC section by re-loading
    after_tags = ID3(mp3_path)
    _, after_apic = pick_largest_apic(after_tags)
    after_size = len(after_apic.data) if after_apic else 0

    return ("cleaned", art_size, after_size)


def main():
    ap = argparse.ArgumentParser(
        description="Strip oversized embedded album art from MP3 files."
    )
    ap.add_argument("path", type=Path,
                    help="Directory to walk (recursively) for .mp3 files.")
    ap.add_argument("--max-art-bytes", type=int, default=64 * 1024,
                    help="APIC frames larger than this are dropped. "
                         "Default: 65536 (64 KB). Use 0 to drop all art.")
    ap.add_argument("--dry-run", action="store_true",
                    help="Report what would be done; don't modify any files.")
    ap.add_argument("--export-cover", action="store_true",
                    help="Save the largest dropped image as cover.png (or "
                         ".jpg) in the album folder (first track wins; "
                         "existing covers are not overwritten).")
    ap.add_argument("--png", dest="prefer_png", action="store_true",
                    default=True,
                    help="Convert exported covers to PNG (default; requires "
                         "Pillow). Recommended because LVGL's LODEPNG decoder "
                         "is enabled in firmware while TJPGD is not.")
    ap.add_argument("--no-png", dest="prefer_png", action="store_false",
                    help="Write exported covers in their source format "
                         "(usually JPG) without conversion.")
    args = ap.parse_args()

    if not args.path.exists():
        sys.stderr.write(f"error: path does not exist: {args.path}\n")
        sys.exit(1)

    if args.path.is_file():
        mp3_files = [args.path] if args.path.suffix.lower() == ".mp3" else []
    else:
        mp3_files = sorted(args.path.rglob("*.mp3"))

    if not mp3_files:
        print(f"No .mp3 files found under {args.path}")
        return

    mode = "DRY RUN — no changes will be written" if args.dry_run else "cleaning"
    print(f"{mode}: {len(mp3_files)} mp3 files under {args.path}")
    print(f"Drop threshold: {human_bytes(args.max_art_bytes)}")
    if args.export_cover:
        if args.prefer_png and HAVE_PILLOW:
            print("Exporting dropped covers as cover.png per album folder "
                  "(converted via Pillow, downscaled to 600px max)")
        elif args.prefer_png and not HAVE_PILLOW:
            print("Note: --png requested but Pillow not installed. Falling "
                  "back to source format. To enable PNG conversion:")
            print("    pip install Pillow")
            print("Exporting dropped covers in source format per album folder")
        else:
            print("Exporting dropped covers in source format per album folder")
    print()

    counts = {"cleaned": 0, "skipped-small": 0, "skipped-no-art": 0, "error": 0}
    bytes_saved = 0

    for f in mp3_files:
        try:
            rel = f.relative_to(args.path)
        except ValueError:
            rel = f
        status, before, after = clean_one_file(
            f, args.max_art_bytes, args.dry_run, args.export_cover,
            args.prefer_png)
        counts[status] += 1

        if status == "cleaned":
            saved = before - after
            bytes_saved += saved
            arrow = "would drop" if args.dry_run else "dropped"
            print(f"  {arrow} {human_bytes(before)} from {rel}")
        elif status == "error":
            print(f"  ERROR on {rel}")
        # skipped-small and skipped-no-art are silent — keeps output focused
        # on what actually changed.

    print()
    print(f"Summary:")
    print(f"  {counts['cleaned']} files {'would be ' if args.dry_run else ''}cleaned")
    print(f"  {counts['skipped-small']} files left alone (art already under threshold)")
    print(f"  {counts['skipped-no-art']} files had no embedded art")
    if counts["error"]:
        print(f"  {counts['error']} files had errors (see above)")
    print(f"  Total bytes {'that would be ' if args.dry_run else ''}saved: "
          f"{human_bytes(bytes_saved)}")


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""Bake Twemoji PNGs into Meck-P4's LVGL image-font sources.

Usage:
  python3 tools/bake_p4_emoji.py tools/p4_emoji_registry.txt <twemoji 72x72 dir> components/meshcore

Reads the registry (one "<name> <hex-seq> [asset]" per line, no fe0f; two
regional indicators = a flag pair; ZWJ sequences are rejected as codepoints),
loads each Twemoji 72x72 PNG (jdecked/twemoji, assets/72x72/<seq>.png, or
<asset>.png when the optional third field names a different picture for that
codepoint), scales it with Lanczos to each size in SIZES and writes RGB565A8
LVGL 9 image descriptors:

  meck_emoji_<size>.c   one file per size, one image per registry entry
  meck_emoji_gen.h      externs, counts and table declarations
  meck_emoji_tables.c   MECK_EMOJI_SIZES / _CP / _IMG and the flag-pair tables

meck_emoji_blank.c is static and not touched. The script refuses to write
anything if an asset is missing, so a bad asset path cannot produce an empty
table. Requires Pillow (python3 -m pip install --user pillow).
"""
import os, sys
from PIL import Image

SIZES = [14, 16, 18, 22, 24, 28, 30, 32]
RI_LO, RI_HI = 0x1F1E6, 0x1F1FF


def read_registry(path):
    singles, pairs = [], []
    seen = set()
    for ln, raw in enumerate(open(path, encoding='utf-8'), 1):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 2:
            sys.exit(f"{path}:{ln}: expected '<name> <hex-seq>'")
        name, seq = parts[0], parts[1].lower()
        asset = parts[2].lower() if len(parts) >= 3 else seq   # optional picture override
        if not all(c.isalnum() or c == '_' for c in name):
            sys.exit(f"{path}:{ln}: bad name '{name}' (letters, digits, underscore only)")
        if name in seen:
            sys.exit(f"{path}:{ln}: duplicate name '{name}'")
        seen.add(name)
        cps = [int(p, 16) for p in seq.split('-')]
        if len(cps) == 1:
            singles.append((name, cps[0], asset))
        elif len(cps) == 2 and all(RI_LO <= c <= RI_HI for c in cps):
            pairs.append((name, cps[0], cps[1], asset))
        else:
            sys.exit(f"{path}:{ln}: '{seq}' is not a single codepoint or a regional-indicator pair "
                     f"(ZWJ sequences are not supported)")
    return singles, pairs


def asset_path(assets, seq):
    return os.path.join(assets, seq + '.png')


def bake(png, size):
    """Return the RGB565A8 byte payload: w*h*2 bytes of little-endian RGB565
    followed by w*h bytes of alpha (LVGL 9 LV_COLOR_FORMAT_RGB565A8)."""
    im = Image.open(png).convert('RGBA').resize((size, size), Image.LANCZOS)
    rgb, alpha = bytearray(), bytearray()
    for r, g, b, a in im.getdata():
        v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
        rgb += bytes((v & 0xFF, v >> 8))
        alpha.append(a)
    return bytes(rgb), bytes(alpha)


def emit_map(out, name, size, rgb, alpha):
    upper = f"LV_ATTRIBUTE_EMOJI_{name.upper()}_{size}"
    out.append("")
    out.append("#ifndef LV_ATTRIBUTE_MEM_ALIGN")
    out.append("#define LV_ATTRIBUTE_MEM_ALIGN")
    out.append("#endif")
    out.append("")
    out.append(f"#ifndef {upper}")
    out.append(f"#define {upper}")
    out.append("#endif")
    out.append("")
    out.append("static const")
    out.append(f"LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST {upper}")
    out.append(f"uint8_t emoji_{name}_{size}_map[] = {{")
    out.append("")
    row = size * 2
    for i in range(0, len(rgb), row):
        out.append("    " + ",".join(f"0x{b:02x}" for b in rgb[i:i + row]) + ",")
    for i in range(0, len(alpha), size):
        out.append("    " + ",".join(f"0x{b:02x}" for b in alpha[i:i + size]) + ",")
    out.append("")
    out.append("};")
    out.append("")
    out.append(f"const lv_image_dsc_t emoji_{name}_{size} = {{")
    out.append("  .header.magic = LV_IMAGE_HEADER_MAGIC,")
    out.append("  .header.cf = LV_COLOR_FORMAT_RGB565A8,")
    out.append("  .header.flags = 0,")
    out.append(f"  .header.w = {size},")
    out.append(f"  .header.h = {size},")
    out.append(f"  .header.stride = {size * 2},")
    out.append(f"  .data_size = sizeof(emoji_{name}_{size}_map),")
    out.append(f"  .data = emoji_{name}_{size}_map,")
    out.append("};")
    out.append("")


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    reg, assets, outdir = sys.argv[1:4]
    singles, pairs = read_registry(reg)
    entries = [(n, seq) for n, _, seq in singles] + [(n, seq) for n, _, _, seq in pairs]
    missing = [seq for _, seq in entries if not os.path.isfile(asset_path(assets, seq))]
    if missing:
        sys.exit("Missing Twemoji assets (nothing written):\n  " + "\n  ".join(missing))

    # Bake everything first; write nothing until all sizes succeeded.
    files = {}
    for size in SIZES:
        out = [f"/* Generated: Twemoji (jdecked) emoji at {size}px, RGB565A8, LVGL 9.2. */",
               "/* Graphics (c) Twitter/Twemoji contributors, CC BY 4.0. */",
               "",
               '#include "lvgl.h"',
               ""]
        for name, seq in entries:
            rgb, alpha = bake(asset_path(assets, seq), size)
            emit_map(out, name, size, rgb, alpha)
        files[f"meck_emoji_{size}.c"] = "\n".join(out) + "\n"

    h = ["/* Generated declarations for Meck emoji assets. Do not edit by hand. */",
         "#ifndef MECK_EMOJI_GEN_H",
         "#define MECK_EMOJI_GEN_H",
         '#include "lvgl.h"',
         "#ifdef __cplusplus",
         'extern "C" {',
         "#endif",
         ""]
    for size in SIZES:
        for name, _ in entries:
            h.append(f"extern const lv_image_dsc_t emoji_{name}_{size};")
    h.append("extern const lv_image_dsc_t emoji_blank;")
    h.append("")
    h.append(f"#define MECK_EMOJI_SIZE_COUNT {len(SIZES)}")
    h.append(f"#define MECK_EMOJI_COUNT {len(singles)}")
    h.append(f"#define MECK_EMOJI_PAIR_COUNT {len(pairs)}")
    h.append("extern const uint16_t MECK_EMOJI_SIZES[MECK_EMOJI_SIZE_COUNT];")
    h.append("extern const uint32_t MECK_EMOJI_CP[MECK_EMOJI_COUNT];")
    h.append("extern const lv_image_dsc_t * const MECK_EMOJI_IMG[MECK_EMOJI_SIZE_COUNT][MECK_EMOJI_COUNT];")
    h.append("/* Regional-indicator pairs (flags): first and second codepoint of each. */")
    h.append("extern const uint32_t MECK_EMOJI_PAIR_CP[MECK_EMOJI_PAIR_COUNT][2];")
    h.append("extern const lv_image_dsc_t * const MECK_EMOJI_PAIR_IMG[MECK_EMOJI_SIZE_COUNT][MECK_EMOJI_PAIR_COUNT];")
    h += ["", "#ifdef __cplusplus", "}", "#endif", "#endif /* MECK_EMOJI_GEN_H */", ""]
    files["meck_emoji_gen.h"] = "\n".join(h)

    t = ["/* Generated lookup tables for Meck emoji assets. Do not edit by hand. */",
         '#include "lvgl.h"',
         '#include "meck_emoji_gen.h"',
         "",
         "const uint16_t MECK_EMOJI_SIZES[MECK_EMOJI_SIZE_COUNT] = { " + ", ".join(str(s) for s in SIZES) + " };",
         "",
         "const uint32_t MECK_EMOJI_CP[MECK_EMOJI_COUNT] = {"]
    for name, cp, _ in singles:
        t.append(f"    0x{cp:X}, /* {name} */")
    t.append("};")
    t.append("")
    t.append("const lv_image_dsc_t * const MECK_EMOJI_IMG[MECK_EMOJI_SIZE_COUNT][MECK_EMOJI_COUNT] = {")
    for size in SIZES:
        t.append("    { " + ", ".join(f"&emoji_{n}_{size}" for n, _, _ in singles) + " },")
    t.append("};")
    t.append("")
    t.append("const uint32_t MECK_EMOJI_PAIR_CP[MECK_EMOJI_PAIR_COUNT][2] = {")
    for name, a, b, _ in pairs:
        t.append(f"    {{ 0x{a:X}, 0x{b:X} }}, /* {name} */")
    t.append("};")
    t.append("")
    t.append("const lv_image_dsc_t * const MECK_EMOJI_PAIR_IMG[MECK_EMOJI_SIZE_COUNT][MECK_EMOJI_PAIR_COUNT] = {")
    for size in SIZES:
        t.append("    { " + ", ".join(f"&emoji_{n}_{size}" for n, _, _, _ in pairs) + " },")
    t.append("};")
    t.append("")
    files["meck_emoji_tables.c"] = "\n".join(t)

    os.makedirs(outdir, exist_ok=True)
    for fn, text in files.items():
        with open(os.path.join(outdir, fn), "w", encoding="ascii", newline="\n") as f:
            f.write(text)
    print(f"{len(singles)} emoji + {len(pairs)} flag pairs at {len(SIZES)} sizes -> {outdir}")


if __name__ == "__main__":
    main()

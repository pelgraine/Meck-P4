// MeckEmoji.h
//
// Inline colour-emoji rendering for Meck-P4. Registers an LVGL image-font
// (lv_imgfont) per text size and attaches it as the fallback of the matching
// meck_montserrat_* font, so emoji codepoints the Montserrat fonts can't draw
// are rendered from baked-in Twemoji images instead of the missing-glyph box.
//
// Requires CONFIG_LV_USE_IMGFONT=y (menuconfig).

#ifndef MECK_EMOJI_H
#define MECK_EMOJI_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Create the per-size emoji image-fonts and wire each one as the fallback of
// the matching meck_montserrat_* font. Call once from meck_ui_init() after
// LVGL is up, before any text is drawn.
void meck_emoji_init(void);

// True if cp is a codepoint Meck can render as an emoji, so strip_unrenderable
// keeps it instead of dropping it. Covers the 77 single-codepoint emoji, the
// AU flag's two regional indicators (U+1F1E6, U+1F1FA), and the VS-16 emoji
// presentation selector (U+FE0F).
bool meck_emoji_is_renderable(uint32_t cp);

#ifdef __cplusplus
}
#endif

#endif // MECK_EMOJI_H

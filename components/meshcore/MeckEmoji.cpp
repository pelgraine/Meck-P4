// MeckEmoji.cpp
//
// See MeckEmoji.h. Requires CONFIG_LV_USE_IMGFONT=y.

#include <stdint.h>
#include "lvgl.h"
#include "MeckEmoji.h"
#include "meck_emoji_gen.h"

// The Montserrat text fonts. Declared non-const here because their
// definitions in meck_montserrat_*.c now drop the const qualifier, so that
// meck_emoji_init() can set each font's .fallback to its matching-size emoji
// image-font at runtime.
extern "C" {
    extern lv_font_t meck_montserrat_14;
    extern lv_font_t meck_montserrat_16;
    extern lv_font_t meck_montserrat_18;
    extern lv_font_t meck_montserrat_22;
    extern lv_font_t meck_montserrat_24;
    extern lv_font_t meck_montserrat_28;
    extern lv_font_t meck_montserrat_30;
    extern lv_font_t meck_montserrat_32;
}

#define VS16  0xFE0F    // variation selector-16 (emoji presentation)
#define RI_A  0x1F1E6   // regional indicator A   (AU flag = A + U)
#define RI_U  0x1F1FA   // regional indicator U

// imgfont path callback. Maps a codepoint to its image at the size bound to
// this font instance (the size index is passed as user_data). Returns NULL
// for anything unsupported, letting LVGL continue down the font chain.
//
// LVGL advances one codepoint at a time and does not consume unicode_next on
// our behalf, so trailing combiners (VS-16, the second regional indicator)
// are mapped to a 1px transparent spacer to keep them from rendering as a
// box after the base emoji.
static const void* meck_emoji_path_cb(const lv_font_t* font, uint32_t unicode,
                                      uint32_t unicode_next, int32_t* offset_y,
                                      void* user_data) {
    LV_UNUSED(font);
    int si = (int)(intptr_t)user_data;
    *offset_y = 0;

    if (unicode == VS16 || unicode == RI_U) return &emoji_blank;

    if (unicode == RI_A) {
        if (unicode_next == RI_U) return MECK_EMOJI_FLAG_AU[si];
        return NULL;
    }

    for (int j = 0; j < MECK_EMOJI_COUNT; j++) {
        if (MECK_EMOJI_CP[j] == unicode) return MECK_EMOJI_IMG[si][j];
    }
    return NULL;
}

static lv_font_t* g_emoji_font[MECK_EMOJI_SIZE_COUNT];

static lv_font_t* meck_font_for_size(uint16_t px) {
    switch (px) {
        case 14: return &meck_montserrat_14;
        case 16: return &meck_montserrat_16;
        case 18: return &meck_montserrat_18;
        case 22: return &meck_montserrat_22;
        case 24: return &meck_montserrat_24;
        case 28: return &meck_montserrat_28;
        case 30: return &meck_montserrat_30;
        case 32: return &meck_montserrat_32;
        default: return NULL;
    }
}

void meck_emoji_init(void) {
    for (int i = 0; i < MECK_EMOJI_SIZE_COUNT; i++) {
        uint16_t px = MECK_EMOJI_SIZES[i];
        g_emoji_font[i] = lv_imgfont_create(px, meck_emoji_path_cb,
                                            (void*)(intptr_t)i);
        meck_font_for_size(px)->fallback = g_emoji_font[i];
    }
}

bool meck_emoji_is_renderable(uint32_t cp) {
    if (cp == VS16) return true;
    for (int j = 0; j < MECK_EMOJI_COUNT; j++) {
        if (MECK_EMOJI_CP[j] == cp) return true;
    }
    return false;
}
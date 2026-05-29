// MeckEmojiPicker.cpp
//
// See MeckEmojiPicker.h. Requires CONFIG_LV_USE_IMGFONT=y (the cells render
// emoji through the meck_montserrat_* fallback wired by meck_emoji_init()).

#include "lvgl.h"
#include "MeckEmojiPicker.h"
#include "meck_emoji_gen.h"

// Cells render through a Montserrat font so its emoji fallback kicks in.
extern "C" {
    extern lv_font_t meck_montserrat_28;
}

// Pickable set: the 77 single-codepoint emoji. (The AU flag, a two-codepoint
// sequence, was removed; it did not render reliably on-device.)
#define MECK_PICKER_TOTAL MECK_EMOJI_COUNT

// UTF-8 strings for each pickable emoji, built once. 12 bytes is ample for
// any single codepoint (max 4 bytes) plus a terminator.
static char g_utf8[MECK_PICKER_TOTAL][12];
static bool g_built = false;

static lv_obj_t *g_overlay = NULL;
static lv_obj_t *g_panel   = NULL;
static lv_obj_t *g_kb      = NULL;
static lv_obj_t *g_ta      = NULL;

static int meck_utf8_encode(uint32_t cp, char *out) {
    if (cp < 0x80) {
        out[0] = (char)cp;
        return 1;
    } else if (cp < 0x800) {
        out[0] = (char)(0xC0 | (cp >> 6));
        out[1] = (char)(0x80 | (cp & 0x3F));
        return 2;
    } else if (cp < 0x10000) {
        out[0] = (char)(0xE0 | (cp >> 12));
        out[1] = (char)(0x80 | ((cp >> 6) & 0x3F));
        out[2] = (char)(0x80 | (cp & 0x3F));
        return 3;
    } else {
        out[0] = (char)(0xF0 | (cp >> 18));
        out[1] = (char)(0x80 | ((cp >> 12) & 0x3F));
        out[2] = (char)(0x80 | ((cp >> 6) & 0x3F));
        out[3] = (char)(0x80 | (cp & 0x3F));
        return 4;
    }
}

static void meck_picker_build_list(void) {
    if (g_built) return;
    for (int i = 0; i < MECK_EMOJI_COUNT; i++) {
        int n = meck_utf8_encode(MECK_EMOJI_CP[i], g_utf8[i]);
        g_utf8[i][n] = '\0';
    }
    g_built = true;
}

bool meck_emoji_picker_is_open(void) { return g_overlay != NULL; }

void meck_emoji_picker_close(void) {
    if (g_kb) {
        lv_obj_remove_state(g_kb, LV_STATE_DISABLED);
        g_kb = NULL;
    }
    if (g_panel) {
        lv_obj_delete(g_panel);
        g_panel = NULL;
    }
    if (g_overlay) {
        lv_obj_delete(g_overlay);
        g_overlay = NULL;
    }
    g_ta = NULL;
}

// Tap landed on the modal overlay (outside the panel): dismiss.
static void on_picker_overlay_pressed(lv_event_t *e) {
    LV_UNUSED(e);
    meck_emoji_picker_close();
}

// Tap landed on an emoji cell: insert its UTF-8 and dismiss.
static void on_picker_cell_clicked(lv_event_t *e) {
    const char *utf8 = (const char *)lv_event_get_user_data(e);
    if (utf8 && g_ta) lv_textarea_add_text(g_ta, utf8);
    meck_emoji_picker_close();
}

void meck_emoji_picker_open(lv_obj_t *kb, lv_obj_t *ta, bool dark) {
    meck_emoji_picker_close();
    meck_picker_build_list();

    lv_obj_t *scr = lv_screen_active();
    int32_t sw = lv_obj_get_width(scr);
    int32_t sh = lv_obj_get_height(scr);

    // Full-screen transparent modal overlay. Tap outside the panel closes.
    g_overlay = lv_obj_create(scr);
    lv_obj_remove_style_all(g_overlay);
    lv_obj_set_size(g_overlay, sw, sh);
    lv_obj_set_pos(g_overlay, 0, 0);
    lv_obj_add_flag(g_overlay, LV_OBJ_FLAG_CLICKABLE);
    // Don't let picker taps pull focus off the composer textarea, or the
    // keyboard's focus/defocus layout gets out of sync (composer left raised).
    lv_obj_remove_flag(g_overlay, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_add_event_cb(g_overlay, on_picker_overlay_pressed,
                        LV_EVENT_CLICKED, NULL);

    // Centred panel holding a scrollable wrapped grid of emoji cells.
    g_panel = lv_obj_create(scr);
    lv_obj_set_size(g_panel, (sw * 90) / 100, (sh * 60) / 100);
    lv_obj_center(g_panel);
    lv_obj_add_flag(g_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(g_panel, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_set_flex_flow(g_panel, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(g_panel, LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_set_scroll_dir(g_panel, LV_DIR_VER);
    lv_obj_set_style_pad_all(g_panel, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_row(g_panel, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_column(g_panel, 6, LV_PART_MAIN);
    lv_obj_set_style_radius(g_panel, 10, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(g_panel, 16, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(g_panel, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(g_panel, LV_OPA_60, LV_PART_MAIN);
    // Larger, always-visible scrollbar (easier to grab given the touch panel).
    lv_obj_set_scrollbar_mode(g_panel, LV_SCROLLBAR_MODE_ON);
    lv_obj_set_style_width(g_panel, 12, LV_PART_SCROLLBAR);
    lv_obj_set_style_pad_right(g_panel, 4, LV_PART_SCROLLBAR);
    lv_obj_set_style_radius(g_panel, 6, LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_color(g_panel, lv_color_make(130, 130, 140),
                              LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(g_panel, LV_OPA_70, LV_PART_SCROLLBAR);
    if (dark) {
        lv_obj_set_style_bg_color(g_panel, lv_color_make(15, 15, 20),
                                  LV_PART_MAIN);
        lv_obj_set_style_border_color(g_panel, lv_color_make(60, 60, 75),
                                      LV_PART_MAIN);
        lv_obj_set_style_border_width(g_panel, 1, LV_PART_MAIN);
    }

    for (int i = 0; i < MECK_PICKER_TOTAL; i++) {
        lv_obj_t *cell = lv_button_create(g_panel);
        lv_obj_remove_flag(cell, LV_OBJ_FLAG_CLICK_FOCUSABLE);
        lv_obj_set_size(cell, 56, 56);
        lv_obj_set_style_radius(cell, 6, LV_PART_MAIN);
        if (dark) {
            lv_obj_set_style_bg_color(cell, lv_color_make(45, 45, 55),
                                      LV_PART_MAIN);
            lv_obj_set_style_bg_color(cell, lv_color_make(80, 80, 95),
                                      LV_PART_MAIN | LV_STATE_PRESSED);
        }
        lv_obj_t *lbl = lv_label_create(cell);
        lv_obj_set_style_text_font(lbl, &meck_montserrat_28, LV_PART_MAIN);
        lv_label_set_text(lbl, g_utf8[i]);
        lv_obj_center(lbl);
        lv_obj_add_event_cb(cell, on_picker_cell_clicked, LV_EVENT_CLICKED,
                            g_utf8[i]);
    }

    if (kb) {
        g_kb = kb;
        lv_obj_add_state(kb, LV_STATE_DISABLED);
    }
    g_ta = ta;
}
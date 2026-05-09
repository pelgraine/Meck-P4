/*
 * MeckUI.cpp — LVGL UI for Meck on T-Display P4
 *
 * Tileview tiles (swipe horizontally from home):
 *   - Tile 0: Home (title, freq header, MSG/RX/RSSI counters, 3x2 nav grid)
 *   - Tile 1: Recent Heard (live list of received adverts)
 *   - Tile 2: Radio Details (current freq/BW/SF/CR/TX/sync word)
 *   - Tile 3: Advert (long-press to send manual advert)
 *   - Tile 4: GPS (status/fix/sats/position/altitude + last NMEA sentences)
 *   - Tile 5: Battery Gauge (placeholder, awaiting accessor wiring)
 *   - Tile 6: Hibernate (long-press handler comes in B.5)
 *
 * Sub-screens reached from home grid:
 *   - Settings (editable: name, preset, TX power, UTC; identity read-only)
 *     - Radio Preset picker (17 community presets)
 *     - Name edit overlay (textarea + virtual keyboard)
 *   - Channel Picker (color-coded list, unread badges, delete, add)
 *     - Add Channel overlay (textarea + virtual keyboard)
 *     - Messages screen (per channel, compose + send via deferred queue)
 *   - Contacts (scrollable list, favourites, identity + path)
 *     - Contact Detail (full identity hex, path info, fav toggle)
 *
 * Threading
 * ---------
 * LilyGo runs LVGL on lvgl_ui_task. We coordinate via their lock
 * (_lock_t lvgl_api_lock, declared non-static in main.cpp).
 *
 * Radio config and message send are deferred to meck_task to avoid SPI
 * bus contention. See radio_request_reconfig and meck_request_send_text
 * in target.h. Read-only mesh accessors (getContactByIdx, getMessages,
 * getChannel, getNumContacts, etc) and persistence-only writes
 * (addHashChannel, deleteChannel) are safe to call directly from LVGL.
 */

#include "MeckUI.h"
#include "meck.h"
#include "target.h"
#include "MeckMesh.h"
#include "MeckDataStore.h"

#include <sys/lock.h>
#include "lvgl.h"
#include "t_display_p4_driver.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <climits>

extern _lock_t lvgl_api_lock;

#define NOTCH_SAFE_X  20
#define NOTCH_SAFE_Y  20

// Firmware identity, surfaced on the Settings screen. The home screen now
// shows the user's chosen node name instead.
#define MECK_FIRMWARE_NAME    "Meck P4"
#define MECK_FIRMWARE_VERSION "0.1"

// ============================================================================
// Radio preset table (17 community presets)
// ============================================================================

struct RadioPreset {
    const char* name;
    float freq;
    float bw;
    uint8_t sf;
    uint8_t cr;
    uint8_t tx_power;
};

static const RadioPreset RADIO_PRESETS[] = {
    { "Australia (Narrow)",       916.575f,  62.5f,  7, 8, 22 },
    { "Australia (Mid)",          915.075f, 125.0f,  9, 5, 22 },
    { "Australia: SA, WA",        923.125f,  62.5f,  8, 8, 22 },
    { "Australia: QLD",           923.125f,  62.5f,  8, 5, 22 },
    { "EU/UK (Narrow)",           869.618f,  62.5f,  8, 8, 14 },
    { "EU/UK (Deprecated)",       869.525f, 250.0f, 11, 5, 14 },
    { "Czech Republic (Narrow)",  869.432f,  62.5f,  7, 5, 14 },
    { "EU 433MHz (Long Range)",   433.650f, 250.0f, 11, 5, 10 },
    { "EU 433MHz (Narrow)",       433.650f,  62.5f,  8, 8, 10 },
    { "New Zealand",              917.375f, 250.0f, 11, 5, 22 },
    { "New Zealand (Narrow)",     917.375f,  62.5f,  7, 5, 22 },
    { "Portugal 433",             433.375f,  62.5f,  9, 6, 10 },
    { "Portugal 868",             869.618f,  62.5f,  7, 6, 14 },
    { "Switzerland",              869.618f,  62.5f,  8, 8, 14 },
    { "USA/Canada (Recommended)", 910.525f,  62.5f,  7, 5, 22 },
    { "Vietnam (Narrow)",         920.250f,  62.5f,  8, 5, 22 },
    { "Vietnam (Deprecated)",     920.250f, 250.0f, 11, 5, 22 },
};
#define NUM_RADIO_PRESETS (sizeof(RADIO_PRESETS) / sizeof(RADIO_PRESETS[0]))

static const uint8_t TX_POWER_OPTIONS[] = { 10, 14, 17, 20, 22 };
#define NUM_TX_POWER_OPTIONS 5

// Channel + sender colour cycle. Used by:
//   - channel picker bubble borders / text
//   - active channel name in the messages header
//   - home tile borders in Multi colour scheme
//   - per-sender message bubble bg + sender label
// Order: the first 6 entries match the original palette so existing
// channels keep their colour. Entries 6+ are added to widen the hash
// distribution for sender colours where collisions matter (hundreds
// of nodes means 6 buckets is far too coarse). Ordering also matters
// for the channel picker — adjacent indices are kept visually distinct
// so channels 0/1/2... don't look the same on the list.
static const lv_palette_t CH_COLORS[] = {
    LV_PALETTE_CYAN,         LV_PALETTE_ORANGE,
    LV_PALETTE_LIGHT_GREEN,  LV_PALETTE_PURPLE,
    LV_PALETTE_YELLOW,       LV_PALETTE_PINK,
    LV_PALETTE_BLUE,         LV_PALETTE_RED,
    LV_PALETTE_TEAL,         LV_PALETTE_AMBER,
    LV_PALETTE_DEEP_PURPLE,  LV_PALETTE_LIME,
    LV_PALETTE_INDIGO,       LV_PALETTE_DEEP_ORANGE,
    LV_PALETTE_LIGHT_BLUE,   LV_PALETTE_GREEN,
};
#define NUM_CH_COLORS ((int)(sizeof(CH_COLORS) / sizeof(CH_COLORS[0])))

// ============================================================================
// Home tile colour scheme. Stored in its own NVS namespace ("meckui") so we
// don't have to add a field to P4NodePrefs (which would change the on-disk
// schema for the prefs blob). Adding a new scheme is two changes: extend
// the enum and add a branch in get_tile_colors().
// ============================================================================
typedef enum {
    HOME_COLOR_PLAIN = 0,   // dark bg + grey-blue border, original look
    HOME_COLOR_MULTI = 1,   // dark bg + per-tile coloured border (CH_COLORS)
    HOME_COLOR_COUNT
} MeckHomeColor;

static int g_home_color_scheme = HOME_COLOR_PLAIN;

#define MECK_HOME_TILE_COUNT 10
static lv_obj_t *tile_buttons[MECK_HOME_TILE_COUNT] = {};
static int       tile_button_count = 0;

// ============================================================================
// LVGL objects (file-scope so the refresh timer can update them)
// ============================================================================

static lv_obj_t *scr_home          = NULL;
static lv_obj_t *g_tileview        = NULL;

// Home tile header labels
static lv_obj_t *lbl_home_title    = NULL;
static lv_obj_t *lbl_home_packets  = NULL;
static lv_obj_t *lbl_home_rssi     = NULL;
static lv_obj_t *lbl_home_unread   = NULL;

// Detail tile labels
static lv_obj_t *lbl_recent_list   = NULL;
static lv_obj_t *lbl_radio_detail  = NULL;
static lv_obj_t *lbl_advert_status = NULL;
static lv_obj_t *lbl_gps_detail    = NULL;
static lv_obj_t *lbl_battery_detail = NULL;

// Noise floor display cache. The estimator itself lives in P4SX1262Radio
// (sampled every 2 s under the SPI lock via SX126x GetRssiInst, opcode
// 0x15). We cache the last-displayed value here so we only redraw the
// radio detail label when it actually changes.
static int g_last_noise_floor_displayed = INT_MIN;

// Settings screen
static lv_obj_t *scr_settings      = NULL;
static lv_obj_t *lbl_set_name      = NULL;
static lv_obj_t *lbl_set_radio     = NULL;
static lv_obj_t *lbl_set_txpower   = NULL;
static lv_obj_t *lbl_set_utc       = NULL;
static lv_obj_t *lbl_set_pathhash  = NULL;
static lv_obj_t *lbl_set_homecolor = NULL;
static lv_obj_t *lbl_set_identity  = NULL;
static lv_obj_t *obj_name_edit_panel = NULL;
static lv_obj_t *ta_settings_name    = NULL;
static lv_obj_t *kb_settings         = NULL;

// Radio preset picker
static lv_obj_t *scr_radio_picker  = NULL;

// Channel picker
static lv_obj_t *scr_channel_picker  = NULL;
static lv_obj_t *obj_ch_picker_scroll = NULL;
static lv_obj_t *lbl_picker_unread[8] = {};
static lv_obj_t *obj_ch_add_panel  = NULL;
static lv_obj_t *ta_ch_add         = NULL;
static lv_obj_t *kb_ch_add         = NULL;

// Messages screen
static lv_obj_t *scr_messages           = NULL;
static lv_obj_t *lbl_msg_channel_name   = NULL;
static lv_obj_t *lbl_messages_body      = NULL;
static lv_obj_t *obj_msg_scroll         = NULL;
static lv_obj_t *ta_compose             = NULL;
static lv_obj_t *kb_compose             = NULL;
static lv_obj_t *btn_send               = NULL;
static uint8_t  g_active_channel        = 0;

// Contacts
static lv_obj_t *scr_contacts            = NULL;
static lv_obj_t *obj_contacts_scroll     = NULL;
static lv_obj_t *scr_contact_detail      = NULL;
static lv_obj_t *lbl_contact_detail_body = NULL;
static int g_selected_contact_idx        = -1;

// ============================================================================
// Forward declarations
// ============================================================================

static void goto_home(lv_event_t *e);
static void goto_settings(lv_event_t *e);
static void goto_channel_picker(lv_event_t *e);
static void goto_contacts(lv_event_t *e);
static void goto_contacts_from_detail(lv_event_t *e);
static void goto_channel_n(lv_event_t *e);
static void load_channel_view(uint8_t ch_idx);
static void rebuild_message_bubbles(uint8_t ch_idx);

static void settings_update_labels();
static void update_radio_detail_label();
static void refresh_channel_picker();
static void refresh_contacts_list();

static void on_settings_name_tap(lv_event_t *e);
static void on_settings_name_save(lv_event_t *e);
static void on_settings_name_cancel(lv_event_t *e);
static void on_settings_kb_event(lv_event_t *e);
static void on_settings_radio_tap(lv_event_t *e);
static void on_settings_txpower_tap(lv_event_t *e);
static void on_settings_utc_tap(lv_event_t *e);
static void on_radio_preset_select(lv_event_t *e);

static void on_send_clicked(lv_event_t *e);
static void on_compose_focused(lv_event_t *e);
static void on_compose_defocused(lv_event_t *e);
static void on_kb_event(lv_event_t *e);

static void on_contact_tap(lv_event_t *e);
static void on_contact_fav_toggle(lv_event_t *e);

static void on_ch_delete(lv_event_t *e);
static void on_ch_add_tap(lv_event_t *e);
static void on_ch_add_save(lv_event_t *e);
static void on_ch_add_cancel(lv_event_t *e);
static void on_ch_add_kb_event(lv_event_t *e);

// ============================================================================
// Home grid stub callbacks (for tiles that don't have ports yet)
// ============================================================================

static void cb_todo_reader(lv_event_t* e)   { printf("MeckUI: Reader tile clicked (TODO)\n"); }
static void cb_todo_notes(lv_event_t* e)    { printf("MeckUI: Notes tile clicked (TODO)\n"); }
static void cb_todo_discover(lv_event_t* e) { printf("MeckUI: Discover tile clicked (TODO)\n"); }
static void cb_todo_trace(lv_event_t* e)    { printf("MeckUI: Trace tile clicked (TODO)\n"); }
static void cb_todo_maps(lv_event_t* e)     { printf("MeckUI: Maps tile clicked (TODO)\n"); }
static void cb_todo_audio(lv_event_t* e)    { printf("MeckUI: Audio tile clicked (TODO)\n"); }
static void cb_todo_web(lv_event_t* e)      { printf("MeckUI: Web tile clicked (TODO)\n"); }

// ============================================================================
// Home tile colour scheme: NVS persistence + style application.
// Stored under namespace "meckui", key "home_color" as a uint8_t. nvs_open
// failures fall back silently to the default scheme (Plain).
// ============================================================================

static void load_home_color_from_nvs() {
    nvs_handle_t h;
    if (nvs_open("meckui", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t v = HOME_COLOR_PLAIN;
    if (nvs_get_u8(h, "home_color", &v) == ESP_OK && v < HOME_COLOR_COUNT) {
        g_home_color_scheme = v;
    }
    nvs_close(h);
}

static void save_home_color_to_nvs() {
    nvs_handle_t h;
    if (nvs_open("meckui", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "home_color", (uint8_t)g_home_color_scheme);
    nvs_commit(h);
    nvs_close(h);
}

// Returns the bg/border colours and border width for tile `idx` under the
// current scheme. Adding a new scheme = adding a branch here.
static void get_tile_colors(int idx, lv_color_t *bg, lv_color_t *border,
                            int *border_w) {
    switch (g_home_color_scheme) {
        case HOME_COLOR_MULTI: {
            // Match channel-bubble look: dark bg + 2 px coloured border per
            // tile, palette indexed off tile position so adjacent tiles
            // never share a colour.
            *bg       = lv_color_make(25, 25, 35);
            *border   = lv_palette_main(CH_COLORS[idx % NUM_CH_COLORS]);
            *border_w = 2;
            break;
        }
        case HOME_COLOR_PLAIN:
        default: {
            *bg       = lv_color_make(30, 30, 40);
            *border   = lv_color_make(80, 80, 100);
            *border_w = 1;
            break;
        }
    }
}

// Repaint every registered tile to match the current scheme. Cheap: just
// style setters, no re-layout.
static void apply_tile_colors() {
    for (int i = 0; i < tile_button_count; i++) {
        if (!tile_buttons[i]) continue;
        lv_color_t bg, border;
        int border_w;
        get_tile_colors(i, &bg, &border, &border_w);
        lv_obj_set_style_bg_color(tile_buttons[i], bg, 0);
        lv_obj_set_style_border_color(tile_buttons[i], border, 0);
        lv_obj_set_style_border_width(tile_buttons[i], border_w, 0);
    }
}

// ============================================================================
// Helpers (back button, tile button, settings row)
// ============================================================================

static lv_obj_t* create_back_button(lv_obj_t *parent) {
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_size(btn, 80, 40);
    lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn, 8, 0);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(lbl);

    lv_obj_add_event_cb(btn, goto_home, LV_EVENT_CLICKED, NULL);
    return btn;
}

static lv_obj_t* create_tile_button(lv_obj_t *parent, const char *label,
                                    lv_event_cb_t cb, int col, int row) {
    // 2-column grid (was 3). With margins of 20 px each side and a 10 px
    // gap between cols, tile width is (SCREEN_WIDTH - 50) / 2.
    int tileW = (SCREEN_WIDTH - 50) / 2;
    int tileH = 170;          // tall enough to feel tappable; icons + label fit
    int gapX  = 10;
    int gapY  = 10;
    int gridX = 20;
    int gridY = 140;

    int x = gridX + col * (tileW + gapX);
    int y = gridY + row * (tileH + gapY);

    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_size(btn, tileW, tileH);
    lv_obj_set_pos(btn, x, y);
    // Scheme-driven colours. Index by tile creation order so colours follow
    // the same left-to-right, top-to-bottom progression in Multi mode.
    int tile_idx = tile_button_count;
    lv_color_t tile_bg, tile_border;
    int tile_border_w;
    get_tile_colors(tile_idx, &tile_bg, &tile_border, &tile_border_w);
    lv_obj_set_style_bg_color(btn, tile_bg, 0);
    lv_obj_set_style_bg_color(btn, lv_color_make(50, 50, 70), LV_STATE_PRESSED);
    lv_obj_set_style_radius(btn, 12, 0);
    lv_obj_set_style_border_color(btn, tile_border, 0);
    lv_obj_set_style_border_width(btn, tile_border_w, 0);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, label);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    // Was _18; bumped to _28 to match the home title and to make icons
    // (which are glyphs in the same font) visibly larger.
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(lbl);

    if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

    if (tile_button_count < MECK_HOME_TILE_COUNT) {
        tile_buttons[tile_button_count++] = btn;
    }
    return btn;
}

static lv_obj_t* create_settings_row(lv_obj_t *parent, const char *label,
                                     lv_obj_t **value_label, lv_event_cb_t cb, int y) {
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_size(btn, SCREEN_WIDTH - 40, 55);
    lv_obj_set_pos(btn, 20, y);
    lv_obj_set_style_bg_color(btn, lv_color_make(25, 25, 35), 0);
    lv_obj_set_style_radius(btn, 10, 0);
    lv_obj_set_style_border_width(btn, 1, 0);
    lv_obj_set_style_border_color(btn, lv_color_make(50, 50, 60), 0);
    if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, label);
    lv_obj_set_style_text_color(lbl, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 5, -12);

    lv_obj_t *val = lv_label_create(btn);
    lv_label_set_text(val, "...");
    lv_obj_set_style_text_color(val, lv_color_white(), 0);
    lv_obj_set_style_text_font(val, &lv_font_montserrat_18, 0);
    lv_obj_align(val, LV_ALIGN_LEFT_MID, 5, 10);
    if (value_label) *value_label = val;

    lv_obj_t *arrow = lv_label_create(btn);
    lv_label_set_text(arrow, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(arrow, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(arrow, &lv_font_montserrat_16, 0);
    lv_obj_align(arrow, LV_ALIGN_RIGHT_MID, -5, 0);

    return btn;
}

// ============================================================================
// settings_update_labels
// ============================================================================

static void settings_update_labels() {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    if (lbl_set_name) {
        lv_label_set_text(lbl_set_name, prefs->node_name);
    }
    if (lbl_set_radio) {
        // Find matching preset name (or "Custom")
        const char* preset_name = "Custom";
        for (int i = 0; i < (int)NUM_RADIO_PRESETS; i++) {
            if (RADIO_PRESETS[i].freq == prefs->freq &&
                RADIO_PRESETS[i].bw   == prefs->bw &&
                RADIO_PRESETS[i].sf   == prefs->sf &&
                RADIO_PRESETS[i].cr   == prefs->cr) {
                preset_name = RADIO_PRESETS[i].name;
                break;
            }
        }
        lv_label_set_text(lbl_set_radio, preset_name);
    }
    if (lbl_set_txpower) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%d dBm", (int)prefs->tx_power_dbm);
        lv_label_set_text(lbl_set_txpower, buf);
    }
        if (lbl_set_pathhash) {
        const char* phs;
        switch (prefs->path_hash_mode) {
            case 0:  phs = "1 byte";  break;
            case 1:  phs = "2 bytes"; break;
            case 2:  phs = "3 bytes"; break;
            default: phs = "INVALID"; break;
        }
        lv_label_set_text(lbl_set_pathhash, phs);
    }
    if (lbl_set_utc) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%+d hours", (int)prefs->utc_offset_hours);
        lv_label_set_text(lbl_set_utc, buf);
    }
    if (lbl_set_homecolor) {
        const char* hcs;
        switch (g_home_color_scheme) {
            case HOME_COLOR_PLAIN: hcs = "Plain"; break;
            case HOME_COLOR_MULTI: hcs = "Multi"; break;
            default:               hcs = "?";     break;
        }
        lv_label_set_text(lbl_set_homecolor, hcs);
    }
}

// ============================================================================
// format_local_time
// ----------------------------------------------------------------------------
// Format a UTC epoch into a local display string, applying prefs->utc_offset_hours.
// Same calendar day (in local time) -> "8:11PM"; otherwise "15/May/26 8:53PM".
// If our clock isn't synced (epoch < 1750000000), always show date+time so a
// stale fallback timestamp doesn't get misread as "today".
// ============================================================================

static void format_local_time(uint32_t utc_epoch, char* buf, size_t buf_len) {
    if (buf_len == 0) return;
    if (utc_epoch == 0) { buf[0] = '\0'; return; }

    Meck* mesh = meck_get_instance();
    int8_t offset = 0;
    uint32_t now_utc = 0;
    if (mesh) {
        P4NodePrefs* prefs = mesh->getNodePrefs();
        if (prefs) offset = prefs->utc_offset_hours;
        mesh::RTCClock* rtc = mesh->getRTCClock();
        if (rtc) now_utc = rtc->getCurrentTime();
    }
    bool clock_synced = (now_utc >= 1750000000U);

    time_t local_msg = (time_t)utc_epoch + (int32_t)offset * 3600;
    struct tm tm_msg;
    gmtime_r(&local_msg, &tm_msg);

    bool same_day = false;
    if (clock_synced) {
        time_t local_now = (time_t)now_utc + (int32_t)offset * 3600;
        struct tm tm_now;
        gmtime_r(&local_now, &tm_now);
        same_day = (tm_msg.tm_year == tm_now.tm_year &&
                    tm_msg.tm_yday == tm_now.tm_yday);
    }

    if (same_day) {
        strftime(buf, buf_len, "%I:%M%p", &tm_msg);
    } else {
        strftime(buf, buf_len, "%d/%b/%y %I:%M%p", &tm_msg);
    }
}

// ============================================================================
// strip_unrenderable
// ----------------------------------------------------------------------------
// Copy `src` to `dst`, dropping any UTF-8 codepoint above U+00FF. The default
// Montserrat fonts compiled with LVGL cover ASCII + Latin-1 supplement (so
// é, ñ, ø all work), but emoji, CJK, and other higher-plane Unicode render
// as the missing-glyph "tofu" box. Pre-stripping at render time gives clean
// text instead of brackets.
// ============================================================================

static void strip_unrenderable(const char *src, char *dst, size_t dst_sz) {
    if (!dst || dst_sz == 0) return;
    if (!src) { dst[0] = '\0'; return; }
    size_t di = 0;
    const unsigned char *p = (const unsigned char*)src;
    while (*p && di + 1 < dst_sz) {
        unsigned char b = *p;
        if (b < 0x80) {
            // ASCII
            dst[di++] = (char)b;
            p++;
        } else if ((b & 0xE0) == 0xC0) {
            // 2-byte UTF-8: codepoint U+0080..U+07FF
            if ((p[1] & 0xC0) != 0x80) { p++; continue; }  // malformed, skip
            uint32_t cp = ((b & 0x1F) << 6) | (p[1] & 0x3F);
            if (cp <= 0x00FF && di + 2 < dst_sz) {
                dst[di++] = (char)p[0];
                dst[di++] = (char)p[1];
            }
            p += 2;
        } else if ((b & 0xF0) == 0xE0) {
            // 3-byte UTF-8: codepoint > 0x07FF, always above 0x00FF — drop
            p += 3;
        } else if ((b & 0xF8) == 0xF0) {
            // 4-byte UTF-8: emoji and other supplementary planes — drop
            p += 4;
        } else {
            // continuation or invalid lead byte
            p++;
        }
    }
    dst[di] = '\0';
}

// ============================================================================
// sender_bubble_color
// ----------------------------------------------------------------------------
// Hash a sender name into a hue and return an HSV-derived RGB colour for
// the message bubble background. This replaced an earlier 16-entry palette
// approach which collided heavily across hundreds of nodes — going to HSV
// gives effectively one bucket per hue degree (360 unique buckets), and
// fixed S/V keep the result readable under white body text regardless of
// hue. The same name always maps to the same colour.
// ============================================================================

static lv_color_t sender_bubble_color(const char *name) {
    if (!name || !*name) {
        return lv_color_make(40, 40, 50);   // neutral grey for nameless
    }
    unsigned hash = 5381;
    for (const char *p = name; *p; p++) {
        hash = hash * 33 + (unsigned char)*p;
    }
    // Saturation 70 keeps the colour clearly chromatic without being
    // garish; value 35 keeps it dark enough that white text reads on
    // every hue (yellow stays manageable, deep blue doesn't crush).
    return lv_color_hsv_to_rgb((uint16_t)(hash % 360), 70, 35);
}

// Companion to sender_bubble_color: same hash, but higher value so the
// resulting colour sits visibly *above* the bubble bg. Used as the fill
// for the sender-name chip so the name reads as a distinct pill rather
// than blending into the body text.
static lv_color_t sender_chip_color(const char *name) {
    if (!name || !*name) {
        return lv_color_make(180, 180, 200);  // light grey fallback
    }
    unsigned hash = 5381;
    for (const char *p = name; *p; p++) {
        hash = hash * 33 + (unsigned char)*p;
    }
    // S=60, V=75 — light enough that black text reads on every hue,
    // saturated enough to still feel like the sender's colour.
    return lv_color_hsv_to_rgb((uint16_t)(hash % 360), 60, 75);
}

// ============================================================================
// rebuild_message_bubbles
// ----------------------------------------------------------------------------
// Tear down and rebuild the message list inside obj_msg_scroll. Each message
// becomes a row (full width, transparent, horizontal flex) containing an
// inner bubble. Bubbles align right for messages we sent (sender == our
// node name), left otherwise. Sender names get their own colour from a
// hash-into-CH_COLORS for visual identification across the conversation.
// ============================================================================

static void rebuild_message_bubbles(uint8_t ch_idx) {
    if (!obj_msg_scroll) return;
    Meck* mesh = meck_get_instance();

    // Burn down old bubbles. obj_msg_scroll is the parent for everything
    // we render here, so a single clean() resets the whole conversation.
    lv_obj_clean(obj_msg_scroll);
    lbl_messages_body = NULL;   // it lived as a child of obj_msg_scroll

    // Make sure the scroll container lays its children top-to-bottom.
    // (Idempotent — safe to set every rebuild.)
    lv_obj_set_layout(obj_msg_scroll, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(obj_msg_scroll, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(obj_msg_scroll, 6, 0);

    P4ChannelMessage msgs[16];
    int n = mesh ? mesh->getMessages(msgs, 16, ch_idx) : 0;

    if (n == 0) {
        lv_obj_t *empty = lv_label_create(obj_msg_scroll);
        lv_label_set_text(empty, "No messages yet.");
        lv_obj_set_style_text_color(empty, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_obj_set_style_text_font(empty, &lv_font_montserrat_18, 0);
        lbl_messages_body = empty;  // keep dirty-flag gate happy
        return;
    }

    // Our own node name, used to detect sent vs received messages.
    P4NodePrefs* prefs = mesh ? mesh->getNodePrefs() : nullptr;
    char my_name_clean[40] = "";
    if (prefs && prefs->node_name[0]) {
        strip_unrenderable(prefs->node_name, my_name_clean, sizeof(my_name_clean));
    }

    int row_max_w = (SCREEN_WIDTH - 20) - 20; // scroll inner width minus its pad
    int bubble_max_w = (int)((float)row_max_w * 0.78f);

    // getMessages returns newest-first. We want oldest at top, newest at
    // bottom (standard chat order), so iterate reverse.
    for (int i = n - 1; i >= 0; i--) {
        // Strip non-renderable Unicode from the raw text. After this step
        // we operate on a buffer that the font can actually display.
        char clean_text[256];
        strip_unrenderable(msgs[i].text, clean_text, sizeof(clean_text));

        // Split sender / body on first ": ". If there's no colon, treat
        // the whole thing as body with empty sender.
        char sender[64] = "";
        const char *body = clean_text;
        const char *colon = strstr(clean_text, ": ");
        if (colon) {
            int sender_len = (int)(colon - clean_text);
            if (sender_len > 0 && sender_len < (int)sizeof(sender)) {
                memcpy(sender, clean_text, sender_len);
                sender[sender_len] = '\0';
                body = colon + 2;
            }
        }
        // Trim leading "- " / spaces that some sender strings carry.
        char *trim_sender = sender;
        while (*trim_sender == '-' || *trim_sender == ' ') trim_sender++;

        bool is_sent = (my_name_clean[0] != '\0' &&
                        strcmp(trim_sender, my_name_clean) == 0);

        // Per-sender colour from sender_bubble_color() — hashes the name
        // (parsed sender for received, our own node name for sent) onto
        // a 360-degree hue circle. Same name → same colour every time.
        const char *color_name = is_sent ? my_name_clean : trim_sender;

        // ---- Row: full-width, transparent, holds the bubble. ----
        lv_obj_t *row = lv_obj_create(obj_msg_scroll);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_scroll_dir(row, LV_DIR_NONE);
        lv_obj_set_layout(row, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row,
            is_sent ? LV_FLEX_ALIGN_END : LV_FLEX_ALIGN_START,
            LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

        // ---- Bubble: rounded, padded, content-sized up to bubble_max_w. ----
        lv_obj_t *bubble = lv_obj_create(row);
        lv_obj_set_width(bubble, LV_SIZE_CONTENT);
        lv_obj_set_height(bubble, LV_SIZE_CONTENT);
        lv_obj_set_style_max_width(bubble, bubble_max_w, 0);
        lv_obj_set_style_radius(bubble, 12, 0);
        lv_obj_set_style_pad_all(bubble, 10, 0);
        lv_obj_set_style_border_width(bubble, 0, 0);
        lv_obj_set_scroll_dir(bubble, LV_DIR_NONE);
        lv_obj_set_style_bg_color(bubble, sender_bubble_color(color_name), 0);
        lv_obj_set_layout(bubble, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(bubble, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(bubble, 2, 0);

        // Sender name on received messages only; sent messages don't need
        // it (the right-alignment makes "from me" obvious). Wrapped in a
        // pill-shaped chip whose colour comes from sender_chip_color() —
        // same hue as the bubble, but lighter — so the name reads as a
        // distinct label rather than blending with the body text.
        if (!is_sent && trim_sender[0]) {
            lv_obj_t *chip = lv_obj_create(bubble);
            lv_obj_set_size(chip, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
            lv_obj_set_style_bg_color(chip, sender_chip_color(color_name), 0);
            lv_obj_set_style_radius(chip, 8, 0);
            lv_obj_set_style_border_width(chip, 0, 0);
            lv_obj_set_style_pad_left(chip, 10, 0);
            lv_obj_set_style_pad_right(chip, 10, 0);
            lv_obj_set_style_pad_top(chip, 3, 0);
            lv_obj_set_style_pad_bottom(chip, 3, 0);
            lv_obj_clear_flag(chip, LV_OBJ_FLAG_SCROLLABLE);

            lv_obj_t *lbl_sender = lv_label_create(chip);
            lv_label_set_text(lbl_sender, trim_sender);
            // Black text on a V=75 chip reads cleanly across every hue,
            // even on yellow and lime where white would wash out.
            lv_obj_set_style_text_color(lbl_sender, lv_color_black(), 0);
            lv_obj_set_style_text_font(lbl_sender, &lv_font_montserrat_22, 0);
        }

        // Message body. Width-bounded so wrapping kicks in inside the bubble.
        lv_obj_t *lbl_body = lv_label_create(bubble);
        lv_label_set_text(lbl_body, body);
        lv_obj_set_style_text_color(lbl_body, lv_color_white(), 0);
        lv_obj_set_style_text_font(lbl_body, &lv_font_montserrat_22, 0);
        lv_label_set_long_mode(lbl_body, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(lbl_body, bubble_max_w - 22);

        // Timestamp footer in muted grey.
        char timebuf[32];
        format_local_time(msgs[i].timestamp, timebuf, sizeof(timebuf));
        if (timebuf[0]) {
            lv_obj_t *lbl_time = lv_label_create(bubble);
            lv_label_set_text(lbl_time, timebuf);
            lv_obj_set_style_text_color(lbl_time, lv_palette_main(LV_PALETTE_GREY), 0);
            lv_obj_set_style_text_font(lbl_time, &lv_font_montserrat_14, 0);
            // Right-justify timestamp on sent bubbles to mirror chat-app convention.
            lv_obj_set_style_text_align(lbl_time,
                is_sent ? LV_TEXT_ALIGN_RIGHT : LV_TEXT_ALIGN_LEFT, 0);
        }
    }

    // Hold a non-NULL pointer in lbl_messages_body so the timer callback's
    // dirty-check still gates correctly. We point it at the last bubble
    // (which gets cleaned along with everything else on next rebuild).
    lbl_messages_body = lv_obj_get_child(obj_msg_scroll, -1);

    lv_obj_scroll_to_y(obj_msg_scroll, LV_COORD_MAX, LV_ANIM_OFF);
}

// ============================================================================
// update_radio_detail_label
// ============================================================================

static void update_radio_detail_label() {
    if (!lbl_radio_detail) return;
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    // Real noise floor from the radio_driver estimator. Backed by SX126x
    // GetRssiInst (opcode 0x15) sampled every ~2 s in P4SX1262Radio's
    // recvRaw. Cold-starts at -120 dBm and converges within a few samples.
    int nf = radio_driver.getNoiseFloor();

    char buf[320];
    snprintf(buf, sizeof(buf),
        "Frequency:     %.3f MHz\n"
        "Bandwidth:     %.1f kHz\n"
        "Spread Factor: SF%d\n"
        "Coding Rate:   4/%d\n"
        "TX Power:      %d dBm\n"
        "Sync Word:     0x1424\n"
        "Noise Floor:   %d dBm",
        prefs->freq, prefs->bw, prefs->sf, prefs->cr,
        (int)prefs->tx_power_dbm,
        nf);
    lv_label_set_text(lbl_radio_detail, buf);
}

// ============================================================================
// Settings tap handlers
// ============================================================================

static void on_settings_name_save(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    if (ta_settings_name) {
        const char* text = lv_textarea_get_text(ta_settings_name);
        if (text && text[0]) {
            strncpy(prefs->node_name, text, sizeof(prefs->node_name) - 1);
            prefs->node_name[sizeof(prefs->node_name) - 1] = '\0';
            mesh->getDataStore()->savePrefs(*prefs);
            printf("Settings: node name changed to '%s'\n", prefs->node_name);
        }
    }
    if (obj_name_edit_panel) lv_obj_add_flag(obj_name_edit_panel, LV_OBJ_FLAG_HIDDEN);
    settings_update_labels();
}

static void on_settings_name_cancel(lv_event_t *e) {
    if (obj_name_edit_panel) lv_obj_add_flag(obj_name_edit_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_settings_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY)       on_settings_name_save(NULL);
    else if (code == LV_EVENT_CANCEL) on_settings_name_cancel(NULL);
}

static void on_settings_name_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    if (obj_name_edit_panel && ta_settings_name) {
        lv_textarea_set_text(ta_settings_name, prefs->node_name);
        lv_obj_remove_flag(obj_name_edit_panel, LV_OBJ_FLAG_HIDDEN);
        if (kb_settings) lv_keyboard_set_textarea(kb_settings, ta_settings_name);
    }
}

static void on_settings_radio_tap(lv_event_t *e) {
    if (scr_radio_picker) lv_screen_load(scr_radio_picker);
}

static void on_radio_preset_select(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= (int)NUM_RADIO_PRESETS) return;

    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    prefs->freq         = RADIO_PRESETS[idx].freq;
    prefs->bw           = RADIO_PRESETS[idx].bw;
    prefs->sf           = RADIO_PRESETS[idx].sf;
    prefs->cr           = RADIO_PRESETS[idx].cr;
    prefs->tx_power_dbm = RADIO_PRESETS[idx].tx_power;
    mesh->getDataStore()->savePrefs(*prefs);

    radio_request_reconfig(prefs->freq, prefs->bw, prefs->sf, prefs->cr, prefs->tx_power_dbm);

    printf("Settings: radio preset '%s' selected\n", RADIO_PRESETS[idx].name);
    settings_update_labels();
    update_radio_detail_label();
    if (scr_settings) lv_screen_load(scr_settings);
}

static void on_settings_txpower_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    int cur = -1;
    for (int i = 0; i < NUM_TX_POWER_OPTIONS; i++) {
        if (TX_POWER_OPTIONS[i] == prefs->tx_power_dbm) { cur = i; break; }
    }
    cur = (cur + 1) % NUM_TX_POWER_OPTIONS;
    prefs->tx_power_dbm = TX_POWER_OPTIONS[cur];
    mesh->getDataStore()->savePrefs(*prefs);

    radio_request_reconfig(prefs->freq, prefs->bw, prefs->sf, prefs->cr, prefs->tx_power_dbm);

    printf("Settings: TX power = %d dBm\n", prefs->tx_power_dbm);
    settings_update_labels();
    update_radio_detail_label();
}
static void on_settings_pathhash_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    // Cycle 0 -> 1 -> 2 -> 0. Mode 3 is reserved per companion protocol, skip it.
    prefs->path_hash_mode = (prefs->path_hash_mode + 1) % 3;
    mesh->getDataStore()->savePrefs(*prefs);

    printf("Settings: Path hash mode = %u (sends %u byte(s))\n",
           (unsigned)prefs->path_hash_mode,
           (unsigned)(prefs->path_hash_mode + 1));
    settings_update_labels();
}
static void on_settings_utc_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    prefs->utc_offset_hours++;
    if (prefs->utc_offset_hours > 14) prefs->utc_offset_hours = -12;
    mesh->getDataStore()->savePrefs(*prefs);
    printf("Settings: UTC offset = %+d\n", (int)prefs->utc_offset_hours);
    settings_update_labels();
}

// Home Color row. Bound to BOTH LV_EVENT_CLICKED (tap = forward cycle) and
// LV_EVENT_GESTURE (swipe-left = forward, swipe-right = back). With only two
// schemes today both directions land on the other one, but the structure is
// ready for more schemes to be added to the enum.
static void on_settings_homecolor_input(lv_event_t *e) {
    int dir = +1;  // tap (LV_EVENT_CLICKED) defaults to forward
    if (lv_event_get_code(e) == LV_EVENT_GESTURE) {
        lv_dir_t d = lv_indev_get_gesture_dir(lv_indev_active());
        if (d == LV_DIR_LEFT)  dir = +1;
        else if (d == LV_DIR_RIGHT) dir = -1;
        else return;  // vertical swipe: ignore so scrolling still works
    }
    int n = (int)HOME_COLOR_COUNT;
    g_home_color_scheme = ((g_home_color_scheme + dir) % n + n) % n;
    save_home_color_to_nvs();
    apply_tile_colors();
    settings_update_labels();
    printf("Settings: home color = %d\n", g_home_color_scheme);
}

// ============================================================================
// Messages screen handlers (lifted from old:193-230)
// ============================================================================

static void on_send_clicked(lv_event_t *e) {
    if (!ta_compose) return;
    const char *text = lv_textarea_get_text(ta_compose);
    if (text && text[0]) {
        // Defer to meck_task to avoid SPI race with mesh.loop()
        meck_request_send_text(g_active_channel, text);
        lv_textarea_set_text(ta_compose, "");
    }
    if (kb_compose) lv_obj_add_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
}

static void on_compose_focused(lv_event_t *e) {
    if (kb_compose) {
        lv_keyboard_set_textarea(kb_compose, ta_compose);
        lv_obj_remove_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
        if (obj_msg_scroll) lv_obj_set_height(obj_msg_scroll, SCREEN_HEIGHT - 410);
        if (ta_compose) lv_obj_align(ta_compose, LV_ALIGN_BOTTOM_LEFT, 10, -295);
        if (btn_send)   lv_obj_align(btn_send,   LV_ALIGN_BOTTOM_RIGHT, -10, -295);
    }
}

static void on_compose_defocused(lv_event_t *e) {
    if (kb_compose) {
        lv_obj_add_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
        if (obj_msg_scroll) lv_obj_set_height(obj_msg_scroll, SCREEN_HEIGHT - 130);
        if (ta_compose) lv_obj_align(ta_compose, LV_ALIGN_BOTTOM_LEFT, 10, -10);
        if (btn_send)   lv_obj_align(btn_send,   LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    }
}

static void on_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        on_send_clicked(NULL);
    } else if (code == LV_EVENT_CANCEL) {
        if (kb_compose) lv_obj_add_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
        if (obj_msg_scroll) lv_obj_set_height(obj_msg_scroll, SCREEN_HEIGHT - 130);
        if (ta_compose) lv_obj_align(ta_compose, LV_ALIGN_BOTTOM_LEFT, 10, -10);
        if (btn_send)   lv_obj_align(btn_send,   LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    }
}

// ============================================================================
// Contacts screen handlers (lifted from old:369-415)
// ============================================================================

static void on_contact_tap(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    g_selected_contact_idx = idx;

    Meck* mesh = meck_get_instance();
    if (!mesh || !lbl_contact_detail_body) return;

    ContactInfo ci;
    if (!mesh->getContactByIdx(idx, ci)) return;

    char buf[512];
    snprintf(buf, sizeof(buf),
        "Name:     %s\n\n"
        "Type:     %d\n"
        "Flags:    0x%02X %s\n\n"
        "Identity:\n"
        "  %02X%02X%02X%02X %02X%02X%02X%02X\n"
        "  %02X%02X%02X%02X %02X%02X%02X%02X\n\n"
        "Path:     %s (%d hops)\n"
        "Last:     %lu",
        ci.name, ci.type,
        ci.flags, (ci.flags & 0x01) ? "[FAV]" : "",
        ci.id.pub_key[0], ci.id.pub_key[1], ci.id.pub_key[2], ci.id.pub_key[3],
        ci.id.pub_key[4], ci.id.pub_key[5], ci.id.pub_key[6], ci.id.pub_key[7],
        ci.id.pub_key[8], ci.id.pub_key[9], ci.id.pub_key[10], ci.id.pub_key[11],
        ci.id.pub_key[12], ci.id.pub_key[13], ci.id.pub_key[14], ci.id.pub_key[15],
        ci.out_path_len == OUT_PATH_UNKNOWN ? "unknown" : "known",
        ci.out_path_len == OUT_PATH_UNKNOWN ? 0 : (ci.out_path_len & 0x3F),
        (unsigned long)ci.last_advert_timestamp);

    lv_label_set_text(lbl_contact_detail_body, buf);
    if (scr_contact_detail) lv_screen_load(scr_contact_detail);
}

static void on_contact_fav_toggle(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh || g_selected_contact_idx < 0) return;
    ContactInfo ci;
    if (!mesh->getContactByIdx(g_selected_contact_idx, ci)) return;

    ci.flags ^= 0x01;
    // BaseChatMesh doesn't expose setContactFlags yet, so this toggle is UI-only
    // until persistence path is added.
    printf("Contacts: toggled favourite for '%s' -> 0x%02X\n", ci.name, ci.flags);

    // Refresh detail view by passing through the same idx
    // Re-render using same logic as on_contact_tap
    char buf[512];
    snprintf(buf, sizeof(buf),
        "Name:     %s\n\n"
        "Type:     %d\n"
        "Flags:    0x%02X %s\n\n(toggle is UI-only until persistence wired)",
        ci.name, ci.type, ci.flags, (ci.flags & 0x01) ? "[FAV]" : "");
    if (lbl_contact_detail_body) lv_label_set_text(lbl_contact_detail_body, buf);
}

static void goto_contacts_from_detail(lv_event_t *e) {
    if (scr_contacts) lv_screen_load(scr_contacts);
}

// ============================================================================
// Channel handlers (lifted from old:683-748)
// ============================================================================

static void on_ch_delete(lv_event_t *e) {
    intptr_t idx = (intptr_t)lv_event_get_user_data(e);
    Meck* mesh = meck_get_instance();
    if (!mesh || idx < 0 || idx >= mesh->getActiveChannelCount()) return;

    if (mesh->getActiveChannelCount() <= 1) {
        printf("Channels: cannot delete last channel\n");
        return;
    }

    ChannelDetails ch;
    char name[P4_CHANNEL_NAME_MAX] = {};
    if (mesh->getChannel((uint8_t)idx, ch)) {
        strncpy(name, ch.name, sizeof(name) - 1);
    }
    mesh->deleteChannel((uint8_t)idx);
    printf("Channels: deleted '%s' (idx %d)\n", name, (int)idx);
    refresh_channel_picker();
}

static void on_ch_add_save(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!ta_ch_add || !mesh) return;
    const char* text = lv_textarea_get_text(ta_ch_add);
    if (!text || !text[0]) return;

    char name[P4_CHANNEL_NAME_MAX] = {};
    if (text[0] == '#') {
        strncpy(name, text, sizeof(name) - 1);
    } else {
        name[0] = '#';
        strncpy(name + 1, text, sizeof(name) - 2);
    }
    mesh->addHashChannel(name);
    printf("Channels: added '%s'\n", name);
    if (obj_ch_add_panel) lv_obj_add_flag(obj_ch_add_panel, LV_OBJ_FLAG_HIDDEN);
    refresh_channel_picker();
}

static void on_ch_add_cancel(lv_event_t *e) {
    if (obj_ch_add_panel) lv_obj_add_flag(obj_ch_add_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_ch_add_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY)       on_ch_add_save(NULL);
    else if (code == LV_EVENT_CANCEL) on_ch_add_cancel(NULL);
}

static void on_ch_add_tap(lv_event_t *e) {
    if (obj_ch_add_panel && ta_ch_add) {
        lv_textarea_set_text(ta_ch_add, "#");
        lv_obj_remove_flag(obj_ch_add_panel, LV_OBJ_FLAG_HIDDEN);
        if (kb_ch_add) lv_keyboard_set_textarea(kb_ch_add, ta_ch_add);
    }
}

// ============================================================================
// Channel navigation
// ============================================================================

static void goto_channel_n(lv_event_t *e) {
    intptr_t idx = (intptr_t)lv_event_get_user_data(e);
    load_channel_view((uint8_t)idx);
}

// ============================================================================
// Tile 0: Home
// ============================================================================

static void create_page_home(lv_obj_t *page) {
    Meck* mesh = meck_get_instance();
    P4NodePrefs* prefs = mesh ? mesh->getNodePrefs() : nullptr;

    // Reload the home colour scheme from NVS (idempotent — safe even on a
    // rebuild) and clear the tile registry so this build's tiles are the
    // only ones tracked.
    load_home_color_from_nvs();
    tile_button_count = 0;
    for (int i = 0; i < MECK_HOME_TILE_COUNT; i++) tile_buttons[i] = NULL;

    // Title is the user-chosen node name. Updated each tick by
    // ui_update_timer_cb so a rename in Settings shows up live.
    lbl_home_title = lv_label_create(page);
    const char *initial_name = (prefs && prefs->node_name[0])
                             ? prefs->node_name
                             : "(no name)";
    lv_label_set_text(lbl_home_title, initial_name);
    lv_obj_set_style_text_color(lbl_home_title, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_home_title, &lv_font_montserrat_28, 0);
    lv_obj_align(lbl_home_title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    lv_obj_t *radio = lv_label_create(page);
    lv_label_set_long_mode(radio, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(radio, SCREEN_WIDTH / 2 - 40);
    char radio_str[64];
    if (prefs) {
        snprintf(radio_str, sizeof(radio_str), "%.3f MHz\nBW%.0f SF%d CR4/%d",
                 prefs->freq, prefs->bw, prefs->sf, prefs->cr);
    } else {
        snprintf(radio_str, sizeof(radio_str), "(no prefs)");
    }
    lv_label_set_text(radio, radio_str);
    lv_obj_set_style_text_color(radio, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_text_font(radio, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(radio, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(radio, LV_ALIGN_TOP_RIGHT, -5, NOTCH_SAFE_Y);

    lbl_home_unread = lv_label_create(page);
    lv_label_set_text(lbl_home_unread, "MSG: 0");
    lv_obj_set_style_text_color(lbl_home_unread, lv_color_make(0, 255, 100), 0);
    lv_obj_set_style_text_font(lbl_home_unread, &lv_font_montserrat_18, 0);
    lv_obj_align(lbl_home_unread, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, 60);

    lbl_home_packets = lv_label_create(page);
    lv_label_set_text(lbl_home_packets, "RX: 0");
    lv_obj_set_style_text_color(lbl_home_packets, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(lbl_home_packets, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_home_packets, LV_ALIGN_TOP_RIGHT, -10, 60);

    lbl_home_rssi = lv_label_create(page);
    lv_label_set_text(lbl_home_rssi, "");
    lv_obj_set_style_text_color(lbl_home_rssi, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(lbl_home_rssi, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_home_rssi, LV_ALIGN_TOP_RIGHT, -10, 80);

    // Navigation grid: 10 tiles in 2 columns × 5 rows. First two rows are
    // the high-traffic items; placeholder tiles for Trace, Maps, Audio, and
    // Web are at the bottom as visual markers for future work. Tap any
    // placeholder to print a TODO line to serial. The Web tile will host
    // the text-based browser + IRC client ported from other Meck builds.
    create_tile_button(page, LV_SYMBOL_ENVELOPE "\nMessages", goto_channel_picker, 0, 0);
    create_tile_button(page, LV_SYMBOL_LIST     "\nContacts", goto_contacts,       1, 0);
    create_tile_button(page, LV_SYMBOL_SETTINGS "\nSettings", goto_settings,       0, 1);
    create_tile_button(page, LV_SYMBOL_FILE     "\nReader",   cb_todo_reader,      1, 1);
    create_tile_button(page, LV_SYMBOL_EDIT     "\nNotes",    cb_todo_notes,       0, 2);
    create_tile_button(page, LV_SYMBOL_GPS      "\nDiscover", cb_todo_discover,    1, 2);
    create_tile_button(page, LV_SYMBOL_SHUFFLE  "\nTrace",    cb_todo_trace,       0, 3);
    create_tile_button(page, LV_SYMBOL_IMAGE    "\nMaps",     cb_todo_maps,        1, 3);
    create_tile_button(page, LV_SYMBOL_AUDIO    "\nAudio",    cb_todo_audio,       0, 4);
    create_tile_button(page, LV_SYMBOL_WIFI     "\nWeb",      cb_todo_web,         1, 4);

    lv_obj_t *hint = lv_label_create(page);
    lv_label_set_text(hint, "Swipe left for more pages " LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(hint, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -20);
}

// ============================================================================
// Tile 1: Recent Heard
// ============================================================================

static void create_page_recent(lv_obj_t *page) {
    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, "Recent Heard");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    lbl_recent_list = lv_label_create(page);
    lv_label_set_text(lbl_recent_list, "Waiting for adverts...");
    lv_obj_set_style_text_color(lbl_recent_list, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_recent_list, &lv_font_montserrat_16, 0);
    lv_label_set_long_mode(lbl_recent_list, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_recent_list, SCREEN_WIDTH - 40);
    lv_obj_align(lbl_recent_list, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, 60);
}

// ============================================================================
// Tile 2: Radio Details
// ============================================================================

static void create_page_radio(lv_obj_t *page) {
    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, "Radio Details");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_YELLOW), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    lbl_radio_detail = lv_label_create(page);
    lv_label_set_text(lbl_radio_detail, "Loading...");
    lv_obj_set_style_text_color(lbl_radio_detail, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_radio_detail, &lv_font_montserrat_18, 0);
    lv_label_set_long_mode(lbl_radio_detail, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_radio_detail, SCREEN_WIDTH - 40);
    lv_obj_align(lbl_radio_detail, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, 60);

    update_radio_detail_label();
}

// ============================================================================
// Tile 3: Advert (long-press to send manual advert)
// ============================================================================

static void advert_toggle_cb(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    mesh::Packet* adv = mesh->createSelfAdvert(prefs->node_name);
    if (adv) {
        mesh->sendFlood(adv);
        printf("Meck: manual advert sent (name='%s')\n", prefs->node_name);
    } else {
        printf("Meck: createSelfAdvert returned null\n");
    }
}

static void create_page_advert(lv_obj_t *page) {
    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, "Advertising");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    lv_obj_t *icon = lv_label_create(page);
    lv_label_set_text(icon, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(icon, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_28, 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -30);

    lbl_advert_status = lv_label_create(page);
    lv_label_set_text(lbl_advert_status, "Advert\n\nLong press to send");
    lv_obj_set_style_text_color(lbl_advert_status, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_advert_status, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_align(lbl_advert_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl_advert_status, LV_ALIGN_CENTER, 0, 30);

    lv_obj_add_event_cb(page, advert_toggle_cb, LV_EVENT_LONG_PRESSED, NULL);
    lv_obj_add_flag(page, LV_OBJ_FLAG_CLICKABLE);
}

// ============================================================================
// Tile 4: GPS
//
// Layout: title (green) + single detail block with 6 sections:
//   Status, Fix, Satellites, Position, Altitude, Sentences (count + rate).
//
// Data is pulled via meck_gps_get_snapshot() in ui_update_timer_cb at 500ms.
// device_gps_task in main.cpp populates Sys_Status.l76k continuously, so
// display values are at most ~500ms stale.
// ============================================================================

static void create_page_gps(lv_obj_t *page) {
    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, "GPS");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    lbl_gps_detail = lv_label_create(page);
    lv_label_set_text(lbl_gps_detail, "Loading...");
    lv_obj_set_style_text_color(lbl_gps_detail, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_gps_detail, &lv_font_montserrat_18, 0);
    lv_label_set_long_mode(lbl_gps_detail, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_gps_detail, SCREEN_WIDTH - 40);
    lv_obj_align(lbl_gps_detail, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, 60);
}

// ============================================================================
// Tile 5: Battery (placeholder)
// ============================================================================

static void create_page_battery(lv_obj_t *page) {
    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, "Battery Gauge");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    lbl_battery_detail = lv_label_create(page);
    lv_label_set_text(lbl_battery_detail, "Reading BQ27220...");
    lv_obj_set_style_text_color(lbl_battery_detail, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_battery_detail, &lv_font_montserrat_18, 0);
    lv_label_set_long_mode(lbl_battery_detail, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_battery_detail, SCREEN_WIDTH - 40);
    lv_obj_align(lbl_battery_detail, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, 60);
}

// ============================================================================
// Tile 6: Hibernate
// ============================================================================

static void create_page_shutdown(lv_obj_t *page) {
    lv_obj_t *icon = lv_label_create(page);
    lv_label_set_text(icon, LV_SYMBOL_POWER);
    lv_obj_set_style_text_color(icon, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_28, 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -40);

    lv_obj_t *lbl = lv_label_create(page);
    lv_label_set_text(lbl, "Hibernate\n\nLong press to enter\ndeep sleep");
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 30);
}

// ============================================================================
// Settings screen
// ============================================================================

static void create_settings_screen() {
    scr_settings = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_settings, lv_color_black(), 0);

    create_back_button(scr_settings);

    lv_obj_t *title = lv_label_create(scr_settings);
    lv_label_set_text(title, "Settings");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 100, 18);

    lv_obj_t *scroll = lv_obj_create(scr_settings);
    lv_obj_set_size(scroll, SCREEN_WIDTH, SCREEN_HEIGHT - 60);
    lv_obj_set_pos(scroll, 0, 60);
    lv_obj_set_style_bg_opa(scroll, 0, 0);
    lv_obj_set_style_border_width(scroll, 0, 0);
    lv_obj_set_style_pad_all(scroll, 0, 0);
    lv_obj_set_scroll_dir(scroll, LV_DIR_VER);

    int y = 5;
    create_settings_row(scroll, "Node Name",                 &lbl_set_name,    on_settings_name_tap,    y);
    y += 65;
    create_settings_row(scroll, "Radio Preset",              &lbl_set_radio,   on_settings_radio_tap,   y);
    y += 65;
    create_settings_row(scroll, "TX Power (tap to cycle)",   &lbl_set_txpower, on_settings_txpower_tap, y);
    y += 65;
    create_settings_row(scroll, "Path Hash (tap to cycle)", &lbl_set_pathhash, on_settings_pathhash_tap, y);
    y += 65;
    create_settings_row(scroll, "UTC Offset (tap to cycle)", &lbl_set_utc,     on_settings_utc_tap,     y);
    y += 65;
    // Home Color: tap to forward-cycle. (Swipe gesture is also wired up
    // below for future use, but tap is the documented interaction since
    // small rows often catch the touch as a tap before a swipe registers.)
    lv_obj_t *home_color_row = create_settings_row(
        scroll, "Home Color (tap to cycle)", &lbl_set_homecolor,
        on_settings_homecolor_input, y);
    lv_obj_add_event_cb(home_color_row, on_settings_homecolor_input,
                        LV_EVENT_GESTURE, NULL);
    y += 65;

    lv_obj_t *id_row = lv_obj_create(scroll);
    lv_obj_set_size(id_row, SCREEN_WIDTH - 40, 110);
    lv_obj_set_pos(id_row, 20, y);
    lv_obj_set_style_bg_color(id_row, lv_color_make(15, 15, 20), 0);
    lv_obj_set_style_radius(id_row, 10, 0);
    lv_obj_set_style_border_width(id_row, 0, 0);
    lv_obj_set_style_pad_all(id_row, 10, 0);

    lv_obj_t *id_title = lv_label_create(id_row);
    lv_label_set_text(id_title, "Identity (public key)");
    lv_obj_set_style_text_color(id_title, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(id_title, &lv_font_montserrat_14, 0);
    lv_obj_align(id_title, LV_ALIGN_TOP_LEFT, 0, 0);

    lbl_set_identity = lv_label_create(id_row);
    Meck* mesh = meck_get_instance();
    if (mesh) {
        const uint8_t* pk = mesh->getIdentity().pub_key;
        char idbuf[160];
        snprintf(idbuf, sizeof(idbuf),
            "%02X%02X%02X%02X %02X%02X%02X%02X\n"
            "%02X%02X%02X%02X %02X%02X%02X%02X\n"
            "%02X%02X%02X%02X %02X%02X%02X%02X\n"
            "%02X%02X%02X%02X %02X%02X%02X%02X",
            pk[0],pk[1],pk[2],pk[3],     pk[4],pk[5],pk[6],pk[7],
            pk[8],pk[9],pk[10],pk[11],   pk[12],pk[13],pk[14],pk[15],
            pk[16],pk[17],pk[18],pk[19], pk[20],pk[21],pk[22],pk[23],
            pk[24],pk[25],pk[26],pk[27], pk[28],pk[29],pk[30],pk[31]);
        lv_label_set_text(lbl_set_identity, idbuf);
    } else {
        lv_label_set_text(lbl_set_identity, "Not initialized");
    }
    lv_obj_set_style_text_color(lbl_set_identity, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_text_font(lbl_set_identity, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_set_identity, LV_ALIGN_TOP_LEFT, 0, 22);

    // Firmware row at the bottom of the settings list. Static; not tappable.
    // The home screen used to display "Meck P4" as its title; now that the
    // title shows the user's chosen node name, the firmware identity belongs
    // here on the Settings screen instead.
    y += 120;  // clear past the identity panel (height 110 + a small gap)
    lv_obj_t *fw_row = lv_obj_create(scroll);
    lv_obj_set_size(fw_row, SCREEN_WIDTH - 40, 55);
    lv_obj_set_pos(fw_row, 20, y);
    lv_obj_set_style_bg_color(fw_row, lv_color_make(15, 15, 20), 0);
    lv_obj_set_style_radius(fw_row, 10, 0);
    lv_obj_set_style_border_width(fw_row, 0, 0);
    lv_obj_set_style_pad_all(fw_row, 10, 0);

    lv_obj_t *fw_title_lbl = lv_label_create(fw_row);
    lv_label_set_text(fw_title_lbl, "Firmware");
    lv_obj_set_style_text_color(fw_title_lbl, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(fw_title_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(fw_title_lbl, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *fw_value_lbl = lv_label_create(fw_row);
    lv_label_set_text(fw_value_lbl, MECK_FIRMWARE_NAME " v" MECK_FIRMWARE_VERSION);
    lv_obj_set_style_text_color(fw_value_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(fw_value_lbl, &lv_font_montserrat_16, 0);
    lv_obj_align(fw_value_lbl, LV_ALIGN_RIGHT_MID, 0, 0);

    // Name edit overlay
    obj_name_edit_panel = lv_obj_create(scr_settings);
    lv_obj_set_size(obj_name_edit_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_name_edit_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_name_edit_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_name_edit_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(obj_name_edit_panel, 0, 0);
    lv_obj_add_flag(obj_name_edit_panel, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *edit_title = lv_label_create(obj_name_edit_panel);
    lv_label_set_text(edit_title, "Edit Node Name");
    lv_obj_set_style_text_color(edit_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(edit_title, &lv_font_montserrat_22, 0);
    lv_obj_align(edit_title, LV_ALIGN_TOP_MID, 0, 20);

    ta_settings_name = lv_textarea_create(obj_name_edit_panel);
    lv_obj_set_size(ta_settings_name, SCREEN_WIDTH - 40, 50);
    lv_obj_align(ta_settings_name, LV_ALIGN_TOP_MID, 0, 60);
    lv_textarea_set_one_line(ta_settings_name, true);
    lv_textarea_set_max_length(ta_settings_name, 30);
    lv_obj_set_style_bg_color(ta_settings_name, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_settings_name, lv_color_white(), 0);
    lv_obj_set_style_text_font(ta_settings_name, &lv_font_montserrat_18, 0);
    lv_obj_set_style_border_color(ta_settings_name, lv_palette_main(LV_PALETTE_CYAN), 0);

    kb_settings = lv_keyboard_create(obj_name_edit_panel);
    lv_obj_set_size(kb_settings, SCREEN_WIDTH, 280);
    lv_obj_align(kb_settings, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_textarea(kb_settings, ta_settings_name);
    lv_obj_add_event_cb(kb_settings, on_settings_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_settings, on_settings_kb_event, LV_EVENT_CANCEL, NULL);

    settings_update_labels();
}

// ============================================================================
// Radio preset picker screen
// ============================================================================

static void create_radio_picker_screen() {
    scr_radio_picker = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_radio_picker, lv_color_black(), 0);

    lv_obj_t *btn_back = lv_button_create(scr_radio_picker);
    lv_obj_set_size(btn_back, 80, 40);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    lv_obj_set_style_text_font(bl, &lv_font_montserrat_16, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, goto_settings, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr_radio_picker);
    lv_label_set_text(title, "Radio Preset");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 100, 18);

    lv_obj_t *scroll = lv_obj_create(scr_radio_picker);
    lv_obj_set_size(scroll, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 70);
    lv_obj_set_pos(scroll, 10, 60);
    lv_obj_set_style_bg_color(scroll, lv_color_make(10, 10, 15), 0);
    lv_obj_set_style_border_width(scroll, 0, 0);
    lv_obj_set_style_radius(scroll, 8, 0);
    lv_obj_set_style_pad_all(scroll, 5, 0);
    lv_obj_set_scroll_dir(scroll, LV_DIR_VER);
    lv_obj_set_flex_flow(scroll, LV_FLEX_FLOW_COLUMN);

    for (int i = 0; i < (int)NUM_RADIO_PRESETS; i++) {
        lv_obj_t *btn = lv_button_create(scroll);
        lv_obj_set_size(btn, SCREEN_WIDTH - 40, 60);
        lv_obj_set_style_bg_color(btn, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(btn, 8, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, lv_color_make(50, 50, 60), 0);
        lv_obj_add_event_cb(btn, on_radio_preset_select, LV_EVENT_CLICKED, (void*)(intptr_t)i);

        lv_obj_t *name_lbl = lv_label_create(btn);
        lv_label_set_text(name_lbl, RADIO_PRESETS[i].name);
        lv_obj_set_style_text_color(name_lbl, lv_color_white(), 0);
        lv_obj_set_style_text_font(name_lbl, &lv_font_montserrat_16, 0);
        lv_obj_align(name_lbl, LV_ALIGN_LEFT_MID, 5, -8);

        char detail[64];
        snprintf(detail, sizeof(detail), "%.3f MHz  BW%.0f  SF%d  CR4/%d",
                 RADIO_PRESETS[i].freq, RADIO_PRESETS[i].bw,
                 RADIO_PRESETS[i].sf, RADIO_PRESETS[i].cr);
        lv_obj_t *det_lbl = lv_label_create(btn);
        lv_label_set_text(det_lbl, detail);
        lv_obj_set_style_text_color(det_lbl, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_obj_set_style_text_font(det_lbl, &lv_font_montserrat_14, 0);
        lv_obj_align(det_lbl, LV_ALIGN_LEFT_MID, 5, 12);
    }
}

// ============================================================================
// Channel picker screen (lifted from old:759-906)
// ============================================================================

static void refresh_channel_picker() {
    Meck* mesh = meck_get_instance();
    if (!obj_ch_picker_scroll || !mesh) return;

    lv_obj_clean(obj_ch_picker_scroll);
    memset(lbl_picker_unread, 0, sizeof(lbl_picker_unread));

    int num_ch = mesh->getActiveChannelCount();
    int y = 5;

    for (int i = 0; i < num_ch && i < 8; i++) {
        ChannelDetails ch;
        if (!mesh->getChannel(i, ch) || ch.name[0] == '\0') continue;

        lv_color_t color = lv_palette_main(CH_COLORS[i % NUM_CH_COLORS]);

        lv_obj_t *btn = lv_button_create(obj_ch_picker_scroll);
        lv_obj_set_size(btn, SCREEN_WIDTH - 40, 75);
        lv_obj_set_pos(btn, 0, y);
        lv_obj_set_style_bg_color(btn, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(btn, 12, 0);
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_border_color(btn, color, 0);
        lv_obj_add_event_cb(btn, goto_channel_n, LV_EVENT_CLICKED, (void*)(intptr_t)i);

        lv_obj_t *lbl_name = lv_label_create(btn);
        lv_label_set_text(lbl_name, ch.name);
        lv_obj_set_style_text_color(lbl_name, color, 0);
        lv_obj_set_style_text_font(lbl_name, &lv_font_montserrat_22, 0);
        lv_obj_align(lbl_name, LV_ALIGN_LEFT_MID, 10, 0);

        lv_obj_t *btn_del = lv_button_create(btn);
        lv_obj_set_size(btn_del, 40, 40);
        lv_obj_align(btn_del, LV_ALIGN_RIGHT_MID, -5, 0);
        lv_obj_set_style_bg_color(btn_del, lv_color_make(80, 20, 20), 0);
        lv_obj_set_style_radius(btn_del, 8, 0);
        lv_obj_add_event_cb(btn_del, on_ch_delete, LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_t *x_lbl = lv_label_create(btn_del);
        lv_label_set_text(x_lbl, LV_SYMBOL_CLOSE);
        lv_obj_set_style_text_color(x_lbl, lv_color_white(), 0);
        lv_obj_center(x_lbl);

        lbl_picker_unread[i] = lv_label_create(btn);
        lv_label_set_text(lbl_picker_unread[i], "");
        lv_obj_set_style_text_color(lbl_picker_unread[i], lv_color_make(100, 255, 100), 0);
        lv_obj_set_style_text_font(lbl_picker_unread[i], &lv_font_montserrat_18, 0);
        lv_obj_align(lbl_picker_unread[i], LV_ALIGN_RIGHT_MID, -55, 0);

        y += 85;
    }

    // "Add Channel" button at the bottom
    lv_obj_t *btn_add = lv_button_create(obj_ch_picker_scroll);
    lv_obj_set_size(btn_add, SCREEN_WIDTH - 40, 60);
    lv_obj_set_pos(btn_add, 0, y);
    lv_obj_set_style_bg_color(btn_add, lv_color_make(20, 40, 20), 0);
    lv_obj_set_style_radius(btn_add, 12, 0);
    lv_obj_set_style_border_width(btn_add, 2, 0);
    lv_obj_set_style_border_color(btn_add, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_add_event_cb(btn_add, on_ch_add_tap, LV_EVENT_CLICKED, NULL);

    lv_obj_t *add_lbl = lv_label_create(btn_add);
    lv_label_set_text(add_lbl, LV_SYMBOL_PLUS " Add Channel");
    lv_obj_set_style_text_color(add_lbl, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(add_lbl, &lv_font_montserrat_22, 0);
    lv_obj_center(add_lbl);
}

static void create_channel_picker_screen() {
    scr_channel_picker = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_channel_picker, lv_color_black(), 0);

    create_back_button(scr_channel_picker);

    lv_obj_t *title = lv_label_create(scr_channel_picker);
    lv_label_set_text(title, "Channels");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 100, 18);

    obj_ch_picker_scroll = lv_obj_create(scr_channel_picker);
    lv_obj_set_size(obj_ch_picker_scroll, SCREEN_WIDTH, SCREEN_HEIGHT - 60);
    lv_obj_set_pos(obj_ch_picker_scroll, 20, 60);
    lv_obj_set_style_bg_opa(obj_ch_picker_scroll, 0, 0);
    lv_obj_set_style_border_width(obj_ch_picker_scroll, 0, 0);
    lv_obj_set_style_pad_all(obj_ch_picker_scroll, 0, 0);
    lv_obj_set_scroll_dir(obj_ch_picker_scroll, LV_DIR_VER);

    refresh_channel_picker();

    // ---- "Add Channel" overlay (hidden by default) ----
    obj_ch_add_panel = lv_obj_create(scr_channel_picker);
    lv_obj_set_size(obj_ch_add_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_ch_add_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_ch_add_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_ch_add_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(obj_ch_add_panel, 0, 0);
    lv_obj_add_flag(obj_ch_add_panel, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *add_title = lv_label_create(obj_ch_add_panel);
    lv_label_set_text(add_title, "Add Channel");
    lv_obj_set_style_text_color(add_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(add_title, &lv_font_montserrat_22, 0);
    lv_obj_align(add_title, LV_ALIGN_TOP_MID, 0, 20);

    lv_obj_t *hint = lv_label_create(obj_ch_add_panel);
    lv_label_set_text(hint, "Enter channel name (e.g. #sydney)");
    lv_obj_set_style_text_color(hint, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 50);

    ta_ch_add = lv_textarea_create(obj_ch_add_panel);
    lv_obj_set_size(ta_ch_add, SCREEN_WIDTH - 40, 50);
    lv_obj_align(ta_ch_add, LV_ALIGN_TOP_MID, 0, 80);
    lv_textarea_set_one_line(ta_ch_add, true);
    lv_textarea_set_max_length(ta_ch_add, 30);
    lv_textarea_set_text(ta_ch_add, "#");
    lv_obj_set_style_bg_color(ta_ch_add, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_ch_add, lv_color_white(), 0);
    lv_obj_set_style_text_font(ta_ch_add, &lv_font_montserrat_18, 0);
    lv_obj_set_style_border_color(ta_ch_add, lv_palette_main(LV_PALETTE_GREEN), 0);

    kb_ch_add = lv_keyboard_create(obj_ch_add_panel);
    lv_obj_set_size(kb_ch_add, SCREEN_WIDTH, 280);
    lv_obj_align(kb_ch_add, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_textarea(kb_ch_add, ta_ch_add);
    lv_obj_add_event_cb(kb_ch_add, on_ch_add_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_ch_add, on_ch_add_kb_event, LV_EVENT_CANCEL, NULL);
}

// ============================================================================
// Messages screen (lifted from old:908-1023)
// ============================================================================

static void create_messages_screen() {
    scr_messages = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_messages, lv_color_black(), 0);

    // Back -> channel picker (NOT home)
    lv_obj_t *btn_back = lv_button_create(scr_messages);
    lv_obj_set_size(btn_back, 80, 40);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    lv_obj_set_style_text_font(bl, &lv_font_montserrat_16, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, goto_channel_picker, LV_EVENT_CLICKED, NULL);

    lbl_msg_channel_name = lv_label_create(scr_messages);
    lv_label_set_text(lbl_msg_channel_name, "#public");
    lv_obj_set_style_text_color(lbl_msg_channel_name, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_text_font(lbl_msg_channel_name, &lv_font_montserrat_22, 0);
    lv_obj_align(lbl_msg_channel_name, LV_ALIGN_TOP_LEFT, 100, 18);

    obj_msg_scroll = lv_obj_create(scr_messages);
    lv_obj_set_size(obj_msg_scroll, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 130);
    lv_obj_set_pos(obj_msg_scroll, 10, 60);
    lv_obj_set_style_bg_color(obj_msg_scroll, lv_color_make(10, 10, 15), 0);
    lv_obj_set_style_border_width(obj_msg_scroll, 0, 0);
    lv_obj_set_style_radius(obj_msg_scroll, 8, 0);
    lv_obj_set_style_pad_all(obj_msg_scroll, 10, 0);
    lv_obj_set_scroll_dir(obj_msg_scroll, LV_DIR_VER);
    // rebuild_message_bubbles() populates this container with one bubble
    // per message and (re)applies a flex column layout. We deliberately
    // don't add a static body label here — the previous implementation
    // dumped all messages into one big label, which couldn't do
    // bubbles/alignment/sender colouring.

    ta_compose = lv_textarea_create(scr_messages);
    lv_obj_set_size(ta_compose, SCREEN_WIDTH - 100, 50);
    lv_obj_align(ta_compose, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_textarea_set_placeholder_text(ta_compose, "Type message...");
    lv_textarea_set_one_line(ta_compose, true);
    lv_textarea_set_max_length(ta_compose, 180);
    lv_obj_set_style_bg_color(ta_compose, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_compose, lv_color_white(), 0);
    lv_obj_set_style_text_font(ta_compose, &lv_font_montserrat_16, 0);
    lv_obj_set_style_border_color(ta_compose, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_border_width(ta_compose, 1, 0);
    lv_obj_set_style_radius(ta_compose, 8, 0);
    lv_obj_add_event_cb(ta_compose, on_compose_focused,   LV_EVENT_FOCUSED,   NULL);
    lv_obj_add_event_cb(ta_compose, on_compose_defocused, LV_EVENT_DEFOCUSED, NULL);

    btn_send = lv_button_create(scr_messages);
    lv_obj_set_size(btn_send, 70, 50);
    lv_obj_align(btn_send, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_bg_color(btn_send, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(btn_send, 8, 0);
    lv_obj_t *send_lbl = lv_label_create(btn_send);
    lv_label_set_text(send_lbl, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(send_lbl, lv_color_black(), 0);
    lv_obj_set_style_text_font(send_lbl, &lv_font_montserrat_22, 0);
    lv_obj_center(send_lbl);
    lv_obj_add_event_cb(btn_send, on_send_clicked, LV_EVENT_CLICKED, NULL);

    kb_compose = lv_keyboard_create(scr_messages);
    lv_obj_set_size(kb_compose, SCREEN_WIDTH, 280);
    lv_obj_align(kb_compose, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_keyboard_set_textarea(kb_compose, ta_compose);
    lv_obj_add_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(kb_compose, on_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_compose, on_kb_event, LV_EVENT_CANCEL, NULL);
}

// ============================================================================
// Load a specific channel into the messages view
// ============================================================================

static void load_channel_view(uint8_t ch_idx) {
    g_active_channel = ch_idx;
    Meck* mesh = meck_get_instance();

    if (lbl_msg_channel_name && mesh) {
        ChannelDetails ch;
        if (mesh->getChannel(ch_idx, ch) && ch.name[0] != '\0') {
            lv_label_set_text(lbl_msg_channel_name, ch.name);
        } else {
            lv_label_set_text(lbl_msg_channel_name, "???");
        }
        lv_color_t color = lv_palette_main(CH_COLORS[ch_idx % NUM_CH_COLORS]);
        lv_obj_set_style_text_color(lbl_msg_channel_name, color, 0);
    }

    rebuild_message_bubbles(ch_idx);

    if (ta_compose) lv_textarea_set_text(ta_compose, "");
    if (kb_compose) lv_obj_add_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
    if (obj_msg_scroll) lv_obj_set_height(obj_msg_scroll, SCREEN_HEIGHT - 130);

    if (mesh) mesh->clearUnread(ch_idx);
    if (scr_messages) lv_screen_load(scr_messages);
}

// ============================================================================
// Contacts screen (lifted from old:1028-1108)
// ============================================================================

static void create_contacts_screen() {
    scr_contacts = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_contacts, lv_color_black(), 0);

    create_back_button(scr_contacts);

    lv_obj_t *title = lv_label_create(scr_contacts);
    lv_label_set_text(title, "Contacts");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 100, 18);

    obj_contacts_scroll = lv_obj_create(scr_contacts);
    lv_obj_set_size(obj_contacts_scroll, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 70);
    lv_obj_set_pos(obj_contacts_scroll, 10, 60);
    lv_obj_set_style_bg_color(obj_contacts_scroll, lv_color_make(10, 10, 15), 0);
    lv_obj_set_style_border_width(obj_contacts_scroll, 0, 0);
    lv_obj_set_style_radius(obj_contacts_scroll, 8, 0);
    lv_obj_set_style_pad_all(obj_contacts_scroll, 5, 0);
    lv_obj_set_scroll_dir(obj_contacts_scroll, LV_DIR_VER);
    lv_obj_set_flex_flow(obj_contacts_scroll, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(obj_contacts_scroll, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
}

static void refresh_contacts_list() {
    Meck* mesh = meck_get_instance();
    if (!obj_contacts_scroll || !mesh) return;

    uint32_t child_cnt = lv_obj_get_child_count(obj_contacts_scroll);
    for (int i = (int)child_cnt - 1; i >= 0; i--) {
        lv_obj_delete(lv_obj_get_child(obj_contacts_scroll, i));
    }

    int n = mesh->getNumContacts();
    if (n == 0) {
        lv_obj_t *empty = lv_label_create(obj_contacts_scroll);
        lv_label_set_text(empty, "No contacts yet.\nAdverts will populate\nthis list.");
        lv_obj_set_style_text_color(empty, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_obj_set_style_text_font(empty, &lv_font_montserrat_16, 0);
        return;
    }

    for (int i = 0; i < n && i < 50; i++) {
        ContactInfo ci;
        if (!mesh->getContactByIdx(i, ci)) continue;

        lv_obj_t *row = lv_button_create(obj_contacts_scroll);
        lv_obj_set_size(row, SCREEN_WIDTH - 40, 55);
        lv_obj_set_style_bg_color(row, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(row, 8, 0);
        lv_obj_set_style_border_width(row, 1, 0);
        lv_obj_set_style_border_color(row,
            (ci.flags & 0x01) ? lv_palette_main(LV_PALETTE_YELLOW)
                              : lv_color_make(50, 50, 60), 0);
        lv_obj_add_event_cb(row, on_contact_tap, LV_EVENT_CLICKED, (void*)(intptr_t)i);

        lv_obj_t *lbl = lv_label_create(row);
        char name_buf[48];
        snprintf(name_buf, sizeof(name_buf), "%s%s",
                 (ci.flags & 0x01) ? "* " : "", ci.name);
        lv_label_set_text(lbl, name_buf);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 10, 0);

        lv_obj_t *pk = lv_label_create(row);
        char pk_buf[16];
        snprintf(pk_buf, sizeof(pk_buf), "%02X%02X",
                 ci.id.pub_key[0], ci.id.pub_key[1]);
        lv_label_set_text(pk, pk_buf);
        lv_obj_set_style_text_color(pk, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_obj_set_style_text_font(pk, &lv_font_montserrat_14, 0);
        lv_obj_align(pk, LV_ALIGN_RIGHT_MID, -10, 0);
    }
}

// ============================================================================
// Contact detail screen (lifted from old:1111-1166)
// ============================================================================

static void create_contact_detail_screen() {
    scr_contact_detail = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_contact_detail, lv_color_black(), 0);

    // Back -> contacts list (NOT home)
    lv_obj_t *btn_back = lv_button_create(scr_contact_detail);
    lv_obj_set_size(btn_back, 80, 40);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    lv_obj_set_style_text_font(bl, &lv_font_montserrat_16, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, goto_contacts_from_detail, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr_contact_detail);
    lv_label_set_text(title, "Contact");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 100, 18);

    lv_obj_t *btn_fav = lv_button_create(scr_contact_detail);
    lv_obj_set_size(btn_fav, 100, 40);
    lv_obj_align(btn_fav, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_bg_color(btn_fav, lv_palette_main(LV_PALETTE_YELLOW), 0);
    lv_obj_set_style_radius(btn_fav, 8, 0);
    lv_obj_t *fav_lbl = lv_label_create(btn_fav);
    lv_label_set_text(fav_lbl, LV_SYMBOL_OK " Fav");
    lv_obj_set_style_text_color(fav_lbl, lv_color_black(), 0);
    lv_obj_set_style_text_font(fav_lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(fav_lbl);
    lv_obj_add_event_cb(btn_fav, on_contact_fav_toggle, LV_EVENT_CLICKED, NULL);

    lv_obj_t *scroll = lv_obj_create(scr_contact_detail);
    lv_obj_set_size(scroll, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 70);
    lv_obj_set_pos(scroll, 10, 60);
    lv_obj_set_style_bg_color(scroll, lv_color_make(10, 10, 15), 0);
    lv_obj_set_style_border_width(scroll, 0, 0);
    lv_obj_set_style_radius(scroll, 8, 0);
    lv_obj_set_style_pad_all(scroll, 10, 0);

    lbl_contact_detail_body = lv_label_create(scroll);
    lv_label_set_text(lbl_contact_detail_body, "");
    lv_obj_set_style_text_color(lbl_contact_detail_body, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_contact_detail_body, &lv_font_montserrat_16, 0);
    lv_label_set_long_mode(lbl_contact_detail_body, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_contact_detail_body, SCREEN_WIDTH - 50);
}

// ============================================================================
// Navigation callbacks (refresh on entry)
// ============================================================================

static void goto_home(lv_event_t *e) {
    if (scr_home) lv_screen_load(scr_home);
}

static void goto_settings(lv_event_t *e) {
    settings_update_labels();
    if (scr_settings) lv_screen_load(scr_settings);
}

static void goto_channel_picker(lv_event_t *e) {
    refresh_channel_picker();
    if (scr_channel_picker) lv_screen_load(scr_channel_picker);
}

static void goto_contacts(lv_event_t *e) {
    refresh_contacts_list();
    if (scr_contacts) lv_screen_load(scr_contacts);
}

// ============================================================================
// 500ms refresh timer
// ============================================================================

static void ui_update_timer_cb(lv_timer_t *t) {
    Meck* mesh = meck_get_instance();

    // Home title: show the user's chosen node name. Refreshed every tick so
    // a rename in Settings shows up without needing to rebuild the screen.
    if (lbl_home_title && mesh) {
        P4NodePrefs* prefs = mesh->getNodePrefs();
        if (prefs && prefs->node_name[0]) {
            lv_label_set_text(lbl_home_title, prefs->node_name);
        } else {
            lv_label_set_text(lbl_home_title, "(no name)");
        }
    }

    // Home tile: unread count
    if (lbl_home_unread && mesh) {
        int unread = mesh->getUnreadCount();
        lv_label_set_text_fmt(lbl_home_unread, "MSG: %d", unread);
        lv_obj_set_style_text_color(lbl_home_unread,
            unread > 0 ? lv_color_make(0, 255, 100)
                       : lv_palette_main(LV_PALETTE_GREY), 0);
    }

    // Home tile: packet counter
    if (lbl_home_packets) {
        uint32_t rx = radio_driver.getPacketsRecv();
        lv_label_set_text_fmt(lbl_home_packets, "RX: %lu", (unsigned long)rx);
    }

    // Radio detail: redraw when the noise floor moves. The actual
    // sampling happens in P4SX1262Radio under the SPI lock; we just
    // mirror the cached value into the UI when it changes.
    {
        int nf = radio_driver.getNoiseFloor();
        if (nf != g_last_noise_floor_displayed) {
            g_last_noise_floor_displayed = nf;
            update_radio_detail_label();
        }
    }

    // Home tile: last RSSI/SNR
    if (lbl_home_rssi && radio_driver.getPacketsRecv() > 0) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%.0f dBm / %.1f dB",
                 radio_driver.getLastRSSI(), radio_driver.getLastSNR());
        lv_label_set_text(lbl_home_rssi, buf);
    }

    // Battery tile: refresh every tick. Voltage is authoritative; if the
    // BQ27220's reported SoC disagrees with the voltage curve by more than
    // 15 points, surface it so the user can decide what to trust.
    if (lbl_battery_detail) {
        if (!meck_battery_available()) {
            lv_label_set_text(lbl_battery_detail, "BQ27220 not detected");
        } else {
            uint16_t mv         = meck_battery_voltage_mv();
            int16_t  ma         = meck_battery_current_ma();
            uint8_t  pct_chip   = meck_battery_pct_from_chip();
            uint8_t  pct_volts  = meck_battery_pct_from_voltage(mv);
            int8_t   temp_c     = meck_battery_temp_c();
            uint16_t rem_mah    = meck_battery_remaining_mah();
            uint16_t full_mah   = meck_battery_full_charge_mah();
            uint16_t tte_min    = meck_battery_time_to_empty_min();

            const char* current_label;
            int16_t     current_abs = ma < 0 ? -ma : ma;
            if (current_abs < 5)      current_label = "idle";
            else if (ma > 0)          current_label = "charging";
            else                      current_label = "discharging";

            char tte_buf[32];
            if (tte_min == 0xFFFF || tte_min == 0 || ma > 0) {
                snprintf(tte_buf, sizeof(tte_buf), "--");
            } else {
                snprintf(tte_buf, sizeof(tte_buf), "%uh %02um",
                         (unsigned)(tte_min / 60),
                         (unsigned)(tte_min % 60));
            }

            int diff = (int)pct_chip - (int)pct_volts;
            if (diff < 0) diff = -diff;
            const char* trust_note = (diff > 15)
                ? "\nNote: BQ27220 SoC disagrees with voltage,\ncell calibration may need attention."
                : "";

            char buf[512];
            snprintf(buf, sizeof(buf),
                "Voltage:    %u mV (~%u%%)\n"
                "Charge:     %u%%  [BQ27220]\n"
                "Current:    %s%d mA  (%s)\n"
                "Temp:       %d C\n\n"
                "Remaining:  %u / %u mAh\n"
                "Time empty: %s%s",
                (unsigned)mv,           (unsigned)pct_volts,
                (unsigned)pct_chip,
                ma > 0 ? "+" : "",      (int)ma,        current_label,
                (int)temp_c,
                (unsigned)rem_mah,      (unsigned)full_mah,
                tte_buf,
                trust_note);
            lv_label_set_text(lbl_battery_detail, buf);
        }
    }

    // Recent Heard tile
    if (lbl_recent_list && mesh && mesh->isRecentDirty()) {
        P4RecentHeard recent[8];
        int n = mesh->getRecentHeard(recent, 8);
        if (n == 0) {
            lv_label_set_text(lbl_recent_list, "Waiting for adverts...");
        } else {
            char log_buf[800];
            int pos = 0;
            for (int i = 0; i < n; i++) {
                char timebuf[32];
                format_local_time(recent[i].timestamp, timebuf, sizeof(timebuf));
                int w = snprintf(log_buf + pos, sizeof(log_buf) - pos,
                    "%s  [%02X%02X]\n  RSSI %.0f  SNR %.1f  hops %s\n  %s\n\n",
                    recent[i].name,
                    recent[i].pub_key_prefix[0], recent[i].pub_key_prefix[1],
                    recent[i].rssi, recent[i].snr,
                    recent[i].path_len == 0xFF ? "direct" : "flood",
                    timebuf);
                if (w > 0) pos += w;
            }
            log_buf[pos] = '\0';
            lv_label_set_text(lbl_recent_list, log_buf);
        }
    }

    // Messages screen: live update for active channel. Rebuild the bubble
    // list whenever the mesh signals new content. obj_msg_scroll is the
    // sentinel — if it exists, the messages screen has been built.
    if (obj_msg_scroll && mesh && mesh->isMessageDirty()) {
        rebuild_message_bubbles(g_active_channel);
    }

    // Channel picker unread badges
    if (mesh) {
        int num_ch = mesh->getActiveChannelCount();
        for (int i = 0; i < num_ch && i < 8; i++) {
            if (!lbl_picker_unread[i]) continue;
            int unread = mesh->getUnreadCount(i);
            if (unread > 0) {
                char tmp[16];
                snprintf(tmp, sizeof(tmp), "%d new", unread);
                lv_label_set_text(lbl_picker_unread[i], tmp);
            } else {
                lv_label_set_text(lbl_picker_unread[i], "");
            }
        }
    }

    // GPS tile: refresh detail block at 500ms tick.
    // Data is populated by device_gps_task in main.cpp at ~1Hz, so we may
    // see the same values across two consecutive ticks — that's fine.
    // Sentence rate is computed as a delta between consecutive timer ticks,
    // smoothed with a running average over 4 samples (~2 sec window) so
    // the displayed rate doesn't jitter.
    if (lbl_gps_detail) {
        MeckGpsSnapshot snap;
        meck_gps_get_snapshot(&snap);

        if (!snap.init_ok) {
            lv_label_set_text(lbl_gps_detail, "L76K not detected");
        } else {
            // --- Sentence rate calc (smoothed) ---
            static uint32_t prev_count = 0;
            static uint32_t prev_ms    = 0;
            static float    rate_avg   = 0.0f;
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
            if (prev_ms != 0 && now_ms > prev_ms && snap.sentence_count >= prev_count) {
                uint32_t delta_count = snap.sentence_count - prev_count;
                uint32_t delta_ms    = now_ms - prev_ms;
                float    instant_rate = (delta_count * 1000.0f) / delta_ms;
                // Exponential moving average, alpha=0.25 (4-sample equivalent)
                rate_avg = (rate_avg == 0.0f)
                    ? instant_rate
                    : (rate_avg * 0.75f + instant_rate * 0.25f);
            }
            prev_count = snap.sentence_count;
            prev_ms    = now_ms;

            // Status — surface the location_status char + init flag
            const char *status_str;
            switch (snap.status) {
                case 'A': status_str = "Active";       break;
                case 'V': status_str = "Acquiring";    break;
                case 'D': status_str = "Differential"; break;
                default:  status_str = "Unknown";      break;
            }

            // Fix — combine GGA mode with RMC status. Show TTFF once we have it.
            char fix_str[64];
            if (!snap.fix_valid) {
                snprintf(fix_str, sizeof(fix_str), "No fix");
            } else {
                const char *mode_str;
                switch (snap.gps_mode) {
                    case 1:  mode_str = "GPS";  break;
                    case 2:  mode_str = "DGPS"; break;
                    case 6:  mode_str = "DR";   break;
                    default: mode_str = "?";    break;
                }
                snprintf(fix_str, sizeof(fix_str), "%s  TTFF %us",
                    mode_str, (unsigned)snap.time_to_first_fix_s);
            }

            // Satellites — visible/used split + HDOP if meaningful.
            // "Visible" comes from GSV (heard but not necessarily locked);
            // "used" comes from GGA (contributing to current fix). Big gap
            // between visible and used = marginal sky view or RF problem.
            char sat_str[80];
            if (snap.hdop > 0.0f && snap.hdop < 25.0f) {
                snprintf(sat_str, sizeof(sat_str),
                    "%u visible, %u used  HDOP %.1f",
                    (unsigned)snap.sats_visible,
                    (unsigned)snap.sats_used, snap.hdop);
            } else {
                snprintf(sat_str, sizeof(sat_str),
                    "%u visible, %u used",
                    (unsigned)snap.sats_visible,
                    (unsigned)snap.sats_used);
            }

            // Position — decimal degrees, 6dp (~11cm precision)
            char pos_str[64];
            if (snap.fix_valid) {
                snprintf(pos_str, sizeof(pos_str), "%.6f, %.6f",
                    snap.lat_e7 / 1e7, snap.lon_e7 / 1e7);
            } else {
                snprintf(pos_str, sizeof(pos_str), "--");
            }

            // Altitude — meters MSL
            char alt_str[32];
            if (snap.altitude_valid && snap.fix_valid) {
                snprintf(alt_str, sizeof(alt_str), "%.1f m", snap.altitude_m);
            } else {
                snprintf(alt_str, sizeof(alt_str), "--");
            }

            // Sentences — total count + smoothed rate
            char sent_str[48];
            snprintf(sent_str, sizeof(sent_str), "%u  (%.0f/sec)",
                (unsigned)snap.sentence_count, rate_avg);

            // UTC time captured from RMC. Only meaningful once GPS has
            // decoded real time; year >= 2024 rejects pre-lock garbage.
            char time_str[40];
            if (snap.time_valid && snap.year >= 2024) {
                snprintf(time_str, sizeof(time_str),
                    "%04u-%02u-%02u %02u:%02u:%02uz",
                    (unsigned)snap.year, (unsigned)snap.month, (unsigned)snap.day,
                    (unsigned)snap.hour, (unsigned)snap.minute,
                    (unsigned)snap.second);
            } else {
                snprintf(time_str, sizeof(time_str), "--");
            }

            // Soft RTC sync indicator. 0 means it has never been set; any
            // non-zero value means device_gps_task has pushed at least once.
            char rtc_str[40];
            uint32_t rtc_now = meck_clock_get_utc();
            if (rtc_now == 0) {
                snprintf(rtc_str, sizeof(rtc_str), "not set");
            } else {
                snprintf(rtc_str, sizeof(rtc_str), "set (epoch %u)",
                    (unsigned)rtc_now);
            }

            char buf[512];
            snprintf(buf, sizeof(buf),
                "Status:     %s\n"
                "Fix:        %s\n"
                "Satellites: %s\n"
                "Position:   %s\n"
                "Altitude:   %s\n"
                "UTC:        %s\n"
                "RTC:        %s\n"
                "Sentences:  %s",
                status_str, fix_str, sat_str, pos_str, alt_str,
                time_str, rtc_str, sent_str);
            lv_label_set_text(lbl_gps_detail, buf);
        }
    }
}

// ============================================================================
// Public API
// ============================================================================

extern "C" void meck_ui_init() {
    printf("MeckUI: building home screen\n");

    _lock_acquire(&lvgl_api_lock);

    scr_home = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_home, lv_color_black(), 0);

    g_tileview = lv_tileview_create(scr_home);
    lv_obj_set_size(g_tileview, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_style_bg_color(g_tileview, lv_color_black(), 0);

    lv_obj_t *t_home     = lv_tileview_add_tile(g_tileview, 0, 0, LV_DIR_RIGHT);
    lv_obj_t *t_recent   = lv_tileview_add_tile(g_tileview, 1, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));
    lv_obj_t *t_radio    = lv_tileview_add_tile(g_tileview, 2, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));
    lv_obj_t *t_advert   = lv_tileview_add_tile(g_tileview, 3, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));
    lv_obj_t *t_gps      = lv_tileview_add_tile(g_tileview, 4, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));
    lv_obj_t *t_battery  = lv_tileview_add_tile(g_tileview, 5, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));
    lv_obj_t *t_shutdown = lv_tileview_add_tile(g_tileview, 6, 0, LV_DIR_LEFT);

    create_page_home(t_home);
    create_page_recent(t_recent);
    create_page_radio(t_radio);
    create_page_advert(t_advert);
    create_page_gps(t_gps);
    create_page_battery(t_battery);
    create_page_shutdown(t_shutdown);

    create_settings_screen();
    create_radio_picker_screen();
    create_channel_picker_screen();
    create_messages_screen();
    create_contacts_screen();
    create_contact_detail_screen();

    lv_timer_create(ui_update_timer_cb, 500, NULL);

    _lock_release(&lvgl_api_lock);

    printf("MeckUI: all screens ready (home + 6 tiles + settings/picker/messages/contacts)\n");
}

extern "C" void meck_ui_show_home() {
    if (!scr_home) {
        printf("MeckUI: home screen not initialized, call meck_ui_init() first\n");
        return;
    }
    _lock_acquire(&lvgl_api_lock);
    lv_screen_load(scr_home);
    _lock_release(&lvgl_api_lock);
    printf("MeckUI: home screen loaded\n");
}
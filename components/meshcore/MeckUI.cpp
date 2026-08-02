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
 * (createChannel, deleteChannel) are safe to call directly from LVGL.
 */

#include "MeckUI.h"
#include "meck.h"
#include "target.h"
#include "MeckMesh.h"
#include "MeckDataStore.h"
#include "MeckAudioUI.h"
#include "MeckReaderUI.h"
#include "MeckNotesUI.h"
#include "MeckAudio.h"
#include "MeckVoice.h"

// Voice bridge functions (defined in meck_app.cpp)
extern "C" void meck_drain_pending_voice(void);
extern "C" void meck_drain_pending_pictures(void);
extern "C" void meck_request_voice_send(int contact_idx);
extern "C" void meck_request_picture_send(uint8_t ch_idx, const char *filepath);
extern MeckVoice* meck_get_voice_instance(void);
extern "C" int meck_voice_send_get_status(void);
#define VOICE_SEND_IDLE           0
#define VOICE_SEND_WAITING_ACK    1
#define VOICE_SEND_PACKETS_QUEUED 2
#define VOICE_SEND_ACK_TIMEOUT    3
#define VOICE_SEND_NO_PATH        4
#include "NotifSounds.h"
#include "MeckMapScreen.h"

#include <sys/lock.h>
#include <algorithm>
#include "lvgl.h"
#include "MeckEmojiPicker.h"
#include "MeckEmoji.h"
#include "t_display_p4_driver.h"
#include "sdkconfig.h"
#include "MeckCardKB.h"
#include "MeckP4Keyboard.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

// ---------------------------------------------------------------------------
// Hardware keyboard presence (T-Display-P4-Keyboard expansion board).
//
// Set by meck_p4kbd_init() near the end of meck_ui_init once the TCA8418 has
// answered. Declared here rather than beside the driver block because the
// composer focus handlers, which decide whether to reveal the on-screen
// keyboard, sit several thousand lines earlier in this file.
// ---------------------------------------------------------------------------
#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
static bool g_p4kbd_present = false;
static inline bool meck_hw_keyboard_active(void) { return g_p4kbd_present; }
#else
static inline bool meck_hw_keyboard_active(void) { return false; }
#endif

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <dirent.h>
#include <sys/stat.h>

#ifndef PICTURES_FOLDER
#define PICTURES_FOLDER "/sdcard/pictures"
#endif
#include <cstdlib>
#include <ctime>
#include <climits>
#include <cstdarg>   // va_list, va_start, va_end (meck_debug_log_printf)
#include <cerrno>    // errno (Debug Logs fopen error reporting)

// ----------------------------------------------------------------------------
// Orientation-aware logical dimensions
// ----------------------------------------------------------------------------
// The display can be rotated at runtime: lv_display_set_rotation() drives the
// PPA in main.cpp's flush callback, and when rotated 90/270 LVGL reports a
// swapped logical resolution. The UI here is laid out proportionally from
// SCREEN_WIDTH / SCREEN_HEIGHT, so within THIS translation unit we redefine
// them to read the live logical resolution instead of the fixed panel macros
// from t_display_p4_driver.h. At the default 0deg rotation these return the
// same 568x1232, so portrait layout is byte-for-byte unchanged; rotating swaps
// them and every proportional site adapts.
//
// Scope note: this only affects MeckUI.cpp. main.cpp keeps the physical macros
// for draw-buffer/PPA sizing (correct), and the other UI translation units
// (audio/map/reader) still use the fixed macros until they adopt the same
// accessor via a shared header. All SCREEN_WIDTH/SCREEN_HEIGHT uses here are in
// function bodies, so a function-valued macro is safe.
static inline int32_t meck_disp_w() {
    return lv_display_get_horizontal_resolution(lv_display_get_default());
}
static inline int32_t meck_disp_h() {
    return lv_display_get_vertical_resolution(lv_display_get_default());
}
// Keyboard height stays proportional to the screen (portrait baseline was
// 560 px on the 1232-tall panel) so it remains usable when rotated to
// landscape (where a fixed 560 would swallow the whole screen).
static inline int32_t meck_kb_height() {
    return 560 * meck_disp_h() / 1232;
}
#undef SCREEN_WIDTH
#undef SCREEN_HEIGHT
#define SCREEN_WIDTH  (meck_disp_w())
#define SCREEN_HEIGHT (meck_disp_h())

extern _lock_t lvgl_api_lock;

// Custom Meck Montserrat fonts with extended Latin coverage. These are
// generated by lv_font_conv from Montserrat-Medium (same weight LVGL's
// built-in lv_font_montserrat_* uses), with the codepoint range expanded
// to include Latin-1 Supplement (0xA0-0xFF) plus the Czech/Slovak Latin
// Extended-A glyphs we need for the long-press accent popover:
// Č/č (010C/010D), Ě/ě (011A/011B), ň (0148), Ř/ř (0158/0159),
// š (0161), Ž/ž (017D/017E). Visually identical to LVGL's defaults for
// ASCII text — they only differ in glyph coverage. Defined in
// meck_montserrat_<size>.c at the component root.
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

// LVGL doesn't ship a LV_SYMBOL_LOCK in its standard set, but the
// FontAwesome lock glyph (U+F023, "fa-lock") is included in the
// meck_montserrat_* fonts. UTF-8 encoding of U+F023 = EF 80 A3.
#ifndef LV_SYMBOL_LOCK
#define LV_SYMBOL_LOCK "\xEF\x80\xA3"
#endif

// ----------------------------------------------------------------------------
// Font scale registry — live re-style of every text object on toggle
// ----------------------------------------------------------------------------
// LVGL bakes the font pointer into each object's style at the time of the
// lv_obj_set_style_text_font call, so changing P4NodePrefs::font_scale later
// has no retroactive effect. Instead of doing that call directly, screen
// builders use meck_set_font(obj, base, part). That installs the font now
// (calling meck_font(base) to honour the current scale) and remembers the
// triple so meck_font_restyle_all() can walk the registry on toggle and
// reapply the right scaled font to every entry. LV_EVENT_DELETE removes
// dead entries when LVGL frees the underlying object.
struct MeckFontReg {
    lv_obj_t* obj;
    const lv_font_t* base;
    lv_style_selector_t part;
};
#define MECK_FONT_REG_MAX 1024
static MeckFontReg g_font_reg[MECK_FONT_REG_MAX];
static int g_font_reg_count = 0;

static const lv_font_t* meck_font(const lv_font_t* base) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return base;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs || prefs->font_scale == 0) return base;
    if (prefs->font_scale == 1) {
        // Larger: bump to next size. 32 is the ceiling.
        if (base == &meck_montserrat_14) return &meck_montserrat_16;
        if (base == &meck_montserrat_16) return &meck_montserrat_18;
        if (base == &meck_montserrat_18) return &meck_montserrat_22;
        if (base == &meck_montserrat_22) return &meck_montserrat_24;
        if (base == &meck_montserrat_24) return &meck_montserrat_28;
        if (base == &meck_montserrat_28) return &meck_montserrat_30;
        return base;
    }
    // font_scale == 2 (Extra Large): double-jump everywhere. New sizes
    // 30 and 32 mean even 24 and 28 get a real second step now.
    if (base == &meck_montserrat_14) return &meck_montserrat_18;
    if (base == &meck_montserrat_16) return &meck_montserrat_22;
    if (base == &meck_montserrat_18) return &meck_montserrat_24;
    if (base == &meck_montserrat_22) return &meck_montserrat_28;
    if (base == &meck_montserrat_24) return &meck_montserrat_30;
    if (base == &meck_montserrat_28) return &meck_montserrat_32;
    return base;
}

static const char* meck_font_scale_name(uint8_t scale) {
    switch (scale) {
        case 0:  return "Classic";
        case 1:  return "Larger";
        case 2:  return "Extra Large";
        default: return "?";
    }
}

// Pick a font for the home-screen node name based on its character count.
// Long names get a smaller font so they fit on the panel without being
// obscured by the camera punch-hole or running off the right edge.
// Thresholds: ≤12 → 28pt, 13-17 → 24pt, 18-19 → 22pt, ≥20 → 18pt.
static const lv_font_t* meck_node_name_font(const char* name) {
    if (!name) return &meck_montserrat_24;
    size_t len = strlen(name);
    if (len <= 12) return &meck_montserrat_28;
    if (len <= 17) return &meck_montserrat_24;
    if (len <= 19) return &meck_montserrat_22;
    return &meck_montserrat_18;
}

static void meck_font_reg_remove_cb(lv_event_t* e) {
    lv_obj_t* target = (lv_obj_t*)lv_event_get_target(e);
    for (int i = 0; i < g_font_reg_count; i++) {
        if (g_font_reg[i].obj == target) {
            g_font_reg[i] = g_font_reg[g_font_reg_count - 1];
            g_font_reg_count--;
            return;
        }
    }
}

static void meck_set_font(lv_obj_t* obj, const lv_font_t* base, lv_style_selector_t part) {
    lv_obj_set_style_text_font(obj, meck_font(base), part);
    if (g_font_reg_count < MECK_FONT_REG_MAX) {
        g_font_reg[g_font_reg_count].obj  = obj;
        g_font_reg[g_font_reg_count].base = base;
        g_font_reg[g_font_reg_count].part = part;
        g_font_reg_count++;
        lv_obj_add_event_cb(obj, meck_font_reg_remove_cb, LV_EVENT_DELETE, NULL);
    }
}

// Exposed to sibling UI modules (e.g. MeckReaderUI) so screens built outside
// this file can register their labels with the font-scale system and honour
// the Settings font-size preference like the rest of the UI.
extern "C" void meck_ui_set_font(lv_obj_t* obj, const lv_font_t* base,
                                 lv_style_selector_t part) {
    meck_set_font(obj, base, part);
}

// Update an already-registered object's base font (or register fresh if
// not present). Use when an object's underlying base font needs to change
// at runtime — e.g. the home node name picks its base from name length, so
// renaming the node may bump it from 24 to 22.
static void meck_update_font(lv_obj_t* obj, const lv_font_t* base, lv_style_selector_t part) {
    lv_obj_set_style_text_font(obj, meck_font(base), part);
    for (int i = 0; i < g_font_reg_count; i++) {
        if (g_font_reg[i].obj == obj && g_font_reg[i].part == part) {
            g_font_reg[i].base = base;
            return;
        }
    }
    if (g_font_reg_count < MECK_FONT_REG_MAX) {
        g_font_reg[g_font_reg_count].obj  = obj;
        g_font_reg[g_font_reg_count].base = base;
        g_font_reg[g_font_reg_count].part = part;
        g_font_reg_count++;
        lv_obj_add_event_cb(obj, meck_font_reg_remove_cb, LV_EVENT_DELETE, NULL);
    }
}

static void meck_font_restyle_all() {
    for (int i = 0; i < g_font_reg_count; i++) {
        lv_obj_set_style_text_font(g_font_reg[i].obj,
                                   meck_font(g_font_reg[i].base),
                                   g_font_reg[i].part);
    }
}

#if defined(CONFIG_SCREEN_TYPE_RM69A10)
  #define MECK_IS_AMOLED  1
  #define NOTCH_SAFE_X  30
  #define NOTCH_SAFE_Y  35
#else
  #define MECK_IS_AMOLED  0
  #define NOTCH_SAFE_X  20
  #define NOTCH_SAFE_Y  20
#endif

// Firmware identity is defined in meck.h (MECK_FIRMWARE_NAME, MECK_FIRMWARE_VERSION).

// Auto-add config bits in P4NodePrefs::autoadd_config. Same bit layout as
// upstream Meck so a future prefs sync between firmwares stays sane. Bit 0
// (overwrite-oldest) is independent of the per-type bits.
#define AUTO_ADD_OVERWRITE_OLDEST (1 << 0)   // 0x01
#define AUTO_ADD_CHAT             (1 << 1)   // 0x02
#define AUTO_ADD_REPEATER         (1 << 2)   // 0x04
#define AUTO_ADD_ROOM_SERVER      (1 << 3)   // 0x08
#define AUTO_ADD_SENSOR           (1 << 4)   // 0x10
#define AUTO_ADD_ALL_TYPES        (AUTO_ADD_CHAT | AUTO_ADD_REPEATER | \
                                   AUTO_ADD_ROOM_SERVER | AUTO_ADD_SENSOR)

// Contact-mode picker values. Maps to (manual_add_contacts, autoadd_config)
// pairs — see settings_contacts_apply_mode() / settings_contacts_get_mode().
#define CONTACT_MODE_AUTO_ALL 0   // every type, auto-added
#define CONTACT_MODE_CUSTOM   1   // per-type bits in autoadd_config
#define CONTACT_MODE_MANUAL   2   // no auto-add at all
#define CONTACT_MODE_COUNT    3

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

// Screen brightness levels — eight-step ladder spanning the full visible
// range. Lowest level (32) is dim but readable in a dark room; highest
// (255) is full HBM. Cycle order matches the array order.
// Brightness slider range is 12-100% (raw 31-255). 12% is the lowest
// step where the AMOLED is still comfortably visible; below that the
// panel goes too dim to read.
#define MECK_BRIGHTNESS_MIN_PCT 12
#define MECK_BRIGHTNESS_MAX_PCT 100

// Auto screen-off timeouts in minutes. 0 means "never". User cycles
// forward through the ladder; matches the convention of every other
// "tap to cycle" setting on this device.
static const uint8_t SCREEN_OFF_OPTIONS[] = { 0, 1, 2, 5, 10, 30 };
#define NUM_SCREEN_OFF_OPTIONS 6

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

#define MECK_HOME_TILE_COUNT 12
static lv_obj_t *tile_buttons[MECK_HOME_TILE_COUNT] = {};
static int       tile_button_count = 0;

// ============================================================================
// LVGL objects (file-scope so the refresh timer can update them)
// ============================================================================

static lv_obj_t *scr_home          = NULL;
static lv_obj_t *g_tileview        = NULL;
// The seven home tiles, kept so the active page index can be resolved.
// lv_tileview_get_tile_act() hands back the tile object but not its column, and
// the objects are otherwise locals in meck_ui_build_screens().
// PAGE count, not MECK_HOME_TILE_COUNT: a second same-named define here
// silently redefined the button-grid macro above to 7, capping button
// registration so Trace, Audio and Web were all styled as index 7 (red).
#define MECK_HOME_PAGE_COUNT 7
static lv_obj_t *g_home_tiles[MECK_HOME_PAGE_COUNT] = { NULL };

// Web reader browser screen (Stage 3)
static lv_obj_t   *scr_web          = NULL;
static lv_obj_t   *lbl_web_content  = NULL;
#define WEB_RECOLOR_CAP 32768   // PSRAM recolor buffer (~2x the 16 KB text cap)
static char       *g_web_recolor_buf = NULL;   // recolored copy of page text
static lv_obj_t   *lbl_web_status   = NULL;
static lv_timer_t *g_web_poll_timer = NULL;
static char        g_web_current_url[256] = {0};
static lv_obj_t   *btn_web_links       = NULL;
static lv_obj_t   *lbl_web_links       = NULL;
static lv_obj_t   *obj_web_link_panel  = NULL;
static lv_obj_t   *obj_web_link_scroll = NULL;
static lv_obj_t   *btn_web_go          = NULL;
static lv_obj_t   *obj_web_url_panel   = NULL;
static lv_obj_t   *ta_web_url          = NULL;
static lv_obj_t   *kb_web_url          = NULL;
static lv_obj_t   *btn_web_addbm       = NULL;
static lv_obj_t   *obj_web_search_panel = NULL;
static lv_obj_t   *ta_web_search       = NULL;
static lv_obj_t   *kb_web_search       = NULL;
static lv_obj_t   *obj_web_home        = NULL;
static lv_obj_t   *obj_web_home_scroll = NULL;
static lv_obj_t   *lbl_web_toast       = NULL;
static lv_timer_t *g_web_toast_timer   = NULL;
static lv_obj_t   *lbl_web_wip_toast   = NULL;
static lv_timer_t *g_web_wip_timer     = NULL;

// Web form fill (Stage 5): a "Forms (N)" button, a selection panel for pages
// with more than one form, and a fill overlay with one editable textarea per
// visible field sharing a single Meck keyboard.
#define WEB_FILL_MAX_FIELDS 16   // mirrors WEB_MAX_FORM_FIELDS in MeckWebHtml.h
static lv_obj_t   *btn_web_forms       = NULL;
static lv_obj_t   *lbl_web_forms       = NULL;
static lv_obj_t   *obj_web_form_panel  = NULL;
static lv_obj_t   *obj_web_form_scroll = NULL;
static lv_obj_t   *obj_web_fill_panel  = NULL;
static lv_obj_t   *obj_web_fill_scroll = NULL;
static lv_obj_t   *kb_web_fill         = NULL;
static lv_obj_t   *lbl_web_fill_title  = NULL;
static lv_obj_t   *g_fill_ta[WEB_FILL_MAX_FIELDS]        = {0};  // shown field textareas
static int         g_fill_field_idx[WEB_FILL_MAX_FIELDS] = {0};  // textarea -> form field index
static int         g_fill_ta_count     = 0;
static int         g_fill_form_idx     = -1;                     // form being filled

// Home tile header labels
static lv_obj_t *lbl_home_title    = NULL;
// Clock + battery labels are mirrored on every tileview page (7 pages in the
// home tileview). The ui_update_timer_cb walks both arrays each interval.
static lv_obj_t *lbl_home_clock[MECK_HOME_PAGE_COUNT]   = {};
static lv_obj_t *lbl_home_battery[MECK_HOME_PAGE_COUNT] = {};
static lv_obj_t *lbl_home_audio[MECK_HOME_PAGE_COUNT]   = {};

// Header refresh counters for ui_update_timer_cb (500ms/tick): the clock fills
// once a minute (120 ticks), the battery once every five minutes (600 ticks).
// File-scope so the orientation rebuild can reset them to their period and have
// the next tick repopulate the freshly-rebuilt (empty) labels immediately,
// instead of leaving them blank until the next wrap.
static int clock_ticks   = 120;
static int battery_ticks = 600;

// Battery header display mode: false = percentage, true = voltage. Toggled
// by tapping any battery label; applies to every mirrored copy.
static bool g_batt_show_voltage = false;

// Clock + battery labels for every non-home screen and modal. Slot 0..7
// reserved for full screens (Settings, Settings/Contacts, Radio Picker,
// Channel Picker, Messages, Contacts, Contact Detail, Discover); slot
// 8..9 for the two semi-transparent overlay modals (name-edit, channel-add).
// Slot 10 = Trace Path screen, slot 11 = Path Editor screen.
// Slots 12-13 = Channels settings + Channel detail screens.
// Slots 15-17 = Admin sub-screens (placeholder, fw info, neighbours).
// Updated by the same timer block that drives the home page arrays.
#define MECK_SCREEN_HOST_COUNT 20
static lv_obj_t *lbl_screen_clock[MECK_SCREEN_HOST_COUNT]   = {};
static lv_obj_t *lbl_screen_battery[MECK_SCREEN_HOST_COUNT] = {};
static lv_obj_t *lbl_screen_audio[MECK_SCREEN_HOST_COUNT]   = {};

static lv_obj_t *lbl_home_unread   = NULL;
static lv_obj_t *lbl_home_ble_pin = NULL;

// Detail tile labels
static lv_obj_t *lbl_recent_list   = NULL;
static lv_obj_t *lbl_radio_detail  = NULL;
static lv_obj_t *lbl_advert_status = NULL;
static lv_obj_t *lbl_gps_detail    = NULL;
static lv_obj_t *lbl_battery_detail = NULL;
#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
// Keyboard-pack column on the battery tile. One gauge serves both batteries
// (the selector puts exactly one of them on the rail), so g_p4batt_source is
// the user's declaration of the physical switch position: 0 = internal cell,
// 1 = keyboard pack. It decides which column shows the live readings.
static lv_obj_t *lbl_kbd_batt_header = NULL;
static lv_obj_t *lbl_kbd_batt_detail = NULL;
static lv_obj_t *lbl_kbd_batt_cap    = NULL;
static uint8_t   g_p4batt_source  = 0;
static uint16_t  g_p4batt_cap_mah = 5000;   // declared pack capacity (1 or 2 cells)
#endif

// Noise floor display cache. The estimator itself lives in P4SX1262Radio
// (sampled every 2 s under the SPI lock via SX126x GetRssiInst, opcode
// 0x15). We cache the last-displayed value here so we only redraw the
// radio detail label when it actually changes.
static int g_last_noise_floor_displayed = INT_MIN;

// Settings screen
static lv_obj_t *scr_settings      = NULL;
static lv_obj_t *lbl_set_name      = NULL;
static lv_obj_t *lbl_set_freq      = NULL;
static lv_obj_t *lbl_set_bw        = NULL;
static lv_obj_t *lbl_set_sf        = NULL;
static lv_obj_t *lbl_set_cr        = NULL;
static lv_obj_t *lbl_set_radio     = NULL;
static lv_obj_t *lbl_set_txpower   = NULL;
static lv_obj_t *lbl_set_utc       = NULL;
static lv_obj_t *lbl_set_pathhash  = NULL;
static lv_obj_t *lbl_set_homecolor = NULL;

// Debug Logs subpage (Settings > Debug Logs)
// Captures Meck source printf calls (via meck_log.h macro substitution) to
// /sdcard/meshcore/logs/log_<unix>.txt while active. When Start is tapped
// the file is fopen'd and g_debug_log_active flips true; meck_debug_log_printf
// then writes to the file instead of UART. Sync write per line under a
// recursive mutex so cross-task printf calls serialise on the FILE*.
static lv_obj_t *scr_debug_logs        = NULL;
static lv_obj_t *lbl_set_debug_logs    = NULL;  // value label on the Settings row

// "Not yet implemented" placeholder screen — shared by Reader, Notes, Web,
// Voice, Camera tiles. Title label is updated per-tile before screen load.
static lv_obj_t *scr_not_implemented   = NULL;
static lv_obj_t *lbl_not_impl_title    = NULL;
static lv_obj_t *lbl_debug_status      = NULL;  // status text on the subpage
static lv_obj_t *btn_debug_start       = NULL;
static lv_obj_t *btn_debug_stop        = NULL;
static lv_obj_t *btn_debug_export      = NULL;

static bool              g_debug_log_active = false;
static FILE             *g_debug_log_fp    = nullptr;
static SemaphoreHandle_t g_debug_log_mutex = nullptr;  // recursive
static char g_debug_log_path[96]    = "";     // most-recent session file path
static char g_debug_export_path[96] = "";     // most-recent export path (after Export tap)
static lv_obj_t *slider_set_brightness   = NULL;
static lv_obj_t *lbl_set_brightness_pct  = NULL;
#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
static lv_obj_t *slider_set_kb_backlight  = NULL;
static lv_obj_t *lbl_set_kb_backlight_pct = NULL;
#endif
static lv_obj_t *lbl_set_screen_off  = NULL;
static lv_obj_t *lbl_set_kb_theme    = NULL;
static lv_obj_t *lbl_set_kb_layout   = NULL;
static lv_obj_t *lbl_set_font_scale  = NULL;
#if MECK_BLE_ENABLED
static lv_obj_t *lbl_set_ble         = NULL;
#endif
static lv_obj_t *lbl_set_drafts      = NULL;
static lv_obj_t *lbl_set_antenna     = NULL;
static lv_obj_t *lbl_set_orientation = NULL;

// WiFi settings sub-screen (Settings > WiFi Companion)
static lv_obj_t *scr_settings_wifi        = NULL;
// BLE keyboard pairing sub-screen (Settings > BLE Keyboard)
#if MECK_BLE_KEYBOARD_ENABLED
static lv_obj_t   *scr_settings_keyboard  = NULL;
static lv_obj_t   *lbl_set_kbd_toggle     = NULL;
static lv_obj_t   *lbl_set_kbd_status     = NULL;
static lv_obj_t   *obj_kbd_scan_scroll    = NULL;
static lv_timer_t *g_kbd_scan_timer       = NULL;
#endif
static lv_obj_t *lbl_set_wifi_toggle      = NULL;
static lv_obj_t *lbl_set_wifi_ssid        = NULL;
static lv_obj_t *lbl_set_wifi_pass        = NULL;
static lv_obj_t *lbl_set_wifi_ip          = NULL;
static lv_obj_t *obj_wifi_edit_panel      = NULL;
static lv_obj_t *ta_wifi_edit             = NULL;
static lv_obj_t *kb_wifi_edit             = NULL;
static int       g_wifi_edit_field        = 0;  // 0=ssid, 1=password
static lv_obj_t *lbl_set_identity  = NULL;
static lv_obj_t *obj_name_edit_panel = NULL;
static lv_obj_t *ta_settings_name    = NULL;
static lv_obj_t *kb_settings         = NULL;

// Numeric edit overlay (shared by Frequency and Spreading Factor text editors)
static lv_obj_t *obj_num_edit_panel = NULL;
static lv_obj_t *ta_num_edit        = NULL;
static lv_obj_t *kb_num_edit        = NULL;
static int        g_num_edit_field  = 0;  // 0 = freq, 1 = SF, 2 = BW

// Default Region edit overlay (text editor for region scope name)
static lv_obj_t *obj_region_edit_panel = NULL;
static lv_obj_t *ta_region_edit        = NULL;
static lv_obj_t *kb_region_edit        = NULL;
static lv_obj_t *lbl_set_region        = NULL;

// Channels settings sub-screen (Settings > Channels)
static lv_obj_t *scr_settings_channels   = NULL;
static lv_obj_t *obj_ch_settings_scroll  = NULL;

// Channel detail sub-screen (Settings > Channels > [channel])
static lv_obj_t *scr_channel_detail      = NULL;
static lv_obj_t *lbl_ch_detail_name      = NULL;
static lv_obj_t *lbl_ch_detail_scope     = NULL;
static lv_obj_t *lbl_ch_detail_notif     = NULL;
static lv_obj_t *lbl_ch_detail_delete    = NULL;
static int        g_detail_channel_idx   = -1;
static uint32_t   g_delete_confirm_until = 0;   // millis deadline for tap-again

// Channel scope edit overlay (on channel detail screen)
static lv_obj_t *obj_ch_scope_edit_panel = NULL;
static lv_obj_t *ta_ch_scope_edit        = NULL;
static lv_obj_t *kb_ch_scope_edit        = NULL;

// Channel add overlay (on channels settings sub-screen)
static lv_obj_t *obj_ch_settings_add_panel = NULL;
static lv_obj_t *ta_ch_settings_add        = NULL;
static lv_obj_t *kb_ch_settings_add        = NULL;

// Tone picker overlay (on channel detail screen)
static lv_obj_t *obj_tone_picker_panel  = NULL;
static lv_obj_t *obj_tone_picker_scroll = NULL;
static lv_obj_t *lbl_ch_detail_tone     = NULL;

// Position settings sub-screen (Settings > Position)
static lv_obj_t *scr_settings_position   = NULL;
static lv_obj_t *lbl_pos_lat_value       = NULL;
static lv_obj_t *lbl_pos_lon_value       = NULL;
static lv_obj_t *lbl_pos_mode_value      = NULL;

// Position numeric edit overlay (on scr_settings_position)
static lv_obj_t *obj_pos_edit_panel      = NULL;
static lv_obj_t *ta_pos_edit             = NULL;
static lv_obj_t *kb_pos_edit             = NULL;
static int       g_pos_edit_field        = 0;  // 3 = lat, 4 = lon

static lv_obj_t *btn_attach              = NULL;  // + button left of compose
static lv_obj_t *obj_attach_menu         = NULL;  // long-press popup menu
static lv_obj_t *obj_attach_picker       = NULL;  // file picker overlay

// Share channel — contact picker overlay on scr_channel_detail
static lv_obj_t *obj_ch_share_picker_panel  = NULL;
static lv_obj_t *obj_ch_share_picker_scroll = NULL;
static int g_share_channel_idx              = -1;

// Channel-share delivery feedback. After a share DM is sent we poll
// lookupDMAckStatus() in ui_update_timer_cb: on ack -> "Sent!", on timeout
// retry up to 3 attempts, then "Failed". Status shows in a top-layer label.
static lv_obj_t   *g_share_status_lbl     = NULL;
static lv_timer_t *g_share_dismiss_timer  = NULL;
static bool        g_share_active         = false;
static int         g_share_st_channel     = -1;
static int         g_share_st_contact     = -1;
static uint32_t    g_share_st_ack         = 0;
static int         g_share_st_attempt     = 0;

// Voice screens — landing (Inbox/Record picker), inbox, and record
static lv_obj_t *scr_voice_landing          = NULL;  // Inbox / Record picker
static lv_obj_t *btn_voice_inbox            = NULL;  // Inbox button on landing
static lv_obj_t *lbl_voice_inbox_badge      = NULL;  // unread count badge
static lv_obj_t *btn_voice_record_nav       = NULL;  // Record button on landing

static lv_obj_t *scr_voice_inbox            = NULL;  // received message list
static lv_obj_t *obj_voice_inbox_scroll     = NULL;  // scrollable container

static lv_obj_t *scr_voice                  = NULL;  // recording screen
static lv_obj_t *lbl_voice_status           = NULL;
static lv_obj_t *lbl_voice_timer            = NULL;
static lv_obj_t *btn_voice_record           = NULL;
static lv_obj_t *lbl_voice_record_btn       = NULL;
static lv_obj_t *btn_voice_play             = NULL;
static lv_obj_t *btn_voice_send             = NULL;
static lv_obj_t *btn_voice_discard          = NULL;
static lv_obj_t *obj_voice_contact_panel    = NULL;
static lv_obj_t *obj_voice_contact_scroll   = NULL;
static bool      g_voice_has_recording      = false;
static char      g_voice_last_wav_path[96]  = {};

// Voice send status tracking
static uint32_t  g_voice_send_complete_ms   = 0;     // millis when send should be done
static bool      g_voice_send_in_flight     = false;

// Previous recordings scroll area on record screen
static lv_obj_t *obj_voice_prev_scroll      = NULL;

// Voice inbox metadata — persisted to /sdcard/voice/inbox.cfg
#define VOICE_INBOX_MAX       32
#define VOICE_INBOX_FOLDER    "/sdcard/voice/inbox"
#define VOICE_INBOX_CFG       "/sdcard/voice/inbox.cfg"

struct VoiceInboxEntry {
    char     senderName[32];
    char     filename[48];      // just the basename, inside VOICE_INBOX_FOLDER
    uint32_t timestamp;         // UTC epoch
    uint8_t  durationSec;
    bool     played;
    bool     valid;
};

static VoiceInboxEntry g_voice_inbox[VOICE_INBOX_MAX] = {};
static int g_voice_inbox_count   = 0;
static int g_voice_inbox_unread  = 0;

// Radio preset picker
static lv_obj_t *scr_radio_picker  = NULL;

// Channel picker
static lv_obj_t *scr_channel_picker  = NULL;
static lv_obj_t *obj_ch_picker_scroll = NULL;
static lv_obj_t *lbl_picker_unread[MAX_GROUP_CHANNELS] = {};
// Cached handle for the Direct Messages pseudo-row's unread badge. Set
// when refresh_channel_picker constructs the row; read by
// ui_update_timer_cb to update the badge text without a full picker
// rebuild. Null when the picker hasn't been built yet (or has been
// cleaned and not yet refreshed).
static lv_obj_t *lbl_picker_dm_unread = NULL;
static lv_obj_t *obj_ch_add_panel  = NULL;
static lv_obj_t *ta_ch_add         = NULL;
static lv_obj_t *kb_ch_add         = NULL;

// DM inbox — list of contacts with at least one DM (sent or received),
// sorted newest-first. Entered from the channel picker's Direct Messages
// pseudo-row. Per-row tap opens the contact's existing DM conversation
// view via load_dm_view().
static lv_obj_t *scr_dm_inbox       = NULL;
static lv_obj_t *obj_dm_inbox_scroll = NULL;

// Messages screen
static lv_obj_t *scr_messages           = NULL;
static lv_obj_t *lbl_msg_channel_name   = NULL;
static lv_obj_t *lbl_msg_channel_scope  = NULL;
static lv_obj_t *lbl_messages_body      = NULL;
static lv_obj_t *obj_msg_scroll         = NULL;
static lv_obj_t *ta_compose             = NULL;
static lv_obj_t *kb_compose             = NULL;
static lv_obj_t *btn_send               = NULL;
static uint8_t  g_active_channel        = 0;

// In-RAM per-channel message drafts (channels only, not persisted across
// reboot). Restored when entering a channel, captured when leaving the
// messages screen, cleared on send. Gated by prefs->save_drafts.
static char     g_channel_drafts[MAX_GROUP_CHANNELS][160] = {};

// ============================================================================
// DMs — per-contact conversation rings + state
// ----------------------------------------------------------------------------
// The DM conversation view reuses scr_messages so all the textarea/keyboard
// re-layout logic stays in one place. The active dataset (channel vs DM) is
// selected by g_in_dm_mode; the send and back-button paths branch on this
// flag.
//
// Storage: per-contact ring of up to MECK_DM_PER_CONTACT messages. Rings
// are allocated lazily from PSRAM on first message touching that contact,
// so the cost scales with actual DM usage rather than MAX_CONTACTS (1500).
// Allocation is via heap_caps_calloc with MALLOC_CAP_SPIRAM (matches the
// pattern MeckMesh.h uses for the channel message rings and contacts
// array), freed only if/when the contact is deleted
// (TODO: deleteContactByIdx needs to call into a DM-side helper to free
// and compact the rings; not on v0.3.4 roadmap, flagged so we don't get
// caught by stale pointers when contact deletion is exercised).
//
// Sent DMs are added to the ring immediately (local-echo, in-RAM only).
// Received DMs are added when meck_drain_pending_dms invokes the
// registered callback. Piece 5 will add SD persistence for the received
// half so they survive reboot.
// ============================================================================
#define MECK_DM_PER_CONTACT  20

struct DMMessage {
    uint32_t timestamp;       // unique sender timestamp (or our send time for outgoing)
    bool     outgoing;        // true = we sent it, false = we received it
    uint8_t  path_len;        // recv only: 0xFF direct, else flood hop count
    int8_t   snr_x4;          // recv only: SNR * 4
    uint32_t expected_ack;    // outgoing only; matched against incoming ACKs (piece 5)
    bool     acked;           // outgoing only; flipped true when ACK matched (piece 5)
    char     text[160];       // raw message body
    // SD persistence tracking.
    //   file_offset      — byte offset of this message's record in
    //                      dms.bin. 0 = not persisted (save failed,
    //                      or loaded from disk where the loader didn't
    //                      track offsets).
    //   persisted_acked  — shadow of the on-disk ACKED flag bit. Used
    //                      by the auto-refresh rewrite path to detect
    //                      RAM→disk drift (RAM acked=true while disk
    //                      hasn't been rewritten yet) without spamming
    //                      SD writes on every render tick.
    //   from_disk        — true if this entry was loaded by
    //                      load_persisted_dms_into_rings, false if it
    //                      arrived via the runtime path. Lets the
    //                      renderer pick "Sent" (unknown final state)
    //                      vs "Sending..." (fresh, awaiting expected_ack)
    //                      for outgoing bubbles with expected_ack=0.
    uint32_t file_offset;
    bool     persisted_acked;
    bool     from_disk;
};

struct DMRing {
    DMMessage* messages;      // heap_caps_calloc'd, MECK_DM_PER_CONTACT entries in PSRAM
    int        count;         // valid entries in ring
    int        newest_idx;    // most-recent slot; oldest = (newest+1) % depth (when full)
    int        unread;        // received-but-not-viewed
};

// One ring per contact index. nullptr until a message arrives or is sent.
// MAX_CONTACTS is 1500 on P4; pointer array is 12 KB on PSRAM, cheap.
static DMRing* g_dm_rings[MAX_CONTACTS] = {nullptr};

// View state — which contact's conversation is currently open. -1 when
// the DM screen isn't loaded.
static int  g_active_dm_contact_idx = -1;
static bool g_in_dm_mode            = false;

// ============================================================================
// Per-room post ring (Piece B)
// ----------------------------------------------------------------------------
// Mirror of the DM ring, scoped per room contact. Posts arrive from
// onSignedMessageRecv (via meck_drain_pending_posts → meck_post_recv_dispatch)
// and on boot from load_persisted_posts_into_rings reading posts.bin.
//
// Differences from DMRing:
//   - depth 32 to match upstream MyMesh MAX_UNSYNCED_POSTS (typical push
//     burst after login)
//   - no `unread` field — the room view is a passive history, the home
//     screen unread counter only tracks DMs and channel messages
//   - sender_prefix[4] carries the original author so the renderer can
//     show "Alice: hello" (resolved against contacts) or
//     "Unknown <abcd1234>: hello"
//   - no outgoing-message persistence path yet (Piece C will add the
//     composer, with the same local-echo + RAM-only convention as DMs)
// ============================================================================
#define MECK_POST_PER_ROOM  32

struct PostMessage {
    uint32_t timestamp;
    uint8_t  sender_prefix[4];   // original author's pub_key prefix
    uint8_t  path_len;           // 0xFF direct, else hop count when post arrived
    int8_t   snr_x4;
    char     text[160];          // null-terminated; matches P4_POST_TEXT_LEN headroom
    uint32_t file_offset;        // byte offset in posts.bin; 0 if not persisted / loaded
    bool     from_disk;          // true if hydrated from posts.bin, false if runtime
};

struct PostRing {
    PostMessage* messages;       // heap_caps_calloc'd, MECK_POST_PER_ROOM entries in PSRAM
    int          count;
    int          newest_idx;
};

static PostRing* g_post_rings[MAX_CONTACTS] = {nullptr};

// View state — which room's history is currently open. -1 when scr_room_messages
// isn't the active screen.
static int  g_active_room_contact_idx = -1;
static bool g_in_room_mode            = false;

// Contacts
static lv_obj_t *scr_contacts            = NULL;
static lv_obj_t *obj_contacts_scroll     = NULL;
static lv_obj_t *obj_contacts_filter_bar = NULL;  // chip row above the list
static lv_obj_t *scr_contact_detail      = NULL;
static lv_obj_t *lbl_contact_detail_body = NULL;
static int g_selected_contact_idx        = -1;

// Trace Path screen (manual hex hop entry, runs PAYLOAD_TYPE_TRACE,
// shows per-hop SNR results). Screen-host slot 10.
static lv_obj_t *scr_trace              = NULL;
static lv_obj_t *dd_trace_path_size     = NULL;  // dropdown: 1-byte / 2-byte
static lv_obj_t *ta_trace_path          = NULL;  // textarea: hex hops, comma-sep
static lv_obj_t *btn_trace_run          = NULL;
static lv_obj_t *cont_trace_results     = NULL;  // scrollable result list
static lv_obj_t *lbl_trace_status       = NULL;  // "Running…" / error text
static lv_obj_t *kb_trace               = NULL;  // keyboard for ta_trace_path
static unsigned long g_trace_sent_ms    = 0;     // millis() when run pressed
static bool          g_trace_in_flight  = false; // waiting on response

// Path Editor screen (manual hex hop entry for a contact's out_path).
// Screen-host slot 11. Operates on g_selected_contact_idx, set by the
// Edit Path button on scr_contact_detail.
static lv_obj_t *scr_path_editor              = NULL;
static lv_obj_t *lbl_path_editor_contact      = NULL;
static lv_obj_t *dd_path_editor_size          = NULL;  // 1-byte / 2-byte / 3-byte
static lv_obj_t *ta_path_editor_path          = NULL;
static lv_obj_t *lbl_path_editor_status       = NULL;
static lv_obj_t *kb_path_editor               = NULL;  // keyboard for ta_path_editor_path

// Contact custom path editor — fwd decl so the contact detail screen
// builder can wire its Edit Path button to the screen-load callback.
static void goto_path_editor(lv_event_t *e);

// Contact list filter. Persists across navigations to the contacts screen
// so the user lands back where they were. Mirrors upstream Meck's enum.
typedef enum {
    CONTACT_FILTER_ALL = 0,
    CONTACT_FILTER_CHAT,
    CONTACT_FILTER_REPEATER,
    CONTACT_FILTER_ROOM,
    CONTACT_FILTER_SENSOR,
    CONTACT_FILTER_FAV,
    CONTACT_FILTER_COUNT
} ContactFilter;

static ContactFilter g_contact_filter = CONTACT_FILTER_ALL;

// Contacts → auto-add submenu under Settings
static lv_obj_t *scr_settings_contacts        = NULL;
static lv_obj_t *lbl_set_contact_mode         = NULL;
static lv_obj_t *lbl_set_autoadd_chat         = NULL;
static lv_obj_t *lbl_set_autoadd_repeater     = NULL;
static lv_obj_t *lbl_set_autoadd_room         = NULL;
static lv_obj_t *lbl_set_autoadd_sensor       = NULL;
static lv_obj_t *lbl_set_autoadd_overwrite    = NULL;
static lv_obj_t *lbl_set_backup_status        = NULL;

// Export Config modal — overlay panel on scr_settings with four section
// checkboxes (Identity, Channels, Contacts, Radio settings) plus Cancel
// and Export buttons. Built once at settings-screen construction time
// and hidden/shown via LV_OBJ_FLAG_HIDDEN. See create_settings_screen.
static lv_obj_t *obj_export_panel       = NULL;
static lv_obj_t *cb_export_identity     = NULL;
static lv_obj_t *cb_export_channels     = NULL;
static lv_obj_t *cb_export_contacts     = NULL;
static lv_obj_t *cb_export_radio        = NULL;
static lv_obj_t *lbl_export_warn        = NULL;

// ----------------------------------------------------------------------------
// Retry-send modal (channel + DM)
// ----------------------------------------------------------------------------
// Long-pressing a sent bubble that hasn't been relayed/acked within the
// threshold opens this modal with a single "Retry Send" button. Confirming
// re-queues the body text via the normal send path (new timestamp; the
// recipient may see a duplicate if the original arrives late).
//
// Channel threshold: heard_count==0 and (now_utc - msg.timestamp) >= 18 s.
// DM threshold: lookupDMAckStatus reports a non-acked entry whose
// (now - sent_ms) >= timeout_ms (i.e. the "Failed" state rebuild_dm_bubbles
// already shows).

// Channel-side threshold in seconds. DM side reuses the existing per-DM
// ACK timeout from lookupDMAckStatus.
static const uint32_t MECK_RETRY_CHANNEL_THRESHOLD_SEC = 18;

// UTC second at which the active channel's earliest still-"Sending" sent
// message flips to "Failed". The UI timer redraws once when this is reached,
// instead of rebuilding the bubble list every tick (which runs lv_obj_clean()
// and would tear down any in-progress long-press). 0 = nothing pending.
static uint32_t g_chan_flip_deadline = 0;

// Per-bubble retry context, attached as user_data on each sent bubble at
// rebuild time and freed via LV_EVENT_DELETE when the bubble is destroyed
// (next rebuild or screen unload).
struct BubbleRetryCtx {
    bool     is_dm;             // false = channel send, true = DM send
    bool     is_outgoing;       // true = we sent this, false = received
    int      target_idx;        // channel_idx (channel) or contact_idx (DM)
    uint32_t timestamp;         // message timestamp — matching key for current
                                // state lookup at long-press time
    uint32_t expected_ack;      // DM only; 0 for channel. Used with
                                // lookupDMAckStatus for the Failed check.
    char     body[200];         // text to resend. For channel, the
                                // "sender: " prefix has been stripped.
    // Path view data — populated from P4ChannelMessage at bubble creation
    uint8_t  path_len;          // encoded path_len from message
    uint8_t  msg_path_hash_size;
    uint8_t  msg_path_hash_count;
    uint8_t  msg_path_hashes[16];
    uint8_t  echo_hash_size;
    uint8_t  echo_hash_count;
    uint8_t  echo_hashes[16];
    uint8_t  heard_count;
    char     sender[64];        // parsed sender name (received channel msgs
                                // only; empty for sent and DM). Used to
                                // prefill a "@sender " reply.
};

// Modal widgets — created once and attached to scr_messages so they
// overlay both channel and DM views (DM reuses scr_messages, see note
// near scr_messages declaration).
static lv_obj_t *obj_retry_panel  = NULL;
static lv_obj_t *lbl_retry_prompt = NULL;

// Populated by on_bubble_long_pressed when eligibility passes, read by
// on_retry_modal_send. Body must outlive the modal open since the bubble
// (and its ctx) could be freed by a rebuild between long-press and tap.
static bool g_retry_pending_is_dm    = false;
static int  g_retry_pending_idx      = -1;
static char g_retry_pending_body[200] = "";

// Path info popup — shown on long-press of any bubble
static lv_obj_t *obj_path_panel     = NULL;
static lv_obj_t *lbl_path_info      = NULL;
static char g_path_copy_text[256]   = "";  // text to paste on "Copy Path"

// Reply button on the path popup — shown only when a received channel
// bubble is long-pressed. Prefills the composer with "@sender ".
static lv_obj_t *btn_path_reply     = NULL;
static char g_reply_pending_sender[64] = "";

// Discover (repeaters-only, mirrors Meck T-Deck Pro's F-key Discover)
static lv_obj_t *scr_discover         = NULL;
static lv_obj_t *obj_discover_scroll  = NULL;
static lv_obj_t *lbl_discover_status  = NULL;

// ============================================================================
// Repeater Admin state — piece 3
// ----------------------------------------------------------------------------
// Login screen state plus a temporary "logged-in" placeholder screen that
// piece 4 will replace with the three-tab admin home (Status / Command
// Line / Settings). For now the placeholder just confirms the session
// landed and shows is_admin / clock_at_login so the user can verify the
// protocol layer is working end-to-end.
//
// Password handling: characters are typed into the textarea normally,
// each one displays as the typed char for ~1500ms before resolving to a
// privacy dot. A separate raw-text buffer holds the actual password
// (the textarea content gets masked on display). g_admin_pwd_reveal
// is the "show full plaintext" eye-toggle override.
//
// Password cache: 8-slot LRU keyed by contact_idx. RAM-only — cleared on
// reboot, no persistence to SD. Populated only after a successful login.
// ============================================================================
static lv_obj_t *scr_admin_login            = NULL;
static lv_obj_t *scr_admin_home             = NULL;  // 3-tab logged-in home (piece 4)
static lv_obj_t *scr_room_messages          = NULL;  // room post history (Piece A: stub)
static lv_obj_t *lbl_room_messages_title    = NULL;  // room contact name in scr_room_messages header
static lv_obj_t *btn_room_admin             = NULL;  // top-right Admin overlay on scr_room_messages, visible only when g_admin_session_is_admin
static lv_obj_t *obj_room_scroll            = NULL;  // bubble scroll container inside scr_room_messages (Piece B)
static lv_obj_t *ta_room_compose            = NULL;  // composer textarea on scr_room_messages (Piece C)
static lv_obj_t *btn_room_send              = NULL;  // composer send button (Piece C)
static lv_obj_t *kb_room_compose            = NULL;  // composer keyboard (Piece C)
static lv_obj_t *btn_contact_admin          = NULL;  // visible only for ADV_TYPE_REPEATER/ROOM
static lv_obj_t *lbl_contact_admin_btn      = NULL;  // text inside btn_contact_admin; updated per contact type ("Admin" for repeater, "Login" for room)
static lv_obj_t *btn_contact_dm             = NULL;  // visible only for ADV_TYPE_CHAT
static lv_obj_t *btn_contact_ping           = NULL;  // visible only for ADV_TYPE_REPEATER/ROOM

// ---- Ping UI state ----
// Ping result modal (green success / red failure), prefix warning
// dialog, and byte-size chooser (long-press).
static lv_obj_t *obj_ping_result_panel      = NULL;  // full-screen overlay for result
static lv_obj_t *lbl_ping_result            = NULL;  // multi-line result text
static lv_obj_t *obj_ping_warning_panel     = NULL;  // prefix warning overlay
static lv_obj_t *obj_ping_chooser_panel     = NULL;  // 1/2 byte long-press chooser

static bool     g_ping_in_flight            = false;
static uint8_t  g_ping_byte_size            = 1;     // 1 or 2
static unsigned long g_ping_timeout_ms      = 0;     // millis() when ping times out
#define MECK_PING_TIMEOUT_SEC  15
static lv_obj_t *lbl_admin_login_title      = NULL;
static lv_obj_t *lbl_admin_login_subtitle   = NULL;
static lv_obj_t *lbl_admin_login_status     = NULL;
static lv_obj_t *ta_admin_password          = NULL;
static lv_obj_t *kb_admin_password          = NULL;
static lv_obj_t *btn_admin_password_eye     = NULL;
static lv_obj_t *cb_admin_remember          = NULL;
static lv_obj_t *btn_admin_login_submit     = NULL;
static lv_obj_t *lbl_admin_login_submit     = NULL;

// ---- Admin home (menu list) + subpage widgets ----
// scr_admin_home is the menu list (banner + 4 row buttons). Each
// row taps into a dedicated subpage screen; no banner on subpages.
static lv_obj_t *obj_admin_banner           = NULL;
static lv_obj_t *lbl_admin_banner           = NULL;
// Subpage screens.
static lv_obj_t *scr_admin_status           = NULL;
static lv_obj_t *scr_admin_cmd              = NULL;
static lv_obj_t *scr_admin_settings         = NULL;
static lv_obj_t *scr_admin_send_advert      = NULL;
// Status subpage widgets.
static lv_obj_t *btn_admin_status_refresh   = NULL;
static lv_obj_t *lbl_admin_status_refresh   = NULL;
static lv_obj_t *lbl_admin_status_updated   = NULL;  // "Updated Ns ago" / "Refreshing..."
static lv_obj_t *lbl_admin_status_body      = NULL;  // big multi-line label
// Send Advert subpage widgets.
static lv_obj_t *btn_admin_send_advert      = NULL;
static lv_obj_t *lbl_admin_send_advert_status = NULL;
static lv_obj_t *lbl_admin_send_advert_response = NULL;
// Cmd Line subpage widgets (piece 6).
//   obj_admin_cmd_scroll is the scrollback container; each entry is a
//   pair of labels (the command typed, then the repeater's reply).
//   ta_admin_cmd_input + kb_admin_cmd_input are the input controls.
//   The keyboard slides up on textarea focus and the textarea + send
//   button lift above it; same pattern as compose on the DM screen.
#define ADMIN_CMD_MAX_LEN     100
#define ADMIN_CMD_MAX_ENTRIES 50
static lv_obj_t *obj_admin_cmd_scroll       = NULL;
static lv_obj_t *ta_admin_cmd_input         = NULL;
static lv_obj_t *kb_admin_cmd_input         = NULL;
static lv_obj_t *btn_admin_cmd_send         = NULL;
// Settings subpage widgets (piece 5).
//   obj_admin_settings_list is the scroll container holding the rows.
//   Rows are torn down and rebuilt on every screen entry so the visible
//   set reflects the current session role (admin vs guest).
static lv_obj_t *obj_admin_settings_list    = NULL;
// Shared placeholder subpage for individual settings, reused for all
// 10 settings until each is built out. The title and body labels are
// reset on every entry to reflect which row was tapped.
static lv_obj_t *scr_admin_setting_placeholder  = NULL;
static lv_obj_t *lbl_admin_setting_placeholder_title = NULL;
static lv_obj_t *lbl_admin_setting_placeholder_body  = NULL;

// Firmware Info subpage — sends "ver" CLI command on entry and on
// Refresh, parses the "<version> (Build: <date>)" reply from upstream
// CommonCLI.cpp into two labels.
static lv_obj_t *scr_admin_fw_info             = NULL;
static lv_obj_t *lbl_admin_fw_info_status      = NULL;
static lv_obj_t *lbl_admin_fw_info_version     = NULL;
static lv_obj_t *lbl_admin_fw_info_build       = NULL;
static lv_obj_t *btn_admin_fw_info_refresh     = NULL;

// Neighbours subpage — uses the binary REQ_TYPE_GET_NEIGHBOURS path
// (rather than the text CLI 'neighbors' command which the repeater
// caps at ~8 entries). Paginated: on entry/Refresh we request
// offset=0, then iterate with each dispatch until we've received
// the full neighbour list reported by the first response.
static lv_obj_t *scr_admin_neighbours          = NULL;
static lv_obj_t *lbl_admin_neighbours_status   = NULL;
static lv_obj_t *obj_admin_neighbours_scroll   = NULL;
static lv_obj_t *btn_admin_neighbours_refresh  = NULL;

// Cached most-recent status response, populated by the dispatch
// callback. Re-rendered on subpage entry and on refresh.
// last_update == 0 means "never received yet" (show placeholder text).
static RepeaterStats  g_admin_last_status        = {};
static unsigned long  g_admin_status_last_update = 0;
static uint32_t       g_admin_status_clock_tag   = 0;
static bool           g_admin_status_in_flight   = false;

// In-flight flags for the two CLI-using subpages. The CLI dispatcher
// (meck_admin_cli_dispatch) routes the reply to whichever one is set.
// Only one can be in flight at a time — the textarea/buttons on each
// subpage disable themselves while their flag is set.
static bool           g_admin_send_advert_in_flight = false;
static bool           g_admin_cmd_in_flight         = false;
static bool           g_admin_fw_in_flight          = false;

// Firmware Info send timestamp — used by the 500ms ui_update_timer_cb
// to time out a stuck "Refreshing..." after MECK_ADMIN_FW_TIMEOUT_MS.
static unsigned long  g_admin_fw_sent_ms            = 0;
#define MECK_ADMIN_FW_TIMEOUT_MS 20000

// ---- Neighbours pagination state ----
// _in_flight is true from the first page request until either the last
// page lands, a send failure surfaces, or the timeout elapses. The
// pagination accumulator tracks how many entries have been rendered
// vs how many the repeater claimed in the first page, so we know
// when to stop iterating.
static bool          g_admin_neighbours_in_flight        = false;
static unsigned long g_admin_neighbours_sent_ms          = 0;
static uint16_t      g_admin_neighbours_total            = 0;
static uint16_t      g_admin_neighbours_received         = 0;
static uint16_t      g_admin_neighbours_next_offset      = 0;
#define MECK_ADMIN_NEIGHBOURS_PAGE_SIZE    10
#define MECK_ADMIN_NEIGHBOURS_PREFIX_LEN   4
#define MECK_ADMIN_NEIGHBOURS_ORDER_BY     0   // 0 = newest to oldest
#define MECK_ADMIN_NEIGHBOURS_TIMEOUT_MS   20000

// Which contact the login screen is currently targeting (set on entry
// from on_contact_admin_tap, used by the submit handler and callbacks).
static int g_admin_login_contact_idx = -1;

// Cached routing-mode badge string for the Login button. Set on screen
// entry from the contact's current out_path_len.
static bool g_admin_login_routing_direct = false;

// Raw password buffer. The textarea displays either dots or a mix of
// the most-recent-char-as-cleartext + dots (see reveal-then-dot
// rendering). This buffer is the source of truth used by the submit
// handler.
#define MECK_ADMIN_PASSWORD_MAX 63
static char g_admin_pwd_raw[MECK_ADMIN_PASSWORD_MAX + 1] = {};
static int  g_admin_pwd_len = 0;

// Reveal-then-dot bookkeeping. _last_char_ms is millis() of the most
// recent keypress; the 500ms timer re-renders the masked display so
// the last character resolves to a dot ~1500ms after it was typed.
static unsigned long g_admin_pwd_last_char_ms = 0;
static const uint32_t MECK_ADMIN_PWD_REVEAL_MS = 1500;

// Set true while admin_render_password_masked is calling
// lv_textarea_set_text. Any LV_EVENT_VALUE_CHANGED fired during that
// programmatic update is OUR write, not a user keypress, and must NOT
// re-enter the change handler — that path infinite-recurses and blows
// the stack (the symptom that crashed the first admin-tap).
static bool g_admin_pwd_render_in_progress = false;

// Eye-toggle for full reveal. When true, the textarea displays the
// raw password verbatim and bypasses the dot masking. Resets to false
// on screen entry.
static bool g_admin_pwd_reveal_all = false;

// Login state machine. PASSWORD_ENTRY is the initial state; LOGGING_IN
// is set on submit and cleared by either the response or a timeout.
typedef enum {
    ADMIN_STATE_IDLE = 0,
    ADMIN_STATE_PASSWORD_ENTRY,
    ADMIN_STATE_LOGGING_IN,
    ADMIN_STATE_LOGGED_IN,
} AdminUiState;
static AdminUiState g_admin_ui_state = ADMIN_STATE_IDLE;

// Submit-side bookkeeping for the LOGGING_IN countdown / timeout.
static unsigned long g_admin_login_sent_ms      = 0;
static uint32_t      g_admin_login_timeout_ms   = 0;

// Captured session state from the most recent successful login.
// Display values for the placeholder logged-in screen; piece 4's
// admin home will read these too.
static bool     g_admin_session_is_admin    = false;
static uint8_t  g_admin_session_permissions = 0;
static uint8_t  g_admin_session_fw_ver      = 0;
static uint32_t g_admin_session_clock_tag   = 0;

// Password cache — RAM-only, 8-slot LRU keyed by contact_idx.
// Populated only after a successful login. On re-entry to the admin
// login screen for a known-good contact, the cached password
// pre-fills the textarea but the user must still tap Log In.
#define MECK_ADMIN_PWD_CACHE_SIZE 8
struct AdminPwdCacheEntry {
    int  contact_idx;
    char password[MECK_ADMIN_PASSWORD_MAX + 1];
};
static AdminPwdCacheEntry g_admin_pwd_cache[MECK_ADMIN_PWD_CACHE_SIZE] = {};
static int g_admin_pwd_cache_count = 0;

// ============================================================================
// Forward declarations
// ============================================================================

static void goto_home(lv_event_t *e);
static void goto_settings(lv_event_t *e);
static void goto_channel_picker(lv_event_t *e);
static void goto_messages_back(lv_event_t *e);
static void goto_dm_inbox(lv_event_t *e);
static void refresh_dm_inbox_list();
static void create_dm_inbox_screen();
static void goto_contacts(lv_event_t *e);
static void goto_contacts_from_detail(lv_event_t *e);
static void goto_channel_n(lv_event_t *e);
static void load_channel_view(uint8_t ch_idx);
static void load_dm_view(int contact_idx);
static void rebuild_message_bubbles(uint8_t ch_idx);

static void settings_update_labels();
static void update_radio_detail_label();
static void refresh_channel_picker();
static void refresh_contacts_list();
static void goto_discover(lv_event_t *e);
static void refresh_discover_list();
static void create_discover_screen();
static void goto_trace(lv_event_t *e);
static void create_trace_screen();
static void create_path_editor_screen();

static void on_settings_name_tap(lv_event_t *e);
static void on_settings_name_save(lv_event_t *e);
static void on_settings_name_cancel(lv_event_t *e);
static void on_settings_kb_event(lv_event_t *e);
static void on_settings_radio_tap(lv_event_t *e);
static void on_settings_txpower_tap(lv_event_t *e);
static void on_settings_utc_tap(lv_event_t *e);
static void on_settings_freq_tap(lv_event_t *e);
static void on_settings_bw_tap(lv_event_t *e);
static void on_settings_sf_tap(lv_event_t *e);
static void on_settings_cr_tap(lv_event_t *e);
static void on_num_edit_save(lv_event_t *e);
static void on_num_edit_cancel(lv_event_t *e);
static void on_num_edit_kb_event(lv_event_t *e);
static void on_settings_region_tap(lv_event_t *e);
static void on_region_edit_save(lv_event_t *e);
static void on_region_edit_cancel(lv_event_t *e);
static void on_region_edit_kb_event(lv_event_t *e);
static void goto_settings_channels(lv_event_t *e);
static void on_ch_settings_row_tap(lv_event_t *e);
static void on_ch_settings_add_tap(lv_event_t *e);
static void on_ch_detail_scope_tap(lv_event_t *e);
static void on_ch_detail_notif_tap(lv_event_t *e);
static void on_ch_detail_delete_tap(lv_event_t *e);
static void on_ch_scope_edit_save(lv_event_t *e);
static void on_ch_scope_edit_cancel(lv_event_t *e);
static void on_ch_scope_edit_kb_event(lv_event_t *e);
static void on_ch_settings_add_save(lv_event_t *e);
static void on_ch_settings_add_kb_event(lv_event_t *e);
static void refresh_channels_settings_list();
static void refresh_channel_detail_labels();
static void create_settings_channels_screen();
static void create_channel_detail_screen();
static void on_ch_detail_tone_tap(lv_event_t *e);
static void on_dm_tone_tap(lv_event_t *e);
static void on_tone_picker_select(lv_event_t *e);
static void on_tone_picker_cancel(lv_event_t *e);
static void refresh_tone_picker_list();
static void on_radio_preset_select(lv_event_t *e);

// Position subscreen
static void goto_settings_position(lv_event_t *e);
static void on_pos_lat_tap(lv_event_t *e);
static void on_pos_lon_tap(lv_event_t *e);
static void on_pos_mode_tap(lv_event_t *e);
static void create_settings_position_screen();
static void refresh_position_labels();

// Attachment menu
static void on_attach_long_press(lv_event_t *e);
static void on_attach_pick_file(lv_event_t *e);
static void on_attach_share_pos(lv_event_t *e);
static void on_attach_cancel(lv_event_t *e);

// Channel share picker
static void on_ch_detail_share_tap(lv_event_t *e);
static void on_ch_share_picker_select(lv_event_t *e);
static void on_ch_share_picker_cancel(lv_event_t *e);

// Voice screens
static void on_voice_record_tap(lv_event_t *e);
static void on_voice_play_tap(lv_event_t *e);
static void on_voice_send_tap(lv_event_t *e);
static void on_voice_contact_select(lv_event_t *e);
static void on_voice_contact_cancel(lv_event_t *e);
static void on_voice_discard_tap(lv_event_t *e);
static void create_voice_landing_screen(void);
static void create_voice_inbox_screen(void);
static void create_voice_record_screen(void);
static void voice_update_ui(void);
static void voice_inbox_load(void);
static void voice_inbox_save(void);
static void voice_inbox_add(const char *senderName, const char *wavPath,
                            uint32_t timestamp, uint8_t durationSec);
static void voice_inbox_refresh(void);
static void voice_inbox_update_badge(void);
static void voice_prev_refresh(void);

static void on_send_clicked(lv_event_t *e);
static void on_compose_focused(lv_event_t *e);
static void on_compose_defocused(lv_event_t *e);
static void on_kb_event(lv_event_t *e);

static void on_contact_tap(lv_event_t *e);
static void on_contact_fav_toggle(lv_event_t *e);
static void on_contact_long_press(lv_event_t *e);
static void on_contact_delete(lv_event_t *e);
static void on_contact_send_dm_tap(lv_event_t *e);
static void on_contacts_screen_gesture(lv_event_t *e);
static void on_filter_chip_tap(lv_event_t *e);

// Repeater Admin — pieces 3 + 4
static void create_admin_login_screen();
static void create_admin_home_screen();
static void on_contact_admin_tap(lv_event_t *e);
static void on_admin_login_back(lv_event_t *e);
static void on_admin_login_submit(lv_event_t *e);
static void on_admin_password_textarea_change(lv_event_t *e);
static void on_admin_password_focused(lv_event_t *e);
static void on_admin_password_defocused(lv_event_t *e);
static void on_admin_password_eye_tap(lv_event_t *e);
static void on_admin_home_back(lv_event_t *e);
static void on_admin_subpage_back(lv_event_t *e);
// Ping (zero-hop trace)
static void on_contact_ping_tap(lv_event_t *e);
static void on_contact_ping_longpress(lv_event_t *e);
static void ping_send_now(uint8_t byte_size);
static void ping_dismiss_result(lv_event_t *e);
static void ping_show_result(bool success, const char *body);
static void ping_dismiss_warning(lv_event_t *e);
static void ping_confirm_warning(lv_event_t *e);
static void ping_chooser_select(lv_event_t *e);
// Room messages (Piece A: stub screen, no storage/rendering yet)
static void create_room_messages_screen();
static void on_room_back(lv_event_t *e);
static void on_room_admin_tap(lv_event_t *e);
static void room_messages_screen_enter(int contact_idx, const char* contact_name);
// Room post storage + rendering (Piece B)
static PostRing* post_ring_get_or_alloc(int contact_idx);
static int  post_ring_append(int contact_idx, const PostMessage& msg);
static int  post_ring_copy_chronological(int contact_idx, PostMessage* out, int max_out);
static bool post_ring_has_post(int contact_idx, uint32_t timestamp,
                                const uint8_t* sender_prefix);
static void resolve_post_author(const uint8_t* sender_prefix, char* out, int out_len);
static void rebuild_room_bubbles(int contact_idx);
static void meck_post_recv_dispatch(const uint8_t* room_pub_key,
                                    const char* room_name,
                                    const uint8_t* sender_prefix,
                                    const char* text,
                                    uint32_t sender_timestamp,
                                    uint8_t path_len,
                                    int8_t snr_x4);
static void load_persisted_posts_into_rings();
// Room composer (Piece C)
static bool post_is_outgoing(const uint8_t* sender_prefix);
static void meck_relayout_room_compose(void);
static void on_room_send_clicked(lv_event_t *e);
static void on_room_compose_focused(lv_event_t *e);
static void on_room_compose_defocused(lv_event_t *e);
static void on_room_compose_size_changed(lv_event_t *e);
static void on_room_kb_event(lv_event_t *e);
static void on_admin_menu_status_tap(lv_event_t *e);
static void on_admin_menu_cmd_tap(lv_event_t *e);
static void on_admin_menu_settings_tap(lv_event_t *e);
static void on_admin_menu_send_advert_tap(lv_event_t *e);
static void on_admin_status_refresh_tap(lv_event_t *e);
static void on_admin_send_advert_tap(lv_event_t *e);
static void admin_render_password_masked();
static void admin_login_show_status(const char* text, lv_color_t color);
static void admin_pwd_cache_put(int contact_idx, const char* password);
static const char* admin_pwd_cache_get(int contact_idx);
static void admin_login_screen_enter(int contact_idx);
static void admin_home_update_banner(bool is_admin, const char* contact_name);
static void admin_render_status_body();
static void create_admin_status_screen();
static void create_admin_cmd_screen();
static void create_admin_settings_screen();
static void create_admin_send_advert_screen();
static void create_admin_setting_placeholder_screen();
static void admin_settings_rebuild_list();
static void on_admin_setting_row_tap(lv_event_t *e);
static void on_admin_setting_back(lv_event_t *e);
static void create_admin_fw_info_screen();
static void admin_fw_info_request();
static void admin_fw_info_parse_and_render(const char* response);
static void on_admin_fw_info_refresh_tap(lv_event_t *e);
static void on_admin_fw_info_back(lv_event_t *e);
static void create_admin_neighbours_screen();
static void admin_neighbours_request_first_page();
static void admin_neighbours_request_next_page();
static void admin_neighbours_dispatch(uint16_t total_count,
                                       uint16_t page_count,
                                       uint16_t offset,
                                       const uint8_t* entries,
                                       int contact_idx);
static void admin_neighbours_append_row(const uint8_t* prefix,
                                         uint32_t secs_ago,
                                         int8_t snr_x4);
static void on_admin_neighbours_refresh_tap(lv_event_t *e);
static void on_admin_neighbours_back(lv_event_t *e);
static void on_admin_cmd_send_tap(lv_event_t *e);
static void on_admin_cmd_input_focused(lv_event_t *e);
static void on_admin_cmd_input_defocused(lv_event_t *e);
static void on_admin_cmd_kb_event(lv_event_t *e);
static void admin_cmd_add_entry(const char* cmd, const char* response,
                                 bool is_error);
static void admin_cmd_send_current();

// Settings → Contacts sub-screen
static void create_settings_contacts_screen();
static void goto_settings_contacts(lv_event_t *e);
static void goto_settings_from_contacts(lv_event_t *e);
static void settings_contacts_update_labels();
static int  settings_contacts_get_mode();
static void settings_contacts_apply_mode(int mode);
static void on_settings_contact_mode_tap(lv_event_t *e);
static void on_settings_autoadd_chat_tap(lv_event_t *e);
static void on_settings_autoadd_repeater_tap(lv_event_t *e);
static void on_settings_autoadd_room_tap(lv_event_t *e);
static void on_settings_autoadd_sensor_tap(lv_event_t *e);
static void on_settings_autoadd_overwrite_tap(lv_event_t *e);
static void on_settings_backup_to_sd_tap(lv_event_t *e);
static void on_settings_export_config_tap(lv_event_t *e);
// Debug Logs subpage handlers
static void on_settings_debug_logs_tap(lv_event_t *e);
static void create_debug_logs_screen();
static void on_debug_log_start_tap(lv_event_t *e);
static void on_debug_log_stop_tap(lv_event_t *e);
static void on_debug_log_export_tap(lv_event_t *e);
static void on_debug_logs_back(lv_event_t *e);
static void debug_log_update_status();
static void on_export_modal_identity_changed(lv_event_t *e);
static void on_export_modal_cancel(lv_event_t *e);
static void on_export_modal_export(lv_event_t *e);
// Retry-send long-press feature (channel + DM)
static void on_bubble_long_pressed(lv_event_t *e);
static void on_bubble_retry_ctx_delete(lv_event_t *e);
static void on_retry_modal_send(lv_event_t *e);
static void on_retry_modal_cancel(lv_event_t *e);
static void create_retry_modal();
static void create_path_modal();
static void on_settings_brightness_slider_event(lv_event_t *e);
#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
static void on_settings_kb_backlight_slider_event(lv_event_t *e);
#endif
static void on_settings_screen_off_tap(lv_event_t *e);
static void on_settings_kb_theme_tap(lv_event_t *e);
static void on_settings_kb_layout_tap(lv_event_t *e);
static void on_settings_font_scale_tap(lv_event_t *e);
#if MECK_BLE_ENABLED
static void on_settings_ble_tap(lv_event_t *e);
#endif
static void on_settings_drafts_tap(lv_event_t *e);
static void on_settings_antenna_tap(lv_event_t *e);
static void on_settings_orientation_tap(lv_event_t *e);
// Screen build / teardown / live orientation rebuild (orientation applies live).
static void meck_ui_build_screens();
static void meck_ui_teardown_screens();
static void meck_ui_apply_orientation_rebuild(void *unused);
static void goto_settings_wifi(lv_event_t *e);
static void create_settings_wifi_screen();
static void create_web_screen();
static void cb_open_web(lv_event_t *e);
#if MECK_BLE_KEYBOARD_ENABLED
static void goto_settings_keyboard(lv_event_t *e);
static void goto_settings_from_keyboard(lv_event_t *e);
static void create_settings_keyboard_screen();
#endif
static void on_gps_tile_long_press(lv_event_t *e);

static void on_ch_add_tap(lv_event_t *e);
static void on_ch_add_save(lv_event_t *e);
static void on_ch_add_cancel(lv_event_t *e);
static void on_ch_add_kb_event(lv_event_t *e);

// ============================================================================
// Home grid stub callbacks (for tiles that don't have ports yet)
// ============================================================================

static void show_not_implemented(const char* feature_name) {
    if (lbl_not_impl_title) lv_label_set_text(lbl_not_impl_title, feature_name);
    if (scr_not_implemented) lv_screen_load(scr_not_implemented);
}
static void cb_todo_reader(lv_event_t* e)   { meck_reader_ui_show_browser(); }
static void cb_todo_notes(lv_event_t* e)    { (void)e; meck_notes_ui_show_browser(); }
static void cb_todo_discover(lv_event_t* e) {
    // Wired to the Discover tile on the home grid. Mirrors Meck T-Deck
    // Pro's F-key Discover: shows nearby repeaters from the same heard-
    // adverts ring the Last Heard list reads, with an "Add" affordance
    // per row and a Rescan button that floods a self-advert so other
    // nodes can pick us up.
    goto_discover(e);
}
static void cb_todo_trace(lv_event_t* e)    { goto_trace(e); }
static void cb_todo_maps(lv_event_t* e)     { meck_map_ui_show(); }
static void goto_audio_browser(lv_event_t* e) {
    (void)e;
    meck_audio_ui_show_browser();
}
static void cb_todo_web(lv_event_t* e)      { show_not_implemented("Web"); }
static void cb_todo_voice(lv_event_t* e)    { show_not_implemented("Voice"); }
static void cb_todo_camera(lv_event_t* e)   { show_not_implemented("Camera"); }

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

#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
static void load_p4batt_prefs_from_nvs() {
    nvs_handle_t h;
    if (nvs_open("meckui", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t  src = 0;
    uint16_t cap = 5000;
    if (nvs_get_u8(h, "p4batt_src", &src) == ESP_OK && src <= 1) {
        g_p4batt_source = src;
    }
    if (nvs_get_u16(h, "p4batt_cap", &cap) == ESP_OK &&
        (cap == 5000 || cap == 10000)) {
        g_p4batt_cap_mah = cap;
    }
    nvs_close(h);
}

static void save_p4batt_prefs_to_nvs() {
    nvs_handle_t h;
    if (nvs_open("meckui", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h,  "p4batt_src", g_p4batt_source);
    nvs_set_u16(h, "p4batt_cap", g_p4batt_cap_mah);
    nvs_commit(h);
    nvs_close(h);
}
#endif // CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD

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

    // Web (creation idx 9): deep purple instead of the palette's amber,
    // which stacked directly under Notes' yellow in the landscape grid.
    // (Rose #FF0055 was tried and read as a near-twin of Discover's pink.)
    if (idx == 9) *border = lv_palette_main(LV_PALETTE_DEEP_PURPLE);
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
    lv_obj_set_size(btn, 100, 70);
    lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn, 8, 0);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    meck_set_font(lbl, &meck_montserrat_18, 0);
    lv_obj_center(lbl);

    lv_obj_add_event_cb(btn, goto_home, LV_EVENT_CLICKED, NULL);
    return btn;
}

// Lock a top-level screen container so it cannot pan or show a scrollbar.
// Every screen we build does its actual scrolling via inner widgets
// (settings list, message scroll, contact list, filter chip bar). Leaving
// the outer screen container scrollable means edge-taps near the keyboard
// or list boundaries are interpreted as pan gestures and slide the entire
// screen sideways — confusing and easy to trigger accidentally. Call this
// right after each scr_X = lv_obj_create(NULL).
//
// Linkage note: kept non-static so MeckAudioUI.cpp can reuse it through
// an extern "C" forward declaration. Same convention as Meck's other
// cross-translation-unit helpers.
extern "C" void lock_screen_scroll(lv_obj_t *scr) {
    if (!scr) return;
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
}

static lv_obj_t* create_tile_button(lv_obj_t *parent, const char *label,
                                    lv_event_cb_t cb, int col, int row) {
    // Grid is orientation-aware. Portrait: 2 columns x 5 rows (unchanged).
    // Landscape: 5 columns x 2 rows. The call sites pass (col,row) in the
    // portrait 2-column convention; we fold that to a linear index and
    // re-derive the cell for the current column count, so the same 10 tiles
    // reflow without touching any call site.
    int gapX  = 10;
    int gapY  = 10;
    int gridX = 20;
    bool landscape = (SCREEN_WIDTH > SCREEN_HEIGHT);
    // Landscape header is now two lines (IP lifted up to the battery row), so
    // the grid starts higher; portrait keeps the original 140.
    int gridY = landscape ? 110 : 140;
    int cols  = landscape ? 5 : 2;
    int tileW = (SCREEN_WIDTH - 2 * gridX - (cols - 1) * gapX) / cols;
    // Portrait keeps the original fixed 168 (5 rows fill the 1232 panel).
    // Landscape fits 2 rows into the height below the header, reserving 55 px
    // at the bottom so the swipe hint and the tileview scrollbar stay clear.
    int tileH = landscape ? ((SCREEN_HEIGHT - gridY - 55 - gapY) / 2) : 168;

    int idx = row * 2 + col;   // linear order from the portrait 2-col call sites
    int c   = idx % cols;
    int r   = idx / cols;
    int x = gridX + c * (tileW + gapX);
    int y = gridY + r * (tileH + gapY);

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
    meck_set_font(lbl, &meck_montserrat_28, 0);
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
    meck_set_font(lbl, &meck_montserrat_14, 0);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 5, -12);

    lv_obj_t *val = lv_label_create(btn);
    lv_label_set_text(val, "...");
    lv_obj_set_style_text_color(val, lv_color_white(), 0);
    meck_set_font(val, &meck_montserrat_18, 0);
    lv_obj_align(val, LV_ALIGN_LEFT_MID, 5, 10);
    if (value_label) *value_label = val;

    lv_obj_t *arrow = lv_label_create(btn);
    lv_label_set_text(arrow, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(arrow, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(arrow, &meck_montserrat_16, 0);
    lv_obj_align(arrow, LV_ALIGN_RIGHT_MID, -5, 0);

    return btn;
}

// Standardised sizing + layout for every Meck virtual keyboard. The
// default LVGL keymap dedicates ~6% of row width to each letter and 4-5%
// to function keys (ABC, backspace, enter, keyboard icon, etc) which on
// our screen translates to fat grey keys squeezing the letters. The
// custom ctrl_maps below shrink the function keys and widen the letters
// so they're closer to a phone-keyboard ratio. Row 4's icon buttons get
// shrunk hard so the space bar gets most of the row's width and the row
// stops looking empty.
//
// Anything positioned relative to the keyboard top (textarea, send
// button, message scroll height) reads MECK_KB_HEIGHT so layout stays
// consistent if the value changes later.
#define MECK_KB_HEIGHT (meck_kb_height())

// LVGL 9 + C++20: OR'ing a part constant with a state constant is between
// two different anonymous enums, which triggers
// -Wdeprecated-enum-enum-conversion. Wrapping both in uint32_t before the
// OR keeps the call sites readable and silences the warning across every
// lv_obj_*_style_* call that takes a part + state selector.
#define MECK_SEL(part, state) \
    ((lv_style_selector_t)((uint32_t)(part) | (uint32_t)(state)))

// ----------------------------------------------------------------------------
// CLICK_TRIG (enabled on every cell of every Meck ctrl_map via the macros
// below). Two reasons it's the right setting:
//
//   1. Symbol-key fix. LVGL's default keyboard ctrl_map sets CLICK_TRIG on
//      its function buttons (via LV_KEYBOARD_CTRL_BUTTON_FLAGS in
//      lv_keyboard.h). If our text-mode ctrl_maps didn't, a single tap on
//      "1#" would fire VALUE_CHANGED on press (no flag, our map) AND again
//      on release (CLICK_TRIG, LVGL's default special map after the mode
//      flip), and the default keyboard handler would toggle the mode twice
//      — the keyboard would appear to revert as soon as the user released.
//      Consistent CLICK_TRIG across every Meck map eliminates the asymmetry.
//
//   2. Accent popovers. With CLICK_TRIG, letters commit on SHORT_CLICKED
//      (release) instead of PRESSED, which lets on_kb_long_press() open an
//      accent popover for the held letter before any character has been
//      inserted into the textarea. After LONG_PRESSED fires, LVGL still
//      fires CLICKED on release; on_kb_long_press defends against that by
//      clearing the buttonmatrix's selected_button to NONE, so the default
//      handler bails before reaching its text-insertion path.
// ----------------------------------------------------------------------------

// Casts to keep g++ happy: lv_buttonmatrix_ctrl_t is a strongly-typed enum
// in C++, so OR'ing with an int width factor yields int (which can't
// implicitly convert back to the enum). MKB = width + CLICK_TRIG cell;
// MKB_NR adds NO_REPEAT on top.
#define MKB(w)    ((lv_buttonmatrix_ctrl_t)(LV_BUTTONMATRIX_CTRL_CLICK_TRIG | (w)))
#define MKB_NR(w) ((lv_buttonmatrix_ctrl_t)(LV_BUTTONMATRIX_CTRL_NO_REPEAT | LV_BUTTONMATRIX_CTRL_CLICK_TRIG | (w)))

// Width-factor layout — each entry is a LVGL button-matrix ctrl flag OR'd
// with the cell's width factor (relative to other cells in the same row).
// Lower-case map. Layout:
//   row 1: 10 letters                                    (backspace moved down)
//   row 2: 9 letters + big backspace                     (delete enlarged to 2x letter)
//   row 3: ',' + 7 letters + '.' + enter                 (',' took 1#'s slot)
//   row 4: [<] + '-' + ABC + space + 1# + [>]            (1# now flanks space;
//                                                         arrows pushed to the ends)
// Mode-switch behaviour for "1#" and "ABC"/"abc" keys is by label match
// in LVGL's default event handler, so position doesn't break anything.
static const char *meck_kb_map_lc[] = {
    "q","w","e","r","t","y","u","i","o","p",                                "\n",
    "a","s","d","f","g","h","j","k","l",            LV_SYMBOL_BACKSPACE,    "\n",
    ",","z","x","c","v","b","n","m",".",            LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "ABC", " ", "1#", LV_SYMBOL_RIGHT,
    ""
};

// Width factors. Row 1 letters at 7 (10 letters, total 70). Row 2 letters
// at 7 + delete at 14 (~double a letter so it's easy to hit, total 77).
// Row 3: ',' at 8 (took 1#'s slot, same width), letters at 7, '.' at 5,
// enter at 7 (total 69). Row 4: arrows at 2 on the ends, '-' at 3, ABC at
// 4, space at 10, 1# at 3 (took the old ',' slot — mode-switch so MKB_NR
// like ABC). Total still 24.
static const lv_buttonmatrix_ctrl_t meck_kb_ctrl_lc[] = {
    MKB(7),     MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7),
    MKB(7),     MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB_NR(14),
    MKB(8),     MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(5), MKB_NR(7),
    MKB(2),     MKB_NR(3), MKB_NR(4), MKB(10), MKB_NR(3), MKB(2)
};

// Upper-case map: same structure as lowercase with capital letters and
// "abc" instead of "ABC" as the case-toggle button.
static const char *meck_kb_map_uc[] = {
    "Q","W","E","R","T","Y","U","I","O","P",                                "\n",
    "A","S","D","F","G","H","J","K","L",            LV_SYMBOL_BACKSPACE,    "\n",
    ",","Z","X","C","V","B","N","M",".",            LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "abc", " ", "1#", LV_SYMBOL_RIGHT,
    ""
};

// Special / numbers map. Reached by tapping "1#" in either text mode.
// Tapping "abc" in row 2 col 0 OR in row 4 returns to TEXT_LOWER (LVGL's
// default handler dispatches both by label match).
//   row 1: digits 0-9 + backspace
//   row 2: abc + common chat symbols + enter
//   row 3: punctuation incl. '_' and ':' (relocated from the alpha rows)
//   row 4: same nav row as alpha for muscle-memory consistency
//
// IMPORTANT: ctrl_map below sets CLICK_TRIG on every cell, same as the
// alpha-mode ctrl_maps — see the long comment near MECK_KB_HEIGHT above
// for why consistency across all modes matters.
static const char *meck_kb_map_special[] = {
    "1","2","3","4","5","6","7","8","9","0",            LV_SYMBOL_BACKSPACE,  "\n",
    "abc","@","#","$","%","&","*","(",")","\"",         LV_SYMBOL_NEW_LINE,   "\n",
    "_","-","=","+","/","!","?",":",";","'",",",".",                          "\n",
    LV_SYMBOL_KEYBOARD, LV_SYMBOL_LEFT, "abc", " ", LV_SYMBOL_RIGHT, LV_SYMBOL_OK,
    ""
};

// Special-map widths. Row totals match the alpha rows above so the keys
// align horizontally across mode switches:
//   row 1: 11 cells, total 78 (10 digits at 7 + backspace at 8)
//   row 2: 11 cells, total 78 (abc at 8 + 9 syms at 7 + enter at 7)
//   row 3: 12 cells, total 84 — symbols are touch-typed less than letters
//                               so equal width at 7 is fine
//   row 4: same as alpha
static const lv_buttonmatrix_ctrl_t meck_kb_ctrl_special[] = {
    MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB_NR(8),
    MKB_NR(8), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB_NR(7),
    MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7),
    MKB_NR(2), MKB(2), MKB_NR(5), MKB(8), MKB(2), MKB_NR(2)
};

// ----------------------------------------------------------------------------
// Alternate physical layouts: AZERTY (French) and QWERTZ (German).
// ----------------------------------------------------------------------------
// AZERTY differs from QWERTY structurally: A and Q swap, W and Z swap, M moves
// from row 3 to row 2's end. Net effect: row 2 gains a letter (12 cells),
// row 3 loses one (9 cells). Row 1 stays at 11 cells. So AZERTY needs its
// own ctrl_map.
//
// QWERTZ differs from QWERTY by a single Y↔Z swap. Same shape everywhere, so
// it shares meck_kb_ctrl_lc.
//
// Same CLICK_TRIG-everywhere rule applies — see the long comment near
// MECK_KB_HEIGHT.
// ----------------------------------------------------------------------------

static const char *meck_kb_map_lc_azerty[] = {
    "a","z","e","r","t","y","u","i","o","p",                                "\n",
    "q","s","d","f","g","h","j","k","l","m",        LV_SYMBOL_BACKSPACE,    "\n",
    ",","w","x","c","v","b","n",".",                LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "ABC", " ", "1#", LV_SYMBOL_RIGHT,
    ""
};

static const char *meck_kb_map_uc_azerty[] = {
    "A","Z","E","R","T","Y","U","I","O","P",                                "\n",
    "Q","S","D","F","G","H","J","K","L","M",        LV_SYMBOL_BACKSPACE,    "\n",
    ",","W","X","C","V","B","N",".",                LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "abc", " ", "1#", LV_SYMBOL_RIGHT,
    ""
};

// AZERTY width factors (matching the QWERTY layout):
//   row 1: 10 letters at 7 (total 70)
//   row 2: 10 letters at 7 + big backspace at 14 (total 84). AZERTY's 'm'
//          stays on row 2 (one more letter than QWERTY/QWERTZ row 2).
//   row 3: ',' at 8 (took 1#'s slot) + 6 letters at 7 + '.' at 5 + enter
//          at 7 (total 62). One less letter than QWERTY/QWERTZ row 3
//          since 'm' is on row 2.
//   row 4: same as QWERTY — arrows on the ends, 1# next to space.
static const lv_buttonmatrix_ctrl_t meck_kb_ctrl_azerty[] = {
    MKB(7),     MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7),
    MKB(7),     MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB_NR(14),
    MKB(8),     MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(5), MKB_NR(7),
    MKB(2),     MKB_NR(3), MKB_NR(4), MKB(10), MKB_NR(3), MKB(2)
};

// QWERTZ: identical shape to QWERTY, only Y↔Z positions swap. Reuses
// meck_kb_ctrl_lc as the ctrl_map.
static const char *meck_kb_map_lc_qwertz[] = {
    "q","w","e","r","t","z","u","i","o","p",                                "\n",
    "a","s","d","f","g","h","j","k","l",            LV_SYMBOL_BACKSPACE,    "\n",
    ",","y","x","c","v","b","n","m",".",            LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "ABC", " ", "1#", LV_SYMBOL_RIGHT,
    ""
};

static const char *meck_kb_map_uc_qwertz[] = {
    "Q","W","E","R","T","Z","U","I","O","P",                                "\n",
    "A","S","D","F","G","H","J","K","L",            LV_SYMBOL_BACKSPACE,    "\n",
    ",","Y","X","C","V","B","N","M",".",            LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "abc", " ", "1#", LV_SYMBOL_RIGHT,
    ""
};

// ----------------------------------------------------------------------------
// Composer keyboard maps. Identical to the maps above except row 4 carries an
// emoji key (MECK_EMOJI_KEY) to the right of the space bar; the space
// bar narrows from 10 to 7 width units to make room. Applied only to
// kb_compose and kb_room_compose by meck_style_keyboard(). Tapping the emoji
// key opens the picker (see on_kb_emoji_event); it never types.
// ----------------------------------------------------------------------------
static const char *meck_kb_map_lc_em[] = {
    "q","w","e","r","t","y","u","i","o","p",                                "\n",
    "a","s","d","f","g","h","j","k","l",            LV_SYMBOL_BACKSPACE,    "\n",
    ",","z","x","c","v","b","n","m",".",            LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "ABC", " ", MECK_EMOJI_KEY, "1#", LV_SYMBOL_RIGHT,
    ""
};

static const char *meck_kb_map_uc_em[] = {
    "Q","W","E","R","T","Y","U","I","O","P",                                "\n",
    "A","S","D","F","G","H","J","K","L",            LV_SYMBOL_BACKSPACE,    "\n",
    ",","Z","X","C","V","B","N","M",".",            LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "abc", " ", MECK_EMOJI_KEY, "1#", LV_SYMBOL_RIGHT,
    ""
};

// Row 4 widths: arrows 2/2, '-' 3, ABC 4, space 7 (was 10), emoji 3, 1# 3.
// Row total stays 24. Rows 1-3 are copied unchanged from meck_kb_ctrl_lc.
static const lv_buttonmatrix_ctrl_t meck_kb_ctrl_em[] = {
    MKB(7),     MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7),
    MKB(7),     MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB_NR(14),
    MKB(8),     MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(5), MKB_NR(7),
    MKB(2),     MKB_NR(3), MKB_NR(4), MKB(7), MKB_NR(3), MKB_NR(3), MKB(2)
};

static const char *meck_kb_map_lc_azerty_em[] = {
    "a","z","e","r","t","y","u","i","o","p",                                "\n",
    "q","s","d","f","g","h","j","k","l","m",        LV_SYMBOL_BACKSPACE,    "\n",
    ",","w","x","c","v","b","n",".",                LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "ABC", " ", MECK_EMOJI_KEY, "1#", LV_SYMBOL_RIGHT,
    ""
};

static const char *meck_kb_map_uc_azerty_em[] = {
    "A","Z","E","R","T","Y","U","I","O","P",                                "\n",
    "Q","S","D","F","G","H","J","K","L","M",        LV_SYMBOL_BACKSPACE,    "\n",
    ",","W","X","C","V","B","N",".",                LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "abc", " ", MECK_EMOJI_KEY, "1#", LV_SYMBOL_RIGHT,
    ""
};

// AZERTY composer widths: rows 1-3 copied from meck_kb_ctrl_azerty, row 4
// same as meck_kb_ctrl_em row 4.
static const lv_buttonmatrix_ctrl_t meck_kb_ctrl_azerty_em[] = {
    MKB(7),     MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7),
    MKB(7),     MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB_NR(14),
    MKB(8),     MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(5), MKB_NR(7),
    MKB(2),     MKB_NR(3), MKB_NR(4), MKB(7), MKB_NR(3), MKB_NR(3), MKB(2)
};

static const char *meck_kb_map_lc_qwertz_em[] = {
    "q","w","e","r","t","z","u","i","o","p",                                "\n",
    "a","s","d","f","g","h","j","k","l",            LV_SYMBOL_BACKSPACE,    "\n",
    ",","y","x","c","v","b","n","m",".",            LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "ABC", " ", MECK_EMOJI_KEY, "1#", LV_SYMBOL_RIGHT,
    ""
};

static const char *meck_kb_map_uc_qwertz_em[] = {
    "Q","W","E","R","T","Z","U","I","O","P",                                "\n",
    "A","S","D","F","G","H","J","K","L",            LV_SYMBOL_BACKSPACE,    "\n",
    ",","Y","X","C","V","B","N","M",".",            LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "abc", " ", MECK_EMOJI_KEY, "1#", LV_SYMBOL_RIGHT,
    ""
};

// QWERTZ composer maps reuse meck_kb_ctrl_em (same cell shape as QWERTY).

// ----------------------------------------------------------------------------
// Cyrillic ЙЦУКЕН layout (kb_layout == 3). Standard Russian arrangement, which
// Bulgarian users also commonly use. Cyrillic has more letters than Latin, so
// the alpha rows are 12/12/12 cells and it needs its own ctrl_map. The control
// keys stay Latin ("ABC"/"abc"/"1#") because lv_keyboard matches those exact
// labels to switch case/number modes. The alpha-row "," is dropped (still on
// the 1# map) to make room for ё.
//
// Note: Ukrainian (і ї є ґ) and Serbian (ђ ј љ њ ћ џ) use different letter sets
// and would each need their own layout; this one is Russian/Bulgarian.
// ----------------------------------------------------------------------------
static const char *meck_kb_map_lc_cyr[] = {
    "й","ц","у","к","е","н","г","ш","щ","з","х","ъ",                        "\n",
    "ф","ы","в","а","п","р","о","л","д","ж","э",    LV_SYMBOL_BACKSPACE,    "\n",
    "я","ч","с","м","и","т","ь","б","ю","ё",".",    LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "ABC", " ", "1#", LV_SYMBOL_RIGHT,
    ""
};

static const char *meck_kb_map_uc_cyr[] = {
    "Й","Ц","У","К","Е","Н","Г","Ш","Щ","З","Х","Ъ",                        "\n",
    "Ф","Ы","В","А","П","Р","О","Л","Д","Ж","Э",    LV_SYMBOL_BACKSPACE,    "\n",
    "Я","Ч","С","М","И","Т","Ь","Б","Ю","Ё",".",    LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "abc", " ", "1#", LV_SYMBOL_RIGHT,
    ""
};

// Cyrillic widths: 12 letters/row at 7; backspace 14, '.' 5, enter 7. Row 4
// matches QWERTY. Each row fills the keyboard width independently, so the
// wider letter rows just render slightly narrower cells than the Latin layouts.
static const lv_buttonmatrix_ctrl_t meck_kb_ctrl_cyr[] = {
    MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7),
    MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB_NR(14),
    MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(5), MKB_NR(7),
    MKB(2), MKB_NR(3), MKB_NR(4), MKB(10), MKB_NR(3), MKB(2)
};

// Cyrillic composer maps: row 4 gains the emoji key (space shrinks), matching
// the Latin _em variants. Rows 1-3 are unchanged from the base Cyrillic maps.
static const char *meck_kb_map_lc_cyr_em[] = {
    "й","ц","у","к","е","н","г","ш","щ","з","х","ъ",                        "\n",
    "ф","ы","в","а","п","р","о","л","д","ж","э",    LV_SYMBOL_BACKSPACE,    "\n",
    "я","ч","с","м","и","т","ь","б","ю","ё",".",    LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "ABC", " ", MECK_EMOJI_KEY, "1#", LV_SYMBOL_RIGHT,
    ""
};

static const char *meck_kb_map_uc_cyr_em[] = {
    "Й","Ц","У","К","Е","Н","Г","Ш","Щ","З","Х","Ъ",                        "\n",
    "Ф","Ы","В","А","П","Р","О","Л","Д","Ж","Э",    LV_SYMBOL_BACKSPACE,    "\n",
    "Я","Ч","С","М","И","Т","Ь","Б","Ю","Ё",".",    LV_SYMBOL_NEW_LINE,     "\n",
    LV_SYMBOL_LEFT, "-", "abc", " ", MECK_EMOJI_KEY, "1#", LV_SYMBOL_RIGHT,
    ""
};

static const lv_buttonmatrix_ctrl_t meck_kb_ctrl_cyr_em[] = {
    MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7),
    MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB_NR(14),
    MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(5), MKB_NR(7),
    MKB(2), MKB_NR(3), MKB_NR(4), MKB(7), MKB_NR(3), MKB_NR(3), MKB(2)
};

static const char *meck_kb_map_special_em[] = {
    "1","2","3","4","5","6","7","8","9","0",            LV_SYMBOL_BACKSPACE,  "\n",
    "abc","@","#","$","%","&","*","(",")","\"",         LV_SYMBOL_NEW_LINE,   "\n",
    "_","-","=","+","/","!","?",":",";","'",",",".",                          "\n",
    LV_SYMBOL_KEYBOARD, LV_SYMBOL_LEFT, "abc", " ", MECK_EMOJI_KEY, LV_SYMBOL_RIGHT, LV_SYMBOL_OK,
    ""
};

// Special composer widths: rows 1-3 copied from meck_kb_ctrl_special, row 4
// narrows space from 8 to 5 and places emoji (3) to its right (row total stays 21).
static const lv_buttonmatrix_ctrl_t meck_kb_ctrl_special_em[] = {
    MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB_NR(8),
    MKB_NR(8), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB_NR(7),
    MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7), MKB(7),
    MKB_NR(2), MKB(2), MKB_NR(5), MKB(5), MKB_NR(3), MKB(2), MKB_NR(2)
};

// ============================================================================
// Accent popover — iOS-style long-press popover showing Czech + French
// accented variants of the held letter. Tap a variant to insert it; tap
// anywhere else to dismiss. Disabled in symbol/number mode (no diacritics
// for digits or punctuation) and on letters with no entries in the table.
// ============================================================================

// Czech + French variants per base letter. Order within each group:
// Czech-only first, then French (with shared characters like 'é' / 'à' /
// 'ç' kept in the order a user is most likely to want them).
static const char *const meck_accents_a_lc[] = {"á", "à", "â", NULL};
static const char *const meck_accents_c_lc[] = {"č", "ç", NULL};
static const char *const meck_accents_e_lc[] = {"é", "è", "ê", "ë", "ě", NULL};
static const char *const meck_accents_i_lc[] = {"í", "ï", NULL};
static const char *const meck_accents_o_lc[] = {"ô", NULL};
static const char *const meck_accents_r_lc[] = {"ř", NULL};
static const char *const meck_accents_s_lc[] = {"š", NULL};
static const char *const meck_accents_u_lc[] = {"ù", "û", "ü", NULL};
static const char *const meck_accents_y_lc[] = {"ý", NULL};
static const char *const meck_accents_z_lc[] = {"ž", NULL};

static const char *const meck_accents_a_uc[] = {"Á", "À", "Â", NULL};
static const char *const meck_accents_c_uc[] = {"Č", "Ç", NULL};
static const char *const meck_accents_e_uc[] = {"É", "È", "Ê", "Ë", "Ě", NULL};
static const char *const meck_accents_i_uc[] = {"Í", "Ï", NULL};
static const char *const meck_accents_o_uc[] = {"Ô", NULL};
static const char *const meck_accents_r_uc[] = {"Ř", NULL};
static const char *const meck_accents_s_uc[] = {"Š", NULL};
static const char *const meck_accents_u_uc[] = {"Ù", "Û", "Ü", NULL};
static const char *const meck_accents_y_uc[] = {"Ý", NULL};
static const char *const meck_accents_z_uc[] = {"Ž", NULL};

typedef struct {
    char base;
    const char *const *variants;
} MeckAccentEntry;

static const MeckAccentEntry meck_accent_table_lc[] = {
    {'a', meck_accents_a_lc},
    {'c', meck_accents_c_lc},
    {'e', meck_accents_e_lc},
    {'i', meck_accents_i_lc},
    {'o', meck_accents_o_lc},
    {'r', meck_accents_r_lc},
    {'s', meck_accents_s_lc},
    {'u', meck_accents_u_lc},
    {'y', meck_accents_y_lc},
    {'z', meck_accents_z_lc},
};

static const MeckAccentEntry meck_accent_table_uc[] = {
    {'A', meck_accents_a_uc},
    {'C', meck_accents_c_uc},
    {'E', meck_accents_e_uc},
    {'I', meck_accents_i_uc},
    {'O', meck_accents_o_uc},
    {'R', meck_accents_r_uc},
    {'S', meck_accents_s_uc},
    {'U', meck_accents_u_uc},
    {'Y', meck_accents_y_uc},
    {'Z', meck_accents_z_uc},
};

// Lookup the variant list for a base ASCII letter; returns NULL if the
// letter has no accent entries (so the long-press handler can no-op).
static const char *const *meck_lookup_accents(char base) {
    bool is_upper = (base >= 'A' && base <= 'Z');
    const MeckAccentEntry *table = is_upper ? meck_accent_table_uc
                                            : meck_accent_table_lc;
    size_t n = is_upper
        ? (sizeof(meck_accent_table_uc) / sizeof(meck_accent_table_uc[0]))
        : (sizeof(meck_accent_table_lc) / sizeof(meck_accent_table_lc[0]));
    for (size_t i = 0; i < n; i++) {
        if (table[i].base == base) return table[i].variants;
    }
    return NULL;
}

// ----------------------------------------------------------------------------
// Popover state. Only one popover can be visible at a time. The overlay is
// a full-screen transparent rect drawn between the keyboard and the
// popover; it catches taps outside the popover and dismisses it (modal
// behaviour). The popover itself is a small horizontal buttonmatrix shown
// directly above the held key.
// ----------------------------------------------------------------------------
static lv_obj_t *g_accent_overlay   = NULL;
static lv_obj_t *g_accent_popover   = NULL;
static lv_obj_t *g_accent_target_ta = NULL; // textarea to insert into
static lv_obj_t *g_accent_source_kb = NULL; // keyboard that opened us

// Buttonmatrix map storage. Slot 0 is the base letter ("tap me to type the
// plain letter and dismiss"); slots 1..N are the variants; final slot is
// the LVGL "" terminator. 16 is plenty for any single letter's accent set.
static const char *g_accent_map[16] = {};

static void meck_close_accent_popover() {
    if (g_accent_source_kb) {
        lv_obj_remove_state(g_accent_source_kb, LV_STATE_DISABLED);
        g_accent_source_kb = NULL;
    }
    if (g_accent_popover) {
        lv_obj_delete(g_accent_popover);
        g_accent_popover = NULL;
    }
    if (g_accent_overlay) {
        lv_obj_delete(g_accent_overlay);
        g_accent_overlay = NULL;
    }
    g_accent_target_ta = NULL;
}

// Tap landed outside the popover. Dismiss without typing.
static void on_accent_overlay_pressed(lv_event_t *e) {
    meck_close_accent_popover();
}

// Tap landed on a variant cell (or on the base-letter cell at index 0).
// Insert the cell's text into the textarea and dismiss.
static void on_accent_popover_value_changed(lv_event_t *e) {
    lv_obj_t *bm = (lv_obj_t *)lv_event_get_target(e);
    uint32_t btn_id = lv_buttonmatrix_get_selected_button(bm);
    if (btn_id != LV_BUTTONMATRIX_BUTTON_NONE && g_accent_target_ta) {
        const char *txt = lv_buttonmatrix_get_button_text(bm, btn_id);
        if (txt && txt[0]) {
            lv_textarea_add_text(g_accent_target_ta, txt);
        }
    }
    meck_close_accent_popover();
}

static void meck_open_accent_popover(lv_obj_t *kb, lv_obj_t *ta,
                                      const lv_point_t *anchor,
                                      const char *base,
                                      const char *const *variants) {
    // Make sure no stale popover is lingering before we build a new one.
    meck_close_accent_popover();

    // Build the buttonmatrix map: base letter at slot 0, variants after,
    // "" terminator at the end. Keep one slot in reserve for safety.
    int n = 0;
    g_accent_map[n++] = base;
    for (int i = 0; variants[i] != NULL && n < 14; i++) {
        g_accent_map[n++] = variants[i];
    }
    int cell_count = n;
    g_accent_map[n] = "";

    // Theme: dark unless the user explicitly picked light.
    uint8_t dark = 1;
    Meck *mesh = meck_get_instance();
    if (mesh) {
        P4NodePrefs *prefs = mesh->getNodePrefs();
        if (prefs) dark = (prefs->kb_dark_mode == 0) ? 1 : 0;
    }

    lv_obj_t *parent = lv_screen_active();

    // Full-screen transparent overlay below the popover. lv_obj_remove_style_all
    // clears LVGL's default container styling so the overlay is truly invisible.
    g_accent_overlay = lv_obj_create(parent);
    lv_obj_remove_style_all(g_accent_overlay);
    lv_obj_set_size(g_accent_overlay, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(g_accent_overlay, 0, 0);
    lv_obj_add_flag(g_accent_overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(g_accent_overlay, on_accent_overlay_pressed,
                        LV_EVENT_PRESSED, NULL);

    // The popover itself. Sized by variant count; positioned above the
    // touch point and clipped to the screen width so it can't run off the
    // edges on letters like 'q' or 'p' near the keyboard's left/right.
    g_accent_popover = lv_buttonmatrix_create(parent);
    lv_buttonmatrix_set_map(g_accent_popover, g_accent_map);

    int cell_w = 56;
    int popover_w = cell_w * cell_count;
    int popover_h = 70;

    int x = anchor->x - popover_w / 2;
    if (x < 4)                            x = 4;
    if (x + popover_w > SCREEN_WIDTH - 4) x = SCREEN_WIDTH - popover_w - 4;
    // Place the popover above the touch with enough clearance to sit on
    // top of the held key rather than under the user's fingertip. 70 px
    // is roughly the half-height of a key in our 560 px / 4-row keyboard,
    // so the popover bottom lands near the top edge of the held key.
    // Fallback drops below the touch if there isn't enough room above.
    int y = anchor->y - popover_h - 70;
    if (y < 4) y = anchor->y + 70;

    lv_obj_set_size(g_accent_popover, popover_w, popover_h);
    lv_obj_set_pos(g_accent_popover, x, y);
    lv_obj_set_style_text_font(g_accent_popover,
                                &meck_montserrat_24, LV_PART_ITEMS);
    lv_obj_set_style_pad_all(g_accent_popover, 4, LV_PART_MAIN);
    lv_obj_set_style_pad_row(g_accent_popover, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(g_accent_popover, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(g_accent_popover, 8, LV_PART_MAIN);

    // Shadow lifts the popover off the keyboard underneath in both themes.
    lv_obj_set_style_shadow_width(g_accent_popover, 12, LV_PART_MAIN);
    lv_obj_set_style_shadow_color(g_accent_popover, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_shadow_opa(g_accent_popover, LV_OPA_60, LV_PART_MAIN);

    if (dark) {
        lv_obj_set_style_bg_color(g_accent_popover, lv_color_make(15, 15, 20),
                                  LV_PART_MAIN);
        lv_obj_set_style_bg_color(g_accent_popover, lv_color_make(45, 45, 55),
                                  MECK_SEL(LV_PART_ITEMS, LV_STATE_DEFAULT));
        lv_obj_set_style_bg_color(g_accent_popover, lv_color_make(80, 80, 95),
                                  MECK_SEL(LV_PART_ITEMS, LV_STATE_PRESSED));
        lv_obj_set_style_text_color(g_accent_popover, lv_color_white(),
                                    LV_PART_ITEMS);
        lv_obj_set_style_border_color(g_accent_popover, lv_color_make(60, 60, 75),
                                      LV_PART_ITEMS);
        lv_obj_set_style_border_width(g_accent_popover, 1, LV_PART_ITEMS);
    }
    // Light theme: leave LVGL defaults (white bg, dark text) — looks fine.

    lv_obj_add_event_cb(g_accent_popover, on_accent_popover_value_changed,
                        LV_EVENT_VALUE_CHANGED, NULL);

    // Visual: gray the source keyboard out so it's clear input is captured
    // by the popover. The modal overlay already blocks any taps that would
    // reach the keyboard, but the disabled state gives a clearer visual cue.
    g_accent_source_kb = kb;
    g_accent_target_ta = ta;
    lv_obj_add_state(kb, LV_STATE_DISABLED);
}

// LONG_PRESSED handler. Shared by all three Meck keyboards. Resolves the
// held button to a base letter, looks up its accent set, and opens the
// popover. Clears the source keyboard's selected_button to LV_BUTTONMATRIX_
// BUTTON_NONE so the inevitable release-CLICKED event doesn't trigger
// LVGL's default keyboard handler to type the held letter on top of
// whatever the popover does.
static void on_kb_long_press(lv_event_t *e) {
    lv_obj_t *kb = (lv_obj_t *)lv_event_get_target(e);
    uint32_t btn_id = lv_buttonmatrix_get_selected_button(kb);
    if (btn_id == LV_BUTTONMATRIX_BUTTON_NONE) return;

    const char *txt = lv_buttonmatrix_get_button_text(kb, btn_id);
    if (!txt || strlen(txt) != 1) return;
    char base = txt[0];
    bool is_letter = (base >= 'a' && base <= 'z') || (base >= 'A' && base <= 'Z');
    if (!is_letter) return;

    const char *const *variants = meck_lookup_accents(base);
    if (!variants) return;  // letter has no accents in our tables

    // LVGL 9.3 has no public lv_buttonmatrix_get_button_area, and walking
    // the internal button_areas[] field would be fragile across versions.
    // Instead anchor the popover to the active indev's current touch
    // point — which by construction sits inside the held key, so the
    // popover lands above the right key without us recomputing layout.
    lv_indev_t *indev = lv_indev_active();
    lv_point_t anchor = {SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2};
    if (indev) lv_indev_get_point(indev, &anchor);

    lv_obj_t *ta = lv_keyboard_get_textarea(kb);
    if (!ta) return;

    // Defuse the eventual release-VC: with btn_id_sel = NONE, the default
    // keyboard handler bails before lv_textarea_add_text. See the long
    // CLICK_TRIG comment near MECK_KB_HEIGHT for the full rationale.
    lv_buttonmatrix_set_selected_button(kb, LV_BUTTONMATRIX_BUTTON_NONE);

    meck_open_accent_popover(kb, ta, &anchor, txt, variants);
}

// PRESSED handler for the two composer keyboards. If the pressed key is the
// emoji key, open the picker instead of typing. Nulling the selected button
// makes LVGL's default keyboard handler bail before the release-VALUE_CHANGED
// inserts the key glyph (the same defuse on_kb_long_press uses).
static void on_kb_emoji_event(lv_event_t *e) {
    lv_obj_t *kb = (lv_obj_t *)lv_event_get_target(e);
    uint32_t id = lv_buttonmatrix_get_selected_button(kb);
    if (id == LV_BUTTONMATRIX_BUTTON_NONE) return;

    const char *txt = lv_buttonmatrix_get_button_text(kb, id);
    if (!txt || strcmp(txt, MECK_EMOJI_KEY) != 0) return;

    lv_obj_t *ta = lv_keyboard_get_textarea(kb);
    lv_buttonmatrix_set_selected_button(kb, LV_BUTTONMATRIX_BUTTON_NONE);

    uint8_t dark = 1;
    Meck *mesh = meck_get_instance();
    if (mesh) {
        P4NodePrefs *prefs = mesh->getNodePrefs();
        if (prefs) dark = (prefs->kb_dark_mode == 0) ? 1 : 0;
    }
    meck_emoji_picker_open(kb, ta, dark != 0);
}

// ----------------------------------------------------------------------------
// Keyboard styling — picks layout from prefs->kb_layout and applies the
// dark theme overrides if prefs->kb_dark_mode == 0 (the default).
//
// Idempotent: calling it again on an existing keyboard re-applies the maps
// and theme. meck_kb_restyle_all() below uses that to live-update every
// keyboard when the user toggles a Settings row.
// ----------------------------------------------------------------------------
static void meck_style_keyboard(lv_obj_t *kb) {
    if (!kb) return;
    lv_obj_set_size(kb, SCREEN_WIDTH, MECK_KB_HEIGHT);
    lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    meck_set_font(kb, &meck_montserrat_24, LV_PART_ITEMS);
    // Tighter row pad than before — extra height was redistributing into
    // gaps, not key area. 5 px is enough visual separation; the rest of
    // the height goes to the keys themselves.
    lv_obj_set_style_pad_row(kb, 5, LV_PART_MAIN);
    lv_obj_set_style_pad_column(kb, 4, LV_PART_MAIN);

    // Read prefs. If they aren't loaded yet (unlikely — meck_ui_init runs
    // after meck_app_init — but be defensive), default to dark + QWERTY to
    // match Meck's intended look.
    uint8_t dark   = 1;
    uint8_t layout = 0;
    Meck* mesh = meck_get_instance();
    if (mesh) {
        P4NodePrefs* prefs = mesh->getNodePrefs();
        if (prefs) {
            dark   = (prefs->kb_dark_mode == 0) ? 1 : 0;  // 0 = dark
            layout = prefs->kb_layout;
        }
    }

    // Pick layout. QWERTZ shares QWERTY's ctrl_map (same cell shape).
    const char **map_lc = meck_kb_map_lc;
    const char **map_uc = meck_kb_map_uc;
    const lv_buttonmatrix_ctrl_t *ctrl = meck_kb_ctrl_lc;
    switch (layout) {
        case 1:  // AZERTY
            map_lc = meck_kb_map_lc_azerty;
            map_uc = meck_kb_map_uc_azerty;
            ctrl   = meck_kb_ctrl_azerty;
            break;
        case 2:  // QWERTZ
            map_lc = meck_kb_map_lc_qwertz;
            map_uc = meck_kb_map_uc_qwertz;
            // ctrl stays as meck_kb_ctrl_lc — same shape as QWERTY.
            break;
        case 3:  // Cyrillic ЙЦУКЕН
            map_lc = meck_kb_map_lc_cyr;
            map_uc = meck_kb_map_uc_cyr;
            ctrl   = meck_kb_ctrl_cyr;
            break;
        default: // 0 = QWERTY, also the fallback for any future value
            break;
    }

    // Composer keyboards get an emoji key in row 4 (the space bar shrinks to
    // make room). Every other keyboard keeps the standard maps. The pointer
    // check works because kb_compose/kb_room_compose are assigned before this
    // runs at their creation sites and by the restyle-all path.
    const char **map_special = meck_kb_map_special;
    const lv_buttonmatrix_ctrl_t *ctrl_special = meck_kb_ctrl_special;
    if (kb == kb_compose || kb == kb_room_compose) {
        if (map_lc == meck_kb_map_lc_azerty) {
            map_lc = meck_kb_map_lc_azerty_em;
            map_uc = meck_kb_map_uc_azerty_em;
            ctrl   = meck_kb_ctrl_azerty_em;
        } else if (map_lc == meck_kb_map_lc_qwertz) {
            map_lc = meck_kb_map_lc_qwertz_em;
            map_uc = meck_kb_map_uc_qwertz_em;
            ctrl   = meck_kb_ctrl_em;
        } else if (map_lc == meck_kb_map_lc_cyr) {
            map_lc = meck_kb_map_lc_cyr_em;
            map_uc = meck_kb_map_uc_cyr_em;
            ctrl   = meck_kb_ctrl_cyr_em;
        } else {
            map_lc = meck_kb_map_lc_em;
            map_uc = meck_kb_map_uc_em;
            ctrl   = meck_kb_ctrl_em;
        }
        map_special  = meck_kb_map_special_em;
        ctrl_special = meck_kb_ctrl_special_em;
    }

    // SPECIAL gets its own map + ctrl_map so CLICK_TRIG stays absent in
    // every mode — see the long comment block above the maps for why.
    lv_keyboard_set_map(kb, LV_KEYBOARD_MODE_TEXT_LOWER, map_lc, ctrl);
    lv_keyboard_set_map(kb, LV_KEYBOARD_MODE_TEXT_UPPER, map_uc, ctrl);
    lv_keyboard_set_map(kb, LV_KEYBOARD_MODE_SPECIAL,
                        map_special, ctrl_special);

    // Dark theme: matches the rest of the Meck UI (the panel/ID-row colour
    // is also lv_color_make(15, 15, 20), so the keyboard blends in).
    // Light theme: clear any previously-applied dark overrides so LVGL's
    // built-in theme takes over again. lv_obj_remove_local_style_prop
    // drops the inline property; without these calls a runtime light→dark
    // toggle would leave dark colours stuck on the keyboard.
    if (dark) {
        lv_obj_set_style_bg_color(kb, lv_color_make(15, 15, 20), LV_PART_MAIN);
        lv_obj_set_style_bg_color(kb, lv_color_make(45, 45, 55),
                                  MECK_SEL(LV_PART_ITEMS, LV_STATE_DEFAULT));
        lv_obj_set_style_bg_color(kb, lv_color_make(80, 80, 95),
                                  MECK_SEL(LV_PART_ITEMS, LV_STATE_PRESSED));
        lv_obj_set_style_text_color(kb, lv_color_white(), LV_PART_ITEMS);
        lv_obj_set_style_border_color(kb, lv_color_make(60, 60, 75),
                                      LV_PART_ITEMS);
        lv_obj_set_style_border_width(kb, 1, LV_PART_ITEMS);
    } else {
        lv_obj_remove_local_style_prop(kb, LV_STYLE_BG_COLOR, LV_PART_MAIN);
        lv_obj_remove_local_style_prop(kb, LV_STYLE_BG_COLOR,
                                       MECK_SEL(LV_PART_ITEMS, LV_STATE_DEFAULT));
        lv_obj_remove_local_style_prop(kb, LV_STYLE_BG_COLOR,
                                       MECK_SEL(LV_PART_ITEMS, LV_STATE_PRESSED));
        lv_obj_remove_local_style_prop(kb, LV_STYLE_TEXT_COLOR, LV_PART_ITEMS);
        lv_obj_remove_local_style_prop(kb, LV_STYLE_BORDER_COLOR, LV_PART_ITEMS);
        lv_obj_remove_local_style_prop(kb, LV_STYLE_BORDER_WIDTH, LV_PART_ITEMS);
    }
}

// Exposed for other UI modules (the notes editor) so their on-screen keyboard
// matches the composer's styling and honours the layout preference. Styles at
// call time; if the layout pref changes later, the keyboard re-styles on the
// next orientation rebuild (it isn't in the live restyle-all list).
extern "C" void meck_ui_style_keyboard(lv_obj_t* kb) {
    meck_style_keyboard(kb);
}

// Current LOCAL epoch (mesh RTC UTC plus the user's UTC offset), or 0 if the
// clock isn't synced yet. Exposed so modules can build timestamped filenames
// (e.g. notes) without reaching into the mesh themselves.
extern "C" uint32_t meck_ui_local_epoch(void) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return 0;
    mesh::RTCClock* rtc = mesh->getRTCClock();
    if (!rtc) return 0;
    uint32_t utc = rtc->getCurrentTime();
    if (utc < 1750000000u) return 0;   // not yet synced
    int off = 0;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (prefs) off = prefs->utc_offset_hours;
    return (uint32_t)((int64_t)utc + (int64_t)off * 3600);
}

// Re-apply theme + layout to every active Meck keyboard. Called from the
// Settings toggle handlers so the change is immediately visible the next
// time the user opens any of them, without needing to navigate to the
// compose flow first.
static void meck_kb_restyle_all() {
    if (kb_compose)        meck_style_keyboard(kb_compose);
    if (kb_settings)       meck_style_keyboard(kb_settings);
    if (kb_ch_add)         meck_style_keyboard(kb_ch_add);
    if (kb_admin_password) meck_style_keyboard(kb_admin_password);
    if (kb_admin_cmd_input) meck_style_keyboard(kb_admin_cmd_input);
    if (kb_num_edit)        meck_style_keyboard(kb_num_edit);
    if (kb_region_edit)     meck_style_keyboard(kb_region_edit);
    if (kb_ch_scope_edit)   meck_style_keyboard(kb_ch_scope_edit);
    if (kb_ch_settings_add) meck_style_keyboard(kb_ch_settings_add);
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
    if (lbl_set_freq) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%.3f MHz", (double)prefs->freq);
        lv_label_set_text(lbl_set_freq, buf);
    }
    if (lbl_set_bw) {
        char buf[16];
        if (prefs->bw == (float)(int)prefs->bw) {
            snprintf(buf, sizeof(buf), "%.0f kHz", (double)prefs->bw);
        } else {
            snprintf(buf, sizeof(buf), "%.2f kHz", (double)prefs->bw);
        }
        lv_label_set_text(lbl_set_bw, buf);
    }
    if (lbl_set_sf) {
        char buf[8];
        snprintf(buf, sizeof(buf), "SF %d", (int)prefs->sf);
        lv_label_set_text(lbl_set_sf, buf);
    }
    if (lbl_set_cr) {
        char buf[8];
        snprintf(buf, sizeof(buf), "4/%d", (int)prefs->cr);
        lv_label_set_text(lbl_set_cr, buf);
    }
    if (lbl_set_region) {
        lv_label_set_text(lbl_set_region,
            prefs->default_scope_name[0] ? prefs->default_scope_name : "(none)");
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
    if (slider_set_brightness) {
        uint8_t b = prefs->screen_brightness ? prefs->screen_brightness : 200;
        int pct = (int)((unsigned)b * 100 / 255);
        if (pct < 12) pct = 12;
        if (pct > 100) pct = 100;
        lv_slider_set_value(slider_set_brightness, pct, LV_ANIM_OFF);
        if (lbl_set_brightness_pct) {
            char buf[8];
            snprintf(buf, sizeof(buf), "%d%%", pct);
            lv_label_set_text(lbl_set_brightness_pct, buf);
        }
    }
#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
    if (slider_set_kb_backlight) {
        int kpct = prefs->kb_backlight_pct ? prefs->kb_backlight_pct : 25;
        if (kpct < 5)   kpct = 5;
        if (kpct > 100) kpct = 100;
        lv_slider_set_value(slider_set_kb_backlight, kpct, LV_ANIM_OFF);
        if (lbl_set_kb_backlight_pct) {
            char buf[8];
            snprintf(buf, sizeof(buf), "%d%%", kpct);
            lv_label_set_text(lbl_set_kb_backlight_pct, buf);
        }
    }
#endif
    if (lbl_set_screen_off) {
        char buf[16];
        if (prefs->screen_off_minutes == 0) {
            snprintf(buf, sizeof(buf), "Never");
        } else {
            snprintf(buf, sizeof(buf), "%u min",
                     (unsigned)prefs->screen_off_minutes);
        }
        lv_label_set_text(lbl_set_screen_off, buf);
    }
    if (lbl_set_kb_theme) {
        lv_label_set_text(lbl_set_kb_theme,
            prefs->kb_dark_mode == 0 ? "Dark" : "Light");
    }
    if (lbl_set_kb_layout) {
        const char* name;
        switch (prefs->kb_layout) {
            case 0:  name = "QWERTY"; break;
            case 1:  name = "AZERTY"; break;
            case 2:  name = "QWERTZ"; break;
            case 3:  name = "ЙЦУКЕН"; break;
            default: name = "?";       break;
        }
        lv_label_set_text(lbl_set_kb_layout, name);
    }
    if (lbl_set_font_scale) {
        lv_label_set_text(lbl_set_font_scale, meck_font_scale_name(prefs->font_scale));
    }
#if MECK_BLE_ENABLED
    if (lbl_set_ble) {
        lv_label_set_text(lbl_set_ble,
            prefs->ble_enabled != 0 ? "On" : "Off");
    }
#endif
    if (lbl_set_drafts) {
        lv_label_set_text(lbl_set_drafts,
            prefs->save_drafts != 0 ? "On" : "Off");
    }
    if (lbl_set_antenna) {
        lv_label_set_text(lbl_set_antenna,
            prefs->antenna != 0 ? "External (RF2)" : "Internal (RF1)");
    }
    if (lbl_set_orientation) {
        lv_label_set_text(lbl_set_orientation,
            prefs->orientation != 0 ? "Landscape" : "Portrait");
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
//
// Linkage note: kept non-static so MeckMapScreen.cpp can reuse it through
// an extern "C" forward declaration. Same convention as lock_screen_scroll.
// ============================================================================

extern "C" void strip_unrenderable(const char *src, char *dst, size_t dst_sz) {
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
            // Keep Latin-1 and the Cyrillic block the fonts cover (U+0400..
            // U+045F plus Ґ/ґ at U+0490..U+0491). Other 2-byte codepoints
            // have no glyph, so drop them to avoid tofu boxes.
            bool keep = (cp <= 0x00FF) ||
                        (cp >= 0x0400 && cp <= 0x045F) ||
                        (cp == 0x0490 || cp == 0x0491);
            if (keep && di + 2 < dst_sz) {
                dst[di++] = (char)p[0];
                dst[di++] = (char)p[1];
            }
            p += 2;
        } else if ((b & 0xF0) == 0xE0) {
            // 3-byte UTF-8: U+0800..U+FFFF. Above the Montserrat range, but
            // keep the ones the emoji fallback can render (e.g. U+2639,
            // U+2665, and the VS-16 selector). Drop the rest.
            if ((p[1] & 0xC0) != 0x80 || (p[2] & 0xC0) != 0x80) { p++; continue; }
            uint32_t cp = ((uint32_t)(b & 0x0F) << 12) |
                          ((uint32_t)(p[1] & 0x3F) << 6) |
                          (uint32_t)(p[2] & 0x3F);
            if (meck_emoji_is_renderable(cp) && di + 3 < dst_sz) {
                dst[di++] = (char)p[0];
                dst[di++] = (char)p[1];
                dst[di++] = (char)p[2];
            }
            p += 3;
        } else if ((b & 0xF8) == 0xF0) {
            // 4-byte UTF-8: supplementary planes (most emoji). Keep the ones
            // the emoji fallback can render; drop everything else.
            if ((p[1] & 0xC0) != 0x80 || (p[2] & 0xC0) != 0x80 ||
                (p[3] & 0xC0) != 0x80) { p++; continue; }
            uint32_t cp = ((uint32_t)(b & 0x07) << 18) |
                          ((uint32_t)(p[1] & 0x3F) << 12) |
                          ((uint32_t)(p[2] & 0x3F) << 6) |
                          (uint32_t)(p[3] & 0x3F);
            if (meck_emoji_is_renderable(cp) && di + 4 < dst_sz) {
                dst[di++] = (char)p[0];
                dst[di++] = (char)p[1];
                dst[di++] = (char)p[2];
                dst[di++] = (char)p[3];
            }
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
// DM ring management
// ----------------------------------------------------------------------------
// Lazy per-contact rings (see the DMRing block at the top of this file).
// Helpers here are kept narrow:
//   dm_ring_get_or_alloc()  — fetch or allocate a ring for a contact
//   dm_ring_append()        — append a message, advancing newest_idx and
//                              wrapping over the oldest entry when full
//   dm_ring_clear_unread()  — call when the user opens the conversation
//
// Thread model: all three are called only from the LVGL task. The DM
// receive callback (meck_dm_recv_cb) is invoked from
// meck_drain_pending_dms which runs in the LVGL ui_update_timer_cb, and
// the send-echo path runs in the LVGL on_send_clicked. No mutex needed.
// ============================================================================

static DMRing* dm_ring_get_or_alloc(int contact_idx) {
    if (contact_idx < 0 || contact_idx >= MAX_CONTACTS) return nullptr;
    if (g_dm_rings[contact_idx]) return g_dm_rings[contact_idx];

    DMRing* r = (DMRing*)heap_caps_calloc(1, sizeof(DMRing), MALLOC_CAP_SPIRAM);
    if (!r) {
        printf("dm_ring_get_or_alloc: PSRAM alloc DMRing failed for idx=%d\n", contact_idx);
        return nullptr;
    }
    r->messages = (DMMessage*)heap_caps_calloc(MECK_DM_PER_CONTACT,
                                               sizeof(DMMessage),
                                               MALLOC_CAP_SPIRAM);
    if (!r->messages) {
        printf("dm_ring_get_or_alloc: PSRAM alloc messages failed for idx=%d\n", contact_idx);
        free(r);
        return nullptr;
    }
    r->count      = 0;
    r->newest_idx = -1;
    r->unread     = 0;
    g_dm_rings[contact_idx] = r;
    return r;
}

// Append a message to the per-contact ring. newest_idx advances; oldest
// rolls off when the ring is full. Returns the slot index written, or
// -1 on failure.
static int dm_ring_append(int contact_idx, const DMMessage& msg) {
    DMRing* r = dm_ring_get_or_alloc(contact_idx);
    if (!r) return -1;
    int idx = (r->newest_idx + 1) % MECK_DM_PER_CONTACT;
    r->messages[idx] = msg;
    r->newest_idx = idx;
    if (r->count < MECK_DM_PER_CONTACT) r->count++;
    if (!msg.outgoing) r->unread++;
    return idx;
}

// Mark all messages in a contact's ring as read. Called when the user
// opens the conversation view for that contact.
static void dm_ring_clear_unread(int contact_idx) {
    if (contact_idx < 0 || contact_idx >= MAX_CONTACTS) return;
    DMRing* r = g_dm_rings[contact_idx];
    if (!r) return;
    r->unread = 0;
}

// Sum of unread across all per-contact rings. Used by the home screen
// MSG counter and by the Direct Messages pseudo-row in the channel
// picker. Linear scan over MAX_CONTACTS pointer slots — almost all
// nullptr in normal use, so cheap.
static int dm_total_unread() {
    int total = 0;
    for (int i = 0; i < MAX_CONTACTS; i++) {
        DMRing* r = g_dm_rings[i];
        if (r) total += r->unread;
    }
    return total;
}

// Timestamp of the newest message in a contact's ring. 0 if no ring or
// the ring is empty. Used to sort the DM inbox by most-recent-first.
static uint32_t dm_ring_newest_timestamp(int contact_idx) {
    if (contact_idx < 0 || contact_idx >= MAX_CONTACTS) return 0;
    DMRing* r = g_dm_rings[contact_idx];
    if (!r || r->count == 0 || r->newest_idx < 0) return 0;
    return r->messages[r->newest_idx].timestamp;
}

// Copy the ring contents into a caller-provided buffer in chronological
// order (oldest first). Returns the count copied (0 to MECK_DM_PER_CONTACT).
// Mirrors the shape of Meck::getMessages so rebuild_dm_bubbles can use
// the same iteration pattern as rebuild_message_bubbles.
static int dm_ring_copy_chronological(int contact_idx, DMMessage* out, int max_out) {
    if (contact_idx < 0 || contact_idx >= MAX_CONTACTS) return 0;
    DMRing* r = g_dm_rings[contact_idx];
    if (!r || r->count == 0) return 0;
    int count = (r->count < max_out) ? r->count : max_out;
    int start = (r->newest_idx - count + 1 + MECK_DM_PER_CONTACT) % MECK_DM_PER_CONTACT;
    for (int i = 0; i < count; i++) {
        out[i] = r->messages[(start + i) % MECK_DM_PER_CONTACT];
    }
    return count;
}

// Locate the outgoing message in a ring by its expected_ack handle and
// flip its in-RAM acked state, then rewrite the persisted record on
// disk so the "Delivered" status survives a reboot. Returns true if a
// matching slot was found and updated.
//
// Called from rebuild_dm_bubbles when it detects a RAM→disk drift on
// an outgoing bubble (m.acked=true while m.persisted_acked=false).
// Operates on the live ring rather than a copy so the persisted_acked
// shadow update sticks.
//
// Idempotent — once persisted_acked is true, subsequent calls are
// no-ops. So an over-eager auto-refresh tick that calls this twice
// causes one SD write, not two.
static bool dm_ring_persist_ack_if_needed(int contact_idx,
                                           uint32_t expected_ack) {
    if (contact_idx < 0 || contact_idx >= MAX_CONTACTS) return false;
    if (expected_ack == 0) return false;
    DMRing* r = g_dm_rings[contact_idx];
    if (!r || r->count == 0) return false;

    // Walk all slots looking for the matching outgoing message. Could
    // optimise by knowing the typical match is near newest_idx, but
    // the ring is small (20 entries) so linear is fine.
    for (int i = 0; i < r->count; i++) {
        int slot = (r->newest_idx - i + MECK_DM_PER_CONTACT) % MECK_DM_PER_CONTACT;
        DMMessage& m = r->messages[slot];
        if (!m.outgoing) continue;
        if (m.expected_ack != expected_ack) continue;
        if (m.persisted_acked) return true;  // already done
        if (m.file_offset == 0) {
            // Not persisted to disk (initial save failed). Still flip
            // RAM state to match — the UI will render Delivered — but
            // there's nothing on disk to rewrite.
            m.persisted_acked = true;
            return true;
        }

        // Rebuild the on-disk record from the in-RAM message + the
        // peer hash from the contact (the ring entry doesn't carry
        // dm_peer_hash itself, so we have to look it up by index).
        Meck* mesh = meck_get_instance();
        if (!mesh) return false;
        ContactInfo ci;
        if (!mesh->getContactByIdx((uint32_t)contact_idx, ci)) {
            printf("dm_ring_persist_ack_if_needed: contact[%d] vanished\n",
                   contact_idx);
            return false;
        }

        P4MsgFileRecord rec = {};
        rec.timestamp   = m.timestamp;
        memcpy(&rec.dm_peer_hash, ci.id.pub_key, sizeof(rec.dm_peer_hash));
        rec.channel_idx = 0xFF;
        rec.path_len    = m.path_len;
        rec.snr_x4      = m.snr_x4;
        rec.flags       = P4_DM_FLAG_VALID | P4_DM_FLAG_OUTGOING | P4_DM_FLAG_ACKED;
        rec.heard_count = 0;
        memset(rec.path, 0, sizeof(rec.path));
        strncpy(rec.text, m.text, sizeof(rec.text) - 1);
        rec.text[sizeof(rec.text) - 1] = '\0';

        if (mesh->rewriteDMRecord(m.file_offset, rec)) {
            m.persisted_acked = true;
            printf("dm_ring_persist_ack_if_needed: rewrote DM offset=%u as acked\n",
                   (unsigned)m.file_offset);
            return true;
        }
        printf("dm_ring_persist_ack_if_needed: rewrite failed for offset=%u\n",
               (unsigned)m.file_offset);
        return false;
    }
    return false;
}

// ============================================================================
// Post ring helpers (Piece B)
// ----------------------------------------------------------------------------
// Same shape as the dm_ring_* helpers, scoped per room contact. Storage
// is PSRAM (consistent with DMs). All three are called only from the
// LVGL task (post recv dispatch runs there, screen-entry runs there).
// ============================================================================

static PostRing* post_ring_get_or_alloc(int contact_idx) {
    if (contact_idx < 0 || contact_idx >= MAX_CONTACTS) return nullptr;
    if (g_post_rings[contact_idx]) return g_post_rings[contact_idx];

    PostRing* r = (PostRing*)heap_caps_calloc(1, sizeof(PostRing), MALLOC_CAP_SPIRAM);
    if (!r) {
        printf("post_ring_get_or_alloc: PSRAM alloc PostRing failed for idx=%d\n", contact_idx);
        return nullptr;
    }
    r->messages = (PostMessage*)heap_caps_calloc(MECK_POST_PER_ROOM,
                                                 sizeof(PostMessage),
                                                 MALLOC_CAP_SPIRAM);
    if (!r->messages) {
        printf("post_ring_get_or_alloc: PSRAM alloc messages failed for idx=%d\n", contact_idx);
        free(r);
        return nullptr;
    }
    r->count      = 0;
    r->newest_idx = -1;
    g_post_rings[contact_idx] = r;
    return r;
}

static int post_ring_append(int contact_idx, const PostMessage& msg) {
    PostRing* r = post_ring_get_or_alloc(contact_idx);
    if (!r) return -1;
    int idx = (r->newest_idx + 1) % MECK_POST_PER_ROOM;
    r->messages[idx] = msg;
    r->newest_idx = idx;
    if (r->count < MECK_POST_PER_ROOM) r->count++;
    return idx;
}

// Copy chronological (oldest first). Mirrors dm_ring_copy_chronological.
static int post_ring_copy_chronological(int contact_idx, PostMessage* out, int max_out) {
    if (contact_idx < 0 || contact_idx >= MAX_CONTACTS) return 0;
    PostRing* r = g_post_rings[contact_idx];
    if (!r || r->count == 0) return 0;
    int count = (r->count < max_out) ? r->count : max_out;
    int start = (r->newest_idx - count + 1 + MECK_POST_PER_ROOM) % MECK_POST_PER_ROOM;
    for (int i = 0; i < count; i++) {
        out[i] = r->messages[(start + i) % MECK_POST_PER_ROOM];
    }
    return count;
}

// Dedup probe: returns true if the ring already holds a post matching
// the given (timestamp, sender_prefix) tuple. Used to drop both
// in-session retransmits (server retries when its ACK to us times out)
// and cross-login re-pushes (sendLogin resets sync_since so the server
// pushes everything from scratch on each login). Also used by the
// startup loader to dedup the on-disk record list which may contain
// duplicates from sessions before this fix landed.
static bool post_ring_has_post(int contact_idx, uint32_t timestamp,
                                const uint8_t* sender_prefix) {
    if (contact_idx < 0 || contact_idx >= MAX_CONTACTS) return false;
    PostRing* r = g_post_rings[contact_idx];
    if (!r || r->count == 0) return false;
    for (int i = 0; i < r->count; i++) {
        if (r->messages[i].timestamp == timestamp &&
            memcmp(r->messages[i].sender_prefix, sender_prefix, 4) == 0) {
            return true;
        }
    }
    return false;
}

// Resolve a 4-byte sender_prefix to a displayable author label. Walks
// the contact list looking for a matching pub_key prefix; if found
// returns the contact name. Falls back to "Unknown <hex>" mirroring
// the MeshCore app convention (Image 4 in the room-server reference
// screenshots).
static void resolve_post_author(const uint8_t* sender_prefix, char* out, int out_len) {
    if (!out || out_len <= 0) return;
    out[0] = '\0';

    Meck* mesh = meck_get_instance();
    if (mesh) {
        int n = mesh->getNumContacts();
        ContactInfo ci;
        for (int i = 0; i < n; i++) {
            if (!mesh->getContactByIdx((uint32_t)i, ci)) continue;
            if (memcmp(ci.id.pub_key, sender_prefix, 4) == 0) {
                strncpy(out, ci.name, out_len - 1);
                out[out_len - 1] = '\0';
                return;
            }
        }
    }
    snprintf(out, out_len, "Unknown <%02x%02x%02x%02x>",
             sender_prefix[0], sender_prefix[1], sender_prefix[2], sender_prefix[3]);
}

// Returns true if the 4-byte sender_prefix matches our own self_id
// pub_key prefix — i.e. this post is one we sent ourselves. Used by
// rebuild_room_bubbles to render our own posts right-aligned (cyan,
// no author label) mirroring DM "me" bubbles.
static bool post_is_outgoing(const uint8_t* sender_prefix) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return false;
    return memcmp(mesh->self_id.pub_key, sender_prefix, 4) == 0;
}

// ============================================================================
// Inline picture rendering
// ----------------------------------------------------------------------------
// Messages with body starting with "[PIC:" are treated as picture messages.
// Format: "[PIC:/sdcard/pictures/filename.jpg]" optionally followed by text.
// The image is rendered inside the bubble with the text (if any) below it.
//
// Requires in lv_conf.h:
//   #define LV_USE_FS_POSIX   1
//   #define LV_FS_POSIX_LETTER 'A'
//   #define LV_USE_TJPGD      1      (for JPEG decoding)
//   #define LV_USE_LIBPNG     1      (for PNG decoding, optional)
// ============================================================================

#define PIC_MARKER     "[PIC:"
#define PIC_MARKER_LEN 5
#define PIC_MAX_W      400   // max rendered width inside bubble
#define PIC_MAX_H      300   // max rendered height inside bubble

// Check if a message body is a picture message. Returns the file path
// (inside the marker) or NULL if not a picture message.
static const char* parse_picture_path(const char *body, char *path_out, size_t path_out_len) {
    if (!body || strncmp(body, PIC_MARKER, PIC_MARKER_LEN) != 0) return NULL;
    const char *start = body + PIC_MARKER_LEN;
    const char *end = strchr(start, ']');
    if (!end || end == start) return NULL;
    size_t len = (size_t)(end - start);
    if (len >= path_out_len) len = path_out_len - 1;
    memcpy(path_out, start, len);
    path_out[len] = '\0';
    return end + 1;  // points to any text after the "]"
}

// Create an image object inside a bubble for a picture message.
// Uses LVGL's file system driver (LV_FS_POSIX) to load from SD card.
// Returns true if the image was created, false if the file doesn't exist
// or the decoder can't handle it.
static bool render_picture_in_bubble(lv_obj_t *bubble, const char *sd_path,
                                     int max_w) {
    // Check file exists
    struct stat st;
    if (stat(sd_path, &st) != 0) {
        printf("PIC: file not found: %s\n", sd_path);
        return false;
    }

    // Build LVGL file path: "A:" prefix + SD path for LV_FS_POSIX driver.
    // If the FS driver uses a different letter, change PIC_FS_LETTER.
    char lv_path[128];
    snprintf(lv_path, sizeof(lv_path), "A:%s", sd_path);

    lv_obj_t *img = lv_image_create(bubble);
    lv_image_set_src(img, lv_path);

    // Scale to fit bubble width while preserving aspect ratio.
    // LVGL 9: set_inner_align + scale, or just set max size.
    if (max_w > PIC_MAX_W) max_w = PIC_MAX_W;
    lv_obj_set_width(img, max_w - 22);
    lv_image_set_inner_align(img, LV_IMAGE_ALIGN_CENTER);

    printf("PIC: rendered %s (%ld bytes) in bubble\n",
           sd_path, (long)st.st_size);
    return true;
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

    // Current UTC for send-timeout detection
    uint32_t now_utc = 0;
    if (mesh) {
        mesh::RTCClock* rtc = mesh->getRTCClock();
        if (rtc) now_utc = rtc->getCurrentTime();
    }

    if (n == 0) {
        lv_obj_t *empty = lv_label_create(obj_msg_scroll);
        lv_label_set_text(empty, "No messages yet.");
        lv_obj_set_style_text_color(empty, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(empty, &meck_montserrat_18, 0);
        lbl_messages_body = empty;  // keep dirty-flag gate happy
        g_chan_flip_deadline = 0;   // no messages -> no time-based flip pending
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
    // Soonest UTC second at which a still-"Sending" sent message will age past
    // the retry threshold and flip to "Failed". Written to g_chan_flip_deadline
    // after the loop so the UI timer can redraw exactly once when it is due.
    uint32_t next_flip_deadline = 0;

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

        // A sent, un-echoed post still inside its retry window is the only
        // thing that needs a future time-based redraw (to flip to "Failed").
        if (is_sent && msgs[i].heard_count == 0 && msgs[i].timestamp > 0 &&
            now_utc >= msgs[i].timestamp &&
            (now_utc - msgs[i].timestamp) < MECK_RETRY_CHANNEL_THRESHOLD_SEC) {
            uint32_t dl = msgs[i].timestamp + MECK_RETRY_CHANNEL_THRESHOLD_SEC;
            if (next_flip_deadline == 0 || dl < next_flip_deadline) {
                next_flip_deadline = dl;
            }
        }

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
            meck_set_font(lbl_sender, &meck_montserrat_22, 0);
        }

        // Message body: check for inline picture marker [PIC:path]
        char pic_path[128];
        const char *pic_caption = parse_picture_path(body, pic_path, sizeof(pic_path));
        if (pic_caption) {
            // Render the image inside the bubble
            render_picture_in_bubble(bubble, pic_path, bubble_max_w);
            // If there's caption text after the "]", show it below the image
            if (pic_caption[0] != '\0') {
                lv_obj_t *lbl_cap = lv_label_create(bubble);
                lv_label_set_text(lbl_cap, pic_caption);
                lv_obj_set_style_text_color(lbl_cap, lv_color_white(), 0);
                meck_set_font(lbl_cap, &meck_montserrat_18, 0);
                lv_label_set_long_mode(lbl_cap, LV_LABEL_LONG_WRAP);
                lv_obj_set_width(lbl_cap, bubble_max_w - 22);
            }
        } else {
            // Normal text message
            lv_obj_t *lbl_body = lv_label_create(bubble);
            lv_label_set_text(lbl_body, body);
            lv_obj_set_style_text_color(lbl_body, lv_color_white(), 0);
            meck_set_font(lbl_body, &meck_montserrat_22, 0);
            lv_label_set_long_mode(lbl_body, LV_LABEL_LONG_WRAP);
            lv_obj_set_width(lbl_body, bubble_max_w - 22);
        }

        // Timestamp footer in muted grey, optionally suffixed with hop /
        // heard-by metadata so the user can see at a glance how their
        // message travelled.
        //
        // For received bubbles we append a hop indicator:
        //   path_len == 0xFF   → "direct" (route_direct, no flood)
        //   path_len & 63 == 0 → "direct" (flood broadcast heard from the
        //                        sender with no relay in the path)
        //   else               → "N hops"
        //
        // For sent bubbles we append a heard-by indicator matching what
        // the MeshCore mobile app shows ("Heard N Repeats"):
        //   heard_count == 0   → "Sending…" (no echoes yet — either still
        //                        in flight, never heard back, or out of
        //                        repeater range)
        //   else               → "✓ Heard N Repeats"
        char timebuf[32];
        format_local_time(msgs[i].timestamp, timebuf, sizeof(timebuf));
        char meta[64];
        meta[0] = '\0';
        if (is_sent) {
            uint8_t hc = msgs[i].heard_count;
            if (hc == 0) {
                if (now_utc >= msgs[i].timestamp &&
                    msgs[i].timestamp > 0 &&
                    (now_utc - msgs[i].timestamp) >= MECK_RETRY_CHANNEL_THRESHOLD_SEC) {
                    snprintf(meta, sizeof(meta), LV_SYMBOL_CLOSE " Failed");
                } else {
                    snprintf(meta, sizeof(meta), "Sending...");
                }
            } else {
                snprintf(meta, sizeof(meta), LV_SYMBOL_OK " Heard %u Repeat%s",
                         (unsigned)hc, hc == 1 ? "" : "s");
            }
        } else {
            uint8_t pl = msgs[i].path_len;
            if (pl == 0xFF || (pl & 63) == 0) {
                snprintf(meta, sizeof(meta), "direct");
            } else {
                unsigned hops = pl & 63;
                snprintf(meta, sizeof(meta), "%u hop%s",
                         hops, hops == 1 ? "" : "s");
            }
        }
        char footer[96];
        if (timebuf[0] && meta[0]) {
            snprintf(footer, sizeof(footer), "%s  ·  %s", timebuf, meta);
        } else if (timebuf[0]) {
            snprintf(footer, sizeof(footer), "%s", timebuf);
        } else {
            snprintf(footer, sizeof(footer), "%s", meta);
        }
        if (footer[0]) {
            lv_obj_t *lbl_time = lv_label_create(bubble);
            lv_label_set_text(lbl_time, footer);
            lv_obj_set_style_text_color(lbl_time, lv_color_hex(0xF5F5DC), 0);
            meck_set_font(lbl_time, &meck_montserrat_14, 0);
            // Right-justify on sent bubbles to mirror chat-app convention.
            lv_obj_set_style_text_align(lbl_time,
                is_sent ? LV_TEXT_ALIGN_RIGHT : LV_TEXT_ALIGN_LEFT, 0);
        }

        // Long-press context: attach to ALL bubbles (sent and received).
        // For sent bubbles, enables retry on failure. For all bubbles,
        // enables path view showing repeater chain / echo sources.
        {
            BubbleRetryCtx *ctx = new BubbleRetryCtx();
            ctx->is_dm        = false;
            ctx->is_outgoing  = is_sent;
            ctx->target_idx   = (int)ch_idx;
            ctx->timestamp    = msgs[i].timestamp;
            ctx->expected_ack = 0;
            ctx->path_len     = msgs[i].path_len;
            ctx->heard_count  = msgs[i].heard_count;
            // Path hashes (incoming)
            ctx->msg_path_hash_size  = msgs[i].msg_path_hash_size;
            ctx->msg_path_hash_count = msgs[i].msg_path_hash_count;
            memcpy(ctx->msg_path_hashes, msgs[i].msg_path_hashes,
                   sizeof(ctx->msg_path_hashes));
            // Echo hashes (outgoing)
            ctx->echo_hash_size  = msgs[i].echo_hash_size;
            ctx->echo_hash_count = msgs[i].echo_hash_count;
            memcpy(ctx->echo_hashes, msgs[i].echo_hashes,
                   sizeof(ctx->echo_hashes));
            if (is_sent) {
                strncpy(ctx->body, body, sizeof(ctx->body) - 1);
                ctx->body[sizeof(ctx->body) - 1] = '\0';
            } else {
                ctx->body[0] = '\0';
                strncpy(ctx->sender, trim_sender, sizeof(ctx->sender) - 1);
                ctx->sender[sizeof(ctx->sender) - 1] = '\0';
            }
            lv_obj_set_user_data(bubble, ctx);
            lv_obj_add_event_cb(bubble, on_bubble_long_pressed,
                                LV_EVENT_LONG_PRESSED, NULL);
            lv_obj_add_event_cb(bubble, on_bubble_retry_ctx_delete,
                                LV_EVENT_DELETE, NULL);
        }
    }

    // Hold a non-NULL pointer in lbl_messages_body so the timer callback's
    // dirty-check still gates correctly. We point it at the last bubble
    // (which gets cleaned along with everything else on next rebuild).
    g_chan_flip_deadline = next_flip_deadline;
    lbl_messages_body = lv_obj_get_child(obj_msg_scroll, -1);

    lv_obj_scroll_to_y(obj_msg_scroll, LV_COORD_MAX, LV_ANIM_OFF);
}

// ============================================================================
// rebuild_dm_bubbles
// ----------------------------------------------------------------------------
// DM-mode counterpart of rebuild_message_bubbles. Source data is the
// per-contact ring g_dm_rings[contact_idx] rather than Meck::getMessages.
// Layout idioms (row container, bubble shape, hop indicator) match
// rebuild_message_bubbles for visual consistency between channel and DM
// conversation views. Differences:
//   - Only two bubble colours: one for "me" (right-aligned), one for "them"
//     (left-aligned). The hash-into-hue palette doesn't add useful
//     information when there are only two participants.
//   - No sender chip — the alignment alone tells you who sent it.
//   - ACK indicators on outgoing bubbles are placeholder this piece;
//     piece 5 reads msg.acked and shows "Pending" vs "Delivered".
// ============================================================================

static void rebuild_dm_bubbles(int contact_idx) {
    if (!obj_msg_scroll) return;

    lv_obj_clean(obj_msg_scroll);
    lbl_messages_body = NULL;

    lv_obj_set_layout(obj_msg_scroll, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(obj_msg_scroll, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(obj_msg_scroll, 6, 0);

    DMMessage msgs[MECK_DM_PER_CONTACT];
    int n = dm_ring_copy_chronological(contact_idx, msgs, MECK_DM_PER_CONTACT);

    if (n == 0) {
        lv_obj_t *empty = lv_label_create(obj_msg_scroll);
        lv_label_set_text(empty, "No DMs yet.");
        lv_obj_set_style_text_color(empty, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(empty, &meck_montserrat_18, 0);
        lbl_messages_body = empty;
        return;
    }

    int row_max_w    = (SCREEN_WIDTH - 20) - 20;
    int bubble_max_w = (int)((float)row_max_w * 0.78f);

    // Bubble colours. Outgoing matches the cyan send button so "me" is
    // recognisable at a glance; incoming uses a neutral teal that reads
    // distinct from the cyan but doesn't fight the home-screen palette.
    lv_color_t color_me   = lv_palette_main(LV_PALETTE_CYAN);
    lv_color_t color_them = lv_palette_main(LV_PALETTE_TEAL);

    for (int i = 0; i < n; i++) {
        const DMMessage& m = msgs[i];
        bool is_sent = m.outgoing;

        char clean_text[256];
        strip_unrenderable(m.text, clean_text, sizeof(clean_text));

        // Row: full-width, transparent, alignment by sender direction.
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

        // Bubble.
        lv_obj_t *bubble = lv_obj_create(row);
        lv_obj_set_width(bubble, LV_SIZE_CONTENT);
        lv_obj_set_height(bubble, LV_SIZE_CONTENT);
        lv_obj_set_style_max_width(bubble, bubble_max_w, 0);
        lv_obj_set_style_radius(bubble, 12, 0);
        lv_obj_set_style_pad_all(bubble, 10, 0);
        lv_obj_set_style_border_width(bubble, 0, 0);
        lv_obj_set_scroll_dir(bubble, LV_DIR_NONE);
        lv_obj_set_style_bg_color(bubble, is_sent ? color_me : color_them, 0);
        lv_obj_set_layout(bubble, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(bubble, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(bubble, 2, 0);

        // Body: check for inline picture marker [PIC:path]
        char pic_path[128];
        const char *pic_caption = parse_picture_path(clean_text, pic_path, sizeof(pic_path));
        if (pic_caption) {
            render_picture_in_bubble(bubble, pic_path, bubble_max_w);
            if (pic_caption[0] != '\0') {
                lv_obj_t *lbl_cap = lv_label_create(bubble);
                lv_label_set_text(lbl_cap, pic_caption);
                lv_obj_set_style_text_color(lbl_cap, lv_color_white(), 0);
                meck_set_font(lbl_cap, &meck_montserrat_18, 0);
                lv_label_set_long_mode(lbl_cap, LV_LABEL_LONG_WRAP);
                lv_obj_set_width(lbl_cap, bubble_max_w - 22);
            }
        } else {
            lv_obj_t *lbl_body = lv_label_create(bubble);
            lv_label_set_text(lbl_body, clean_text);
            lv_obj_set_style_text_color(lbl_body, lv_color_white(), 0);
            meck_set_font(lbl_body, &meck_montserrat_22, 0);
            lv_label_set_long_mode(lbl_body, LV_LABEL_LONG_WRAP);
            lv_obj_set_width(lbl_body, bubble_max_w - 22);
        }

        // Footer: timestamp + hop info (received) or status (sent).
        char timebuf[32];
        format_local_time(m.timestamp, timebuf, sizeof(timebuf));
        char meta[64];
        meta[0] = '\0';
        if (is_sent) {
            // Real ACK status now wired (piece 5 half A). Query
            // Meck::lookupDMAckStatus with the expected_ack we got
            // back from the send. Three cases:
            //   1. expected_ack == 0 — the send completion callback
            //      hasn't landed yet, so the bubble is brand new.
            //      Show "Sending..." optimistically.
            //   2. lookupDMAckStatus returns false — the ack table
            //      entry has rolled out. We can't tell if the ACK
            //      landed or not, so leave a neutral "Sent" label.
            //   3. lookupDMAckStatus returns true:
            //      - acked → "✓ Delivered"
            //      - else if (now - sent) < timeout → "Sending..."
            //      - else → "Failed"
            Meck* mesh_for_ack = meck_get_instance();
            if (m.expected_ack == 0) {
                // Two ways to land here:
                //   (a) Brand new send before meck_dm_sent_dispatch
                //       has populated expected_ack — render "Sending..."
                //   (b) Outgoing DM loaded from disk on boot. Live ACK
                //       table is empty across reboot, so expected_ack
                //       stays 0. Render acked-state from the persisted
                //       flag: "Delivered" if it was acked before
                //       reboot, otherwise "Sent" (final state unknown).
                if (m.from_disk) {
                    snprintf(meta, sizeof(meta),
                             m.acked ? LV_SYMBOL_OK " Delivered" : "Sent");
                } else {
                    snprintf(meta, sizeof(meta), "Sending...");
                }
            } else if (!mesh_for_ack) {
                snprintf(meta, sizeof(meta), "Sent");
            } else {
                bool          acked      = false;
                unsigned long sent_ms    = 0;
                uint32_t      timeout_ms = 0;
                if (mesh_for_ack->lookupDMAckStatus(m.expected_ack, acked,
                                                    sent_ms, timeout_ms)) {
                    if (acked) {
                        snprintf(meta, sizeof(meta), LV_SYMBOL_OK " Delivered");
                        // ACK just arrived (or arrived earlier and we're
                        // re-rendering). If the on-disk record still
                        // shows the message as unacked, rewrite it now
                        // so the Delivered status persists across reboot.
                        // The helper is idempotent and gates on
                        // m.persisted_acked, so this is one SD write per
                        // ACK, not per render tick.
                        if (!m.persisted_acked && m.file_offset != 0) {
                            dm_ring_persist_ack_if_needed(contact_idx,
                                                          m.expected_ack);
                        }
                    } else {
                        unsigned long now = (unsigned long)esp_log_timestamp();
                        if (now - sent_ms < timeout_ms) {
                            snprintf(meta, sizeof(meta), "Sending...");
                        } else {
                            snprintf(meta, sizeof(meta), "Failed");
                        }
                    }
                } else {
                    // Rolled out of the ACK table — final state unknown.
                    snprintf(meta, sizeof(meta), "Sent");
                }
            }
        } else {
            uint8_t pl = m.path_len;
            if (pl == 0xFF || (pl & 63) == 0) {
                snprintf(meta, sizeof(meta), "direct");
            } else {
                unsigned hops = pl & 63;
                snprintf(meta, sizeof(meta), "%u hop%s",
                         hops, hops == 1 ? "" : "s");
            }
        }
        char footer[96];
        if (timebuf[0] && meta[0]) {
            snprintf(footer, sizeof(footer), "%s  ·  %s", timebuf, meta);
        } else if (timebuf[0]) {
            snprintf(footer, sizeof(footer), "%s", timebuf);
        } else {
            snprintf(footer, sizeof(footer), "%s", meta);
        }
        if (footer[0]) {
            lv_obj_t *lbl_time = lv_label_create(bubble);
            lv_label_set_text(lbl_time, footer);
            lv_obj_set_style_text_color(lbl_time, lv_color_hex(0xF5F5DC), 0);
            meck_set_font(lbl_time, &meck_montserrat_14, 0);
            lv_obj_set_style_text_align(lbl_time,
                is_sent ? LV_TEXT_ALIGN_RIGHT : LV_TEXT_ALIGN_LEFT, 0);
        }

        // Retry-send: attach long-press handler + per-bubble retry
        // context on every sent DM bubble. Eligibility (Failed status)
        // is re-checked at press time via lookupDMAckStatus.
        if (is_sent) {
            BubbleRetryCtx *ctx = new BubbleRetryCtx();
            ctx->is_dm        = true;
            ctx->target_idx   = contact_idx;
            ctx->timestamp    = m.timestamp;
            ctx->expected_ack = m.expected_ack;
            strncpy(ctx->body, clean_text, sizeof(ctx->body) - 1);
            ctx->body[sizeof(ctx->body) - 1] = '\0';
            lv_obj_set_user_data(bubble, ctx);
            lv_obj_add_event_cb(bubble, on_bubble_long_pressed,
                                LV_EVENT_LONG_PRESSED, NULL);
            lv_obj_add_event_cb(bubble, on_bubble_retry_ctx_delete,
                                LV_EVENT_DELETE, NULL);
        }
    }

    lbl_messages_body = lv_obj_get_child(obj_msg_scroll, -1);
    lv_obj_scroll_to_y(obj_msg_scroll, LV_COORD_MAX, LV_ANIM_OFF);
}

// ============================================================================
// rebuild_room_bubbles (Piece B)
// ----------------------------------------------------------------------------
// Renders the per-room post ring as bubbles inside obj_room_scroll.
// Bubble-style mirroring rebuild_dm_bubbles; differences:
//   - All posts are "incoming" (received from the room) — bubbles
//     left-align. The composer (Piece C) will introduce outgoing
//     bubbles right-aligned.
//   - Each bubble has an author chip above showing the resolved
//     sender name (or "Unknown <hex>" fallback).
//   - No ACK status footer — only timestamp + hops/direct.
// ============================================================================

static void rebuild_room_bubbles(int contact_idx) {
    if (!obj_room_scroll) return;

    lv_obj_clean(obj_room_scroll);

    lv_obj_set_layout(obj_room_scroll, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(obj_room_scroll, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(obj_room_scroll, 6, 0);

    PostMessage msgs[MECK_POST_PER_ROOM];
    int n = post_ring_copy_chronological(contact_idx, msgs, MECK_POST_PER_ROOM);

    if (n == 0) {
        lv_obj_t *empty = lv_label_create(obj_room_scroll);
        lv_label_set_text(empty, "No posts yet.");
        lv_obj_set_style_text_color(empty, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(empty, &meck_montserrat_18, 0);
        return;
    }

    int row_max_w    = (SCREEN_WIDTH - 20) - 20;
    int bubble_max_w = (int)((float)row_max_w * 0.78f);

    // Incoming posts (from other authors): left-aligned, teal, author
    // label above. Outgoing (our own, post_is_outgoing matches self_id
    // prefix): right-aligned, cyan, no author label — same convention
    // as DM bubbles where alignment alone identifies the sender.
    lv_color_t color_me   = lv_palette_main(LV_PALETTE_CYAN);
    lv_color_t color_them = lv_palette_main(LV_PALETTE_TEAL);

    for (int i = 0; i < n; i++) {
        const PostMessage& m = msgs[i];
        bool is_outgoing = post_is_outgoing(m.sender_prefix);

        char clean_text[256];
        strip_unrenderable(m.text, clean_text, sizeof(clean_text));

        // Row: full-width, transparent, alignment by sender direction.
        lv_obj_t *row = lv_obj_create(obj_room_scroll);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_scroll_dir(row, LV_DIR_NONE);
        lv_obj_set_layout(row, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(row,
            is_outgoing ? LV_FLEX_ALIGN_END : LV_FLEX_ALIGN_START,
            LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

        // Author label — above the bubble, incoming only.
        if (!is_outgoing) {
            char author[64];
            resolve_post_author(m.sender_prefix, author, sizeof(author));
            lv_obj_t *lbl_author = lv_label_create(row);
            lv_label_set_text(lbl_author, author);
            lv_obj_set_style_text_color(lbl_author,
                                        lv_palette_main(LV_PALETTE_GREY), 0);
            meck_set_font(lbl_author, &meck_montserrat_14, 0);
        }

        // Bubble.
        lv_obj_t *bubble = lv_obj_create(row);
        lv_obj_set_width(bubble, LV_SIZE_CONTENT);
        lv_obj_set_height(bubble, LV_SIZE_CONTENT);
        lv_obj_set_style_max_width(bubble, bubble_max_w, 0);
        lv_obj_set_style_radius(bubble, 12, 0);
        lv_obj_set_style_pad_all(bubble, 10, 0);
        lv_obj_set_style_border_width(bubble, 0, 0);
        lv_obj_set_scroll_dir(bubble, LV_DIR_NONE);
        lv_obj_set_style_bg_color(bubble, is_outgoing ? color_me : color_them, 0);
        lv_obj_set_layout(bubble, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(bubble, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(bubble, 2, 0);

        // Body: check for inline picture marker [PIC:path]
        char pic_path_r[128];
        const char *pic_caption_r = parse_picture_path(clean_text, pic_path_r, sizeof(pic_path_r));
        if (pic_caption_r) {
            render_picture_in_bubble(bubble, pic_path_r, bubble_max_w);
            if (pic_caption_r[0] != '\0') {
                lv_obj_t *lbl_cap = lv_label_create(bubble);
                lv_label_set_text(lbl_cap, pic_caption_r);
                lv_obj_set_style_text_color(lbl_cap, lv_color_white(), 0);
                meck_set_font(lbl_cap, &meck_montserrat_18, 0);
                lv_label_set_long_mode(lbl_cap, LV_LABEL_LONG_WRAP);
                lv_obj_set_width(lbl_cap, bubble_max_w - 22);
            }
        } else {
            lv_obj_t *lbl_body = lv_label_create(bubble);
            lv_label_set_text(lbl_body, clean_text);
            lv_obj_set_style_text_color(lbl_body, lv_color_white(), 0);
            meck_set_font(lbl_body, &meck_montserrat_22, 0);
            lv_label_set_long_mode(lbl_body, LV_LABEL_LONG_WRAP);
            lv_obj_set_width(lbl_body, bubble_max_w - 22);
        }

        // Footer: timestamp + hops/direct.
        char timebuf[32];
        format_local_time(m.timestamp, timebuf, sizeof(timebuf));
        char meta[64];
        meta[0] = '\0';
        if (m.path_len == 0xFF) {
            snprintf(meta, sizeof(meta), "direct");
        } else {
            unsigned hops = m.path_len & 63;
            snprintf(meta, sizeof(meta), "%u hop%s",
                     hops, hops == 1 ? "" : "s");
        }
        char footer[96];
        if (timebuf[0] && meta[0]) {
            snprintf(footer, sizeof(footer), "%s  ·  %s", timebuf, meta);
        } else if (timebuf[0]) {
            snprintf(footer, sizeof(footer), "%s", timebuf);
        } else {
            snprintf(footer, sizeof(footer), "%s", meta);
        }
        if (footer[0]) {
            lv_obj_t *lbl_time = lv_label_create(bubble);
            lv_label_set_text(lbl_time, footer);
            lv_obj_set_style_text_color(lbl_time, lv_color_hex(0xF5F5DC), 0);
            meck_set_font(lbl_time, &meck_montserrat_14, 0);
            lv_obj_set_style_text_align(lbl_time,
                is_outgoing ? LV_TEXT_ALIGN_RIGHT : LV_TEXT_ALIGN_LEFT, 0);
        }
    }

    lv_obj_scroll_to_y(obj_room_scroll, LV_COORD_MAX, LV_ANIM_OFF);
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
        "Noise Floor:   %d dBm\n"
        "RX Packets:    %lu",
        prefs->freq, prefs->bw, prefs->sf, prefs->cr,
        (int)prefs->tx_power_dbm,
        nf,
        (unsigned long)radio_driver.getPacketsRecv());
    lv_label_set_text(lbl_radio_detail, buf);
}

// ============================================================================
// Settings tap handlers
// ============================================================================

// Shared by the panel text editors. On open: hide the on-screen keyboard
// while the hardware keyboard is present (the kb->textarea binding made by
// the caller stays -- the hardware poll routes Enter/Esc through it), then
// focus the field so the poll has a target; sending LV_EVENT_FOCUSED (not
// just adding the state) starts the cursor blink, same rationale as
// type-to-compose. On close: drop the focus with the panel -- the poll does
// not check visibility, so a still-focused hidden field would keep taking
// keys. Plain t_display_p4 builds compile these to no-ops.
#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
static void meck_panel_edit_opened(lv_obj_t *ta, lv_obj_t *kb) {
    if (kb && meck_hw_keyboard_active())
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    if (ta) {
        lv_obj_add_state(ta, LV_STATE_FOCUSED);
        lv_obj_send_event(ta, LV_EVENT_FOCUSED, NULL);
    }
}
static void meck_panel_edit_closed(lv_obj_t *ta) {
    if (ta) {
        lv_obj_remove_state(ta, LV_STATE_FOCUSED);
        lv_obj_send_event(ta, LV_EVENT_DEFOCUSED, NULL);
    }
}
#else
static inline void meck_panel_edit_opened(lv_obj_t *ta, lv_obj_t *kb) { (void)ta; (void)kb; }
static inline void meck_panel_edit_closed(lv_obj_t *ta) { (void)ta; }
#endif

// Cross-module versions for UI modules built outside this file (the notes
// editor). Same semantics; the bool return reports whether the hardware
// keyboard is present so callers can reclaim the hidden keyboard's space.
extern "C" bool meck_ui_panel_edit_opened(lv_obj_t *ta, lv_obj_t *kb) {
    bool hw = meck_hw_keyboard_active();
    meck_panel_edit_opened(ta, kb);
    return hw;
}
extern "C" void meck_ui_panel_edit_closed(lv_obj_t *ta) {
    meck_panel_edit_closed(ta);
}

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
    meck_panel_edit_closed(ta_settings_name);
    if (obj_name_edit_panel) lv_obj_add_flag(obj_name_edit_panel, LV_OBJ_FLAG_HIDDEN);
    settings_update_labels();
}

static void on_settings_name_cancel(lv_event_t *e) {
    meck_panel_edit_closed(ta_settings_name);
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
        meck_panel_edit_opened(ta_settings_name, kb_settings);
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

// ---- Frequency text editor ----

static void on_settings_freq_tap(lv_event_t *e) {
    if (!obj_num_edit_panel || !ta_num_edit) return;
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    g_num_edit_field = 0;
    char buf[16];
    snprintf(buf, sizeof(buf), "%.3f", (double)prefs->freq);
    lv_textarea_set_text(ta_num_edit, buf);
    lv_obj_remove_flag(obj_num_edit_panel, LV_OBJ_FLAG_HIDDEN);
    if (kb_num_edit) lv_keyboard_set_textarea(kb_num_edit, ta_num_edit);
    meck_panel_edit_opened(ta_num_edit, kb_num_edit);
}

// ---- Spreading Factor text editor ----

static void on_settings_sf_tap(lv_event_t *e) {
    if (!obj_num_edit_panel || !ta_num_edit) return;
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    g_num_edit_field = 1;
    char buf[4];
    snprintf(buf, sizeof(buf), "%d", (int)prefs->sf);
    lv_textarea_set_text(ta_num_edit, buf);
    lv_obj_remove_flag(obj_num_edit_panel, LV_OBJ_FLAG_HIDDEN);
    if (kb_num_edit) lv_keyboard_set_textarea(kb_num_edit, ta_num_edit);
    meck_panel_edit_opened(ta_num_edit, kb_num_edit);
}

static void on_num_edit_save(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!ta_num_edit || !mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    const char* text = lv_textarea_get_text(ta_num_edit);
    if (!text || !text[0]) {
        meck_panel_edit_closed(ta_num_edit);
        if (obj_num_edit_panel) lv_obj_add_flag(obj_num_edit_panel, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    if (g_num_edit_field == 0) {
        // Frequency
        float f = strtof(text, nullptr);
        if (f >= 400.0f && f <= 2500.0f) {
            prefs->freq = f;
            mesh->getDataStore()->savePrefs(*prefs);
            radio_request_reconfig(prefs->freq, prefs->bw, prefs->sf, prefs->cr, prefs->tx_power_dbm);
            printf("Settings: Freq = %.3f MHz\n", (double)f);
            settings_update_labels();
            update_radio_detail_label();
        }
    } else if (g_num_edit_field == 1) {
        // Spreading Factor
        int sf = atoi(text);
        if (sf >= 5 && sf <= 12) {
            prefs->sf = (uint8_t)sf;
            mesh->getDataStore()->savePrefs(*prefs);
            radio_request_reconfig(prefs->freq, prefs->bw, prefs->sf, prefs->cr, prefs->tx_power_dbm);
            printf("Settings: SF = %d\n", sf);
            settings_update_labels();
            update_radio_detail_label();
        }
    } else if (g_num_edit_field == 2) {
        // Bandwidth
        float bw = strtof(text, nullptr);
        if (bw >= 7.8f && bw <= 500.0f) {
            prefs->bw = bw;
            mesh->getDataStore()->savePrefs(*prefs);
            radio_request_reconfig(prefs->freq, prefs->bw, prefs->sf, prefs->cr, prefs->tx_power_dbm);
            printf("Settings: BW = %.2f kHz\n", (double)bw);
            settings_update_labels();
            update_radio_detail_label();
        }
    }
    meck_panel_edit_closed(ta_num_edit);
    if (obj_num_edit_panel) lv_obj_add_flag(obj_num_edit_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_num_edit_cancel(lv_event_t *e) {
    meck_panel_edit_closed(ta_num_edit);
    if (obj_num_edit_panel) lv_obj_add_flag(obj_num_edit_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_num_edit_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY)       on_num_edit_save(NULL);
    else if (code == LV_EVENT_CANCEL) on_num_edit_cancel(NULL);
}

// ---- Bandwidth text editor ----

static void on_settings_bw_tap(lv_event_t *e) {
    if (!obj_num_edit_panel || !ta_num_edit) return;
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    g_num_edit_field = 2;
    char buf[16];
    snprintf(buf, sizeof(buf), "%.2f", (double)prefs->bw);
    lv_textarea_set_text(ta_num_edit, buf);
    lv_obj_remove_flag(obj_num_edit_panel, LV_OBJ_FLAG_HIDDEN);
    if (kb_num_edit) lv_keyboard_set_textarea(kb_num_edit, ta_num_edit);
    meck_panel_edit_opened(ta_num_edit, kb_num_edit);
}

// ---- Coding Rate: tap to cycle (5-8) ----

static void on_settings_cr_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    prefs->cr = (prefs->cr >= 8) ? 5 : prefs->cr + 1;
    mesh->getDataStore()->savePrefs(*prefs);

    radio_request_reconfig(prefs->freq, prefs->bw, prefs->sf, prefs->cr, prefs->tx_power_dbm);

    printf("Settings: CR = 4/%d\n", (int)prefs->cr);
    settings_update_labels();
    update_radio_detail_label();
}

// ---- Default Region text editor ----

static void on_settings_region_tap(lv_event_t *e) {
    if (!obj_region_edit_panel || !ta_region_edit) return;
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    lv_textarea_set_text(ta_region_edit, prefs->default_scope_name);
    lv_obj_remove_flag(obj_region_edit_panel, LV_OBJ_FLAG_HIDDEN);
    if (kb_region_edit) lv_keyboard_set_textarea(kb_region_edit, ta_region_edit);
    meck_panel_edit_opened(ta_region_edit, kb_region_edit);
}

static void on_region_edit_save(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!ta_region_edit || !mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    const char* text = lv_textarea_get_text(ta_region_edit);

    // Copy into prefs (empty = unscoped)
    if (text && text[0]) {
        strncpy(prefs->default_scope_name, text, sizeof(prefs->default_scope_name) - 1);
        prefs->default_scope_name[sizeof(prefs->default_scope_name) - 1] = '\0';
        // Derive the transport key via mesh helper
        mesh->deriveScopeKey(text, prefs->default_scope_key);
    } else {
        memset(prefs->default_scope_name, 0, sizeof(prefs->default_scope_name));
        memset(prefs->default_scope_key, 0, sizeof(prefs->default_scope_key));
    }
    mesh->getDataStore()->savePrefs(*prefs);
    printf("Settings: Default region = '%s'\n",
           prefs->default_scope_name[0] ? prefs->default_scope_name : "(unscoped)");
    settings_update_labels();
    meck_panel_edit_closed(ta_region_edit);
    if (obj_region_edit_panel) lv_obj_add_flag(obj_region_edit_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_region_edit_cancel(lv_event_t *e) {
    meck_panel_edit_closed(ta_region_edit);
    if (obj_region_edit_panel) lv_obj_add_flag(obj_region_edit_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_region_edit_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY)       on_region_edit_save(NULL);
    else if (code == LV_EVENT_CANCEL) on_region_edit_cancel(NULL);
}

// ============================================================================
// Position settings sub-screen handlers
// ============================================================================

static const char* position_mode_label(uint8_t mode) {
    switch (mode) {
        case 1:  return "Manual";
        case 2:  return "Auto-update (GPS)";
        default: return "Off";
    }
}

static void refresh_position_labels() {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    if (lbl_pos_lat_value) {
        if (prefs->position_lat_e7 != 0 || prefs->position_lon_e7 != 0) {
            char buf[24];
            snprintf(buf, sizeof(buf), "%.7f", (double)prefs->position_lat_e7 / 1e7);
            lv_label_set_text(lbl_pos_lat_value, buf);
        } else {
            lv_label_set_text(lbl_pos_lat_value, "not set");
        }
    }
    if (lbl_pos_lon_value) {
        if (prefs->position_lat_e7 != 0 || prefs->position_lon_e7 != 0) {
            char buf[24];
            snprintf(buf, sizeof(buf), "%.7f", (double)prefs->position_lon_e7 / 1e7);
            lv_label_set_text(lbl_pos_lon_value, buf);
        } else {
            lv_label_set_text(lbl_pos_lon_value, "not set");
        }
    }
    if (lbl_pos_mode_value) {
        lv_label_set_text(lbl_pos_mode_value, position_mode_label(prefs->position_mode));
    }
}

static void goto_settings_position(lv_event_t *e) {
    refresh_position_labels();
    if (scr_settings_position) lv_screen_load(scr_settings_position);
}

static void on_pos_edit_save(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!ta_pos_edit || !mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    const char* text = lv_textarea_get_text(ta_pos_edit);
    if (!text || !text[0]) {
        // Empty field — clear the position value
        if (g_pos_edit_field == 3) {
            prefs->position_lat_e7 = 0;
            mesh->getDataStore()->savePrefs(*prefs);
            printf("Settings: Lat cleared\n");
            refresh_position_labels();
        } else if (g_pos_edit_field == 4) {
            prefs->position_lon_e7 = 0;
            mesh->getDataStore()->savePrefs(*prefs);
            printf("Settings: Lon cleared\n");
            refresh_position_labels();
        }
        meck_panel_edit_closed(ta_pos_edit);
        if (obj_pos_edit_panel) lv_obj_add_flag(obj_pos_edit_panel, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    if (g_pos_edit_field == 3) {
        double lat = strtod(text, nullptr);
        if (lat >= -90.0 && lat <= 90.0) {
            prefs->position_lat_e7 = (int32_t)(lat * 1e7);
            mesh->getDataStore()->savePrefs(*prefs);
            printf("Settings: Lat = %.7f\n", lat);
            refresh_position_labels();
        }
    } else if (g_pos_edit_field == 4) {
        double lon = strtod(text, nullptr);
        if (lon >= -180.0 && lon <= 180.0) {
            prefs->position_lon_e7 = (int32_t)(lon * 1e7);
            mesh->getDataStore()->savePrefs(*prefs);
            printf("Settings: Lon = %.7f\n", lon);
            refresh_position_labels();
        }
    }
    meck_panel_edit_closed(ta_pos_edit);
    if (obj_pos_edit_panel) lv_obj_add_flag(obj_pos_edit_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_pos_edit_cancel(lv_event_t *e) {
    meck_panel_edit_closed(ta_pos_edit);
    if (obj_pos_edit_panel) lv_obj_add_flag(obj_pos_edit_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_pos_edit_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY)       on_pos_edit_save(NULL);
    else if (code == LV_EVENT_CANCEL) on_pos_edit_cancel(NULL);
}

static void on_pos_lat_tap(lv_event_t *e) {
    if (!obj_pos_edit_panel || !ta_pos_edit) return;
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    // Don't allow manual edit in auto-GPS mode
    if (prefs->position_mode == 2) return;
    g_pos_edit_field = 3;
    char buf[24];
    if (prefs->position_lat_e7 != 0) {
        snprintf(buf, sizeof(buf), "%.7f", (double)prefs->position_lat_e7 / 1e7);
    } else {
        buf[0] = '\0';
    }
    lv_textarea_set_text(ta_pos_edit, buf);
    lv_obj_remove_flag(obj_pos_edit_panel, LV_OBJ_FLAG_HIDDEN);
    if (kb_pos_edit) lv_keyboard_set_textarea(kb_pos_edit, ta_pos_edit);
    meck_panel_edit_opened(ta_pos_edit, kb_pos_edit);
}

static void on_pos_lon_tap(lv_event_t *e) {
    if (!obj_pos_edit_panel || !ta_pos_edit) return;
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    if (prefs->position_mode == 2) return;
    g_pos_edit_field = 4;
    char buf[24];
    if (prefs->position_lon_e7 != 0) {
        snprintf(buf, sizeof(buf), "%.7f", (double)prefs->position_lon_e7 / 1e7);
    } else {
        buf[0] = '\0';
    }
    lv_textarea_set_text(ta_pos_edit, buf);
    lv_obj_remove_flag(obj_pos_edit_panel, LV_OBJ_FLAG_HIDDEN);
    if (kb_pos_edit) lv_keyboard_set_textarea(kb_pos_edit, ta_pos_edit);
    meck_panel_edit_opened(ta_pos_edit, kb_pos_edit);
}

static void on_pos_mode_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    prefs->position_mode = (prefs->position_mode + 1) % 3;
    mesh->getDataStore()->savePrefs(*prefs);
    printf("Settings: position_mode = %d (%s)\n",
           prefs->position_mode, position_mode_label(prefs->position_mode));
    refresh_position_labels();
}

// ============================================================================
// Attachment menu (+ button long-press)
// ============================================================================

static void attach_menu_close() {
    if (obj_attach_menu) {
        lv_obj_del(obj_attach_menu);
        obj_attach_menu = NULL;
    }
    if (obj_attach_picker) {
        lv_obj_del(obj_attach_picker);
        obj_attach_picker = NULL;
    }
}

static void on_attach_cancel(lv_event_t *e) {
    (void)e;
    attach_menu_close();
}

static void on_attach_share_pos(lv_event_t *e) {
    (void)e;
    attach_menu_close();
    Meck* mesh = meck_get_instance();
    if (!mesh || !ta_compose) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    if (prefs->position_lat_e7 == 0 && prefs->position_lon_e7 == 0) return;

    char pos[32];
    snprintf(pos, sizeof(pos), "%.6f,%.6f",
             (double)prefs->position_lat_e7 / 1e7,
             (double)prefs->position_lon_e7 / 1e7);
    lv_textarea_set_text(ta_compose, pos);
    printf("Attach: shared position %s\n", pos);
}

static void on_attach_pick_file(lv_event_t *e) {
    const char *path = (const char *)lv_event_get_user_data(e);
    attach_menu_close();
    if (!path) return;

    // Queue the picture send as chunked channel messages
    if (!g_in_dm_mode && g_active_channel < MAX_GROUP_CHANNELS) {
        meck_request_picture_send(g_active_channel, path);
        printf("Attach: sending %s on ch[%d]\n", path, g_active_channel);

        // Clear textarea — chunks are sent in the background, not via the
        // compose bar. A [PIC:path] message is injected into the channel
        // ring once all chunks have been transmitted.
        if (ta_compose) {
            lv_textarea_set_text(ta_compose, "");
        }
    } else {
        printf("Attach: picture send only supported on channels for now\n");
        if (ta_compose) {
            lv_textarea_set_text(ta_compose, "[Pictures: channels only]");
        }
    }
}

static void on_attach_long_press(lv_event_t *e) {
    (void)e;
    attach_menu_close();

    if (!scr_messages) return;
    lv_obj_t *active = lv_screen_active();
    if (active != scr_messages) return;

    Meck* mesh = meck_get_instance();
    P4NodePrefs* prefs = mesh ? mesh->getNodePrefs() : nullptr;
    bool has_pos = prefs && (prefs->position_lat_e7 != 0 || prefs->position_lon_e7 != 0);

    // Build compact popup menu
    int menu_h = 60;  // base height for cancel
    if (has_pos) menu_h += 60;
    else menu_h += 35;  // space for "no position" label

    obj_attach_menu = lv_obj_create(scr_messages);
    lv_obj_set_size(obj_attach_menu, SCREEN_WIDTH - 80, menu_h);
    lv_obj_align(obj_attach_menu, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(obj_attach_menu, lv_color_make(15, 15, 25), 0);
    lv_obj_set_style_bg_opa(obj_attach_menu, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(obj_attach_menu, 16, 0);
    lv_obj_set_style_border_width(obj_attach_menu, 1, 0);
    lv_obj_set_style_border_color(obj_attach_menu,
        lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_pad_all(obj_attach_menu, 10, 0);
    lv_obj_clear_flag(obj_attach_menu, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_update_layout(obj_attach_menu);

    int y = 0;

    if (has_pos) {
        lv_obj_t *btn_pos = lv_button_create(obj_attach_menu);
        lv_obj_set_size(btn_pos, lv_obj_get_width(obj_attach_menu) - 20, 50);
        lv_obj_set_pos(btn_pos, 0, y);
        lv_obj_set_style_bg_color(btn_pos, lv_palette_main(LV_PALETTE_ORANGE), 0);
        lv_obj_set_style_radius(btn_pos, 8, 0);
        lv_obj_t *pos_lbl = lv_label_create(btn_pos);
        lv_label_set_text(pos_lbl, "Share Position");
        lv_obj_set_style_text_color(pos_lbl, lv_color_black(), 0);
        meck_set_font(pos_lbl, &meck_montserrat_16, 0);
        lv_obj_center(pos_lbl);
        lv_obj_add_event_cb(btn_pos, on_attach_share_pos, LV_EVENT_CLICKED, NULL);
        y += 55;
    } else {
        lv_obj_t *no_pos = lv_label_create(obj_attach_menu);
        lv_label_set_text(no_pos, "No position set in Settings");
        lv_obj_set_style_text_color(no_pos, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(no_pos, &meck_montserrat_14, 0);
        lv_obj_set_pos(no_pos, 5, y);
        y += 30;
    }

    // Cancel button
    lv_obj_t *btn_cancel = lv_button_create(obj_attach_menu);
    lv_obj_set_size(btn_cancel, lv_obj_get_width(obj_attach_menu) - 20, 45);
    lv_obj_set_pos(btn_cancel, 0, y);
    lv_obj_set_style_bg_color(btn_cancel, lv_color_make(80, 30, 30), 0);
    lv_obj_set_style_radius(btn_cancel, 8, 0);
    lv_obj_t *cancel_lbl = lv_label_create(btn_cancel);
    lv_label_set_text(cancel_lbl, "Cancel");
    lv_obj_set_style_text_color(cancel_lbl, lv_color_white(), 0);
    meck_set_font(cancel_lbl, &meck_montserrat_14, 0);
    lv_obj_center(cancel_lbl);
    lv_obj_add_event_cb(btn_cancel, on_attach_cancel, LV_EVENT_CLICKED, NULL);

    // Picture transfer disabled for v0.3.8 — re-enable when chunked
    // channel message pipeline is stable.
    // (Picture file listing and send UI removed from this menu.)
}


// ============================================================================
// Channel share picker handlers
// ============================================================================

static void on_ch_share_picker_cancel(lv_event_t *e) {
    (void)e;
    if (obj_ch_share_picker_panel) {
        lv_obj_del(obj_ch_share_picker_panel);
        obj_ch_share_picker_panel = NULL;
        obj_ch_share_picker_scroll = NULL;
    }
}

// Channel-share status label (top layer so it survives the picker closing).
static void share_status_dismiss_cb(lv_timer_t *t) {
    (void)t;
    if (g_share_status_lbl) lv_obj_add_flag(g_share_status_lbl, LV_OBJ_FLAG_HIDDEN);
    if (g_share_dismiss_timer) { lv_timer_delete(g_share_dismiss_timer); g_share_dismiss_timer = NULL; }
}

static void share_status_show(const char* text) {
    if (!g_share_status_lbl) {
        g_share_status_lbl = lv_label_create(lv_layer_top());
        lv_obj_set_style_bg_color(g_share_status_lbl, lv_color_make(20, 30, 45), 0);
        lv_obj_set_style_bg_opa(g_share_status_lbl, LV_OPA_COVER, 0);
        lv_obj_set_style_text_color(g_share_status_lbl, lv_color_white(), 0);
        lv_obj_set_style_pad_all(g_share_status_lbl, 12, 0);
        lv_obj_set_style_radius(g_share_status_lbl, 8, 0);
        lv_obj_set_style_text_align(g_share_status_lbl, LV_TEXT_ALIGN_CENTER, 0);
        meck_set_font(g_share_status_lbl, &meck_montserrat_18, 0);
    }
    if (g_share_dismiss_timer) { lv_timer_delete(g_share_dismiss_timer); g_share_dismiss_timer = NULL; }
    lv_label_set_text(g_share_status_lbl, text);
    lv_obj_align(g_share_status_lbl, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_remove_flag(g_share_status_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(g_share_status_lbl);
}

// Terminal state (Sent!/Failed): show text, stop polling, auto-hide after 4s.
static void share_status_finish(const char* text) {
    share_status_show(text);
    g_share_active = false;
    g_share_dismiss_timer = lv_timer_create(share_status_dismiss_cb, 4000, NULL);
}

static void on_ch_share_picker_select(lv_event_t *e) {
    int contact_idx = (int)(intptr_t)lv_event_get_user_data(e);
    Meck *mesh = meck_get_instance();
    if (!mesh || g_share_channel_idx < 0) return;

    int ch = g_share_channel_idx;
    uint32_t ack = 0;
    bool ok = mesh->shareChannelViaDM(ch, contact_idx, &ack);
    on_ch_share_picker_cancel(NULL);

    if (ok && ack) {
        g_share_st_channel = ch;
        g_share_st_contact = contact_idx;
        g_share_st_ack     = ack;
        g_share_st_attempt = 1;
        g_share_active     = true;
        share_status_show("Share Channel Sending (Attempt 1/3)");
    } else {
        share_status_finish("Share failed to send");
    }
}

static void on_ch_detail_share_tap(lv_event_t *e) {
    (void)e;
    Meck *mesh = meck_get_instance();
    if (!mesh || g_detail_channel_idx < 0) return;
    if (!scr_channel_detail) return;

    g_share_channel_idx = g_detail_channel_idx;

    // Clean up any existing picker
    if (obj_ch_share_picker_panel) {
        lv_obj_del(obj_ch_share_picker_panel);
        obj_ch_share_picker_panel = NULL;
    }

    obj_ch_share_picker_panel = lv_obj_create(scr_channel_detail);
    lv_obj_set_size(obj_ch_share_picker_panel, SCREEN_WIDTH - 40, SCREEN_HEIGHT - 80);
    lv_obj_align(obj_ch_share_picker_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(obj_ch_share_picker_panel, lv_color_make(15, 15, 25), 0);
    lv_obj_set_style_bg_opa(obj_ch_share_picker_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(obj_ch_share_picker_panel, 16, 0);
    lv_obj_set_style_border_width(obj_ch_share_picker_panel, 1, 0);
    lv_obj_set_style_border_color(obj_ch_share_picker_panel,
        lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_pad_all(obj_ch_share_picker_panel, 10, 0);
    lv_obj_clear_flag(obj_ch_share_picker_panel, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t *title = lv_label_create(obj_ch_share_picker_panel);
    lv_label_set_text(title, "Share with contact:");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_18, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 5, 5);

    // Cancel button
    lv_obj_t *btn_cancel = lv_button_create(obj_ch_share_picker_panel);
    lv_obj_set_size(btn_cancel, 80, 40);
    lv_obj_align(btn_cancel, LV_ALIGN_TOP_RIGHT, -5, 0);
    lv_obj_set_style_bg_color(btn_cancel, lv_color_make(80, 30, 30), 0);
    lv_obj_set_style_radius(btn_cancel, 8, 0);
    lv_obj_t *cancel_lbl = lv_label_create(btn_cancel);
    lv_label_set_text(cancel_lbl, "Cancel");
    lv_obj_set_style_text_color(cancel_lbl, lv_color_white(), 0);
    meck_set_font(cancel_lbl, &meck_montserrat_14, 0);
    lv_obj_center(cancel_lbl);
    lv_obj_add_event_cb(btn_cancel, on_ch_share_picker_cancel, LV_EVENT_CLICKED, NULL);

    // Scrollable contact list
    obj_ch_share_picker_scroll = lv_obj_create(obj_ch_share_picker_panel);
    lv_obj_set_size(obj_ch_share_picker_scroll,
                    SCREEN_WIDTH - 60,
                    SCREEN_HEIGHT - 140);
    lv_obj_align(obj_ch_share_picker_scroll, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_set_style_bg_opa(obj_ch_share_picker_scroll, 0, 0);
    lv_obj_set_style_border_width(obj_ch_share_picker_scroll, 0, 0);
    lv_obj_set_style_pad_all(obj_ch_share_picker_scroll, 0, 0);
    lv_obj_set_scroll_dir(obj_ch_share_picker_scroll, LV_DIR_VER);

    // Populate with ADV_TYPE_CHAT contacts
    int num_contacts = mesh->getNumContacts();
    int y = 0;
    for (int ci = 0; ci < num_contacts; ci++) {
        ContactInfo contact;
        if (!mesh->getContactByIdx(ci, contact)) continue;
        if (contact.type != 1) continue;  // ADV_TYPE_CHAT only

        lv_obj_t *row = lv_button_create(obj_ch_share_picker_scroll);
        lv_obj_set_size(row, SCREEN_WIDTH - 70, 50);
        lv_obj_set_pos(row, 5, y);
        lv_obj_set_style_bg_color(row, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(row, 8, 0);
        lv_obj_add_event_cb(row, on_ch_share_picker_select, LV_EVENT_CLICKED,
                            (void*)(intptr_t)ci);

        lv_obj_t *name = lv_label_create(row);
        lv_label_set_text(name, contact.name);
        lv_obj_set_style_text_color(name, lv_color_white(), 0);
        meck_set_font(name, &meck_montserrat_16, 0);
        lv_obj_align(name, LV_ALIGN_LEFT_MID, 10, 0);

        y += 55;
    }
    if (y == 0) {
        lv_obj_t *empty = lv_label_create(obj_ch_share_picker_scroll);
        lv_label_set_text(empty, "No chat contacts available");
        lv_obj_set_style_text_color(empty, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(empty, &meck_montserrat_16, 0);
        lv_obj_align(empty, LV_ALIGN_TOP_LEFT, 10, 10);
    }
}

// ============================================================================
// Channels settings sub-screen handlers
// ============================================================================

static const char* notif_label(uint8_t val) {
    switch (val) {
        case 1:  return "Mentions";
        case 2:  return "None";
        default: return "All";
    }
}

static void goto_settings_channels(lv_event_t *e) {
    refresh_channels_settings_list();
    if (scr_settings_channels) lv_screen_load(scr_settings_channels);
}

static void on_ch_settings_row_tap(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    g_detail_channel_idx = idx;
    g_delete_confirm_until = 0;
    refresh_channel_detail_labels();
    if (scr_channel_detail) lv_screen_load(scr_channel_detail);
}

static void on_ch_settings_add_tap(lv_event_t *e) {
    if (obj_ch_settings_add_panel && ta_ch_settings_add) {
        lv_textarea_set_text(ta_ch_settings_add, "");
        lv_obj_remove_flag(obj_ch_settings_add_panel, LV_OBJ_FLAG_HIDDEN);
        if (kb_ch_settings_add) lv_keyboard_set_textarea(kb_ch_settings_add, ta_ch_settings_add);
        meck_panel_edit_opened(ta_ch_settings_add, kb_ch_settings_add);
    }
}

static void on_ch_settings_add_save(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!ta_ch_settings_add || !mesh) return;
    const char* text = lv_textarea_get_text(ta_ch_settings_add);
    if (!text || !text[0]) return;

    // '#' prefix = public (SHA-256 derived secret), no '#' = private (random secret)
    mesh->createChannel(text);
    printf("Channels (settings): added '%s'\n", text);
    meck_panel_edit_closed(ta_ch_settings_add);
    if (obj_ch_settings_add_panel) lv_obj_add_flag(obj_ch_settings_add_panel, LV_OBJ_FLAG_HIDDEN);
    refresh_channels_settings_list();
    // Also refresh the channel picker if it exists
    refresh_channel_picker();
}

static void on_ch_settings_add_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY)  on_ch_settings_add_save(NULL);
    else if (code == LV_EVENT_CANCEL) {
        meck_panel_edit_closed(ta_ch_settings_add);
        if (obj_ch_settings_add_panel)
            lv_obj_add_flag(obj_ch_settings_add_panel, LV_OBJ_FLAG_HIDDEN);
    }
}

// ---- Channel detail handlers ----

static void refresh_channel_detail_labels() {
    Meck* mesh = meck_get_instance();
    if (!mesh || g_detail_channel_idx < 0) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    ChannelDetails ch;
    if (!mesh->getChannel(g_detail_channel_idx, ch)) return;

    if (lbl_ch_detail_name) lv_label_set_text(lbl_ch_detail_name, ch.name);
    if (lbl_ch_detail_scope)
        lv_label_set_text(lbl_ch_detail_scope,
            ch.scope_name[0] ? ch.scope_name : "(device default)");
    if (lbl_ch_detail_notif)
        lv_label_set_text(lbl_ch_detail_notif,
            notif_label(prefs->channel_notif[g_detail_channel_idx]));
    if (lbl_ch_detail_delete)
        lv_label_set_text(lbl_ch_detail_delete, "Delete Channel");
    if (lbl_ch_detail_tone) {
        const char* tone = g_notif_sounds.getSoundForChannel(g_detail_channel_idx);
        lv_label_set_text(lbl_ch_detail_tone,
            (tone && tone[0]) ? tone : "None");
    }
}

static void on_ch_detail_scope_tap(lv_event_t *e) {
    if (!obj_ch_scope_edit_panel || !ta_ch_scope_edit) return;
    Meck* mesh = meck_get_instance();
    if (!mesh || g_detail_channel_idx < 0) return;
    ChannelDetails ch;
    if (!mesh->getChannel(g_detail_channel_idx, ch)) return;
    lv_textarea_set_text(ta_ch_scope_edit, ch.scope_name);
    lv_obj_remove_flag(obj_ch_scope_edit_panel, LV_OBJ_FLAG_HIDDEN);
    if (kb_ch_scope_edit) lv_keyboard_set_textarea(kb_ch_scope_edit, ta_ch_scope_edit);
    meck_panel_edit_opened(ta_ch_scope_edit, kb_ch_scope_edit);
}

static void on_ch_scope_edit_save(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!ta_ch_scope_edit || !mesh || g_detail_channel_idx < 0) return;
    ChannelDetails ch;
    if (!mesh->getChannel(g_detail_channel_idx, ch)) return;
    const char* text = lv_textarea_get_text(ta_ch_scope_edit);
    if (text) {
        strncpy(ch.scope_name, text, sizeof(ch.scope_name) - 1);
        ch.scope_name[sizeof(ch.scope_name) - 1] = '\0';
    } else {
        memset(ch.scope_name, 0, sizeof(ch.scope_name));
    }
    mesh->setChannel(g_detail_channel_idx, ch);
    mesh->saveChannels();
    printf("Channel %d scope set to '%s'\n", g_detail_channel_idx,
           ch.scope_name[0] ? ch.scope_name : "(device default)");
    refresh_channel_detail_labels();
    meck_panel_edit_closed(ta_ch_scope_edit);
    if (obj_ch_scope_edit_panel) lv_obj_add_flag(obj_ch_scope_edit_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_ch_scope_edit_cancel(lv_event_t *e) {
    meck_panel_edit_closed(ta_ch_scope_edit);
    if (obj_ch_scope_edit_panel) lv_obj_add_flag(obj_ch_scope_edit_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_ch_scope_edit_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY)       on_ch_scope_edit_save(NULL);
    else if (code == LV_EVENT_CANCEL) on_ch_scope_edit_cancel(NULL);
}

static void on_ch_detail_notif_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh || g_detail_channel_idx < 0) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    prefs->channel_notif[g_detail_channel_idx] =
        (prefs->channel_notif[g_detail_channel_idx] + 1) % 3;
    mesh->getDataStore()->savePrefs(*prefs);
    printf("Channel %d notif = %s\n", g_detail_channel_idx,
           notif_label(prefs->channel_notif[g_detail_channel_idx]));
    refresh_channel_detail_labels();
}

static void on_ch_detail_delete_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh || g_detail_channel_idx < 0) return;

    // Don't allow deleting the primary channel (index 0)
    if (g_detail_channel_idx == 0) return;

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (g_delete_confirm_until > 0 && now < g_delete_confirm_until) {
        // Confirmed. Route through Meck::deleteChannel, which compacts the
        // channel table, shifts the per-channel message rings and their
        // ch_<N>.bin files down, and shifts the notification prefs. The old
        // empty-slot write here left all of those behind, so the next
        // channel created in the hole inherited the deleted channel's
        // history and settings.
        int deleted_idx = g_detail_channel_idx;
        mesh->deleteChannel((uint8_t)deleted_idx);
        // UI-side state keyed by the same slot numbers shifts in step:
        // compose drafts, notification tones, and the active-channel index.
        for (int i = deleted_idx; i < MAX_GROUP_CHANNELS - 1; i++) {
            memcpy(g_channel_drafts[i], g_channel_drafts[i + 1],
                   sizeof(g_channel_drafts[0]));
        }
        g_channel_drafts[MAX_GROUP_CHANNELS - 1][0] = '\0';
        // Tone map: move each mapping down one slot via the accessors (their
        // bounds checks make out-of-range slots a no-op); only write on a
        // real difference, since every set persists the config.
        for (int i = deleted_idx; i < MAX_GROUP_CHANNELS - 1; i++) {
            const char* next = g_notif_sounds.getSoundForChannel((uint8_t)(i + 1));
            const char* cur  = g_notif_sounds.getSoundForChannel((uint8_t)i);
            if (strcmp(cur, next) != 0) {
                g_notif_sounds.setSoundForChannel((uint8_t)i,
                                                  next[0] ? next : nullptr);
            }
        }
        if (g_notif_sounds.hasSoundForChannel(MAX_GROUP_CHANNELS - 1))
            g_notif_sounds.clearSoundForChannel(MAX_GROUP_CHANNELS - 1);
        if (g_active_channel == (uint8_t)deleted_idx) {
            g_active_channel = 0;
        } else if (g_active_channel > (uint8_t)deleted_idx &&
                   g_active_channel < MAX_GROUP_CHANNELS) {
            g_active_channel--;
        }
        // Navigate back to channels list
        refresh_channels_settings_list();
        refresh_channel_picker();
        if (scr_settings_channels) lv_screen_load(scr_settings_channels);
    } else {
        // First tap — show confirmation prompt
        g_delete_confirm_until = now + 3000;
        if (lbl_ch_detail_delete)
            lv_label_set_text(lbl_ch_detail_delete, "Tap again to confirm");
    }
}

// ---- Notification Tone picker ----

static void on_ch_detail_tone_tap(lv_event_t *e) {
    if (!obj_tone_picker_panel) return;
    g_notif_sounds.scanSoundFiles();
    refresh_tone_picker_list();
    lv_obj_remove_flag(obj_tone_picker_panel, LV_OBJ_FLAG_HIDDEN);
}

// DM notification tone: reuses the shared tone picker with the DM slot
// (channel index 0xFF in NotifSounds). Opened from the DM inbox header.
static void on_dm_tone_tap(lv_event_t *e) {
    if (!obj_tone_picker_panel) return;
    g_detail_channel_idx = 0xFF;  // 0xFF = DM slot
    g_notif_sounds.scanSoundFiles();
    refresh_tone_picker_list();
    lv_obj_remove_flag(obj_tone_picker_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_tone_picker_cancel(lv_event_t *e) {
    if (obj_tone_picker_panel) lv_obj_add_flag(obj_tone_picker_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_tone_picker_select(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (g_detail_channel_idx < 0) return;

    if (idx == 0) {
        // "None" selected
        g_notif_sounds.clearSoundForChannel(g_detail_channel_idx);
    } else {
        int fileIdx = idx - 1;
        const auto& files = g_notif_sounds.getSoundFiles();
        if (fileIdx >= 0 && fileIdx < (int)files.size()) {
            g_notif_sounds.setSoundForChannel(g_detail_channel_idx,
                                               files[fileIdx].c_str());
            // Preview: play the selected tone
            char path[64];
            NotifSounds::buildTonePath(path, sizeof(path), files[fileIdx].c_str());
            meck_audio_codec_wake();
            meck_audio_play_file(path, 0);
            meck_audio_resume();
        }
    }

    // DM slot (0xFF) has no channel-detail label to refresh.
    if (g_detail_channel_idx != 0xFF) refresh_channel_detail_labels();
    if (obj_tone_picker_panel) lv_obj_add_flag(obj_tone_picker_panel, LV_OBJ_FLAG_HIDDEN);
}

static void refresh_tone_picker_list() {
    if (!obj_tone_picker_scroll) return;
    lv_obj_clean(obj_tone_picker_scroll);

    const auto& files = g_notif_sounds.getSoundFiles();
    int y = 5;

    // Row 0: "None (silent)"
    {
        lv_obj_t *btn = lv_button_create(obj_tone_picker_scroll);
        lv_obj_set_size(btn, SCREEN_WIDTH - 40, 55);
        lv_obj_set_pos(btn, 10, y);
        lv_obj_set_style_bg_color(btn, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, lv_color_make(50, 50, 60), 0);
        lv_obj_add_event_cb(btn, on_tone_picker_select, LV_EVENT_CLICKED,
                            (void*)(intptr_t)0);

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, "None (silent)");
        lv_obj_set_style_text_color(lbl, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(lbl, &meck_montserrat_18, 0);
        lv_obj_center(lbl);
        y += 65;
    }

    // One row per scanned tone file
    for (int i = 0; i < (int)files.size(); i++) {
        lv_obj_t *btn = lv_button_create(obj_tone_picker_scroll);
        lv_obj_set_size(btn, SCREEN_WIDTH - 40, 55);
        lv_obj_set_pos(btn, 10, y);
        lv_obj_set_style_bg_color(btn, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, lv_color_make(50, 50, 60), 0);
        lv_obj_add_event_cb(btn, on_tone_picker_select, LV_EVENT_CLICKED,
                            (void*)(intptr_t)(i + 1));

        // Highlight the currently selected tone
        const char* current = g_notif_sounds.getSoundForChannel(g_detail_channel_idx);
        if (current && current[0] && files[i] == current) {
            lv_obj_set_style_border_color(btn, lv_palette_main(LV_PALETTE_CYAN), 0);
        }

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, files[i].c_str());
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        meck_set_font(lbl, &meck_montserrat_18, 0);
        lv_obj_center(lbl);
        y += 65;
    }
}

// ---- Channels settings list rebuild ----

static void refresh_channels_settings_list() {
    Meck* mesh = meck_get_instance();
    if (!obj_ch_settings_scroll || !mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();

    lv_obj_clean(obj_ch_settings_scroll);

    int y = 5;
    for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
        ChannelDetails ch;
        if (!mesh->getChannel(i, ch) || ch.name[0] == '\0') continue;

        lv_obj_t *btn = lv_button_create(obj_ch_settings_scroll);
        lv_obj_set_size(btn, SCREEN_WIDTH - 40, 70);
        lv_obj_set_pos(btn, 10, y);
        lv_obj_set_style_bg_color(btn, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, lv_color_make(50, 50, 60), 0);
        lv_obj_add_event_cb(btn, on_ch_settings_row_tap, LV_EVENT_CLICKED,
                            (void*)(intptr_t)i);

        lv_obj_t *name_lbl = lv_label_create(btn);
        lv_label_set_text(name_lbl, ch.name);
        lv_obj_set_style_text_color(name_lbl, lv_color_white(), 0);
        meck_set_font(name_lbl, &meck_montserrat_18, 0);
        lv_obj_align(name_lbl, LV_ALIGN_LEFT_MID, 10, -10);

        // Scope + notification tag line
        char tag[64];
        const char* scope = ch.scope_name[0] ? ch.scope_name : "*";
        const char* notif = prefs ? notif_label(prefs->channel_notif[i]) : "All";
        snprintf(tag, sizeof(tag), "[%s]  Notif: %s", scope, notif);
        lv_obj_t *tag_lbl = lv_label_create(btn);
        lv_label_set_text(tag_lbl, tag);
        lv_obj_set_style_text_color(tag_lbl, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(tag_lbl, &meck_montserrat_14, 0);
        lv_obj_align(tag_lbl, LV_ALIGN_LEFT_MID, 10, 12);

        lv_obj_t *arrow = lv_label_create(btn);
        lv_label_set_text(arrow, LV_SYMBOL_RIGHT);
        lv_obj_set_style_text_color(arrow, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(arrow, &meck_montserrat_16, 0);
        lv_obj_align(arrow, LV_ALIGN_RIGHT_MID, -10, 0);

        y += 80;
    }

    // "Add Channel" button at the bottom
    lv_obj_t *add_btn = lv_button_create(obj_ch_settings_scroll);
    lv_obj_set_size(add_btn, SCREEN_WIDTH - 40, 55);
    lv_obj_set_pos(add_btn, 10, y);
    lv_obj_set_style_bg_color(add_btn, lv_color_make(25, 25, 35), 0);
    lv_obj_set_style_radius(add_btn, 10, 0);
    lv_obj_set_style_border_width(add_btn, 1, 0);
    lv_obj_set_style_border_color(add_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_add_event_cb(add_btn, on_ch_settings_add_tap, LV_EVENT_CLICKED, NULL);

    lv_obj_t *add_lbl = lv_label_create(add_btn);
    lv_label_set_text(add_lbl, LV_SYMBOL_PLUS " Add Channel (# = public)");
    lv_obj_set_style_text_color(add_lbl, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(add_lbl, &meck_montserrat_18, 0);
    lv_obj_center(add_lbl);
    y += 65;

    // Pending channel invites (received via DM)
    int invite_count = mesh->getPendingInviteCount();
    for (int pi = 0; pi < invite_count; pi++) {
        const PendingChannelInvite* inv = mesh->getPendingInvite(pi);
        if (!inv) continue;

        lv_obj_t *inv_btn = lv_button_create(obj_ch_settings_scroll);
        lv_obj_set_size(inv_btn, SCREEN_WIDTH - 40, 70);
        lv_obj_set_pos(inv_btn, 10, y);
        lv_obj_set_style_bg_color(inv_btn, lv_color_make(30, 30, 15), 0);
        lv_obj_set_style_radius(inv_btn, 10, 0);
        lv_obj_set_style_border_width(inv_btn, 1, 0);
        lv_obj_set_style_border_color(inv_btn, lv_palette_main(LV_PALETTE_YELLOW), 0);

        char inv_text[80];
        snprintf(inv_text, sizeof(inv_text), "Pending: %s", inv->name);
        lv_obj_t *inv_name_lbl = lv_label_create(inv_btn);
        lv_label_set_text(inv_name_lbl, inv_text);
        lv_obj_set_style_text_color(inv_name_lbl, lv_palette_main(LV_PALETTE_YELLOW), 0);
        meck_set_font(inv_name_lbl, &meck_montserrat_18, 0);
        lv_obj_align(inv_name_lbl, LV_ALIGN_LEFT_MID, 10, -10);

        char from_text[64];
        snprintf(from_text, sizeof(from_text), "from %s  (tap = accept)", inv->senderName);
        lv_obj_t *from_lbl = lv_label_create(inv_btn);
        lv_label_set_text(from_lbl, from_text);
        lv_obj_set_style_text_color(from_lbl, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(from_lbl, &meck_montserrat_14, 0);
        lv_obj_align(from_lbl, LV_ALIGN_LEFT_MID, 10, 12);

        // Tap to accept the invite
        lv_obj_add_event_cb(inv_btn, [](lv_event_t *ev) {
            int invite_idx = (int)(intptr_t)lv_event_get_user_data(ev);
            Meck *m = meck_get_instance();
            if (!m) return;
            const PendingChannelInvite* invitation = m->getPendingInvite(invite_idx);
            if (invitation) {
                m->createChannelFromInvite(invitation->name, invitation->secret);
                printf("Channels: accepted invite '%s'\n", invitation->name);
                m->removePendingInvite(invite_idx);
            }
            refresh_channels_settings_list();
            refresh_channel_picker();
        }, LV_EVENT_CLICKED, (void*)(intptr_t)pi);

        // Long-press to dismiss
        lv_obj_add_event_cb(inv_btn, [](lv_event_t *ev) {
            int invite_idx = (int)(intptr_t)lv_event_get_user_data(ev);
            Meck *m = meck_get_instance();
            if (!m) return;
            printf("Channels: dismissed pending invite %d\n", invite_idx);
            m->removePendingInvite(invite_idx);
            refresh_channels_settings_list();
        }, LV_EVENT_LONG_PRESSED, (void*)(intptr_t)pi);

        y += 80;
    }
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
        if (g_in_dm_mode && g_active_dm_contact_idx >= 0) {
            // DM path. Defer to meck_task to avoid SPI race with mesh.loop().
            meck_request_send_dm(g_active_dm_contact_idx, text);

            // Local echo into the per-contact ring so the bubble shows
            // immediately. expected_ack will be filled in async by
            // meck_dm_sent_dispatch once the mesh task has actually
            // transmitted.
            DMMessage echo = {};
            echo.timestamp = meck_clock_get_utc();
            echo.outgoing  = true;
            echo.path_len  = 0;
            echo.snr_x4    = 0;
            echo.expected_ack = 0;
            echo.acked     = false;
            echo.file_offset = 0;
            echo.persisted_acked = false;
            echo.from_disk   = false;
            strncpy(echo.text, text, sizeof(echo.text) - 1);
            echo.text[sizeof(echo.text) - 1] = '\0';

            // Persist immediately so the message survives a reboot even
            // if the ACK never arrives. flags: valid + outgoing, but
            // not acked yet — that bit will be set in place via
            // rewriteDMRecord when the ACK lands (handled by the
            // auto-refresh path in rebuild_dm_bubbles).
            //
            // dm_peer_hash uses the first 4 bytes of the recipient's
            // pub_key, matching what meck_dm_recv_dispatch writes for
            // incoming records. Load-time demux can therefore match
            // both directions of a conversation to the same contact.
            Meck* mesh_for_save = meck_get_instance();
            if (mesh_for_save) {
                ContactInfo ci;
                if (mesh_for_save->getContactByIdx(
                        (uint32_t)g_active_dm_contact_idx, ci)) {
                    P4MsgFileRecord rec = {};
                    rec.timestamp   = echo.timestamp;
                    memcpy(&rec.dm_peer_hash, ci.id.pub_key,
                           sizeof(rec.dm_peer_hash));
                    rec.channel_idx = 0xFF;
                    rec.path_len    = 0;
                    rec.snr_x4      = 0;
                    rec.flags       = P4_DM_FLAG_VALID | P4_DM_FLAG_OUTGOING;
                    rec.heard_count = 0;
                    memset(rec.path, 0, sizeof(rec.path));
                    strncpy(rec.text, text, sizeof(rec.text) - 1);
                    rec.text[sizeof(rec.text) - 1] = '\0';
                    uint32_t saved_offset = 0;
                    if (!mesh_for_save->saveDMRecord(rec, &saved_offset)) {
                        printf("on_send_clicked: SD save failed for outgoing DM\n");
                    }
                    echo.file_offset = saved_offset;
                } else {
                    printf("on_send_clicked: getContactByIdx(%d) failed, "
                           "outgoing DM not persisted\n",
                           g_active_dm_contact_idx);
                }
            }

            dm_ring_append(g_active_dm_contact_idx, echo);
            rebuild_dm_bubbles(g_active_dm_contact_idx);
        } else {
            // Channel path (existing behaviour).
            meck_request_send_text(g_active_channel, text);
            if (g_active_channel < MAX_GROUP_CHANNELS)
                g_channel_drafts[g_active_channel][0] = '\0';
        }
        lv_textarea_set_text(ta_compose, "");
    }
    if (kb_compose) lv_obj_add_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
}

// Recompute obj_msg_scroll's height from the current ta_compose height
// and keyboard visibility. Called whenever either of those changes so the
// message list always shrinks/grows to fill the available vertical space.
// Constants: 90 px top y of obj_msg_scroll (matches the taller back-button
// header) + 10 px bottom margin around ta_compose + 10 px gap between
// scroll and textarea = 110 px reserved outside of the textarea itself.
static void meck_relayout_compose(void) {
    if (!obj_msg_scroll || !ta_compose) return;
    int ta_h = lv_obj_get_height(ta_compose);
    bool kb_shown = kb_compose && !lv_obj_has_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
    int reserved = 110 + ta_h;
    if (kb_shown) reserved += MECK_KB_HEIGHT;
    lv_obj_set_height(obj_msg_scroll, SCREEN_HEIGHT - reserved);
}

static void on_compose_size_changed(lv_event_t *e) {
    // ta_compose is LV_SIZE_CONTENT — LVGL resizes it itself when the
    // text content changes, which fires this event. Forward to the
    // shared layout helper so the message list resizes in lockstep.
    // Also match the send button height to the textarea height so they
    // stay visually aligned as the textarea grows/shrinks.
    if (btn_send && ta_compose) {
        lv_obj_set_height(btn_send, lv_obj_get_height(ta_compose));
    }
    if (btn_attach && ta_compose) {
        lv_obj_set_height(btn_attach, lv_obj_get_height(ta_compose));
    }
    meck_relayout_compose();
}

static void on_compose_focused(lv_event_t *e) {
    if (kb_compose) {
        lv_keyboard_set_textarea(kb_compose, ta_compose);
        // With the hardware keyboard attached the on-screen keyboard stays
        // hidden and the composer keeps its resting position, so the message
        // list gets the full height. The textarea binding above still has to
        // happen: the hardware poll locates its destination through
        // lv_keyboard_get_textarea(kb).
        if (meck_hw_keyboard_active()) {
            meck_relayout_compose();
            return;
        }
        lv_obj_remove_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
        // Lift textarea and send button above the keyboard. Height is
        // dynamic (LV_SIZE_CONTENT) so we only set position here, then
        // let meck_relayout_compose recompute obj_msg_scroll based on
        // the textarea's current height. 15 = gap between keyboard top
        // and textarea bottom.
        if (ta_compose) lv_obj_align(ta_compose, LV_ALIGN_BOTTOM_LEFT, 75, -(15 + MECK_KB_HEIGHT));
        if (btn_send)   lv_obj_align(btn_send,   LV_ALIGN_BOTTOM_RIGHT, -10, -(15 + MECK_KB_HEIGHT));
        if (btn_attach)  lv_obj_align(btn_attach,  LV_ALIGN_BOTTOM_LEFT, 10, -(15 + MECK_KB_HEIGHT));
        meck_relayout_compose();
    }
}

static void on_compose_defocused(lv_event_t *e) {
    // Scrolling the emoji picker can pull focus off the textarea; don't hide
    // the composer keyboard while the picker is open.
    if (meck_emoji_picker_is_open()) return;
    if (kb_compose) {
        lv_obj_add_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
        if (ta_compose) lv_obj_align(ta_compose, LV_ALIGN_BOTTOM_LEFT, 75, -10);
        if (btn_send)   lv_obj_align(btn_send,   LV_ALIGN_BOTTOM_RIGHT, -10, -10);
        if (btn_attach)  lv_obj_align(btn_attach,  LV_ALIGN_BOTTOM_LEFT, 10, -10);
        meck_relayout_compose();
    }
}

static void on_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        on_send_clicked(NULL);
    } else if (code == LV_EVENT_CANCEL) {
        if (kb_compose) lv_obj_add_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
        if (ta_compose) lv_obj_align(ta_compose, LV_ALIGN_BOTTOM_LEFT, 75, -10);
        if (btn_send)   lv_obj_align(btn_send,   LV_ALIGN_BOTTOM_RIGHT, -10, -10);
        if (btn_attach)  lv_obj_align(btn_attach,  LV_ALIGN_BOTTOM_LEFT, 10, -10);
        meck_relayout_compose();
    }
}

// ============================================================================
// Room composer handlers (Piece C)
// ----------------------------------------------------------------------------
// Independent set of widgets from the DM/channel composer so a focused
// textarea on the room screen doesn't try to drive the messages-screen
// keyboard. Same layout idioms; the keyboard rises when ta_room_compose
// is focused and the bubble scroll shrinks to fit.
//
// Send path: meck_request_send_dm with the room contact_idx. The mesh
// task transmits a TXT_TYPE_PLAIN packet to the room; the room server
// (upstream MyMesh) calls addPost on receipt. Local echo is appended to
// the per-room PostRing immediately so the user sees their post; the
// server intentionally does not push the post back to the original
// author (MyMesh.cpp getUnsyncedCount excludes self), so no dedupe.
// ============================================================================

static void meck_relayout_room_compose(void) {
    if (!obj_room_scroll || !ta_room_compose) return;
    int ta_h = lv_obj_get_height(ta_room_compose);
    bool kb_shown = kb_room_compose &&
                    !lv_obj_has_flag(kb_room_compose, LV_OBJ_FLAG_HIDDEN);
    // 110 px top reserve (clock/battery header + title row) + composer
    // height + bottom margin. Keyboard adds MECK_KB_HEIGHT when shown.
    int reserved = 130 + ta_h;
    if (kb_shown) reserved += MECK_KB_HEIGHT;
    lv_obj_set_height(obj_room_scroll, SCREEN_HEIGHT - reserved);
}

static void on_room_compose_size_changed(lv_event_t *e) {
    if (btn_room_send && ta_room_compose) {
        lv_obj_set_height(btn_room_send, lv_obj_get_height(ta_room_compose));
    }
    meck_relayout_room_compose();
}

static void on_room_compose_focused(lv_event_t *e) {
    if (kb_room_compose) {
        lv_keyboard_set_textarea(kb_room_compose, ta_room_compose);
        // See on_compose_focused: hidden on-screen keyboard, textarea still
        // bound so hardware keystrokes can find it.
        if (meck_hw_keyboard_active()) {
            meck_relayout_room_compose();
            return;
        }
        lv_obj_remove_flag(kb_room_compose, LV_OBJ_FLAG_HIDDEN);
        if (ta_room_compose) lv_obj_align(ta_room_compose,
            LV_ALIGN_BOTTOM_LEFT, 10, -(15 + MECK_KB_HEIGHT));
        if (btn_room_send)   lv_obj_align(btn_room_send,
            LV_ALIGN_BOTTOM_RIGHT, -10, -(15 + MECK_KB_HEIGHT));
        meck_relayout_room_compose();
    }
}

static void on_room_compose_defocused(lv_event_t *e) {
    if (meck_emoji_picker_is_open()) return;
    if (kb_room_compose) {
        lv_obj_add_flag(kb_room_compose, LV_OBJ_FLAG_HIDDEN);
        if (ta_room_compose) lv_obj_align(ta_room_compose,
            LV_ALIGN_BOTTOM_LEFT, 10, -10);
        if (btn_room_send)   lv_obj_align(btn_room_send,
            LV_ALIGN_BOTTOM_RIGHT, -10, -10);
        meck_relayout_room_compose();
    }
}

static void on_room_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        on_room_send_clicked(NULL);
    } else if (code == LV_EVENT_CANCEL) {
        if (kb_room_compose) lv_obj_add_flag(kb_room_compose, LV_OBJ_FLAG_HIDDEN);
        if (ta_room_compose) lv_obj_align(ta_room_compose,
            LV_ALIGN_BOTTOM_LEFT, 10, -10);
        if (btn_room_send)   lv_obj_align(btn_room_send,
            LV_ALIGN_BOTTOM_RIGHT, -10, -10);
        meck_relayout_room_compose();
    }
}

static void on_room_send_clicked(lv_event_t *e) {
    if (!ta_room_compose) return;
    if (g_active_room_contact_idx < 0) return;

    const char *text = lv_textarea_get_text(ta_room_compose);
    if (!text || !text[0]) return;

    // Send via the existing DM bridge — the wire format is the same
    // TXT_TYPE_PLAIN packet to the contact's pub_key, and the room
    // server (upstream MyMesh::onPeerDataRecv) processes it as a new
    // post via addPost(). meck_dm_sent_dispatch will fire on completion
    // and harmlessly log "no ring" since we don't allocate a DMRing
    // for the room contact.
    meck_request_send_dm(g_active_room_contact_idx, text);

    // Local echo into the per-room PostRing so the bubble appears
    // immediately. sender_prefix = first 4 bytes of our own pub_key,
    // which post_is_outgoing matches at render time to right-align.
    Meck* mesh = meck_get_instance();
    if (mesh) {
        ContactInfo ci;
        if (mesh->getContactByIdx((uint32_t)g_active_room_contact_idx, ci)) {
            PostMessage pm = {};
            pm.timestamp = meck_clock_get_utc();
            memcpy(pm.sender_prefix, mesh->self_id.pub_key, 4);
            pm.path_len = 0xFF;
            pm.snr_x4 = 0;
            pm.file_offset = 0;
            pm.from_disk = false;
            strncpy(pm.text, text, sizeof(pm.text) - 1);
            pm.text[sizeof(pm.text) - 1] = '\0';

            // Persist immediately so the echoed post survives a
            // reboot. room_pub_key_prefix = first 4 bytes of room
            // contact pub_key (matches the loader's demux key).
            P4PostFileRecord rec = {};
            rec.timestamp = pm.timestamp;
            memcpy(rec.room_pub_key_prefix, ci.id.pub_key, 4);
            memcpy(rec.sender_prefix, mesh->self_id.pub_key, 4);
            rec.flags = P4_POST_FLAG_VALID;
            rec.path_len = pm.path_len;
            rec.snr_x4 = pm.snr_x4;
            rec.reserved = 0;
            strncpy(rec.text, text, sizeof(rec.text) - 1);
            rec.text[sizeof(rec.text) - 1] = '\0';
            uint32_t saved_offset = 0;
            if (!mesh->savePostRecord(rec, &saved_offset)) {
                printf("on_room_send_clicked: SD save failed for outgoing post\n");
            }
            pm.file_offset = saved_offset;

            post_ring_append(g_active_room_contact_idx, pm);
            rebuild_room_bubbles(g_active_room_contact_idx);
        }
    }

    lv_textarea_set_text(ta_room_compose, "");
    if (kb_room_compose) lv_obj_add_flag(kb_room_compose, LV_OBJ_FLAG_HIDDEN);
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

    // DM vs Admin button visibility — chat contacts get DM, repeaters
    // and room servers get Admin. Both buttons share the same slot
    // (right-aligned, 90 px below the top edge) so only one is visible
    // at a time. ADV_TYPE_CHAT=1, ADV_TYPE_REPEATER=2, ADV_TYPE_ROOM=3.
    bool show_admin = (ci.type == 2 || ci.type == 3);
    if (btn_contact_dm) {
        if (show_admin) lv_obj_add_flag(btn_contact_dm, LV_OBJ_FLAG_HIDDEN);
        else            lv_obj_clear_flag(btn_contact_dm, LV_OBJ_FLAG_HIDDEN);
    }
    if (btn_contact_admin) {
        if (show_admin) lv_obj_clear_flag(btn_contact_admin, LV_OBJ_FLAG_HIDDEN);
        else            lv_obj_add_flag(btn_contact_admin, LV_OBJ_FLAG_HIDDEN);
    }
    if (btn_contact_ping) {
        if (show_admin) lv_obj_clear_flag(btn_contact_ping, LV_OBJ_FLAG_HIDDEN);
        else            lv_obj_add_flag(btn_contact_ping, LV_OBJ_FLAG_HIDDEN);
    }
    // Clear any in-flight ping when switching contacts
    g_ping_in_flight = false;
    // Button label: "Admin" for repeaters, "Login" for room servers. The
    // post-login dispatch branches on contact type so room login lands on
    // scr_room_messages while repeater login lands on scr_admin_home.
    if (lbl_contact_admin_btn) {
        if (ci.type == 3) {
            lv_label_set_text(lbl_contact_admin_btn, LV_SYMBOL_SETTINGS " Login");
        } else {
            lv_label_set_text(lbl_contact_admin_btn, LV_SYMBOL_SETTINGS " Admin");
        }
    }

    if (scr_contact_detail) lv_screen_load(scr_contact_detail);
}

static void on_contact_fav_toggle(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh || g_selected_contact_idx < 0) return;
    if (!mesh->toggleContactFavourite(g_selected_contact_idx)) return;

    // Re-read the authoritative state from the contact table so the panel
    // reflects what's actually persisted, not a local mutation.
    ContactInfo ci;
    if (!mesh->getContactByIdx(g_selected_contact_idx, ci)) return;

    char buf[512];
    snprintf(buf, sizeof(buf),
        "Name:     %s\n\n"
        "Type:     %d\n"
        "Flags:    0x%02X %s",
        ci.name, ci.type, ci.flags, (ci.flags & 0x01) ? "[FAV]" : "");
    if (lbl_contact_detail_body) lv_label_set_text(lbl_contact_detail_body, buf);
}

// Long-press a row in the contacts list to toggle its favourite bit
// without going through the detail screen. Same persistence path as the
// detail-screen button — both call into mesh->toggleContactFavourite().
static void on_contact_long_press(lv_event_t *e) {
    intptr_t idx = (intptr_t)lv_event_get_user_data(e);
    Meck* mesh = meck_get_instance();
    if (!mesh || idx < 0) return;
    if (mesh->toggleContactFavourite((int)idx)) {
        // Rebuild the list so the row picks up the new colour/icon. If the
        // current filter is FAV, an unfavourited contact disappears from
        // view; that's the desired behaviour.
        refresh_contacts_list();
    }
}

// Long-press the red trash button on the contact detail screen to delete
// the currently-selected contact. Click is intentionally NOT bound — only
// long-press fires this — so a stray tap doesn't lose the row.
static void on_contact_delete(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh || g_selected_contact_idx < 0) return;
    if (mesh->deleteContactByIdx(g_selected_contact_idx)) {
        g_selected_contact_idx = -1;
        refresh_contacts_list();
        if (scr_contacts) lv_screen_load(scr_contacts);
    }
}

// Swipe left/right on the contacts screen cycles the type filter. Matches
// the home-screen swipe convention and gives a fast way to flip between,
// say, Repeaters and Chat without reaching for the chip bar.
static void on_contacts_screen_gesture(lv_event_t *e) {
    if (lv_event_get_code(e) != LV_EVENT_GESTURE) return;
    lv_dir_t d = lv_indev_get_gesture_dir(lv_indev_active());
    int dir;
    if      (d == LV_DIR_LEFT)  dir = +1;
    else if (d == LV_DIR_RIGHT) dir = -1;
    else                        return;            // ignore vertical
    int n = (int)CONTACT_FILTER_COUNT;
    g_contact_filter = (ContactFilter)(((int)g_contact_filter + dir + n) % n);
    refresh_contacts_list();
}

// Tap a chip in the filter bar to jump directly to that filter. The chip's
// user_data carries the ContactFilter value as an intptr_t.
static void on_filter_chip_tap(lv_event_t *e) {
    intptr_t f = (intptr_t)lv_event_get_user_data(e);
    if (f < 0 || f >= CONTACT_FILTER_COUNT) return;
    g_contact_filter = (ContactFilter)f;
    refresh_contacts_list();
}

static void goto_contacts_from_detail(lv_event_t *e) {
    if (scr_contacts) lv_screen_load(scr_contacts);
}

// ============================================================================
// Channel handlers (lifted from old:683-748)
// ============================================================================

static void on_ch_add_save(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!ta_ch_add || !mesh) return;
    const char* text = lv_textarea_get_text(ta_ch_add);
    if (!text || !text[0]) {
        // Empty confirm doubles as cancel: this modal has no back button,
        // so a bare Add previously left no touch exit. Matches the
        // empty-confirm-exits behaviour of the other panel editors.
        meck_panel_edit_closed(ta_ch_add);
        if (obj_ch_add_panel) lv_obj_add_flag(obj_ch_add_panel, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    // '#' prefix = public, no '#' = private
    mesh->createChannel(text);
    printf("Channels: added '%s'\n", text);
    meck_panel_edit_closed(ta_ch_add);
    if (obj_ch_add_panel) lv_obj_add_flag(obj_ch_add_panel, LV_OBJ_FLAG_HIDDEN);
    refresh_channel_picker();
}

static void on_ch_add_cancel(lv_event_t *e) {
    meck_panel_edit_closed(ta_ch_add);
    if (obj_ch_add_panel) lv_obj_add_flag(obj_ch_add_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_ch_add_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY)       on_ch_add_save(NULL);
    else if (code == LV_EVENT_CANCEL) on_ch_add_cancel(NULL);
}

static void on_ch_add_tap(lv_event_t *e) {
    if (obj_ch_add_panel && ta_ch_add) {
        lv_textarea_set_text(ta_ch_add, "");
        lv_obj_remove_flag(obj_ch_add_panel, LV_OBJ_FLAG_HIDDEN);
        if (kb_ch_add) lv_keyboard_set_textarea(kb_ch_add, ta_ch_add);
        meck_panel_edit_opened(ta_ch_add, kb_ch_add);
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

// Tap on the ">>" now-playing indicator: jump straight back to the player
// for whatever audio is currently loaded, without restarting it. Wired to
// every audio indicator (home tiles, host screens, and the file explorers).
static void on_now_playing_tap(lv_event_t *e) {
    (void)e;
    meck_audio_ui_show_player_current();
}

// Tap any battery label to cycle its display between percentage and voltage.
// Forcing battery_ticks to the period makes the next ui_update_timer_cb tick
// (<=500ms) repaint every mirrored label in the new mode.
static void on_battery_label_tap(lv_event_t *e) {
    (void)e;
    g_batt_show_voltage = !g_batt_show_voltage;
    battery_ticks = 600;
}

#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
// Blue "KBD" header: declares which battery the physical selector has on the
// rail. Live in both states -- it is the way back.
static void on_kbd_batt_header_tap(lv_event_t *e) {
    (void)e;
    g_p4batt_source = g_p4batt_source ? 0 : 1;
    save_p4batt_prefs_to_nvs();
    // The top-right battery % now follows the declared source; force its
    // next 500 ms tick so the toggle shows there immediately instead of at
    // the next 5-minute refresh (same mechanism as on_battery_label_tap
    // and the orientation rebuild).
    battery_ticks = 600;
}

// Capacity line: cycles the declared pack size. Inert (self-gated) while the
// internal cell is the declared source, per the grey-side-is-inert rule.
static void on_kbd_batt_cap_tap(lv_event_t *e) {
    (void)e;
    if (g_p4batt_source != 1) return;
    g_p4batt_cap_mah = (g_p4batt_cap_mah == 5000) ? 10000 : 5000;
    save_p4batt_prefs_to_nvs();
}
#endif // CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD

// Attach the clock + battery pair to a home tile and register them in the
// per-tile arrays so ui_update_timer_cb can drive them. Called once from
// each tile builder. The battery field is space-padded to 4 chars in the
// timer callback so its bounding box never grows; the clock sits at a
// fixed offset to its left so neither moves as digits change.
static void home_attach_clock_battery(lv_obj_t *page, int tile_idx) {
    if (tile_idx < 0 || tile_idx >= MECK_HOME_PAGE_COUNT) return;

    lv_obj_t *bat = lv_label_create(page);
    lv_label_set_text(bat, "");
    meck_set_font(bat, &meck_montserrat_24, 0);
    lv_obj_set_style_text_align(bat, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(bat, LV_ALIGN_TOP_RIGHT, -15, NOTCH_SAFE_Y);
    lv_obj_add_flag(bat, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(bat, on_battery_label_tap, LV_EVENT_CLICKED, NULL);
    lbl_home_battery[tile_idx] = bat;

    lv_obj_t *clk = lv_label_create(page);
    lv_label_set_text(clk, "");
    lv_obj_set_style_text_color(clk, lv_color_white(), 0);
    meck_set_font(clk, &meck_montserrat_24, 0);
#if MECK_IS_AMOLED
    lv_obj_set_style_text_align(clk, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(clk, LV_ALIGN_TOP_MID, 0, NOTCH_SAFE_Y);
#else
    lv_obj_set_style_text_align(clk, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(clk, LV_ALIGN_TOP_RIGHT, -165, NOTCH_SAFE_Y);
#endif
    lbl_home_clock[tile_idx] = clk;

    lv_obj_t *aud = lv_label_create(page);
    lv_label_set_text(aud, "");
    lv_obj_set_style_text_color(aud, lv_palette_main(LV_PALETTE_BLUE), 0);
    meck_set_font(aud, &meck_montserrat_24, 0);
    lv_obj_set_style_text_align(aud, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(aud, LV_ALIGN_TOP_RIGHT, -85, NOTCH_SAFE_Y);
    lv_obj_add_flag(aud, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(aud, on_now_playing_tap, LV_EVENT_CLICKED, NULL);
    lbl_home_audio[tile_idx] = aud;
}

// Reposition the home-page clock (tile 0 only) when the node name is long
// enough to obscure it. If the name is 18+ characters, the clock moves
// below the battery percentage; otherwise it stays in its normal position.
static void home_reposition_clock_for_name(const char* name) {
    lv_obj_t *clk = lbl_home_clock[0];
    if (!clk) return;
    size_t len = name ? strlen(name) : 0;
    if (len >= 18) {
        lv_obj_set_style_text_align(clk, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(clk, LV_ALIGN_TOP_RIGHT, -15, NOTCH_SAFE_Y + 28);
    } else {
#if MECK_IS_AMOLED
        lv_obj_set_style_text_align(clk, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(clk, LV_ALIGN_TOP_MID, 0, NOTCH_SAFE_Y);
#else
        lv_obj_set_style_text_align(clk, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(clk, LV_ALIGN_TOP_RIGHT, -165, NOTCH_SAFE_Y);
#endif
    }
}

// Attach a clock + battery pair to any non-home screen or modal panel.
// Mirrors home_attach_clock_battery but writes into the screen/modal
// arrays. Each caller passes its own slot index (0..MECK_SCREEN_HOST_COUNT-1)
// so the timer can address every label individually. The caller also
// passes the font and y-offset so the clock/battery can be visually
// aligned with that screen's own title (e.g. Messages title is 22pt at
// y=30, Settings is 24pt at y=30).
static void screen_attach_clock_battery(lv_obj_t *parent, int slot,
                                        const lv_font_t* font, int y_offset) {
    if (slot < 0 || slot >= MECK_SCREEN_HOST_COUNT) return;

    lv_obj_t *bat = lv_label_create(parent);
    lv_label_set_text(bat, "");
    meck_set_font(bat, font, 0);
    lv_obj_set_style_text_align(bat, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(bat, LV_ALIGN_TOP_RIGHT, -15, y_offset);
    lv_obj_add_flag(bat, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(bat, on_battery_label_tap, LV_EVENT_CLICKED, NULL);
    lbl_screen_battery[slot] = bat;

    lv_obj_t *clk = lv_label_create(parent);
    lv_label_set_text(clk, "");
    lv_obj_set_style_text_color(clk, lv_color_white(), 0);
    meck_set_font(clk, font, 0);
#if MECK_IS_AMOLED
    lv_obj_set_style_text_align(clk, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(clk, LV_ALIGN_TOP_MID, 0, y_offset);
#else
    lv_obj_set_style_text_align(clk, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(clk, LV_ALIGN_TOP_RIGHT, -165, y_offset);
#endif
    lbl_screen_clock[slot] = clk;

    lv_obj_t *aud = lv_label_create(parent);
    lv_label_set_text(aud, "");
    lv_obj_set_style_text_color(aud, lv_palette_main(LV_PALETTE_BLUE), 0);
    meck_set_font(aud, font, 0);
    lv_obj_set_style_text_align(aud, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(aud, LV_ALIGN_TOP_RIGHT, -85, y_offset);
    lv_obj_add_flag(aud, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(aud, on_now_playing_tap, LV_EVENT_CLICKED, NULL);
    lbl_screen_audio[slot] = aud;
}

// Attach only the tappable ">>" now-playing indicator to a screen that is
// built outside the header-host system (the audio and reader file
// explorers). Registers it in lbl_screen_audio[slot] so the existing UI
// timer drives its visibility, and wires the same tap-to-return handler.
// The caller supplies a free slot and the top-right x/y offsets.
extern "C" void meck_ui_attach_audio_indicator(lv_obj_t *parent, int slot,
                                               const lv_font_t *font,
                                               int32_t x_ofs,
                                               int32_t y_ofs) {
    if (!parent || slot < 0 || slot >= MECK_SCREEN_HOST_COUNT) return;
    lv_obj_t *aud = lv_label_create(parent);
    lv_label_set_text(aud, "");
    lv_obj_set_style_text_color(aud, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_set_style_text_font(aud, font, 0);
    lv_obj_set_style_text_align(aud, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(aud, LV_ALIGN_TOP_RIGHT, x_ofs, y_ofs);
    lv_obj_add_flag(aud, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(aud, on_now_playing_tap, LV_EVENT_CLICKED, NULL);
    lbl_screen_audio[slot] = aud;
}
// ============================================================================

static void create_page_home(lv_obj_t *page) {
    Meck* mesh = meck_get_instance();
    P4NodePrefs* prefs = mesh ? mesh->getNodePrefs() : nullptr;

    // Reload the home colour scheme from NVS (idempotent — safe even on a
    // rebuild) and clear the tile registry so this build's tiles are the
    // only ones tracked.
    load_home_color_from_nvs();
#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
    load_p4batt_prefs_from_nvs();
#endif
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
    meck_set_font(lbl_home_title, meck_node_name_font(initial_name), 0);
    lv_obj_align(lbl_home_title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    home_attach_clock_battery(page, 0);
    home_reposition_clock_for_name(initial_name);

    lbl_home_unread = lv_label_create(page);
    lv_label_set_text(lbl_home_unread, "MSG: 0");
    lv_obj_set_style_text_color(lbl_home_unread, lv_color_make(0, 255, 100), 0);
    meck_set_font(lbl_home_unread, &meck_montserrat_18, 0);
    lv_obj_align(lbl_home_unread, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, 60);

    // BLE pairing PIN — shown only while BLE companion is enabled.
    // Hidden by default; ui_update_timer_cb toggles visibility.
    lbl_home_ble_pin = lv_label_create(page);
    lv_label_set_text(lbl_home_ble_pin, "");
    lv_obj_set_style_text_color(lbl_home_ble_pin,
        lv_palette_main(LV_PALETTE_CYAN), 0);
    meck_set_font(lbl_home_ble_pin, &meck_montserrat_14, 0);
    // Portrait stacks this as the 3rd left-column line (y=95). Landscape has a
    // wide, mostly-empty header row, so lift it up to the battery's level and
    // right-align it into the gap just left of the battery %. Anchoring to the
    // right edge keeps it clear of the clock, which sits centre for short node
    // names and under the battery for long ones. This also shortens the left
    // column to two lines, letting the tile grid move up (see create_tile_button).
    if (SCREEN_WIDTH > SCREEN_HEIGHT) {
        lv_obj_set_style_text_align(lbl_home_ble_pin, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(lbl_home_ble_pin, LV_ALIGN_TOP_RIGHT, -160, NOTCH_SAFE_Y);
    } else {
        lv_obj_align(lbl_home_ble_pin, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, 95);
    }
    lv_obj_add_flag(lbl_home_ble_pin, LV_OBJ_FLAG_HIDDEN);

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
    create_tile_button(page, LV_SYMBOL_IMAGE    "\nMaps",     cb_todo_maps,        1, 3);
    create_tile_button(page, LV_SYMBOL_SHUFFLE  "\nTrace",    cb_todo_trace,       0, 3);
    create_tile_button(page, LV_SYMBOL_AUDIO    "\nAudio",    goto_audio_browser,  0, 4);
    create_tile_button(page, LV_SYMBOL_WIFI     "\nWeb",      cb_open_web,         1, 4);
    // Voice and Camera tiles hidden until implementation is ready
    // create_tile_button(page, LV_SYMBOL_AUDIO    "\nVoice",    cb_todo_voice,       0, 5);
    // lv_obj_t *cam_btn = create_tile_button(page, LV_SYMBOL_IMAGE "\nCamera", cb_todo_camera, 1, 5);
    // if (cam_btn && g_home_color_scheme == HOME_COLOR_MULTI) {
    //     lv_obj_set_style_border_color(cam_btn, lv_palette_main(LV_PALETTE_PURPLE), 0);
    // }

    lv_obj_t *hint = lv_label_create(page);
    lv_label_set_text(hint, "Swipe left for more pages " LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(hint, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(hint, &meck_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -20);
}

// ============================================================================
// Tile 1: Recent Heard
// ============================================================================

static void create_page_recent(lv_obj_t *page) {
    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, "Recent Heard");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    lbl_recent_list = lv_label_create(page);
    lv_label_set_text(lbl_recent_list, "Waiting for adverts...");
    lv_obj_set_style_text_color(lbl_recent_list, lv_color_white(), 0);
    meck_set_font(lbl_recent_list, &meck_montserrat_16, 0);
    lv_label_set_long_mode(lbl_recent_list, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_recent_list, SCREEN_WIDTH - 40);
    lv_obj_align(lbl_recent_list, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, 80);

    home_attach_clock_battery(page, 1);
}

// ============================================================================
// Tile 2: Radio Details
// ============================================================================

static void create_page_radio(lv_obj_t *page) {
    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, "Radio Details");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_YELLOW), 0);
    meck_set_font(title, &meck_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    lbl_radio_detail = lv_label_create(page);
    lv_label_set_text(lbl_radio_detail, "Loading...");
    lv_obj_set_style_text_color(lbl_radio_detail, lv_color_white(), 0);
    meck_set_font(lbl_radio_detail, &meck_montserrat_18, 0);
    lv_label_set_long_mode(lbl_radio_detail, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_radio_detail, SCREEN_WIDTH - 40);
    lv_obj_align(lbl_radio_detail, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, 80);

    update_radio_detail_label();

    home_attach_clock_battery(page, 2);
}

// ============================================================================
// Tile 3: Advert (long-press to send manual advert)
// ============================================================================

static void advert_revert_timer_cb(lv_timer_t *t) {
    if (lbl_advert_status) {
        lv_label_set_text(lbl_advert_status, "Advert\n\nLong press to send");
        lv_obj_set_style_text_color(lbl_advert_status, lv_color_white(), 0);
    }
    lv_timer_delete(t);
}

static void advert_toggle_cb(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    mesh::Packet* adv;
    if (prefs->position_mode != 0 &&
        (prefs->position_lat_e7 != 0 || prefs->position_lon_e7 != 0)) {
        double lat = (double)prefs->position_lat_e7 / 1e7;
        double lon = (double)prefs->position_lon_e7 / 1e7;
        printf("Meck: advert WITH position lat=%.7f lon=%.7f (e7: %ld, %ld)\n",
               lat, lon, (long)prefs->position_lat_e7, (long)prefs->position_lon_e7);
        adv = mesh->createSelfAdvert(prefs->node_name, lat, lon);
    } else {
        printf("Meck: advert WITHOUT position (mode=%d lat_e7=%ld lon_e7=%ld)\n",
               prefs->position_mode,
               (long)prefs->position_lat_e7, (long)prefs->position_lon_e7);
        adv = mesh->createSelfAdvert(prefs->node_name);
    }
    if (adv) {
        mesh->sendFlood(adv);
        printf("Meck: manual advert sent (name='%s' pos_mode=%d payload=%d bytes)\n",
               prefs->node_name, prefs->position_mode, adv->payload_len);
        if (lbl_advert_status) {
            lv_label_set_text(lbl_advert_status, "Advert sent!");
            lv_obj_set_style_text_color(lbl_advert_status,
                lv_palette_main(LV_PALETTE_GREEN), 0);
            lv_timer_create(advert_revert_timer_cb, 3000, NULL);
        }
    }
}

static void create_page_advert(lv_obj_t *page) {
    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, "Adverts");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    lv_obj_t *icon = lv_label_create(page);
    lv_label_set_text(icon, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(icon, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(icon, &meck_montserrat_28, 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -30);

    lbl_advert_status = lv_label_create(page);
    lv_label_set_text(lbl_advert_status, "Advert\n\nLong press to send");
    lv_obj_set_style_text_color(lbl_advert_status, lv_color_white(), 0);
    meck_set_font(lbl_advert_status, &meck_montserrat_18, 0);
    lv_obj_set_style_text_align(lbl_advert_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl_advert_status, LV_ALIGN_CENTER, 0, 30);

    lv_obj_add_event_cb(page, advert_toggle_cb, LV_EVENT_LONG_PRESSED, NULL);
    lv_obj_add_flag(page, LV_OBJ_FLAG_CLICKABLE);

    home_attach_clock_battery(page, 3);
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
    meck_set_font(title, &meck_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    lbl_gps_detail = lv_label_create(page);
    lv_label_set_text(lbl_gps_detail, "Loading...");
    lv_obj_set_style_text_color(lbl_gps_detail, lv_color_white(), 0);
    meck_set_font(lbl_gps_detail, &meck_montserrat_18, 0);
    lv_label_set_long_mode(lbl_gps_detail, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_gps_detail, SCREEN_WIDTH - 40);
    lv_obj_align(lbl_gps_detail, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, 60);

    // Long-press anywhere on the tile toggles the GPS power gate. We
    // attach to the page (not just the label) so the entire tile area
    // is a hit target — easier than aiming for the label itself.
    lv_obj_add_flag(page, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(page, on_gps_tile_long_press,
                        LV_EVENT_LONG_PRESSED, NULL);

    home_attach_clock_battery(page, 4);
}

// ============================================================================
// Tile 5: Battery (placeholder)
// ============================================================================

static void create_page_battery(lv_obj_t *page) {
    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, "Battery Gauge");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    lbl_battery_detail = lv_label_create(page);
    lv_label_set_text(lbl_battery_detail, "Reading BQ27220...");
    lv_obj_set_style_text_color(lbl_battery_detail, lv_color_white(), 0);
    meck_set_font(lbl_battery_detail, &meck_montserrat_18, 0);
    lv_label_set_long_mode(lbl_battery_detail, LV_LABEL_LONG_WRAP);
#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
    // Two-column layout: internal cell left, keyboard pack right. 540 px wide
    // panel, so each column gets just under half and the line labels in the
    // update callback are compressed to match.
    lv_obj_set_width(lbl_battery_detail, (SCREEN_WIDTH / 2) - NOTCH_SAFE_X - 10);
#else
    lv_obj_set_width(lbl_battery_detail, SCREEN_WIDTH - 40);
#endif
    lv_obj_align(lbl_battery_detail, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, 80);

    home_attach_clock_battery(page, 5);

#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
    // Blue keyboard-pack header in the top row: right of the clock, level with
    // the battery percent -- the slot the now-playing label uses on the other
    // tiles. "KBD" rather than "Keyboard": the gap between the clock and the
    // percent is ~80 px and the full word at this size is ~110 px.
    lbl_kbd_batt_header = lv_label_create(page);
    lv_label_set_text(lbl_kbd_batt_header, "KBD");
    lv_obj_set_style_text_color(lbl_kbd_batt_header,
                                lv_palette_main(LV_PALETTE_BLUE), 0);
    meck_set_font(lbl_kbd_batt_header, &meck_montserrat_24, 0);
    lv_obj_set_style_text_align(lbl_kbd_batt_header, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(lbl_kbd_batt_header, LV_ALIGN_TOP_RIGHT, -85, NOTCH_SAFE_Y);
    lv_obj_add_flag(lbl_kbd_batt_header, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lbl_kbd_batt_header, on_kbd_batt_header_tap,
                        LV_EVENT_CLICKED, NULL);

    // This tile's now-playing label shares that slot; hide it here so the two
    // never overlap. The updater null-checks but writes to hidden labels
    // harmlessly, so only the flag is needed.
    if (lbl_home_audio[5]) lv_obj_add_flag(lbl_home_audio[5], LV_OBJ_FLAG_HIDDEN);

    lbl_kbd_batt_detail = lv_label_create(page);
    lv_label_set_text(lbl_kbd_batt_detail, "");
    lv_obj_set_style_text_color(lbl_kbd_batt_detail, lv_color_hex(0x808080), 0);
    meck_set_font(lbl_kbd_batt_detail, &meck_montserrat_18, 0);
    lv_label_set_long_mode(lbl_kbd_batt_detail, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_kbd_batt_detail, (SCREEN_WIDTH / 2) - 25);
    lv_obj_align(lbl_kbd_batt_detail, LV_ALIGN_TOP_LEFT,
                 (SCREEN_WIDTH / 2) + 5, 80);

    lbl_kbd_batt_cap = lv_label_create(page);
    lv_label_set_text(lbl_kbd_batt_cap, "");
    lv_obj_set_style_text_color(lbl_kbd_batt_cap, lv_color_hex(0x808080), 0);
    meck_set_font(lbl_kbd_batt_cap, &meck_montserrat_18, 0);
    lv_obj_align(lbl_kbd_batt_cap, LV_ALIGN_TOP_LEFT,
                 (SCREEN_WIDTH / 2) + 5, 215);
    lv_obj_add_flag(lbl_kbd_batt_cap, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lbl_kbd_batt_cap, on_kbd_batt_cap_tap,
                        LV_EVENT_CLICKED, NULL);
#endif // CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
}

// ============================================================================
// Tile 6: Hibernate
// ============================================================================

static void create_page_shutdown(lv_obj_t *page) {
    lv_obj_t *icon = lv_label_create(page);
    lv_label_set_text(icon, LV_SYMBOL_POWER);
    lv_obj_set_style_text_color(icon, lv_palette_main(LV_PALETTE_RED), 0);
    meck_set_font(icon, &meck_montserrat_28, 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -40);

    lv_obj_t *lbl = lv_label_create(page);
    lv_label_set_text(lbl, "Hibernate\n\nLong press to enter\ndeep sleep");
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    meck_set_font(lbl, &meck_montserrat_18, 0);
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 30);

    home_attach_clock_battery(page, 6);
}

// ============================================================================
// Settings → Contacts sub-screen
// ----------------------------------------------------------------------------
// Surfaces the auto-add config that already lives on P4NodePrefs:
//   manual_add_contacts (bit 0): 0 = auto-add new contacts, 1 = manual only
//   autoadd_config: bit-mask of which adv types to accept (chat/repeater/
//                   room/sensor) plus an "overwrite oldest non-fav" bit
//
// MeckMesh::shouldAutoAddContactType() already reads these on every advert,
// so changes apply immediately to incoming contacts. No mesh patch needed.
// ============================================================================

static int settings_contacts_get_mode() {
    Meck* mesh = meck_get_instance();
    if (!mesh) return CONTACT_MODE_AUTO_ALL;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return CONTACT_MODE_AUTO_ALL;

    if ((prefs->manual_add_contacts & 1) == 0) return CONTACT_MODE_AUTO_ALL;
    if ((prefs->autoadd_config & AUTO_ADD_ALL_TYPES) != 0) return CONTACT_MODE_CUSTOM;
    return CONTACT_MODE_MANUAL;
}

static const char* settings_contacts_mode_label(int mode) {
    switch (mode) {
        case CONTACT_MODE_AUTO_ALL: return "Auto All";
        case CONTACT_MODE_CUSTOM:   return "Custom";
        case CONTACT_MODE_MANUAL:   return "Manual Only";
        default:                    return "?";
    }
}

static void settings_contacts_apply_mode(int mode) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    switch (mode) {
        case CONTACT_MODE_AUTO_ALL:
            prefs->manual_add_contacts &= ~1;          // bit 0 clear → auto
            // Leave per-type bits as they were; in this mode they're ignored
            // by isAutoAddEnabled() but we keep them around for round-tripping
            // to Custom without losing the user's last selection.
            break;
        case CONTACT_MODE_CUSTOM:
            prefs->manual_add_contacts |= 1;
            // If no per-type bits are set, default to all-on so the user
            // sees something useful immediately after entering Custom.
            if ((prefs->autoadd_config & AUTO_ADD_ALL_TYPES) == 0) {
                prefs->autoadd_config |= AUTO_ADD_ALL_TYPES;
            }
            break;
        case CONTACT_MODE_MANUAL:
            prefs->manual_add_contacts |= 1;
            prefs->autoadd_config &= ~AUTO_ADD_ALL_TYPES;
            // Preserve AUTO_ADD_OVERWRITE_OLDEST since it's an orthogonal
            // policy.
            break;
    }
    mesh->getDataStore()->savePrefs(*prefs);
    printf("Contacts: mode=%s manual=0x%02X autoadd=0x%02X\n",
           settings_contacts_mode_label(mode),
           prefs->manual_add_contacts, prefs->autoadd_config);
}

static void settings_contacts_update_labels() {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    if (lbl_set_contact_mode) {
        lv_label_set_text(lbl_set_contact_mode,
            settings_contacts_mode_label(settings_contacts_get_mode()));
    }

    // Per-type rows: show "On" / "Off" derived from autoadd_config bits.
    auto on_off = [](bool b) { return b ? "On" : "Off"; };
    if (lbl_set_autoadd_chat) {
        lv_label_set_text(lbl_set_autoadd_chat,
            on_off((prefs->autoadd_config & AUTO_ADD_CHAT) != 0));
    }
    if (lbl_set_autoadd_repeater) {
        lv_label_set_text(lbl_set_autoadd_repeater,
            on_off((prefs->autoadd_config & AUTO_ADD_REPEATER) != 0));
    }
    if (lbl_set_autoadd_room) {
        lv_label_set_text(lbl_set_autoadd_room,
            on_off((prefs->autoadd_config & AUTO_ADD_ROOM_SERVER) != 0));
    }
    if (lbl_set_autoadd_sensor) {
        lv_label_set_text(lbl_set_autoadd_sensor,
            on_off((prefs->autoadd_config & AUTO_ADD_SENSOR) != 0));
    }
    if (lbl_set_autoadd_overwrite) {
        lv_label_set_text(lbl_set_autoadd_overwrite,
            on_off((prefs->autoadd_config & AUTO_ADD_OVERWRITE_OLDEST) != 0));
    }
}

static void on_settings_contact_mode_tap(lv_event_t *e) {
    int next = (settings_contacts_get_mode() + 1) % CONTACT_MODE_COUNT;
    settings_contacts_apply_mode(next);
    settings_contacts_update_labels();
}

// Per-type toggle handlers — flip a bit, save prefs, refresh labels.
// In Auto All / Manual modes the per-type bits are ignored by the auto-add
// gating logic, but we still let the user touch them so switching back to
// Custom retains their selection.
static void on_settings_autoadd_bit_toggle(uint8_t bit, const char* label) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    prefs->autoadd_config ^= bit;
    mesh->getDataStore()->savePrefs(*prefs);
    printf("Contacts: %s %s (autoadd=0x%02X)\n",
           label,
           (prefs->autoadd_config & bit) ? "ON" : "OFF",
           prefs->autoadd_config);
    settings_contacts_update_labels();
}

static void on_settings_autoadd_chat_tap(lv_event_t *e) {
    on_settings_autoadd_bit_toggle(AUTO_ADD_CHAT, "Chat");
}
static void on_settings_autoadd_repeater_tap(lv_event_t *e) {
    on_settings_autoadd_bit_toggle(AUTO_ADD_REPEATER, "Repeater");
}
static void on_settings_autoadd_room_tap(lv_event_t *e) {
    on_settings_autoadd_bit_toggle(AUTO_ADD_ROOM_SERVER, "Room Server");
}
static void on_settings_autoadd_sensor_tap(lv_event_t *e) {
    on_settings_autoadd_bit_toggle(AUTO_ADD_SENSOR, "Sensor");
}
static void on_settings_autoadd_overwrite_tap(lv_event_t *e) {
    on_settings_autoadd_bit_toggle(AUTO_ADD_OVERWRITE_OLDEST, "Overwrite oldest");
}

// Manual SD backup handler. Calls the datastore force-write path which
// re-emits every NVS blob (identity, prefs, channels, contacts) to the SD
// card. Updates the row's value label with the result so the user gets
// immediate feedback without a popup.
static void on_settings_backup_to_sd_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4DataStore* ds = mesh->getDataStore();
    if (!ds) return;

    int written = ds->backupToSD();
    char buf[24];
    if (written > 0) {
        snprintf(buf, sizeof(buf), "OK (%d)", written);
    } else {
        // Either no SD card mounted or every write failed. The datastore
        // already logs which case it was, so the UI just shows generic
        // "Failed" — checking serial gives the detail.
        snprintf(buf, sizeof(buf), "Failed");
    }
    if (lbl_set_backup_status) {
        lv_label_set_text(lbl_set_backup_status, buf);
        lv_obj_set_style_text_color(lbl_set_backup_status,
            written > 0 ? lv_palette_main(LV_PALETTE_GREEN)
                        : lv_palette_main(LV_PALETTE_RED), 0);
    }
    printf("Settings: backup to SD -> %s\n", buf);
}

// Open the Export Config modal. Defaults: all four section checkboxes
// ticked, identity warning visible. Hidden by clearing LV_OBJ_FLAG_HIDDEN
// on the panel; cancel/export close it the same way.
static void on_settings_export_config_tap(lv_event_t *e) {
    if (!obj_export_panel) return;
    // Reset to defaults each time the modal opens — the user's previous
    // selection on a prior tap shouldn't sticky-influence the next export.
    if (cb_export_identity) lv_obj_add_state(cb_export_identity, LV_STATE_CHECKED);
    if (cb_export_channels) lv_obj_add_state(cb_export_channels, LV_STATE_CHECKED);
    if (cb_export_contacts) lv_obj_add_state(cb_export_contacts, LV_STATE_CHECKED);
    if (cb_export_radio)    lv_obj_add_state(cb_export_radio,    LV_STATE_CHECKED);
    if (lbl_export_warn)    lv_obj_clear_flag(lbl_export_warn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_export_panel, LV_OBJ_FLAG_HIDDEN);
}

// ============================================================================
// Debug Logs subpage handlers
// ----------------------------------------------------------------------------
// Start fopen's /sdcard/meshcore/logs/log_<unix>.txt and flips
// g_debug_log_active true. Meck source files include "meck_log.h" which
// rewrites printf -> meck_debug_log_printf via macro; that function checks
// g_debug_log_active and either writes to the SD file (under mutex) or
// falls through to vprintf (UART) when inactive. Stop closes the file.
// Export stream-copies the most-recent session log to
// /sdcard/meck_debug_<unix>.txt at the SD root for easy access when the
// card is mounted on a computer.
//
// While Start is active, no output goes to the USB serial console — the
// printf wrapper writes only to SD. Serial returns the moment Stop is
// tapped. Writes are sync per line under a recursive FreeRTOS mutex, so
// cross-task printf calls serialise on the FILE* and may stall briefly
// when the SD is slow.
// ============================================================================

// printf wrapper installed via meck_log.h macro substitution. When debug
// logging is active, writes the formatted line to the SD log file under
// the recursive mutex. When inactive, falls through to vprintf so output
// goes to UART as it normally would.
//
// Returns the number of bytes written (matches printf semantics).
extern "C" int meck_debug_log_printf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);

    if (g_debug_log_active && g_debug_log_mutex) {
        xSemaphoreTakeRecursive(g_debug_log_mutex, portMAX_DELAY);
        // Re-check under the mutex — Stop might have just closed the
        // file between our check above and the take.
        if (g_debug_log_active && g_debug_log_fp) {
            int n = vfprintf(g_debug_log_fp, fmt, args);
            fflush(g_debug_log_fp);
            xSemaphoreGiveRecursive(g_debug_log_mutex);
            va_end(args);
            return n;
        }
        xSemaphoreGiveRecursive(g_debug_log_mutex);
        // Fell through — state changed under us. Treat as inactive.
    }

    // Inactive (or transitioning) — write to UART as normal printf would.
    int n = vprintf(fmt, args);
    va_end(args);
    return n;
}

// ESP-IDF log hook (registered in meck_ui_init via esp_log_set_vprintf).
// ESP_LOGx output bypasses printf and the meck_log.h redirection, so this
// captures it into the SD session log. Mirrors meck_debug_log_printf: SD only
// while active, vprintf (UART) when inactive.
extern "C" int meck_debug_log_vprintf(const char* fmt, va_list args) {
    if (g_debug_log_active && g_debug_log_mutex) {
        xSemaphoreTakeRecursive(g_debug_log_mutex, portMAX_DELAY);
        if (g_debug_log_active && g_debug_log_fp) {
            int n = vfprintf(g_debug_log_fp, fmt, args);
            fflush(g_debug_log_fp);
            xSemaphoreGiveRecursive(g_debug_log_mutex);
            return n;
        }
        xSemaphoreGiveRecursive(g_debug_log_mutex);
    }
    return vprintf(fmt, args);
}

// Refresh the button enable states + status label based on
// g_debug_log_active and whether a file/export exists.
static void debug_log_update_status() {
    // Button enable states.
    if (btn_debug_start) {
        if (g_debug_log_active) lv_obj_add_state(btn_debug_start, LV_STATE_DISABLED);
        else                    lv_obj_clear_state(btn_debug_start, LV_STATE_DISABLED);
    }
    if (btn_debug_stop) {
        if (g_debug_log_active) lv_obj_clear_state(btn_debug_stop, LV_STATE_DISABLED);
        else                    lv_obj_add_state(btn_debug_stop, LV_STATE_DISABLED);
    }
    if (btn_debug_export) {
        // Enabled only when capture is stopped AND a file exists.
        bool can_export = (!g_debug_log_active) && (g_debug_log_path[0] != '\0');
        if (can_export) lv_obj_clear_state(btn_debug_export, LV_STATE_DISABLED);
        else            lv_obj_add_state(btn_debug_export, LV_STATE_DISABLED);
    }

    // Status label text — pick the most informative state.
    if (lbl_debug_status) {
        char buf[160];
        if (g_debug_log_active) {
            // Trim path to basename for display.
            const char* slash = strrchr(g_debug_log_path, '/');
            const char* base  = slash ? slash + 1 : g_debug_log_path;
            snprintf(buf, sizeof(buf), "Recording -> %s", base);
        } else if (g_debug_export_path[0] != '\0') {
            const char* slash = strrchr(g_debug_export_path, '/');
            const char* base  = slash ? slash + 1 : g_debug_export_path;
            snprintf(buf, sizeof(buf), "Exported -> %s", base);
        } else if (g_debug_log_path[0] != '\0') {
            const char* slash = strrchr(g_debug_log_path, '/');
            const char* base  = slash ? slash + 1 : g_debug_log_path;
            snprintf(buf, sizeof(buf), "Stopped -> %s", base);
        } else {
            snprintf(buf, sizeof(buf), "Idle");
        }
        lv_label_set_text(lbl_debug_status, buf);
    }

    // Settings row value label.
    if (lbl_set_debug_logs) {
        if (g_debug_log_active) {
            lv_label_set_text(lbl_set_debug_logs, "Recording");
            lv_obj_set_style_text_color(lbl_set_debug_logs,
                lv_palette_main(LV_PALETTE_GREEN), 0);
        } else if (g_debug_log_path[0] != '\0') {
            lv_label_set_text(lbl_set_debug_logs, "Stopped");
            lv_obj_set_style_text_color(lbl_set_debug_logs,
                lv_palette_main(LV_PALETTE_GREY), 0);
        } else {
            lv_label_set_text(lbl_set_debug_logs, "Idle");
            lv_obj_set_style_text_color(lbl_set_debug_logs,
                lv_palette_main(LV_PALETTE_GREY), 0);
        }
    }
}

static void on_debug_log_start_tap(lv_event_t *e) {
    if (g_debug_log_active) return;

    // Lazily create the mutex on first Start. Recursive so a single task
    // calling printf inside another printf-style expression doesn't
    // self-deadlock.
    if (!g_debug_log_mutex) {
        g_debug_log_mutex = xSemaphoreCreateRecursiveMutex();
        if (!g_debug_log_mutex) {
            if (lbl_debug_status) lv_label_set_text(lbl_debug_status, "Start failed: mutex alloc");
            return;
        }
    }

    // Make sure /sdcard/meshcore/logs exists. mkdir on an existing dir
    // returns -1/EEXIST which we tolerate; only real fopen failure later
    // is fatal.
    mkdir("/sdcard/meshcore", 0755);
    mkdir("/sdcard/meshcore/logs", 0755);

    // Filename uses meck_clock_get_utc — if the RTC hasn't sync'd this
    // returns 0 and the filename ends up log_0.txt. Acceptable for a
    // diagnostic feature; once RTC is up the next session is timestamped.
    uint32_t now = meck_clock_get_utc();
    snprintf(g_debug_log_path, sizeof(g_debug_log_path),
             "/sdcard/meshcore/logs/log_%lu.txt", (unsigned long)now);

    FILE *fp = fopen(g_debug_log_path, "w");
    if (!fp) {
        // Direct vprintf-equivalent to UART since we haven't flipped
        // g_debug_log_active yet; this printf goes to UART as normal.
        printf("Debug log: fopen(%s) failed errno=%d\n", g_debug_log_path, errno);
        g_debug_log_path[0] = '\0';
        if (lbl_debug_status) lv_label_set_text(lbl_debug_status, "Start failed (see serial)");
        return;
    }
    // Line-buffer so each newline flushes promptly to SD; default block
    // buffering would hold up to ~4 KB before any output became visible
    // in the file mid-session.
    setvbuf(fp, NULL, _IOLBF, 0);

    // Publish the file pointer and flip active under the mutex so any
    // in-flight printf observers see a consistent state.
    xSemaphoreTakeRecursive(g_debug_log_mutex, portMAX_DELAY);
    g_debug_log_fp = fp;
    g_debug_log_active = true;
    g_debug_export_path[0] = '\0';   // clear stale export label from prior session
    fprintf(g_debug_log_fp, "==== Meck debug log started ts=%lu fw=\"%s\" v%s screen=%s ====\n",
            (unsigned long)now,
            MECK_FIRMWARE_NAME, MECK_FIRMWARE_VERSION,
            MECK_IS_AMOLED ? "AMOLED" : "non-AMOLED");
    fflush(g_debug_log_fp);
    xSemaphoreGiveRecursive(g_debug_log_mutex);

    debug_log_update_status();
}

static void on_debug_log_stop_tap(lv_event_t *e) {
    if (!g_debug_log_active) return;
    if (!g_debug_log_mutex) return;  // shouldn't happen — defensive

    // Take the mutex so any concurrent printf finishes before we close
    // the FILE*. Flip active to false under the mutex so the next
    // printf attempt routes to UART.
    xSemaphoreTakeRecursive(g_debug_log_mutex, portMAX_DELAY);
    if (g_debug_log_fp) {
        fprintf(g_debug_log_fp, "==== Meck debug log stopped ====\n");
        fflush(g_debug_log_fp);
        fclose(g_debug_log_fp);
        g_debug_log_fp = nullptr;
    }
    g_debug_log_active = false;
    xSemaphoreGiveRecursive(g_debug_log_mutex);

    debug_log_update_status();
}

static void on_debug_log_export_tap(lv_event_t *e) {
    if (g_debug_log_active) return;          // shouldn't be enabled, but defend
    if (g_debug_log_path[0] == '\0') return; // no file to export

    uint32_t now = meck_clock_get_utc();
    snprintf(g_debug_export_path, sizeof(g_debug_export_path),
             "/sdcard/meck_debug_%lu.txt", (unsigned long)now);

    FILE* src = fopen(g_debug_log_path, "rb");
    if (!src) {
        printf("Debug log export: fopen(%s, rb) failed errno=%d\n",
               g_debug_log_path, errno);
        g_debug_export_path[0] = '\0';
        if (lbl_debug_status) lv_label_set_text(lbl_debug_status, "Export failed (see serial)");
        return;
    }
    FILE* dst = fopen(g_debug_export_path, "wb");
    if (!dst) {
        printf("Debug log export: fopen(%s, wb) failed errno=%d\n",
               g_debug_export_path, errno);
        fclose(src);
        g_debug_export_path[0] = '\0';
        if (lbl_debug_status) lv_label_set_text(lbl_debug_status, "Export failed (see serial)");
        return;
    }

    char buf[1024];
    size_t n;
    bool ok = true;
    while ((n = fread(buf, 1, sizeof(buf), src)) > 0) {
        if (fwrite(buf, 1, n, dst) != n) { ok = false; break; }
    }
    fclose(src);
    fclose(dst);

    if (!ok) {
        printf("Debug log export: write to %s failed mid-copy\n", g_debug_export_path);
        g_debug_export_path[0] = '\0';
        if (lbl_debug_status) lv_label_set_text(lbl_debug_status, "Export failed (see serial)");
        return;
    }

    debug_log_update_status();
}

static void on_debug_logs_back(lv_event_t *e) {
    if (scr_settings) lv_screen_load(scr_settings);
}

static void on_settings_debug_logs_tap(lv_event_t *e) {
    debug_log_update_status();
    if (scr_debug_logs) lv_screen_load(scr_debug_logs);
}

static void create_debug_logs_screen() {
    scr_debug_logs = lv_obj_create(NULL);
    lock_screen_scroll(scr_debug_logs);
    lv_obj_set_style_bg_color(scr_debug_logs, lv_color_black(), 0);
    screen_attach_clock_battery(scr_debug_logs, 10, &meck_montserrat_22, 30);

    // Back button — top-left.
    lv_obj_t *btn_back = lv_button_create(scr_debug_logs);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_debug_logs_back, LV_EVENT_CLICKED, NULL);

    // Title.
    lv_obj_t *title = lv_label_create(scr_debug_logs);
    lv_label_set_text(title, "Debug Logs");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_CYAN), 0);
    meck_set_font(title, &meck_montserrat_22, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    // Status box — multi-line label inside a panel, room for the
    // current state and an export path.
    lv_obj_t *status_panel = lv_obj_create(scr_debug_logs);
    lv_obj_set_size(status_panel, SCREEN_WIDTH - 40, 120);
    lv_obj_set_pos(status_panel, 20, 110);
    lv_obj_set_style_bg_color(status_panel, lv_color_make(25, 25, 35), 0);
    lv_obj_set_style_border_width(status_panel, 1, 0);
    lv_obj_set_style_border_color(status_panel, lv_color_make(50, 50, 60), 0);
    lv_obj_set_style_radius(status_panel, 10, 0);
    lv_obj_set_style_pad_all(status_panel, 12, 0);
    lock_screen_scroll(status_panel);

    lbl_debug_status = lv_label_create(status_panel);
    lv_label_set_text(lbl_debug_status, "Idle");
    lv_obj_set_style_text_color(lbl_debug_status, lv_color_white(), 0);
    meck_set_font(lbl_debug_status, &meck_montserrat_16, 0);
    lv_label_set_long_mode(lbl_debug_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_debug_status, SCREEN_WIDTH - 80);

    // Help text below the status panel describing what each button does.
    lv_obj_t *lbl_help = lv_label_create(scr_debug_logs);
    lv_label_set_text(lbl_help,
        "Start: begin capturing stdout to SD\n"
        "Stop: close the file (serial returns)\n"
        "Export: copy the file to /sdcard root");
    lv_obj_set_style_text_color(lbl_help, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(lbl_help, &meck_montserrat_14, 0);
    lv_label_set_long_mode(lbl_help, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_help, SCREEN_WIDTH - 40);
    lv_obj_set_pos(lbl_help, 20, 250);

    // Three action buttons, stacked vertically below the help text.
    int btn_y = 360;
    int btn_h = 70;
    int btn_gap = 20;
    int btn_w = SCREEN_WIDTH - 40;

    btn_debug_start = lv_button_create(scr_debug_logs);
    lv_obj_set_size(btn_debug_start, btn_w, btn_h);
    lv_obj_set_pos(btn_debug_start, 20, btn_y);
    lv_obj_set_style_bg_color(btn_debug_start,
        lv_palette_darken(LV_PALETTE_GREEN, 1), 0);
    lv_obj_set_style_radius(btn_debug_start, 8, 0);
    lv_obj_t *sl = lv_label_create(btn_debug_start);
    lv_label_set_text(sl, "Start");
    lv_obj_set_style_text_color(sl, lv_color_white(), 0);
    meck_set_font(sl, &meck_montserrat_18, 0);
    lv_obj_center(sl);
    lv_obj_add_event_cb(btn_debug_start, on_debug_log_start_tap, LV_EVENT_CLICKED, NULL);
    btn_y += btn_h + btn_gap;

    btn_debug_stop = lv_button_create(scr_debug_logs);
    lv_obj_set_size(btn_debug_stop, btn_w, btn_h);
    lv_obj_set_pos(btn_debug_stop, 20, btn_y);
    lv_obj_set_style_bg_color(btn_debug_stop,
        lv_palette_darken(LV_PALETTE_RED, 1), 0);
    lv_obj_set_style_radius(btn_debug_stop, 8, 0);
    lv_obj_t *tl = lv_label_create(btn_debug_stop);
    lv_label_set_text(tl, "Stop");
    lv_obj_set_style_text_color(tl, lv_color_white(), 0);
    meck_set_font(tl, &meck_montserrat_18, 0);
    lv_obj_center(tl);
    lv_obj_add_event_cb(btn_debug_stop, on_debug_log_stop_tap, LV_EVENT_CLICKED, NULL);
    btn_y += btn_h + btn_gap;

    btn_debug_export = lv_button_create(scr_debug_logs);
    lv_obj_set_size(btn_debug_export, btn_w, btn_h);
    lv_obj_set_pos(btn_debug_export, 20, btn_y);
    lv_obj_set_style_bg_color(btn_debug_export,
        lv_palette_darken(LV_PALETTE_INDIGO, 1), 0);
    lv_obj_set_style_radius(btn_debug_export, 8, 0);
    lv_obj_t *xl = lv_label_create(btn_debug_export);
    lv_label_set_text(xl, "Export");
    lv_obj_set_style_text_color(xl, lv_color_white(), 0);
    meck_set_font(xl, &meck_montserrat_18, 0);
    lv_obj_center(xl);
    lv_obj_add_event_cb(btn_debug_export, on_debug_log_export_tap, LV_EVENT_CLICKED, NULL);

    // Set initial button states (Start enabled, Stop/Export disabled).
    debug_log_update_status();
}

// Show/hide the identity-warning sub-label as the Identity checkbox is
// toggled. Warning only makes sense when identity is being included.
static void on_export_modal_identity_changed(lv_event_t *e) {
    if (!cb_export_identity || !lbl_export_warn) return;
    bool checked = lv_obj_has_state(cb_export_identity, LV_STATE_CHECKED);
    if (checked) {
        lv_obj_clear_flag(lbl_export_warn, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(lbl_export_warn, LV_OBJ_FLAG_HIDDEN);
    }
}

// Dismiss the modal without exporting.
static void on_export_modal_cancel(lv_event_t *e) {
    if (obj_export_panel) lv_obj_add_flag(obj_export_panel, LV_OBJ_FLAG_HIDDEN);
}

// Build the flags bitmask from the four checkboxes and call the bridge.
// Reuses lbl_set_backup_status for the result line — saves a row, at the
// cost of the label briefly meaning "last export" when the user has just
// done an export but is still looking at the Backup to SD row name. Worth
// it for the tighter layout.
static void on_export_modal_export(lv_event_t *e) {
    uint32_t flags = 0;
    if (cb_export_identity && lv_obj_has_state(cb_export_identity, LV_STATE_CHECKED))
        flags |= MECK_EXPORT_IDENTITY;
    if (cb_export_channels && lv_obj_has_state(cb_export_channels, LV_STATE_CHECKED))
        flags |= MECK_EXPORT_CHANNELS;
    if (cb_export_contacts && lv_obj_has_state(cb_export_contacts, LV_STATE_CHECKED))
        flags |= MECK_EXPORT_CONTACTS;
    if (cb_export_radio && lv_obj_has_state(cb_export_radio, LV_STATE_CHECKED))
        flags |= MECK_EXPORT_RADIO;

    char filename[64] = {0};
    bool ok = meck_export_to_sd_with_flags(flags, filename, sizeof(filename));

    if (lbl_set_backup_status) {
        char buf[80];
        if (ok) {
            snprintf(buf, sizeof(buf), "OK: %s", filename);
        } else {
            snprintf(buf, sizeof(buf), "Export failed");
        }
        lv_label_set_text(lbl_set_backup_status, buf);
        lv_obj_set_style_text_color(lbl_set_backup_status,
            ok ? lv_palette_main(LV_PALETTE_GREEN)
               : lv_palette_main(LV_PALETTE_RED), 0);
    }
    printf("Settings: export to SD (flags=0x%X) -> %s%s\n",
           (unsigned)flags,
           ok ? "OK " : "FAIL",
           ok ? filename : "");

    if (obj_export_panel) lv_obj_add_flag(obj_export_panel, LV_OBJ_FLAG_HIDDEN);
}

// ============================================================================
// Retry-send long-press feature (channel + DM)
// ----------------------------------------------------------------------------
// rebuild_message_bubbles / rebuild_dm_bubbles attach a long-press handler
// to every sent bubble. The handler re-checks eligibility against the
// current ring/ACK state at press time (not at rebuild time) so 18 seconds
// of elapsed real time becomes retryable without depending on a rebuild
// trigger. If eligible, the retry modal opens.
// ============================================================================

// LV_EVENT_DELETE handler — frees the BubbleRetryCtx that was attached as
// user_data on bubble creation. The bubble's user_data is owned by us;
// LVGL doesn't free it on object destruction so the cleanup is explicit.
static void on_bubble_retry_ctx_delete(lv_event_t *e) {
    lv_obj_t *bubble = (lv_obj_t*)lv_event_get_target(e);
    if (!bubble) return;
    BubbleRetryCtx *ctx = (BubbleRetryCtx*)lv_obj_get_user_data(bubble);
    if (ctx) {
        delete ctx;
        lv_obj_set_user_data(bubble, NULL);
    }
}

// Look up current heard_count for a channel message by (ch_idx, timestamp).
// Returns -1 if not found, otherwise the count. Iterates the channel's
// ring snapshot via getMessages — cheap (only called on long-press).
static int find_channel_msg_heard_count(uint8_t ch_idx, uint32_t timestamp) {
    Meck *mesh = meck_get_instance();
    if (!mesh) return -1;
    // P4_MSG_PER_CHANNEL is 500; one snapshot is ~100 KB — too big for
    // stack. Allocate in PSRAM for the duration of the lookup.
    P4ChannelMessage *snap = (P4ChannelMessage*)heap_caps_malloc(
        P4_MSG_PER_CHANNEL * sizeof(P4ChannelMessage), MALLOC_CAP_SPIRAM);
    if (!snap) return -1;
    int n = mesh->getMessages(snap, P4_MSG_PER_CHANNEL, ch_idx);
    int out = -1;
    for (int i = 0; i < n; i++) {
        if (snap[i].timestamp == timestamp) {
            out = (int)snap[i].heard_count;
            break;
        }
    }
    heap_caps_free(snap);
    return out;
}

// Re-check eligibility against current state. Returns true if the
// retry modal should open for this context.
static bool retry_is_eligible(const BubbleRetryCtx *ctx) {
    if (!ctx) return false;
    if (!ctx->is_outgoing) return false;  // only outgoing can retry
    if (ctx->is_dm) {
        Meck *mesh = meck_get_instance();
        if (!mesh) return false;
        if (ctx->expected_ack == 0) return false;   // no live ACK table entry
        bool          acked      = false;
        unsigned long sent_ms    = 0;
        uint32_t      timeout_ms = 0;
        if (!mesh->lookupDMAckStatus(ctx->expected_ack, acked,
                                     sent_ms, timeout_ms)) {
            return false;                            // rolled out — unknown
        }
        if (acked) return false;                     // delivered
        unsigned long now = (unsigned long)esp_log_timestamp();
        return (now - sent_ms) >= timeout_ms;        // Failed
    } else {
        int hc = find_channel_msg_heard_count((uint8_t)ctx->target_idx,
                                              ctx->timestamp);
        if (hc < 0) return false;                    // not found
        if (hc > 0) return false;                    // already echoed
        uint32_t now = meck_clock_get_utc();
        if (now < ctx->timestamp) return false;      // clock skew
        return (now - ctx->timestamp) >= MECK_RETRY_CHANNEL_THRESHOLD_SEC;
    }
}

// Strip emoji and multi-byte symbols from a name, keeping only ASCII and
// 2-byte UTF-8 (Latin accented characters). Trims leading/trailing spaces.
static void strip_emojis(const char *src, char *dest, int dest_len) {
    int o = 0;
    const uint8_t *p = (const uint8_t*)src;
    while (*p && o < dest_len - 1) {
        if (*p < 0x80) {
            // ASCII
            dest[o++] = (char)*p++;
        } else if ((*p & 0xE0) == 0xC0) {
            // 2-byte UTF-8 (Latin extended, accented chars) — keep
            if (o + 2 < dest_len && (p[1] & 0xC0) == 0x80) {
                dest[o++] = (char)*p++;
                dest[o++] = (char)*p++;
            } else {
                p++;
            }
        } else if ((*p & 0xF0) == 0xE0) {
            // 3-byte UTF-8 (emoji, CJK, symbols) — skip
            p++;
            if ((*p & 0xC0) == 0x80) p++;
            if ((*p & 0xC0) == 0x80) p++;
        } else if ((*p & 0xF8) == 0xF0) {
            // 4-byte UTF-8 (supplementary plane emoji) — skip
            p++;
            if ((*p & 0xC0) == 0x80) p++;
            if ((*p & 0xC0) == 0x80) p++;
            if ((*p & 0xC0) == 0x80) p++;
        } else {
            p++;  // malformed — skip
        }
    }
    dest[o] = '\0';
    // Trim leading spaces
    int start = 0;
    while (dest[start] == ' ') start++;
    if (start > 0) memmove(dest, dest + start, strlen(dest + start) + 1);
    // Trim trailing spaces and dashes
    int len = strlen(dest);
    while (len > 0 && (dest[len-1] == ' ' || dest[len-1] == '-')) dest[--len] = '\0';
}

// Look up a path hash against the contact list. Returns the contact name
// (emoji-stripped) or a hex string if no match is found.
// hex_out is always filled with the 2-4 char hex prefix.
static const char* lookup_hash_name(const uint8_t *hash, uint8_t hash_size,
                                     char *name_out, int name_out_len,
                                     char *hex_out, int hex_out_len) {
    // Always format the hex prefix
    if (hash_size == 1)
        snprintf(hex_out, hex_out_len, "%02X", hash[0]);
    else
        snprintf(hex_out, hex_out_len, "%02X%02X", hash[0], hash[1]);

    Meck *mesh = meck_get_instance();
    if (mesh) {
        ContactInfo *ci = mesh->lookupContactByPubKey(hash, hash_size);
        if (ci && ci->name[0] != '\0') {
            strip_emojis(ci->name, name_out, name_out_len);
            if (name_out[0] != '\0') return name_out;
        }
    }
    name_out[0] = '\0';
    return NULL;  // no name found
}

// Build a human-readable path description from a BubbleRetryCtx.
// Writes into g_path_copy_text for the "Copy Path" button.
static void build_path_info_text(const BubbleRetryCtx *ctx) {
    char buf[256];
    int pos = 0;

    if (ctx->is_outgoing) {
        // Outgoing: show echo sources
        if (ctx->heard_count == 0) {
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                            "No repeater echoes received");
        } else if (ctx->echo_hash_count > 0) {
            pos += snprintf(buf + pos, sizeof(buf) - pos, "Heard by:\n");
            for (int i = 0; i < ctx->echo_hash_count && i < 8; i++) {
                char name[48], hex[8];
                const char *n = lookup_hash_name(
                    &ctx->echo_hashes[i * ctx->echo_hash_size],
                    ctx->echo_hash_size, name, sizeof(name),
                    hex, sizeof(hex));
                if (n)
                    pos += snprintf(buf + pos, sizeof(buf) - pos,
                                    "  %d. (%s) %s\n", i + 1, hex, n);
                else
                    pos += snprintf(buf + pos, sizeof(buf) - pos,
                                    "  %d. (%s) Unknown\n", i + 1, hex);
                if (pos >= (int)sizeof(buf) - 20) break;
            }
            if (ctx->heard_count > ctx->echo_hash_count) {
                pos += snprintf(buf + pos, sizeof(buf) - pos,
                                "  +%d more",
                                ctx->heard_count - ctx->echo_hash_count);
            }
        } else {
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                            "Heard %d Repeat%s",
                            ctx->heard_count,
                            ctx->heard_count == 1 ? "" : "s");
        }
    } else {
        // Incoming: show path as numbered list
        uint8_t hops = ctx->msg_path_hash_count;
        if (ctx->path_len == 0xFF) {
            pos += snprintf(buf + pos, sizeof(buf) - pos, "Route: direct");
        } else if (hops == 0) {
            pos += snprintf(buf + pos, sizeof(buf) - pos, "Route: direct (0 hops)");
        } else {
            pos += snprintf(buf + pos, sizeof(buf) - pos, "Route (%d hop%s):\n",
                            hops, hops == 1 ? "" : "s");
            for (int i = 0; i < hops && i < 8; i++) {
                char name[48], hex[8];
                const char *n = lookup_hash_name(
                    &ctx->msg_path_hashes[i * ctx->msg_path_hash_size],
                    ctx->msg_path_hash_size, name, sizeof(name),
                    hex, sizeof(hex));
                if (n)
                    pos += snprintf(buf + pos, sizeof(buf) - pos,
                                    "  %d. (%s) %s\n", i + 1, hex, n);
                else
                    pos += snprintf(buf + pos, sizeof(buf) - pos,
                                    "  %d. (%s) Unknown\n", i + 1, hex);
                if (pos >= (int)sizeof(buf) - 20) break;
            }
        }
    }
    buf[sizeof(buf) - 1] = '\0';
    strncpy(g_path_copy_text, buf, sizeof(g_path_copy_text) - 1);
    g_path_copy_text[sizeof(g_path_copy_text) - 1] = '\0';
}

static void on_path_modal_close(lv_event_t *e) {
    if (obj_path_panel) lv_obj_add_flag(obj_path_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_path_modal_copy(lv_event_t *e) {
    if (ta_compose && g_path_copy_text[0] != '\0') {
        lv_textarea_set_text(ta_compose, g_path_copy_text);
    }
    if (obj_path_panel) lv_obj_add_flag(obj_path_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_path_modal_reply(lv_event_t *e) {
    if (ta_compose && g_reply_pending_sender[0] != '\0') {
        char prefill[72];
        // Bracketed form: other MeshCore devices and the companion app parse
        // "@[node name]" as a node-name tag. The plain "@name" form is not
        // recognised as a tag and breaks on names containing spaces.
        snprintf(prefill, sizeof(prefill), "@[%s] ", g_reply_pending_sender);
        lv_textarea_set_text(ta_compose, prefill);
    }
    if (obj_path_panel) lv_obj_add_flag(obj_path_panel, LV_OBJ_FLAG_HIDDEN);
}

// LV_EVENT_LONG_PRESSED handler — attached to every bubble (sent and
// received). For sent bubbles that are retry-eligible, opens the retry
// modal. For all bubbles, shows path/echo info with a Copy button.
static void on_bubble_long_pressed(lv_event_t *e) {
    lv_obj_t *bubble = (lv_obj_t*)lv_event_get_target(e);
    if (!bubble) return;
    BubbleRetryCtx *ctx = (BubbleRetryCtx*)lv_obj_get_user_data(bubble);
    if (!ctx) return;

    // For outgoing failed messages, show retry modal (existing behaviour)
    if (ctx->is_outgoing && retry_is_eligible(ctx)) {
        g_retry_pending_is_dm = ctx->is_dm;
        g_retry_pending_idx   = ctx->target_idx;
        strncpy(g_retry_pending_body, ctx->body, sizeof(g_retry_pending_body) - 1);
        g_retry_pending_body[sizeof(g_retry_pending_body) - 1] = '\0';
        if (lbl_retry_prompt) {
            char preview[80];
            snprintf(preview, sizeof(preview), "Retry sending:\n\"%s\"",
                     g_retry_pending_body);
            lv_label_set_text(lbl_retry_prompt, preview);
        }
        if (obj_retry_panel) {
            lv_obj_clear_flag(obj_retry_panel, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(obj_retry_panel);
        }
        return;
    }

    // For all other cases, show path info popup
    build_path_info_text(ctx);
    if (lbl_path_info) {
        lv_label_set_text(lbl_path_info, g_path_copy_text);
    }
    // Offer Reply only for received channel messages (not sent, not DM).
    if (!ctx->is_outgoing && !ctx->is_dm && ctx->sender[0] != '\0') {
        strncpy(g_reply_pending_sender, ctx->sender,
                sizeof(g_reply_pending_sender) - 1);
        g_reply_pending_sender[sizeof(g_reply_pending_sender) - 1] = '\0';
        if (btn_path_reply) lv_obj_clear_flag(btn_path_reply, LV_OBJ_FLAG_HIDDEN);
    } else {
        g_reply_pending_sender[0] = '\0';
        if (btn_path_reply) lv_obj_add_flag(btn_path_reply, LV_OBJ_FLAG_HIDDEN);
    }
    if (obj_path_panel) {
        lv_obj_clear_flag(obj_path_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(obj_path_panel);
    }
}

static void on_retry_modal_cancel(lv_event_t *e) {
    if (obj_retry_panel) lv_obj_add_flag(obj_retry_panel, LV_OBJ_FLAG_HIDDEN);
    g_retry_pending_idx = -1;
    g_retry_pending_body[0] = '\0';
}

static void on_retry_modal_send(lv_event_t *e) {
    if (g_retry_pending_idx < 0 || g_retry_pending_body[0] == '\0') {
        if (obj_retry_panel) lv_obj_add_flag(obj_retry_panel, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    if (g_retry_pending_is_dm) {
        meck_request_send_dm(g_retry_pending_idx, g_retry_pending_body);
    } else {
        meck_request_send_text((uint8_t)g_retry_pending_idx,
                               g_retry_pending_body);
    }
    if (obj_retry_panel) lv_obj_add_flag(obj_retry_panel, LV_OBJ_FLAG_HIDDEN);
    g_retry_pending_idx = -1;
    g_retry_pending_body[0] = '\0';
}

// Build the retry confirmation modal as an overlay on scr_messages.
// scr_messages is reused by the DM conversation view, so one modal
// covers both channel and DM long-press cases.
static void create_retry_modal() {
    if (!scr_messages) return;

    obj_retry_panel = lv_obj_create(scr_messages);
    lv_obj_set_size(obj_retry_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_retry_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_retry_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_retry_panel, LV_OPA_80, 0);
    lv_obj_set_style_border_width(obj_retry_panel, 0, 0);
    lv_obj_add_flag(obj_retry_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_retry_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_retry_panel, LV_SCROLLBAR_MODE_OFF);

    // Inner panel — centred card with the prompt and two buttons.
    int card_w = SCREEN_WIDTH - 80;
    int card_h = 280;
    lv_obj_t *card = lv_obj_create(obj_retry_panel);
    lv_obj_set_size(card, card_w, card_h);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_make(25, 25, 35), 0);
    lv_obj_set_style_radius(card, 12, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(card, lv_color_make(60, 60, 80), 0);
    lv_obj_set_style_pad_all(card, 16, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lbl_retry_prompt = lv_label_create(card);
    lv_label_set_text(lbl_retry_prompt, "Retry sending?");
    lv_obj_set_style_text_color(lbl_retry_prompt, lv_color_white(), 0);
    meck_set_font(lbl_retry_prompt, &meck_montserrat_18, 0);
    lv_label_set_long_mode(lbl_retry_prompt, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_retry_prompt, card_w - 32);
    lv_obj_align(lbl_retry_prompt, LV_ALIGN_TOP_LEFT, 0, 0);

    // Retry Send button — cyan, matches the existing send button colour.
    lv_obj_t *btn_send = lv_button_create(card);
    lv_obj_set_size(btn_send, card_w - 32, 60);
    lv_obj_align(btn_send, LV_ALIGN_BOTTOM_MID, 0, -70);
    lv_obj_set_style_bg_color(btn_send, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(btn_send, 8, 0);
    lv_obj_t *lbl_send = lv_label_create(btn_send);
    lv_label_set_text(lbl_send, "Retry Send");
    lv_obj_set_style_text_color(lbl_send, lv_color_white(), 0);
    meck_set_font(lbl_send, &meck_montserrat_18, 0);
    lv_obj_center(lbl_send);
    lv_obj_add_event_cb(btn_send, on_retry_modal_send, LV_EVENT_CLICKED, NULL);

    // Cancel button — neutral grey.
    lv_obj_t *btn_cancel = lv_button_create(card);
    lv_obj_set_size(btn_cancel, card_w - 32, 60);
    lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(btn_cancel, lv_color_make(60, 60, 70), 0);
    lv_obj_set_style_radius(btn_cancel, 8, 0);
    lv_obj_t *lbl_cancel = lv_label_create(btn_cancel);
    lv_label_set_text(lbl_cancel, "Cancel");
    lv_obj_set_style_text_color(lbl_cancel, lv_color_white(), 0);
    meck_set_font(lbl_cancel, &meck_montserrat_18, 0);
    lv_obj_center(lbl_cancel);
    lv_obj_add_event_cb(btn_cancel, on_retry_modal_cancel, LV_EVENT_CLICKED, NULL);
}

// Path info modal — shows repeater chain for incoming messages or echo
// sources for outgoing messages. "Copy Path" pastes the text into compose.
static void create_path_modal() {
    if (!scr_messages) return;

    obj_path_panel = lv_obj_create(scr_messages);
    lv_obj_set_size(obj_path_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_path_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_path_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_path_panel, LV_OPA_80, 0);
    lv_obj_set_style_border_width(obj_path_panel, 0, 0);
    lv_obj_add_flag(obj_path_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_path_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_path_panel, LV_SCROLLBAR_MODE_OFF);

    int card_w = SCREEN_WIDTH - 80;
    int card_h = 360;
    lv_obj_t *card = lv_obj_create(obj_path_panel);
    lv_obj_set_size(card, card_w, card_h);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_make(25, 25, 35), 0);
    lv_obj_set_style_radius(card, 12, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(card, lv_palette_main(LV_PALETTE_TEAL), 0);
    lv_obj_set_style_pad_all(card, 16, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, "Message Path");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_TEAL), 0);
    meck_set_font(title, &meck_montserrat_22, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 0);

    lbl_path_info = lv_label_create(card);
    lv_label_set_text(lbl_path_info, "");
    lv_obj_set_style_text_color(lbl_path_info, lv_color_white(), 0);
    meck_set_font(lbl_path_info, &meck_montserrat_16, 0);
    lv_label_set_long_mode(lbl_path_info, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_path_info, card_w - 32);
    lv_obj_align(lbl_path_info, LV_ALIGN_TOP_LEFT, 0, 35);

    // Reply button — hidden unless a received channel bubble is long-pressed
    // (shown/positioned above Copy Path when visible).
    btn_path_reply = lv_button_create(card);
    lv_obj_set_size(btn_path_reply, card_w - 32, 55);
    lv_obj_align(btn_path_reply, LV_ALIGN_BOTTOM_MID, 0, -130);
    lv_obj_set_style_bg_color(btn_path_reply, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(btn_path_reply, 8, 0);
    lv_obj_t *lbl_reply = lv_label_create(btn_path_reply);
    lv_label_set_text(lbl_reply, "Reply");
    lv_obj_set_style_text_color(lbl_reply, lv_color_white(), 0);
    meck_set_font(lbl_reply, &meck_montserrat_18, 0);
    lv_obj_center(lbl_reply);
    lv_obj_add_event_cb(btn_path_reply, on_path_modal_reply, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(btn_path_reply, LV_OBJ_FLAG_HIDDEN);

    // Copy Path button
    lv_obj_t *btn_copy = lv_button_create(card);
    lv_obj_set_size(btn_copy, card_w - 32, 55);
    lv_obj_align(btn_copy, LV_ALIGN_BOTTOM_MID, 0, -65);
    lv_obj_set_style_bg_color(btn_copy, lv_palette_main(LV_PALETTE_TEAL), 0);
    lv_obj_set_style_radius(btn_copy, 8, 0);
    lv_obj_t *lbl_copy = lv_label_create(btn_copy);
    lv_label_set_text(lbl_copy, "Copy Path");
    lv_obj_set_style_text_color(lbl_copy, lv_color_white(), 0);
    meck_set_font(lbl_copy, &meck_montserrat_18, 0);
    lv_obj_center(lbl_copy);
    lv_obj_add_event_cb(btn_copy, on_path_modal_copy, LV_EVENT_CLICKED, NULL);

    // Close button
    lv_obj_t *btn_close = lv_button_create(card);
    lv_obj_set_size(btn_close, card_w - 32, 55);
    lv_obj_align(btn_close, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(btn_close, lv_color_make(60, 60, 70), 0);
    lv_obj_set_style_radius(btn_close, 8, 0);
    lv_obj_t *lbl_close = lv_label_create(btn_close);
    lv_label_set_text(lbl_close, "Close");
    lv_obj_set_style_text_color(lbl_close, lv_color_white(), 0);
    meck_set_font(lbl_close, &meck_montserrat_18, 0);
    lv_obj_center(lbl_close);
    lv_obj_add_event_cb(btn_close, on_path_modal_close, LV_EVENT_CLICKED, NULL);
}

// Slider event handler. LV_EVENT_VALUE_CHANGED fires continuously as the
// user drags; we apply the new brightness to the panel immediately so the
// change is visible while sliding, but don't persist until release to
// avoid hammering NVS. LV_EVENT_RELEASED fires once when the finger lifts.
static void on_settings_brightness_slider_event(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    if (!slider_set_brightness) return;

    int pct = (int)lv_slider_get_value(slider_set_brightness);
    if (pct < MECK_BRIGHTNESS_MIN_PCT) pct = MECK_BRIGHTNESS_MIN_PCT;
    if (pct > MECK_BRIGHTNESS_MAX_PCT) pct = MECK_BRIGHTNESS_MAX_PCT;
    uint8_t raw = (uint8_t)((pct * 255 + 50) / 100);

    meck_screen_set_brightness(raw);

    if (lbl_set_brightness_pct) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d%%", pct);
        lv_label_set_text(lbl_set_brightness_pct, buf);
    }

    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_RELEASED) {
        prefs->screen_brightness = raw;
        mesh->getDataStore()->savePrefs(*prefs);
        printf("Settings: brightness = %u (%d%%)\n", (unsigned)raw, pct);
    }
}

static void on_settings_screen_off_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    int cur = -1;
    for (int i = 0; i < NUM_SCREEN_OFF_OPTIONS; i++) {
        if (SCREEN_OFF_OPTIONS[i] == prefs->screen_off_minutes) { cur = i; break; }
    }
    cur = (cur + 1) % NUM_SCREEN_OFF_OPTIONS;
    prefs->screen_off_minutes = SCREEN_OFF_OPTIONS[cur];
    mesh->getDataStore()->savePrefs(*prefs);

    if (lbl_set_screen_off) {
        char buf[40];
        if (prefs->screen_off_minutes == 0) {
            snprintf(buf, sizeof(buf), "Never");
        } else {
            snprintf(buf, sizeof(buf), "%u min",
                     (unsigned)prefs->screen_off_minutes);
        }
        lv_label_set_text(lbl_set_screen_off, buf);
    }
    printf("Settings: screen off = %u min\n",
           (unsigned)prefs->screen_off_minutes);
}

// Keyboard theme toggle: 0 = dark (default), 1 = light. Two states only —
// a single tap flips between them. Live-applies via meck_kb_restyle_all()
// so the next keyboard the user opens (or any already-built one) shows the
// new theme without reboot.
static void on_settings_kb_theme_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    prefs->kb_dark_mode = (prefs->kb_dark_mode == 0) ? 1 : 0;
    mesh->getDataStore()->savePrefs(*prefs);

    if (lbl_set_kb_theme) {
        lv_label_set_text(lbl_set_kb_theme,
            prefs->kb_dark_mode == 0 ? "Dark" : "Light");
    }
    meck_kb_restyle_all();
    printf("Settings: kb theme = %s\n",
           prefs->kb_dark_mode == 0 ? "Dark" : "Light");
}

// Keyboard layout cycle: 0 = QWERTY, 1 = AZERTY, 2 = QWERTZ. The label
// shows the new layout; the underlying maps swap via meck_kb_restyle_all.
static void on_settings_kb_layout_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    prefs->kb_layout = (uint8_t)((prefs->kb_layout + 1) % 4);
    mesh->getDataStore()->savePrefs(*prefs);

    const char* name = "?";
    switch (prefs->kb_layout) {
        case 0: name = "QWERTY"; break;
        case 1: name = "AZERTY"; break;
        case 2: name = "QWERTZ"; break;
        case 3: name = "ЙЦУКЕН"; break;
    }
    if (lbl_set_kb_layout) lv_label_set_text(lbl_set_kb_layout, name);
    meck_kb_restyle_all();
    printf("Settings: kb layout = %s\n", name);
}

// Font size cycle: 0 = Classic, 1 = Larger, 2 = Extra Large. Each scale
// step maps every active label's base font to a larger pre-built size
// (Larger steps one row; Extra Large double-jumps where possible). 28 is
// the ceiling so 24/28 don't grow further. Walks the font registry to
// retroactively rescale every existing object, since LVGL bakes the font
// pointer at creation.
static void on_settings_font_scale_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    prefs->font_scale = (uint8_t)((prefs->font_scale + 1) % 3);
    mesh->getDataStore()->savePrefs(*prefs);

    if (lbl_set_font_scale) {
        lv_label_set_text(lbl_set_font_scale, meck_font_scale_name(prefs->font_scale));
    }
    meck_font_restyle_all();
    printf("Settings: font scale = %s\n", meck_font_scale_name(prefs->font_scale));
}

#if MECK_BLE_ENABLED
static void on_settings_ble_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    prefs->ble_enabled = (prefs->ble_enabled != 0) ? 0 : 1;
    mesh->getDataStore()->savePrefs(*prefs);

    meck_ble_set_enabled(prefs->ble_enabled != 0);

    if (lbl_set_ble) {
        lv_label_set_text(lbl_set_ble,
            prefs->ble_enabled != 0 ? "On" : "Off");
    }
    printf("Settings: BLE companion = %s\n",
           prefs->ble_enabled != 0 ? "On" : "Off");
}
#endif

// ============================================================================
// WiFi Companion sub-screen (Settings > WiFi Companion)
// ============================================================================

static void settings_wifi_update_labels() {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    if (lbl_set_wifi_toggle)
        lv_label_set_text(lbl_set_wifi_toggle, prefs->wifi_enabled ? "On" : "Off");
    if (lbl_set_wifi_ssid)
        lv_label_set_text(lbl_set_wifi_ssid, prefs->wifi_ssid[0] ? prefs->wifi_ssid : "(not set)");
    if (lbl_set_wifi_pass)
        lv_label_set_text(lbl_set_wifi_pass, prefs->wifi_password[0] ? "****" : "(not set)");
    if (lbl_set_wifi_ip) {
        const char* ip = meck_wifi_get_ip();
        lv_label_set_text(lbl_set_wifi_ip, (ip && ip[0]) ? ip : "Not connected");
    }
}

static void on_settings_wifi_toggle_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    prefs->wifi_enabled = (prefs->wifi_enabled != 0) ? 0 : 1;
    mesh->getDataStore()->savePrefs(*prefs);

    meck_wifi_set_enabled(prefs->wifi_enabled != 0);

#if MECK_BLE_ENABLED
    // If enabling WiFi, disable BLE in prefs too (mutual exclusivity)
    if (prefs->wifi_enabled && prefs->ble_enabled) {
        prefs->ble_enabled = 0;
        mesh->getDataStore()->savePrefs(*prefs);
    }
#endif

    settings_wifi_update_labels();
    printf("Settings: WiFi companion = %s\n",
           prefs->wifi_enabled != 0 ? "On" : "Off");
}

static void on_wifi_edit_save(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    if (ta_wifi_edit) {
        const char* text = lv_textarea_get_text(ta_wifi_edit);
        if (text) {
            if (g_wifi_edit_field == 0) {
                strncpy(prefs->wifi_ssid, text, sizeof(prefs->wifi_ssid) - 1);
                prefs->wifi_ssid[sizeof(prefs->wifi_ssid) - 1] = '\0';
                printf("Settings: WiFi SSID = '%s'\n", prefs->wifi_ssid);
            } else {
                strncpy(prefs->wifi_password, text, sizeof(prefs->wifi_password) - 1);
                prefs->wifi_password[sizeof(prefs->wifi_password) - 1] = '\0';
                printf("Settings: WiFi password updated\n");
            }
            mesh->getDataStore()->savePrefs(*prefs);
        }
    }
    meck_panel_edit_closed(ta_wifi_edit);
    if (obj_wifi_edit_panel) lv_obj_add_flag(obj_wifi_edit_panel, LV_OBJ_FLAG_HIDDEN);
    settings_wifi_update_labels();
}

static void on_wifi_edit_cancel(lv_event_t *e) {
    meck_panel_edit_closed(ta_wifi_edit);
    if (obj_wifi_edit_panel) lv_obj_add_flag(obj_wifi_edit_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_wifi_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY)       on_wifi_edit_save(NULL);
    else if (code == LV_EVENT_CANCEL) on_wifi_edit_cancel(NULL);
}

static void on_settings_wifi_ssid_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    g_wifi_edit_field = 0;
    if (obj_wifi_edit_panel && ta_wifi_edit) {
        lv_textarea_set_text(ta_wifi_edit, prefs->wifi_ssid);
        lv_textarea_set_password_mode(ta_wifi_edit, false);
        lv_obj_remove_flag(obj_wifi_edit_panel, LV_OBJ_FLAG_HIDDEN);
        if (kb_wifi_edit) lv_keyboard_set_textarea(kb_wifi_edit, ta_wifi_edit);
        meck_panel_edit_opened(ta_wifi_edit, kb_wifi_edit);
    }
}

static void on_settings_wifi_pass_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    g_wifi_edit_field = 1;
    if (obj_wifi_edit_panel && ta_wifi_edit) {
        lv_textarea_set_text(ta_wifi_edit, prefs->wifi_password);
        lv_textarea_set_password_mode(ta_wifi_edit, false);
        lv_obj_remove_flag(obj_wifi_edit_panel, LV_OBJ_FLAG_HIDDEN);
        if (kb_wifi_edit) lv_keyboard_set_textarea(kb_wifi_edit, ta_wifi_edit);
        meck_panel_edit_opened(ta_wifi_edit, kb_wifi_edit);
    }
}

#if MECK_BLE_KEYBOARD_ENABLED
// ============================================================================
// BLE keyboard pairing sub-screen (Settings > BLE Keyboard)
// ============================================================================

static const char* kbd_state_text() {
    if (meck_kbd_is_connected()) return "Connected";
    switch (meck_kbd_state()) {
        case 0:  return "Off";
        case 1:  return "Idle";
        case 2:  return "Searching...";
        case 3:  return "Reconnecting...";
        case 4:  return "Connecting...";
        case 5:  return "Pairing...";
        case 6:
        case 7:  return "Setting up...";
        case 8:  return "Connected";
        default: return "...";
    }
}

static void settings_kbd_update_labels() {
    if (lbl_set_kbd_toggle)
        lv_label_set_text(lbl_set_kbd_toggle, meck_kbd_is_enabled() ? "On" : "Off");
    if (lbl_set_kbd_status)
        lv_label_set_text(lbl_set_kbd_status, kbd_state_text());
}

// Tap a discovered keyboard -> persist its address and connect.
static void on_kbd_scan_pick(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    char addr[18] = {0}, name[24] = {0}; int8_t rssi = 0;
    if (!meck_kbd_get_scan(idx, addr, name, &rssi)) return;

    Meck* mesh = meck_get_instance();
    if (mesh) {
        P4NodePrefs* prefs = mesh->getNodePrefs();
        if (prefs) {
            strncpy(prefs->kbd_addr, addr, sizeof(prefs->kbd_addr) - 1);
            prefs->kbd_addr[sizeof(prefs->kbd_addr) - 1] = '\0';
            mesh->getDataStore()->savePrefs(*prefs);
        }
    }
    meck_kbd_stop_browse();
    meck_kbd_connect(addr);
    settings_kbd_update_labels();
}

static void refresh_kbd_scan_list() {
    if (!obj_kbd_scan_scroll) return;
    lv_obj_clean(obj_kbd_scan_scroll);

    int n = meck_kbd_scan_count();
    int y = 5;
    if (n == 0) {
        lv_obj_t *empty = lv_label_create(obj_kbd_scan_scroll);
        lv_label_set_text(empty, meck_kbd_state() == 2 ? "Searching..."
                                                       : "Tap Scan to search");
        lv_obj_set_style_text_color(empty, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(empty, &meck_montserrat_16, 0);
        lv_obj_set_pos(empty, 10, y);
        return;
    }
    for (int i = 0; i < n; i++) {
        char addr[18] = {0}, name[24] = {0}; int8_t rssi = 0;
        if (!meck_kbd_get_scan(i, addr, name, &rssi)) continue;

        lv_obj_t *btn = lv_button_create(obj_kbd_scan_scroll);
        lv_obj_set_size(btn, SCREEN_WIDTH - 60, 60);
        lv_obj_set_pos(btn, 0, y);
        lv_obj_set_style_bg_color(btn, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, lv_palette_main(LV_PALETTE_CYAN), 0);
        lv_obj_add_event_cb(btn, on_kbd_scan_pick, LV_EVENT_CLICKED, (void*)(intptr_t)i);

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, name[0] ? name : addr);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        meck_set_font(lbl, &meck_montserrat_18, 0);
        lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 8, -10);

        lv_obj_t *sub = lv_label_create(btn);
        lv_label_set_text_fmt(sub, "%s  (%d dBm)", addr, (int)rssi);
        lv_obj_set_style_text_color(sub, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(sub, &meck_montserrat_14, 0);
        lv_obj_align(sub, LV_ALIGN_LEFT_MID, 8, 12);

        y += 70;
    }
}

static void kbd_scan_timer_cb(lv_timer_t *t) {
    (void)t;
    refresh_kbd_scan_list();
    settings_kbd_update_labels();
}

static void on_settings_kbd_toggle_tap(lv_event_t *e) {
    (void)e;
    bool now = !meck_kbd_is_enabled();
    if (now) {
        // Mutual exclusivity: drop the WiFi companion (pref + runtime).
        Meck* mesh = meck_get_instance();
        if (mesh) {
            P4NodePrefs* prefs = mesh->getNodePrefs();
            if (prefs && prefs->wifi_enabled) {
                prefs->wifi_enabled = 0;
                mesh->getDataStore()->savePrefs(*prefs);
            }
        }
        meck_wifi_set_enabled(false);
    }
    meck_kbd_set_enabled(now);
    settings_kbd_update_labels();
    printf("Settings: BLE keyboard = %s\n", now ? "On" : "Off");
}

static void on_settings_kbd_scan_tap(lv_event_t *e) {
    if (!meck_kbd_is_enabled()) on_settings_kbd_toggle_tap(NULL);  // turn on first
    meck_kbd_start_browse();
    settings_kbd_update_labels();
}

static void goto_settings_from_keyboard(lv_event_t *e) {
    (void)e;
    meck_kbd_stop_browse();
    if (g_kbd_scan_timer) { lv_timer_del(g_kbd_scan_timer); g_kbd_scan_timer = NULL; }
    if (scr_settings) lv_screen_load(scr_settings);
}

static void goto_settings_keyboard(lv_event_t *e) {
    (void)e;
    settings_kbd_update_labels();
    refresh_kbd_scan_list();
    if (!g_kbd_scan_timer)
        g_kbd_scan_timer = lv_timer_create(kbd_scan_timer_cb, 500, NULL);
    if (scr_settings_keyboard) lv_screen_load(scr_settings_keyboard);
}

static void create_settings_keyboard_screen() {
    scr_settings_keyboard = lv_obj_create(NULL);
    lock_screen_scroll(scr_settings_keyboard);
    lv_obj_set_style_bg_color(scr_settings_keyboard, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr_settings_keyboard, LV_OPA_COVER, 0);
    screen_attach_clock_battery(scr_settings_keyboard, 1, &meck_montserrat_24, 30);
    if (lbl_screen_clock[1]) {
        lv_obj_set_style_text_align(lbl_screen_clock[1], LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(lbl_screen_clock[1], LV_ALIGN_TOP_RIGHT, -15, 55);
    }

    lv_obj_t *btn_back = lv_button_create(scr_settings_keyboard);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, goto_settings_from_keyboard, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr_settings_keyboard);
    lv_label_set_text(title, "BLE Keyboard");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_CYAN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    // Toggle + status rows sit above the (scrollable) results list. Absolute Y
    // below the 90px header, matching the WiFi screen's header offset.
    create_settings_row(scr_settings_keyboard, "Keyboard (tap to toggle)",
        &lbl_set_kbd_toggle, on_settings_kbd_toggle_tap, 95);
    create_settings_row(scr_settings_keyboard, "Status",
        &lbl_set_kbd_status, NULL, 160);

    lv_obj_t *btn_scan = lv_button_create(scr_settings_keyboard);
    lv_obj_set_size(btn_scan, SCREEN_WIDTH - 40, 55);
    lv_obj_set_pos(btn_scan, 20, 225);
    lv_obj_set_style_bg_color(btn_scan, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(btn_scan, 10, 0);
    lv_obj_t *scan_lbl = lv_label_create(btn_scan);
    lv_label_set_text(scan_lbl, "Scan for keyboards");
    lv_obj_set_style_text_color(scan_lbl, lv_color_black(), 0);
    meck_set_font(scan_lbl, &meck_montserrat_18, 0);
    lv_obj_center(scan_lbl);
    lv_obj_add_event_cb(btn_scan, on_settings_kbd_scan_tap, LV_EVENT_CLICKED, NULL);

    // Results list (scrollable).
    obj_kbd_scan_scroll = lv_obj_create(scr_settings_keyboard);
    lv_obj_set_size(obj_kbd_scan_scroll, SCREEN_WIDTH, SCREEN_HEIGHT - 295);
    lv_obj_set_pos(obj_kbd_scan_scroll, 0, 290);
    lv_obj_set_style_bg_color(obj_kbd_scan_scroll, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(obj_kbd_scan_scroll, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(obj_kbd_scan_scroll, 0, 0);
    lv_obj_set_style_pad_all(obj_kbd_scan_scroll, 10, 0);
    lv_obj_set_scroll_dir(obj_kbd_scan_scroll, LV_DIR_VER);

    settings_kbd_update_labels();
}
#endif // MECK_BLE_KEYBOARD_ENABLED

static void goto_settings_from_wifi(lv_event_t *e) {
    // Update main settings labels (BLE may have changed via mutual exclusivity)
    settings_update_labels();
    if (scr_settings) lv_screen_load(scr_settings);
}

static void goto_settings_wifi(lv_event_t *e) {
    settings_wifi_update_labels();
    if (scr_settings_wifi) lv_screen_load(scr_settings_wifi);
}

// ---- Web reader browser screen (Stage 3) -----------------------------------
// The fetch runs on meck_task; this screen posts a request via
// meck_web_request() and polls meck_web_result_ready() on an LVGL timer,
// rendering the reader-mode text once it arrives.
//
// scr_web hosts two views toggled by visibility: a landing menu
// (obj_web_home) modelled on the upstream Meck "Web Reader" home screen, and
// the reader view (status + scrolling text). The menu lists an inert IRC
// placeholder, a Web (enter URL) entry, a Search (DuckDuckGo Lite) entry, and
// Bookmarks / History sections drawn only when populated. Bookmarks and
// history persist to the SD card. Tapping a link, bookmark, history row, or a
// confirmed URL/search loads the reader; Back walks the in-page history then
// returns to the menu, and Back on the menu leaves the browser.

#define WEB_HISTORY_MAX     16        // in-page back-stack depth
#define WEB_MAX_BOOKMARKS   20
#define WEB_MAX_VISITED     30
#define WEB_URL_STORE_LEN   256
#define WEB_BOOKMARKS_FILE  "/sdcard/meshcore/web/bookmarks.txt"
#define WEB_VISITED_FILE    "/sdcard/meshcore/web/history.txt"

static char (*g_web_history)[256] = NULL;          // in-page back-stack (PSRAM)
static int  g_web_history_depth = 0;

static char (*g_web_bookmarks)[WEB_URL_STORE_LEN] = NULL;
static int  g_web_bookmark_count = 0;
static char (*g_web_visited)[WEB_URL_STORE_LEN] = NULL;  // most-recent-first (PSRAM)
static int  g_web_visited_count = 0;
static int  g_web_pending_bm_delete = -1;          // deferred long-press delete

// Forward declarations (the menu builder and its row handlers reference each
// other, and the row handlers call web_go/web_load_url defined further down).
static void web_load_url(const char *url);
static void web_go(const char *url);
static void web_home_rebuild();
static void on_web_go_tap(lv_event_t *e);
static void on_web_search_tap(lv_event_t *e);
static void on_web_bookmark_tap(lv_event_t *e);
static void on_web_bookmark_long(lv_event_t *e);
static void on_web_visited_tap(lv_event_t *e);
static const char* web_field_display(int f, int i);
static bool web_forms_equiv(int a, int b);
static int  web_unique_forms(int *out, int max);
static void web_forms_rebuild();
static void web_fill_rebuild(int f);
static void open_form_fill(int f);
static void on_web_forms_btn(lv_event_t *e);
static void on_web_form_select(lv_event_t *e);
static void on_web_form_panel_close(lv_event_t *e);
static void on_web_fill_submit(lv_event_t *e);
static void on_web_fill_close(lv_event_t *e);
static void on_fill_ta_focus(lv_event_t *e);
static void on_web_fill_kb_event(lv_event_t *e);

// ---- Bookmark & history persistence (plain newline-delimited URLs on SD) ----

static void web_ensure_dir() {
    mkdir("/sdcard/meshcore", 0755);
    mkdir("/sdcard/meshcore/web", 0755);
}

static void web_strip_eol(char *s) {
    size_t n = strlen(s);
    while (n > 0 && (s[n-1] == '\n' || s[n-1] == '\r' ||
                     s[n-1] == ' '  || s[n-1] == '\t')) {
        s[--n] = '\0';
    }
}

static void web_load_list(const char *path, char store[][WEB_URL_STORE_LEN],
                          int *count, int maxEntries) {
    *count = 0;
    if (!p4_sdcard_is_mounted()) return;
    FILE *f = fopen(path, "r");
    if (!f) return;
    char line[WEB_URL_STORE_LEN];
    while (*count < maxEntries && fgets(line, sizeof(line), f)) {
        web_strip_eol(line);
        if (line[0] == '\0') continue;
        strncpy(store[*count], line, WEB_URL_STORE_LEN - 1);
        store[*count][WEB_URL_STORE_LEN - 1] = '\0';
        (*count)++;
    }
    fclose(f);
}

static void web_save_list(const char *path, char store[][WEB_URL_STORE_LEN],
                          int count) {
    if (!p4_sdcard_is_mounted()) return;
    web_ensure_dir();
    FILE *f = fopen(path, "w");
    if (!f) return;
    for (int i = 0; i < count; i++) fprintf(f, "%s\n", store[i]);
    fclose(f);
}

// Insert url at the front of store, removing any existing copy and capping the
// list. Mirrors the upstream add-to-front / dedup / cap behaviour.
static void web_list_push_front(char store[][WEB_URL_STORE_LEN], int *count,
                                int maxEntries, const char *url) {
    if (!url || !url[0]) return;
    // Drop an existing copy.
    for (int i = 0; i < *count; i++) {
        if (strcmp(store[i], url) == 0) {
            for (int j = i; j < *count - 1; j++)
                strncpy(store[j], store[j+1], WEB_URL_STORE_LEN);
            (*count)--;
            break;
        }
    }
    // Shift down and insert at front.
    int top = (*count < maxEntries) ? *count : maxEntries - 1;
    for (int j = top; j > 0; j--)
        strncpy(store[j], store[j-1], WEB_URL_STORE_LEN);
    strncpy(store[0], url, WEB_URL_STORE_LEN - 1);
    store[0][WEB_URL_STORE_LEN - 1] = '\0';
    if (*count < maxEntries) (*count)++;
}

static void web_load_bookmarks() {
    web_load_list(WEB_BOOKMARKS_FILE, g_web_bookmarks, &g_web_bookmark_count,
                  WEB_MAX_BOOKMARKS);
}

static void web_add_bookmark(const char *url) {
    web_list_push_front(g_web_bookmarks, &g_web_bookmark_count,
                        WEB_MAX_BOOKMARKS, url);
    web_save_list(WEB_BOOKMARKS_FILE, g_web_bookmarks, g_web_bookmark_count);
}

static void web_remove_bookmark(int idx) {
    if (idx < 0 || idx >= g_web_bookmark_count) return;
    for (int j = idx; j < g_web_bookmark_count - 1; j++)
        strncpy(g_web_bookmarks[j], g_web_bookmarks[j+1], WEB_URL_STORE_LEN);
    g_web_bookmark_count--;
    web_save_list(WEB_BOOKMARKS_FILE, g_web_bookmarks, g_web_bookmark_count);
}

static void web_load_visited() {
    web_load_list(WEB_VISITED_FILE, g_web_visited, &g_web_visited_count,
                  WEB_MAX_VISITED);
}

static void web_add_visited(const char *url) {
    web_list_push_front(g_web_visited, &g_web_visited_count,
                        WEB_MAX_VISITED, url);
    web_save_list(WEB_VISITED_FILE, g_web_visited, g_web_visited_count);
}

// ---- Navigation -------------------------------------------------------------

// Kick off a fetch of url and show the "Loading" state. Hides the landing menu
// (we are entering the reader). Does not touch history; web_go() decides that.
static void web_load_url(const char *url) {
    strncpy(g_web_current_url, url, sizeof(g_web_current_url) - 1);
    g_web_current_url[sizeof(g_web_current_url) - 1] = '\0';
    if (obj_web_home)    lv_obj_add_flag(obj_web_home, LV_OBJ_FLAG_HIDDEN);
    if (lbl_web_content) lv_label_set_text(lbl_web_content, "Loading...");
    if (lbl_web_status)  lv_label_set_text(lbl_web_status, g_web_current_url);
    if (btn_web_links)   lv_obj_add_flag(btn_web_links, LV_OBJ_FLAG_HIDDEN);
    if (btn_web_forms)   lv_obj_add_flag(btn_web_forms, LV_OBJ_FLAG_HIDDEN);
    meck_web_request(g_web_current_url);
}

// Navigate to url. From the landing menu this is a fresh start (clears the
// in-page back-stack); from the reader it pushes the current page so Back
// returns to it.
static void web_go(const char *url) {
    if (!url || !url[0]) return;
    bool fromMenu = obj_web_home && !lv_obj_has_flag(obj_web_home, LV_OBJ_FLAG_HIDDEN);
    if (fromMenu) {
        g_web_history_depth = 0;
    } else if (g_web_history_depth < WEB_HISTORY_MAX) {
        strncpy(g_web_history[g_web_history_depth], g_web_current_url, 255);
        g_web_history[g_web_history_depth][255] = '\0';
        g_web_history_depth++;
    }
    web_load_url(url);
}

// Insert LVGL recolor markup for display: headings (lines the parser wrapped as
// "* ... *") in dark red, link "[N]" markers in darker royal blue. Literal '#'
// in page text is escaped to "##" so it is not taken as a recolor command.
static void web_recolor_text(const char *in, char *out, int cap) {
    const char *HEAD = "#B22222 ";   // dark red (firebrick)
    const char *LINK = "#808080 ";   // muted grey (de-emphasised marker, not a control)
    int o = 0;
    bool at_line_start = true;
    int i = 0;
    while (in[i] && o < cap - 1) {
        // Heading: a line that begins "* " and ends " *".
        if (at_line_start && in[i] == '*' && in[i + 1] == ' ') {
            int j = i + 2;
            while (in[j] && in[j] != '\n') j++;
            if (j >= i + 4 && in[j - 1] == '*' && in[j - 2] == ' ') {
                for (const char *p = HEAD; *p && o < cap - 1; p++) out[o++] = *p;
                for (int k = i + 2; k < j - 2 && o < cap - 1; k++) {
                    if (in[k] == '#') { if (o < cap - 1) out[o++] = '#'; if (o < cap - 1) out[o++] = '#'; }
                    else out[o++] = in[k];
                }
                if (o < cap - 1) out[o++] = '#';
                i = j;
                at_line_start = false;
                continue;
            }
        }
        // Link marker: "[" digits "]".
        if (in[i] == '[') {
            int j = i + 1;
            while (in[j] >= '0' && in[j] <= '9') j++;
            if (j > i + 1 && in[j] == ']') {
                for (const char *p = LINK; *p && o < cap - 1; p++) out[o++] = *p;
                for (int k = i; k <= j && o < cap - 1; k++) out[o++] = in[k];
                if (o < cap - 1) out[o++] = '#';
                i = j + 1;
                at_line_start = false;
                continue;
            }
        }
        char c = in[i];
        if (c == '#') { if (o < cap - 1) out[o++] = '#'; if (o < cap - 1) out[o++] = '#'; }
        else out[o++] = c;
        at_line_start = (c == '\n');
        i++;
    }
    out[o] = '\0';
}

static void web_poll_cb(lv_timer_t *t) {
    (void)t;
    // Deferred bookmark delete: run here, outside input-event dispatch, so we
    // never destroy a row button from inside its own long-press handler.
    if (g_web_pending_bm_delete >= 0) {
        web_remove_bookmark(g_web_pending_bm_delete);
        g_web_pending_bm_delete = -1;
        web_home_rebuild();
    }
    if (!meck_web_result_ready()) return;
    if (meck_web_ok()) {
        if (lbl_web_content) {
            if (g_web_recolor_buf) {
                web_recolor_text(meck_web_text(), g_web_recolor_buf, WEB_RECOLOR_CAP);
                lv_label_set_text(lbl_web_content, g_web_recolor_buf);
            } else {
                lv_label_set_text(lbl_web_content, meck_web_text());
            }
        }
        if (lbl_web_content && meck_web_truncated())
            lv_label_ins_text(lbl_web_content, 0, "Page too large to load fully.\n\n");
        int n = meck_web_link_count();
        if (lbl_web_status) {
            char st[80];
            snprintf(st, sizeof(st), "%s   (%d links)", g_web_current_url, n);
            lv_label_set_text(lbl_web_status, st);
        }
        if (btn_web_links) {
            if (lbl_web_links) {
                char lt[24];
                snprintf(lt, sizeof(lt), "Links (%d)", n);
                lv_label_set_text(lbl_web_links, lt);
            }
            if (n > 0) lv_obj_remove_flag(btn_web_links, LV_OBJ_FLAG_HIDDEN);
            else       lv_obj_add_flag(btn_web_links, LV_OBJ_FLAG_HIDDEN);
        }
        if (btn_web_forms) {
            int idx[16];
            int fn = web_unique_forms(idx, 16);
            if (lbl_web_forms) {
                char ft[24];
                snprintf(ft, sizeof(ft), "Forms (%d)", fn);
                lv_label_set_text(lbl_web_forms, ft);
            }
            if (fn > 0) lv_obj_remove_flag(btn_web_forms, LV_OBJ_FLAG_HIDDEN);
            else        lv_obj_add_flag(btn_web_forms, LV_OBJ_FLAG_HIDDEN);
        }
        web_add_visited(g_web_current_url);   // record the successful page
    } else {
        if (lbl_web_content)
            lv_label_set_text(lbl_web_content, "Fetch failed (see debug log).");
        if (lbl_web_status) lv_label_set_text(lbl_web_status, g_web_current_url);
        if (btn_web_links)  lv_obj_add_flag(btn_web_links, LV_OBJ_FLAG_HIDDEN);
        if (btn_web_forms)  lv_obj_add_flag(btn_web_forms, LV_OBJ_FLAG_HIDDEN);
    }
    meck_web_ack_result();
}

static void web_home_show() {
    web_home_rebuild();
    if (obj_web_home) lv_obj_remove_flag(obj_web_home, LV_OBJ_FLAG_HIDDEN);
}

static void web_navigate_back(lv_event_t *e) {
    (void)e;
    // On the landing menu, Back leaves the browser.
    if (obj_web_home && !lv_obj_has_flag(obj_web_home, LV_OBJ_FLAG_HIDDEN)) {
        if (g_web_poll_timer) { lv_timer_delete(g_web_poll_timer); g_web_poll_timer = NULL; }
        if (scr_home) lv_screen_load(scr_home);
        return;
    }
    // In the reader: walk the in-page back-stack first.
    if (g_web_history_depth > 0) {
        g_web_history_depth--;
        web_load_url(g_web_history[g_web_history_depth]);
        return;
    }
    // Back-stack empty: return to the landing menu.
    web_home_show();
}

static void on_web_link_tap(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    const char *u = meck_web_link_url(idx);
    if (!u || !u[0]) return;
    char next[256];
    strncpy(next, u, sizeof(next) - 1);
    next[sizeof(next) - 1] = '\0';
    if (obj_web_link_panel) lv_obj_add_flag(obj_web_link_panel, LV_OBJ_FLAG_HIDDEN);
    web_go(next);   // reader-mode: pushes the current page onto the back-stack
}

static void web_links_rebuild() {
    if (!obj_web_link_scroll) return;
    lv_obj_clean(obj_web_link_scroll);
    int n = meck_web_link_count();
    int y = 5;
    for (int i = 0; i < n; i++) {
        lv_obj_t *btn = lv_button_create(obj_web_link_scroll);
        lv_obj_set_size(btn, SCREEN_WIDTH - 60, 60);
        lv_obj_set_pos(btn, 10, y);
        lv_obj_set_style_bg_color(btn, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, lv_color_make(50, 50, 60), 0);
        lv_obj_add_event_cb(btn, on_web_link_tap, LV_EVENT_CLICKED, (void*)(intptr_t)i);

        const char *t = meck_web_link_text(i);
        if (!t[0]) t = meck_web_link_url(i);
        char rowtxt[96];
        snprintf(rowtxt, sizeof(rowtxt), "[%d] %s", i + 1, t);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, rowtxt);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        meck_set_font(lbl, &meck_montserrat_16, 0);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
        lv_obj_set_width(lbl, SCREEN_WIDTH - 90);
        lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 10, 0);

        y += 70;
    }
}

static void on_web_links_btn(lv_event_t *e) {
    (void)e;
    if (meck_web_link_count() <= 0) return;
    web_links_rebuild();
    if (obj_web_link_panel) lv_obj_remove_flag(obj_web_link_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_web_link_panel_close(lv_event_t *e) {
    (void)e;
    if (obj_web_link_panel) lv_obj_add_flag(obj_web_link_panel, LV_OBJ_FLAG_HIDDEN);
}

// ---- URL entry overlay ------------------------------------------------------
// A textarea + Meck keyboard, mirroring the WiFi editor. Confirm navigates to
// the typed URL.
static void on_web_url_save(lv_event_t *e) {
    (void)e;
    meck_panel_edit_closed(ta_web_url);
    if (ta_web_url) {
        const char *text = lv_textarea_get_text(ta_web_url);
        if (text && text[0]) {
            char next[256];
            strncpy(next, text, sizeof(next) - 1);
            next[sizeof(next) - 1] = '\0';
            if (obj_web_url_panel) lv_obj_add_flag(obj_web_url_panel, LV_OBJ_FLAG_HIDDEN);
            web_go(next);
            return;
        }
    }
    if (obj_web_url_panel) lv_obj_add_flag(obj_web_url_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_web_url_cancel(lv_event_t *e) {
    (void)e;
    meck_panel_edit_closed(ta_web_url);
    if (obj_web_url_panel) lv_obj_add_flag(obj_web_url_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_web_url_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY)       on_web_url_save(NULL);
    else if (code == LV_EVENT_CANCEL) on_web_url_cancel(NULL);
}

static void on_web_go_tap(lv_event_t *e) {
    (void)e;
    if (obj_web_url_panel && ta_web_url) {
        lv_textarea_set_text(ta_web_url, g_web_current_url);
        lv_obj_remove_flag(obj_web_url_panel, LV_OBJ_FLAG_HIDDEN);
        if (kb_web_url) lv_keyboard_set_textarea(kb_web_url, ta_web_url);
        meck_panel_edit_opened(ta_web_url, kb_web_url);
    }
}

// ---- Search entry overlay (DuckDuckGo Lite) ---------------------------------
static void on_web_search_save(lv_event_t *e) {
    (void)e;
    meck_panel_edit_closed(ta_web_search);
    if (ta_web_search) {
        const char *q = lv_textarea_get_text(ta_web_search);
        if (q && q[0]) {
            // URL-encode the query (spaces -> +, unreserved literal, else %XX).
            char enc[256];
            int ei = 0;
            for (int i = 0; q[i] && ei < (int)sizeof(enc) - 4; i++) {
                char ch = q[i];
                if (ch == ' ') {
                    enc[ei++] = '+';
                } else if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                           (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' ||
                           ch == '.' || ch == '~') {
                    enc[ei++] = ch;
                } else {
                    snprintf(enc + ei, 4, "%%%02X", (unsigned char)ch);
                    ei += 3;
                }
            }
            enc[ei] = '\0';
            char url[256];
            snprintf(url, sizeof(url), "https://html.duckduckgo.com/lite/?q=%s", enc);
            if (obj_web_search_panel) lv_obj_add_flag(obj_web_search_panel, LV_OBJ_FLAG_HIDDEN);
            web_go(url);
            return;
        }
    }
    if (obj_web_search_panel) lv_obj_add_flag(obj_web_search_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_web_search_cancel(lv_event_t *e) {
    (void)e;
    meck_panel_edit_closed(ta_web_search);
    if (obj_web_search_panel) lv_obj_add_flag(obj_web_search_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_web_search_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY)       on_web_search_save(NULL);
    else if (code == LV_EVENT_CANCEL) on_web_search_cancel(NULL);
}

static void on_web_search_tap(lv_event_t *e) {
    (void)e;
    if (obj_web_search_panel && ta_web_search) {
        lv_textarea_set_text(ta_web_search, "");
        lv_obj_remove_flag(obj_web_search_panel, LV_OBJ_FLAG_HIDDEN);
        if (kb_web_search) lv_keyboard_set_textarea(kb_web_search, ta_web_search);
        meck_panel_edit_opened(ta_web_search, kb_web_search);
    }
}

// ---- Web form fill (Stage 5) ------------------------------------------------
// Rebuild the fill overlay's rows for form f: one labelled, pre-filled textarea
// per visible (text/password) field. Hidden/submit/checkbox fields are not
// shown (hidden fields are still carried through by the URL builder). All
// textareas share kb_web_fill, re-pointed on focus.
static void web_fill_rebuild(int f) {
    if (!obj_web_fill_scroll) return;
    lv_obj_clean(obj_web_fill_scroll);
    g_fill_ta_count = 0;
    if (kb_web_fill) lv_keyboard_set_textarea(kb_web_fill, NULL);

    int nfields = meck_web_form_field_count(f);
    int y = 5;
    for (int i = 0; i < nfields && g_fill_ta_count < WEB_FILL_MAX_FIELDS; i++) {
        char t = meck_web_form_field_type(f, i);
        if (t != 't' && t != 'p') continue;

        const char *lbltxt = web_field_display(f, i);

        lv_obj_t *lbl = lv_label_create(obj_web_fill_scroll);
        lv_label_set_text(lbl, lbltxt);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        meck_set_font(lbl, &meck_montserrat_16, 0);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
        lv_obj_set_width(lbl, SCREEN_WIDTH - 60);
        lv_obj_set_pos(lbl, 10, y);
        y += 32;

        lv_obj_t *ta = lv_textarea_create(obj_web_fill_scroll);
        lv_obj_set_size(ta, SCREEN_WIDTH - 60, 50);
        lv_obj_set_pos(ta, 10, y);
        lv_textarea_set_one_line(ta, true);
        lv_textarea_set_max_length(ta, 127);
        if (t == 'p') lv_textarea_set_password_mode(ta, true);
        lv_textarea_set_text(ta, meck_web_form_field_value(f, i));
        lv_obj_set_style_bg_color(ta, lv_color_make(30, 30, 40), 0);
        lv_obj_set_style_text_color(ta, lv_color_white(), 0);
        meck_set_font(ta, &meck_montserrat_18, 0);
        lv_obj_set_style_border_color(ta, lv_palette_main(LV_PALETTE_CYAN), 0);
        lv_obj_add_event_cb(ta, on_fill_ta_focus, LV_EVENT_FOCUSED, NULL);
        y += 62;

        g_fill_ta[g_fill_ta_count]        = ta;
        g_fill_field_idx[g_fill_ta_count] = i;
        g_fill_ta_count++;
    }

    if (g_fill_ta_count > 0 && kb_web_fill)
        lv_keyboard_set_textarea(kb_web_fill, g_fill_ta[0]);
}

static void open_form_fill(int f) {
    if (f < 0 || f >= meck_web_form_count()) return;
    g_fill_form_idx = f;
    if (lbl_web_fill_title) {
        char tt[32];
        snprintf(tt, sizeof(tt), "Fill form %d", f + 1);
        lv_label_set_text(lbl_web_fill_title, tt);
    }
    web_fill_rebuild(f);
    if (obj_web_form_panel) lv_obj_add_flag(obj_web_form_panel, LV_OBJ_FLAG_HIDDEN);
    if (obj_web_fill_panel) lv_obj_remove_flag(obj_web_fill_panel, LV_OBJ_FLAG_HIDDEN);
    // Focus the first field; its FOCUSED handler re-points kb_web_fill.
    meck_panel_edit_opened(g_fill_ta_count > 0 ? g_fill_ta[0] : NULL, kb_web_fill);
}

static void on_fill_ta_focus(lv_event_t *e) {
    lv_obj_t *ta = (lv_obj_t *)lv_event_get_target(e);
    if (kb_web_fill && ta) lv_keyboard_set_textarea(kb_web_fill, ta);
}

static void on_web_fill_submit(lv_event_t *e) {
    (void)e;
    int f = g_fill_form_idx;
    if (f < 0) return;
    for (int k = 0; k < g_fill_ta_count; k++) {
        if (!g_fill_ta[k]) continue;
        const char *v = lv_textarea_get_text(g_fill_ta[k]);
        meck_web_form_set_field_value(f, g_fill_field_idx[k], v ? v : "");
    }
    for (int k = 0; k < g_fill_ta_count; k++) meck_panel_edit_closed(g_fill_ta[k]);
    char url[512];
    int len = meck_web_form_build_get_url(f, url, sizeof(url));
    g_fill_form_idx = -1;
    if (kb_web_fill) lv_keyboard_set_textarea(kb_web_fill, NULL);
    if (obj_web_fill_panel) lv_obj_add_flag(obj_web_fill_panel, LV_OBJ_FLAG_HIDDEN);
    // Route through web_go so the current page lands on the back-stack and the
    // reader's status/history track the submitted URL.
    if (len > 0) web_go(url);
}

static void on_web_fill_close(lv_event_t *e) {
    (void)e;
    for (int k = 0; k < g_fill_ta_count; k++) meck_panel_edit_closed(g_fill_ta[k]);
    g_fill_form_idx = -1;
    if (kb_web_fill) lv_keyboard_set_textarea(kb_web_fill, NULL);
    if (obj_web_fill_panel) lv_obj_add_flag(obj_web_fill_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_web_fill_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY)       on_web_fill_submit(NULL);
    else if (code == LV_EVENT_CANCEL) on_web_fill_close(NULL);
}

// Friendly display name for a form field: prefer the HTML <label>; otherwise
// map the common search field names to "Search"; otherwise the raw name.
static const char* web_field_display(int f, int i) {
    const char *nm = meck_web_form_field_name(f, i);
    // Map the common search field names first. The parser copies the field
    // name into label when a page has no <label>/placeholder, so checking the
    // label first would mask these (e.g. DuckDuckGo's / AO3's "q").
    if (!strcmp(nm, "q") || !strcmp(nm, "s") ||
        !strcmp(nm, "query") || !strcmp(nm, "search")) return "Search";
    const char *lbl = meck_web_form_field_label(f, i);
    if (lbl[0]) return lbl;
    return nm[0] ? nm : "(field)";
}

// Two forms are duplicates if they post to the same action and have the same
// ordered list of fillable (text/password) field names.
static bool web_forms_equiv(int a, int b) {
    if (strcmp(meck_web_form_action(a), meck_web_form_action(b)) != 0) return false;
    int fa = meck_web_form_field_count(a), fb = meck_web_form_field_count(b);
    int ia = 0, ib = 0;
    for (;;) {
        while (ia < fa) { char t = meck_web_form_field_type(a, ia); if (t == 't' || t == 'p') break; ia++; }
        while (ib < fb) { char t = meck_web_form_field_type(b, ib); if (t == 't' || t == 'p') break; ib++; }
        bool ea = (ia >= fa), eb = (ib >= fb);
        if (ea && eb) return true;
        if (ea != eb) return false;
        if (strcmp(meck_web_form_field_name(a, ia), meck_web_form_field_name(b, ib)) != 0) return false;
        ia++; ib++;
    }
}

// Build the list of forms to offer: those with at least one fillable field,
// later duplicates (see web_forms_equiv) omitted. Writes up to `max` form
// indices into `out`, returns the count. Drives the Forms button label, the
// picker rows, and the skip-the-picker-for-one-form shortcut.
static int web_unique_forms(int *out, int max) {
    int n = meck_web_form_count(), count = 0;
    for (int i = 0; i < n && count < max; i++) {
        int fc = meck_web_form_field_count(i), vis = 0;
        for (int j = 0; j < fc; j++) {
            char t = meck_web_form_field_type(i, j);
            if (t == 't' || t == 'p') { vis++; break; }
        }
        if (!vis) continue;
        bool dup = false;
        for (int k = 0; k < count; k++) if (web_forms_equiv(out[k], i)) { dup = true; break; }
        if (dup) continue;
        out[count++] = i;
    }
    return count;
}

// Rebuild the form-selection panel: one row per unique fillable form, labelled
// with the field names it will ask for.
static void web_forms_rebuild() {
    if (!obj_web_form_scroll) return;
    lv_obj_clean(obj_web_form_scroll);
    int idx[16];
    int n = web_unique_forms(idx, 16);
    int y = 5;
    for (int r = 0; r < n; r++) {
        int i = idx[r];
        int fc = meck_web_form_field_count(i);
        char fields[160];
        int fi = 0, vis = 0;
        fields[0] = '\0';
        for (int j = 0; j < fc; j++) {
            char t = meck_web_form_field_type(i, j);
            if (t != 't' && t != 'p') continue;
            const char *nm = web_field_display(i, j);
            int avail = (int)sizeof(fields) - fi;
            int w = snprintf(fields + fi, avail, "%s%s", vis ? ", " : "", nm);
            if (w < 0 || w >= avail) { fields[sizeof(fields) - 1] = '\0'; vis++; break; }
            fi += w;
            vis++;
        }

        lv_obj_t *btn = lv_button_create(obj_web_form_scroll);
        lv_obj_set_size(btn, SCREEN_WIDTH - 60, 70);
        lv_obj_set_pos(btn, 10, y);
        lv_obj_set_style_bg_color(btn, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, lv_color_make(50, 50, 60), 0);
        lv_obj_add_event_cb(btn, on_web_form_select, LV_EVENT_CLICKED, (void *)(intptr_t)i);

        char rowtxt[200];
        snprintf(rowtxt, sizeof(rowtxt), "Fill: %s", fields);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, rowtxt);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        meck_set_font(lbl, &meck_montserrat_18, 0);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
        lv_obj_set_width(lbl, SCREEN_WIDTH - 90);
        lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 10, 0);

        y += 80;
    }
}

static void on_web_form_select(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    open_form_fill(idx);
}

static void on_web_form_panel_close(lv_event_t *e) {
    (void)e;
    if (obj_web_form_panel) lv_obj_add_flag(obj_web_form_panel, LV_OBJ_FLAG_HIDDEN);
}

static void on_web_forms_btn(lv_event_t *e) {
    (void)e;
    int idx[16];
    int n = web_unique_forms(idx, 16);
    if (n <= 0) return;
    if (n == 1) { open_form_fill(idx[0]); return; }   // only one unique form
    web_forms_rebuild();
    if (obj_web_form_panel) lv_obj_remove_flag(obj_web_form_panel, LV_OBJ_FLAG_HIDDEN);
}

// ---- Landing-menu row handlers ----------------------------------------------
static void on_web_bookmark_tap(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= g_web_bookmark_count) return;
    web_go(g_web_bookmarks[idx]);
}

static void on_web_bookmark_long(lv_event_t *e) {
    // Defer the delete to the poll timer (see web_poll_cb) to avoid destroying
    // this row from inside its own event handler.
    g_web_pending_bm_delete = (int)(intptr_t)lv_event_get_user_data(e);
}

static void on_web_visited_tap(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= g_web_visited_count) return;
    web_go(g_web_visited[idx]);
}

static void web_toast_hide_cb(lv_timer_t *t) {
    (void)t;
    if (lbl_web_toast) lv_obj_add_flag(lbl_web_toast, LV_OBJ_FLAG_HIDDEN);
    if (g_web_toast_timer) { lv_timer_delete(g_web_toast_timer); g_web_toast_timer = NULL; }
}

static void web_wip_toast_hide_cb(lv_timer_t *t) {
    (void)t;
    if (lbl_web_wip_toast) lv_obj_add_flag(lbl_web_wip_toast, LV_OBJ_FLAG_HIDDEN);
    if (g_web_wip_timer) { lv_timer_delete(g_web_wip_timer); g_web_wip_timer = NULL; }
}

static void web_wip_toast_show() {
    if (!lbl_web_wip_toast) return;
    lv_obj_remove_flag(lbl_web_wip_toast, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(lbl_web_wip_toast);
    if (g_web_wip_timer) lv_timer_delete(g_web_wip_timer);
    g_web_wip_timer = lv_timer_create(web_wip_toast_hide_cb, 3500, NULL);
}

static void on_web_addbm_tap(lv_event_t *e) {
    (void)e;
    if (!g_web_current_url[0]) return;
    web_add_bookmark(g_web_current_url);
    if (lbl_web_toast) {
        lv_obj_remove_flag(lbl_web_toast, LV_OBJ_FLAG_HIDDEN);
        if (g_web_toast_timer) lv_timer_delete(g_web_toast_timer);
        g_web_toast_timer = lv_timer_create(web_toast_hide_cb, 1200, NULL);
    }
}

// ---- Landing-menu builder ---------------------------------------------------
// Tappable row in the menu scroll container. Returns the button so the caller
// can attach extra handlers (e.g. long-press delete on bookmarks).
static lv_obj_t *web_menu_row(int y, const char *text, lv_color_t txt,
                              lv_event_cb_t cb, void *ud) {
    lv_obj_t *btn = lv_button_create(obj_web_home_scroll);
    lv_obj_set_size(btn, SCREEN_WIDTH - 40, 50);
    lv_obj_set_pos(btn, 10, y);
    lv_obj_set_style_bg_color(btn, lv_color_make(25, 25, 35), 0);
    lv_obj_set_style_radius(btn, 8, 0);
    if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, txt, 0);
    meck_set_font(lbl, &meck_montserrat_18, 0);
    lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
    lv_obj_set_width(lbl, SCREEN_WIDTH - 70);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 10, 0);
    return btn;
}

static void web_menu_header(int y, const char *text) {
    lv_obj_t *lbl = lv_label_create(obj_web_home_scroll);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(lbl, &meck_montserrat_16, 0);
    lv_obj_set_pos(lbl, 15, y);
}

static void web_home_rebuild() {
    if (!obj_web_home_scroll) return;
    lv_obj_clean(obj_web_home_scroll);
    int y = 5;

    web_menu_row(y, "Web: [Enter URL]", lv_color_white(), on_web_go_tap, NULL);
    y += 60;
    web_menu_row(y, "Search: [DuckDuckGo Lite]", lv_color_white(), on_web_search_tap, NULL);
    y += 60;

    if (g_web_bookmark_count > 0) {
        web_menu_header(y, "-- Bookmarks --");
        y += 30;
        for (int i = 0; i < g_web_bookmark_count; i++) {
            lv_obj_t *row = web_menu_row(y, g_web_bookmarks[i], lv_color_white(),
                                         NULL, NULL);
            lv_obj_add_event_cb(row, on_web_bookmark_tap, LV_EVENT_SHORT_CLICKED,
                                (void*)(intptr_t)i);
            lv_obj_add_event_cb(row, on_web_bookmark_long, LV_EVENT_LONG_PRESSED,
                                (void*)(intptr_t)i);
            y += 60;
        }
    }

    if (g_web_visited_count > 0) {
        web_menu_header(y, "-- History --");
        y += 30;
        for (int i = 0; i < g_web_visited_count; i++) {
            web_menu_row(y, g_web_visited[i], lv_color_white(),
                         on_web_visited_tap, (void*)(intptr_t)i);
            y += 60;
        }
    }
}

static void cb_open_web(lv_event_t *e) {
    (void)e;
    if (!scr_web) return;
    g_web_history_depth = 0;
    g_web_pending_bm_delete = -1;
    if (g_web_toast_timer) { lv_timer_delete(g_web_toast_timer); g_web_toast_timer = NULL; }
    if (lbl_web_toast) lv_obj_add_flag(lbl_web_toast, LV_OBJ_FLAG_HIDDEN);
    web_load_bookmarks();
    web_load_visited();
    web_home_rebuild();
    if (obj_web_link_panel)   lv_obj_add_flag(obj_web_link_panel, LV_OBJ_FLAG_HIDDEN);
    if (obj_web_url_panel)    lv_obj_add_flag(obj_web_url_panel, LV_OBJ_FLAG_HIDDEN);
    if (obj_web_search_panel) lv_obj_add_flag(obj_web_search_panel, LV_OBJ_FLAG_HIDDEN);
    if (obj_web_form_panel)   lv_obj_add_flag(obj_web_form_panel, LV_OBJ_FLAG_HIDDEN);
    if (obj_web_fill_panel)   lv_obj_add_flag(obj_web_fill_panel, LV_OBJ_FLAG_HIDDEN);
    meck_panel_edit_closed(ta_web_url);
    meck_panel_edit_closed(ta_web_search);
    for (int k = 0; k < g_fill_ta_count; k++) meck_panel_edit_closed(g_fill_ta[k]);
    if (obj_web_home)         lv_obj_remove_flag(obj_web_home, LV_OBJ_FLAG_HIDDEN);
    lv_screen_load(scr_web);
    web_wip_toast_show();
    if (!g_web_poll_timer)
        g_web_poll_timer = lv_timer_create(web_poll_cb, 200, NULL);
}

static void create_web_screen() {
    // History / bookmarks / visited live in PSRAM, not internal RAM, so they
    // do not consume the scarce contiguous internal block the LVGL UI task's
    // 100 KB stack needs at boot.
    g_web_history   = (char (*)[256])              heap_caps_calloc(WEB_HISTORY_MAX,   256,               MALLOC_CAP_SPIRAM);
    g_web_bookmarks = (char (*)[WEB_URL_STORE_LEN]) heap_caps_calloc(WEB_MAX_BOOKMARKS, WEB_URL_STORE_LEN, MALLOC_CAP_SPIRAM);
    g_web_visited   = (char (*)[WEB_URL_STORE_LEN]) heap_caps_calloc(WEB_MAX_VISITED,   WEB_URL_STORE_LEN, MALLOC_CAP_SPIRAM);
    scr_web = lv_obj_create(NULL);
    lock_screen_scroll(scr_web);
    lv_obj_set_style_bg_color(scr_web, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr_web, LV_OPA_COVER, 0);
    screen_attach_clock_battery(scr_web, 1, &meck_montserrat_24, 30);
    if (lbl_screen_clock[1]) {
        lv_obj_set_style_text_align(lbl_screen_clock[1], LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(lbl_screen_clock[1], LV_ALIGN_TOP_RIGHT, -15, 55);
    }

    // ---- Reader view (page text). Covered by the landing menu when shown. ----
    lv_obj_t *btn_back = lv_button_create(scr_web);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, web_navigate_back, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr_web);
    lv_label_set_text(title, "Web\nReader");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 25);

    lbl_web_status = lv_label_create(scr_web);
    lv_label_set_text(lbl_web_status, "");
    lv_obj_set_style_text_color(lbl_web_status, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(lbl_web_status, &meck_montserrat_14, 0);
    lv_obj_align(lbl_web_status, LV_ALIGN_TOP_LEFT, 120, 100);

    lv_obj_t *scroll = lv_obj_create(scr_web);
    lv_obj_set_size(scroll, SCREEN_WIDTH, SCREEN_HEIGHT - 270);
    lv_obj_set_pos(scroll, 0, 130);
    lv_obj_set_style_bg_color(scroll, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scroll, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scroll, 0, 0);
    lv_obj_set_style_pad_all(scroll, 10, 0);
    lv_obj_set_scroll_dir(scroll, LV_DIR_VER);

    lbl_web_content = lv_label_create(scroll);
    lv_label_set_long_mode(lbl_web_content, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_web_content, SCREEN_WIDTH - 30);
    lv_obj_set_pos(lbl_web_content, 0, 0);
    lv_obj_set_style_text_color(lbl_web_content, lv_color_white(), 0);
    meck_set_font(lbl_web_content, &meck_montserrat_18, 0);
    lv_label_set_text(lbl_web_content, "Loading...");
    // Colour-code headings and link markers via the label's recolor markup.
    // Buffer lives in PSRAM; only enable recolor if it allocated, so plain text
    // still renders if it did not.
    g_web_recolor_buf = (char *)heap_caps_malloc(WEB_RECOLOR_CAP, MALLOC_CAP_SPIRAM);
    if (g_web_recolor_buf) lv_label_set_recolor(lbl_web_content, true);

    // Floating "Links (N)" button (shown after a fetch that found links).
    btn_web_links = lv_button_create(scr_web);
    lv_obj_set_size(btn_web_links, 150, 55);
    lv_obj_align(btn_web_links, LV_ALIGN_BOTTOM_RIGHT, -12, -12);
    lv_obj_set_style_bg_color(btn_web_links, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(btn_web_links, 10, 0);
    lv_obj_add_flag(btn_web_links, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(btn_web_links, on_web_links_btn, LV_EVENT_CLICKED, NULL);
    lbl_web_links = lv_label_create(btn_web_links);
    lv_label_set_text(lbl_web_links, "Links (0)");
    lv_obj_set_style_text_color(lbl_web_links, lv_color_black(), 0);
    meck_set_font(lbl_web_links, &meck_montserrat_16, 0);
    lv_obj_center(lbl_web_links);

    // "Add bookmark" button (stacked above the Links button, right side).
    btn_web_addbm = lv_button_create(scr_web);
    lv_obj_set_size(btn_web_addbm, 150, 55);
    lv_obj_align(btn_web_addbm, LV_ALIGN_BOTTOM_RIGHT, -12, -75);
    lv_obj_set_style_bg_color(btn_web_addbm, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_web_addbm, 10, 0);
    lv_obj_add_event_cb(btn_web_addbm, on_web_addbm_tap, LV_EVENT_CLICKED, NULL);
    lv_obj_t *abm_lbl = lv_label_create(btn_web_addbm);
    lv_label_set_text(abm_lbl, "+ Bookmark");
    lv_obj_set_style_text_color(abm_lbl, lv_color_white(), 0);
    meck_set_font(abm_lbl, &meck_montserrat_16, 0);
    lv_obj_center(abm_lbl);

    // Always-visible "Go to URL" button (bottom-left) on the reader.
    btn_web_go = lv_button_create(scr_web);
    lv_obj_set_size(btn_web_go, 160, 55);
    lv_obj_align(btn_web_go, LV_ALIGN_BOTTOM_LEFT, 12, -12);
    lv_obj_set_style_bg_color(btn_web_go, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(btn_web_go, 10, 0);
    lv_obj_add_event_cb(btn_web_go, on_web_go_tap, LV_EVENT_CLICKED, NULL);
    lv_obj_t *go_lbl = lv_label_create(btn_web_go);
    lv_label_set_text(go_lbl, "Go to URL");
    lv_obj_set_style_text_color(go_lbl, lv_color_black(), 0);
    meck_set_font(go_lbl, &meck_montserrat_16, 0);
    lv_obj_center(go_lbl);

    // "Forms (N)" button (bottom-left, above Go to URL; shown after a fetch
    // that found at least one form).
    btn_web_forms = lv_button_create(scr_web);
    lv_obj_set_size(btn_web_forms, 160, 55);
    lv_obj_align(btn_web_forms, LV_ALIGN_BOTTOM_LEFT, 12, -75);
    lv_obj_set_style_bg_color(btn_web_forms, lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_radius(btn_web_forms, 10, 0);
    lv_obj_add_flag(btn_web_forms, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(btn_web_forms, on_web_forms_btn, LV_EVENT_CLICKED, NULL);
    lbl_web_forms = lv_label_create(btn_web_forms);
    lv_label_set_text(lbl_web_forms, "Forms (0)");
    lv_obj_set_style_text_color(lbl_web_forms, lv_color_black(), 0);
    meck_set_font(lbl_web_forms, &meck_montserrat_16, 0);
    lv_obj_center(lbl_web_forms);

    // Transient confirmation toast (shown briefly when a bookmark is added).
    lbl_web_toast = lv_label_create(scr_web);
    lv_label_set_text(lbl_web_toast, "Bookmark added");
    lv_obj_set_style_bg_color(lbl_web_toast, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_bg_opa(lbl_web_toast, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(lbl_web_toast, lv_color_black(), 0);
    lv_obj_set_style_pad_all(lbl_web_toast, 12, 0);
    lv_obj_set_style_radius(lbl_web_toast, 10, 0);
    meck_set_font(lbl_web_toast, &meck_montserrat_18, 0);
    lv_obj_align(lbl_web_toast, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(lbl_web_toast, LV_OBJ_FLAG_HIDDEN);

    // Work-in-progress notice, shown briefly each time the web tile is opened.
    lbl_web_wip_toast = lv_label_create(scr_web);
    lv_label_set_text(lbl_web_wip_toast,
                      "The web reader is a work in progress. Some pages may not load correctly yet.");
    lv_obj_set_style_bg_color(lbl_web_wip_toast, lv_palette_main(LV_PALETTE_AMBER), 0);
    lv_obj_set_style_bg_opa(lbl_web_wip_toast, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(lbl_web_wip_toast, lv_color_black(), 0);
    lv_obj_set_style_pad_all(lbl_web_wip_toast, 12, 0);
    lv_obj_set_style_radius(lbl_web_wip_toast, 10, 0);
    meck_set_font(lbl_web_wip_toast, &meck_montserrat_18, 0);
    lv_label_set_long_mode(lbl_web_wip_toast, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_web_wip_toast, SCREEN_WIDTH - 60);
    lv_obj_align(lbl_web_wip_toast, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(lbl_web_wip_toast, LV_OBJ_FLAG_HIDDEN);

    // ---- Landing menu (created above the reader so it covers it when shown) --
    obj_web_home = lv_obj_create(scr_web);
    lv_obj_set_size(obj_web_home, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_web_home, 0, 0);
    lv_obj_set_style_bg_color(obj_web_home, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(obj_web_home, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(obj_web_home, 0, 0);
    lv_obj_add_flag(obj_web_home, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_web_home, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_web_home, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *home_back = lv_button_create(obj_web_home);
    lv_obj_set_size(home_back, 100, 70);
    lv_obj_align(home_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(home_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(home_back, 8, 0);
    lv_obj_add_event_cb(home_back, web_navigate_back, LV_EVENT_CLICKED, NULL);
    lv_obj_t *hbl = lv_label_create(home_back);
    lv_label_set_text(hbl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(hbl, lv_color_white(), 0);
    meck_set_font(hbl, &meck_montserrat_18, 0);
    lv_obj_center(hbl);

    lv_obj_t *home_title = lv_label_create(obj_web_home);
    lv_label_set_text(home_title, "Web Reader");
    lv_obj_set_style_text_color(home_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(home_title, &meck_montserrat_24, 0);
    lv_obj_align(home_title, LV_ALIGN_TOP_LEFT, 120, 25);

    obj_web_home_scroll = lv_obj_create(obj_web_home);
    lv_obj_set_size(obj_web_home_scroll, SCREEN_WIDTH, SCREEN_HEIGHT - 95);
    lv_obj_set_pos(obj_web_home_scroll, 0, 95);
    lv_obj_set_style_bg_color(obj_web_home_scroll, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(obj_web_home_scroll, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(obj_web_home_scroll, 0, 0);
    lv_obj_set_style_pad_all(obj_web_home_scroll, 10, 0);
    lv_obj_set_scroll_dir(obj_web_home_scroll, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(obj_web_home_scroll, LV_SCROLLBAR_MODE_AUTO);

    // ---- Link overlay panel (populated on demand by web_links_rebuild). ------
    obj_web_link_panel = lv_obj_create(scr_web);
    lv_obj_set_size(obj_web_link_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_web_link_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_web_link_panel, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(obj_web_link_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(obj_web_link_panel, 0, 0);
    lv_obj_add_flag(obj_web_link_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_web_link_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_web_link_panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *ov_title = lv_label_create(obj_web_link_panel);
    lv_label_set_text(ov_title, "Links");
    lv_obj_set_style_text_color(ov_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(ov_title, &meck_montserrat_24, 0);
    lv_obj_align(ov_title, LV_ALIGN_TOP_LEFT, 20, 25);

    lv_obj_t *ov_close = lv_button_create(obj_web_link_panel);
    lv_obj_set_size(ov_close, 110, 60);
    lv_obj_align(ov_close, LV_ALIGN_TOP_RIGHT, -12, 18);
    lv_obj_set_style_bg_color(ov_close, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(ov_close, 8, 0);
    lv_obj_add_event_cb(ov_close, on_web_link_panel_close, LV_EVENT_CLICKED, NULL);
    lv_obj_t *cl = lv_label_create(ov_close);
    lv_label_set_text(cl, LV_SYMBOL_CLOSE " Close");
    lv_obj_set_style_text_color(cl, lv_color_white(), 0);
    meck_set_font(cl, &meck_montserrat_16, 0);
    lv_obj_center(cl);

    obj_web_link_scroll = lv_obj_create(obj_web_link_panel);
    lv_obj_set_size(obj_web_link_scroll, SCREEN_WIDTH, SCREEN_HEIGHT - 100);
    lv_obj_set_pos(obj_web_link_scroll, 0, 100);
    lv_obj_set_style_bg_color(obj_web_link_scroll, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(obj_web_link_scroll, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(obj_web_link_scroll, 0, 0);
    lv_obj_set_style_pad_all(obj_web_link_scroll, 10, 0);
    lv_obj_set_scroll_dir(obj_web_link_scroll, LV_DIR_VER);

    // ---- URL entry overlay (dark overlay, textarea, Confirm, Meck keyboard) --
    obj_web_url_panel = lv_obj_create(scr_web);
    lv_obj_set_size(obj_web_url_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_web_url_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_web_url_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_web_url_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(obj_web_url_panel, 0, 0);
    lv_obj_add_flag(obj_web_url_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_web_url_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_web_url_panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *url_title = lv_label_create(obj_web_url_panel);
    lv_label_set_text(url_title, "Go to URL");
    lv_obj_set_style_text_color(url_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(url_title, &meck_montserrat_22, 0);
    lv_obj_align(url_title, LV_ALIGN_TOP_MID, 0, 50);

    ta_web_url = lv_textarea_create(obj_web_url_panel);
    lv_obj_set_size(ta_web_url, SCREEN_WIDTH - 40, 50);
    lv_obj_align(ta_web_url, LV_ALIGN_TOP_MID, 0, 100);
    lv_textarea_set_one_line(ta_web_url, true);
    lv_textarea_set_max_length(ta_web_url, 255);
    lv_obj_set_style_bg_color(ta_web_url, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_web_url, lv_color_white(), 0);
    meck_set_font(ta_web_url, &meck_montserrat_18, 0);
    lv_obj_set_style_border_color(ta_web_url, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_border_color(ta_web_url, lv_color_white(),     LV_PART_CURSOR);
    lv_obj_set_style_border_width(ta_web_url, 2,                     LV_PART_CURSOR);
    lv_obj_set_style_border_side(ta_web_url,  LV_BORDER_SIDE_LEFT,   LV_PART_CURSOR);
    lv_obj_set_style_border_opa(ta_web_url,   LV_OPA_COVER,          LV_PART_CURSOR);

    lv_obj_t *url_confirm = lv_button_create(obj_web_url_panel);
    lv_obj_set_size(url_confirm, SCREEN_WIDTH - 40, 50);
    lv_obj_align(url_confirm, LV_ALIGN_TOP_MID, 0, 165);
    lv_obj_set_style_bg_color(url_confirm, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(url_confirm, 8, 0);
    lv_obj_t *url_confirm_lbl = lv_label_create(url_confirm);
    lv_label_set_text(url_confirm_lbl, "Confirm");
    lv_obj_set_style_text_color(url_confirm_lbl, lv_color_black(), 0);
    meck_set_font(url_confirm_lbl, &meck_montserrat_22, 0);
    lv_obj_center(url_confirm_lbl);
    lv_obj_add_event_cb(url_confirm, on_web_url_save, LV_EVENT_CLICKED, NULL);

    kb_web_url = lv_keyboard_create(obj_web_url_panel);
    meck_style_keyboard(kb_web_url);
    lv_keyboard_set_textarea(kb_web_url, ta_web_url);
    lv_obj_add_event_cb(kb_web_url, on_web_url_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_web_url, on_web_url_kb_event, LV_EVENT_CANCEL, NULL);

    // ---- Search entry overlay (same pattern as the URL overlay) -------------
    obj_web_search_panel = lv_obj_create(scr_web);
    lv_obj_set_size(obj_web_search_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_web_search_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_web_search_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_web_search_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(obj_web_search_panel, 0, 0);
    lv_obj_add_flag(obj_web_search_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_web_search_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_web_search_panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *search_title = lv_label_create(obj_web_search_panel);
    lv_label_set_text(search_title, "Search (DuckDuckGo Lite)");
    lv_obj_set_style_text_color(search_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(search_title, &meck_montserrat_22, 0);
    lv_obj_align(search_title, LV_ALIGN_TOP_MID, 0, 50);

    ta_web_search = lv_textarea_create(obj_web_search_panel);
    lv_obj_set_size(ta_web_search, SCREEN_WIDTH - 40, 50);
    lv_obj_align(ta_web_search, LV_ALIGN_TOP_MID, 0, 100);
    lv_textarea_set_one_line(ta_web_search, true);
    lv_textarea_set_max_length(ta_web_search, 128);
    lv_obj_set_style_bg_color(ta_web_search, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_web_search, lv_color_white(), 0);
    meck_set_font(ta_web_search, &meck_montserrat_18, 0);
    lv_obj_set_style_border_color(ta_web_search, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_border_color(ta_web_search, lv_color_white(),     LV_PART_CURSOR);
    lv_obj_set_style_border_width(ta_web_search, 2,                     LV_PART_CURSOR);
    lv_obj_set_style_border_side(ta_web_search,  LV_BORDER_SIDE_LEFT,   LV_PART_CURSOR);
    lv_obj_set_style_border_opa(ta_web_search,   LV_OPA_COVER,          LV_PART_CURSOR);

    lv_obj_t *search_confirm = lv_button_create(obj_web_search_panel);
    lv_obj_set_size(search_confirm, SCREEN_WIDTH - 40, 50);
    lv_obj_align(search_confirm, LV_ALIGN_TOP_MID, 0, 165);
    lv_obj_set_style_bg_color(search_confirm, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(search_confirm, 8, 0);
    lv_obj_t *search_confirm_lbl = lv_label_create(search_confirm);
    lv_label_set_text(search_confirm_lbl, "Search");
    lv_obj_set_style_text_color(search_confirm_lbl, lv_color_black(), 0);
    meck_set_font(search_confirm_lbl, &meck_montserrat_22, 0);
    lv_obj_center(search_confirm_lbl);
    lv_obj_add_event_cb(search_confirm, on_web_search_save, LV_EVENT_CLICKED, NULL);

    kb_web_search = lv_keyboard_create(obj_web_search_panel);
    meck_style_keyboard(kb_web_search);
    lv_keyboard_set_textarea(kb_web_search, ta_web_search);
    // These were never attached (the handler existed unused), so the search
    // keyboard's OK/close -- and the hardware keyboard's Enter/Esc, which
    // replay the same events -- did nothing.
    lv_obj_add_event_cb(kb_web_search, on_web_search_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_web_search, on_web_search_kb_event, LV_EVENT_CANCEL, NULL);

    // ---- Form selection panel (shown only when a page has >1 form) ----------
    obj_web_form_panel = lv_obj_create(scr_web);
    lv_obj_set_size(obj_web_form_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_web_form_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_web_form_panel, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(obj_web_form_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(obj_web_form_panel, 0, 0);
    lv_obj_add_flag(obj_web_form_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_web_form_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_web_form_panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *fp_title = lv_label_create(obj_web_form_panel);
    lv_label_set_text(fp_title, "Forms");
    lv_obj_set_style_text_color(fp_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(fp_title, &meck_montserrat_24, 0);
    lv_obj_align(fp_title, LV_ALIGN_TOP_LEFT, 20, 25);

    lv_obj_t *fp_close = lv_button_create(obj_web_form_panel);
    lv_obj_set_size(fp_close, 110, 60);
    lv_obj_align(fp_close, LV_ALIGN_TOP_RIGHT, -12, 18);
    lv_obj_set_style_bg_color(fp_close, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(fp_close, 8, 0);
    lv_obj_add_event_cb(fp_close, on_web_form_panel_close, LV_EVENT_CLICKED, NULL);
    lv_obj_t *fp_cl = lv_label_create(fp_close);
    lv_label_set_text(fp_cl, LV_SYMBOL_CLOSE " Close");
    lv_obj_set_style_text_color(fp_cl, lv_color_white(), 0);
    meck_set_font(fp_cl, &meck_montserrat_16, 0);
    lv_obj_center(fp_cl);

    obj_web_form_scroll = lv_obj_create(obj_web_form_panel);
    lv_obj_set_size(obj_web_form_scroll, SCREEN_WIDTH, SCREEN_HEIGHT - 100);
    lv_obj_set_pos(obj_web_form_scroll, 0, 100);
    lv_obj_set_style_bg_color(obj_web_form_scroll, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(obj_web_form_scroll, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(obj_web_form_scroll, 0, 0);
    lv_obj_set_style_pad_all(obj_web_form_scroll, 10, 0);
    lv_obj_set_scroll_dir(obj_web_form_scroll, LV_DIR_VER);

    // ---- Form fill overlay (labelled textareas + shared Meck keyboard) ------
    obj_web_fill_panel = lv_obj_create(scr_web);
    lv_obj_set_size(obj_web_fill_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_web_fill_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_web_fill_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_web_fill_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(obj_web_fill_panel, 0, 0);
    lv_obj_add_flag(obj_web_fill_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_web_fill_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_web_fill_panel, LV_SCROLLBAR_MODE_OFF);

    lbl_web_fill_title = lv_label_create(obj_web_fill_panel);
    lv_label_set_text(lbl_web_fill_title, "Fill form");
    lv_obj_set_style_text_color(lbl_web_fill_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(lbl_web_fill_title, &meck_montserrat_24, 0);
    lv_obj_align(lbl_web_fill_title, LV_ALIGN_TOP_LEFT, 20, 25);

    lv_obj_t *fill_close = lv_button_create(obj_web_fill_panel);
    lv_obj_set_size(fill_close, 110, 60);
    lv_obj_align(fill_close, LV_ALIGN_TOP_RIGHT, -12, 18);
    lv_obj_set_style_bg_color(fill_close, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(fill_close, 8, 0);
    lv_obj_add_event_cb(fill_close, on_web_fill_close, LV_EVENT_CLICKED, NULL);
    lv_obj_t *fill_cl = lv_label_create(fill_close);
    lv_label_set_text(fill_cl, LV_SYMBOL_CLOSE " Close");
    lv_obj_set_style_text_color(fill_cl, lv_color_white(), 0);
    meck_set_font(fill_cl, &meck_montserrat_16, 0);
    lv_obj_center(fill_cl);

    obj_web_fill_scroll = lv_obj_create(obj_web_fill_panel);
    lv_obj_set_size(obj_web_fill_scroll, SCREEN_WIDTH, SCREEN_HEIGHT - MECK_KB_HEIGHT - 165);
    lv_obj_set_pos(obj_web_fill_scroll, 0, 95);
    lv_obj_set_style_bg_opa(obj_web_fill_scroll, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(obj_web_fill_scroll, 0, 0);
    lv_obj_set_style_pad_all(obj_web_fill_scroll, 10, 0);
    lv_obj_set_scroll_dir(obj_web_fill_scroll, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(obj_web_fill_scroll, LV_SCROLLBAR_MODE_AUTO);

    lv_obj_t *fill_submit = lv_button_create(obj_web_fill_panel);
    lv_obj_set_size(fill_submit, SCREEN_WIDTH - 40, 55);
    lv_obj_align(fill_submit, LV_ALIGN_BOTTOM_MID, 0, -(MECK_KB_HEIGHT + 8));
    lv_obj_set_style_bg_color(fill_submit, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(fill_submit, 8, 0);
    lv_obj_add_event_cb(fill_submit, on_web_fill_submit, LV_EVENT_CLICKED, NULL);
    lv_obj_t *fill_submit_lbl = lv_label_create(fill_submit);
    lv_label_set_text(fill_submit_lbl, "Submit");
    lv_obj_set_style_text_color(fill_submit_lbl, lv_color_black(), 0);
    meck_set_font(fill_submit_lbl, &meck_montserrat_22, 0);
    lv_obj_center(fill_submit_lbl);

    kb_web_fill = lv_keyboard_create(obj_web_fill_panel);
    meck_style_keyboard(kb_web_fill);
    lv_keyboard_set_textarea(kb_web_fill, NULL);
    lv_obj_add_event_cb(kb_web_fill, on_web_fill_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_web_fill, on_web_fill_kb_event, LV_EVENT_CANCEL, NULL);
}

static void create_settings_wifi_screen() {
    scr_settings_wifi = lv_obj_create(NULL);
    lock_screen_scroll(scr_settings_wifi);
    lv_obj_set_style_bg_color(scr_settings_wifi, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr_settings_wifi, LV_OPA_COVER, 0);
    screen_attach_clock_battery(scr_settings_wifi, 1, &meck_montserrat_24, 30);
    // Move clock to right side (under battery) so title doesn't overlap
    if (lbl_screen_clock[1]) {
        lv_obj_set_style_text_align(lbl_screen_clock[1], LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(lbl_screen_clock[1], LV_ALIGN_TOP_RIGHT, -15, 55);
    }

    // Back button
    lv_obj_t *btn_back = lv_button_create(scr_settings_wifi);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, goto_settings_from_wifi, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr_settings_wifi);
    lv_label_set_text(title, "WiFi Companion");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    lv_obj_t *scroll = lv_obj_create(scr_settings_wifi);
    lv_obj_set_size(scroll, SCREEN_WIDTH, SCREEN_HEIGHT - 90);
    lv_obj_set_pos(scroll, 0, 90);
    lv_obj_set_style_bg_color(scroll, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scroll, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scroll, 0, 0);
    lv_obj_set_style_pad_all(scroll, 10, 0);
    lv_obj_set_scroll_dir(scroll, LV_DIR_VER);

    int y = 5;
    create_settings_row(scroll, "WiFi (tap to toggle)",
        &lbl_set_wifi_toggle, on_settings_wifi_toggle_tap, y);
    y += 65;
    create_settings_row(scroll, "SSID (tap to edit)",
        &lbl_set_wifi_ssid, on_settings_wifi_ssid_tap, y);
    y += 65;
    create_settings_row(scroll, "Password (tap to edit)",
        &lbl_set_wifi_pass, on_settings_wifi_pass_tap, y);
    y += 65;
    create_settings_row(scroll, "IP Address",
        &lbl_set_wifi_ip, NULL, y);
    y += 65;

    // Hint text
    lv_obj_t *hint = lv_label_create(scroll);
    lv_label_set_text(hint, "Connect companion app to IP:5000\n"
                            "BLE and WiFi are mutually exclusive");
    lv_obj_set_style_text_color(hint, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(hint, &meck_montserrat_14, 0);
    lv_obj_set_pos(hint, 15, y);

    // Edit overlay panel (shared for SSID and Password) — matches the
    // Node Name edit pattern on scr_settings: dark overlay, styled
    // textarea with dark bg + cyan border, single Confirm button, and
    // the standard Meck keyboard.
    obj_wifi_edit_panel = lv_obj_create(scr_settings_wifi);
    lv_obj_set_size(obj_wifi_edit_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_wifi_edit_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_wifi_edit_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_wifi_edit_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(obj_wifi_edit_panel, 0, 0);
    lv_obj_add_flag(obj_wifi_edit_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_wifi_edit_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_wifi_edit_panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *edit_title = lv_label_create(obj_wifi_edit_panel);
    lv_label_set_text(edit_title, "Edit WiFi");
    lv_obj_set_style_text_color(edit_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(edit_title, &meck_montserrat_22, 0);
    lv_obj_align(edit_title, LV_ALIGN_TOP_MID, 0, 50);

    ta_wifi_edit = lv_textarea_create(obj_wifi_edit_panel);
    lv_obj_set_size(ta_wifi_edit, SCREEN_WIDTH - 40, 50);
    lv_obj_align(ta_wifi_edit, LV_ALIGN_TOP_MID, 0, 100);
    lv_textarea_set_one_line(ta_wifi_edit, true);
    lv_textarea_set_max_length(ta_wifi_edit, 64);
    lv_obj_set_style_bg_color(ta_wifi_edit, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_wifi_edit, lv_color_white(), 0);
    meck_set_font(ta_wifi_edit, &meck_montserrat_18, 0);
    lv_obj_set_style_border_color(ta_wifi_edit, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_border_color(ta_wifi_edit, lv_color_white(),     LV_PART_CURSOR);
    lv_obj_set_style_border_width(ta_wifi_edit, 2,                     LV_PART_CURSOR);
    lv_obj_set_style_border_side(ta_wifi_edit,  LV_BORDER_SIDE_LEFT,   LV_PART_CURSOR);
    lv_obj_set_style_border_opa(ta_wifi_edit,   LV_OPA_COVER,          LV_PART_CURSOR);

    lv_obj_t *btn_confirm = lv_button_create(obj_wifi_edit_panel);
    lv_obj_set_size(btn_confirm, SCREEN_WIDTH - 40, 50);
    lv_obj_align(btn_confirm, LV_ALIGN_TOP_MID, 0, 165);
    lv_obj_set_style_bg_color(btn_confirm, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(btn_confirm, 8, 0);
    lv_obj_t *confirm_lbl = lv_label_create(btn_confirm);
    lv_label_set_text(confirm_lbl, "Confirm");
    lv_obj_set_style_text_color(confirm_lbl, lv_color_black(), 0);
    meck_set_font(confirm_lbl, &meck_montserrat_22, 0);
    lv_obj_center(confirm_lbl);
    lv_obj_add_event_cb(btn_confirm, on_wifi_edit_save, LV_EVENT_CLICKED, NULL);

    kb_wifi_edit = lv_keyboard_create(obj_wifi_edit_panel);
    meck_style_keyboard(kb_wifi_edit);
    lv_keyboard_set_textarea(kb_wifi_edit, ta_wifi_edit);
    lv_obj_add_event_cb(kb_wifi_edit, on_wifi_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_wifi_edit, on_wifi_kb_event, LV_EVENT_CANCEL, NULL);

    settings_wifi_update_labels();
}

// Long-press anywhere on the GPS tile toggles the L76K module between
// active and standby. Persisted via prefs->gps_enabled so the choice
// survives reboot — useful for indoor / battery-saving scenarios.
static void on_gps_tile_long_press(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    bool was_on = meck_gps_is_enabled();
    bool now_on = !was_on;
    meck_gps_set_enabled(now_on);
    // Encoding: 1 = on, 2 = off. Reserves 0 as "fresh prefs / use default".
    prefs->gps_enabled = now_on ? 1 : 2;
    mesh->getDataStore()->savePrefs(*prefs);

    // Immediate UI feedback. The 500ms refresh tick will overwrite this,
    // but that gives a smooth crossfade rather than a stale label.
    if (lbl_gps_detail) {
        lv_label_set_text(lbl_gps_detail,
            now_on ? "GPS: enabling..."
                   : "GPS: OFF\nLong-press to enable");
    }
    printf("GPS: %s via long-press\n", now_on ? "ENABLED" : "DISABLED");
}

// 4 Hz idle watcher. Sleep trigger: once LVGL reports inactivity past the
// configured threshold, fade brightness to 0. Wake trigger: any input
// activity. LVGL maintains its own inactive-time counter that gets reset
// whenever any registered indev (touch, keypad) reports a press, so we
// just watch that counter going small to detect a wake. The boot button
// is also polled here as a secondary wake source — useful if the screen
// blanks and a quick button press is more deliberate than a stray touch.
//
// We do NOT disable the indevs during dim. An earlier iteration of this
// timer did, to avoid pocket-touch wakes, but that made every wake feel
// awkward because boot-button-only meant fishing the device out and
// finding a hardware button. Touch wake is the v0.1 behaviour and is
// what users expect.
//
// Timer period is 250 ms so the wake feels snappy. The body is cheap
// (one comparison against a counter) so the higher rate doesn't matter.
static void screen_idle_timer_cb(lv_timer_t *t) {
    (void)t;
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();

    // v0.3.5: full screen-off path. State lives in main.cpp (it owns the
    // panel handle and the LVGL display + indev refs). This timer just
    // decides when to transition based on inactivity / boot button, and
    // calls meck_screen_off() / meck_screen_on() to do the heavy lifting.
    //
    // Wake source in Stage 1 is the boot button only. Touch wake via the
    // XL9535 INT line is planned for a later stage; until then a deliberate
    // BOOT press is the only way out of off-state.

    // Boot button toggles the screen on demand. The 4 Hz poll debounces and
    // edge detection (the press fires once, not every tick it's held) means
    // the same press that turns the screen off can't immediately wake it on
    // the next tick. Handled before the auto-off gate so it works even when
    // auto-off is disabled (screen_off_minutes == 0).
    //
    // wifi_dimmed is the WiFi-companion "soft off" (brightness 0, no DISPOFF).
    // It lives at function scope so both the toggle and the inactivity path
    // see it, since meck_screen_is_off() only reflects the real DISPOFF state.
    static bool wifi_dimmed = false;
    static bool s_boot_was_pressed = false;
    bool boot_now = meck_boot_button_pressed();
    bool boot_edge = boot_now && !s_boot_was_pressed;
    s_boot_was_pressed = boot_now;

    if (prefs && boot_edge) {
        if (meck_screen_is_off() || wifi_dimmed) {
            // Off or WiFi-dimmed → wake.
            if (meck_screen_is_off()) meck_screen_on();
            meck_screen_set_brightness(prefs->screen_brightness ? prefs->screen_brightness : 200);
            wifi_dimmed = false;
            lv_display_trigger_activity(NULL);
            printf("Screen: boot button woke screen\n");
        } else if (meck_wifi_is_enabled()) {
            // WiFi companion active: full sleep kills the SDIO link, so blank
            // by dropping brightness to 0 rather than issuing DISPOFF.
            meck_screen_set_brightness(0);
            wifi_dimmed = true;
            printf("Screen: boot button blanked display (WiFi active)\n");
        } else {
            printf("Screen: boot button turned screen off\n");
            meck_screen_off();
        }
        return;
    }

    // Auto-off disabled — if the screen happens to be off (because the
    // setting changed mid-sleep), wake it back up. Otherwise nothing to do.
    if (!prefs || prefs->screen_off_minutes == 0) {
        if (meck_screen_is_off()) {
            meck_screen_on();
            printf("Screen: auto-off disabled mid-sleep, screen restored\n");
        }
        return;
    }

    uint32_t threshold_ms = (uint32_t)prefs->screen_off_minutes * 60u * 1000u;
    uint32_t inactive_ms = lv_display_get_inactive_time(NULL);

    if (!meck_screen_is_off()) {
        // Sleep path: inactivity-based.
        if (inactive_ms >= threshold_ms) {
            // When WiFi companion is active, entering light sleep kills
            // the SDIO bus (and thus the TCP connection). Instead of full
            // sleep, just blank the display by setting brightness to 0.
            if (meck_wifi_is_enabled()) {
                if (!wifi_dimmed) {
                    meck_screen_set_brightness(0);
                    wifi_dimmed = true;
                    printf("Screen: WiFi active, dimming display (no sleep)\n");
                }
            } else {
                printf("Screen: entering off-state after %u min idle\n",
                       (unsigned)prefs->screen_off_minutes);
                meck_screen_off();
            }
        } else {
            if (wifi_dimmed) {
                // User touched screen — restore brightness
                meck_screen_set_brightness(prefs->screen_brightness);
                wifi_dimmed = false;
            }
        }
    }
}

// Back button on the sub-screen returns to main settings, not home.
static void goto_settings_from_contacts(lv_event_t *e) {
    settings_update_labels();
    if (scr_settings) lv_screen_load(scr_settings);
}

static void goto_settings_contacts(lv_event_t *e) {
    settings_contacts_update_labels();
    if (scr_settings_contacts) lv_screen_load(scr_settings_contacts);
}

static void create_settings_contacts_screen() {
    scr_settings_contacts = lv_obj_create(NULL);
    lock_screen_scroll(scr_settings_contacts);
    lv_obj_set_style_bg_color(scr_settings_contacts, lv_color_black(), 0);
    screen_attach_clock_battery(scr_settings_contacts, 1, &meck_montserrat_24, 30);

    // Custom back button → main Settings (not home).
    lv_obj_t *btn_back = lv_button_create(scr_settings_contacts);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, goto_settings_from_contacts,
                        LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr_settings_contacts);
    lv_label_set_text(title, "Contacts");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    lv_obj_t *scroll = lv_obj_create(scr_settings_contacts);
    lv_obj_set_size(scroll, SCREEN_WIDTH, SCREEN_HEIGHT - 90);
    lv_obj_set_pos(scroll, 0, 90);
    lv_obj_set_style_bg_color(scroll, lv_color_black(), 0);
    lv_obj_set_style_border_width(scroll, 0, 0);
    lv_obj_set_style_pad_all(scroll, 10, 0);
    lv_obj_set_scroll_dir(scroll, LV_DIR_VER);

    int y = 5;

    // Mode picker — taps cycle Auto All → Custom → Manual Only → Auto All...
    create_settings_row(scroll, "Mode (tap to cycle)",
        &lbl_set_contact_mode, on_settings_contact_mode_tap, y);
    y += 65;

    // Per-type auto-add toggles. These only take effect in Custom mode but
    // remain editable in any mode so the selection is preserved.
    create_settings_row(scroll, "Auto-add Chat",
        &lbl_set_autoadd_chat, on_settings_autoadd_chat_tap, y);
    y += 65;
    create_settings_row(scroll, "Auto-add Repeater",
        &lbl_set_autoadd_repeater, on_settings_autoadd_repeater_tap, y);
    y += 65;
    create_settings_row(scroll, "Auto-add Room Server",
        &lbl_set_autoadd_room, on_settings_autoadd_room_tap, y);
    y += 65;
    create_settings_row(scroll, "Auto-add Sensor",
        &lbl_set_autoadd_sensor, on_settings_autoadd_sensor_tap, y);
    y += 65;

    // Overwrite policy — orthogonal to mode. When the contact table is full,
    // controls whether new contacts displace the oldest non-favourited row
    // or are simply dropped.
    create_settings_row(scroll, "Overwrite oldest when full",
        &lbl_set_autoadd_overwrite, on_settings_autoadd_overwrite_tap, y);
    y += 65;
}

// ============================================================================
// Settings screen
// ============================================================================

static void on_settings_drafts_tap(lv_event_t *e) {
    LV_UNUSED(e);
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    prefs->save_drafts = (prefs->save_drafts != 0) ? 0 : 1;
    mesh->getDataStore()->savePrefs(*prefs);

    if (lbl_set_drafts) {
        lv_label_set_text(lbl_set_drafts,
            prefs->save_drafts != 0 ? "On" : "Off");
    }
    printf("Settings: save drafts = %s\n",
           prefs->save_drafts != 0 ? "On" : "Off");
}

static void on_settings_antenna_tap(lv_event_t *e) {
    LV_UNUSED(e);
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    prefs->antenna = (prefs->antenna != 0) ? 0 : 1;
    mesh->getDataStore()->savePrefs(*prefs);
    meck_set_antenna(prefs->antenna);

    if (lbl_set_antenna) {
        lv_label_set_text(lbl_set_antenna,
            prefs->antenna != 0 ? "External (RF2)" : "Internal (RF1)");
    }
    printf("Settings: antenna = %s\n",
           prefs->antenna != 0 ? "External (RF2)" : "Internal (RF1)");
}

static void on_settings_orientation_tap(lv_event_t *e) {
    LV_UNUSED(e);
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    prefs->orientation = (prefs->orientation != 0) ? 0 : 1;
    mesh->getDataStore()->savePrefs(*prefs);

    // Apply live. Defer the rebuild to after this click event completes (via
    // lv_async_call) so we don't delete scr_settings from inside its own
    // handler. The async callback tears down every screen, switches the
    // rotation, and rebuilds at the new orientation.
    printf("Settings: orientation = %s — rebuilding live\n",
           prefs->orientation != 0 ? "Landscape" : "Portrait");
    lv_async_call(meck_ui_apply_orientation_rebuild, NULL);
}

static void create_settings_screen() {
    scr_settings = lv_obj_create(NULL);
    lock_screen_scroll(scr_settings);
    lv_obj_set_style_bg_color(scr_settings, lv_color_black(), 0);
    screen_attach_clock_battery(scr_settings, 0, &meck_montserrat_24, 30);

    create_back_button(scr_settings);

    lv_obj_t *title = lv_label_create(scr_settings);
    lv_label_set_text(title, "Settings");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    lv_obj_t *scroll = lv_obj_create(scr_settings);
    lv_obj_set_size(scroll, SCREEN_WIDTH, SCREEN_HEIGHT - 90);
    lv_obj_set_pos(scroll, 0, 90);
    lv_obj_set_style_bg_opa(scroll, 0, 0);
    lv_obj_set_style_border_width(scroll, 0, 0);
    lv_obj_set_style_pad_all(scroll, 0, 0);
    lv_obj_set_scroll_dir(scroll, LV_DIR_VER);

    int y = 5;

    // Antenna selection (tap to toggle). 0 = Internal (RF1, VCTL HIGH),
    // 1 = External (RF2, VCTL LOW). Persisted in prefs and applied live.
    create_settings_row(scroll, "Antenna (tap to toggle)",
        &lbl_set_antenna, on_settings_antenna_tap, y);
    y += 65;

#if MECK_BLE_ENABLED
    create_settings_row(scroll, "BLE Companion (tap to toggle)",
        &lbl_set_ble, on_settings_ble_tap, y);
    y += 65;
#endif

    // WiFi Companion navigation row → opens sub-screen with SSID/password
    lv_obj_t *wifi_value_lbl = NULL;
    create_settings_row(scroll, "WiFi Companion",
        &wifi_value_lbl, goto_settings_wifi, y);
    if (wifi_value_lbl) {
        lv_label_set_text(wifi_value_lbl, LV_SYMBOL_RIGHT);
        lv_obj_set_style_text_color(wifi_value_lbl,
            lv_palette_main(LV_PALETTE_GREY), 0);
    }
    y += 65;

    // BLE Keyboard navigation row → opens the pairing sub-screen.
    // Hidden from users: the row is skipped at RUNTIME, not compiled out. The
    // keyboard code stays compiled in (MECK_BLE_KEYBOARD_ENABLED=1) so the
    // binary layout matches the build that boots — compiling it out shifts
    // memory and trips an early-boot watchdog loop. show_ble_kbd_row is
    // volatile so the compiler/linker can't dead-strip the row code (and the
    // functions it references), which would shift the layout the same way.
#if MECK_BLE_KEYBOARD_ENABLED
    volatile bool show_ble_kbd_row = false;
    if (show_ble_kbd_row) {
        lv_obj_t *kbd_value_lbl = NULL;
        create_settings_row(scroll, "BLE Keyboard",
            &kbd_value_lbl, goto_settings_keyboard, y);
        if (kbd_value_lbl) {
            lv_label_set_text(kbd_value_lbl, LV_SYMBOL_RIGHT);
            lv_obj_set_style_text_color(kbd_value_lbl,
                lv_palette_main(LV_PALETTE_GREY), 0);
        }
        y += 65;
    }
#endif

    create_settings_row(scroll, "Node Name",                 &lbl_set_name,    on_settings_name_tap,    y);
    y += 65;
    create_settings_row(scroll, "Frequency (tap to edit)",   &lbl_set_freq,    on_settings_freq_tap,    y);
    y += 65;
    create_settings_row(scroll, "Bandwidth (tap to edit)",   &lbl_set_bw,      on_settings_bw_tap,      y);
    y += 65;
    create_settings_row(scroll, "Spreading Factor (tap to edit)", &lbl_set_sf, on_settings_sf_tap,      y);
    y += 65;
    create_settings_row(scroll, "Coding Rate (tap to cycle)",&lbl_set_cr,      on_settings_cr_tap,      y);
    y += 65;
    create_settings_row(scroll, "Radio Preset",              &lbl_set_radio,   on_settings_radio_tap,   y);
    y += 65;
    create_settings_row(scroll, "TX Power (tap to cycle)",   &lbl_set_txpower, on_settings_txpower_tap, y);
    y += 65;
    create_settings_row(scroll, "Path Hash (tap to cycle)", &lbl_set_pathhash, on_settings_pathhash_tap, y);
    y += 65;
    create_settings_row(scroll, "Default Region (tap to edit)", &lbl_set_region, on_settings_region_tap, y);
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

    // Contacts navigation row → opens the Contacts sub-screen with auto-add
    // controls. Uses a chevron in the value slot so it reads as a folder
    // entry rather than a cycle/toggle.
    lv_obj_t *contacts_value_lbl = NULL;
    create_settings_row(scroll, "Contacts",
        &contacts_value_lbl, goto_settings_contacts, y);
    if (contacts_value_lbl) {
        lv_label_set_text(contacts_value_lbl, LV_SYMBOL_RIGHT);
        lv_obj_set_style_text_color(contacts_value_lbl,
            lv_palette_main(LV_PALETTE_GREY), 0);
    }
    y += 65;

    // Channels navigation row → opens the Channels sub-screen for
    // per-channel region scope and notification preferences.
    lv_obj_t *channels_value_lbl = NULL;
    create_settings_row(scroll, "Channels",
        &channels_value_lbl, goto_settings_channels, y);
    if (channels_value_lbl) {
        lv_label_set_text(channels_value_lbl, LV_SYMBOL_RIGHT);
        lv_obj_set_style_text_color(channels_value_lbl,
            lv_palette_main(LV_PALETTE_GREY), 0);
    }
    y += 65;

    // Position navigation row -> opens the Position sub-screen for
    // lat/lon entry, position mode, and Copy Position.
    lv_obj_t *position_value_lbl = NULL;
    create_settings_row(scroll, "Position",
        &position_value_lbl, goto_settings_position, y);
    if (position_value_lbl) {
        lv_label_set_text(position_value_lbl, LV_SYMBOL_RIGHT);
        lv_obj_set_style_text_color(position_value_lbl,
            lv_palette_main(LV_PALETTE_GREY), 0);
    }
    y += 65;

    // Save Draft Messages: keep an unsent channel message in memory and
    // restore it when you re-open that channel (cleared on send). In RAM
    // only, not persisted across reboot.
    create_settings_row(scroll, "Save Draft Messages",
        &lbl_set_drafts, on_settings_drafts_tap, y);
    y += 65;

    // Backup to SD — manual force-write of every NVS blob to the SD card.
    // Safety net in case an automatic SD write was missed (card unmounted,
    // write error, etc.). Tap shows a transient OK / FAIL count.
    lv_obj_t *backup_value_lbl = NULL;
    create_settings_row(scroll, "Backup to SD",
        &backup_value_lbl, on_settings_backup_to_sd_tap, y);
    if (backup_value_lbl) {
        lv_label_set_text(backup_value_lbl, "Tap");
        lv_obj_set_style_text_color(backup_value_lbl,
            lv_palette_main(LV_PALETTE_GREY), 0);
    }
    lbl_set_backup_status = backup_value_lbl;
    y += 65;

    // Export Config — write current state to SD as MeshCore-app-compatible
    // JSON. Opens a modal with four section checkboxes. Reuses
    // lbl_set_backup_status above for its result line (see
    // on_export_modal_export for the trade-off).
    lv_obj_t *export_value_lbl = NULL;
    create_settings_row(scroll, "Export Config",
        &export_value_lbl, on_settings_export_config_tap, y);
    if (export_value_lbl) {
        lv_label_set_text(export_value_lbl, "Tap");
        lv_obj_set_style_text_color(export_value_lbl,
            lv_palette_main(LV_PALETTE_GREY), 0);
    }
    y += 65;

    // Debug Logs — opens scr_debug_logs subpage for Start/Stop/Export of
    // process-wide stdout capture to SD. Value label mirrors capture state
    // ("Idle" / "Recording" / "Stopped") so it's visible without opening
    // the subpage.
    create_settings_row(scroll, "Debug Logs",
        &lbl_set_debug_logs, on_settings_debug_logs_tap, y);
    if (lbl_set_debug_logs) {
        lv_label_set_text(lbl_set_debug_logs, "Idle");
        lv_obj_set_style_text_color(lbl_set_debug_logs,
            lv_palette_main(LV_PALETTE_GREY), 0);
    }
    y += 65;

    // Screen brightness — slider from MECK_BRIGHTNESS_MIN_PCT to
    // MECK_BRIGHTNESS_MAX_PCT in 1% steps. Live-applies to the panel while
    // dragging; persists on release.
    {
        lv_obj_t *row = lv_obj_create(scroll);
        lv_obj_set_size(row, SCREEN_WIDTH - 40, 55);
        lv_obj_set_pos(row, 20, y);
        lv_obj_set_style_bg_color(row, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(row, 10, 0);
        lv_obj_set_style_border_width(row, 1, 0);
        lv_obj_set_style_border_color(row, lv_color_make(50, 50, 60), 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *title_lbl = lv_label_create(row);
        lv_label_set_text(title_lbl, "Brightness");
        lv_obj_set_style_text_color(title_lbl, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(title_lbl, &meck_montserrat_14, 0);
        lv_obj_align(title_lbl, LV_ALIGN_LEFT_MID, 10, -12);

        lbl_set_brightness_pct = lv_label_create(row);
        lv_label_set_text(lbl_set_brightness_pct, "...");
        lv_obj_set_style_text_color(lbl_set_brightness_pct, lv_color_white(), 0);
        meck_set_font(lbl_set_brightness_pct, &meck_montserrat_14, 0);
        lv_obj_align(lbl_set_brightness_pct, LV_ALIGN_RIGHT_MID, -10, -12);

        slider_set_brightness = lv_slider_create(row);
        lv_obj_set_size(slider_set_brightness, SCREEN_WIDTH - 60, 8);
        lv_slider_set_range(slider_set_brightness,
            MECK_BRIGHTNESS_MIN_PCT, MECK_BRIGHTNESS_MAX_PCT);
        lv_obj_align(slider_set_brightness, LV_ALIGN_BOTTOM_MID, 0, -8);
        // Darker cyan accent — LV_PALETTE_CYAN reads light blue on the
        // AMOLED panel, lv_palette_darken brings it closer to the teal
        // shade used for the Identity hex string.
        lv_obj_set_style_bg_color(slider_set_brightness,
            lv_palette_darken(LV_PALETTE_CYAN, 2), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(slider_set_brightness,
            lv_palette_darken(LV_PALETTE_CYAN, 2), LV_PART_KNOB);
        lv_obj_add_event_cb(slider_set_brightness,
            on_settings_brightness_slider_event, LV_EVENT_VALUE_CHANGED, NULL);
        lv_obj_add_event_cb(slider_set_brightness,
            on_settings_brightness_slider_event, LV_EVENT_RELEASED, NULL);
    }
    y += 65;

#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
    // Keyboard backlight brightness -- percent of full drive on the SY7200A
    // enable pin, applied as LEDC duty. 5-100% in 1% steps; 25% is the
    // previous fixed level (~398 mA measured with the backlight on), 100%
    // approaches full drive (~+1 A). Live-applies while dragging -- visible
    // when the backlight is on (LilyGo key toggles it) -- persists on
    // release.
    {
        lv_obj_t *row = lv_obj_create(scroll);
        lv_obj_set_size(row, SCREEN_WIDTH - 40, 55);
        lv_obj_set_pos(row, 20, y);
        lv_obj_set_style_bg_color(row, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(row, 10, 0);
        lv_obj_set_style_border_width(row, 1, 0);
        lv_obj_set_style_border_color(row, lv_color_make(50, 50, 60), 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *title_lbl = lv_label_create(row);
        lv_label_set_text(title_lbl, "Keyboard Backlight");
        lv_obj_set_style_text_color(title_lbl, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(title_lbl, &meck_montserrat_14, 0);
        lv_obj_align(title_lbl, LV_ALIGN_LEFT_MID, 10, -12);

        lbl_set_kb_backlight_pct = lv_label_create(row);
        lv_label_set_text(lbl_set_kb_backlight_pct, "...");
        lv_obj_set_style_text_color(lbl_set_kb_backlight_pct, lv_color_white(), 0);
        meck_set_font(lbl_set_kb_backlight_pct, &meck_montserrat_14, 0);
        lv_obj_align(lbl_set_kb_backlight_pct, LV_ALIGN_RIGHT_MID, -10, -12);

        slider_set_kb_backlight = lv_slider_create(row);
        lv_obj_set_size(slider_set_kb_backlight, SCREEN_WIDTH - 60, 8);
        lv_slider_set_range(slider_set_kb_backlight, 5, 100);
        lv_obj_align(slider_set_kb_backlight, LV_ALIGN_BOTTOM_MID, 0, -8);
        lv_obj_set_style_bg_color(slider_set_kb_backlight,
            lv_palette_darken(LV_PALETTE_CYAN, 2), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(slider_set_kb_backlight,
            lv_palette_darken(LV_PALETTE_CYAN, 2), LV_PART_KNOB);
        lv_obj_add_event_cb(slider_set_kb_backlight,
            on_settings_kb_backlight_slider_event, LV_EVENT_VALUE_CHANGED, NULL);
        lv_obj_add_event_cb(slider_set_kb_backlight,
            on_settings_kb_backlight_slider_event, LV_EVENT_RELEASED, NULL);
    }
    y += 65;
#endif

    // Auto screen-off — six-step ladder (Never / 1 / 2 / 5 / 10 / 30 min).
    // The 4 Hz screen_idle_timer_cb dims to 0 once inactive_ms exceeds the
    // threshold; only a boot-button press wakes the screen back to the
    // chosen brightness. Touch is suppressed while dimmed so a pocket
    // touch can't accidentally wake the device.
    create_settings_row(scroll, "Auto Off (tap to cycle)",
        &lbl_set_screen_off, on_settings_screen_off_tap, y);
    y += 65;

    // Keyboard theme — Dark / Light toggle. Default is Dark; matches the
    // rest of the Meck UI. Applies live via meck_kb_restyle_all() so the
    // next keyboard opened shows the chosen theme without a reboot.
    create_settings_row(scroll, "Keyboard Theme (tap to cycle)",
        &lbl_set_kb_theme, on_settings_kb_theme_tap, y);
    y += 65;

    // Keyboard layout — QWERTY / AZERTY / QWERTZ. Tap cycles; AZERTY uses
    // the French M-on-row-2 arrangement, QWERTZ is QWERTY with Y↔Z. The
    // symbols / numbers page (1#) is shared across all three layouts.
    create_settings_row(scroll, "Keyboard Layout (tap to cycle)",
        &lbl_set_kb_layout, on_settings_kb_layout_tap, y);
    y += 65;

    // Font size — Classic / Larger. Tap cycles. Walks the font registry and
    // retroactively rescales every label so the change is visible without a
    // reboot or screen rebuild.
    create_settings_row(scroll, "Font Size (tap to cycle)",
        &lbl_set_font_scale, on_settings_font_scale_tap, y);
    y += 65;

    // Screen orientation — Portrait / Landscape. The layout is baked into
    // every screen at build time, so this tears down and rebuilds all screens
    // live at the new orientation (see on_settings_orientation_tap).
    create_settings_row(scroll, "Orientation (tap to flip)",
        &lbl_set_orientation, on_settings_orientation_tap, y);
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
    meck_set_font(id_title, &meck_montserrat_14, 0);
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
    meck_set_font(lbl_set_identity, &meck_montserrat_14, 0);
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
    meck_set_font(fw_title_lbl, &meck_montserrat_14, 0);
    lv_obj_align(fw_title_lbl, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *fw_value_lbl = lv_label_create(fw_row);
    lv_label_set_text(fw_value_lbl, MECK_FIRMWARE_NAME " v" MECK_FIRMWARE_VERSION);
    lv_obj_set_style_text_color(fw_value_lbl, lv_color_white(), 0);
    meck_set_font(fw_value_lbl, &meck_montserrat_16, 0);
    lv_obj_align(fw_value_lbl, LV_ALIGN_RIGHT_MID, 0, 0);

    // Name edit overlay
    obj_name_edit_panel = lv_obj_create(scr_settings);
    lv_obj_set_size(obj_name_edit_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_name_edit_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_name_edit_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_name_edit_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(obj_name_edit_panel, 0, 0);
    lv_obj_add_flag(obj_name_edit_panel, LV_OBJ_FLAG_HIDDEN);
    // Lock the panel down — no scrolling, no scrollbars. Otherwise edge
    // taps near the keyboard's left/right corners pan the whole screen.
    lv_obj_clear_flag(obj_name_edit_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_name_edit_panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *edit_title = lv_label_create(obj_name_edit_panel);
    lv_label_set_text(edit_title, "Edit Node Name");
    lv_obj_set_style_text_color(edit_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(edit_title, &meck_montserrat_22, 0);
    lv_obj_align(edit_title, LV_ALIGN_TOP_MID, 0, 50);

    ta_settings_name = lv_textarea_create(obj_name_edit_panel);
    lv_obj_set_size(ta_settings_name, SCREEN_WIDTH - 40, 50);
    lv_obj_align(ta_settings_name, LV_ALIGN_TOP_MID, 0, 100);
    lv_textarea_set_one_line(ta_settings_name, true);
    lv_textarea_set_max_length(ta_settings_name, 30);
    lv_obj_set_style_bg_color(ta_settings_name, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_settings_name, lv_color_white(), 0);
    meck_set_font(ta_settings_name, &meck_montserrat_18, 0);
    lv_obj_set_style_border_color(ta_settings_name, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_border_color(ta_settings_name, lv_color_white(),     LV_PART_CURSOR);
    lv_obj_set_style_border_width(ta_settings_name, 2,                     LV_PART_CURSOR);
    lv_obj_set_style_border_side(ta_settings_name,  LV_BORDER_SIDE_LEFT,   LV_PART_CURSOR);
    lv_obj_set_style_border_opa(ta_settings_name,   LV_OPA_COVER,          LV_PART_CURSOR);

    lv_obj_t *btn_name_confirm = lv_button_create(obj_name_edit_panel);
    lv_obj_set_size(btn_name_confirm, SCREEN_WIDTH - 40, 50);
    lv_obj_align(btn_name_confirm, LV_ALIGN_TOP_MID, 0, 165);
    lv_obj_set_style_bg_color(btn_name_confirm, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(btn_name_confirm, 8, 0);
    lv_obj_t *name_confirm_lbl = lv_label_create(btn_name_confirm);
    lv_label_set_text(name_confirm_lbl, "Confirm");
    lv_obj_set_style_text_color(name_confirm_lbl, lv_color_black(), 0);
    meck_set_font(name_confirm_lbl, &meck_montserrat_22, 0);
    lv_obj_center(name_confirm_lbl);
    lv_obj_add_event_cb(btn_name_confirm, on_settings_name_save, LV_EVENT_CLICKED, NULL);

    kb_settings = lv_keyboard_create(obj_name_edit_panel);
    meck_style_keyboard(kb_settings);
    lv_keyboard_set_textarea(kb_settings, ta_settings_name);
    lv_obj_add_event_cb(kb_settings, on_settings_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_settings, on_settings_kb_event, LV_EVENT_CANCEL, NULL);
    lv_obj_add_event_cb(kb_settings, on_kb_long_press,
                        LV_EVENT_LONG_PRESSED, NULL);

    // ---- Numeric edit overlay (Frequency / Spreading Factor) ----
    obj_num_edit_panel = lv_obj_create(scr_settings);
    lv_obj_set_size(obj_num_edit_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_num_edit_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_num_edit_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_num_edit_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(obj_num_edit_panel, 0, 0);
    lv_obj_add_flag(obj_num_edit_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_num_edit_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_num_edit_panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *num_edit_title = lv_label_create(obj_num_edit_panel);
    lv_label_set_text(num_edit_title, "Edit Value");
    lv_obj_set_style_text_color(num_edit_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(num_edit_title, &meck_montserrat_22, 0);
    lv_obj_align(num_edit_title, LV_ALIGN_TOP_MID, 0, 50);

    ta_num_edit = lv_textarea_create(obj_num_edit_panel);
    lv_obj_set_size(ta_num_edit, SCREEN_WIDTH - 40, 50);
    lv_obj_align(ta_num_edit, LV_ALIGN_TOP_MID, 0, 100);
    lv_textarea_set_one_line(ta_num_edit, true);
    lv_textarea_set_max_length(ta_num_edit, 12);
    lv_obj_set_style_bg_color(ta_num_edit, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_num_edit, lv_color_white(), 0);
    meck_set_font(ta_num_edit, &meck_montserrat_18, 0);
    lv_obj_set_style_border_color(ta_num_edit, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_border_color(ta_num_edit, lv_color_white(),     LV_PART_CURSOR);
    lv_obj_set_style_border_width(ta_num_edit, 2,                     LV_PART_CURSOR);
    lv_obj_set_style_border_side(ta_num_edit,  LV_BORDER_SIDE_LEFT,   LV_PART_CURSOR);
    lv_obj_set_style_border_opa(ta_num_edit,   LV_OPA_COVER,          LV_PART_CURSOR);

    lv_obj_t *btn_num_confirm = lv_button_create(obj_num_edit_panel);
    lv_obj_set_size(btn_num_confirm, SCREEN_WIDTH - 40, 50);
    lv_obj_align(btn_num_confirm, LV_ALIGN_TOP_MID, 0, 165);
    lv_obj_set_style_bg_color(btn_num_confirm, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(btn_num_confirm, 8, 0);
    lv_obj_t *num_confirm_lbl = lv_label_create(btn_num_confirm);
    lv_label_set_text(num_confirm_lbl, "Confirm");
    lv_obj_set_style_text_color(num_confirm_lbl, lv_color_black(), 0);
    meck_set_font(num_confirm_lbl, &meck_montserrat_22, 0);
    lv_obj_center(num_confirm_lbl);
    lv_obj_add_event_cb(btn_num_confirm, on_num_edit_save, LV_EVENT_CLICKED, NULL);

    kb_num_edit = lv_keyboard_create(obj_num_edit_panel);
    meck_style_keyboard(kb_num_edit);
    lv_keyboard_set_textarea(kb_num_edit, ta_num_edit);
    lv_obj_add_event_cb(kb_num_edit, on_num_edit_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_num_edit, on_num_edit_kb_event, LV_EVENT_CANCEL, NULL);

    // ---- Default Region edit overlay ----
    obj_region_edit_panel = lv_obj_create(scr_settings);
    lv_obj_set_size(obj_region_edit_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_region_edit_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_region_edit_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_region_edit_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(obj_region_edit_panel, 0, 0);
    lv_obj_add_flag(obj_region_edit_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_region_edit_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_region_edit_panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *region_edit_title = lv_label_create(obj_region_edit_panel);
    lv_label_set_text(region_edit_title, "Default Region");
    lv_obj_set_style_text_color(region_edit_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(region_edit_title, &meck_montserrat_22, 0);
    lv_obj_align(region_edit_title, LV_ALIGN_TOP_MID, 0, 50);

    lv_obj_t *region_hint = lv_label_create(obj_region_edit_panel);
    lv_label_set_text(region_hint, "e.g. au-nsw (empty = unscoped)");
    lv_obj_set_style_text_color(region_hint, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(region_hint, &meck_montserrat_14, 0);
    lv_obj_align(region_hint, LV_ALIGN_TOP_MID, 0, 90);

    ta_region_edit = lv_textarea_create(obj_region_edit_panel);
    lv_obj_set_size(ta_region_edit, SCREEN_WIDTH - 40, 50);
    lv_obj_align(ta_region_edit, LV_ALIGN_TOP_MID, 0, 120);
    lv_textarea_set_one_line(ta_region_edit, true);
    lv_textarea_set_max_length(ta_region_edit, 30);
    lv_obj_set_style_bg_color(ta_region_edit, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_region_edit, lv_color_white(), 0);
    meck_set_font(ta_region_edit, &meck_montserrat_18, 0);
    lv_obj_set_style_border_color(ta_region_edit, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_border_color(ta_region_edit, lv_color_white(),     LV_PART_CURSOR);
    lv_obj_set_style_border_width(ta_region_edit, 2,                     LV_PART_CURSOR);
    lv_obj_set_style_border_side(ta_region_edit,  LV_BORDER_SIDE_LEFT,   LV_PART_CURSOR);
    lv_obj_set_style_border_opa(ta_region_edit,   LV_OPA_COVER,          LV_PART_CURSOR);

    lv_obj_t *btn_region_confirm = lv_button_create(obj_region_edit_panel);
    lv_obj_set_size(btn_region_confirm, SCREEN_WIDTH - 40, 50);
    lv_obj_align(btn_region_confirm, LV_ALIGN_TOP_MID, 0, 185);
    lv_obj_set_style_bg_color(btn_region_confirm, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(btn_region_confirm, 8, 0);
    lv_obj_t *region_confirm_lbl = lv_label_create(btn_region_confirm);
    lv_label_set_text(region_confirm_lbl, "Confirm");
    lv_obj_set_style_text_color(region_confirm_lbl, lv_color_black(), 0);
    meck_set_font(region_confirm_lbl, &meck_montserrat_22, 0);
    lv_obj_center(region_confirm_lbl);
    lv_obj_add_event_cb(btn_region_confirm, on_region_edit_save, LV_EVENT_CLICKED, NULL);

    kb_region_edit = lv_keyboard_create(obj_region_edit_panel);
    meck_style_keyboard(kb_region_edit);
    lv_keyboard_set_textarea(kb_region_edit, ta_region_edit);
    lv_obj_add_event_cb(kb_region_edit, on_region_edit_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_region_edit, on_region_edit_kb_event, LV_EVENT_CANCEL, NULL);
    lv_obj_add_event_cb(kb_region_edit, on_kb_long_press,
                        LV_EVENT_LONG_PRESSED, NULL);

    // ---- Export Config modal ----
    // Same overlay pattern as the Edit Node Name modal above: full-screen
    // panel, 90% black background, hidden by default. Built once at
    // settings-screen construction time so the tap handler just unhides.
    // Title at y=50 clears the camera punch-hole on the TFT variant; the
    // checkboxes step down from there with the warning sub-label tucked
    // under the Identity row.
    obj_export_panel = lv_obj_create(scr_settings);
    lv_obj_set_size(obj_export_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_export_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_export_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_export_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(obj_export_panel, 0, 0);
    lv_obj_add_flag(obj_export_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_export_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_export_panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *export_title = lv_label_create(obj_export_panel);
    lv_label_set_text(export_title, "Export Config");
    lv_obj_set_style_text_color(export_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(export_title, &meck_montserrat_22, 0);
    lv_obj_align(export_title, LV_ALIGN_TOP_MID, 0, 50);

    // Identity checkbox + small warning sub-label.
    cb_export_identity = lv_checkbox_create(obj_export_panel);
    lv_checkbox_set_text(cb_export_identity, "Identity (incl. private key)");
    lv_obj_set_style_text_color(cb_export_identity, lv_color_white(), 0);
    meck_set_font(cb_export_identity, &meck_montserrat_18, 0);
    lv_obj_align(cb_export_identity, LV_ALIGN_TOP_LEFT, 30, 110);
    lv_obj_add_state(cb_export_identity, LV_STATE_CHECKED);
    lv_obj_add_event_cb(cb_export_identity,
                        on_export_modal_identity_changed,
                        LV_EVENT_VALUE_CHANGED, NULL);

    lbl_export_warn = lv_label_create(obj_export_panel);
    lv_label_set_text(lbl_export_warn,
        "Anyone with this file can impersonate your node.");
    lv_obj_set_style_text_color(lbl_export_warn,
        lv_palette_main(LV_PALETTE_ORANGE), 0);
    meck_set_font(lbl_export_warn, &meck_montserrat_14, 0);
    lv_label_set_long_mode(lbl_export_warn, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_export_warn, SCREEN_WIDTH - 80);
    lv_obj_align(lbl_export_warn, LV_ALIGN_TOP_LEFT, 60, 150);

    // Channels.
    cb_export_channels = lv_checkbox_create(obj_export_panel);
    lv_checkbox_set_text(cb_export_channels, "Channels");
    lv_obj_set_style_text_color(cb_export_channels, lv_color_white(), 0);
    meck_set_font(cb_export_channels, &meck_montserrat_18, 0);
    lv_obj_align(cb_export_channels, LV_ALIGN_TOP_LEFT, 30, 200);
    lv_obj_add_state(cb_export_channels, LV_STATE_CHECKED);

    // Contacts.
    cb_export_contacts = lv_checkbox_create(obj_export_panel);
    lv_checkbox_set_text(cb_export_contacts, "Contacts");
    lv_obj_set_style_text_color(cb_export_contacts, lv_color_white(), 0);
    meck_set_font(cb_export_contacts, &meck_montserrat_18, 0);
    lv_obj_align(cb_export_contacts, LV_ALIGN_TOP_LEFT, 30, 250);
    lv_obj_add_state(cb_export_contacts, LV_STATE_CHECKED);

    // Radio settings.
    cb_export_radio = lv_checkbox_create(obj_export_panel);
    lv_checkbox_set_text(cb_export_radio, "Radio settings");
    lv_obj_set_style_text_color(cb_export_radio, lv_color_white(), 0);
    meck_set_font(cb_export_radio, &meck_montserrat_18, 0);
    lv_obj_align(cb_export_radio, LV_ALIGN_TOP_LEFT, 30, 300);
    lv_obj_add_state(cb_export_radio, LV_STATE_CHECKED);

    // Cancel button (grey, bottom-left).
    lv_obj_t *btn_export_cancel = lv_button_create(obj_export_panel);
    lv_obj_set_size(btn_export_cancel, 140, 70);
    lv_obj_align(btn_export_cancel, LV_ALIGN_BOTTOM_LEFT, 30, -30);
    lv_obj_set_style_bg_color(btn_export_cancel, lv_color_make(60, 60, 60), 0);
    lv_obj_set_style_radius(btn_export_cancel, 8, 0);
    lv_obj_t *lbl_cancel = lv_label_create(btn_export_cancel);
    lv_label_set_text(lbl_cancel, "Cancel");
    lv_obj_set_style_text_color(lbl_cancel, lv_color_white(), 0);
    meck_set_font(lbl_cancel, &meck_montserrat_18, 0);
    lv_obj_center(lbl_cancel);
    lv_obj_add_event_cb(btn_export_cancel, on_export_modal_cancel,
                        LV_EVENT_CLICKED, NULL);

    // Export button (cyan, bottom-right).
    lv_obj_t *btn_export_go = lv_button_create(obj_export_panel);
    lv_obj_set_size(btn_export_go, 140, 70);
    lv_obj_align(btn_export_go, LV_ALIGN_BOTTOM_RIGHT, -30, -30);
    lv_obj_set_style_bg_color(btn_export_go,
        lv_palette_darken(LV_PALETTE_CYAN, 2), 0);
    lv_obj_set_style_radius(btn_export_go, 8, 0);
    lv_obj_t *lbl_go = lv_label_create(btn_export_go);
    lv_label_set_text(lbl_go, "Export");
    lv_obj_set_style_text_color(lbl_go, lv_color_white(), 0);
    meck_set_font(lbl_go, &meck_montserrat_18, 0);
    lv_obj_center(lbl_go);
    lv_obj_add_event_cb(btn_export_go, on_export_modal_export,
                        LV_EVENT_CLICKED, NULL);

    settings_update_labels();
}

// ============================================================================
// Radio preset picker screen
// ============================================================================

// ============================================================================
// Channels settings sub-screen (Settings > Channels)
// ============================================================================

static void create_settings_channels_screen() {
    scr_settings_channels = lv_obj_create(NULL);
    lock_screen_scroll(scr_settings_channels);
    lv_obj_set_style_bg_color(scr_settings_channels, lv_color_black(), 0);
    screen_attach_clock_battery(scr_settings_channels, 12, &meck_montserrat_24, 30);

    // Back button → return to main settings
    lv_obj_t *btn_back = lv_button_create(scr_settings_channels);
    lv_obj_set_size(btn_back, 80, 50);
    lv_obj_set_pos(btn_back, 10, 25);
    lv_obj_set_style_bg_opa(btn_back, 0, 0);
    lv_obj_t *back_lbl = lv_label_create(btn_back);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(back_lbl, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(back_lbl, &meck_montserrat_24, 0);
    lv_obj_center(back_lbl);
    lv_obj_add_event_cb(btn_back, [](lv_event_t *e) {
        if (scr_settings) lv_screen_load(scr_settings);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr_settings_channels);
    lv_label_set_text(title, "Channels");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    obj_ch_settings_scroll = lv_obj_create(scr_settings_channels);
    lv_obj_set_size(obj_ch_settings_scroll, SCREEN_WIDTH, SCREEN_HEIGHT - 90);
    lv_obj_set_pos(obj_ch_settings_scroll, 0, 90);
    lv_obj_set_style_bg_opa(obj_ch_settings_scroll, 0, 0);
    lv_obj_set_style_border_width(obj_ch_settings_scroll, 0, 0);
    lv_obj_set_style_pad_all(obj_ch_settings_scroll, 0, 0);
    lv_obj_set_scroll_dir(obj_ch_settings_scroll, LV_DIR_VER);

    // ---- Add Channel overlay (hidden by default) ----
    obj_ch_settings_add_panel = lv_obj_create(scr_settings_channels);
    lv_obj_set_size(obj_ch_settings_add_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_ch_settings_add_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_ch_settings_add_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_ch_settings_add_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(obj_ch_settings_add_panel, 0, 0);
    lv_obj_add_flag(obj_ch_settings_add_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_ch_settings_add_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_ch_settings_add_panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *add_title = lv_label_create(obj_ch_settings_add_panel);
    lv_label_set_text(add_title, "Add Channel");
    lv_obj_set_style_text_color(add_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(add_title, &meck_montserrat_22, 0);
    lv_obj_align(add_title, LV_ALIGN_TOP_MID, 0, 50);

    lv_obj_t *hint = lv_label_create(obj_ch_settings_add_panel);
    lv_label_set_text(hint, "# = public, no # = private (e.g. #sydney)");
    lv_obj_set_style_text_color(hint, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(hint, &meck_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 90);

    ta_ch_settings_add = lv_textarea_create(obj_ch_settings_add_panel);
    lv_obj_set_size(ta_ch_settings_add, SCREEN_WIDTH - 40, 50);
    lv_obj_align(ta_ch_settings_add, LV_ALIGN_TOP_MID, 0, 120);
    lv_textarea_set_one_line(ta_ch_settings_add, true);
    lv_textarea_set_max_length(ta_ch_settings_add, 30);
    lv_textarea_set_text(ta_ch_settings_add, "#");
    lv_obj_set_style_bg_color(ta_ch_settings_add, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_ch_settings_add, lv_color_white(), 0);
    meck_set_font(ta_ch_settings_add, &meck_montserrat_18, 0);
    lv_obj_set_style_border_color(ta_ch_settings_add, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_border_color(ta_ch_settings_add, lv_color_white(),     LV_PART_CURSOR);
    lv_obj_set_style_border_width(ta_ch_settings_add, 2,                     LV_PART_CURSOR);
    lv_obj_set_style_border_side(ta_ch_settings_add,  LV_BORDER_SIDE_LEFT,   LV_PART_CURSOR);
    lv_obj_set_style_border_opa(ta_ch_settings_add,   LV_OPA_COVER,          LV_PART_CURSOR);

    lv_obj_t *btn_confirm = lv_button_create(obj_ch_settings_add_panel);
    lv_obj_set_size(btn_confirm, SCREEN_WIDTH - 40, 50);
    lv_obj_align(btn_confirm, LV_ALIGN_TOP_MID, 0, 185);
    lv_obj_set_style_bg_color(btn_confirm, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_radius(btn_confirm, 8, 0);
    lv_obj_t *confirm_lbl = lv_label_create(btn_confirm);
    lv_label_set_text(confirm_lbl, "Confirm");
    lv_obj_set_style_text_color(confirm_lbl, lv_color_black(), 0);
    meck_set_font(confirm_lbl, &meck_montserrat_22, 0);
    lv_obj_center(confirm_lbl);
    lv_obj_add_event_cb(btn_confirm, on_ch_settings_add_save, LV_EVENT_CLICKED, NULL);

    kb_ch_settings_add = lv_keyboard_create(obj_ch_settings_add_panel);
    meck_style_keyboard(kb_ch_settings_add);
    lv_keyboard_set_textarea(kb_ch_settings_add, ta_ch_settings_add);
    lv_obj_add_event_cb(kb_ch_settings_add, on_ch_settings_add_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_ch_settings_add, on_ch_settings_add_kb_event, LV_EVENT_CANCEL, NULL);
    lv_obj_add_event_cb(kb_ch_settings_add, on_kb_long_press,
                        LV_EVENT_LONG_PRESSED, NULL);
}

// ============================================================================
// Channel detail sub-screen (Settings > Channels > [channel])
// ============================================================================

static void create_channel_detail_screen() {
    scr_channel_detail = lv_obj_create(NULL);
    lock_screen_scroll(scr_channel_detail);
    lv_obj_set_style_bg_color(scr_channel_detail, lv_color_black(), 0);
    screen_attach_clock_battery(scr_channel_detail, 13, &meck_montserrat_24, 30);

    // Back button → return to channels list
    lv_obj_t *btn_back = lv_button_create(scr_channel_detail);
    lv_obj_set_size(btn_back, 80, 50);
    lv_obj_set_pos(btn_back, 10, 25);
    lv_obj_set_style_bg_opa(btn_back, 0, 0);
    lv_obj_t *back_lbl = lv_label_create(btn_back);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(back_lbl, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(back_lbl, &meck_montserrat_24, 0);
    lv_obj_center(back_lbl);
    lv_obj_add_event_cb(btn_back, [](lv_event_t *e) {
        refresh_channels_settings_list();
        if (scr_settings_channels) lv_screen_load(scr_settings_channels);
    }, LV_EVENT_CLICKED, NULL);

    // Channel name title (updated by refresh_channel_detail_labels)
    lbl_ch_detail_name = lv_label_create(scr_channel_detail);
    lv_label_set_text(lbl_ch_detail_name, "");
    lv_obj_set_style_text_color(lbl_ch_detail_name, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(lbl_ch_detail_name, &meck_montserrat_24, 0);
    lv_obj_align(lbl_ch_detail_name, LV_ALIGN_TOP_LEFT, 120, 30);

    int y = 100;

    // Row 1: Region Scope
    create_settings_row(scr_channel_detail, "Region Scope",
        &lbl_ch_detail_scope, on_ch_detail_scope_tap, y);
    y += 75;

    // Row 2: Notifications
    create_settings_row(scr_channel_detail, "Notifications (tap to cycle)",
        &lbl_ch_detail_notif, on_ch_detail_notif_tap, y);
    y += 75;

    // Row 3: Notification Tone
    create_settings_row(scr_channel_detail, "Notification Tone",
        &lbl_ch_detail_tone, on_ch_detail_tone_tap, y);
    y += 75;

    // Row 4: Share Channel (sends [MECK:CH] DM with channel secret)
    {
        lv_obj_t *share_btn = lv_button_create(scr_channel_detail);
        lv_obj_set_size(share_btn, SCREEN_WIDTH - 40, 55);
        lv_obj_set_pos(share_btn, 20, y);
        lv_obj_set_style_bg_color(share_btn, lv_color_make(20, 40, 60), 0);
        lv_obj_set_style_radius(share_btn, 10, 0);
        lv_obj_set_style_border_width(share_btn, 1, 0);
        lv_obj_set_style_border_color(share_btn, lv_palette_main(LV_PALETTE_CYAN), 0);
        lv_obj_add_event_cb(share_btn, on_ch_detail_share_tap, LV_EVENT_CLICKED, NULL);

        lv_obj_t *share_lbl = lv_label_create(share_btn);
        lv_label_set_text(share_lbl, "Share Channel");
        lv_obj_set_style_text_color(share_lbl, lv_palette_main(LV_PALETTE_CYAN), 0);
        meck_set_font(share_lbl, &meck_montserrat_18, 0);
        lv_obj_center(share_lbl);
    }
    y += 65;

    // Row 5: Delete Channel (not shown for primary channel; handled in tap)
    {
        lv_obj_t *del_btn = lv_button_create(scr_channel_detail);
        lv_obj_set_size(del_btn, SCREEN_WIDTH - 40, 55);
        lv_obj_set_pos(del_btn, 20, y);
        lv_obj_set_style_bg_color(del_btn, lv_color_make(60, 20, 20), 0);
        lv_obj_set_style_radius(del_btn, 10, 0);
        lv_obj_set_style_border_width(del_btn, 1, 0);
        lv_obj_set_style_border_color(del_btn, lv_palette_main(LV_PALETTE_RED), 0);
        lv_obj_add_event_cb(del_btn, on_ch_detail_delete_tap, LV_EVENT_CLICKED, NULL);

        lbl_ch_detail_delete = lv_label_create(del_btn);
        lv_label_set_text(lbl_ch_detail_delete, "Delete Channel");
        lv_obj_set_style_text_color(lbl_ch_detail_delete, lv_palette_main(LV_PALETTE_RED), 0);
        meck_set_font(lbl_ch_detail_delete, &meck_montserrat_18, 0);
        lv_obj_center(lbl_ch_detail_delete);
    }

    // ---- Scope edit overlay (on this screen) ----
    obj_ch_scope_edit_panel = lv_obj_create(scr_channel_detail);
    lv_obj_set_size(obj_ch_scope_edit_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_ch_scope_edit_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_ch_scope_edit_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_ch_scope_edit_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(obj_ch_scope_edit_panel, 0, 0);
    lv_obj_add_flag(obj_ch_scope_edit_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_ch_scope_edit_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_ch_scope_edit_panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *scope_title = lv_label_create(obj_ch_scope_edit_panel);
    lv_label_set_text(scope_title, "Channel Region");
    lv_obj_set_style_text_color(scope_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(scope_title, &meck_montserrat_22, 0);
    lv_obj_align(scope_title, LV_ALIGN_TOP_MID, 0, 50);

    lv_obj_t *scope_hint = lv_label_create(obj_ch_scope_edit_panel);
    lv_label_set_text(scope_hint, "Empty = use device default");
    lv_obj_set_style_text_color(scope_hint, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(scope_hint, &meck_montserrat_14, 0);
    lv_obj_align(scope_hint, LV_ALIGN_TOP_MID, 0, 90);

    ta_ch_scope_edit = lv_textarea_create(obj_ch_scope_edit_panel);
    lv_obj_set_size(ta_ch_scope_edit, SCREEN_WIDTH - 40, 50);
    lv_obj_align(ta_ch_scope_edit, LV_ALIGN_TOP_MID, 0, 120);
    lv_textarea_set_one_line(ta_ch_scope_edit, true);
    lv_textarea_set_max_length(ta_ch_scope_edit, 30);
    lv_obj_set_style_bg_color(ta_ch_scope_edit, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_ch_scope_edit, lv_color_white(), 0);
    meck_set_font(ta_ch_scope_edit, &meck_montserrat_18, 0);
    lv_obj_set_style_border_color(ta_ch_scope_edit, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_border_color(ta_ch_scope_edit, lv_color_white(),     LV_PART_CURSOR);
    lv_obj_set_style_border_width(ta_ch_scope_edit, 2,                     LV_PART_CURSOR);
    lv_obj_set_style_border_side(ta_ch_scope_edit,  LV_BORDER_SIDE_LEFT,   LV_PART_CURSOR);
    lv_obj_set_style_border_opa(ta_ch_scope_edit,   LV_OPA_COVER,          LV_PART_CURSOR);

    lv_obj_t *btn_scope_confirm = lv_button_create(obj_ch_scope_edit_panel);
    lv_obj_set_size(btn_scope_confirm, SCREEN_WIDTH - 40, 50);
    lv_obj_align(btn_scope_confirm, LV_ALIGN_TOP_MID, 0, 185);
    lv_obj_set_style_bg_color(btn_scope_confirm, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(btn_scope_confirm, 8, 0);
    lv_obj_t *scope_confirm_lbl = lv_label_create(btn_scope_confirm);
    lv_label_set_text(scope_confirm_lbl, "Confirm");
    lv_obj_set_style_text_color(scope_confirm_lbl, lv_color_black(), 0);
    meck_set_font(scope_confirm_lbl, &meck_montserrat_22, 0);
    lv_obj_center(scope_confirm_lbl);
    lv_obj_add_event_cb(btn_scope_confirm, on_ch_scope_edit_save, LV_EVENT_CLICKED, NULL);

    kb_ch_scope_edit = lv_keyboard_create(obj_ch_scope_edit_panel);
    meck_style_keyboard(kb_ch_scope_edit);
    lv_keyboard_set_textarea(kb_ch_scope_edit, ta_ch_scope_edit);
    lv_obj_add_event_cb(kb_ch_scope_edit, on_ch_scope_edit_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_ch_scope_edit, on_ch_scope_edit_kb_event, LV_EVENT_CANCEL, NULL);
    lv_obj_add_event_cb(kb_ch_scope_edit, on_kb_long_press,
                        LV_EVENT_LONG_PRESSED, NULL);

    // ---- Tone picker overlay ----
    // Parented to the top layer (not this screen) so it can also be opened
    // from the DM inbox for the DM notification tone.
    obj_tone_picker_panel = lv_obj_create(lv_layer_top());
    lv_obj_set_size(obj_tone_picker_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_tone_picker_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_tone_picker_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_tone_picker_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(obj_tone_picker_panel, 0, 0);
    lv_obj_add_flag(obj_tone_picker_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_tone_picker_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_tone_picker_panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *tone_title = lv_label_create(obj_tone_picker_panel);
    lv_label_set_text(tone_title, "Select Tone");
    lv_obj_set_style_text_color(tone_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(tone_title, &meck_montserrat_22, 0);
    lv_obj_align(tone_title, LV_ALIGN_TOP_MID, 0, 50);

    lv_obj_t *tone_hint = lv_label_create(obj_tone_picker_panel);
    lv_label_set_text(tone_hint, "Tap to select and preview");
    lv_obj_set_style_text_color(tone_hint, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(tone_hint, &meck_montserrat_14, 0);
    lv_obj_align(tone_hint, LV_ALIGN_TOP_MID, 0, 85);

    // Cancel button
    lv_obj_t *tone_cancel = lv_button_create(obj_tone_picker_panel);
    lv_obj_set_size(tone_cancel, 80, 40);
    lv_obj_align(tone_cancel, LV_ALIGN_TOP_LEFT, 10, 45);
    lv_obj_set_style_bg_opa(tone_cancel, 0, 0);
    lv_obj_t *cancel_lbl = lv_label_create(tone_cancel);
    lv_label_set_text(cancel_lbl, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(cancel_lbl, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(cancel_lbl, &meck_montserrat_22, 0);
    lv_obj_center(cancel_lbl);
    lv_obj_add_event_cb(tone_cancel, on_tone_picker_cancel, LV_EVENT_CLICKED, NULL);

    // Scrollable list area
    obj_tone_picker_scroll = lv_obj_create(obj_tone_picker_panel);
    lv_obj_set_size(obj_tone_picker_scroll, SCREEN_WIDTH, SCREEN_HEIGHT - 110);
    lv_obj_set_pos(obj_tone_picker_scroll, 0, 110);
    lv_obj_set_style_bg_opa(obj_tone_picker_scroll, 0, 0);
    lv_obj_set_style_border_width(obj_tone_picker_scroll, 0, 0);
    lv_obj_set_style_pad_all(obj_tone_picker_scroll, 0, 0);
    lv_obj_set_scroll_dir(obj_tone_picker_scroll, LV_DIR_VER);
}

static void create_radio_picker_screen() {
    scr_radio_picker = lv_obj_create(NULL);
    lock_screen_scroll(scr_radio_picker);
    lv_obj_set_style_bg_color(scr_radio_picker, lv_color_black(), 0);
    screen_attach_clock_battery(scr_radio_picker, 2, &meck_montserrat_24, 30);

    lv_obj_t *btn_back = lv_button_create(scr_radio_picker);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, goto_settings, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr_radio_picker);
    lv_label_set_text(title, "Radio Preset");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    lv_obj_t *scroll = lv_obj_create(scr_radio_picker);
    lv_obj_set_size(scroll, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 100);
    lv_obj_set_pos(scroll, 10, 90);
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
        meck_set_font(name_lbl, &meck_montserrat_16, 0);
        lv_obj_align(name_lbl, LV_ALIGN_LEFT_MID, 5, -8);

        char detail[64];
        snprintf(detail, sizeof(detail), "%.3f MHz  BW%.0f  SF%d  CR4/%d",
                 RADIO_PRESETS[i].freq, RADIO_PRESETS[i].bw,
                 RADIO_PRESETS[i].sf, RADIO_PRESETS[i].cr);
        lv_obj_t *det_lbl = lv_label_create(btn);
        lv_label_set_text(det_lbl, detail);
        lv_obj_set_style_text_color(det_lbl, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(det_lbl, &meck_montserrat_14, 0);
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
    lbl_picker_dm_unread = NULL;

    int y = 5;

    // ---- Direct Messages pseudo-row ----
    // Always shown, regardless of whether any DMs exist (zero state lives
    // on the inbox screen as "No DMs yet"). Teal border + label, chosen
    // to read distinctly from the default-cyan #public channel that
    // sits directly below. No delete button — DMs are a virtual
    // category, not a removable channel.
    {
        // Teal rather than cyan: cyan is the default channel colour
        // (CH_COLORS[0]), used by the public channel directly below,
        // and the two rows were visually identical. Teal is the colour
        // of the Audio home tile border (CH_COLORS[8]) and reads
        // clearly distinct against cyan in the same scroll view.
        lv_color_t dm_color = lv_palette_main(LV_PALETTE_TEAL);

        lv_obj_t *btn_dm = lv_button_create(obj_ch_picker_scroll);
        lv_obj_set_size(btn_dm, SCREEN_WIDTH - 40, 75);
        lv_obj_set_pos(btn_dm, 0, y);
        lv_obj_set_style_bg_color(btn_dm, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(btn_dm, 12, 0);
        lv_obj_set_style_border_width(btn_dm, 2, 0);
        lv_obj_set_style_border_color(btn_dm, dm_color, 0);
        lv_obj_add_event_cb(btn_dm, goto_dm_inbox, LV_EVENT_CLICKED, NULL);

        lv_obj_t *lbl_dm_name = lv_label_create(btn_dm);
        lv_label_set_text(lbl_dm_name, "Direct Messages");
        lv_obj_set_style_text_color(lbl_dm_name, dm_color, 0);
        meck_set_font(lbl_dm_name, &meck_montserrat_22, 0);
        lv_obj_align(lbl_dm_name, LV_ALIGN_LEFT_MID, 10, 0);

        // Total unread badge, mirroring the per-channel unread display.
        // Label handle is cached into lbl_picker_dm_unread so the
        // ui_update_timer_cb can refresh the text live as new DMs
        // arrive without needing to rebuild the whole picker.
        int dm_unread = dm_total_unread();
        lv_obj_t *lbl_dm_unread = lv_label_create(btn_dm);
        if (dm_unread > 0) {
            lv_label_set_text_fmt(lbl_dm_unread, "%d new", dm_unread);
        } else {
            lv_label_set_text(lbl_dm_unread, "");
        }
        lv_obj_set_style_text_color(lbl_dm_unread, lv_color_make(100, 255, 100), 0);
        meck_set_font(lbl_dm_unread, &meck_montserrat_18, 0);
        lv_obj_align(lbl_dm_unread, LV_ALIGN_RIGHT_MID, -10, 0);
        lbl_picker_dm_unread = lbl_dm_unread;

        y += 85;
    }

    int num_ch = mesh->getActiveChannelCount();

    for (int i = 0; i < num_ch && i < MAX_GROUP_CHANNELS; i++) {
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
        meck_set_font(lbl_name, &meck_montserrat_22, 0);
        lv_obj_align(lbl_name, LV_ALIGN_LEFT_MID, 10, 0);

        lbl_picker_unread[i] = lv_label_create(btn);
        lv_label_set_text(lbl_picker_unread[i], "");
        lv_obj_set_style_text_color(lbl_picker_unread[i], lv_color_make(100, 255, 100), 0);
        meck_set_font(lbl_picker_unread[i], &meck_montserrat_18, 0);
        lv_obj_align(lbl_picker_unread[i], LV_ALIGN_RIGHT_MID, -10, 0);

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
    meck_set_font(add_lbl, &meck_montserrat_22, 0);
    lv_obj_center(add_lbl);
}

// ============================================================================
// "Not yet implemented" placeholder screen
// ============================================================================

static void create_not_implemented_screen() {
    scr_not_implemented = lv_obj_create(NULL);
    lock_screen_scroll(scr_not_implemented);
    lv_obj_set_style_bg_color(scr_not_implemented, lv_color_black(), 0);
    screen_attach_clock_battery(scr_not_implemented, 12, &meck_montserrat_24, 30);

    // Back button
    lv_obj_t *btn_back = lv_button_create(scr_not_implemented);
    lv_obj_set_size(btn_back, 80, 50);
    lv_obj_set_pos(btn_back, 10, 25);
    lv_obj_set_style_bg_opa(btn_back, 0, 0);
    lv_obj_t *back_lbl = lv_label_create(btn_back);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(back_lbl, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(back_lbl, &meck_montserrat_24, 0);
    lv_obj_center(back_lbl);
    lv_obj_add_event_cb(btn_back, [](lv_event_t *e) {
        if (scr_home) lv_screen_load(scr_home);
    }, LV_EVENT_CLICKED, NULL);

    // Feature name (updated by show_not_implemented before screen load)
    lbl_not_impl_title = lv_label_create(scr_not_implemented);
    lv_label_set_text(lbl_not_impl_title, "");
    lv_obj_set_style_text_color(lbl_not_impl_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(lbl_not_impl_title, &meck_montserrat_28, 0);
    lv_obj_align(lbl_not_impl_title, LV_ALIGN_TOP_LEFT, 120, 30);

    lv_obj_t *msg = lv_label_create(scr_not_implemented);
    lv_label_set_text(msg, "This feature has not yet\nbeen implemented.\n\nCheck back in a future\nfirmware update.");
    lv_obj_set_style_text_color(msg, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(msg, &meck_montserrat_18, 0);
    lv_label_set_long_mode(msg, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(msg, SCREEN_WIDTH - 60);
    lv_obj_align(msg, LV_ALIGN_CENTER, 0, 0);
}

// ============================================================================
// Position settings sub-screen (Settings > Position)
// ============================================================================

static void create_settings_position_screen() {
    scr_settings_position = lv_obj_create(NULL);
    lock_screen_scroll(scr_settings_position);
    lv_obj_set_style_bg_color(scr_settings_position, lv_color_black(), 0);
    screen_attach_clock_battery(scr_settings_position, 12, &meck_montserrat_24, 30);

    // Back button
    lv_obj_t *btn_back = lv_button_create(scr_settings_position);
    lv_obj_set_size(btn_back, 80, 50);
    lv_obj_set_pos(btn_back, 10, 25);
    lv_obj_set_style_bg_opa(btn_back, 0, 0);
    lv_obj_t *back_lbl = lv_label_create(btn_back);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(back_lbl, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(back_lbl, &meck_montserrat_24, 0);
    lv_obj_center(back_lbl);
    lv_obj_add_event_cb(btn_back, [](lv_event_t *e) {
        if (scr_settings) lv_screen_load(scr_settings);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr_settings_position);
    lv_label_set_text(title, "Position");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    int y = 100;

    // Latitude row
    create_settings_row(scr_settings_position, "Latitude (tap to edit)",
        &lbl_pos_lat_value, on_pos_lat_tap, y);
    y += 75;

    // Longitude row
    create_settings_row(scr_settings_position, "Longitude (tap to edit)",
        &lbl_pos_lon_value, on_pos_lon_tap, y);
    y += 75;

    // Position mode cycle row
    create_settings_row(scr_settings_position, "Share Position (tap to cycle)",
        &lbl_pos_mode_value, on_pos_mode_tap, y);
    y += 75;

    // ---- Numeric edit overlay (for lat/lon entry) ----
    obj_pos_edit_panel = lv_obj_create(scr_settings_position);
    lv_obj_set_size(obj_pos_edit_panel, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(obj_pos_edit_panel, 0, 0);
    lv_obj_set_style_bg_color(obj_pos_edit_panel, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(obj_pos_edit_panel, LV_OPA_90, 0);
    lv_obj_set_style_border_width(obj_pos_edit_panel, 0, 0);
    lv_obj_add_flag(obj_pos_edit_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(obj_pos_edit_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_pos_edit_panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *edit_title = lv_label_create(obj_pos_edit_panel);
    lv_label_set_text(edit_title, "Enter value");
    lv_obj_set_style_text_color(edit_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(edit_title, &meck_montserrat_22, 0);
    lv_obj_align(edit_title, LV_ALIGN_TOP_MID, 0, 50);

    ta_pos_edit = lv_textarea_create(obj_pos_edit_panel);
    lv_obj_set_size(ta_pos_edit, SCREEN_WIDTH - 40, 50);
    lv_obj_align(ta_pos_edit, LV_ALIGN_TOP_MID, 0, 100);
    lv_textarea_set_one_line(ta_pos_edit, true);
    lv_textarea_set_max_length(ta_pos_edit, 20);
    lv_obj_set_style_bg_color(ta_pos_edit, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_pos_edit, lv_color_white(), 0);
    meck_set_font(ta_pos_edit, &meck_montserrat_18, 0);
    lv_obj_set_style_border_color(ta_pos_edit, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_border_width(ta_pos_edit, 1, 0);
    lv_obj_set_style_radius(ta_pos_edit, 8, 0);

    kb_pos_edit = lv_keyboard_create(obj_pos_edit_panel);
    lv_keyboard_set_mode(kb_pos_edit, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(kb_pos_edit, ta_pos_edit);
    meck_style_keyboard(kb_pos_edit);
    lv_obj_add_event_cb(kb_pos_edit, on_pos_edit_kb_event, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(kb_pos_edit, on_pos_edit_kb_event, LV_EVENT_CANCEL, NULL);

    refresh_position_labels();
}

// ============================================================================
// Voice screen — recording, review, send
// ============================================================================

// Forward: MeckVoice instance accessor from meck_app.cpp
// (declared at file scope near includes)

// Forward: voice send request
// (declared at file scope near includes)

static void voice_update_ui(void) {
    MeckVoice* voice = meck_get_voice_instance();
    if (!voice) return;

    bool recording = voice->isRecording();
    bool has_rec = g_voice_has_recording;

    // Status label
    if (lbl_voice_status) {
        if (recording) {
            lv_label_set_text(lbl_voice_status, "Recording...");
            lv_obj_set_style_text_color(lbl_voice_status,
                lv_palette_main(LV_PALETTE_RED), 0);
        } else if (has_rec) {
            char buf[48];
            snprintf(buf, sizeof(buf), "Recorded %.1fs", voice->getRecDurationSec());
            lv_label_set_text(lbl_voice_status, buf);
            lv_obj_set_style_text_color(lbl_voice_status,
                lv_palette_main(LV_PALETTE_GREEN), 0);
        } else {
            lv_label_set_text(lbl_voice_status, "Tap Record to start");
            lv_obj_set_style_text_color(lbl_voice_status,
                lv_palette_main(LV_PALETTE_GREY), 0);
        }
    }

    // Timer label (during recording)
    if (lbl_voice_timer) {
        if (recording) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%.1fs / %ds",
                     voice->getRecDurationSec(), VOICE_MAX_SECONDS);
            lv_label_set_text(lbl_voice_timer, buf);
        } else {
            lv_label_set_text(lbl_voice_timer, "");
        }
    }

    // Record button label
    if (lbl_voice_record_btn) {
        if (recording)
            lv_label_set_text(lbl_voice_record_btn, "Stop");
        else if (has_rec)
            lv_label_set_text(lbl_voice_record_btn, "New");
        else
            lv_label_set_text(lbl_voice_record_btn, "Record");
    }

    // Record button color
    if (btn_voice_record) {
        lv_obj_set_style_bg_color(btn_voice_record,
            recording ? lv_color_make(180, 30, 30)
                      : lv_palette_main(LV_PALETTE_RED), 0);
    }

    // Play + Send + Discard buttons: visible only after a recording
    if (btn_voice_play) {
        if (has_rec && !recording)
            lv_obj_clear_flag(btn_voice_play, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(btn_voice_play, LV_OBJ_FLAG_HIDDEN);
    }
    if (btn_voice_send) {
        if (has_rec && !recording)
            lv_obj_clear_flag(btn_voice_send, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(btn_voice_send, LV_OBJ_FLAG_HIDDEN);
    }
    if (btn_voice_discard) {
        if (has_rec && !recording)
            lv_obj_clear_flag(btn_voice_discard, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(btn_voice_discard, LV_OBJ_FLAG_HIDDEN);
    }
}

static void on_voice_record_tap(lv_event_t *e) {
    (void)e;
    MeckVoice* voice = meck_get_voice_instance();
    if (!voice) return;

    if (voice->isRecording()) {
        voice->stopRecording();
        if (voice->getRecSamples() > 4000) {  // > 0.25s
            g_voice_has_recording = true;
            // Save to SD as WAV
            uint32_t ts = (uint32_t)(esp_timer_get_time() / 1000000ULL);
            char filename[48];
            snprintf(filename, sizeof(filename), "voice_%06lu.wav",
                     (unsigned long)(ts % 1000000UL));
            voice->saveToWav(filename);
            snprintf(g_voice_last_wav_path, sizeof(g_voice_last_wav_path),
                     "%s/%s", VOICE_FOLDER, filename);
#ifdef HAVE_CODEC2
            voice->encodeCodec2();
#endif
        } else {
            g_voice_has_recording = false;
        }
    } else {
        g_voice_has_recording = false;
        voice->startRecording();
    }
    voice_update_ui();
}

static void on_voice_play_tap(lv_event_t *e) {
    (void)e;
    if (g_voice_last_wav_path[0] == '\0') {
        printf("Voice: no WAV to play\n");
        return;
    }
    printf("Voice: playing %s\n", g_voice_last_wav_path);
    // Boost volume for voice playback (default 50% is too quiet)
    meck_audio_set_volume_pct(70);
    meck_audio_codec_wake();
    meck_audio_play_file(g_voice_last_wav_path, 0);
    meck_audio_resume();
    if (lbl_voice_status) {
        lv_label_set_text(lbl_voice_status, "Playing...");
        lv_obj_set_style_text_color(lbl_voice_status,
            lv_palette_main(LV_PALETTE_TEAL), 0);
    }
}

static void on_voice_discard_tap(lv_event_t *e) {
    (void)e;
    // Delete the WAV file from SD if it exists
    if (g_voice_last_wav_path[0] != '\0') {
        if (remove(g_voice_last_wav_path) == 0) {
            printf("Voice: deleted %s\n", g_voice_last_wav_path);
        } else {
            printf("Voice: failed to delete %s\n", g_voice_last_wav_path);
        }
    }
    g_voice_has_recording = false;
    g_voice_last_wav_path[0] = '\0';
    if (lbl_voice_status) {
        lv_label_set_text(lbl_voice_status, "Tap Record to start");
        lv_obj_set_style_text_color(lbl_voice_status,
            lv_palette_main(LV_PALETTE_GREY), 0);
    }
    voice_update_ui();
    voice_prev_refresh();  // refresh the list to reflect deletion
    printf("Voice: recording discarded\n");
}

// Load a previous recording from the list
static void on_voice_prev_row_tap(lv_event_t *e) {
    const char *path = (const char *)lv_event_get_user_data(e);
    if (!path || !path[0]) return;

    strncpy(g_voice_last_wav_path, path, sizeof(g_voice_last_wav_path) - 1);
    g_voice_last_wav_path[sizeof(g_voice_last_wav_path) - 1] = '\0';
    g_voice_has_recording = true;

    // Re-encode Codec2 from the loaded WAV is deferred until Send.
    // For now, just let the user Play it. The Codec2 data from the
    // original recording session may still be in MeckVoice if it
    // was the most recent encode.

    if (lbl_voice_status) {
        const char *slash = strrchr(path, '/');
        const char *base = slash ? slash + 1 : path;
        char buf[64];
        snprintf(buf, sizeof(buf), "Loaded: %s", base);
        lv_label_set_text(lbl_voice_status, buf);
        lv_obj_set_style_text_color(lbl_voice_status,
            lv_palette_main(LV_PALETTE_TEAL), 0);
    }
    voice_update_ui();
    printf("Voice: loaded previous recording %s\n", path);
}

// Refresh the previous recordings list on the record screen
static void voice_prev_refresh(void) {
    if (!obj_voice_prev_scroll) return;
    lv_obj_clean(obj_voice_prev_scroll);

    if (!p4_sdcard_is_mounted()) return;

    // Scan /sdcard/voice/ for voice_*.wav files
    DIR *dir = opendir(VOICE_FOLDER);
    if (!dir) return;

    // Collect filenames (max 16 most recent)
    static char paths[16][96];
    int count = 0;

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL && count < 16) {
        if (ent->d_type != DT_REG) continue;
        if (strncmp(ent->d_name, "voice_", 6) != 0) continue;
        const char *ext = strrchr(ent->d_name, '.');
        if (!ext || strcmp(ext, ".wav") != 0) continue;

        snprintf(paths[count], sizeof(paths[count]), "%s/%s",
                 VOICE_FOLDER, ent->d_name);
        count++;
    }
    closedir(dir);

    if (count == 0) {
        lv_obj_t *empty = lv_label_create(obj_voice_prev_scroll);
        lv_label_set_text(empty, "No previous recordings");
        lv_obj_set_style_text_color(empty, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(empty, &meck_montserrat_14, 0);
        lv_obj_set_pos(empty, 10, 5);
        return;
    }

    // Sort newest first (filenames contain timestamps, reverse alpha works)
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (strcmp(paths[i], paths[j]) < 0) {
                char tmp[96];
                memcpy(tmp, paths[i], 96);
                memcpy(paths[i], paths[j], 96);
                memcpy(paths[j], tmp, 96);
            }
        }
    }

    int y = 0;
    for (int i = 0; i < count; i++) {
        const char *slash = strrchr(paths[i], '/');
        const char *base = slash ? slash + 1 : paths[i];

        // Get file size for duration estimate
        struct stat st;
        int durSec = 0;
        if (stat(paths[i], &st) == 0) {
            // 44100Hz stereo 16-bit = 176400 bytes/sec
            durSec = (int)((st.st_size - 44) / 176400);
        }

        lv_obj_t *row = lv_button_create(obj_voice_prev_scroll);
        lv_obj_set_size(row, SCREEN_WIDTH - 70, 40);
        lv_obj_set_pos(row, 5, y);
        lv_obj_set_style_bg_color(row, lv_color_make(20, 20, 30), 0);
        lv_obj_set_style_radius(row, 6, 0);
        lv_obj_add_event_cb(row, on_voice_prev_row_tap, LV_EVENT_CLICKED,
                            (void*)paths[i]);  // static lifetime

        lv_obj_t *name = lv_label_create(row);
        char label[64];
        snprintf(label, sizeof(label), "%s  (%ds)", base, durSec);
        lv_label_set_text(name, label);
        lv_obj_set_style_text_color(name, lv_color_white(), 0);
        meck_set_font(name, &meck_montserrat_14, 0);
        lv_obj_align(name, LV_ALIGN_LEFT_MID, 5, 0);

        y += 44;
    }
}

static void on_voice_contact_cancel(lv_event_t *e) {
    (void)e;
    if (obj_voice_contact_panel) {
        lv_obj_del(obj_voice_contact_panel);
        obj_voice_contact_panel = NULL;
        obj_voice_contact_scroll = NULL;
    }
}

static void on_voice_contact_select(lv_event_t *e) {
    int contact_idx = (int)(intptr_t)lv_event_get_user_data(e);
    meck_request_voice_send(contact_idx);

    ContactInfo ci;
    Meck *mesh = meck_get_instance();
    if (mesh && mesh->getContactByIdx((uint32_t)contact_idx, ci)) {
        printf("Voice: sending to %s\n", ci.name);
    }

    on_voice_contact_cancel(NULL);

    // Track send completion: ACK wait (up to 15s) + first packet at 1.5s
    // + subsequent packets 3s apart. Use a generous estimate.
    MeckVoice* voice = meck_get_voice_instance();
    int numPkts = voice ? voice->getPacketCount() : 1;
    // Worst case: 15s ACK wait + 1.5s + (numPkts-1)*3s + 2s margin
    uint32_t maxDelay = 15000 + 1500 + (uint32_t)(numPkts - 1) * 3000 + 2000;
    g_voice_send_complete_ms = (uint32_t)(esp_timer_get_time() / 1000ULL) + maxDelay;
    g_voice_send_in_flight = true;

    if (lbl_voice_status) {
        char buf[64];
        snprintf(buf, sizeof(buf), "Sending to %s...", ci.name);
        lv_label_set_text(lbl_voice_status, buf);
        lv_obj_set_style_text_color(lbl_voice_status,
            lv_palette_main(LV_PALETTE_ORANGE), 0);
    }
}

static void on_voice_send_tap(lv_event_t *e) {
    (void)e;
    Meck *mesh = meck_get_instance();
    if (!mesh || !scr_voice) return;

    MeckVoice* voice = meck_get_voice_instance();
    if (!voice) return;

    // If Codec2 isn't valid but we have a loaded WAV (previous recording),
    // load it into the record buffer and encode now.
    if (!voice->isCodec2Valid() && g_voice_has_recording
        && g_voice_last_wav_path[0] != '\0') {
        if (lbl_voice_status) {
            lv_label_set_text(lbl_voice_status, "Encoding...");
            lv_obj_set_style_text_color(lbl_voice_status,
                lv_palette_main(LV_PALETTE_ORANGE), 0);
        }
        printf("Voice: loading WAV for send: %s\n", g_voice_last_wav_path);
        if (voice->loadFromWav(g_voice_last_wav_path)) {
            voice->encodeCodec2();
        }
    }

    if (!voice->isCodec2Valid()) {
        if (lbl_voice_status) {
            lv_label_set_text(lbl_voice_status, "Record or select a message first");
            lv_obj_set_style_text_color(lbl_voice_status,
                lv_palette_main(LV_PALETTE_ORANGE), 0);
        }
        printf("Voice: send blocked, Codec2 not valid\n");
        return;
    }

    // Clean up any existing picker
    if (obj_voice_contact_panel) {
        lv_obj_del(obj_voice_contact_panel);
        obj_voice_contact_panel = NULL;
    }

    // Build contact picker overlay (same pattern as channel share)
    obj_voice_contact_panel = lv_obj_create(scr_voice);
    lv_obj_set_size(obj_voice_contact_panel, SCREEN_WIDTH - 40, SCREEN_HEIGHT - 80);
    lv_obj_align(obj_voice_contact_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(obj_voice_contact_panel, lv_color_make(15, 15, 25), 0);
    lv_obj_set_style_bg_opa(obj_voice_contact_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(obj_voice_contact_panel, 16, 0);
    lv_obj_set_style_border_width(obj_voice_contact_panel, 1, 0);
    lv_obj_set_style_border_color(obj_voice_contact_panel,
        lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_pad_all(obj_voice_contact_panel, 10, 0);
    lv_obj_clear_flag(obj_voice_contact_panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(obj_voice_contact_panel);
    lv_label_set_text(title, "Send voice to:");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_18, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 5, 5);

    lv_obj_t *btn_cancel = lv_button_create(obj_voice_contact_panel);
    lv_obj_set_size(btn_cancel, 80, 40);
    lv_obj_align(btn_cancel, LV_ALIGN_TOP_RIGHT, -5, 0);
    lv_obj_set_style_bg_color(btn_cancel, lv_color_make(80, 30, 30), 0);
    lv_obj_set_style_radius(btn_cancel, 8, 0);
    lv_obj_t *cancel_lbl = lv_label_create(btn_cancel);
    lv_label_set_text(cancel_lbl, "Cancel");
    lv_obj_set_style_text_color(cancel_lbl, lv_color_white(), 0);
    meck_set_font(cancel_lbl, &meck_montserrat_14, 0);
    lv_obj_center(cancel_lbl);
    lv_obj_add_event_cb(btn_cancel, on_voice_contact_cancel, LV_EVENT_CLICKED, NULL);

    obj_voice_contact_scroll = lv_obj_create(obj_voice_contact_panel);
    lv_obj_update_layout(obj_voice_contact_panel);  // resolve size before query
    lv_obj_set_size(obj_voice_contact_scroll,
                    lv_obj_get_width(obj_voice_contact_panel) - 20,
                    lv_obj_get_height(obj_voice_contact_panel) - 60);
    lv_obj_align(obj_voice_contact_scroll, LV_ALIGN_BOTTOM_MID, 0, -5);
    lv_obj_set_style_bg_opa(obj_voice_contact_scroll, 0, 0);
    lv_obj_set_style_border_width(obj_voice_contact_scroll, 0, 0);
    lv_obj_set_style_pad_all(obj_voice_contact_scroll, 0, 0);
    lv_obj_set_scroll_dir(obj_voice_contact_scroll, LV_DIR_VER);

    int num_contacts = mesh->getNumContacts();
    // Type distribution diagnostic
    int type_counts[8] = {};
    for (int ci = 0; ci < num_contacts && ci < 500; ci++) {
        ContactInfo tc;
        if (mesh->getContactByIdx(ci, tc) && tc.type < 8) type_counts[tc.type]++;
    }
    printf("Voice send: %d contacts (types: 0=%d 1/chat=%d 2/rptr=%d 3/room=%d 4=%d 5=%d), scroll=%dx%d\n",
           num_contacts,
           type_counts[0], type_counts[1], type_counts[2], type_counts[3],
           type_counts[4], type_counts[5],
           (int)lv_obj_get_width(obj_voice_contact_scroll),
           (int)lv_obj_get_height(obj_voice_contact_scroll));
    int y = 0;

    // Two passes: favourites first, then non-favourites
    for (int pass = 0; pass < 2; pass++) {
        for (int ci = 0; ci < num_contacts; ci++) {
            ContactInfo contact;
            if (!mesh->getContactByIdx(ci, contact)) continue;
            if (contact.type != 1) continue;  // ADV_TYPE_CHAT only

            bool isFav = (contact.flags & 0x01) != 0;
            if (pass == 0 && !isFav) continue;  // pass 0: favs only
            if (pass == 1 && isFav) continue;    // pass 1: non-favs only

            lv_obj_t *row = lv_button_create(obj_voice_contact_scroll);
            lv_obj_set_size(row, lv_obj_get_width(obj_voice_contact_scroll) - 10, 50);
            lv_obj_set_pos(row, 5, y);
            lv_obj_set_style_bg_color(row, lv_color_make(25, 25, 35), 0);
            lv_obj_set_style_radius(row, 8, 0);
            lv_obj_add_event_cb(row, on_voice_contact_select, LV_EVENT_CLICKED,
                                (void*)(intptr_t)ci);

            lv_obj_t *name = lv_label_create(row);
            char display[48];
            if (isFav) {
                snprintf(display, sizeof(display), "%s %s", LV_SYMBOL_OK, contact.name);
            } else {
                snprintf(display, sizeof(display), "%s", contact.name);
            }
            lv_label_set_text(name, display);
            lv_obj_set_style_text_color(name,
                isFav ? lv_palette_main(LV_PALETTE_YELLOW) : lv_color_white(), 0);
            meck_set_font(name, &meck_montserrat_16, 0);
            lv_obj_align(name, LV_ALIGN_LEFT_MID, 10, 0);

            y += 55;
            printf("  [%d] %s (type=%d fav=%d)\n", ci, contact.name, contact.type, isFav);
        }
    }

    if (y == 0) {
        lv_obj_t *empty = lv_label_create(obj_voice_contact_scroll);
        lv_label_set_text(empty, "No chat contacts");
        lv_obj_set_style_text_color(empty, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(empty, &meck_montserrat_16, 0);
        lv_obj_set_pos(empty, 20, 20);
        printf("Voice send: no ADV_TYPE_CHAT contacts found\n");
    }
}

// ============================================================================
// Voice Inbox persistence — /sdcard/voice/inbox.cfg
// ============================================================================

static void voice_inbox_load(void) {
    g_voice_inbox_count = 0;
    g_voice_inbox_unread = 0;
    memset(g_voice_inbox, 0, sizeof(g_voice_inbox));

    if (!p4_sdcard_is_mounted()) return;

    FILE *f = fopen(VOICE_INBOX_CFG, "rb");
    if (!f) return;

    // Simple binary format: count (uint8_t) then entries
    uint8_t count = 0;
    if (fread(&count, 1, 1, f) != 1) { fclose(f); return; }
    if (count > VOICE_INBOX_MAX) count = VOICE_INBOX_MAX;

    for (int i = 0; i < count; i++) {
        VoiceInboxEntry& e = g_voice_inbox[i];
        if (fread(e.senderName, 1, 32, f) != 32) break;
        if (fread(e.filename, 1, 48, f) != 48) break;
        if (fread(&e.timestamp, 4, 1, f) != 1) break;
        if (fread(&e.durationSec, 1, 1, f) != 1) break;
        uint8_t played = 0;
        if (fread(&played, 1, 1, f) != 1) break;
        e.played = (played != 0);
        e.valid = true;
        g_voice_inbox_count++;
        if (!e.played) g_voice_inbox_unread++;
    }
    fclose(f);
    printf("Voice inbox: loaded %d entries (%d unread)\n",
           g_voice_inbox_count, g_voice_inbox_unread);
}

static void voice_inbox_save(void) {
    if (!p4_sdcard_is_mounted()) return;

    struct stat st;
    if (stat("/sdcard/voice", &st) != 0) mkdir("/sdcard/voice", 0755);

    FILE *f = fopen(VOICE_INBOX_CFG, "wb");
    if (!f) return;

    uint8_t count = (uint8_t)g_voice_inbox_count;
    fwrite(&count, 1, 1, f);
    for (int i = 0; i < count; i++) {
        VoiceInboxEntry& e = g_voice_inbox[i];
        fwrite(e.senderName, 1, 32, f);
        fwrite(e.filename, 1, 48, f);
        fwrite(&e.timestamp, 4, 1, f);
        fwrite(&e.durationSec, 1, 1, f);
        uint8_t played = e.played ? 1 : 0;
        fwrite(&played, 1, 1, f);
    }
    fclose(f);
}

static void voice_inbox_add(const char *senderName, const char *wavPath,
                            uint32_t timestamp, uint8_t durationSec) {
    // If full, drop oldest
    if (g_voice_inbox_count >= VOICE_INBOX_MAX) {
        memmove(&g_voice_inbox[0], &g_voice_inbox[1],
                sizeof(VoiceInboxEntry) * (VOICE_INBOX_MAX - 1));
        g_voice_inbox_count = VOICE_INBOX_MAX - 1;
    }

    VoiceInboxEntry& e = g_voice_inbox[g_voice_inbox_count];
    memset(&e, 0, sizeof(e));
    strncpy(e.senderName, senderName, 31);
    // Extract just the filename from the full path
    const char *slash = strrchr(wavPath, '/');
    const char *base = slash ? slash + 1 : wavPath;
    strncpy(e.filename, base, 47);
    e.timestamp = timestamp;
    e.durationSec = durationSec;
    e.played = false;
    e.valid = true;
    g_voice_inbox_count++;
    g_voice_inbox_unread++;

    voice_inbox_save();
    printf("Voice inbox: added '%s' from %s (%ds), %d total, %d unread\n",
           base, senderName, durationSec,
           g_voice_inbox_count, g_voice_inbox_unread);
}

static void voice_inbox_update_badge(void) {
    if (!lbl_voice_inbox_badge) return;
    if (g_voice_inbox_unread > 0) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", g_voice_inbox_unread);
        lv_label_set_text(lbl_voice_inbox_badge, buf);
        lv_obj_clear_flag(lbl_voice_inbox_badge, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(lbl_voice_inbox_badge, LV_OBJ_FLAG_HIDDEN);
    }
}

// ============================================================================
// Voice Inbox screen — list of received voice messages
// ============================================================================

static void on_voice_inbox_row_tap(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= g_voice_inbox_count) return;

    VoiceInboxEntry& entry = g_voice_inbox[idx];
    char path[128];
    snprintf(path, sizeof(path), "%s/%s", VOICE_INBOX_FOLDER, entry.filename);

    // Mark as played
    if (!entry.played) {
        entry.played = true;
        g_voice_inbox_unread--;
        if (g_voice_inbox_unread < 0) g_voice_inbox_unread = 0;
        voice_inbox_save();
        voice_inbox_update_badge();
    }

    // Play the message
    printf("Voice inbox: playing [%d] %s from %s\n", idx, path, entry.senderName);
    meck_audio_set_volume_pct(70);
    meck_audio_codec_wake();
    meck_audio_play_file(path, 0);
    meck_audio_resume();
}

static void voice_inbox_refresh(void) {
    if (!obj_voice_inbox_scroll) return;

    // Clear existing rows
    lv_obj_clean(obj_voice_inbox_scroll);

    if (g_voice_inbox_count == 0) {
        lv_obj_t *empty = lv_label_create(obj_voice_inbox_scroll);
        lv_label_set_text(empty, "No voice messages");
        lv_obj_set_style_text_color(empty, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(empty, &meck_montserrat_18, 0);
        lv_obj_set_pos(empty, 20, 20);
        return;
    }

    // Build rows newest-first
    int y = 0;
    for (int i = g_voice_inbox_count - 1; i >= 0; i--) {
        VoiceInboxEntry& e = g_voice_inbox[i];
        if (!e.valid) continue;

        lv_obj_t *row = lv_button_create(obj_voice_inbox_scroll);
        lv_obj_set_size(row, lv_obj_get_width(obj_voice_inbox_scroll) - 10, 65);
        lv_obj_set_pos(row, 5, y);
        lv_obj_set_style_bg_color(row, lv_color_make(20, 20, 30), 0);
        lv_obj_set_style_radius(row, 8, 0);
        lv_obj_add_event_cb(row, on_voice_inbox_row_tap, LV_EVENT_CLICKED,
                            (void*)(intptr_t)i);

        // Unplayed indicator (green dot)
        if (!e.played) {
            lv_obj_t *dot = lv_obj_create(row);
            lv_obj_set_size(dot, 10, 10);
            lv_obj_align(dot, LV_ALIGN_LEFT_MID, 5, 0);
            lv_obj_set_style_bg_color(dot, lv_palette_main(LV_PALETTE_GREEN), 0);
            lv_obj_set_style_radius(dot, 5, 0);
            lv_obj_set_style_border_width(dot, 0, 0);
        }

        // Sender name
        lv_obj_t *name = lv_label_create(row);
        lv_label_set_text(name, e.senderName);
        lv_obj_set_style_text_color(name, lv_color_white(), 0);
        meck_set_font(name, &meck_montserrat_16, 0);
        lv_obj_align(name, LV_ALIGN_LEFT_MID, e.played ? 10 : 22, -10);

        // Duration
        char meta[32];
        snprintf(meta, sizeof(meta), "%ds", e.durationSec);
        lv_obj_t *dur = lv_label_create(row);
        lv_label_set_text(dur, meta);
        lv_obj_set_style_text_color(dur, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(dur, &meck_montserrat_14, 0);
        lv_obj_align(dur, LV_ALIGN_LEFT_MID, e.played ? 10 : 22, 12);

        // Time
        if (e.timestamp > 0) {
            char timebuf[24];
            format_local_time(e.timestamp, timebuf, sizeof(timebuf));
            lv_obj_t *tlbl = lv_label_create(row);
            lv_label_set_text(tlbl, timebuf);
            lv_obj_set_style_text_color(tlbl, lv_palette_main(LV_PALETTE_GREY), 0);
            meck_set_font(tlbl, &meck_montserrat_14, 0);
            lv_obj_align(tlbl, LV_ALIGN_RIGHT_MID, -10, 0);
        }

        y += 70;
    }
}

static void create_voice_inbox_screen(void) {
    scr_voice_inbox = lv_obj_create(NULL);
    lock_screen_scroll(scr_voice_inbox);
    lv_obj_set_style_bg_color(scr_voice_inbox, lv_color_black(), 0);
    screen_attach_clock_battery(scr_voice_inbox, 12, &meck_montserrat_24, 30);

    lv_obj_t *btn_back = lv_button_create(scr_voice_inbox);
    lv_obj_set_size(btn_back, 80, 50);
    lv_obj_set_pos(btn_back, 10, 25);
    lv_obj_set_style_bg_opa(btn_back, 0, 0);
    lv_obj_t *back_lbl = lv_label_create(btn_back);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(back_lbl, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(back_lbl, &meck_montserrat_24, 0);
    lv_obj_center(back_lbl);
    lv_obj_add_event_cb(btn_back, [](lv_event_t *e) {
        if (scr_voice_landing) lv_screen_load(scr_voice_landing);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr_voice_inbox);
    lv_label_set_text(title, "Voice Inbox");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 100, 30);

    // Scrollable message list
    obj_voice_inbox_scroll = lv_obj_create(scr_voice_inbox);
    lv_obj_set_size(obj_voice_inbox_scroll, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 90);
    lv_obj_set_pos(obj_voice_inbox_scroll, 10, 80);
    lv_obj_set_style_bg_opa(obj_voice_inbox_scroll, 0, 0);
    lv_obj_set_style_border_width(obj_voice_inbox_scroll, 0, 0);
    lv_obj_set_style_pad_all(obj_voice_inbox_scroll, 0, 0);
    lv_obj_set_scroll_dir(obj_voice_inbox_scroll, LV_DIR_VER);
}

// ============================================================================
// Voice Landing screen — Inbox / Record picker
// ============================================================================

static void create_voice_landing_screen(void) {
    scr_voice_landing = lv_obj_create(NULL);
    lock_screen_scroll(scr_voice_landing);
    lv_obj_set_style_bg_color(scr_voice_landing, lv_color_black(), 0);
    screen_attach_clock_battery(scr_voice_landing, 12, &meck_montserrat_24, 30);

    lv_obj_t *btn_back = lv_button_create(scr_voice_landing);
    lv_obj_set_size(btn_back, 80, 50);
    lv_obj_set_pos(btn_back, 10, 25);
    lv_obj_set_style_bg_opa(btn_back, 0, 0);
    lv_obj_t *back_lbl = lv_label_create(btn_back);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(back_lbl, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(back_lbl, &meck_montserrat_24, 0);
    lv_obj_center(back_lbl);
    lv_obj_add_event_cb(btn_back, [](lv_event_t *e) {
        if (scr_home) lv_screen_load(scr_home);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr_voice_landing);
    lv_label_set_text(title, "Voice");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    // Inbox button — large, left side
    btn_voice_inbox = lv_button_create(scr_voice_landing);
    lv_obj_set_size(btn_voice_inbox, 220, 200);
    lv_obj_align(btn_voice_inbox, LV_ALIGN_CENTER, -120, 20);
    lv_obj_set_style_bg_color(btn_voice_inbox,
        lv_color_make(20, 40, 60), 0);
    lv_obj_set_style_radius(btn_voice_inbox, 16, 0);
    lv_obj_set_style_border_width(btn_voice_inbox, 2, 0);
    lv_obj_set_style_border_color(btn_voice_inbox,
        lv_palette_main(LV_PALETTE_TEAL), 0);

    lv_obj_t *inbox_icon = lv_label_create(btn_voice_inbox);
    lv_label_set_text(inbox_icon, LV_SYMBOL_ENVELOPE);
    lv_obj_set_style_text_color(inbox_icon, lv_palette_main(LV_PALETTE_TEAL), 0);
    meck_set_font(inbox_icon, &meck_montserrat_32, 0);
    lv_obj_align(inbox_icon, LV_ALIGN_CENTER, 0, -20);

    lv_obj_t *inbox_lbl = lv_label_create(btn_voice_inbox);
    lv_label_set_text(inbox_lbl, "Inbox");
    lv_obj_set_style_text_color(inbox_lbl, lv_color_white(), 0);
    meck_set_font(inbox_lbl, &meck_montserrat_22, 0);
    lv_obj_align(inbox_lbl, LV_ALIGN_CENTER, 0, 25);

    // Unread badge (top-right of inbox button)
    lbl_voice_inbox_badge = lv_label_create(btn_voice_inbox);
    lv_label_set_text(lbl_voice_inbox_badge, "0");
    lv_obj_set_style_text_color(lbl_voice_inbox_badge, lv_color_white(), 0);
    lv_obj_set_style_bg_color(lbl_voice_inbox_badge,
        lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_bg_opa(lbl_voice_inbox_badge, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(lbl_voice_inbox_badge, 10, 0);
    lv_obj_set_style_pad_hor(lbl_voice_inbox_badge, 8, 0);
    lv_obj_set_style_pad_ver(lbl_voice_inbox_badge, 3, 0);
    meck_set_font(lbl_voice_inbox_badge, &meck_montserrat_16, 0);
    lv_obj_align(lbl_voice_inbox_badge, LV_ALIGN_TOP_RIGHT, 5, -5);
    lv_obj_add_flag(lbl_voice_inbox_badge, LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_event_cb(btn_voice_inbox, [](lv_event_t *e) {
        voice_inbox_refresh();
        if (scr_voice_inbox) lv_screen_load(scr_voice_inbox);
    }, LV_EVENT_CLICKED, NULL);

    // Record button — large, right side
    btn_voice_record_nav = lv_button_create(scr_voice_landing);
    lv_obj_set_size(btn_voice_record_nav, 220, 200);
    lv_obj_align(btn_voice_record_nav, LV_ALIGN_CENTER, 120, 20);
    lv_obj_set_style_bg_color(btn_voice_record_nav,
        lv_color_make(50, 20, 20), 0);
    lv_obj_set_style_radius(btn_voice_record_nav, 16, 0);
    lv_obj_set_style_border_width(btn_voice_record_nav, 2, 0);
    lv_obj_set_style_border_color(btn_voice_record_nav,
        lv_palette_main(LV_PALETTE_RED), 0);

    lv_obj_t *rec_icon = lv_label_create(btn_voice_record_nav);
    lv_label_set_text(rec_icon, LV_SYMBOL_AUDIO);
    lv_obj_set_style_text_color(rec_icon, lv_palette_main(LV_PALETTE_RED), 0);
    meck_set_font(rec_icon, &meck_montserrat_32, 0);
    lv_obj_align(rec_icon, LV_ALIGN_CENTER, 0, -20);

    lv_obj_t *rec_lbl = lv_label_create(btn_voice_record_nav);
    lv_label_set_text(rec_lbl, "Record");
    lv_obj_set_style_text_color(rec_lbl, lv_color_white(), 0);
    meck_set_font(rec_lbl, &meck_montserrat_22, 0);
    lv_obj_align(rec_lbl, LV_ALIGN_CENTER, 0, 25);

    lv_obj_add_event_cb(btn_voice_record_nav, [](lv_event_t *e) {
        // Reset stale state if MeckVoice has no recording
        MeckVoice* v = meck_get_voice_instance();
        if (v && v->getRecSamples() == 0 && g_voice_last_wav_path[0] == '\0') {
            g_voice_has_recording = false;
        }
        voice_update_ui();
        voice_prev_refresh();
        if (scr_voice) lv_screen_load(scr_voice);
    }, LV_EVENT_CLICKED, NULL);
}

// ============================================================================
// Voice Record screen — recording, review, send
// ============================================================================

static void create_voice_record_screen(void) {
    scr_voice = lv_obj_create(NULL);
    lock_screen_scroll(scr_voice);
    lv_obj_set_style_bg_color(scr_voice, lv_color_black(), 0);
    screen_attach_clock_battery(scr_voice, 12, &meck_montserrat_24, 30);

    // Back button (returns to voice landing, not home)
    lv_obj_t *btn_back = lv_button_create(scr_voice);
    lv_obj_set_size(btn_back, 80, 50);
    lv_obj_set_pos(btn_back, 10, 25);
    lv_obj_set_style_bg_opa(btn_back, 0, 0);
    lv_obj_t *back_lbl = lv_label_create(btn_back);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(back_lbl, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(back_lbl, &meck_montserrat_24, 0);
    lv_obj_center(back_lbl);
    lv_obj_add_event_cb(btn_back, [](lv_event_t *e) {
        // Stop recording if active
        MeckVoice* v = meck_get_voice_instance();
        if (v && v->isRecording()) v->stopRecording();
        if (scr_voice_landing) lv_screen_load(scr_voice_landing);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr_voice);
    lv_label_set_text(title, "Voice");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    // Status label
    lbl_voice_status = lv_label_create(scr_voice);
    lv_label_set_text(lbl_voice_status, "Tap Record to start");
    lv_obj_set_style_text_color(lbl_voice_status,
        lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(lbl_voice_status, &meck_montserrat_22, 0);
    lv_obj_align(lbl_voice_status, LV_ALIGN_TOP_MID, 0, 120);

    // Timer label (shown during recording)
    lbl_voice_timer = lv_label_create(scr_voice);
    lv_label_set_text(lbl_voice_timer, "");
    lv_obj_set_style_text_color(lbl_voice_timer,
        lv_palette_main(LV_PALETTE_RED), 0);
    meck_set_font(lbl_voice_timer, &meck_montserrat_28, 0);
    lv_obj_align(lbl_voice_timer, LV_ALIGN_CENTER, 0, -60);

    // Record button — large, centered
    btn_voice_record = lv_button_create(scr_voice);
    lv_obj_set_size(btn_voice_record, 200, 80);
    lv_obj_align(btn_voice_record, LV_ALIGN_CENTER, 0, 40);
    lv_obj_set_style_bg_color(btn_voice_record,
        lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_radius(btn_voice_record, 16, 0);
    lbl_voice_record_btn = lv_label_create(btn_voice_record);
    lv_label_set_text(lbl_voice_record_btn, "Record");
    lv_obj_set_style_text_color(lbl_voice_record_btn, lv_color_white(), 0);
    meck_set_font(lbl_voice_record_btn, &meck_montserrat_28, 0);
    lv_obj_center(lbl_voice_record_btn);
    lv_obj_add_event_cb(btn_voice_record, on_voice_record_tap,
                        LV_EVENT_CLICKED, NULL);

    // Play button (hidden until recording complete)
    btn_voice_play = lv_button_create(scr_voice);
    lv_obj_set_size(btn_voice_play, 120, 60);
    lv_obj_align(btn_voice_play, LV_ALIGN_CENTER, -130, 150);
    lv_obj_set_style_bg_color(btn_voice_play,
        lv_palette_main(LV_PALETTE_TEAL), 0);
    lv_obj_set_style_radius(btn_voice_play, 12, 0);
    lv_obj_t *play_lbl = lv_label_create(btn_voice_play);
    lv_label_set_text(play_lbl, "Play");
    lv_obj_set_style_text_color(play_lbl, lv_color_white(), 0);
    meck_set_font(play_lbl, &meck_montserrat_22, 0);
    lv_obj_center(play_lbl);
    lv_obj_add_event_cb(btn_voice_play, on_voice_play_tap,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(btn_voice_play, LV_OBJ_FLAG_HIDDEN);

    // Discard button (hidden until recording complete)
    btn_voice_discard = lv_button_create(scr_voice);
    lv_obj_set_size(btn_voice_discard, 120, 60);
    lv_obj_align(btn_voice_discard, LV_ALIGN_CENTER, 0, 150);
    lv_obj_set_style_bg_color(btn_voice_discard,
        lv_color_make(80, 30, 30), 0);
    lv_obj_set_style_radius(btn_voice_discard, 12, 0);
    lv_obj_t *discard_lbl = lv_label_create(btn_voice_discard);
    lv_label_set_text(discard_lbl, "Discard");
    lv_obj_set_style_text_color(discard_lbl, lv_color_white(), 0);
    meck_set_font(discard_lbl, &meck_montserrat_22, 0);
    lv_obj_center(discard_lbl);
    lv_obj_add_event_cb(btn_voice_discard, on_voice_discard_tap,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(btn_voice_discard, LV_OBJ_FLAG_HIDDEN);

    // Send button (hidden until recording + Codec2 encode complete)
    btn_voice_send = lv_button_create(scr_voice);
    lv_obj_set_size(btn_voice_send, 120, 60);
    lv_obj_align(btn_voice_send, LV_ALIGN_CENTER, 130, 150);
    lv_obj_set_style_bg_color(btn_voice_send,
        lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(btn_voice_send, 12, 0);
    lv_obj_t *send_lbl = lv_label_create(btn_voice_send);
    lv_label_set_text(send_lbl, "Send");
    lv_obj_set_style_text_color(send_lbl, lv_color_black(), 0);
    meck_set_font(send_lbl, &meck_montserrat_22, 0);
    lv_obj_center(send_lbl);
    lv_obj_add_event_cb(btn_voice_send, on_voice_send_tap,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(btn_voice_send, LV_OBJ_FLAG_HIDDEN);

    // Previous recordings header — positioned below the action buttons
    lv_obj_t *prev_hdr = lv_label_create(scr_voice);
    lv_label_set_text(prev_hdr, "Previous recordings:");
    lv_obj_set_style_text_color(prev_hdr, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(prev_hdr, &meck_montserrat_14, 0);
    lv_obj_set_pos(prev_hdr, 20, 810);

    // Scrollable list of previous recordings (below header to bottom)
    obj_voice_prev_scroll = lv_obj_create(scr_voice);
    lv_obj_set_pos(obj_voice_prev_scroll, 20, 840);
    lv_obj_set_size(obj_voice_prev_scroll, SCREEN_WIDTH - 40, SCREEN_HEIGHT - 860);
    lv_obj_set_style_bg_opa(obj_voice_prev_scroll, 0, 0);
    lv_obj_set_style_border_width(obj_voice_prev_scroll, 0, 0);
    lv_obj_set_style_pad_all(obj_voice_prev_scroll, 0, 0);
    lv_obj_set_scroll_dir(obj_voice_prev_scroll, LV_DIR_VER);
}

static void create_channel_picker_screen() {
    scr_channel_picker = lv_obj_create(NULL);
    lock_screen_scroll(scr_channel_picker);
    lv_obj_set_style_bg_color(scr_channel_picker, lv_color_black(), 0);
    screen_attach_clock_battery(scr_channel_picker, 3, &meck_montserrat_24, 30);
    // Move clock under the battery so the "Channels" title doesn't overlap it
    // (mirrors the DM inbox / message thread / WiFi / debug screens).
    if (lbl_screen_clock[3]) {
        lv_obj_set_style_text_align(lbl_screen_clock[3], LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(lbl_screen_clock[3], LV_ALIGN_TOP_RIGHT, -15, 55);
    }

    create_back_button(scr_channel_picker);

    lv_obj_t *title = lv_label_create(scr_channel_picker);
    lv_label_set_text(title, "Channels");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    obj_ch_picker_scroll = lv_obj_create(scr_channel_picker);
    lv_obj_set_size(obj_ch_picker_scroll, SCREEN_WIDTH, SCREEN_HEIGHT - 90);
    lv_obj_set_pos(obj_ch_picker_scroll, 20, 90);
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
    // Lock the panel down — no scrolling, no scrollbars. Otherwise edge
    // taps near the keyboard's left/right corners pan the whole screen.
    lv_obj_clear_flag(obj_ch_add_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(obj_ch_add_panel, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *add_title = lv_label_create(obj_ch_add_panel);
    lv_label_set_text(add_title, "Add Channel");
    lv_obj_set_style_text_color(add_title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(add_title, &meck_montserrat_22, 0);
    lv_obj_align(add_title, LV_ALIGN_TOP_MID, 0, 50);

    lv_obj_t *hint = lv_label_create(obj_ch_add_panel);
    lv_label_set_text(hint, "# = public, no # = private (e.g. #sydney)");
    lv_obj_set_style_text_color(hint, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(hint, &meck_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 90);

    ta_ch_add = lv_textarea_create(obj_ch_add_panel);
    lv_obj_set_size(ta_ch_add, SCREEN_WIDTH - 40, 50);
    lv_obj_align(ta_ch_add, LV_ALIGN_TOP_MID, 0, 120);
    lv_textarea_set_one_line(ta_ch_add, true);
    lv_textarea_set_max_length(ta_ch_add, 30);
    lv_textarea_set_text(ta_ch_add, "");
    lv_obj_set_style_bg_color(ta_ch_add, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_ch_add, lv_color_white(), 0);
    meck_set_font(ta_ch_add, &meck_montserrat_18, 0);
    lv_obj_set_style_border_color(ta_ch_add, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_border_color(ta_ch_add, lv_color_white(),     LV_PART_CURSOR);
    lv_obj_set_style_border_width(ta_ch_add, 2,                     LV_PART_CURSOR);
    lv_obj_set_style_border_side(ta_ch_add,  LV_BORDER_SIDE_LEFT,   LV_PART_CURSOR);
    lv_obj_set_style_border_opa(ta_ch_add,   LV_OPA_COVER,          LV_PART_CURSOR);

    // Confirm button between the textarea and the keyboard. Provides an
    // obvious tap target for submitting the channel name — the keyboard's
    // enter key (LV_SYMBOL_NEW_LINE) also fires on_ch_add_save via
    // LV_EVENT_READY, but users don't recognise it as the confirm action.
    lv_obj_t *btn_ch_confirm = lv_button_create(obj_ch_add_panel);
    lv_obj_set_size(btn_ch_confirm, SCREEN_WIDTH - 40, 50);
    lv_obj_align(btn_ch_confirm, LV_ALIGN_TOP_MID, 0, 185);
    lv_obj_set_style_bg_color(btn_ch_confirm, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_radius(btn_ch_confirm, 8, 0);
    lv_obj_t *confirm_lbl = lv_label_create(btn_ch_confirm);
    lv_label_set_text(confirm_lbl, "Confirm");
    lv_obj_set_style_text_color(confirm_lbl, lv_color_black(), 0);
    meck_set_font(confirm_lbl, &meck_montserrat_22, 0);
    lv_obj_center(confirm_lbl);
    lv_obj_add_event_cb(btn_ch_confirm, on_ch_add_save, LV_EVENT_CLICKED, NULL);

    // Cancel button under Confirm. Without this the only way out of the
    // modal was the keyboard CANCEL event, which has no visible affordance.
    // Wired to on_ch_add_cancel, which just hides the panel.
    lv_obj_t *btn_ch_cancel = lv_button_create(obj_ch_add_panel);
    lv_obj_set_size(btn_ch_cancel, SCREEN_WIDTH - 40, 50);
    lv_obj_align(btn_ch_cancel, LV_ALIGN_TOP_MID, 0, 247);
    lv_obj_set_style_bg_color(btn_ch_cancel, lv_color_make(70, 70, 80), 0);
    lv_obj_set_style_radius(btn_ch_cancel, 8, 0);
    lv_obj_t *cancel_lbl = lv_label_create(btn_ch_cancel);
    lv_label_set_text(cancel_lbl, "Cancel");
    lv_obj_set_style_text_color(cancel_lbl, lv_color_white(), 0);
    meck_set_font(cancel_lbl, &meck_montserrat_22, 0);
    lv_obj_center(cancel_lbl);
    lv_obj_add_event_cb(btn_ch_cancel, on_ch_add_cancel, LV_EVENT_CLICKED, NULL);

    kb_ch_add = lv_keyboard_create(obj_ch_add_panel);
    meck_style_keyboard(kb_ch_add);
    lv_keyboard_set_textarea(kb_ch_add, ta_ch_add);
    lv_obj_add_event_cb(kb_ch_add, on_ch_add_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_ch_add, on_ch_add_kb_event, LV_EVENT_CANCEL, NULL);
    lv_obj_add_event_cb(kb_ch_add, on_kb_long_press,
                        LV_EVENT_LONG_PRESSED, NULL);
}

// ============================================================================
// DM inbox screen
// ----------------------------------------------------------------------------
// Lists contacts who have at least one DM (sent or received), sorted
// newest-first. Each row shows name + unread badge; tap opens
// load_dm_view(idx) which switches into scr_messages with DM mode active.
//
// Why a separate screen rather than reusing the contacts list with a
// filter: the contacts list is for browsing/managing all contacts (with
// favourite/delete affordances). The DM inbox is for picking back up a
// conversation. Different mental model, different visual density, much
// shorter list. Keeping them separate avoids modal complexity.
// ============================================================================

// Per-tap handler: open the conversation for the contact tied to this row.
// The contact index is encoded as the user_data on the row's click event.
static void on_dm_inbox_row_tap(lv_event_t *e) {
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    load_dm_view(idx);
}

static void refresh_dm_inbox_list() {
    Meck* mesh = meck_get_instance();
    if (!obj_dm_inbox_scroll || !mesh) return;

    lv_obj_clean(obj_dm_inbox_scroll);

    // Build a list of (contact_idx, newest_timestamp) for every contact
    // that has a ring with at least one message. Sort newest-first by
    // timestamp. MAX_CONTACTS=1500 but the count of rings is typically
    // small (only contacts the user has actually DM'd or been DM'd by),
    // so the scan + sort is cheap. Worst-case sizing is MAX_CONTACTS,
    // hence heap-allocated rather than on the stack — 1500 * 8 bytes
    // is 12KB which can blow the LVGL task's stack frame.
    struct Entry { int idx; uint32_t ts; };
    Entry* entries = (Entry*)heap_caps_calloc(MAX_CONTACTS, sizeof(Entry),
                                              MALLOC_CAP_SPIRAM);
    if (!entries) {
        printf("refresh_dm_inbox_list: PSRAM alloc failed\n");
        return;
    }

    int n = 0;
    for (int i = 0; i < MAX_CONTACTS; i++) {
        DMRing* r = g_dm_rings[i];
        if (!r || r->count == 0) continue;
        entries[n].idx = i;
        entries[n].ts  = dm_ring_newest_timestamp(i);
        n++;
    }

    // Simple insertion sort, descending by timestamp. For the small n
    // we expect (handful to a few dozen) this is fine — and stable, so
    // ties (rare with unique sender timestamps) keep their scan order.
    for (int i = 1; i < n; i++) {
        Entry cur = entries[i];
        int j = i - 1;
        while (j >= 0 && entries[j].ts < cur.ts) {
            entries[j + 1] = entries[j];
            j--;
        }
        entries[j + 1] = cur;
    }

    if (n == 0) {
        lv_obj_t *empty = lv_label_create(obj_dm_inbox_scroll);
        lv_label_set_text(empty, "No DMs yet.");
        lv_obj_set_style_text_color(empty, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(empty, &meck_montserrat_22, 0);
        lv_obj_align(empty, LV_ALIGN_TOP_MID, 0, 30);
        free(entries);
        return;
    }

    int y = 5;
    lv_color_t row_color = lv_palette_main(LV_PALETTE_CYAN);
    for (int i = 0; i < n; i++) {
        ContactInfo ci;
        if (!mesh->getContactByIdx((uint32_t)entries[i].idx, ci)) continue;
        DMRing* r = g_dm_rings[entries[i].idx];
        if (!r) continue;  // shouldn't happen, defensive

        lv_obj_t *btn = lv_button_create(obj_dm_inbox_scroll);
        lv_obj_set_size(btn, SCREEN_WIDTH - 40, 75);
        lv_obj_set_pos(btn, 0, y);
        lv_obj_set_style_bg_color(btn, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(btn, 12, 0);
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_border_color(btn, row_color, 0);
        lv_obj_add_event_cb(btn, on_dm_inbox_row_tap, LV_EVENT_CLICKED,
                            (void*)(intptr_t)entries[i].idx);

        // Contact name. Stripped of unrenderable chars (same treatment
        // the contacts list applies) so emoji-heavy names don't tofu.
        char clean_name[64];
        strip_unrenderable(ci.name, clean_name, sizeof(clean_name));

        lv_obj_t *lbl_name = lv_label_create(btn);
        lv_label_set_text(lbl_name, clean_name);
        lv_obj_set_style_text_color(lbl_name, lv_color_white(), 0);
        meck_set_font(lbl_name, &meck_montserrat_22, 0);
        lv_obj_align(lbl_name, LV_ALIGN_LEFT_MID, 10, 0);

        if (r->unread > 0) {
            lv_obj_t *lbl_unread = lv_label_create(btn);
            lv_label_set_text_fmt(lbl_unread, "%d new", r->unread);
            lv_obj_set_style_text_color(lbl_unread, lv_color_make(100, 255, 100), 0);
            meck_set_font(lbl_unread, &meck_montserrat_18, 0);
            lv_obj_align(lbl_unread, LV_ALIGN_RIGHT_MID, -10, 0);
        }

        y += 85;
    }

    free(entries);
}

static void create_dm_inbox_screen() {
    scr_dm_inbox = lv_obj_create(NULL);
    lock_screen_scroll(scr_dm_inbox);
    lv_obj_set_style_bg_color(scr_dm_inbox, lv_color_black(), 0);
    // Slot 8 (slots 0-5,7 used elsewhere; 6 is free but reserved). 8
    // groups visually with the message-related screens at slot 4 +5.
    screen_attach_clock_battery(scr_dm_inbox, 8, &meck_montserrat_24, 30);
    // Move clock under the battery so the long "Direct Messages" title
    // doesn't overlap it (mirrors the message thread / WiFi / debug screens).
    if (lbl_screen_clock[8]) {
        lv_obj_set_style_text_align(lbl_screen_clock[8], LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(lbl_screen_clock[8], LV_ALIGN_TOP_RIGHT, -15, 55);
    }

    lv_obj_t *btn_back = lv_button_create(scr_dm_inbox);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, goto_channel_picker, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr_dm_inbox);
    lv_label_set_text(title, "Direct Messages");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_CYAN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    // Notification tone button — opens the shared tone picker for the DM
    // slot (0xFF). Placed left of the battery/clock cluster in the header.
    lv_obj_t *btn_dm_tone = lv_button_create(scr_dm_inbox);
    lv_obj_set_size(btn_dm_tone, 52, 44);
    lv_obj_align(btn_dm_tone, LV_ALIGN_TOP_RIGHT, -160, 12);
    lv_obj_set_style_bg_color(btn_dm_tone, lv_color_make(30, 40, 55), 0);
    lv_obj_set_style_radius(btn_dm_tone, 8, 0);
    lv_obj_t *dm_tone_lbl = lv_label_create(btn_dm_tone);
    lv_label_set_text(dm_tone_lbl, LV_SYMBOL_AUDIO);
    lv_obj_set_style_text_color(dm_tone_lbl, lv_palette_main(LV_PALETTE_CYAN), 0);
    meck_set_font(dm_tone_lbl, &meck_montserrat_22, 0);
    lv_obj_center(dm_tone_lbl);
    lv_obj_add_event_cb(btn_dm_tone, on_dm_tone_tap, LV_EVENT_CLICKED, NULL);

    obj_dm_inbox_scroll = lv_obj_create(scr_dm_inbox);
    lv_obj_set_size(obj_dm_inbox_scroll, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 100);
    lv_obj_set_pos(obj_dm_inbox_scroll, 10, 90);
    lv_obj_set_style_bg_color(obj_dm_inbox_scroll, lv_color_make(10, 10, 15), 0);
    lv_obj_set_style_border_width(obj_dm_inbox_scroll, 0, 0);
    lv_obj_set_style_radius(obj_dm_inbox_scroll, 8, 0);
    lv_obj_set_style_pad_all(obj_dm_inbox_scroll, 10, 0);
    lv_obj_set_scroll_dir(obj_dm_inbox_scroll, LV_DIR_VER);
}

// Navigation entry-point: refresh the list and load the screen. Wired
// to the Direct Messages pseudo-row in the channel picker.
static void goto_dm_inbox(lv_event_t *e) {
    refresh_dm_inbox_list();
    if (scr_dm_inbox) lv_screen_load(scr_dm_inbox);
}

// ============================================================================
// Messages screen (lifted from old:908-1023)
// ============================================================================

// Capture the current channel draft when leaving the messages screen (any
// exit path). In-RAM only; gated by prefs->save_drafts and channel mode.
static void on_messages_screen_unload(lv_event_t *e) {
    LV_UNUSED(e);
    Meck* mesh = meck_get_instance();
    P4NodePrefs* prefs = mesh ? mesh->getNodePrefs() : nullptr;
    if (!prefs || prefs->save_drafts == 0) return;
    if (g_in_dm_mode || g_active_channel >= MAX_GROUP_CHANNELS) return;
    if (!ta_compose) return;
    const char* t = lv_textarea_get_text(ta_compose);
    strncpy(g_channel_drafts[g_active_channel], t ? t : "",
            sizeof(g_channel_drafts[0]) - 1);
    g_channel_drafts[g_active_channel][sizeof(g_channel_drafts[0]) - 1] = '\0';
}

static void create_messages_screen() {
    scr_messages = lv_obj_create(NULL);
    lv_obj_add_event_cb(scr_messages, on_messages_screen_unload,
                        LV_EVENT_SCREEN_UNLOAD_START, NULL);
    lock_screen_scroll(scr_messages);
    lv_obj_set_style_bg_color(scr_messages, lv_color_black(), 0);
    screen_attach_clock_battery(scr_messages, 4, &meck_montserrat_22, 30);
    // Move clock to right side (under battery) so long channel names don't overlap
    if (lbl_screen_clock[4]) {
        lv_obj_set_style_text_align(lbl_screen_clock[4], LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(lbl_screen_clock[4], LV_ALIGN_TOP_RIGHT, -15, 55);
    }

    // Back -> channel picker (NOT home)
    lv_obj_t *btn_back = lv_button_create(scr_messages);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, goto_messages_back, LV_EVENT_CLICKED, NULL);

    lbl_msg_channel_name = lv_label_create(scr_messages);
    lv_label_set_text(lbl_msg_channel_name, "#public");
    lv_obj_set_style_text_color(lbl_msg_channel_name, lv_palette_main(LV_PALETTE_CYAN), 0);
    meck_set_font(lbl_msg_channel_name, &meck_montserrat_22, 0);
    lv_obj_align(lbl_msg_channel_name, LV_ALIGN_TOP_LEFT, 120, 30);

    lbl_msg_channel_scope = lv_label_create(scr_messages);
    lv_label_set_text(lbl_msg_channel_scope, "");
    lv_obj_set_style_text_color(lbl_msg_channel_scope, lv_color_white(), 0);
    meck_set_font(lbl_msg_channel_scope, &meck_montserrat_14, 0);
    lv_obj_align(lbl_msg_channel_scope, LV_ALIGN_TOP_LEFT, 120, 58);

    obj_msg_scroll = lv_obj_create(scr_messages);
    lv_obj_set_size(obj_msg_scroll, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 160);
    lv_obj_set_pos(obj_msg_scroll, 10, 90);
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

    // Attachment + button (left of compose area, long-press to open menu)
    // Mirrors btn_send layout: same height, same y offset, height synced
    // to textarea in on_compose_size_changed.
    btn_attach = lv_button_create(scr_messages);
    lv_obj_set_size(btn_attach, 60, 50);
    lv_obj_align(btn_attach, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_set_style_bg_color(btn_attach, lv_color_make(50, 50, 60), 0);
    lv_obj_set_style_radius(btn_attach, 8, 0);
    lv_obj_t *attach_lbl = lv_label_create(btn_attach);
    lv_label_set_text(attach_lbl, "+");
    lv_obj_set_style_text_color(attach_lbl, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(attach_lbl, &meck_montserrat_28, 0);
    lv_obj_center(attach_lbl);
    lv_obj_add_event_cb(btn_attach, on_attach_long_press,
                        LV_EVENT_LONG_PRESSED, NULL);

    ta_compose = lv_textarea_create(scr_messages);
    lv_obj_set_width(ta_compose, SCREEN_WIDTH - 160);
    lv_obj_set_height(ta_compose, LV_SIZE_CONTENT);
    lv_obj_set_style_min_height(ta_compose, 50, 0);
    lv_obj_set_style_max_height(ta_compose, 150, 0);
    lv_obj_align(ta_compose, LV_ALIGN_BOTTOM_LEFT, 75, -10);
    lv_textarea_set_placeholder_text(ta_compose, "Type message...");
    // Multi-line so the keyboard's LV_SYMBOL_NEW_LINE (Enter) key inserts
    // a '\n' instead of being silently rejected. Sending is via the
    // on-screen btn_send or the keyboard's LV_SYMBOL_OK (✓), both routed
    // through on_send_clicked. Height is LV_SIZE_CONTENT with a 50 px min
    // and 150 px max — the textarea starts at one line, grows as the user
    // types into a second/third line, and caps at ~5 lines. The
    // on_compose_size_changed callback below keeps obj_msg_scroll in sync
    // by recomputing its height whenever the textarea resizes.
    lv_textarea_set_one_line(ta_compose, false);
    // Scrollbar off — auto-grow means content height never exceeds view
    // height in normal use, but suppressing the bar guarantees no jitter
    // even if a long single word ever hits the max-height clamp.
    lv_obj_set_scrollbar_mode(ta_compose, LV_SCROLLBAR_MODE_OFF);
    lv_textarea_set_max_length(ta_compose, 180);
    lv_obj_set_style_bg_color(ta_compose, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_compose, lv_color_white(), 0);
    meck_set_font(ta_compose, &meck_montserrat_16, 0);
    lv_obj_set_style_border_color(ta_compose, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_border_width(ta_compose, 1, 0);
    lv_obj_set_style_radius(ta_compose, 8, 0);
    // Cursor styling: LVGL's default cursor is drawn with the inherited
    // text colour as a left-side border, which against our dark textarea
    // background was effectively invisible. Make it a 2 px white bar so
    // the insertion point is clearly visible while typing.
    lv_obj_set_style_border_color(ta_compose, lv_color_white(),       LV_PART_CURSOR);
    lv_obj_set_style_border_width(ta_compose, 2,                       LV_PART_CURSOR);
    lv_obj_set_style_border_side(ta_compose,  LV_BORDER_SIDE_LEFT,     LV_PART_CURSOR);
    lv_obj_set_style_border_opa(ta_compose,   LV_OPA_COVER,            LV_PART_CURSOR);
    // Blink the cursor. start_cursor_blink() in LVGL's textarea reads
    // anim_duration on LV_PART_CURSOR: 0 leaves the cursor solid, non-zero
    // drives the on/off animation. Set it here rather than inheriting
    // whatever the theme happens to default to.
    lv_obj_set_style_anim_duration(ta_compose, 500, LV_PART_CURSOR);
    lv_obj_add_event_cb(ta_compose, on_compose_focused,      LV_EVENT_FOCUSED,      NULL);
    lv_obj_add_event_cb(ta_compose, on_compose_defocused,    LV_EVENT_DEFOCUSED,    NULL);
    lv_obj_add_event_cb(ta_compose, on_compose_size_changed, LV_EVENT_SIZE_CHANGED, NULL);

    btn_send = lv_button_create(scr_messages);
    lv_obj_set_size(btn_send, 70, 50);
    lv_obj_align(btn_send, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_bg_color(btn_send, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(btn_send, 8, 0);
    lv_obj_t *send_lbl = lv_label_create(btn_send);
    lv_label_set_text(send_lbl, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(send_lbl, lv_color_black(), 0);
    meck_set_font(send_lbl, &meck_montserrat_22, 0);
    lv_obj_center(send_lbl);
    lv_obj_add_event_cb(btn_send, on_send_clicked, LV_EVENT_CLICKED, NULL);

    kb_compose = lv_keyboard_create(scr_messages);
    meck_style_keyboard(kb_compose);
    lv_keyboard_set_textarea(kb_compose, ta_compose);
    lv_obj_add_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(kb_compose, on_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_compose, on_kb_event, LV_EVENT_CANCEL, NULL);
    lv_obj_add_event_cb(kb_compose, on_kb_long_press,
                        LV_EVENT_LONG_PRESSED, NULL);
    lv_obj_add_event_cb(kb_compose, on_kb_emoji_event,
                        LV_EVENT_PRESSED, NULL);
}

// ============================================================================
// Load a specific channel into the messages view
// ============================================================================

static void load_channel_view(uint8_t ch_idx) {
    g_active_channel = ch_idx;
    g_in_dm_mode = false;  // shared scr_messages — make sure we're not in DM mode
    Meck* mesh = meck_get_instance();

    if (lbl_msg_channel_name && mesh) {
        ChannelDetails ch;
        if (mesh->getChannel(ch_idx, ch) && ch.name[0] != '\0') {
            lv_label_set_text(lbl_msg_channel_name, ch.name);
            // Show per-channel or device-default scope under the title
            if (lbl_msg_channel_scope) {
                P4NodePrefs* np = mesh->getNodePrefs();
                const char* scope = (ch.scope_name[0] != '\0') ? ch.scope_name
                                  : (np && np->default_scope_name[0] != '\0')
                                    ? np->default_scope_name : nullptr;
                if (scope) {
                    char buf[40];
                    snprintf(buf, sizeof(buf), "[%s]", scope);
                    lv_label_set_text(lbl_msg_channel_scope, buf);
                } else {
                    lv_label_set_text(lbl_msg_channel_scope, "");
                }
            }
        } else {
            lv_label_set_text(lbl_msg_channel_name, "???");
            if (lbl_msg_channel_scope) lv_label_set_text(lbl_msg_channel_scope, "");
        }
        lv_color_t color = lv_palette_main(CH_COLORS[ch_idx % NUM_CH_COLORS]);
        lv_obj_set_style_text_color(lbl_msg_channel_name, color, 0);
    }

    rebuild_message_bubbles(ch_idx);

    if (ta_compose) {
        P4NodePrefs* dprefs = mesh ? mesh->getNodePrefs() : nullptr;
        if (dprefs && dprefs->save_drafts != 0 && ch_idx < MAX_GROUP_CHANNELS)
            lv_textarea_set_text(ta_compose, g_channel_drafts[ch_idx]);
        else
            lv_textarea_set_text(ta_compose, "");
    }
    if (kb_compose) lv_obj_add_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
    // Textarea was just cleared, so it'll be back at its min_height (50
    // px). meck_relayout_compose picks that up and sizes the message
    // list to match — same as the hardcoded SCREEN_HEIGHT - 130 used to
    // do, but without us having to keep the constant in sync.
    meck_relayout_compose();

    if (mesh) mesh->clearUnread(ch_idx);
    if (scr_messages) lv_screen_load(scr_messages);
}

// ============================================================================
// Contacts screen (lifted from old:1028-1108)
// ============================================================================

static void create_contacts_screen() {
    scr_contacts = lv_obj_create(NULL);
    lock_screen_scroll(scr_contacts);
    lv_obj_set_style_bg_color(scr_contacts, lv_color_black(), 0);
    screen_attach_clock_battery(scr_contacts, 5, &meck_montserrat_24, 30);

    create_back_button(scr_contacts);

    lv_obj_t *title = lv_label_create(scr_contacts);
    lv_label_set_text(title, "Contacts");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    // ---- Filter chip bar ----
    // Horizontal flex row of chip buttons. The "active" chip gets a bright
    // cyan border so the current filter is obvious at a glance. Selecting
    // a chip is also reachable by swiping the screen left/right (see
    // on_contacts_screen_gesture). Chips are filled in refresh_contacts_list
    // so the active highlight stays in sync with the underlying state.
    obj_contacts_filter_bar = lv_obj_create(scr_contacts);
    lv_obj_set_size(obj_contacts_filter_bar, SCREEN_WIDTH - 20, 50);
    lv_obj_set_pos(obj_contacts_filter_bar, 10, 90);
    lv_obj_set_style_bg_color(obj_contacts_filter_bar, lv_color_black(), 0);
    lv_obj_set_style_border_width(obj_contacts_filter_bar, 0, 0);
    lv_obj_set_style_pad_all(obj_contacts_filter_bar, 0, 0);
    lv_obj_set_layout(obj_contacts_filter_bar, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(obj_contacts_filter_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(obj_contacts_filter_bar,
        LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(obj_contacts_filter_bar, 6, 0);
    lv_obj_set_scroll_dir(obj_contacts_filter_bar, LV_DIR_HOR);

    obj_contacts_scroll = lv_obj_create(scr_contacts);
    lv_obj_set_size(obj_contacts_scroll, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 160);
    lv_obj_set_pos(obj_contacts_scroll, 10, 150);
    lv_obj_set_style_bg_color(obj_contacts_scroll, lv_color_make(10, 10, 15), 0);
    lv_obj_set_style_border_width(obj_contacts_scroll, 0, 0);
    lv_obj_set_style_radius(obj_contacts_scroll, 8, 0);
    lv_obj_set_style_pad_all(obj_contacts_scroll, 5, 0);
    lv_obj_set_scroll_dir(obj_contacts_scroll, LV_DIR_VER);
    // Rows are manually positioned in refresh_contacts_list rather than
    // flex-laid-out: with ~700 contacts the per-row flex relayout stalled the
    // LVGL task long enough to trip the task watchdog on every Contacts open.

    // Swipe left/right anywhere on the screen cycles filters.
    lv_obj_add_event_cb(scr_contacts, on_contacts_screen_gesture,
                        LV_EVENT_GESTURE, NULL);
}

// Local helpers — keep ADV_TYPE_* opaque since the upstream enum's name
// isn't always in scope. Numeric values match BaseChatMesh / MeshCore.
static const char* contact_type_badge(uint8_t adv_type) {
    switch (adv_type) {
        case 1: return "C";    // ADV_TYPE_CHAT
        case 2: return "R";    // ADV_TYPE_REPEATER
        case 3: return "RS";   // ADV_TYPE_ROOM
        case 4: return "S";    // ADV_TYPE_SENSOR
        default: return "?";
    }
}

static lv_color_t contact_type_color(uint8_t adv_type) {
    switch (adv_type) {
        case 1: return lv_palette_main(LV_PALETTE_CYAN);
        case 2: return lv_palette_main(LV_PALETTE_ORANGE);
        case 3: return lv_palette_main(LV_PALETTE_PURPLE);
        case 4: return lv_palette_main(LV_PALETTE_LIGHT_GREEN);
        default: return lv_palette_main(LV_PALETTE_GREY);
    }
}

static const char* contact_filter_label(ContactFilter f) {
    switch (f) {
        case CONTACT_FILTER_ALL:      return "All";
        case CONTACT_FILTER_CHAT:     return "Chat";
        case CONTACT_FILTER_REPEATER: return "Rptr";
        case CONTACT_FILTER_ROOM:     return "Room";
        case CONTACT_FILTER_SENSOR:   return "Sens";
        case CONTACT_FILTER_FAV:      return "Fav";
        default:                      return "?";
    }
}

static bool contact_matches_filter(const ContactInfo& ci, ContactFilter f) {
    switch (f) {
        case CONTACT_FILTER_ALL:      return true;
        case CONTACT_FILTER_CHAT:     return ci.type == 1;
        case CONTACT_FILTER_REPEATER: return ci.type == 2;
        case CONTACT_FILTER_ROOM:     return ci.type == 3;
        case CONTACT_FILTER_SENSOR:   return ci.type == 4;
        case CONTACT_FILTER_FAV:      return (ci.flags & 0x01) != 0;
        default: return true;
    }
}

static void refresh_contacts_list() {
    Meck* mesh = meck_get_instance();
    if (!obj_contacts_scroll || !mesh) return;

    // ---- Rebuild filter chips ----
    if (obj_contacts_filter_bar) {
        uint32_t bar_cnt = lv_obj_get_child_count(obj_contacts_filter_bar);
        for (int i = (int)bar_cnt - 1; i >= 0; i--) {
            lv_obj_delete(lv_obj_get_child(obj_contacts_filter_bar, i));
        }
        for (int f = 0; f < (int)CONTACT_FILTER_COUNT; f++) {
            lv_obj_t *chip = lv_button_create(obj_contacts_filter_bar);
            lv_obj_set_size(chip, LV_SIZE_CONTENT, 36);
            lv_obj_set_style_radius(chip, 18, 0);
            lv_obj_set_style_bg_color(chip, lv_color_make(25, 25, 35), 0);
            lv_obj_set_style_border_width(chip, 2, 0);
            // Active filter gets a bright cyan border so the selection is
            // unambiguous; inactive chips fade to a neutral grey.
            bool active = (f == (int)g_contact_filter);
            lv_obj_set_style_border_color(chip,
                active ? lv_palette_main(LV_PALETTE_CYAN)
                       : lv_color_make(60, 60, 70), 0);
            lv_obj_set_style_pad_left(chip, 12, 0);
            lv_obj_set_style_pad_right(chip, 12, 0);
            lv_obj_clear_flag(chip, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_event_cb(chip, on_filter_chip_tap,
                                LV_EVENT_CLICKED, (void*)(intptr_t)f);

            lv_obj_t *lbl = lv_label_create(chip);
            lv_label_set_text(lbl, contact_filter_label((ContactFilter)f));
            lv_obj_set_style_text_color(lbl,
                active ? lv_color_white()
                       : lv_palette_main(LV_PALETTE_GREY), 0);
            meck_set_font(lbl, &meck_montserrat_16, 0);
            lv_obj_center(lbl);
        }
    }

    // ---- Rebuild contact rows ----
    uint32_t child_cnt = lv_obj_get_child_count(obj_contacts_scroll);
    for (int i = (int)child_cnt - 1; i >= 0; i--) {
        lv_obj_delete(lv_obj_get_child(obj_contacts_scroll, i));
    }

    int n = mesh->getNumContacts();
    if (n == 0) {
        lv_obj_t *empty = lv_label_create(obj_contacts_scroll);
        lv_label_set_text(empty, "No contacts yet.\nAdverts will populate\nthis list.");
        lv_obj_set_style_text_color(empty, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(empty, &meck_montserrat_16, 0);
        return;
    }

    // Build a sorted index array over ALL contacts (regardless of filter).
    // Sort key is ci.lastmod descending, which is our local RTC time at the
    // moment we last processed activity from this contact (advert receipt,
    // message receipt, or message send). Originally tried sorting by
    // last_advert_timestamp, but that field is the sender's claimed clock
    // time embedded in the advert payload, not our reception time — so
    // nodes with forward-dated clocks squat at the top forever and nodes
    // with behind/stuck clocks sink to the bottom regardless of when we
    // actually heard from them. lastmod uses our own RTC and is bumped on
    // every advert receipt (see MeckMesh::onAdvertRecv override, which
    // works around BaseChatMesh's replay guard that would otherwise leave
    // lastmod unupdated for nodes whose claimed clock doesn't advance).
    //
    // stable_sort keeps contacts with identical timestamps (e.g. ones
    // never heard, lastmod=0) in their creation order. The filter is
    // applied below during row construction, so the sort itself does not
    // depend on g_contact_filter.
    //
    // Static BSS allocation (1500 * 8 = ~12 KB). The function only runs
    // on the LVGL task so reuse between calls is safe. Avoids piling 12 KB
    // onto the LVGL task stack.
    struct ContactSortEntry { int idx; uint32_t ts; };
    static ContactSortEntry s_sort_buf[MAX_CONTACTS];
    int sort_count = 0;
    for (int i = 0; i < n && sort_count < MAX_CONTACTS; i++) {
        ContactInfo ci;
        if (!mesh->getContactByIdx(i, ci)) continue;
        s_sort_buf[sort_count].idx = i;
        s_sort_buf[sort_count].ts  = ci.lastmod;
        sort_count++;
    }
    std::stable_sort(s_sort_buf, s_sort_buf + sort_count,
        [](const ContactSortEntry& a, const ContactSortEntry& b) {
            return a.ts > b.ts;
        });

    int shown = 0;
    int row_y = 0;
    for (int s = 0; s < sort_count; s++) {
        int i = s_sort_buf[s].idx;
        ContactInfo ci;
        if (!mesh->getContactByIdx(i, ci)) continue;
        if (!contact_matches_filter(ci, g_contact_filter)) continue;
        shown++;

        lv_obj_t *row = lv_button_create(obj_contacts_scroll);
        lv_obj_set_size(row, SCREEN_WIDTH - 40, 60);
        lv_obj_set_pos(row, 5, row_y);
        row_y += 65;
        lv_obj_set_style_bg_color(row, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(row, 8, 0);
        lv_obj_set_style_border_width(row, 1, 0);
        lv_obj_set_style_border_color(row,
            (ci.flags & 0x01) ? lv_palette_main(LV_PALETTE_YELLOW)
                              : lv_color_make(50, 50, 60), 0);
        // Tap row → detail screen. Favouriting moved to the star button
        // below (a tap, not a long-press) so it can't collide with list
        // scrolling: LVGL suppresses CLICKED once a press becomes a scroll,
        // whereas LONG_PRESSED could fire mid-swipe and favourite by accident.
        lv_obj_add_event_cb(row, on_contact_tap,
                            LV_EVENT_CLICKED,    (void*)(intptr_t)i);

        // Star button (yellow if favourited, dim grey otherwise). Transparent
        // hit area around the glyph; CLICKED toggles favourite for this row.
        // Kept narrow (ends ~x=28) so it clears the name label at +28.
        lv_obj_t *star = lv_button_create(row);
        lv_obj_set_size(star, 26, 48);
        lv_obj_align(star, LV_ALIGN_LEFT_MID, 2, 0);
        lv_obj_set_style_bg_opa(star, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(star, 0, 0);
        lv_obj_set_style_shadow_width(star, 0, 0);
        lv_obj_set_style_pad_all(star, 0, 0);
        lv_obj_add_event_cb(star, on_contact_long_press,
                            LV_EVENT_CLICKED, (void*)(intptr_t)i);
        lv_obj_t *star_lbl = lv_label_create(star);
        lv_label_set_text(star_lbl, (ci.flags & 0x01) ? "*" : " ");
        lv_obj_set_style_text_color(star_lbl,
            (ci.flags & 0x01) ? lv_palette_main(LV_PALETTE_YELLOW)
                              : lv_color_make(70, 70, 80), 0);
        meck_set_font(star_lbl, &meck_montserrat_22, 0);
        lv_obj_center(star_lbl);

        // Name. Strip emoji / higher-plane Unicode so they don't render
        // as tofu boxes (Montserrat covers Latin-1 only).
        char clean_name[64];
        strip_unrenderable(ci.name, clean_name, sizeof(clean_name));
        lv_obj_t *lbl = lv_label_create(row);
        lv_label_set_text(lbl, clean_name);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        meck_set_font(lbl, &meck_montserrat_18, 0);
        lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 28, 0);

        // Type badge — small coloured pill on the right with the C/R/RS/S
        // letter so the contact list reads at a glance even when filtered
        // to "All".
        lv_obj_t *type_chip = lv_obj_create(row);
        lv_obj_set_size(type_chip, 38, 24);
        lv_obj_align(type_chip, LV_ALIGN_RIGHT_MID, -54, 0);
        lv_obj_set_style_radius(type_chip, 12, 0);
        lv_obj_set_style_border_width(type_chip, 0, 0);
        lv_obj_set_style_pad_all(type_chip, 0, 0);
        lv_obj_set_style_bg_color(type_chip, contact_type_color(ci.type), 0);
        lv_obj_clear_flag(type_chip, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(type_chip, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_t *type_lbl = lv_label_create(type_chip);
        lv_label_set_text(type_lbl, contact_type_badge(ci.type));
        lv_obj_set_style_text_color(type_lbl, lv_color_black(), 0);
        meck_set_font(type_lbl, &meck_montserrat_14, 0);
        lv_obj_center(type_lbl);

        // Pub-key prefix on the far right.
        lv_obj_t *pk = lv_label_create(row);
        char pk_buf[16];
        snprintf(pk_buf, sizeof(pk_buf), "%02X%02X",
                 ci.id.pub_key[0], ci.id.pub_key[1]);
        lv_label_set_text(pk, pk_buf);
        lv_obj_set_style_text_color(pk, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(pk, &meck_montserrat_14, 0);
        lv_obj_align(pk, LV_ALIGN_RIGHT_MID, -10, 0);
    }

    // If a filter ate the entire list, surface a hint so the empty screen
    // doesn't look like a bug.
    if (shown == 0) {
        lv_obj_t *empty = lv_label_create(obj_contacts_scroll);
        char buf[80];
        snprintf(buf, sizeof(buf), "No %s contacts.\nSwipe to change filter.",
                 contact_filter_label(g_contact_filter));
        lv_label_set_text(empty, buf);
        lv_obj_set_style_text_color(empty, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(empty, &meck_montserrat_16, 0);
    }
}

// ============================================================================
// Discover screen — nearby repeaters with per-row Add and a Rescan button
// ----------------------------------------------------------------------------
// Mirrors Meck T-Deck Pro's Discover (F-key) behaviour: this is a
// REPEATERS-ONLY view. Adverts are received passively into the same
// _recent[] ring the Last Heard list reads from; here we just filter
// to ADV_TYPE_REPEATER and render. Each row gets either an "Add"
// button (if the repeater isn't yet a contact) or a dim "Added" label
// (if it is). The Rescan button floods a self-advert so we announce
// ourselves, and refreshes the list from the latest ring state.
// ============================================================================

// Tap handler bound to each row's Add button. The user-data carries the
// discovered_idx (matches getDiscovered() index at draw time). The Meck
// instance also exposes addContactFromDiscovered so we don't have to
// reach into ContactInfo plumbing here.
static void on_discover_add_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    int idx = (int)(intptr_t)lv_event_get_user_data(e);

    int n = mesh->getDiscoveredCount();
    if (idx < 0 || idx >= n) {
        refresh_discover_list();  // ring shifted under us
        return;
    }
    const DiscoveredNode& d = mesh->getDiscovered(idx);
    if (!d.valid) {
        refresh_discover_list();
        return;
    }
    if (mesh->addContactFromDiscovered(d)) {
        refresh_discover_list();  // [+] state now flips to "Added"
    }
}

// Rescan: kick off a fresh active discovery scan. This both transmits a
// zero-hop self-advert (eliciting zero-hop response adverts from any
// neighbour with the feature enabled) AND opens a P4_DISCOVERY_WINDOW_MS
// capture window during which incoming adverts are tracked separately
// from the general _recent[] ring with their live SNR.
static void on_discover_rescan_tap(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    mesh->startDiscovery();
    refresh_discover_list();
}

// LVGL timer that polls Meck::consumeDiscoveryDirty() ~3× per second
// while the Discover screen is the active screen. Refreshes the list
// only when something changed (new advert captured, or window closed).
// Created in goto_discover, deleted when the screen unloads.
static lv_timer_t *discover_poll_timer = NULL;

static void discover_poll_cb(lv_timer_t *t) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    // If the user navigated away from the discover screen, kill the
    // timer to stop wasting cycles.
    if (lv_screen_active() != scr_discover) {
        if (discover_poll_timer) {
            lv_timer_delete(discover_poll_timer);
            discover_poll_timer = NULL;
        }
        return;
    }
    if (mesh->consumeDiscoveryDirty()) {
        refresh_discover_list();
    }
}

static void create_discover_screen() {
    scr_discover = lv_obj_create(NULL);
    lock_screen_scroll(scr_discover);
    lv_obj_set_style_bg_color(scr_discover, lv_color_black(), 0);
    screen_attach_clock_battery(scr_discover, 7, &meck_montserrat_24, 30);

    create_back_button(scr_discover);

    lv_obj_t *title = lv_label_create(scr_discover);
    lv_label_set_text(title, "Discover");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    // Status text under the title — "Scanning... N found" while active,
    // "Scan done: N found" once the window closes. The polling timer
    // (discover_poll_timer) updates this via refresh_discover_list when
    // something changes.
    lbl_discover_status = lv_label_create(scr_discover);
    lv_label_set_text(lbl_discover_status, "");
    lv_obj_set_style_text_color(lbl_discover_status,
        lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(lbl_discover_status, &meck_montserrat_16, 0);
    lv_obj_align(lbl_discover_status, LV_ALIGN_TOP_LEFT, 120, 65);

    // Rescan button — top-right of the screen. Starts a fresh active
    // scan (clears the captured list, transmits a zero-hop probe, opens
    // the capture window again).
    lv_obj_t *btn_rescan = lv_button_create(scr_discover);
    lv_obj_set_size(btn_rescan, 140, 60);
    lv_obj_align(btn_rescan, LV_ALIGN_TOP_RIGHT, -20, 30);
    lv_obj_set_style_bg_color(btn_rescan, lv_color_make(40, 40, 60), 0);
    lv_obj_set_style_radius(btn_rescan, 8, 0);
    lv_obj_set_style_border_width(btn_rescan, 1, 0);
    lv_obj_set_style_border_color(btn_rescan,
        lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_add_event_cb(btn_rescan, on_discover_rescan_tap,
                        LV_EVENT_CLICKED, NULL);

    lv_obj_t *rescan_lbl = lv_label_create(btn_rescan);
    lv_label_set_text(rescan_lbl, "Rescan");
    lv_obj_set_style_text_color(rescan_lbl, lv_color_white(), 0);
    meck_set_font(rescan_lbl, &meck_montserrat_18, 0);
    lv_obj_center(rescan_lbl);

    // Body: scrollable column of rows.
    obj_discover_scroll = lv_obj_create(scr_discover);
    lv_obj_set_size(obj_discover_scroll, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 160);
    lv_obj_set_pos(obj_discover_scroll, 10, 150);
    lv_obj_set_style_bg_color(obj_discover_scroll, lv_color_make(10, 10, 15), 0);
    lv_obj_set_style_border_width(obj_discover_scroll, 0, 0);
    lv_obj_set_style_radius(obj_discover_scroll, 8, 0);
    lv_obj_set_style_pad_all(obj_discover_scroll, 5, 0);
    lv_obj_set_scroll_dir(obj_discover_scroll, LV_DIR_VER);
    lv_obj_set_flex_flow(obj_discover_scroll, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(obj_discover_scroll,
        LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
}

// Render the active-discovery result list. Each row shows:
//   - Type letter (R/C/S/N) + name
//   - Short pub_key prefix + (SNR in dB) OR (hop count) + "in contacts" marker
//
// Distinguishes live (heard during this scan, snr_x4 != 0) from cached
// (pre-seeded from _recent[] at scan start, snr_x4 == 0):
//   - Live entries:  "XXXXXXXX • +N.NdB" (SNR colour-graded)
//   - Cached entries: "XXXXXXXX • N hops" (grey, indicates not fresh)
static void refresh_discover_list() {
    Meck* mesh = meck_get_instance();
    if (!obj_discover_scroll || !mesh) return;

    // Drop existing rows
    uint32_t child_cnt = lv_obj_get_child_count(obj_discover_scroll);
    for (int i = (int)child_cnt - 1; i >= 0; i--) {
        lv_obj_delete(lv_obj_get_child(obj_discover_scroll, i));
    }

    int n = mesh->getDiscoveredCount();
    bool active = mesh->isDiscoveryActive();

    // Status line
    if (lbl_discover_status) {
        char buf[64];
        if (active) {
            snprintf(buf, sizeof(buf), "Scanning...  %d repeater%s found",
                     n, n == 1 ? "" : "s");
        } else if (n == 0) {
            snprintf(buf, sizeof(buf),
                     "No repeaters heard yet tap Rescan");
        } else {
            snprintf(buf, sizeof(buf), "Scan done:  %d repeater%s found",
                     n, n == 1 ? "" : "s");
        }
        lv_label_set_text(lbl_discover_status, buf);
        lv_obj_set_style_text_color(lbl_discover_status,
            active ? lv_palette_main(LV_PALETTE_CYAN)
                   : lv_palette_main(LV_PALETTE_GREY), 0);
    }

    if (n == 0) {
        lv_obj_t *empty = lv_label_create(obj_discover_scroll);
        if (active) {
            lv_label_set_text(empty,
                "Listening for zero-hop repeater\n"
                "responses... Repeaters with\n"
                "advert.interval > 0 should\n"
                "reply within 30 seconds.");
        } else {
            lv_label_set_text(empty,
                "Tap Rescan to send a zero-hop probe.\n"
                "Nearby repeaters with response enabled\n"
                "will reply with their own advert.");
        }
        lv_obj_set_style_text_color(empty, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(empty, &meck_montserrat_16, 0);
        return;
    }

    for (int i = 0; i < n; i++) {
        const DiscoveredNode& d = mesh->getDiscovered(i);
        if (!d.valid) continue;

        lv_obj_t *row = lv_obj_create(obj_discover_scroll);
        lv_obj_set_size(row, SCREEN_WIDTH - 40, 80);
        lv_obj_set_style_bg_color(row, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(row, 8, 0);
        lv_obj_set_style_border_width(row, 1, 0);
        lv_obj_set_style_border_color(row, lv_color_make(50, 50, 60), 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(row, 8, 0);

        // Discover is repeater-only, so the per-row prefix is always 'R'.
        // (No need for a type switch — that filter happens at capture
        // time in MeckMesh.h's onAdvertRecv and startDiscovery.)
        lv_obj_t *name = lv_label_create(row);
        char namebuf[40];
        snprintf(namebuf, sizeof(namebuf), "R %s",
                 d.name[0] ? d.name : "?");
        lv_label_set_text(name, namebuf);
        lv_obj_set_style_text_color(name, lv_color_white(), 0);
        meck_set_font(name, &meck_montserrat_22, 0);
        lv_obj_align(name, LV_ALIGN_TOP_LEFT, 0, 0);

        // Meta line: short id, then either SNR (live) or hops (cached).
        char meta[64];
        if (d.snr_x4 != 0) {
            float snr_db = d.snr_x4 / 4.0f;
            snprintf(meta, sizeof(meta),
                     "%02X%02X%02X%02X  •  %+.1fdB SNR  •  zero-hop",
                     d.pub_key[0], d.pub_key[1], d.pub_key[2], d.pub_key[3],
                     snr_db);
        } else if (d.path_len == 0) {
            snprintf(meta, sizeof(meta),
                     "%02X%02X%02X%02X  •  cached  •  no signal data",
                     d.pub_key[0], d.pub_key[1], d.pub_key[2], d.pub_key[3]);
        } else {
            snprintf(meta, sizeof(meta),
                     "%02X%02X%02X%02X  •  cached  •  %d hop%s away",
                     d.pub_key[0], d.pub_key[1], d.pub_key[2], d.pub_key[3],
                     d.path_len, d.path_len == 1 ? "" : "s");
        }

        lv_obj_t *meta_lbl = lv_label_create(row);
        lv_label_set_text(meta_lbl, meta);

        // Live entries get SNR colour grading. Cached entries stay grey
        // to signal "this is from before the scan; signal data is stale".
        lv_color_t meta_col;
        if (d.snr_x4 != 0) {
            int snr_db = d.snr_x4 / 4;
            meta_col = (snr_db >= 0)  ? lv_palette_main(LV_PALETTE_GREEN)
                     : (snr_db > -5)  ? lv_palette_main(LV_PALETTE_YELLOW)
                                      : lv_palette_main(LV_PALETTE_RED);
        } else {
            meta_col = lv_palette_main(LV_PALETTE_GREY);
        }
        lv_obj_set_style_text_color(meta_lbl, meta_col, 0);
        meck_set_font(meta_lbl, &meck_montserrat_14, 0);
        lv_obj_align(meta_lbl, LV_ALIGN_BOTTOM_LEFT, 0, 0);

        // Right side: Add button OR Added badge.
        if (d.already_in_contacts) {
            lv_obj_t *added = lv_label_create(row);
            lv_label_set_text(added, LV_SYMBOL_OK " Added");
            lv_obj_set_style_text_color(added,
                lv_palette_main(LV_PALETTE_GREY), 0);
            meck_set_font(added, &meck_montserrat_16, 0);
            lv_obj_align(added, LV_ALIGN_RIGHT_MID, 0, 0);
        } else {
            lv_obj_t *add_btn = lv_button_create(row);
            lv_obj_set_size(add_btn, 100, 50);
            lv_obj_align(add_btn, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_obj_set_style_bg_color(add_btn,
                lv_palette_main(LV_PALETTE_CYAN), 0);
            lv_obj_set_style_radius(add_btn, 8, 0);
            lv_obj_set_style_border_width(add_btn, 0, 0);
            lv_obj_add_event_cb(add_btn, on_discover_add_tap,
                                LV_EVENT_CLICKED, (void*)(intptr_t)i);

            lv_obj_t *add_lbl = lv_label_create(add_btn);
            lv_label_set_text(add_lbl, LV_SYMBOL_PLUS " Add");
            lv_obj_set_style_text_color(add_lbl, lv_color_black(), 0);
            meck_set_font(add_lbl, &meck_montserrat_18, 0);
            lv_obj_center(add_lbl);
        }
    }
}

static void goto_discover(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (mesh) {
        // Auto-start a scan when the screen opens. The user can manually
        // re-trigger via Rescan if they want a fresh window. Skipping the
        // auto-start when one is already running avoids stomping the
        // in-flight window if the user just popped back from a sub-screen.
        if (!mesh->isDiscoveryActive()) {
            mesh->startDiscovery();
        }
    }
    refresh_discover_list();
    if (scr_discover) lv_screen_load(scr_discover);

    // Poll for incoming adverts ~3× per second while the screen is up.
    // Created here (not in create_discover_screen) so it only runs while
    // the screen is actually visible.
    if (!discover_poll_timer) {
        discover_poll_timer = lv_timer_create(discover_poll_cb, 300, NULL);
    }
}

// ============================================================================
// Contact detail screen (lifted from old:1111-1166)
// ============================================================================

static void create_contact_detail_screen() {
    scr_contact_detail = lv_obj_create(NULL);
    lock_screen_scroll(scr_contact_detail);
    lv_obj_set_style_bg_color(scr_contact_detail, lv_color_black(), 0);

    // Back -> contacts list (NOT home)
    lv_obj_t *btn_back = lv_button_create(scr_contact_detail);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, goto_contacts_from_detail, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr_contact_detail);
    lv_label_set_text(title, "Contact");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    lv_obj_t *btn_fav = lv_button_create(scr_contact_detail);
    lv_obj_set_size(btn_fav, 100, 70);
    lv_obj_align(btn_fav, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_bg_color(btn_fav, lv_palette_main(LV_PALETTE_YELLOW), 0);
    lv_obj_set_style_radius(btn_fav, 8, 0);
    lv_obj_t *fav_lbl = lv_label_create(btn_fav);
    lv_label_set_text(fav_lbl, LV_SYMBOL_OK " Fav");
    lv_obj_set_style_text_color(fav_lbl, lv_color_black(), 0);
    meck_set_font(fav_lbl, &meck_montserrat_16, 0);
    lv_obj_center(fav_lbl);
    lv_obj_add_event_cb(btn_fav, on_contact_fav_toggle, LV_EVENT_CLICKED, NULL);

    // Delete button — long-press only, so a stray tap doesn't lose a
    // contact. Useful for cleaning up duplicates or contacts whose owners
    // have rotated keys.
    lv_obj_t *btn_del = lv_button_create(scr_contact_detail);
    lv_obj_set_size(btn_del, 100, 70);
    lv_obj_align(btn_del, LV_ALIGN_TOP_RIGHT, -120, 10);
    lv_obj_set_style_bg_color(btn_del, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_radius(btn_del, 8, 0);
    lv_obj_t *del_lbl = lv_label_create(btn_del);
    lv_label_set_text(del_lbl, LV_SYMBOL_TRASH " Hold");
    lv_obj_set_style_text_color(del_lbl, lv_color_white(), 0);
    meck_set_font(del_lbl, &meck_montserrat_16, 0);
    lv_obj_center(del_lbl);
    lv_obj_add_event_cb(btn_del, on_contact_delete, LV_EVENT_LONG_PRESSED, NULL);

    // Send DM — opens scr_messages in DM mode for this contact. Cyan to
    // match the Identity / Export accent and the send-button colour on
    // scr_messages itself.
    // Send DM — opens scr_messages in DM mode for this contact. Cyan to
    // match the Identity / Export accent and the send-button colour on
    // scr_messages itself. Positioned directly below the Fav button (not
    // left of Delete) so it clears the TFT camera punch-hole, which sits
    // along the top edge of the screen.
    //
    // DM and Admin share the same slot — DM shows for chat contacts,
    // Admin shows for repeaters and room servers. The on_contact_tap
    // handler flips visibility based on ci.type each time a contact is
    // selected.
    btn_contact_dm = lv_button_create(scr_contact_detail);
    lv_obj_set_size(btn_contact_dm, 100, 70);
    lv_obj_align(btn_contact_dm, LV_ALIGN_TOP_RIGHT, -10, 90);
    lv_obj_set_style_bg_color(btn_contact_dm,
        lv_palette_darken(LV_PALETTE_CYAN, 2), 0);
    lv_obj_set_style_radius(btn_contact_dm, 8, 0);
    lv_obj_t *dm_lbl = lv_label_create(btn_contact_dm);
    lv_label_set_text(dm_lbl, LV_SYMBOL_ENVELOPE " DM");
    lv_obj_set_style_text_color(dm_lbl, lv_color_white(), 0);
    meck_set_font(dm_lbl, &meck_montserrat_16, 0);
    lv_obj_center(dm_lbl);
    lv_obj_add_event_cb(btn_contact_dm, on_contact_send_dm_tap, LV_EVENT_CLICKED, NULL);

    // Admin — opens scr_admin_login for this contact. Shown only when
    // ci.type is ADV_TYPE_REPEATER (2) or ADV_TYPE_ROOM (3). Hidden
    // by default; on_contact_tap reveals it when appropriate.
    btn_contact_admin = lv_button_create(scr_contact_detail);
    lv_obj_set_size(btn_contact_admin, 100, 70);
    lv_obj_align(btn_contact_admin, LV_ALIGN_TOP_RIGHT, -10, 90);
    lv_obj_set_style_bg_color(btn_contact_admin,
        lv_palette_darken(LV_PALETTE_INDIGO, 1), 0);
    lv_obj_set_style_radius(btn_contact_admin, 8, 0);
    lv_obj_t *adm_lbl = lv_label_create(btn_contact_admin);
    lbl_contact_admin_btn = adm_lbl;
    lv_label_set_text(adm_lbl, LV_SYMBOL_SETTINGS " Admin");
    lv_obj_set_style_text_color(adm_lbl, lv_color_white(), 0);
    meck_set_font(adm_lbl, &meck_montserrat_16, 0);
    lv_obj_center(adm_lbl);
    lv_obj_add_event_cb(btn_contact_admin, on_contact_admin_tap, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(btn_contact_admin, LV_OBJ_FLAG_HIDDEN);

    // Edit Path — opens scr_path_editor for the currently-selected contact.
    // Sits left of the DM/Admin slot so it doesn't overlap either. Useful
    // for repeaters and room servers 3+ hops away where flood discovery
    // is unreliable; user can pin a known-good route manually.
    lv_obj_t *btn_edit_path = lv_button_create(scr_contact_detail);
    lv_obj_set_size(btn_edit_path, 100, 70);
    lv_obj_align(btn_edit_path, LV_ALIGN_TOP_RIGHT, -120, 90);
    lv_obj_set_style_bg_color(btn_edit_path,
        lv_palette_darken(LV_PALETTE_TEAL, 1), 0);
    lv_obj_set_style_radius(btn_edit_path, 8, 0);
    lv_obj_t *ep_lbl = lv_label_create(btn_edit_path);
    lv_label_set_text(ep_lbl, LV_SYMBOL_SHUFFLE " Path");
    lv_obj_set_style_text_color(ep_lbl, lv_color_white(), 0);
    meck_set_font(ep_lbl, &meck_montserrat_16, 0);
    lv_obj_center(ep_lbl);
    lv_obj_add_event_cb(btn_edit_path, goto_path_editor, LV_EVENT_CLICKED, NULL);

    lv_obj_t *scroll = lv_obj_create(scr_contact_detail);
    lv_obj_set_size(scroll, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 180);
    lv_obj_set_pos(scroll, 10, 170);
    lv_obj_set_style_bg_color(scroll, lv_color_make(10, 10, 15), 0);
    lv_obj_set_style_border_width(scroll, 0, 0);
    lv_obj_set_style_radius(scroll, 8, 0);
    lv_obj_set_style_pad_all(scroll, 10, 0);

    lbl_contact_detail_body = lv_label_create(scroll);
    lv_label_set_text(lbl_contact_detail_body, "");
    lv_obj_set_style_text_color(lbl_contact_detail_body, lv_color_white(), 0);
    meck_set_font(lbl_contact_detail_body, &meck_montserrat_16, 0);
    lv_label_set_long_mode(lbl_contact_detail_body, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_contact_detail_body, SCREEN_WIDTH - 50);

    // Ping — sends a zero-hop trace to the repeater. Fuchsia, below Admin.
    // Default 1-byte; long-press opens 1/2 byte chooser.
    // MUST be created after the scroll container so it renders on top —
    // the scroll starts at y=170 and the ping button sits at y=170 too.
    btn_contact_ping = lv_button_create(scr_contact_detail);
    lv_obj_set_size(btn_contact_ping, 100, 70);
    lv_obj_align(btn_contact_ping, LV_ALIGN_TOP_RIGHT, -10, 170);
    lv_obj_set_style_bg_color(btn_contact_ping,
        lv_color_make(200, 50, 150), 0);  // fuchsia
    lv_obj_set_style_radius(btn_contact_ping, 8, 0);
    lv_obj_t *ping_lbl = lv_label_create(btn_contact_ping);
    lv_label_set_text(ping_lbl, "Ping");
    lv_obj_set_style_text_color(ping_lbl, lv_color_white(), 0);
    meck_set_font(ping_lbl, &meck_montserrat_16, 0);
    lv_obj_center(ping_lbl);
    lv_obj_add_event_cb(btn_contact_ping, on_contact_ping_tap,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(btn_contact_ping, on_contact_ping_longpress,
                        LV_EVENT_LONG_PRESSED, NULL);
    lv_obj_add_flag(btn_contact_ping, LV_OBJ_FLAG_HIDDEN);
}

// ============================================================================
// Trace Path screen + Path Editor screen
// ----------------------------------------------------------------------------
// Both screens share a manual hex hop entry pattern: dropdown for hash
// size (1 or 2 bytes per hop), textarea for comma-separated hex hops, and
// an action button. Differences:
//
//   Trace Path:    Run Trace → createTrace + sendDirect(TRACE flavoured),
//                  result list rendered from Meck::consumeTraceResult()
//                  on each ui_update_timer_cb tick.
//   Path Editor:   Save → Meck::setContactCustomPath(idx, bytes, encoded)
//                  Reset to Auto → Meck::clearContactCustomPath(idx)
//
// Manual entry only — no repeater picker. Per user spec: with 600+
// repeaters in a regional mesh, scrolling a picker is more friction
// than typing four hex digits per hop.
// ============================================================================

// Parse comma-separated hex hops into out_buf. Returns hops on success,
// -1 on parse error. For 2-byte mode, big-endian byte order so the on-
// screen "3601" maps to bytes 0x36, 0x01 (same convention as upstream
// Tracescreen.h::parseTypedPath). Skips whitespace and stray spaces.
static int parse_hex_hop_path(const char *src, int bytes_per_hop,
                              uint8_t *out_buf, int max_hops) {
    if (!src || !out_buf || bytes_per_hop < 1 || bytes_per_hop > 2) return -1;
    int hops = 0;
    const char *p = src;
    while (*p && hops < max_hops) {
        while (*p == ',' || *p == ' ' || *p == '\t' || *p == '\n') p++;
        if (*p == '\0') break;

        char *end;
        long val = strtol(p, &end, 16);
        if (end == p) return -1;
        p = end;

        if (bytes_per_hop == 1) {
            if (val < 0 || val > 0xFF) return -1;
            out_buf[hops] = (uint8_t)val;
        } else {
            if (val < 0 || val > 0xFFFF) return -1;
            out_buf[hops * 2]     = (uint8_t)((val >> 8) & 0xFF);
            out_buf[hops * 2 + 1] = (uint8_t)(val & 0xFF);
        }
        hops++;
    }
    while (*p == ',' || *p == ' ' || *p == '\t' || *p == '\n') p++;
    if (*p != '\0') return -1;  // trailing junk after last token
    return hops;
}

// Look up a contact name by hash prefix. bytes_per_hop is 1 or 2 (matches
// the trace's encoded path_sz). Returns true if a contact matched, false
// otherwise. Repeaters preferred (ADV_TYPE_REPEATER == 2); falls back to
// any contact.
static bool trace_find_name_for_hash(const uint8_t *hash, int bytes_per_hop,
                                     char *out_name, size_t out_name_len) {
    Meck *mesh = meck_get_instance();
    if (!mesh) return false;
    uint32_t n = (uint32_t)mesh->getNumContacts();
    ContactInfo c;
    // Pass 1: repeaters only
    for (uint32_t i = 0; i < n; i++) {
        if (!mesh->getContactByIdx(i, c)) continue;
        if (c.type != 2 /* ADV_TYPE_REPEATER */) continue;
        if (memcmp(c.id.pub_key, hash, bytes_per_hop) == 0) {
            strncpy(out_name, c.name, out_name_len);
            out_name[out_name_len - 1] = '\0';
            return true;
        }
    }
    // Pass 2: any contact
    for (uint32_t i = 0; i < n; i++) {
        if (!mesh->getContactByIdx(i, c)) continue;
        if (memcmp(c.id.pub_key, hash, bytes_per_hop) == 0) {
            strncpy(out_name, c.name, out_name_len);
            out_name[out_name_len - 1] = '\0';
            return true;
        }
    }
    return false;
}

// ----- Trace screen ---------------------------------------------------------

// Re-show the keyboard when the user taps the textarea after a previous
// Run Trace has hidden it. Pure show-only — defocus handler is intentionally
// not wired (keyboard stays put until the next Run Trace), which matches
// the user's "Option B" spec.
static void on_trace_textarea_focused(lv_event_t *e) {
    (void)e;
    // Hardware keyboard: on-screen keyboard stays down (goto_trace hid it
    // and gave the results container the full height).
    if (meck_hw_keyboard_active()) return;
    if (kb_trace) lv_obj_remove_flag(kb_trace, LV_OBJ_FLAG_HIDDEN);
    // Shrink the results container back to its keyboard-visible height so
    // its bottom edge sits above the keyboard, not behind it.
    if (cont_trace_results) {
        lv_obj_set_height(cont_trace_results,
            SCREEN_HEIGHT - (NOTCH_SAFE_Y + 540) - MECK_KB_HEIGHT - 10);
    }
}

static void on_trace_back_clicked(lv_event_t *e) {
    (void)e;
    Meck *mesh = meck_get_instance();
    if (mesh) mesh->traceClearPending();
    g_trace_in_flight = false;
    if (scr_home) lv_screen_load(scr_home);
}

// Wipe any previously-rendered result rows.
static void trace_clear_results() {
    if (!cont_trace_results) return;
    lv_obj_clean(cont_trace_results);
}

// Build one result row (avatar circle + name + subtitle + SNR). Matches
// the screenshot style: greyish badge with the hop's hash hex on the left,
// repeater name in bold above a smaller subtitle line, SNR text + a 3-bar
// signal icon on the right. Used for the originator row, each hop, and
// the final "Received trace response" row.
static void trace_add_result_row(const char *badge_text,
                                 const char *name,
                                 const char *subtitle,
                                 const char *snr_text,   // NULL to hide
                                 int snr_x4,             // for bar colouring
                                 bool snr_known) {
    if (!cont_trace_results) return;

    lv_obj_t *row = lv_obj_create(cont_trace_results);
    lv_obj_set_width(row, SCREEN_WIDTH - 60);
    lv_obj_set_height(row, 90);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    // Badge — 50 px circle on the left
    lv_obj_t *badge = lv_obj_create(row);
    lv_obj_set_size(badge, 50, 50);
    lv_obj_align(badge, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_radius(badge, 25, 0);
    lv_obj_set_style_bg_color(badge, lv_color_make(80, 90, 100), 0);
    lv_obj_set_style_border_width(badge, 0, 0);
    lv_obj_clear_flag(badge, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *blbl = lv_label_create(badge);
    lv_label_set_text(blbl, badge_text);
    lv_obj_set_style_text_color(blbl, lv_color_white(), 0);
    meck_set_font(blbl, &meck_montserrat_16, 0);
    lv_obj_center(blbl);

    // Name (top line)
    lv_obj_t *nl = lv_label_create(row);
    lv_label_set_text(nl, name);
    lv_obj_set_style_text_color(nl, lv_color_white(), 0);
    meck_set_font(nl, &meck_montserrat_18, 0);
    lv_obj_align(nl, LV_ALIGN_TOP_LEFT, 65, 0);
    lv_obj_set_width(nl, SCREEN_WIDTH - 60 - 65 - 80);
    lv_label_set_long_mode(nl, LV_LABEL_LONG_DOT);

    // Subtitle (e.g. "Hop 2 · Repeated the packet")
    lv_obj_t *sl = lv_label_create(row);
    lv_label_set_text(sl, subtitle);
    lv_obj_set_style_text_color(sl, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(sl, &meck_montserrat_14, 0);
    lv_obj_align(sl, LV_ALIGN_TOP_LEFT, 65, 30);
    lv_obj_set_width(sl, SCREEN_WIDTH - 60 - 65 - 80);
    lv_label_set_long_mode(sl, LV_LABEL_LONG_DOT);

    // SNR text on the bottom-left of the body area
    if (snr_text) {
        lv_obj_t *snl = lv_label_create(row);
        lv_label_set_text(snl, snr_text);
        lv_obj_set_style_text_color(snl,
            lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(snl, &meck_montserrat_14, 0);
        lv_obj_align(snl, LV_ALIGN_TOP_LEFT, 65, 55);
    }

    // Signal bars on the far right. Three short rects, coloured by SNR
    // bands (matches upstream drawSignalBars in Tracescreen.h: high ≥8 dB,
    // mid ≥3 dB, low ≥-5 dB). Unknown SNR shows all bars grey.
    if (snr_known) {
        float snr = snr_x4 / 4.0f;
        int bars = 0;
        if (snr >= -5.0f) bars = 1;
        if (snr >= 3.0f)  bars = 2;
        if (snr >= 8.0f)  bars = 3;
        lv_color_t col_on  = (bars >= 2) ? lv_palette_main(LV_PALETTE_GREEN)
                                          : lv_palette_main(LV_PALETTE_RED);
        lv_color_t col_off = lv_color_make(60, 60, 60);
        for (int i = 0; i < 3; i++) {
            lv_obj_t *bar = lv_obj_create(row);
            int h = 10 + i * 8;
            lv_obj_set_size(bar, 8, h);
            lv_obj_align(bar, LV_ALIGN_RIGHT_MID, -((2 - i) * 12) - 4,
                         (30 - h) / 2);
            lv_obj_set_style_radius(bar, 2, 0);
            lv_obj_set_style_border_width(bar, 0, 0);
            lv_obj_set_style_bg_color(bar, (i < bars) ? col_on : col_off, 0);
            lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
        }
    }
}

// Render a result snapshot into the scroll container. Called from the
// ui_update_timer_cb hook the moment Meck::consumeTraceResult returns true.
static void trace_render_result(const Meck::MeckTraceResult &r) {
    trace_clear_results();

    Meck *mesh = meck_get_instance();
    P4NodePrefs *prefs = mesh ? mesh->getNodePrefs() : NULL;
    const char *self_name = (prefs && prefs->node_name[0])
                             ? prefs->node_name : "me";

    // First row: this device (originator). Subtitle = "Started the trace".
    // SNR shown here is the final_snr — the SNR we heard the response at,
    // i.e. the inbound leg of the last hop.
    char snr_buf[32];
    float final_snr_db = r.final_snr / 4.0f;
    snprintf(snr_buf, sizeof(snr_buf), "SNR: %.2fdB",
             final_snr_db);
    trace_add_result_row("•", self_name, "Started the trace",
                          snr_buf, r.final_snr, true);

    // Per-hop rows
    for (int h = 0; h < r.hop_count; h++) {
        const uint8_t *hash = &r.path_hashes[h * r.bytes_per_hop];
        char badge[8];
        if (r.bytes_per_hop == 1) {
            snprintf(badge, sizeof(badge), "%02X", hash[0]);
        } else {
            snprintf(badge, sizeof(badge), "%02X", hash[0]);  // 1st byte
        }
        char name_buf[40];
        if (!trace_find_name_for_hash(hash, r.bytes_per_hop,
                                       name_buf, sizeof(name_buf))) {
            if (r.bytes_per_hop == 1) {
                snprintf(name_buf, sizeof(name_buf), "0x%02X", hash[0]);
            } else {
                snprintf(name_buf, sizeof(name_buf), "0x%02X%02X",
                         hash[0], hash[1]);
            }
        }
        char subtitle[40];
        snprintf(subtitle, sizeof(subtitle),
                 "Hop %d " LV_SYMBOL_BULLET " Repeated the packet", h + 1);
        char snrl[32];
        float hop_snr = r.path_snrs[h] / 4.0f;
        snprintf(snrl, sizeof(snrl), "SNR: %.2fdB", hop_snr);
        trace_add_result_row(badge, name_buf, subtitle, snrl,
                              r.path_snrs[h], true);
    }

    // Final row: response received locally
    char durs[40];
    snprintf(durs, sizeof(durs), "Duration: %ums",
             (unsigned)r.duration_ms);
    trace_add_result_row(LV_SYMBOL_OK, self_name, "Received trace response",
                          durs, r.final_snr, false);

    if (lbl_trace_status) {
        char st[64];
        snprintf(st, sizeof(st), "Result: %u hops, %ums",
                 (unsigned)r.hop_count, (unsigned)r.duration_ms);
        lv_label_set_text(lbl_trace_status, st);
    }
    g_trace_in_flight = false;
}

static void on_trace_path_clear_clicked(lv_event_t *e) {
    (void)e;
    if (ta_trace_path) lv_textarea_set_text(ta_trace_path, "");
}

static void on_trace_run_clicked(lv_event_t *e) {
    (void)e;
    Meck *mesh = meck_get_instance();
    if (!mesh) return;
    if (!ta_trace_path || !dd_trace_path_size) return;

    int sel = lv_dropdown_get_selected(dd_trace_path_size);
    int bytes_per_hop = (sel == 1) ? 2 : 1;

    const char *txt = lv_textarea_get_text(ta_trace_path);
    uint8_t path_buf[Meck::MECK_TRACE_MAX_HOPS * 2];
    int hops = parse_hex_hop_path(txt, bytes_per_hop, path_buf,
                                   Meck::MECK_TRACE_MAX_HOPS);
    if (hops < 0) {
        if (lbl_trace_status) {
            lv_label_set_text(lbl_trace_status,
                "Parse error: enter comma-separated hex hops");
        }
        return;
    }
    if (hops == 0) {
        if (lbl_trace_status) {
            lv_label_set_text(lbl_trace_status, "Enter at least one hop");
        }
        return;
    }

    // Generate tag + auth from the RNG, then build and send.
    uint32_t tag = 0, auth = 0;
    mesh::RNG *rng = mesh->getRNG();
    if (rng) {
        rng->random((uint8_t*)&tag,  4);
        rng->random((uint8_t*)&auth, 4);
    }
    uint8_t flags = (bytes_per_hop == 2) ? 1 : 0;   // path_sz in low 2 bits

    mesh::Packet *pkt = mesh->createTrace(tag, auth, flags);
    if (!pkt) {
        if (lbl_trace_status) {
            lv_label_set_text(lbl_trace_status,
                "Send failed: packet pool empty");
        }
        return;
    }

    uint8_t path_byte_len = (uint8_t)(hops * bytes_per_hop);
    g_trace_sent_ms = (unsigned long)millis();
    mesh->traceMarkSent(tag, auth, g_trace_sent_ms);
    mesh->sendDirect(pkt, path_buf, path_byte_len);

    g_trace_in_flight = true;
    trace_clear_results();
    if (lbl_trace_status) {
        char st[80];
        snprintf(st, sizeof(st),
            "Running trace: %d hop%s, %d-byte mode (tag 0x%08X)",
            hops, hops == 1 ? "" : "s", bytes_per_hop, (unsigned)tag);
        lv_label_set_text(lbl_trace_status, st);
    }
    // Hide the keyboard so the result list / status are visible. The
    // FOCUSED handler on ta_trace_path brings it back if the user taps
    // into the textarea again.
    if (kb_trace) lv_obj_add_flag(kb_trace, LV_OBJ_FLAG_HIDDEN);
    // Grow the results container to occupy the space the keyboard just
    // freed, otherwise rows render into a tiny scrollable window with
    // most of the screen below sitting empty.
    if (cont_trace_results) {
        lv_obj_set_height(cont_trace_results,
            SCREEN_HEIGHT - (NOTCH_SAFE_Y + 540) - 10);
    }
    printf("MeckUI: Trace sent tag=0x%08X hops=%d bpp=%d byte_len=%u\n",
           (unsigned)tag, hops, bytes_per_hop, (unsigned)path_byte_len);
}

static void create_trace_screen() {
    scr_trace = lv_obj_create(NULL);
    lock_screen_scroll(scr_trace);
    lv_obj_set_style_bg_color(scr_trace, lv_color_black(), 0);

    // Back button — top-left
    lv_obj_t *btn_back = lv_button_create(scr_trace);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, NOTCH_SAFE_Y);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_trace_back_clicked, LV_EVENT_CLICKED, NULL);

    // Title
    lv_obj_t *title = lv_label_create(scr_trace);
    lv_label_set_text(title, "Trace Path");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, NOTCH_SAFE_Y + 20);

    // Header clock/battery/audio indicator (slot 10)
    screen_attach_clock_battery(scr_trace, 10, &meck_montserrat_24,
                                 NOTCH_SAFE_Y + 20);

    // Info banner — wording per spec (no picker, hex entry only)
    lv_obj_t *banner = lv_obj_create(scr_trace);
    lv_obj_set_size(banner, SCREEN_WIDTH - 20, 80);
    lv_obj_align(banner, LV_ALIGN_TOP_MID, 0, NOTCH_SAFE_Y + 100);
    lv_obj_set_style_bg_color(banner, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_set_style_radius(banner, 8, 0);
    lv_obj_set_style_border_width(banner, 0, 0);
    lv_obj_clear_flag(banner, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *bantxt = lv_label_create(banner);
    lv_label_set_text(bantxt,
        "Enter repeater hash prefixes separated by commas, then tap "
        "Run Trace. You must be able to hear the last repeater directly.");
    lv_obj_set_style_text_color(bantxt, lv_color_white(), 0);
    meck_set_font(bantxt, &meck_montserrat_14, 0);
    lv_label_set_long_mode(bantxt, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(bantxt, SCREEN_WIDTH - 60);
    lv_obj_center(bantxt);

    // Path Size dropdown
    lv_obj_t *psl = lv_label_create(scr_trace);
    lv_label_set_text(psl, "Path Size");
    lv_obj_set_style_text_color(psl,
        lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(psl, &meck_montserrat_14, 0);
    lv_obj_align(psl, LV_ALIGN_TOP_LEFT, 20, NOTCH_SAFE_Y + 200);

    dd_trace_path_size = lv_dropdown_create(scr_trace);
    lv_dropdown_set_options(dd_trace_path_size,
        "1-byte (max 63 hops)\n2-byte (max 32 hops)");
    lv_dropdown_set_selected(dd_trace_path_size, 1);  // default 2-byte
    // The default LVGL dropdown indicator is LV_SYMBOL_DOWN (FontAwesome
    // chevron-down), which the meck_montserrat_* fonts don't include, so it
    // renders as a tofu box. Override with a plain ASCII "v".
    lv_dropdown_set_symbol(dd_trace_path_size, "v");
    lv_obj_set_size(dd_trace_path_size, SCREEN_WIDTH - 40, 50);
    lv_obj_align(dd_trace_path_size, LV_ALIGN_TOP_LEFT, 20,
                 NOTCH_SAFE_Y + 225);
    lv_obj_set_style_bg_color(dd_trace_path_size,
        lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(dd_trace_path_size, lv_color_white(), 0);
    meck_set_font(dd_trace_path_size, &meck_montserrat_18, 0);

    // Path textarea
    lv_obj_t *ptl = lv_label_create(scr_trace);
    lv_label_set_text(ptl, "Path (hex hops, comma-separated)");
    lv_obj_set_style_text_color(ptl,
        lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(ptl, &meck_montserrat_14, 0);
    lv_obj_align(ptl, LV_ALIGN_TOP_LEFT, 20, NOTCH_SAFE_Y + 290);

    ta_trace_path = lv_textarea_create(scr_trace);
    lv_obj_set_size(ta_trace_path, SCREEN_WIDTH - 110, 50);
    lv_obj_align(ta_trace_path, LV_ALIGN_TOP_LEFT, 20, NOTCH_SAFE_Y + 315);
    lv_textarea_set_one_line(ta_trace_path, true);
    lv_textarea_set_max_length(ta_trace_path, 200);
    lv_textarea_set_placeholder_text(ta_trace_path, "e.g. 3601,2198,3601");
    lv_obj_set_style_bg_color(ta_trace_path,
        lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_trace_path, lv_color_white(), 0);
    meck_set_font(ta_trace_path, &meck_montserrat_18, 0);
    // Cursor styling — same as ta_compose: white 2px left border, full
    // opacity. Without this the cursor inherits the textarea text colour
    // and is effectively invisible against the dark background.
    lv_obj_set_style_border_color(ta_trace_path, lv_color_white(),     LV_PART_CURSOR);
    lv_obj_set_style_border_width(ta_trace_path, 2,                     LV_PART_CURSOR);
    lv_obj_set_style_border_side(ta_trace_path,  LV_BORDER_SIDE_LEFT,   LV_PART_CURSOR);
    lv_obj_set_style_border_opa(ta_trace_path,   LV_OPA_COVER,          LV_PART_CURSOR);
    // Bring the keyboard back when the textarea is tapped — paired with
    // the hide call in on_trace_run_clicked. Without this the keyboard
    // stays gone after a Run Trace and the user can't edit the path
    // without leaving and re-entering the screen.
    lv_obj_add_event_cb(ta_trace_path, on_trace_textarea_focused,
                        LV_EVENT_FOCUSED, NULL);

    // X button to clear the path field
    lv_obj_t *btn_clear = lv_button_create(scr_trace);
    lv_obj_set_size(btn_clear, 70, 50);
    lv_obj_align(btn_clear, LV_ALIGN_TOP_RIGHT, -20, NOTCH_SAFE_Y + 315);
    lv_obj_set_style_bg_color(btn_clear,
        lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_clear, 8, 0);
    lv_obj_t *cl = lv_label_create(btn_clear);
    lv_label_set_text(cl, LV_SYMBOL_CLOSE);
    lv_obj_set_style_text_color(cl, lv_color_white(), 0);
    meck_set_font(cl, &meck_montserrat_18, 0);
    lv_obj_center(cl);
    lv_obj_add_event_cb(btn_clear, on_trace_path_clear_clicked,
                        LV_EVENT_CLICKED, NULL);

    // Run Trace button
    btn_trace_run = lv_button_create(scr_trace);
    lv_obj_set_size(btn_trace_run, SCREEN_WIDTH - 40, 70);
    lv_obj_align(btn_trace_run, LV_ALIGN_TOP_MID, 0, NOTCH_SAFE_Y + 415);
    lv_obj_set_style_bg_color(btn_trace_run,
        lv_palette_darken(LV_PALETTE_INDIGO, 2), 0);
    lv_obj_set_style_radius(btn_trace_run, 35, 0);
    lv_obj_t *rl = lv_label_create(btn_trace_run);
    lv_label_set_text(rl, "Run Trace");
    lv_obj_set_style_text_color(rl, lv_color_white(), 0);
    meck_set_font(rl, &meck_montserrat_22, 0);
    lv_obj_center(rl);
    lv_obj_add_event_cb(btn_trace_run, on_trace_run_clicked,
                        LV_EVENT_CLICKED, NULL);

    // Status label
    lbl_trace_status = lv_label_create(scr_trace);
    lv_label_set_text(lbl_trace_status, "");
    lv_obj_set_style_text_color(lbl_trace_status,
        lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(lbl_trace_status, &meck_montserrat_14, 0);
    lv_obj_align(lbl_trace_status, LV_ALIGN_TOP_MID, 0, NOTCH_SAFE_Y + 500);

    // Results scroll container — fills the remainder of the screen above
    // the keyboard area. The keyboard is created hidden and slides up on
    // textarea focus per the standard meck_style_keyboard pattern; when
    // visible it covers the bottom MECK_KB_HEIGHT pixels.
    cont_trace_results = lv_obj_create(scr_trace);
    lv_obj_set_size(cont_trace_results, SCREEN_WIDTH - 40,
                    SCREEN_HEIGHT - (NOTCH_SAFE_Y + 540) - MECK_KB_HEIGHT - 10);
    lv_obj_align(cont_trace_results, LV_ALIGN_TOP_MID, 0,
                 NOTCH_SAFE_Y + 530);
    lv_obj_set_style_bg_opa(cont_trace_results, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont_trace_results, 0, 0);
    lv_obj_set_flex_flow(cont_trace_results, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(cont_trace_results, 8, 0);

    // Keyboard — attached to the path textarea. Stored in kb_trace so
    // the FOCUSED handler (on_trace_textarea_focused) can re-show it
    // after on_trace_run_clicked hides it on send.
    lv_obj_t *kb = lv_keyboard_create(scr_trace);
    meck_style_keyboard(kb);
    lv_keyboard_set_textarea(kb, ta_trace_path);
    lv_obj_add_event_cb(kb, on_kb_long_press, LV_EVENT_LONG_PRESSED, NULL);
    kb_trace = kb;
}

static void goto_trace(lv_event_t *e) {
    (void)e;
    if (!scr_trace) return;
    // Fresh entry — clear any leftover state.
    if (ta_trace_path) lv_textarea_set_text(ta_trace_path, "");
    if (lbl_trace_status) lv_label_set_text(lbl_trace_status, "");
    trace_clear_results();
    g_trace_in_flight = false;
    Meck *mesh = meck_get_instance();
    if (mesh) mesh->traceClearPending();
    if (meck_hw_keyboard_active()) {
        // Hardware keyboard: this screen's on-screen keyboard is created
        // visible, so hide it on entry and give the results container the
        // full height, mirroring the post-Run-Trace layout.
        if (kb_trace) lv_obj_add_flag(kb_trace, LV_OBJ_FLAG_HIDDEN);
        if (cont_trace_results) {
            lv_obj_set_height(cont_trace_results,
                SCREEN_HEIGHT - (NOTCH_SAFE_Y + 540) - 10);
        }
    }
    lv_screen_load(scr_trace);
}

// ----- Path editor screen ---------------------------------------------------

// Mirror of on_trace_textarea_focused — brings the keyboard back when
// the user taps the textarea after Save / Reset hid it.
static void on_path_editor_textarea_focused(lv_event_t *e) {
    (void)e;
    if (meck_hw_keyboard_active()) return;
    if (kb_path_editor) lv_obj_remove_flag(kb_path_editor, LV_OBJ_FLAG_HIDDEN);
}

static void on_path_editor_back_clicked(lv_event_t *e) {
    (void)e;
    if (scr_contact_detail) lv_screen_load(scr_contact_detail);
}

static void on_path_editor_save_clicked(lv_event_t *e) {
    (void)e;
    if (g_selected_contact_idx < 0) return;
    Meck *mesh = meck_get_instance();
    if (!mesh || !ta_path_editor_path || !dd_path_editor_size) return;

    int sel = lv_dropdown_get_selected(dd_path_editor_size);
    int bytes_per_hop = (sel == 1) ? 2 : 1;

    const char *txt = lv_textarea_get_text(ta_path_editor_path);
    uint8_t path_buf[MAX_PATH_SIZE];
    int hops = parse_hex_hop_path(txt, bytes_per_hop, path_buf,
                                   MAX_PATH_SIZE / bytes_per_hop);
    if (hops < 0) {
        if (lbl_path_editor_status) {
            lv_label_set_text(lbl_path_editor_status,
                "Parse error: enter comma-separated hex hops");
        }
        return;
    }
    if (hops == 0) {
        // Empty textarea + Save -> save as a 0-hop direct path (encoded
        // out_path_len = mode<<6 | 0). The MeshCore base layer sees a
        // non-UNKNOWN out_path_len and routes via sendDirect with a
        // 0-byte path, which is the same wire format as sendZeroHop.
        // To revert to flood routing, the user taps "Reset to Flood".
        uint8_t mode = (uint8_t)(bytes_per_hop - 1);
        uint8_t encoded = (uint8_t)(mode << 6);  // hops = 0 in low 6 bits
        if (!mesh->setContactCustomPath(g_selected_contact_idx,
                                         path_buf, encoded)) {
            if (lbl_path_editor_status) {
                lv_label_set_text(lbl_path_editor_status,
                    "Save failed: contact not found");
            }
            return;
        }
        if (lbl_path_editor_status) {
            char st[80];
            snprintf(st, sizeof(st),
                "Saved: 0-hop direct (%d-byte mode)", bytes_per_hop);
            lv_label_set_text(lbl_path_editor_status, st);
        }
        if (kb_path_editor) lv_obj_add_flag(kb_path_editor, LV_OBJ_FLAG_HIDDEN);
        printf("MeckUI: PathEditor saved contact=%d 0-hop direct bpp=%d encoded=0x%02X\n",
               g_selected_contact_idx, bytes_per_hop, (unsigned)encoded);
        return;
    }
    // Encode: bits[7:6]=mode (bytes_per_hop-1), bits[5:0]=hops
    uint8_t mode = (uint8_t)(bytes_per_hop - 1);
    uint8_t encoded = (uint8_t)((mode << 6) | (hops & 0x3F));
    if (!mesh->setContactCustomPath(g_selected_contact_idx,
                                     path_buf, encoded)) {
        if (lbl_path_editor_status) {
            lv_label_set_text(lbl_path_editor_status,
                "Save failed: contact not found");
        }
        return;
    }
    if (lbl_path_editor_status) {
        char st[80];
        snprintf(st, sizeof(st),
            "Saved: %d-hop %d-byte path", hops, bytes_per_hop);
        lv_label_set_text(lbl_path_editor_status, st);
    }
    if (kb_path_editor) lv_obj_add_flag(kb_path_editor, LV_OBJ_FLAG_HIDDEN);
    printf("MeckUI: PathEditor saved contact=%d hops=%d bpp=%d encoded=0x%02X\n",
           g_selected_contact_idx, hops, bytes_per_hop, (unsigned)encoded);
}

static void on_path_editor_reset_clicked(lv_event_t *e) {
    (void)e;
    if (g_selected_contact_idx < 0) return;
    Meck *mesh = meck_get_instance();
    if (!mesh) return;
    if (ta_path_editor_path) lv_textarea_set_text(ta_path_editor_path, "");
    if (mesh->clearContactCustomPath(g_selected_contact_idx)) {
        if (lbl_path_editor_status) {
            lv_label_set_text(lbl_path_editor_status,
                "Cleared — path will be auto-discovered");
        }
        if (kb_path_editor) lv_obj_add_flag(kb_path_editor, LV_OBJ_FLAG_HIDDEN);
    }
}

// Fill the editor's UI from the current contact's stored path. Called by
// goto_path_editor each time the screen is opened so a re-entry reflects
// the latest persisted state (including any path changes from incoming
// advertisements while we were away).
static void path_editor_load_for_contact(int idx) {
    Meck *mesh = meck_get_instance();
    if (!mesh || idx < 0) return;
    ContactInfo c;
    if (!mesh->getContactByIdx((uint32_t)idx, c)) return;

    if (lbl_path_editor_contact) {
        lv_label_set_text_fmt(lbl_path_editor_contact, "%s", c.name);
    }
    if (lbl_path_editor_status) lv_label_set_text(lbl_path_editor_status, "");

    if (c.out_path_len == OUT_PATH_UNKNOWN) {
        if (dd_path_editor_size) lv_dropdown_set_selected(dd_path_editor_size, 0);
        if (ta_path_editor_path) lv_textarea_set_text(ta_path_editor_path, "");
        return;
    }
    uint8_t hops      = c.out_path_len & 0x3F;
    uint8_t hash_size = (c.out_path_len >> 6) + 1;
    if (hash_size > 2) hash_size = 2;  // Editor caps at 2-byte (no 3-byte UI yet)

    if (dd_path_editor_size) {
        lv_dropdown_set_selected(dd_path_editor_size,
                                  (hash_size == 2) ? 1 : 0);
    }
    // Build comma-separated hex text from the stored path
    char buf[200] = {0};
    int pos = 0;
    for (int h = 0; h < hops; h++) {
        if (h > 0 && pos < (int)sizeof(buf) - 1) buf[pos++] = ',';
        if (hash_size == 1) {
            int n = snprintf(&buf[pos], sizeof(buf) - pos, "%02X",
                             c.out_path[h]);
            if (n > 0) pos += n;
        } else {
            int n = snprintf(&buf[pos], sizeof(buf) - pos, "%02X%02X",
                             c.out_path[h * 2], c.out_path[h * 2 + 1]);
            if (n > 0) pos += n;
        }
    }
    if (ta_path_editor_path) lv_textarea_set_text(ta_path_editor_path, buf);
}

static void create_path_editor_screen() {
    scr_path_editor = lv_obj_create(NULL);
    lock_screen_scroll(scr_path_editor);
    lv_obj_set_style_bg_color(scr_path_editor, lv_color_black(), 0);

    // Back button -> contact detail
    lv_obj_t *btn_back = lv_button_create(scr_path_editor);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, NOTCH_SAFE_Y);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_path_editor_back_clicked,
                        LV_EVENT_CLICKED, NULL);

    // Title
    lv_obj_t *title = lv_label_create(scr_path_editor);
    lv_label_set_text(title, "Edit Path");
    lv_obj_set_style_text_color(title,
        lv_palette_main(LV_PALETTE_TEAL), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, NOTCH_SAFE_Y + 20);

    // Contact name (set by path_editor_load_for_contact)
    lbl_path_editor_contact = lv_label_create(scr_path_editor);
    lv_label_set_text(lbl_path_editor_contact, "");
    lv_obj_set_style_text_color(lbl_path_editor_contact, lv_color_white(), 0);
    meck_set_font(lbl_path_editor_contact, &meck_montserrat_18, 0);
    lv_obj_align(lbl_path_editor_contact, LV_ALIGN_TOP_LEFT, 120,
                 NOTCH_SAFE_Y + 55);

    // Header clock/battery/audio indicator (slot 11)
    screen_attach_clock_battery(scr_path_editor, 11, &meck_montserrat_24,
                                 NOTCH_SAFE_Y + 20);

    // Path Size dropdown
    lv_obj_t *psl = lv_label_create(scr_path_editor);
    lv_label_set_text(psl, "Path Size");
    lv_obj_set_style_text_color(psl,
        lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(psl, &meck_montserrat_14, 0);
    lv_obj_align(psl, LV_ALIGN_TOP_LEFT, 20, NOTCH_SAFE_Y + 130);

    dd_path_editor_size = lv_dropdown_create(scr_path_editor);
    lv_dropdown_set_options(dd_path_editor_size,
        "1-byte (max 63 hops)\n2-byte (max 32 hops)");
    lv_dropdown_set_selected(dd_path_editor_size, 0);
    // Same tofu fix as the trace screen dropdown.
    lv_dropdown_set_symbol(dd_path_editor_size, "v");
    lv_obj_set_size(dd_path_editor_size, SCREEN_WIDTH - 40, 50);
    lv_obj_align(dd_path_editor_size, LV_ALIGN_TOP_LEFT, 20,
                 NOTCH_SAFE_Y + 155);
    lv_obj_set_style_bg_color(dd_path_editor_size,
        lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(dd_path_editor_size, lv_color_white(), 0);
    meck_set_font(dd_path_editor_size, &meck_montserrat_18, 0);

    // Path textarea
    lv_obj_t *ptl = lv_label_create(scr_path_editor);
    lv_label_set_text(ptl, "Path (hex hops, comma-separated). Empty Save = direct.");
    lv_obj_set_style_text_color(ptl,
        lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(ptl, &meck_montserrat_14, 0);
    lv_obj_align(ptl, LV_ALIGN_TOP_LEFT, 20, NOTCH_SAFE_Y + 220);

    ta_path_editor_path = lv_textarea_create(scr_path_editor);
    lv_obj_set_size(ta_path_editor_path, SCREEN_WIDTH - 40, 50);
    lv_obj_align(ta_path_editor_path, LV_ALIGN_TOP_LEFT, 20,
                 NOTCH_SAFE_Y + 245);
    lv_textarea_set_one_line(ta_path_editor_path, true);
    lv_textarea_set_max_length(ta_path_editor_path, 200);
    lv_textarea_set_placeholder_text(ta_path_editor_path,
        "e.g. 3601,2198 (2-byte) or 36,21 (1-byte)");
    lv_obj_set_style_bg_color(ta_path_editor_path,
        lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_path_editor_path, lv_color_white(), 0);
    meck_set_font(ta_path_editor_path, &meck_montserrat_18, 0);
    // Cursor styling — same as ta_compose: white 2px left border, full opacity.
    lv_obj_set_style_border_color(ta_path_editor_path, lv_color_white(),     LV_PART_CURSOR);
    lv_obj_set_style_border_width(ta_path_editor_path, 2,                     LV_PART_CURSOR);
    lv_obj_set_style_border_side(ta_path_editor_path,  LV_BORDER_SIDE_LEFT,   LV_PART_CURSOR);
    lv_obj_set_style_border_opa(ta_path_editor_path,   LV_OPA_COVER,          LV_PART_CURSOR);
    // Bring the keyboard back when the textarea is tapped — paired with
    // the hide calls in on_path_editor_save_clicked /
    // on_path_editor_reset_clicked.
    lv_obj_add_event_cb(ta_path_editor_path, on_path_editor_textarea_focused,
                        LV_EVENT_FOCUSED, NULL);

    // Save button
    lv_obj_t *btn_save = lv_button_create(scr_path_editor);
    lv_obj_set_size(btn_save, (SCREEN_WIDTH - 60) / 2, 70);
    lv_obj_align(btn_save, LV_ALIGN_TOP_LEFT, 20, NOTCH_SAFE_Y + 345);
    lv_obj_set_style_bg_color(btn_save,
        lv_palette_darken(LV_PALETTE_TEAL, 2), 0);
    lv_obj_set_style_radius(btn_save, 8, 0);
    lv_obj_t *sl = lv_label_create(btn_save);
    lv_label_set_text(sl, "Save");
    lv_obj_set_style_text_color(sl, lv_color_white(), 0);
    meck_set_font(sl, &meck_montserrat_18, 0);
    lv_obj_center(sl);
    lv_obj_add_event_cb(btn_save, on_path_editor_save_clicked,
                        LV_EVENT_CLICKED, NULL);

    // Reset to auto button
    lv_obj_t *btn_reset = lv_button_create(scr_path_editor);
    lv_obj_set_size(btn_reset, (SCREEN_WIDTH - 60) / 2, 70);
    lv_obj_align(btn_reset, LV_ALIGN_TOP_RIGHT, -20, NOTCH_SAFE_Y + 345);
    lv_obj_set_style_bg_color(btn_reset,
        lv_color_make(60, 60, 60), 0);
    lv_obj_set_style_radius(btn_reset, 8, 0);
    lv_obj_t *rl = lv_label_create(btn_reset);
    lv_label_set_text(rl, "Reset to Flood");
    lv_obj_set_style_text_color(rl, lv_color_white(), 0);
    meck_set_font(rl, &meck_montserrat_16, 0);
    lv_obj_center(rl);
    lv_obj_add_event_cb(btn_reset, on_path_editor_reset_clicked,
                        LV_EVENT_CLICKED, NULL);

    // Status label
    lbl_path_editor_status = lv_label_create(scr_path_editor);
    lv_label_set_text(lbl_path_editor_status, "");
    lv_obj_set_style_text_color(lbl_path_editor_status,
        lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(lbl_path_editor_status, &meck_montserrat_14, 0);
    lv_obj_align(lbl_path_editor_status, LV_ALIGN_TOP_MID, 0,
                 NOTCH_SAFE_Y + 430);

    // Keyboard — stored in kb_path_editor so the FOCUSED handler can
    // re-show it after Save / Reset hide it.
    lv_obj_t *kb = lv_keyboard_create(scr_path_editor);
    meck_style_keyboard(kb);
    lv_keyboard_set_textarea(kb, ta_path_editor_path);
    lv_obj_add_event_cb(kb, on_kb_long_press, LV_EVENT_LONG_PRESSED, NULL);
    kb_path_editor = kb;
}

static void goto_path_editor(lv_event_t *e) {
    (void)e;
    if (!scr_path_editor) return;
    path_editor_load_for_contact(g_selected_contact_idx);
    if (meck_hw_keyboard_active() && kb_path_editor)
        lv_obj_add_flag(kb_path_editor, LV_OBJ_FLAG_HIDDEN);
    lv_screen_load(scr_path_editor);
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

// Mode-aware back from the messages screen. In channel mode, returns to
// the channel picker (existing behaviour). In DM mode, returns to the
// contact detail screen for the contact we were chatting with. Clears
// g_in_dm_mode so the next screen-load doesn't accidentally render with
// DM data. The send/keyboard widgets are shared between channel and DM
// modes, so we just hide the keyboard rather than rebuild anything.
static void goto_messages_back(lv_event_t *e) {
    if (g_in_dm_mode) {
        g_in_dm_mode = false;
        // Keep g_active_dm_contact_idx valid so the DM screen can be
        // reloaded with state-restore if the user re-enters from the
        // same contact detail.
        if (kb_compose) lv_obj_add_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
        if (scr_contact_detail) lv_screen_load(scr_contact_detail);
    } else {
        goto_channel_picker(e);
    }
}

// Load DM conversation for a specific contact. Mirrors load_channel_view's
// shape: stash the active contact, populate the title label, rebuild the
// bubbles, clear the textarea, reset layout, mark unread → 0, present
// scr_messages.
//
// scr_messages is reused (one screen, two modes) so all the shared
// textarea/keyboard/relayout wiring stays in one place. g_in_dm_mode is
// the flag the rest of the code branches on; on_send_clicked picks the
// right send path, the back button picks the right return target, and
// rebuild_dm_bubbles is called from here instead of rebuild_message_bubbles.
static void load_dm_view(int contact_idx) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;

    ContactInfo ci;
    if (!mesh->getContactByIdx((uint32_t)contact_idx, ci)) {
        printf("load_dm_view: contact idx %d not found\n", contact_idx);
        return;
    }

    g_active_dm_contact_idx = contact_idx;
    g_in_dm_mode = true;

    // Title shows the contact name, coloured cyan to match the send
    // accent. (Channel mode uses CH_COLORS hashed by channel index;
    // there's no equivalent palette concept for contacts here.)
    if (lbl_msg_channel_name) {
        lv_label_set_text(lbl_msg_channel_name, ci.name);
        lv_obj_set_style_text_color(lbl_msg_channel_name,
            lv_palette_main(LV_PALETTE_CYAN), 0);
    }

    rebuild_dm_bubbles(contact_idx);

    if (ta_compose) lv_textarea_set_text(ta_compose, "");
    if (kb_compose) lv_obj_add_flag(kb_compose, LV_OBJ_FLAG_HIDDEN);
    meck_relayout_compose();

    dm_ring_clear_unread(contact_idx);
    if (scr_messages) lv_screen_load(scr_messages);
}

// Settings/Contacts entry-point: open the DM conversation for the contact
// whose detail screen is currently displayed. Wired to the Send DM button
// on scr_contact_detail.
static void on_contact_send_dm_tap(lv_event_t *e) {
    if (g_selected_contact_idx < 0) return;
    load_dm_view(g_selected_contact_idx);
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

    // Channel-share delivery feedback: poll ack status, retry up to 3 times.
    if (g_share_active && mesh) {
        bool acked = false;
        unsigned long sent = 0;
        uint32_t timeout = 0;
        if (mesh->lookupDMAckStatus(g_share_st_ack, acked, sent, timeout)) {
            if (acked) {
                share_status_finish("Sent!");
            } else if (millis() - sent > timeout) {
                if (g_share_st_attempt < 3) {
                    g_share_st_attempt++;
                    uint32_t ack2 = 0;
                    if (mesh->shareChannelViaDM(g_share_st_channel,
                                                g_share_st_contact, &ack2) && ack2) {
                        g_share_st_ack = ack2;
                        char sbuf[48];
                        snprintf(sbuf, sizeof(sbuf),
                            "Share Channel Sending (Attempt %d/3)", g_share_st_attempt);
                        share_status_show(sbuf);
                    } else {
                        share_status_finish("Share failed to send");
                    }
                } else {
                    share_status_finish("Failed");
                }
            }
            // else: still within the timeout window, keep waiting
        } else {
            // ack entry evicted from the circular table before we saw a
            // result; stop polling and let the status auto-dismiss.
            g_share_active = false;
            if (!g_share_dismiss_timer)
                g_share_dismiss_timer = lv_timer_create(share_status_dismiss_cb, 4000, NULL);
        }
    }

    // Drain anything queued by onMessageRecv since the last tick. This
    // runs on the LVGL task, so the callback invoked inside is safe to
    // touch LVGL state. See meck_register_dm_recv_callback below for the
    // dispatch function.
    meck_drain_pending_dms();
    // Same pattern for send completions — the mesh task records the
    // expected_ack after a successful transmit, the UI thread picks it
    // up here and writes it onto the matching outgoing bubble.
    meck_drain_pending_dm_sends();
    // Room post receive drain (Piece B). Posts pushed by a room server
    // after a successful login arrive on meck_task via
    // onSignedMessageRecv; LVGL task drains here and dispatches to the
    // per-room PostRing.
    meck_drain_pending_posts();
    // Admin response drain — piece 2 bridge. Cheap when nothing is
    // queued (early-return on empty rings).
    meck_drain_pending_admin_responses();

    // Voice over LoRa: drain pending VE3 envelopes and voice data packets.
    // When an incoming session is complete, auto-play via MeckVoice.
    {
        // Voice and picture transfer disabled for v0.3.8 — re-enable when
        // the send/receive pipeline is stable.
        // meck_drain_pending_voice();
        // meck_drain_pending_pictures();

        MeckVoice* voice = meck_get_voice_instance();
        if (voice && voice->isIncomingComplete()) {
            const MeckVoice::InSession& in = voice->getInSession();
            printf("MeckUI: voice session complete from %s (%lu bytes)\n",
                   in.senderName, (unsigned long)in.dataBytes);

#ifdef HAVE_CODEC2
            // Reassemble packets in order
            uint8_t ordered[VOICE_C2_MAX_BYTES];
            uint32_t orderedLen = voice->reassembleIncoming(ordered, sizeof(ordered));

            if (orderedLen > 0 && voice->decodeCodec2(ordered, orderedLen)) {
                // Ensure inbox folder exists
                struct stat st;
                if (stat(VOICE_INBOX_FOLDER, &st) != 0) {
                    mkdir("/sdcard/voice", 0755);
                    mkdir(VOICE_INBOX_FOLDER, 0755);
                }

                // Save decoded audio as WAV to inbox folder
                uint32_t ts = meck_clock_get_utc();
                char filename[48];
                snprintf(filename, sizeof(filename), "voice_rx_%06lu.wav",
                         (unsigned long)(ts % 1000000UL));

                // Save with inbox folder path
                char fullpath[128];
                snprintf(fullpath, sizeof(fullpath), "%s/%s",
                         VOICE_INBOX_FOLDER, filename);

                // Use saveToWav which writes to VOICE_FOLDER by default.
                // For inbox, we need to write directly.
                voice->saveToWav(filename);

                // Move the file from VOICE_FOLDER to VOICE_INBOX_FOLDER
                char srcPath[128];
                snprintf(srcPath, sizeof(srcPath), "%s/%s", VOICE_FOLDER, filename);
                rename(srcPath, fullpath);

                // Calculate duration from Codec2 frame count
                uint8_t durSec = (uint8_t)(in.dataBytes / VOICE_C2_FRAME_BYTES
                                           * VOICE_C2_FRAME_MS / 1000);

                // Add to inbox metadata
                voice_inbox_add(in.senderName, fullpath, ts, durSec);
                voice_inbox_update_badge();

                printf("MeckUI: voice from %s saved to inbox (%s, %ds)\n",
                       in.senderName, filename, durSec);
            } else {
                printf("MeckUI: Codec2 decode failed for incoming voice\n");
            }
#else
            printf("MeckUI: voice received but HAVE_CODEC2 not enabled\n");
#endif
            voice->clearIncoming();
        }

        // Live UI update during recording: call recordTick to pull
        // samples from the mic DMA, and refresh the voice screen.
        if (voice->isRecording()) {
            voice->recordTick();
            if (scr_voice && lv_screen_active() == scr_voice) {
                voice_update_ui();
            }
        }

        // Detect playback completion: if we're on the voice screen and
        // the status says "Playing..." but the audio backend has gone
        // idle, update the label.
        if (scr_voice && lv_screen_active() == scr_voice && lbl_voice_status) {
            MeckAudioState st = meck_audio_get_state();
            if (st == MECK_AUDIO_STATE_IDLE || st == MECK_AUDIO_STATE_EOF) {
                const char *cur = lv_label_get_text(lbl_voice_status);
                if (cur && strncmp(cur, "Playing", 7) == 0) {
                    if (g_voice_has_recording) {
                        char buf[48];
                        snprintf(buf, sizeof(buf), "Recorded %.1fs",
                                 voice->getRecDurationSec());
                        lv_label_set_text(lbl_voice_status, buf);
                        lv_obj_set_style_text_color(lbl_voice_status,
                            lv_palette_main(LV_PALETTE_GREEN), 0);
                    } else {
                        lv_label_set_text(lbl_voice_status, "Playback finished");
                        lv_obj_set_style_text_color(lbl_voice_status,
                            lv_palette_main(LV_PALETTE_GREEN), 0);
                    }
                }
            }
        }

        // Detect voice send status changes
        if (g_voice_send_in_flight && lbl_voice_status) {
            int vs = meck_voice_send_get_status();
            if (vs == VOICE_SEND_PACKETS_QUEUED) {
                // Packets are queued via sendDirect. Estimate completion
                // from packet count.
                uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
                if (now_ms >= g_voice_send_complete_ms) {
                    g_voice_send_in_flight = false;
                    const char *cur = lv_label_get_text(lbl_voice_status);
                    if (cur && strncmp(cur, "Sending", 7) == 0) {
                        lv_label_set_text(lbl_voice_status, "Sent");
                        lv_obj_set_style_text_color(lbl_voice_status,
                            lv_palette_main(LV_PALETTE_GREEN), 0);
                        printf("Voice: send complete (all packets transmitted)\n");
                    }
                }
            } else if (vs == VOICE_SEND_ACK_TIMEOUT) {
                g_voice_send_in_flight = false;
                lv_label_set_text(lbl_voice_status, "Failed: no path");
                lv_obj_set_style_text_color(lbl_voice_status,
                    lv_palette_main(LV_PALETTE_RED), 0);
                printf("Voice: UI updated - ACK timeout, no path\n");
            } else if (vs == VOICE_SEND_NO_PATH) {
                g_voice_send_in_flight = false;
                lv_label_set_text(lbl_voice_status, "Failed: path unknown");
                lv_obj_set_style_text_color(lbl_voice_status,
                    lv_palette_main(LV_PALETTE_RED), 0);
                printf("Voice: UI updated - ACK received but no path\n");
            }
        }
    }

    // Admin login screen: reveal-then-dot re-render. Causes the
    // most-recently-typed character to resolve to • once the reveal
    // window has elapsed. Cheap — admin_render_password_masked
    // short-circuits when the masked string hasn't changed.
    if (scr_admin_login && lv_screen_active() == scr_admin_login) {
        admin_render_password_masked();
    }

    // Admin login timeout: if we're in LOGGING_IN state and the
    // response hasn't arrived within g_admin_login_timeout_ms, fall
    // back to PASSWORD_ENTRY with a timeout message.
    if (g_admin_ui_state == ADMIN_STATE_LOGGING_IN) {
        unsigned long now = (unsigned long)esp_log_timestamp();
        if (g_admin_login_timeout_ms > 0 &&
            (now - g_admin_login_sent_ms) > g_admin_login_timeout_ms) {
            printf("Admin: login timed out after %ums\n",
                   (unsigned)g_admin_login_timeout_ms);
            g_admin_ui_state = ADMIN_STATE_PASSWORD_ENTRY;
            admin_login_show_status("Timed out. No response from repeater.",
                                    lv_palette_main(LV_PALETTE_RED));
            if (btn_admin_login_submit) {
                lv_obj_clear_state(btn_admin_login_submit, LV_STATE_DISABLED);
            }
            // Tear down the meck-side pending state too, so a late
            // response doesn't unexpectedly transition us back to
            // LOGGED_IN. (clearAdminSession clears _pending_login_pk.)
            meck_admin_clear_session();
        }
    }

    // Firmware Info timeout: if a "ver" request has been in flight for
    // longer than MECK_ADMIN_FW_TIMEOUT_MS without a dispatch, surface
    // a red timeout message and re-enable the Refresh button.
    if (g_admin_fw_in_flight) {
        unsigned long now = (unsigned long)esp_log_timestamp();
        if ((now - g_admin_fw_sent_ms) > MECK_ADMIN_FW_TIMEOUT_MS) {
            printf("Admin: fw info request timed out after %ums\n",
                   (unsigned)MECK_ADMIN_FW_TIMEOUT_MS);
            g_admin_fw_in_flight = false;
            if (lbl_admin_fw_info_status) {
                lv_label_set_text(lbl_admin_fw_info_status,
                                  "Request timed out. Tap Refresh to retry.");
                lv_obj_set_style_text_color(lbl_admin_fw_info_status,
                                            lv_palette_main(LV_PALETTE_RED), 0);
            }
            if (btn_admin_fw_info_refresh) {
                lv_obj_clear_state(btn_admin_fw_info_refresh, LV_STATE_DISABLED);
            }
        }
    }

    // Neighbours timeout: the in-flight flag stays true across the
    // multi-page pagination, so the timer resets at every dispatch
    // (admin_neighbours_dispatch updates _sent_ms before sending the
    // next page). If a page is silently lost, the elapsed time since
    // the last successful dispatch hits the threshold and we abort
    // with whatever entries have already been rendered.
    if (g_admin_neighbours_in_flight) {
        unsigned long now = (unsigned long)esp_log_timestamp();
        if ((now - g_admin_neighbours_sent_ms) > MECK_ADMIN_NEIGHBOURS_TIMEOUT_MS) {
            printf("Admin: neighbours request timed out after %ums "
                   "(received=%u/%u)\n",
                   (unsigned)MECK_ADMIN_NEIGHBOURS_TIMEOUT_MS,
                   (unsigned)g_admin_neighbours_received,
                   (unsigned)g_admin_neighbours_total);
            g_admin_neighbours_in_flight = false;
            if (lbl_admin_neighbours_status) {
                lv_label_set_text(lbl_admin_neighbours_status,
                                  "Request timed out. Tap Refresh to retry.");
                lv_obj_set_style_text_color(lbl_admin_neighbours_status,
                                            lv_palette_main(LV_PALETTE_RED), 0);
            }
            if (btn_admin_neighbours_refresh) {
                lv_obj_clear_state(btn_admin_neighbours_refresh, LV_STATE_DISABLED);
            }
        }
    }

    // Home title: show the user's chosen node name. Refreshed every tick so
    // a rename in Settings shows up without needing to rebuild the screen.
    // Font is sized to the name length so long names stay on screen without
    // running past the camera punch-hole.
    if (lbl_home_title && mesh) {
        P4NodePrefs* prefs = mesh->getNodePrefs();
        const char* show = (prefs && prefs->node_name[0]) ? prefs->node_name : "(no name)";
        lv_label_set_text(lbl_home_title, show);
        meck_update_font(lbl_home_title, meck_node_name_font(show), 0);
        home_reposition_clock_for_name(show);
    }

    // Home tile: unread count
    if (lbl_home_unread && mesh) {
        int ch_unread = mesh->getUnreadCount();
        int dm_unread = dm_total_unread();
        int unread = ch_unread + dm_unread;
        lv_label_set_text_fmt(lbl_home_unread, "MSG: %d", unread);
        lv_obj_set_style_text_color(lbl_home_unread,
            unread > 0 ? lv_color_make(0, 255, 100)
                       : lv_palette_main(LV_PALETTE_GREY), 0);
    }

    // WiFi IP / BLE PIN display — show whichever is active
    if (lbl_home_ble_pin && mesh) {
        P4NodePrefs* bp = mesh->getNodePrefs();
        const char* wifi_ip = meck_wifi_get_ip();
        if (bp && bp->wifi_enabled && wifi_ip && wifi_ip[0]) {
            lv_label_set_text_fmt(lbl_home_ble_pin,
                "WiFi: %s:5000", wifi_ip);
            lv_obj_clear_flag(lbl_home_ble_pin, LV_OBJ_FLAG_HIDDEN);
#if MECK_BLE_ENABLED
        } else if (bp && bp->ble_enabled && bp->ble_pin != 0) {
            lv_label_set_text_fmt(lbl_home_ble_pin,
                "BLE PIN: %06lu",
                (unsigned long)bp->ble_pin);
            lv_obj_clear_flag(lbl_home_ble_pin, LV_OBJ_FLAG_HIDDEN);
#endif
        } else {
            lv_obj_add_flag(lbl_home_ble_pin, LV_OBJ_FLAG_HIDDEN);
        }
    }

    // Live-refresh WiFi IP on the settings sub-screen (deferred enable
    // means the IP isn't available when the toggle first fires)
    if (lbl_set_wifi_ip) {
        const char* ip = meck_wifi_get_ip();
        lv_label_set_text(lbl_set_wifi_ip, (ip && ip[0]) ? ip : "Not connected");
    }

    // DM inbox live refresh. If the user is sitting on the inbox screen
    // when a new DM arrives, the list needs to rebuild so the unread
    // badge updates and a brand-new contact appears as a row. Gated on
    // active-screen check so we don't rebuild offscreen for nothing —
    // the rebuild is cheap (small n) but pointless when not visible.
    if (scr_dm_inbox && lv_screen_active() == scr_dm_inbox) {
        refresh_dm_inbox_list();
    }

    // DM conversation live refresh. When the user is sitting on
    // scr_messages in DM mode, periodic rebuilds let outgoing-bubble
    // status footers transition through "Sending..." → "Delivered" /
    // "Failed" as ACKs arrive or timeouts elapse. Without this, an
    // outgoing bubble would stay on whatever status it had when last
    // rebuilt (typically "Sending...") until something else triggered
    // a redraw. The rebuild is small and gated on visibility.
    if (g_in_dm_mode && scr_messages && lv_screen_active() == scr_messages
        && g_active_dm_contact_idx >= 0) {
        rebuild_dm_bubbles(g_active_dm_contact_idx);
    }

    // Public channel live refresh. The only time-based reason to redraw an
    // otherwise-clean channel is to flip a sent, un-echoed post from
    // "Sending..." to "Failed" once it passes the retry threshold; there is no
    // dirty event for that, it is purely elapsed time. rebuild_message_bubbles
    // records the soonest such flip in g_chan_flip_deadline, so we rebuild only
    // when it is actually due rather than every tick; a full rebuild runs
    // lv_obj_clean() and would tear down an in-progress long-press on a bubble.
    // New content and heard-count changes set the dirty flag and are handled by
    // the isMessageDirty() rebuild further below.
    if (!g_in_dm_mode && scr_messages && lv_screen_active() == scr_messages
        && g_active_channel < MAX_GROUP_CHANNELS && mesh && g_chan_flip_deadline != 0) {
        uint32_t now_utc = 0;
        mesh::RTCClock* rtc = mesh->getRTCClock();
        if (rtc) now_utc = rtc->getCurrentTime();
        if (now_utc != 0 && now_utc >= g_chan_flip_deadline) {
            rebuild_message_bubbles(g_active_channel);
        }
    }

    // Home tiles: clock once per minute. Timer fires at 500ms so 120 ticks
    // is one minute. Counters start at the period so the first tick fills
    // the labels rather than leaving them blank for the period. If the
    // clock isn't synced (epoch < 1750000000) the labels stay empty. The
    // same string is mirrored to every home tile so the header is
    // consistent across the tileview.
    if (++clock_ticks >= 120) {
        clock_ticks = 0;
        uint32_t now_utc = 0;
        int8_t   offset = 0;
        if (mesh) {
            mesh::RTCClock* rtc = mesh->getRTCClock();
            if (rtc) now_utc = rtc->getCurrentTime();
            P4NodePrefs* prefs = mesh->getNodePrefs();
            if (prefs) offset = prefs->utc_offset_hours;
        }
        char buf[8];
        bool have_time = (now_utc >= 1750000000U);
        if (have_time) {
            time_t local = (time_t)now_utc + (int32_t)offset * 3600;
            struct tm tm_local;
            gmtime_r(&local, &tm_local);
            strftime(buf, sizeof(buf), "%H:%M", &tm_local);
        }
        for (int i = 0; i < MECK_HOME_PAGE_COUNT; i++) {
            if (!lbl_home_clock[i]) continue;
            lv_label_set_text(lbl_home_clock[i], have_time ? buf : "");
        }
        for (int i = 0; i < MECK_SCREEN_HOST_COUNT; i++) {
            if (!lbl_screen_clock[i]) continue;
            lv_label_set_text(lbl_screen_clock[i], have_time ? buf : "");
        }
    }

    // Home tiles: battery % every 5 minutes (600 ticks at 500ms). Colour-
    // coded: green ≥70, orange 40-69, yellow 20-39, red <20. Mirrored to
    // every home tile.
    if (++battery_ticks >= 600) {
        battery_ticks = 0;
        bool available = meck_battery_available();
        uint8_t pct = 0;
        lv_color_t col = lv_color_white();
        if (available) {
            // Chip-based SoC via the BQ27220's coulomb counter. Trustworthy
            // post-calibration (see meck_battery_calibrate() in target.cpp);
            // before calibration landed this used pct_from_voltage as a
            // stopgap. Battery Gauge detail tile still shows raw voltage as
            // a cross-check.
#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
            // Follow the declared source: while the keyboard pack is live,
            // the chip SoC is computed against the internal cell's 1000 mAh
            // and is meaningless for the pack, so use the voltage-curve %
            // -- the same number the pack column on the battery tile shows.
            pct = (g_p4batt_source == 1)
                      ? meck_battery_pct_from_voltage(meck_battery_voltage_mv())
                      : meck_battery_pct_from_chip();
#else
            pct = meck_battery_pct_from_chip();
#endif
            if      (pct >= 70) col = lv_palette_main(LV_PALETTE_GREEN);
            else if (pct >= 40) col = lv_palette_main(LV_PALETTE_ORANGE);
            else if (pct >= 20) col = lv_palette_main(LV_PALETTE_YELLOW);
            else                col = lv_palette_main(LV_PALETTE_RED);
        }
        char batt_txt[12] = "";
        if (available) {
            if (g_batt_show_voltage) {
                uint16_t mv = meck_battery_voltage_mv();
                snprintf(batt_txt, sizeof(batt_txt), "%u.%02u",
                         (unsigned)(mv / 1000), (unsigned)((mv % 1000) / 10));
            } else {
                snprintf(batt_txt, sizeof(batt_txt), "%3u%%", (unsigned)pct);
            }
        }
        for (int i = 0; i < MECK_HOME_PAGE_COUNT; i++) {
            if (!lbl_home_battery[i]) continue;
            if (available) {
                lv_label_set_text(lbl_home_battery[i], batt_txt);
                lv_obj_set_style_text_color(lbl_home_battery[i], col, 0);
            } else {
                lv_label_set_text(lbl_home_battery[i], "");
            }
        }
        for (int i = 0; i < MECK_SCREEN_HOST_COUNT; i++) {
            if (!lbl_screen_battery[i]) continue;
            if (available) {
                lv_label_set_text(lbl_screen_battery[i], batt_txt);
                lv_obj_set_style_text_color(lbl_screen_battery[i], col, 0);
            } else {
                lv_label_set_text(lbl_screen_battery[i], "");
            }
        }
    }

    // Audio indicator: ">>" shown next to the battery while the audio
    // backend is in PLAYING state. Updated every tick so the indicator
    // appears/disappears within 500ms of the user starting or pausing
    // playback. Mirrored to every home tile and every non-home screen.
    {
        const char *txt = (meck_audio_get_state() == MECK_AUDIO_STATE_PLAYING)
                          ? ">>" : "";
        for (int i = 0; i < MECK_HOME_PAGE_COUNT; i++) {
            if (!lbl_home_audio[i]) continue;
            lv_label_set_text(lbl_home_audio[i], txt);
        }
        for (int i = 0; i < MECK_SCREEN_HOST_COUNT; i++) {
            if (!lbl_screen_audio[i]) continue;
            lv_label_set_text(lbl_screen_audio[i], txt);
        }
    }

    // Trace path: poll the Meck instance for a result while the trace
    // screen is active. consumeTraceResult clears the dirty flag and
    // returns true once per match, so this is cheap when idle.
    {
        Meck *mesh_t = meck_get_instance();
        if (mesh_t) {
            if (g_trace_in_flight && scr_trace) {
                Meck::MeckTraceResult tr;
                if (mesh_t->consumeTraceResult(tr)) {
                    trace_render_result(tr);
                } else if (g_trace_sent_ms != 0) {
                    // 30 s timeout — matches the upstream TRACE_TIMEOUT_MS
                    unsigned long elapsed =
                        (unsigned long)millis() - g_trace_sent_ms;
                    if (elapsed >= 30000UL) {
                        if (lbl_trace_status) {
                            lv_label_set_text(lbl_trace_status,
                                "Timed out (no response in 30s)");
                        }
                        g_trace_in_flight = false;
                        g_trace_sent_ms = 0;
                        mesh_t->traceClearPending();
                    }
                }
            }
        }
    }

    // ---- Ping result polling ----
    // Same pattern as trace polling. Poll consumePingResult and show
    // the result overlay on the contact detail screen.
    if (g_ping_in_flight) {
        Meck *mesh_p = meck_get_instance();
        if (mesh_p) {
            Meck::MeckPingResult pr;
            if (mesh_p->consumePingResult(pr)) {
                g_ping_in_flight = false;
                char body[160];
                snprintf(body, sizeof(body),
                         "To: %s\nDuration: %lums\n"
                         "SNR there: %.2fdB\nSNR back: %.2fdB",
                         pr.target_name,
                         (unsigned long)pr.duration_ms,
                         (float)pr.snr_there / 4.0f,
                         (float)pr.snr_back / 4.0f);
                ping_show_result(true, body);
            } else if (g_ping_timeout_ms != 0 &&
                       (unsigned long)millis() >= g_ping_timeout_ms) {
                g_ping_in_flight = false;
                g_ping_timeout_ms = 0;
                mesh_p->pingClearPending();
                ping_show_result(false, "timeout");
            }
        }
    }

    // ---- GPS auto-update for position (every 15 minutes) ----
    // When position_mode == 2 (auto-GPS), refresh lat/lon from GPS snapshot
    // and save to prefs. 1800 ticks at 500ms = 15 minutes.
    static int gps_position_ticks = 1800;
    if (++gps_position_ticks >= 1800) {
        gps_position_ticks = 0;
        Meck *mesh_gps = meck_get_instance();
        if (mesh_gps) {
            P4NodePrefs* prefs = mesh_gps->getNodePrefs();
            if (prefs && prefs->position_mode == 2 && meck_gps_is_enabled()) {
                MeckGpsSnapshot snap;
                meck_gps_get_snapshot(&snap);
                if (snap.fix_valid && snap.lat_e7 != 0 && snap.lon_e7 != 0) {
                    int32_t new_lat = snap.lat_e7;
                    int32_t new_lon = snap.lon_e7;
                    if (new_lat != prefs->position_lat_e7 ||
                        new_lon != prefs->position_lon_e7) {
                        prefs->position_lat_e7 = new_lat;
                        prefs->position_lon_e7 = new_lon;
                        mesh_gps->getDataStore()->savePrefs(*prefs);
                        printf("Position: auto-updated from GPS %.6f,%.6f\n",
                               snap.lat_e7 / 1e7, snap.lon_e7 / 1e7);
                        // Refresh labels if the position screen is visible
                        if (scr_settings_position &&
                            lv_screen_active() == scr_settings_position) {
                            refresh_position_labels();
                        }
                    }
                }
            }
        }
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

    // Battery tile: refresh every tick. Voltage is authoritative; if the
    // BQ27220's reported SoC disagrees with the voltage curve by more than
    // 30 points, surface it so the user can decide what to trust.
    //
    // Gated on the battery tile actually being visible. lbl_battery_detail
    // is created on first visit and never destroyed, so a bare NULL check
    // would poll BQ27220 8 times every 500 ms forever after first visit.
    // That I2C churn (and the PM-lock IPC traffic it generates) combined
    // with active DFS was reliably crashing the device with a system
    // watchdog reset in esp_ipc_isr_waiting_for_finish_cmd.
    bool battery_tile_visible =
        lbl_battery_detail
        && lv_screen_active() == scr_home
        && g_tileview
        && lv_obj_get_parent(lbl_battery_detail) == lv_tileview_get_tile_act(g_tileview)
        && !meck_screen_is_off();
    if (battery_tile_visible) {
#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
        // Two columns, one gauge. The declared source gets the live white
        // column; the other side is grey and inert. Headers keep their colour
        // in both states.
        const bool kbd_src = (g_p4batt_source == 1);
        lv_obj_set_style_text_color(lbl_battery_detail,
            kbd_src ? lv_color_hex(0x808080) : lv_color_white(), 0);
        if (lbl_kbd_batt_detail)
            lv_obj_set_style_text_color(lbl_kbd_batt_detail,
                kbd_src ? lv_color_white() : lv_color_hex(0x808080), 0);
        if (lbl_kbd_batt_cap)
            lv_obj_set_style_text_color(lbl_kbd_batt_cap,
                kbd_src ? lv_color_white() : lv_color_hex(0x808080), 0);
#endif
        if (!meck_battery_available()) {
            lv_label_set_text(lbl_battery_detail, "BQ27220 not detected");
#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
            if (lbl_kbd_batt_detail) lv_label_set_text(lbl_kbd_batt_detail, "");
            if (lbl_kbd_batt_cap)    lv_label_set_text(lbl_kbd_batt_cap, "");
#endif
        } else {
            uint16_t mv         = meck_battery_voltage_mv();
            int16_t  ma         = meck_battery_current_ma();
            uint8_t  pct_chip   = meck_battery_pct_from_chip();
            uint8_t  pct_volts  = meck_battery_pct_from_voltage(mv);
            int8_t   temp_c     = meck_battery_temp_c();
            // Gauging temperature: raw Temperature() register (0.1 K) --
            // the value the chip is actually using for CEDV math. A raw 0
            // (failed read) converts to -273.1 C.
            uint16_t gtemp_raw  = meck_battery_gauging_temp_raw();
            int      gt_tenths  = (int)gtemp_raw - 2731;
            int      gt_abs     = gt_tenths < 0 ? -gt_tenths : gt_tenths;
            uint16_t rem_mah    = meck_battery_remaining_mah();
            uint16_t full_mah   = meck_battery_full_charge_mah();
            uint16_t tte_min    = meck_battery_time_to_empty_min();

            int16_t     current_abs = ma < 0 ? -ma : ma;
#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
            const char* current_label;
            if (current_abs < 5)      current_label = "idle";
            else if (ma > 0)          current_label = "chg";
            else                      current_label = "dis";
#endif

            char tte_buf[32];
            if (tte_min == 0xFFFF || tte_min == 0 || ma > 0) {
                snprintf(tte_buf, sizeof(tte_buf), "--");
            } else {
                snprintf(tte_buf, sizeof(tte_buf), "%uh %02um",
                         (unsigned)(tte_min / 60),
                         (unsigned)(tte_min % 60));
            }

#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
            char buf[512];
            char kbuf[256];
            char capbuf[48];
            if (!kbd_src) {
                // Internal cell live; keyboard side idle. Labels compressed to
                // fit the half-width column.
                int diff = (int)pct_chip - (int)pct_volts;
                if (diff < 0) diff = -diff;
                const char* trust_note = (diff > 30)
                    ? "\nNote: chip vs volt\nSoC differ, check\ncalibration."
                    : "";
                snprintf(buf, sizeof(buf),
                    "V:    %u mV\n"
                    "Chg:  %u%% [chip]\n"
                    "Cur:  %s%d mA %s\n"
                    "Temp: %d C\n"
                    "GaugeT: %s%d.%d C\n"
                    "Rem:  %u/%u\n"
                    "TTE:  %s%s",
                    (unsigned)mv,
                    (unsigned)pct_chip,
                    ma > 0 ? "+" : "", (int)ma, current_label,
                    (int)temp_c,
                    gt_tenths < 0 ? "-" : "", gt_abs / 10, gt_abs % 10,
                    (unsigned)rem_mah, (unsigned)full_mah,
                    tte_buf,
                    trust_note);
                snprintf(kbuf, sizeof(kbuf), "Pack not\nselected");
                snprintf(capbuf, sizeof(capbuf), "Cap: %u mAh",
                         (unsigned)g_p4batt_cap_mah);
            } else {
                // Keyboard pack live. Percentage from the voltage curve --
                // the chip's SoC is calibrated for the internal cell and is
                // meaningless for this pack. Remaining is declared-capacity
                // arithmetic, marked as an estimate.
                uint32_t est_rem =
                    (uint32_t)g_p4batt_cap_mah * (uint32_t)pct_volts / 100u;
                // Pack time-to-empty: estimated remaining divided by the
                // present discharge current. Only shown while actually
                // discharging; "--" when idle or charging. Same ~ estimate
                // marking as the Chg/Rem rows.
                char ktte_buf[16];
                if (ma < 0 && current_abs >= 5) {
                    uint32_t est_min =
                        est_rem * 60u / (uint32_t)current_abs;
                    snprintf(ktte_buf, sizeof(ktte_buf), "~%uh %02um",
                             (unsigned)(est_min / 60),
                             (unsigned)(est_min % 60));
                } else {
                    snprintf(ktte_buf, sizeof(ktte_buf), "--");
                }
                snprintf(buf, sizeof(buf),
                    "Not in circuit\n(source: kbd)");
                snprintf(kbuf, sizeof(kbuf),
                    "V:    %u mV\n"
                    "Chg:  ~%u%% est.\n"
                    "Cur:  %s%d mA %s\n"
                    "Rem:  ~%u/%u est.\n"
                    "TTE:  %s",
                    (unsigned)mv,
                    (unsigned)pct_volts,
                    ma > 0 ? "+" : "", (int)ma, current_label,
                    (unsigned)est_rem, (unsigned)g_p4batt_cap_mah,
                    ktte_buf);
                snprintf(capbuf, sizeof(capbuf), "%s",
                    (g_p4batt_cap_mah == 10000)
                        ? "Cap: 10,000mAh (tap to change)"
                        : "Cap: 1 x 5000mAh (tap to change)");
            }
            lv_label_set_text(lbl_battery_detail, buf);
            if (lbl_kbd_batt_detail) lv_label_set_text(lbl_kbd_batt_detail, kbuf);
            if (lbl_kbd_batt_cap)    lv_label_set_text(lbl_kbd_batt_cap, capbuf);
#else
            const char* current_label_full;
            if (current_abs < 5)      current_label_full = "idle";
            else if (ma > 0)          current_label_full = "charging";
            else                      current_label_full = "discharging";

            int diff = (int)pct_chip - (int)pct_volts;
            if (diff < 0) diff = -diff;
            const char* trust_note = (diff > 30)
                ? "\nNote: BQ27220 SoC disagrees with voltage,\ncell calibration may need attention."
                : "";

            char buf[512];
            snprintf(buf, sizeof(buf),
                "Voltage:    %u mV\n"
                "Charge:     %u%%  [BQ27220]\n"
                "Current:    %s%d mA  (%s)\n"
                "Chip temp:  %d C\n"
                "Gauge temp: %s%d.%d C\n\n"
                "Remaining:  %u / %u mAh\n"
                "Time empty: %s%s",
                (unsigned)mv,
                (unsigned)pct_chip,
                ma > 0 ? "+" : "",      (int)ma,        current_label_full,
                (int)temp_c,
                gt_tenths < 0 ? "-" : "", gt_abs / 10, gt_abs % 10,
                (unsigned)rem_mah,      (unsigned)full_mah,
                tte_buf,
                trust_note);
            lv_label_set_text(lbl_battery_detail, buf);
#endif // CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
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
                // Strip emoji / higher-plane Unicode from the heard
                // node's name so they render cleanly instead of tofu.
                char clean_name[64];
                strip_unrenderable(recent[i].name, clean_name, sizeof(clean_name));
                int w = snprintf(log_buf + pos, sizeof(log_buf) - pos,
                    "%s  [%02X%02X]\n  RSSI %.0f  SNR %.1f  hops %s\n  %s\n\n",
                    clean_name,
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

    // Channel picker unread badges + notification tone trigger
    if (mesh) {
        static int prev_unread[MAX_GROUP_CHANNELS] = {};
        int num_ch = mesh->getActiveChannelCount();
        P4NodePrefs* nprefs = mesh->getNodePrefs();
        for (int i = 0; i < num_ch && i < MAX_GROUP_CHANNELS; i++) {
            if (!lbl_picker_unread[i]) continue;
            int unread = mesh->getUnreadCount(i);
            if (unread > 0) {
                char tmp[16];
                snprintf(tmp, sizeof(tmp), "%d new", unread);
                lv_label_set_text(lbl_picker_unread[i], tmp);
            } else {
                lv_label_set_text(lbl_picker_unread[i], "");
            }
            // Tone trigger: unread count just increased for this channel
            if (unread > prev_unread[i] && nprefs) {
                uint8_t notif = nprefs->channel_notif[i];
                if (notif != 2 && g_notif_sounds.hasSoundForChannel(i)) {
                    // Don't interrupt active audio playback
                    MeckAudioState audio_st = meck_audio_get_state();
                    if (audio_st != MECK_AUDIO_STATE_PLAYING &&
                        audio_st != MECK_AUDIO_STATE_PAUSED) {
                        char path[64];
                        NotifSounds::buildTonePath(path, sizeof(path),
                            g_notif_sounds.getSoundForChannel(i));
                        meck_audio_codec_wake();
                        meck_audio_play_file(path, 0);
                        meck_audio_resume();
                    }
                }
            }
            prev_unread[i] = unread;
        }
    }

    // Direct Messages pseudo-row badge — mirrors the per-channel pattern
    // above but reads from the in-RAM DM rings rather than mesh unread
    // counts. Gated on the label handle existing so it short-circuits
    // before the picker has been built for the first time.
    if (lbl_picker_dm_unread) {
        static int prev_dm_unread = 0;
        int dm_unread = dm_total_unread();
        if (dm_unread > 0) {
            char tmp[16];
            snprintf(tmp, sizeof(tmp), "%d new", dm_unread);
            lv_label_set_text(lbl_picker_dm_unread, tmp);
        } else {
            lv_label_set_text(lbl_picker_dm_unread, "");
        }
        // Tone trigger: DM unread count just increased
        if (dm_unread > prev_dm_unread && g_notif_sounds.hasSoundForChannel(0xFF)) {
            MeckAudioState audio_st = meck_audio_get_state();
            if (audio_st != MECK_AUDIO_STATE_PLAYING &&
                audio_st != MECK_AUDIO_STATE_PAUSED) {
                char path[64];
                NotifSounds::buildTonePath(path, sizeof(path),
                    g_notif_sounds.getSoundForChannel(0xFF));
                meck_audio_codec_wake();
                meck_audio_play_file(path, 0);
                meck_audio_resume();
            }
        }
        prev_dm_unread = dm_unread;
    }

    // GPS tile: refresh detail block at 500ms tick.
    // Data is populated by device_gps_task in main.cpp at ~1Hz, so we may
    // see the same values across two consecutive ticks — that's fine.
    // Sentence rate is computed as a delta between consecutive timer ticks,
    // smoothed with a running average over 4 samples (~2 sec window) so
    // the displayed rate doesn't jitter.
    if (lbl_gps_detail) {
        // Fast path: GPS gated off by user. Show a clear disabled state
        // so a stale snapshot doesn't read like "L76K acquiring forever".
        if (!meck_gps_is_enabled()) {
            lv_label_set_text(lbl_gps_detail,
                "GPS: OFF\nLong-press to enable");
        } else {
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
        }   // end of `if (meck_gps_is_enabled())` else branch
    }
}

// ============================================================================
// Public API
// ============================================================================

// ============================================================================
// Boot-time hydration of per-contact DM rings from /sdcard/meshcore/dms.bin
// ----------------------------------------------------------------------------
// Called once from meck_ui_init after the mesh + contacts are loaded.
// Reads the tail of dms.bin (newest records win when the file is larger
// than the load cap), iterates each record, looks up the matching
// contact by comparing dm_peer_hash (first 4 bytes of pub_key) against
// each contact's pub_key. On match, append to the contact's ring as an
// incoming DM.
//
// Records whose dm_peer_hash matches no current contact are silently
// dropped (the contact may have been deleted since the DM was received).
// In the rare hash-collision case (~1 in 3800 at 1500 contacts), DMs
// would be attributed to whichever colliding contact has the lower
// index — graceful failure, not catastrophic.
//
// Cap: MECK_DM_LOAD_CAP records on boot. Far more than typical use
// (1500 contacts × 20 messages per ring = 30000 records is the
// theoretical max in-RAM capacity, but daily use produces far less)
// and the loader caps memory and SD I/O time at boot.
// ============================================================================

#define MECK_DM_LOAD_CAP 2000

static void load_persisted_dms_into_rings() {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;

    // Heap-allocate the record buffer from PSRAM — 2000 records of 320
    // bytes is 640 KB which would blow any stack and is too big for
    // the LVGL task's regular heap. PSRAM is fine for transient buffers.
    P4MsgFileRecord* recs = (P4MsgFileRecord*)heap_caps_calloc(
        MECK_DM_LOAD_CAP, sizeof(P4MsgFileRecord), MALLOC_CAP_SPIRAM);
    if (!recs) {
        printf("load_persisted_dms_into_rings: PSRAM alloc failed\n");
        return;
    }

    int loaded = mesh->loadDMRecords(recs, MECK_DM_LOAD_CAP);
    if (loaded <= 0) {
        free(recs);
        return;
    }
    printf("load_persisted_dms_into_rings: loaded %d records from SD\n", loaded);

    // Build a contact lookup table indexed by the 4-byte pub_key prefix
    // so the demux is O(R+C) not O(R*C). R=records (up to 2000),
    // C=contacts (up to 2000 here); the naive nested-loop pass would
    // be 4M memcmps. The lookup table is two parallel PSRAM arrays:
    // hashes[i] holds the 4-byte prefix, idx[i] holds the contact_idx.
    // Linear scan over hashes[] is fast (4-byte memcmp, contiguous
    // memory), and the contact count is the bound rather than the
    // record count.
    int num_contacts = mesh->getNumContacts();
    uint32_t* hashes = (uint32_t*)heap_caps_calloc(
        num_contacts > 0 ? num_contacts : 1, sizeof(uint32_t),
        MALLOC_CAP_SPIRAM);
    int* idx_table = (int*)heap_caps_calloc(
        num_contacts > 0 ? num_contacts : 1, sizeof(int),
        MALLOC_CAP_SPIRAM);
    if (!hashes || !idx_table) {
        printf("load_persisted_dms_into_rings: PSRAM alloc for lookup failed\n");
        if (hashes) free(hashes);
        if (idx_table) free(idx_table);
        free(recs);
        return;
    }
    {
        ContactInfo ci;
        for (int c = 0; c < num_contacts; c++) {
            if (!mesh->getContactByIdx((uint32_t)c, ci)) continue;
            memcpy(&hashes[c], ci.id.pub_key, sizeof(uint32_t));
            idx_table[c] = c;
        }
    }

    int demuxed = 0;
    int dropped = 0;
    for (int r = 0; r < loaded; r++) {
        const P4MsgFileRecord& rec = recs[r];
        if ((rec.flags & 0x01) == 0) continue;
        if (rec.channel_idx != 0xFF) continue;

        // Linear scan of the prefix table. Could be replaced with a
        // hash map for O(1) but at 2000 contacts the linear scan is
        // already < 1ms per record on the P4.
        uint32_t target = 0;
        memcpy(&target, &rec.dm_peer_hash, sizeof(target));
        int contact_idx = -1;
        for (int c = 0; c < num_contacts; c++) {
            if (hashes[c] == target) {
                contact_idx = idx_table[c];
                break;
            }
        }
        if (contact_idx < 0) {
            dropped++;
            continue;
        }

        DMMessage m = {};
        m.timestamp    = rec.timestamp;
        m.outgoing     = (rec.flags & P4_DM_FLAG_OUTGOING) != 0;
        m.path_len     = rec.path_len;
        m.snr_x4       = rec.snr_x4;
        m.expected_ack = 0;
        m.acked        = (rec.flags & P4_DM_FLAG_ACKED) != 0;
        // file_offset stays 0: the loader doesn't track per-record
        // offsets and a loaded outgoing DM doesn't need to be rewritten
        // (no live ACK table entry exists post-reboot, so no new ACK
        // will match to trigger a rewrite).
        m.file_offset      = 0;
        m.persisted_acked  = m.acked;  // on-disk and RAM match at load time
        m.from_disk        = true;
        strncpy(m.text, rec.text, sizeof(m.text) - 1);
        m.text[sizeof(m.text) - 1] = '\0';
        dm_ring_append(contact_idx, m);
        demuxed++;
    }

    // Clear unread on every ring — startup is "user has seen what's
    // there", same as how channel-message loads don't mark unread.
    // Without this the home screen would show every persisted DM as
    // unread on each boot, which would be intrusive.
    for (int c = 0; c < MAX_CONTACTS; c++) {
        dm_ring_clear_unread(c);
    }

    printf("load_persisted_dms_into_rings: demuxed=%d dropped=%d\n",
           demuxed, dropped);
    free(hashes);
    free(idx_table);
    free(recs);
}

// ============================================================================
// DM receive dispatcher
// ----------------------------------------------------------------------------
// Invoked by meck_drain_pending_dms (on the LVGL task). Looks up the
// sender's contact by pub_key, appends the message to that contact's
// ring, increments the unread counter. If the user is currently sitting
// on the DM conversation view for that exact contact, triggers a
// re-render so the bubble appears live.
//
// If the sender has no matching contact entry (e.g. they were just
// auto-removed, or auto-add is off and they're a stranger), the DM is
// dropped with a serial log. This is the simplest correct behaviour;
// piece 4's inbox could later show a "pending" pile for unknown senders.
// ============================================================================

static void meck_dm_recv_dispatch(const uint8_t* from_pub_key,
                                  const char* from_name,
                                  const char* text,
                                  uint32_t sender_timestamp,
                                  uint8_t path_len,
                                  int8_t snr_x4) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;

    // Find the contact index by pub_key. We have to scan because there's
    // no public-key → idx index in P4DataStore. MAX_CONTACTS is 1500;
    // linear is fine at one-per-DM frequency.
    int contact_idx = -1;
    int n = mesh->getNumContacts();
    ContactInfo ci;
    for (int i = 0; i < n; i++) {
        if (!mesh->getContactByIdx((uint32_t)i, ci)) continue;
        if (memcmp(ci.id.pub_key, from_pub_key, PUB_KEY_SIZE) == 0) {
            contact_idx = i;
            break;
        }
    }
    if (contact_idx < 0) {
        printf("meck_dm_recv_dispatch: no contact for sender %s, dropping DM\n",
               from_name);
        return;
    }

    DMMessage m = {};
    m.timestamp    = sender_timestamp;
    m.outgoing     = false;
    m.path_len     = path_len;
    m.snr_x4       = snr_x4;
    m.expected_ack = 0;
    m.acked        = false;
    m.file_offset  = 0;
    m.persisted_acked = false;
    m.from_disk    = false;
    strncpy(m.text, text, sizeof(m.text) - 1);
    m.text[sizeof(m.text) - 1] = '\0';

    // Persist to /sdcard/meshcore/dms.bin so the message survives a
    // reboot. P4MsgFileRecord layout shared with the channel store —
    // we populate channel_idx=0xFF (the DM convention) and dm_peer_hash
    // = first 4 bytes of the sender's pub_key. flags = valid bit only
    // (not outgoing, not acked — incoming DMs are immutable once
    // received).
    P4MsgFileRecord rec = {};
    rec.timestamp     = sender_timestamp;
    memcpy(&rec.dm_peer_hash, from_pub_key, sizeof(rec.dm_peer_hash));
    rec.channel_idx   = 0xFF;
    rec.path_len      = path_len;
    rec.snr_x4        = snr_x4;
    rec.flags         = P4_DM_FLAG_VALID;
    rec.heard_count   = 0;             // unused for received DMs
    memset(rec.path, 0, sizeof(rec.path));
    strncpy(rec.text, text, sizeof(rec.text) - 1);
    rec.text[sizeof(rec.text) - 1] = '\0';
    uint32_t saved_offset = 0;
    if (!mesh->saveDMRecord(rec, &saved_offset)) {
        printf("meck_dm_recv_dispatch: SD save failed for DM from %s\n", from_name);
    }
    m.file_offset = saved_offset;

    dm_ring_append(contact_idx, m);

    // Live re-render if this DM is for the conversation currently open.
    if (g_in_dm_mode && g_active_dm_contact_idx == contact_idx) {
        rebuild_dm_bubbles(contact_idx);
        // We're looking at it — clear unread so the inbox doesn't badge
        // the contact for a message the user has clearly just seen.
        dm_ring_clear_unread(contact_idx);
    }

    printf("meck_dm_recv_dispatch: stored DM from %s (idx=%d)\n",
           from_name, contact_idx);
}

// ============================================================================
// Room post receive dispatcher (Piece B)
// ----------------------------------------------------------------------------
// Invoked by meck_drain_pending_posts on the LVGL task. Locates the
// room contact by pub_key, appends the post to that contact's PostRing,
// persists to /sdcard/meshcore/posts.bin, and if the user is currently
// sitting on the room messages view for that exact room, triggers a
// live re-render.
//
// Drops the post if there's no matching contact (server pushed a post
// from a room we don't have in our contact list — shouldn't happen
// since login implies the contact exists, but defensive).
// ============================================================================

static void meck_post_recv_dispatch(const uint8_t* room_pub_key,
                                    const char* room_name,
                                    const uint8_t* sender_prefix,
                                    const char* text,
                                    uint32_t sender_timestamp,
                                    uint8_t path_len,
                                    int8_t snr_x4) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;

    int contact_idx = -1;
    int n = mesh->getNumContacts();
    ContactInfo ci;
    for (int i = 0; i < n; i++) {
        if (!mesh->getContactByIdx((uint32_t)i, ci)) continue;
        if (memcmp(ci.id.pub_key, room_pub_key, PUB_KEY_SIZE) == 0) {
            contact_idx = i;
            break;
        }
    }
    if (contact_idx < 0) {
        printf("meck_post_recv_dispatch: no contact for room %s, dropping post\n",
               room_name);
        return;
    }

    // Dedup against existing ring entries by (timestamp, sender_prefix).
    // Two sources of duplicates we need to swallow here:
    //   1. Server retransmits the post when its ACK to us times out
    //      (visible in logs as onSendTimeout followed by the same post
    //      arriving again with the same timestamp).
    //   2. sendLogin resets sync_since to 0 for room contacts (see
    //      MeckMesh.h sendLogin's room-server branch), so every login
    //      causes the server to re-push every post we've ever seen.
    // Skip BEFORE persisting so we don't bloat posts.bin with copies.
    if (post_ring_has_post(contact_idx, sender_timestamp, sender_prefix)) {
        printf("meck_post_recv_dispatch: duplicate post (ts=%u sender=%02X%02X%02X%02X) "
               "for room %s, skipping\n",
               (unsigned)sender_timestamp,
               sender_prefix[0], sender_prefix[1], sender_prefix[2], sender_prefix[3],
               room_name);
        return;
    }

    PostMessage pm = {};
    pm.timestamp   = sender_timestamp;
    memcpy(pm.sender_prefix, sender_prefix, 4);
    pm.path_len    = path_len;
    pm.snr_x4      = snr_x4;
    pm.file_offset = 0;
    pm.from_disk   = false;
    strncpy(pm.text, text, sizeof(pm.text) - 1);
    pm.text[sizeof(pm.text) - 1] = '\0';

    // Persist to /sdcard/meshcore/posts.bin.
    P4PostFileRecord rec = {};
    rec.timestamp = sender_timestamp;
    memcpy(rec.room_pub_key_prefix, room_pub_key, 4);
    memcpy(rec.sender_prefix, sender_prefix, 4);
    rec.flags     = P4_POST_FLAG_VALID;
    rec.path_len  = path_len;
    rec.snr_x4    = snr_x4;
    rec.reserved  = 0;
    strncpy(rec.text, text, sizeof(rec.text) - 1);
    rec.text[sizeof(rec.text) - 1] = '\0';
    uint32_t saved_offset = 0;
    if (!mesh->savePostRecord(rec, &saved_offset)) {
        printf("meck_post_recv_dispatch: SD save failed for post from %s\n", room_name);
    }
    pm.file_offset = saved_offset;

    post_ring_append(contact_idx, pm);

    // Live re-render if the user is on this room's screen.
    if (g_in_room_mode && g_active_room_contact_idx == contact_idx) {
        rebuild_room_bubbles(contact_idx);
    }

    printf("meck_post_recv_dispatch: stored post from room %s (idx=%d)\n",
           room_name, contact_idx);
}

// ============================================================================
// Persisted post loader (Piece B)
// ----------------------------------------------------------------------------
// Hydrate per-room PostRings from /sdcard/meshcore/posts.bin on boot.
// Same demux strategy as load_persisted_dms_into_rings: build a
// 4-byte-prefix → contact_idx lookup table, walk records, match on
// room_pub_key_prefix.
// ============================================================================
#define MECK_POST_LOAD_CAP 2000

static void load_persisted_posts_into_rings() {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;

    P4PostFileRecord* recs = (P4PostFileRecord*)heap_caps_calloc(
        MECK_POST_LOAD_CAP, sizeof(P4PostFileRecord), MALLOC_CAP_SPIRAM);
    if (!recs) {
        printf("load_persisted_posts_into_rings: PSRAM alloc failed\n");
        return;
    }

    int loaded = mesh->loadPostRecords(recs, MECK_POST_LOAD_CAP);
    if (loaded <= 0) {
        free(recs);
        return;
    }
    printf("load_persisted_posts_into_rings: loaded %d records from SD\n", loaded);

    int num_contacts = mesh->getNumContacts();
    uint32_t* hashes = (uint32_t*)heap_caps_calloc(
        num_contacts > 0 ? num_contacts : 1, sizeof(uint32_t),
        MALLOC_CAP_SPIRAM);
    int* idx_table = (int*)heap_caps_calloc(
        num_contacts > 0 ? num_contacts : 1, sizeof(int),
        MALLOC_CAP_SPIRAM);
    if (!hashes || !idx_table) {
        printf("load_persisted_posts_into_rings: PSRAM alloc for lookup failed\n");
        if (hashes) free(hashes);
        if (idx_table) free(idx_table);
        free(recs);
        return;
    }
    {
        ContactInfo ci;
        for (int c = 0; c < num_contacts; c++) {
            if (!mesh->getContactByIdx((uint32_t)c, ci)) continue;
            memcpy(&hashes[c], ci.id.pub_key, sizeof(uint32_t));
            idx_table[c] = c;
        }
    }

    int demuxed = 0;
    int dropped = 0;
    for (int r = 0; r < loaded; r++) {
        const P4PostFileRecord& rec = recs[r];
        if ((rec.flags & P4_POST_FLAG_VALID) == 0) continue;

        uint32_t target = 0;
        memcpy(&target, rec.room_pub_key_prefix, sizeof(target));
        int contact_idx = -1;
        for (int c = 0; c < num_contacts; c++) {
            if (hashes[c] == target) {
                contact_idx = idx_table[c];
                break;
            }
        }
        if (contact_idx < 0) {
            dropped++;
            continue;
        }

        // Dedup against the in-progress ring. The on-disk file may
        // contain duplicate records from sessions before the dispatch
        // dedup landed (server retransmits + re-pushes on each login).
        // Skipping here means the first occurrence per (ts, sender)
        // wins and subsequent copies are dropped silently.
        if (post_ring_has_post(contact_idx, rec.timestamp, rec.sender_prefix)) {
            dropped++;
            continue;
        }

        PostMessage pm = {};
        pm.timestamp   = rec.timestamp;
        memcpy(pm.sender_prefix, rec.sender_prefix, 4);
        pm.path_len    = rec.path_len;
        pm.snr_x4      = rec.snr_x4;
        pm.file_offset = 0;
        pm.from_disk   = true;
        strncpy(pm.text, rec.text, sizeof(pm.text) - 1);
        pm.text[sizeof(pm.text) - 1] = '\0';
        post_ring_append(contact_idx, pm);
        demuxed++;
    }

    printf("load_persisted_posts_into_rings: demuxed=%d dropped=%d\n",
           demuxed, dropped);
    free(hashes);
    free(idx_table);
    free(recs);
}

// ============================================================================
// DM send completion dispatcher
// ----------------------------------------------------------------------------
// Invoked by meck_drain_pending_dm_sends (on the LVGL task) after a
// queued DM has been transmitted. Locates the matching outgoing
// DMMessage in the contact's ring — the most recent outgoing entry with
// expected_ack == 0 — and writes the expected_ack and est_timeout onto
// it. The bubble's status footer (rebuild_dm_bubbles) reads these via
// Meck::lookupDMAckStatus on next render.
//
// "Most recent outgoing with expected_ack==0" is robust against the
// rare race where two sends are queued back-to-back: the first send's
// completion writes onto the older bubble (deeper in the ring), the
// second send's completion writes onto the newer one. As long as the
// completions arrive in send order (they will — mesh task processes
// them sequentially from the queue), the pairing is correct.
// ============================================================================

static void meck_dm_sent_dispatch(int contact_idx,
                                  uint32_t expected_ack,
                                  uint32_t est_timeout_ms) {
    if (contact_idx < 0 || contact_idx >= MAX_CONTACTS) return;
    DMRing* r = g_dm_rings[contact_idx];
    if (!r || r->count == 0) {
        printf("meck_dm_sent_dispatch: no ring for contact[%d]\n", contact_idx);
        return;
    }

    // Walk newest-first looking for an outgoing message with expected_ack
    // still 0 (i.e. send hasn't been correlated yet). Found one → write
    // expected_ack onto it and stop. The ring stores up to
    // MECK_DM_PER_CONTACT slots; the matching slot is almost always at
    // newest_idx since the send was just appended on the LVGL task.
    int idx = r->newest_idx;
    for (int i = 0; i < r->count; i++) {
        DMMessage& m = r->messages[idx];
        if (m.outgoing && m.expected_ack == 0) {
            m.expected_ack = expected_ack;
            // est_timeout isn't stored on DMMessage — it's looked up
            // fresh from Meck's ACK table each render. Logging it here
            // so a serial trace can correlate.
            printf("meck_dm_sent_dispatch: contact[%d] slot[%d] ack=0x%08X timeout=%ums\n",
                   contact_idx, idx, (unsigned)expected_ack, (unsigned)est_timeout_ms);

            // Live re-render if this DM is for the conversation currently
            // open — refreshes the bubble's status from "Sending..."
            // to whatever lookupDMAckStatus reports right now (still
            // pending, since the ACK probably hasn't landed yet, but
            // the bubble now has an ack handle for future renders).
            if (g_in_dm_mode && g_active_dm_contact_idx == contact_idx) {
                rebuild_dm_bubbles(contact_idx);
            }
            return;
        }
        idx = (idx - 1 + MECK_DM_PER_CONTACT) % MECK_DM_PER_CONTACT;
    }
    printf("meck_dm_sent_dispatch: contact[%d] no matching outgoing slot\n",
           contact_idx);
}

// ============================================================================
// Repeater Admin — piece 3
// ----------------------------------------------------------------------------
// Login screen, RAM-only password cache, reveal-then-dot textarea
// rendering, plus a temporary "logged in" placeholder screen so the
// login path can be exercised end-to-end before piece 4 builds the
// real admin home.
// ============================================================================

// ---- Password cache (RAM-only, 8-slot LRU) ----

static const char* admin_pwd_cache_get(int contact_idx) {
    for (int i = 0; i < g_admin_pwd_cache_count; i++) {
        if (g_admin_pwd_cache[i].contact_idx == contact_idx) {
            return g_admin_pwd_cache[i].password;
        }
    }
    return nullptr;
}

static void admin_pwd_cache_put(int contact_idx, const char* password) {
    if (!password) return;
    // Existing entry? Update in place.
    for (int i = 0; i < g_admin_pwd_cache_count; i++) {
        if (g_admin_pwd_cache[i].contact_idx == contact_idx) {
            strncpy(g_admin_pwd_cache[i].password, password,
                    sizeof(g_admin_pwd_cache[i].password) - 1);
            g_admin_pwd_cache[i].password[sizeof(g_admin_pwd_cache[i].password) - 1] = '\0';
            return;
        }
    }
    // Fresh slot or LRU evict?
    if (g_admin_pwd_cache_count < MECK_ADMIN_PWD_CACHE_SIZE) {
        int slot = g_admin_pwd_cache_count++;
        g_admin_pwd_cache[slot].contact_idx = contact_idx;
        strncpy(g_admin_pwd_cache[slot].password, password,
                sizeof(g_admin_pwd_cache[slot].password) - 1);
        g_admin_pwd_cache[slot].password[sizeof(g_admin_pwd_cache[slot].password) - 1] = '\0';
    } else {
        // Shift everything down, evicting slot 0 (oldest). New entry
        // lands in the last slot.
        for (int i = 0; i < MECK_ADMIN_PWD_CACHE_SIZE - 1; i++) {
            g_admin_pwd_cache[i] = g_admin_pwd_cache[i + 1];
        }
        int slot = MECK_ADMIN_PWD_CACHE_SIZE - 1;
        g_admin_pwd_cache[slot].contact_idx = contact_idx;
        strncpy(g_admin_pwd_cache[slot].password, password,
                sizeof(g_admin_pwd_cache[slot].password) - 1);
        g_admin_pwd_cache[slot].password[sizeof(g_admin_pwd_cache[slot].password) - 1] = '\0';
    }
}

// ---- Password textarea rendering ----
//
// The textarea content always shows the masked view: cleartext for the
// most-recently-typed char within MECK_ADMIN_PWD_REVEAL_MS, dots
// elsewhere. The "raw" buffer (g_admin_pwd_raw) holds the actual
// password. If the eye-toggle is on, the full plaintext is shown.
static void admin_render_password_masked() {
    if (!ta_admin_password) return;

    // Build the display string. Falls into two paths:
    //   - empty buffer → display = ""
    //   - non-empty buffer → display = dots (with optional last-char
    //     reveal), or full plaintext if Show/Hide is toggled on.
    //
    // Each • (U+2022) is 3 bytes in UTF-8 (0xE2 0x80 0xA2). The bullet
    // glyph is included in the Meck Montserrat fonts as of v0.3.5.
    char display[MECK_ADMIN_PASSWORD_MAX * 4 + 1] = {};

    if (g_admin_pwd_len == 0) {
        // Empty buffer — display stays "".
    } else if (g_admin_pwd_reveal_all) {
        // Full reveal — show the raw buffer verbatim.
        strncpy(display, g_admin_pwd_raw, sizeof(display) - 1);
    } else {
        unsigned long now = (unsigned long)esp_log_timestamp();
        bool show_last_char =
            (g_admin_pwd_last_char_ms != 0 &&
             (now - g_admin_pwd_last_char_ms) < MECK_ADMIN_PWD_REVEAL_MS);

        int n_dots = show_last_char ? (g_admin_pwd_len - 1) : g_admin_pwd_len;
        if (n_dots < 0) n_dots = 0;
        int pos = 0;
        for (int i = 0; i < n_dots && pos < (int)sizeof(display) - 4; i++) {
            display[pos++] = (char)0xE2;
            display[pos++] = (char)0x80;
            display[pos++] = (char)0xA2;
        }
        if (show_last_char && g_admin_pwd_len > 0 &&
            pos < (int)sizeof(display) - 1) {
            display[pos++] = g_admin_pwd_raw[g_admin_pwd_len - 1];
        }
        display[pos] = '\0';
    }

    // Compare against current textarea content so we only call set_text
    // when something actually changed. Avoids LVGL re-rendering on every
    // 500ms tick when nothing is happening AND prevents infinite
    // recursion through our own LV_EVENT_VALUE_CHANGED handler — which
    // would otherwise blow the stack on screen entry.
    //
    // Belt and braces: the g_admin_pwd_render_in_progress flag also
    // gates the change handler so even if a future LVGL version starts
    // firing VALUE_CHANGED on no-op set_text calls, we still don't
    // re-enter.
    const char* current = lv_textarea_get_text(ta_admin_password);
    if (!current || strcmp(current, display) != 0) {
        g_admin_pwd_render_in_progress = true;
        lv_textarea_set_text(ta_admin_password, display);
        g_admin_pwd_render_in_progress = false;
    }
}

// ---- Status line helper ----
static void admin_login_show_status(const char* text, lv_color_t color) {
    if (!lbl_admin_login_status) return;
    lv_label_set_text(lbl_admin_login_status, text ? text : "");
    lv_obj_set_style_text_color(lbl_admin_login_status, color, 0);
}

// ---- Event handlers ----

static void on_admin_password_textarea_change(lv_event_t *e) {
    // Programmatic update from admin_render_password_masked — not a
    // user keypress. Skip; otherwise we infinite-recurse and blow the
    // stack.
    if (g_admin_pwd_render_in_progress) return;

    // Read the current textarea content. We're going to derive the raw
    // password from the diff between the previous masked display and
    // the new content, then re-mask.
    //
    // Simpler approach: trust LVGL's keyboard to send LV_EVENT_VALUE_CHANGED
    // on each keypress, but inspect the change relative to our raw
    // buffer. LVGL textareas don't expose per-key events directly; the
    // VALUE_CHANGED carries the full updated text. We have to figure
    // out what changed.
    //
    // For a password field this is tractable: the only legitimate
    // operations are append-one-char and delete-one-char (no paste,
    // no cursor movement). Compare length to detect which.
    if (!ta_admin_password) return;
    const char* now_text = lv_textarea_get_text(ta_admin_password);
    if (!now_text) return;

    // Count visible "characters" in the new text. With dot-masking,
    // each • is 3 bytes; with reveal-all, every byte is a char. We
    // also have the in-flight "most recent char as cleartext" case
    // where the textarea has (n-1) dots plus one cleartext char.
    //
    // The pragmatic approach: count UTF-8 codepoints in the new text.
    int new_chars = 0;
    for (const char* p = now_text; *p; ) {
        unsigned char c = (unsigned char)*p;
        if (c < 0x80) p += 1;
        else if ((c & 0xE0) == 0xC0) p += 2;
        else if ((c & 0xF0) == 0xE0) p += 3;
        else if ((c & 0xF8) == 0xF0) p += 4;
        else p += 1;
        new_chars++;
    }

    if (new_chars > g_admin_pwd_len) {
        // Appended characters. Find the last codepoint in the new text
        // and treat it as the just-typed character. Multi-char appends
        // (paste) we can't recover the chars for — just append the
        // last visible char and call it good. The expected operation
        // is single-keypress so this is the normal path.
        const char* last_cp = now_text;
        for (const char* p = now_text; *p; ) {
            last_cp = p;
            unsigned char c = (unsigned char)*p;
            if (c < 0x80) p += 1;
            else if ((c & 0xE0) == 0xC0) p += 2;
            else if ((c & 0xF0) == 0xE0) p += 3;
            else if ((c & 0xF8) == 0xF0) p += 4;
            else p += 1;
        }
        // Only accept ASCII chars into the raw buffer — anything else
        // is a dot (•) from our own masking that we shouldn't store.
        // If a non-ASCII char arrives (UTF-8 from the keyboard), we
        // log and skip; the password protocol on the repeater side is
        // ASCII-only anyway.
        unsigned char last_byte = (unsigned char)*last_cp;
        if (last_byte < 0x80 && last_byte >= 0x20 && last_byte < 0x7F) {
            if (g_admin_pwd_len < MECK_ADMIN_PASSWORD_MAX) {
                g_admin_pwd_raw[g_admin_pwd_len++] = (char)last_byte;
                g_admin_pwd_raw[g_admin_pwd_len] = '\0';
                g_admin_pwd_last_char_ms = (unsigned long)esp_log_timestamp();
            }
        } else if (last_byte == 0xE2) {
            // The • we just rendered — ignore, this isn't a real
            // keypress. Re-render to undo any state change.
        }
    } else if (new_chars < g_admin_pwd_len) {
        // Characters deleted. Truncate the raw buffer to match the
        // new length. (We assume deletes happen one-at-a-time via
        // backspace.)
        int delta = g_admin_pwd_len - new_chars;
        g_admin_pwd_len -= delta;
        if (g_admin_pwd_len < 0) g_admin_pwd_len = 0;
        g_admin_pwd_raw[g_admin_pwd_len] = '\0';
    }
    // Always re-render after a change so the dot mask reasserts.
    admin_render_password_masked();
}

static void on_admin_password_eye_tap(lv_event_t *e) {
    g_admin_pwd_reveal_all = !g_admin_pwd_reveal_all;
    if (btn_admin_password_eye) {
        lv_obj_t* eye_lbl = lv_obj_get_child(btn_admin_password_eye, 0);
        if (eye_lbl) {
            lv_label_set_text(eye_lbl,
                g_admin_pwd_reveal_all ? LV_SYMBOL_EYE_CLOSE : LV_SYMBOL_EYE_OPEN);
        }
    }
    admin_render_password_masked();
}

static void on_admin_password_focused(lv_event_t *e) {
    if (kb_admin_password && ta_admin_password) {
        lv_keyboard_set_textarea(kb_admin_password, ta_admin_password);
        // Hardware keyboard: keep the on-screen keyboard down; the binding
        // above stays so the poll routes Enter/Esc through it.
        if (meck_hw_keyboard_active()) return;
        lv_obj_clear_flag(kb_admin_password, LV_OBJ_FLAG_HIDDEN);
    }
}

static void on_admin_password_defocused(lv_event_t *e) {
    if (kb_admin_password) {
        lv_obj_add_flag(kb_admin_password, LV_OBJ_FLAG_HIDDEN);
    }
}

static void on_admin_login_submit(lv_event_t *e) {
    if (g_admin_login_contact_idx < 0) return;
    if (g_admin_ui_state == ADMIN_STATE_LOGGING_IN) return;  // debounce

    // Allow empty password — the repeater accepts blank for the
    // "already in ACL" case, and the user might be a known admin who
    // wants to test the path. The repeater itself will silently fail
    // if the password doesn't match.
    printf("Admin: submitting login to contact[%d], pwd_len=%d\n",
           g_admin_login_contact_idx, g_admin_pwd_len);

    g_admin_ui_state = ADMIN_STATE_LOGGING_IN;
    g_admin_login_sent_ms = (unsigned long)esp_log_timestamp();
    g_admin_login_timeout_ms = 15000;  // initial guess; updated by send-result
    admin_login_show_status("Logging in...",
                            lv_palette_main(LV_PALETTE_YELLOW));

    // Disable the Login button while in-flight so a double-tap can't
    // queue a second send.
    if (btn_admin_login_submit) {
        lv_obj_add_state(btn_admin_login_submit, LV_STATE_DISABLED);
    }

    meck_request_admin_login(g_admin_login_contact_idx, g_admin_pwd_raw);
}

static void on_admin_login_back(lv_event_t *e) {
    printf("Admin: back from login screen\n");
    // Tear down any active session and any pending queues. Wipes the
    // password buffer in meck_app's queue too.
    meck_admin_clear_session();
    g_admin_ui_state = ADMIN_STATE_IDLE;
    // Wipe the raw password buffer on the UI side too.
    memset(g_admin_pwd_raw, 0, sizeof(g_admin_pwd_raw));
    g_admin_pwd_len = 0;
    g_admin_pwd_reveal_all = false;
    g_admin_pwd_last_char_ms = 0;
    if (scr_contact_detail) lv_screen_load(scr_contact_detail);
}

static void on_admin_home_back(lv_event_t *e) {
    printf("Admin: back from home screen\n");
    meck_admin_clear_session();
    g_admin_ui_state = ADMIN_STATE_IDLE;
    memset(g_admin_pwd_raw, 0, sizeof(g_admin_pwd_raw));
    g_admin_pwd_len = 0;
    // Reset cached status state so a re-login starts fresh.
    g_admin_status_last_update = 0;
    g_admin_status_in_flight   = false;
    g_admin_send_advert_in_flight = false;
    g_admin_cmd_in_flight         = false;
    g_admin_fw_in_flight          = false;
    g_admin_neighbours_in_flight  = false;
    if (scr_contact_detail) lv_screen_load(scr_contact_detail);
}

// Generic subpage back handler — returns to the admin home menu list.
// Used by every admin subpage's back button.
static void on_admin_subpage_back(lv_event_t *e) {
    if (scr_admin_home) lv_screen_load(scr_admin_home);
}

// Menu-row taps → navigate to subpage screens.
static void on_admin_menu_status_tap(lv_event_t *e) {
    printf("Admin menu: Status tapped\n");
    if (scr_admin_status) lv_screen_load(scr_admin_status);
}

static void on_admin_menu_cmd_tap(lv_event_t *e) {
    printf("Admin menu: Cmd Line tapped\n");
    if (scr_admin_cmd) lv_screen_load(scr_admin_cmd);
}

static void on_admin_menu_settings_tap(lv_event_t *e) {
    printf("Admin menu: Settings tapped\n");
    // Rebuild the list every entry so the visible set reflects the
    // current session role (admin sees all rows, guest sees only the
    // non-admin-gated ones).
    admin_settings_rebuild_list();
    if (scr_admin_settings) lv_screen_load(scr_admin_settings);
}

static void on_admin_menu_send_advert_tap(lv_event_t *e) {
    printf("Admin menu: Send Advert tapped\n");
    // Reset the in-flight state and response label so each visit
    // starts clean.
    g_admin_send_advert_in_flight = false;
    if (lbl_admin_send_advert_status) {
        lv_label_set_text(lbl_admin_send_advert_status,
                          "Tap the button to send an advert command.");
        lv_obj_set_style_text_color(lbl_admin_send_advert_status,
                                    lv_color_make(180, 180, 180), 0);
    }
    if (lbl_admin_send_advert_response) {
        lv_label_set_text(lbl_admin_send_advert_response, "");
    }
    if (btn_admin_send_advert) {
        lv_obj_clear_state(btn_admin_send_advert, LV_STATE_DISABLED);
    }
    if (scr_admin_send_advert) lv_screen_load(scr_admin_send_advert);
}

// Update the persistent banner colour + text based on session role.
// Green for admin, yellow for guest. Called from login dispatch.
static void admin_home_update_banner(bool is_admin, const char* contact_name) {
    if (!obj_admin_banner || !lbl_admin_banner) return;
    lv_obj_set_style_bg_color(obj_admin_banner,
        is_admin ? lv_palette_darken(LV_PALETTE_GREEN, 2)
                 : lv_palette_darken(LV_PALETTE_YELLOW, 3),
        0);
    char buf[96];
    snprintf(buf, sizeof(buf), "Logged in as %s: %s",
             is_admin ? "Admin" : "Guest",
             contact_name && contact_name[0] ? contact_name : "?");
    lv_label_set_text(lbl_admin_banner, buf);
    // Yellow background reads poorly with white text — switch to black.
    lv_obj_set_style_text_color(lbl_admin_banner,
        is_admin ? lv_color_white() : lv_color_black(), 0);
}

// Render the most-recent RepeaterStats into lbl_admin_status_body.
// Called once on status response arrival and again every tab entry
// (so the "Ns ago" footer line stays roughly current).
static void admin_render_status_body() {
    if (!lbl_admin_status_body) return;

    if (g_admin_status_last_update == 0) {
        lv_label_set_text(lbl_admin_status_body,
                          "No status data yet.\n\n"
                          "Tap the Refresh button to request the\n"
                          "repeater's current statistics.");
        return;
    }

    const RepeaterStats& s = g_admin_last_status;

    // Format helpers — keep these local to avoid file-scope clutter.
    auto fmt_uptime = [](uint32_t secs, char* out, size_t out_sz) {
        uint32_t days  = secs / 86400U;
        uint32_t hours = (secs % 86400U) / 3600U;
        uint32_t mins  = (secs % 3600U) / 60U;
        uint32_t s2    = secs % 60U;
        if (days > 0) {
            snprintf(out, out_sz, "%ud %uh %um", (unsigned)days,
                     (unsigned)hours, (unsigned)mins);
        } else if (hours > 0) {
            snprintf(out, out_sz, "%uh %um %us", (unsigned)hours,
                     (unsigned)mins, (unsigned)s2);
        } else {
            snprintf(out, out_sz, "%um %us", (unsigned)mins, (unsigned)s2);
        }
    };

    auto fmt_airtime = [](uint32_t secs, char* out, size_t out_sz) {
        // Airtime is usually small; show in m:ss form past 60s, plain
        // seconds otherwise.
        if (secs < 60) {
            snprintf(out, out_sz, "%u s", (unsigned)secs);
        } else {
            uint32_t mins = secs / 60U;
            uint32_t s2   = secs % 60U;
            snprintf(out, out_sz, "%um %us", (unsigned)mins, (unsigned)s2);
        }
    };

    auto fmt_clock = [](uint32_t epoch, char* out, size_t out_sz) {
        time_t t = (time_t)epoch;
        struct tm tm_buf;
        gmtime_r(&t, &tm_buf);
        strftime(out, out_sz, "%d %b %Y %H:%M UTC", &tm_buf);
    };

    char uptime_buf[32];
    fmt_uptime(s.total_up_time_secs, uptime_buf, sizeof(uptime_buf));

    char tx_air_buf[32], rx_air_buf[32];
    fmt_airtime(s.total_air_time_secs,    tx_air_buf, sizeof(tx_air_buf));
    fmt_airtime(s.total_rx_air_time_secs, rx_air_buf, sizeof(rx_air_buf));

    char clock_buf[48] = "?";
    if (g_admin_status_clock_tag != 0) {
        fmt_clock(g_admin_status_clock_tag, clock_buf, sizeof(clock_buf));
    } else if (g_admin_session_clock_tag != 0) {
        fmt_clock(g_admin_session_clock_tag, clock_buf, sizeof(clock_buf));
    }

    float battery_v = s.batt_milli_volts / 1000.0f;
    float snr_db    = s.last_snr / 4.0f;

    char body[1024];
    snprintf(body, sizeof(body),
             "Battery: %.2f V (%u mV)\n"
             "Clock at last reply: %s\n"
             "Uptime: %s\n"
             "TX airtime: %s\n"
             "RX airtime: %s\n"
             "Last RSSI: %d dBm\n"
             "Last SNR: %.2f dB\n"
             "Noise floor: %d dBm\n"
             "\n"
             "Packets sent: %u total (%u flood, %u direct)\n"
             "Packets received: %u total (%u flood, %u direct)\n"
             "Duplicates seen: %u direct, %u flood\n"
             "Receive errors: %u\n"
             "TX queue length: %u\n"
             "Debug error events: %u",
             (double)battery_v, (unsigned)s.batt_milli_volts,
             clock_buf,
             uptime_buf,
             tx_air_buf,
             rx_air_buf,
             (int)s.last_rssi,
             (double)snr_db,
             (int)s.noise_floor,
             (unsigned)s.n_packets_sent,
             (unsigned)s.n_sent_flood, (unsigned)s.n_sent_direct,
             (unsigned)s.n_packets_recv,
             (unsigned)s.n_recv_flood, (unsigned)s.n_recv_direct,
             (unsigned)s.n_direct_dups, (unsigned)s.n_flood_dups,
             (unsigned)s.n_recv_errors,
             (unsigned)s.curr_tx_queue_len,
             (unsigned)s.err_events);

    lv_label_set_text(lbl_admin_status_body, body);
}

static void on_admin_status_refresh_tap(lv_event_t *e) {
    if (g_admin_login_contact_idx < 0) return;
    if (g_admin_status_in_flight) return;  // debounce

    printf("Admin: status refresh requested for contact[%d]\n",
           g_admin_login_contact_idx);

    g_admin_status_in_flight = true;
    if (lbl_admin_status_updated) {
        lv_label_set_text(lbl_admin_status_updated, "Refreshing...");
        lv_obj_set_style_text_color(lbl_admin_status_updated,
                                    lv_palette_main(LV_PALETTE_YELLOW), 0);
    }
    if (btn_admin_status_refresh) {
        lv_obj_add_state(btn_admin_status_refresh, LV_STATE_DISABLED);
    }
    meck_request_admin_status(g_admin_login_contact_idx);
}

static void on_admin_send_advert_tap(lv_event_t *e) {
    if (g_admin_login_contact_idx < 0) return;
    if (g_admin_send_advert_in_flight) return;  // debounce

    printf("Admin: Send Advert requested for contact[%d]\n",
           g_admin_login_contact_idx);

    g_admin_send_advert_in_flight = true;
    if (lbl_admin_send_advert_status) {
        lv_label_set_text(lbl_admin_send_advert_status,
                          "Sending advert command...");
        lv_obj_set_style_text_color(lbl_admin_send_advert_status,
                                    lv_palette_main(LV_PALETTE_YELLOW), 0);
    }
    if (lbl_admin_send_advert_response) {
        lv_label_set_text(lbl_admin_send_advert_response, "");
    }
    if (btn_admin_send_advert) {
        lv_obj_add_state(btn_admin_send_advert, LV_STATE_DISABLED);
    }
    meck_request_admin_cli(g_admin_login_contact_idx, "advert");
}

// Bridge callbacks (fired from meck_drain_pending_admin_responses on
// the LVGL task). Safe to touch LVGL state here.

static void meck_admin_send_result_dispatch(meck_admin_req_type_t type,
                                              bool success,
                                              uint32_t est_timeout_ms) {
    printf("Admin: send result type=%d success=%d est_timeout=%u\n",
           (int)type, success ? 1 : 0, (unsigned)est_timeout_ms);

    if (type == MECK_ADMIN_REQ_LOGIN) {
        if (!success) {
            // Send itself failed (contact unreachable / MSG_SEND_FAILED).
            // Drop back to PASSWORD_ENTRY so the user can retry.
            g_admin_ui_state = ADMIN_STATE_PASSWORD_ENTRY;
            admin_login_show_status("Send failed. Check the path and try again.",
                                    lv_palette_main(LV_PALETTE_RED));
            if (btn_admin_login_submit) {
                lv_obj_clear_state(btn_admin_login_submit, LV_STATE_DISABLED);
            }
        } else {
            // Send dispatched. Update the timeout from the mesh's
            // estimate; the login response should arrive within this
            // window. Pad +5s for the SD-bound rendering overhead on
            // our side (matches upstream behaviour).
            g_admin_login_timeout_ms = est_timeout_ms + 5000;
            char buf[64];
            snprintf(buf, sizeof(buf), "Logging in... (timeout %us)",
                     (unsigned)(g_admin_login_timeout_ms / 1000));
            admin_login_show_status(buf, lv_palette_main(LV_PALETTE_YELLOW));
        }
        return;
    }

    if (type == MECK_ADMIN_REQ_STATUS) {
        if (!success) {
            // Send itself failed. Clear the in-flight flag and
            // surface the error so the user can retry.
            g_admin_status_in_flight = false;
            if (lbl_admin_status_updated) {
                lv_label_set_text(lbl_admin_status_updated,
                                  "Send failed. Tap Refresh to retry.");
                lv_obj_set_style_text_color(lbl_admin_status_updated,
                                            lv_palette_main(LV_PALETTE_RED), 0);
            }
            if (btn_admin_status_refresh) {
                lv_obj_clear_state(btn_admin_status_refresh, LV_STATE_DISABLED);
            }
        }
        // success: leave g_admin_status_in_flight=true and the
        // yellow "Refreshing..." label until the response arrives
        // (or the user navigates away).
        return;
    }

    if (type == MECK_ADMIN_REQ_CLI) {
        if (!success && g_admin_send_advert_in_flight) {
            g_admin_send_advert_in_flight = false;
            if (lbl_admin_send_advert_status) {
                lv_label_set_text(lbl_admin_send_advert_status,
                                  "Send failed. Tap the button to retry.");
                lv_obj_set_style_text_color(lbl_admin_send_advert_status,
                                            lv_palette_main(LV_PALETTE_RED), 0);
            }
            if (btn_admin_send_advert) {
                lv_obj_clear_state(btn_admin_send_advert, LV_STATE_DISABLED);
            }
        }
        if (!success && g_admin_cmd_in_flight) {
            // Replace the "(waiting...)" placeholder we appended at
            // send time with a red error response. Same delete +
            // re-add pattern as the success path in
            // meck_admin_cli_dispatch.
            g_admin_cmd_in_flight = false;
            char last_cmd[ADMIN_CMD_MAX_LEN + 8] = "";
            if (obj_admin_cmd_scroll) {
                uint32_t n = lv_obj_get_child_count(obj_admin_cmd_scroll);
                if (n > 0) {
                    lv_obj_t* last_row = lv_obj_get_child(obj_admin_cmd_scroll, n - 1);
                    if (last_row && lv_obj_get_child_count(last_row) >= 1) {
                        lv_obj_t* cmd_lbl = lv_obj_get_child(last_row, 0);
                        const char* t = lv_label_get_text(cmd_lbl);
                        if (t) {
                            const char* src = (t[0] == '>' && t[1] == ' ') ? t + 2 : t;
                            strncpy(last_cmd, src, sizeof(last_cmd) - 1);
                        }
                    }
                    lv_obj_delete(last_row);
                }
            }
            admin_cmd_add_entry(last_cmd,
                "Send failed. Check the path and try again.", true);
            if (btn_admin_cmd_send) {
                lv_obj_clear_state(btn_admin_cmd_send, LV_STATE_DISABLED);
            }
        }
        if (!success && g_admin_fw_in_flight) {
            g_admin_fw_in_flight = false;
            if (lbl_admin_fw_info_status) {
                lv_label_set_text(lbl_admin_fw_info_status,
                                  "Send failed. Tap Refresh to retry.");
                lv_obj_set_style_text_color(lbl_admin_fw_info_status,
                                            lv_palette_main(LV_PALETTE_RED), 0);
            }
            if (btn_admin_fw_info_refresh) {
                lv_obj_clear_state(btn_admin_fw_info_refresh, LV_STATE_DISABLED);
            }
        }
        return;
    }

    if (type == MECK_ADMIN_REQ_NEIGHBOURS) {
        if (!success) {
            g_admin_neighbours_in_flight = false;
            if (lbl_admin_neighbours_status) {
                lv_label_set_text(lbl_admin_neighbours_status,
                                  "Send failed. Tap Refresh to retry.");
                lv_obj_set_style_text_color(lbl_admin_neighbours_status,
                                            lv_palette_main(LV_PALETTE_RED), 0);
            }
            if (btn_admin_neighbours_refresh) {
                lv_obj_clear_state(btn_admin_neighbours_refresh, LV_STATE_DISABLED);
            }
        }
        // success: leave g_admin_neighbours_in_flight=true and the
        // yellow status text until the response arrives. If this was
        // a mid-pagination send (not the first page), the
        // accumulator state is preserved and the next response will
        // continue appending.
        return;
    }

    // Telemetry send-result will be handled when piece 5 wires up the
    // telemetry view.
}

static void meck_admin_login_dispatch(bool success, uint8_t is_admin,
                                       uint8_t permissions,
                                       uint8_t fw_ver_level,
                                       uint32_t clock_tag,
                                       int contact_idx) {
    printf("Admin: login dispatch success=%d is_admin=%u perms=0x%02X "
           "fw_ver=%u clock=%u contact=%d\n",
           success ? 1 : 0, (unsigned)is_admin, (unsigned)permissions,
           (unsigned)fw_ver_level, (unsigned)clock_tag, contact_idx);

    if (!success) {
        g_admin_ui_state = ADMIN_STATE_PASSWORD_ENTRY;
        admin_login_show_status("Login failed. Check the password.",
                                lv_palette_main(LV_PALETTE_RED));
        if (btn_admin_login_submit) {
            lv_obj_clear_state(btn_admin_login_submit, LV_STATE_DISABLED);
        }
        return;
    }

    // Success path. Cache the password ONLY if the user opted in via
    // the Remember Password checkbox AND the password was non-empty
    // (caching a blank doesn't help anyone).
    bool remember = false;
    if (cb_admin_remember) {
        remember = lv_obj_has_state(cb_admin_remember, LV_STATE_CHECKED);
    }
    if (remember && g_admin_pwd_len > 0) {
        admin_pwd_cache_put(contact_idx, g_admin_pwd_raw);
        printf("Admin: cached password for contact[%d]\n", contact_idx);
    }

    g_admin_session_is_admin    = (is_admin != 0);
    g_admin_session_permissions = permissions;
    g_admin_session_fw_ver      = fw_ver_level;
    g_admin_session_clock_tag   = clock_tag;
    g_admin_ui_state            = ADMIN_STATE_LOGGED_IN;

    // Wipe raw password buffer post-success. The cache (if enabled)
    // holds a copy; the screen no longer needs it.
    memset(g_admin_pwd_raw, 0, sizeof(g_admin_pwd_raw));
    g_admin_pwd_len = 0;
    if (ta_admin_password) {
        g_admin_pwd_render_in_progress = true;
        lv_textarea_set_text(ta_admin_password, "");
        g_admin_pwd_render_in_progress = false;
    }

    // Reset cached status state so a fresh login starts with the
    // "Tap Refresh" placeholder rather than stale numbers from a
    // previous session on a different repeater.
    g_admin_status_last_update = 0;
    g_admin_status_clock_tag   = 0;
    g_admin_status_in_flight   = false;
    memset(&g_admin_last_status, 0, sizeof(g_admin_last_status));
    if (btn_admin_status_refresh) {
        lv_obj_clear_state(btn_admin_status_refresh, LV_STATE_DISABLED);
    }
    if (lbl_admin_status_updated) {
        lv_label_set_text(lbl_admin_status_updated, "Tap Refresh to load");
        lv_obj_set_style_text_color(lbl_admin_status_updated,
                                    lv_color_make(180, 180, 180), 0);
    }
    admin_render_status_body();

    // Populate the banner + load the admin home menu list.
    Meck* mesh = meck_get_instance();
    ContactInfo ci = {};
    const char* contact_name = "?";
    if (mesh && mesh->getContactByIdx((uint32_t)contact_idx, ci)) {
        contact_name = ci.name;
    }
    admin_home_update_banner(is_admin != 0, contact_name);

    // Also reset the Send Advert flow so a fresh login starts clean
    // if the user goes straight there from a previous session.
    g_admin_send_advert_in_flight = false;
    if (lbl_admin_send_advert_status) {
        lv_label_set_text(lbl_admin_send_advert_status,
                          "Tap the button to send an advert command.");
        lv_obj_set_style_text_color(lbl_admin_send_advert_status,
                                    lv_color_make(180, 180, 180), 0);
    }
    if (lbl_admin_send_advert_response) {
        lv_label_set_text(lbl_admin_send_advert_response, "");
    }
    if (btn_admin_send_advert) {
        lv_obj_clear_state(btn_admin_send_advert, LV_STATE_DISABLED);
    }

    // Reset Cmd Line scrollback + input on fresh login. A different
    // repeater is a different context; old responses shouldn't carry
    // over.
    g_admin_cmd_in_flight = false;
    if (obj_admin_cmd_scroll) {
        lv_obj_clean(obj_admin_cmd_scroll);
    }
    if (ta_admin_cmd_input) {
        lv_textarea_set_text(ta_admin_cmd_input, "");
    }
    if (btn_admin_cmd_send) {
        lv_obj_clear_state(btn_admin_cmd_send, LV_STATE_DISABLED);
    }

    // Reset Firmware Info subpage on fresh login. Clear in-flight flag
    // and blank out the cached values so a re-entry triggers a fresh
    // "ver" request rather than displaying the previous repeater's
    // values.
    g_admin_fw_in_flight = false;
    if (lbl_admin_fw_info_status) {
        lv_label_set_text(lbl_admin_fw_info_status, "");
    }
    if (lbl_admin_fw_info_version) {
        lv_label_set_text(lbl_admin_fw_info_version, "");
    }
    if (lbl_admin_fw_info_build) {
        lv_label_set_text(lbl_admin_fw_info_build, "");
    }
    if (btn_admin_fw_info_refresh) {
        lv_obj_clear_state(btn_admin_fw_info_refresh, LV_STATE_DISABLED);
    }

    // Reset Neighbours subpage on fresh login. Clear pagination state
    // and the list contents so a re-entry triggers a fresh request
    // rather than displaying the previous repeater's neighbours.
    g_admin_neighbours_in_flight   = false;
    g_admin_neighbours_total       = 0;
    g_admin_neighbours_received    = 0;
    g_admin_neighbours_next_offset = 0;
    if (lbl_admin_neighbours_status) {
        lv_label_set_text(lbl_admin_neighbours_status, "");
    }
    if (obj_admin_neighbours_scroll) {
        lv_obj_clean(obj_admin_neighbours_scroll);
    }
    if (btn_admin_neighbours_refresh) {
        lv_obj_clear_state(btn_admin_neighbours_refresh, LV_STATE_DISABLED);
    }

    // Post-login routing: room contacts land on scr_room_messages (post
    // history view, with optional top-right Admin overlay when
    // g_admin_session_is_admin is true). Repeater contacts keep the
    // existing path into scr_admin_home.
    if (ci.type == 3) {
        room_messages_screen_enter(contact_idx, contact_name);
    } else {
        if (scr_admin_home) lv_screen_load(scr_admin_home);
    }
}

// Status response dispatch — fired by the bridge when a
// REQ_TYPE_GET_STATUS reply lands. Updates the cached stats struct,
// re-enables the refresh button, and re-renders the Status body.
//
// We don't gate on contact_idx == g_admin_login_contact_idx here —
// if the user backed out and re-entered, the stats are still useful
// to display until they overwrite. Stale-session handling is
// elsewhere (the back button clears the cache).
static void meck_admin_status_dispatch(const RepeaterStats* stats,
                                        uint32_t clock_tag,
                                        int contact_idx) {
    if (!stats) return;
    printf("Admin: status dispatch contact=%d clock_tag=%u\n",
           contact_idx, (unsigned)clock_tag);

    memcpy(&g_admin_last_status, stats, sizeof(RepeaterStats));
    g_admin_status_clock_tag   = clock_tag;
    g_admin_status_last_update = (unsigned long)esp_log_timestamp();
    g_admin_status_in_flight   = false;

    if (lbl_admin_status_updated) {
        lv_label_set_text(lbl_admin_status_updated, "Updated just now");
        lv_obj_set_style_text_color(lbl_admin_status_updated,
                                    lv_palette_main(LV_PALETTE_GREEN), 0);
    }
    if (btn_admin_status_refresh) {
        lv_obj_clear_state(btn_admin_status_refresh, LV_STATE_DISABLED);
    }
    admin_render_status_body();
}

// CLI response dispatch. Currently consumed by the Send Advert
// subpage (the only thing in piece 4 that issues a CLI command).
// Piece 6 will share this same dispatcher with the Cmd Line subpage.
static void meck_admin_cli_dispatch(const char* response, int contact_idx) {
    printf("Admin: CLI dispatch contact=%d response=\"%.80s%s\"\n",
           contact_idx,
           response ? response : "",
           (response && strlen(response) > 80) ? "..." : "");

    if (g_admin_send_advert_in_flight) {
        // Send Advert flow — populate the result on the dedicated
        // subpage. The screen may not be visible (user could have
        // backed out before the reply arrived), but updating the
        // label is harmless either way.
        g_admin_send_advert_in_flight = false;
        if (lbl_admin_send_advert_status) {
            lv_label_set_text(lbl_admin_send_advert_status,
                              "Advert command sent.");
            lv_obj_set_style_text_color(lbl_admin_send_advert_status,
                                        lv_palette_main(LV_PALETTE_GREEN), 0);
        }
        if (lbl_admin_send_advert_response && response) {
            lv_label_set_text(lbl_admin_send_advert_response, response);
        }
        if (btn_admin_send_advert) {
            lv_obj_clear_state(btn_admin_send_advert, LV_STATE_DISABLED);
        }
        return;
    }

    if (g_admin_cmd_in_flight) {
        // Cmd Line flow — replace the "(waiting...)" placeholder in
        // the most recent scrollback entry with the real response.
        // Simplest implementation: just delete the last entry and
        // re-add it with the response filled in. We need the command
        // text from somewhere — pull it from the (waiting...) entry
        // before deleting.
        g_admin_cmd_in_flight = false;

        char last_cmd[ADMIN_CMD_MAX_LEN + 8] = "";
        if (obj_admin_cmd_scroll) {
            uint32_t n = lv_obj_get_child_count(obj_admin_cmd_scroll);
            if (n > 0) {
                lv_obj_t* last_row = lv_obj_get_child(obj_admin_cmd_scroll, n - 1);
                if (last_row && lv_obj_get_child_count(last_row) >= 1) {
                    lv_obj_t* cmd_lbl = lv_obj_get_child(last_row, 0);
                    const char* t = lv_label_get_text(cmd_lbl);
                    if (t) {
                        // Strip the "> " prefix we added at entry time.
                        const char* src = (t[0] == '>' && t[1] == ' ') ? t + 2 : t;
                        strncpy(last_cmd, src, sizeof(last_cmd) - 1);
                    }
                }
                lv_obj_delete(last_row);
            }
        }
        admin_cmd_add_entry(last_cmd, response ? response : "(no reply)", false);
        if (btn_admin_cmd_send) {
            lv_obj_clear_state(btn_admin_cmd_send, LV_STATE_DISABLED);
        }
        return;
    }

    if (g_admin_fw_in_flight) {
        // Firmware Info flow — response format is from upstream
        // CommonCLI.cpp's `ver` handler:
        //   sprintf(reply, "%s (Build: %s)", firmwareVer, buildDate);
        // admin_fw_info_parse_and_render splits it into the two
        // labels.
        g_admin_fw_in_flight = false;
        admin_fw_info_parse_and_render(response ? response : "");
        if (btn_admin_fw_info_refresh) {
            lv_obj_clear_state(btn_admin_fw_info_refresh, LV_STATE_DISABLED);
        }
        return;
    }
}

// ---- Entry point from contact detail ----

static void admin_login_screen_enter(int contact_idx) {
    g_admin_login_contact_idx = contact_idx;
    g_admin_ui_state = ADMIN_STATE_PASSWORD_ENTRY;
    g_admin_pwd_len = 0;
    memset(g_admin_pwd_raw, 0, sizeof(g_admin_pwd_raw));
    g_admin_pwd_last_char_ms = 0;
    g_admin_pwd_reveal_all = false;

    Meck* mesh = meck_get_instance();
    ContactInfo ci = {};
    bool have_contact = mesh && mesh->getContactByIdx((uint32_t)contact_idx, ci);

    // Subtitle: contact name. Title stays "Repeater Login" for both
    // repeaters and room servers — the screen handles both the same
    // way and "Login" is the user-facing concept.
    if (lbl_admin_login_subtitle) {
        lv_label_set_text(lbl_admin_login_subtitle,
                          have_contact ? ci.name : "(unknown contact)");
    }

    // Routing-mode badge for the Login button. Based on the contact's
    // CURRENT out_path_len at screen entry time. Login itself always
    // forces flood inside Meck::uiLoginToRepeater, but the badge
    // reflects the path state for subsequent admin commands.
    g_admin_login_routing_direct =
        have_contact && (ci.out_path_len != OUT_PATH_UNKNOWN);
    if (lbl_admin_login_submit) {
        char btn_text[32];
        snprintf(btn_text, sizeof(btn_text), "Log In  ·  %s",
                 g_admin_login_routing_direct ? "Direct" : "Flood");
        lv_label_set_text(lbl_admin_login_submit, btn_text);
    }
    if (btn_admin_login_submit) {
        lv_obj_clear_state(btn_admin_login_submit, LV_STATE_DISABLED);
    }

    // Pre-fill cached password if we have one. Update the raw buffer
    // and let admin_render_password_masked render the masked view.
    const char* cached = admin_pwd_cache_get(contact_idx);
    if (cached && cached[0]) {
        size_t n = strnlen(cached, MECK_ADMIN_PASSWORD_MAX);
        memcpy(g_admin_pwd_raw, cached, n);
        g_admin_pwd_raw[n] = '\0';
        g_admin_pwd_len = (int)n;
        // Pre-filled chars don't trigger reveal — render dots only.
        g_admin_pwd_last_char_ms = 0;
        // Check the Remember Password checkbox since we obviously
        // remembered last time.
        if (cb_admin_remember) {
            lv_obj_add_state(cb_admin_remember, LV_STATE_CHECKED);
        }
    } else {
        if (cb_admin_remember) {
            lv_obj_add_state(cb_admin_remember, LV_STATE_CHECKED);
        }
    }
    admin_render_password_masked();

    admin_login_show_status("", lv_color_white());

    if (scr_admin_login) lv_screen_load(scr_admin_login);
}

// ============================================================================
// Ping UI — zero-hop trace to a repeater
// ============================================================================

// Dismiss the result overlay (success or failure).
static void ping_dismiss_result(lv_event_t *e) {
    (void)e;
    if (obj_ping_result_panel) {
        lv_obj_del(obj_ping_result_panel);
        obj_ping_result_panel = NULL;
        lbl_ping_result = NULL;
    }
}

// Show a result overlay (green for success, red for failure).
static void ping_show_result(bool success, const char *body) {
    if (obj_ping_result_panel) {
        lv_obj_del(obj_ping_result_panel);
        obj_ping_result_panel = NULL;
    }
    if (!scr_contact_detail) return;

    obj_ping_result_panel = lv_obj_create(scr_contact_detail);
    lv_obj_set_size(obj_ping_result_panel, SCREEN_WIDTH - 60, LV_SIZE_CONTENT);
    lv_obj_align(obj_ping_result_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(obj_ping_result_panel,
        success ? lv_color_make(50, 180, 80) : lv_color_make(220, 80, 70), 0);
    lv_obj_set_style_bg_opa(obj_ping_result_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(obj_ping_result_panel, 16, 0);
    lv_obj_set_style_pad_all(obj_ping_result_panel, 20, 0);
    lv_obj_set_style_border_width(obj_ping_result_panel, 0, 0);
    lv_obj_clear_flag(obj_ping_result_panel, LV_OBJ_FLAG_SCROLLABLE);

    // Icon
    lv_obj_t *icon = lv_label_create(obj_ping_result_panel);
    if (success) {
        lv_label_set_text(icon, LV_SYMBOL_OK);
    } else {
        lv_label_set_text(icon, LV_SYMBOL_WARNING);
    }
    lv_obj_set_style_text_color(icon, lv_color_white(), 0);
    meck_set_font(icon, &meck_montserrat_28, 0);
    lv_obj_align(icon, LV_ALIGN_TOP_LEFT, 0, 0);

    // Title
    lv_obj_t *title = lv_label_create(obj_ping_result_panel);
    lv_label_set_text(title, success ? "Ping Success" : "Ping Failed");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    meck_set_font(title, &meck_montserrat_22, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 40, 0);

    // Body
    lbl_ping_result = lv_label_create(obj_ping_result_panel);
    lv_label_set_text(lbl_ping_result, body);
    lv_obj_set_style_text_color(lbl_ping_result, lv_color_white(), 0);
    meck_set_font(lbl_ping_result, &meck_montserrat_18, 0);
    lv_obj_set_width(lbl_ping_result, SCREEN_WIDTH - 120);
    lv_obj_align(lbl_ping_result, LV_ALIGN_TOP_LEFT, 10, 40);

    // Tap anywhere on the panel to dismiss
    lv_obj_add_event_cb(obj_ping_result_panel, ping_dismiss_result,
                        LV_EVENT_CLICKED, NULL);
}

// Dismiss the prefix warning overlay without sending.
static void ping_dismiss_warning(lv_event_t *e) {
    (void)e;
    if (obj_ping_warning_panel) {
        lv_obj_del(obj_ping_warning_panel);
        obj_ping_warning_panel = NULL;
    }
}

// Confirm the prefix warning and send the ping.
static void ping_confirm_warning(lv_event_t *e) {
    (void)e;
    if (obj_ping_warning_panel) {
        lv_obj_del(obj_ping_warning_panel);
        obj_ping_warning_panel = NULL;
    }
    ping_send_now(g_ping_byte_size);
}

// Show a prefix collision warning dialog.
static void ping_show_warning(int dup_count, uint8_t prefix_byte) {
    if (obj_ping_warning_panel) {
        lv_obj_del(obj_ping_warning_panel);
        obj_ping_warning_panel = NULL;
    }
    if (!scr_contact_detail) return;

    obj_ping_warning_panel = lv_obj_create(scr_contact_detail);
    lv_obj_set_size(obj_ping_warning_panel, SCREEN_WIDTH - 40, LV_SIZE_CONTENT);
    lv_obj_align(obj_ping_warning_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(obj_ping_warning_panel,
        lv_color_make(50, 50, 55), 0);
    lv_obj_set_style_bg_opa(obj_ping_warning_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(obj_ping_warning_panel, 16, 0);
    lv_obj_set_style_pad_all(obj_ping_warning_panel, 20, 0);
    lv_obj_set_style_border_width(obj_ping_warning_panel, 0, 0);
    lv_obj_clear_flag(obj_ping_warning_panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(obj_ping_warning_panel);
    lv_label_set_text(title, "Ping Warning");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    meck_set_font(title, &meck_montserrat_22, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 0);

    char msg[160];
    snprintf(msg, sizeof(msg),
             "You know %d repeaters or room servers with prefix <%02X>. "
             "Unfortunately we won't know which of them replies to this "
             "ping request.",
             dup_count, (unsigned)prefix_byte);
    lv_obj_t *body = lv_label_create(obj_ping_warning_panel);
    lv_label_set_text(body, msg);
    lv_obj_set_style_text_color(body, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(body, &meck_montserrat_16, 0);
    lv_obj_set_width(body, SCREEN_WIDTH - 100);
    lv_obj_align(body, LV_ALIGN_TOP_LEFT, 0, 35);

    // Dismiss button
    lv_obj_t *btn_dismiss = lv_button_create(obj_ping_warning_panel);
    lv_obj_set_size(btn_dismiss, 140, 50);
    lv_obj_align(btn_dismiss, LV_ALIGN_BOTTOM_LEFT, 0, 10);
    lv_obj_set_style_bg_color(btn_dismiss, lv_color_make(80, 80, 85), 0);
    lv_obj_set_style_radius(btn_dismiss, 12, 0);
    lv_obj_t *dl = lv_label_create(btn_dismiss);
    lv_label_set_text(dl, "Dismiss");
    lv_obj_set_style_text_color(dl, lv_color_white(), 0);
    meck_set_font(dl, &meck_montserrat_16, 0);
    lv_obj_center(dl);
    lv_obj_add_event_cb(btn_dismiss, ping_dismiss_warning,
                        LV_EVENT_CLICKED, NULL);

    // Confirm button
    lv_obj_t *btn_confirm = lv_button_create(obj_ping_warning_panel);
    lv_obj_set_size(btn_confirm, 140, 50);
    lv_obj_align(btn_confirm, LV_ALIGN_BOTTOM_RIGHT, 0, 10);
    lv_obj_set_style_bg_color(btn_confirm, lv_color_make(80, 80, 85), 0);
    lv_obj_set_style_radius(btn_confirm, 12, 0);
    lv_obj_t *cl = lv_label_create(btn_confirm);
    lv_label_set_text(cl, "Confirm");
    lv_obj_set_style_text_color(cl, lv_palette_main(LV_PALETTE_RED), 0);
    meck_set_font(cl, &meck_montserrat_16, 0);
    lv_obj_center(cl);
    lv_obj_add_event_cb(btn_confirm, ping_confirm_warning,
                        LV_EVENT_CLICKED, NULL);
}

// Actually send the ping trace.
static void ping_send_now(uint8_t byte_size) {
    Meck *mesh = meck_get_instance();
    if (!mesh || g_selected_contact_idx < 0) return;

    if (!mesh->uiSendPing(g_selected_contact_idx, byte_size)) {
        ping_show_result(false, "Send failed");
        return;
    }

    g_ping_in_flight = true;
    g_ping_byte_size = byte_size;
    g_ping_timeout_ms = (unsigned long)millis() +
                        (MECK_PING_TIMEOUT_SEC * 1000);
}

// Tap handler: 1-byte ping with duplicate prefix check.
static void on_contact_ping_tap(lv_event_t *e) {
    (void)e;
    Meck *mesh = meck_get_instance();
    if (!mesh || g_selected_contact_idx < 0) return;
    if (g_ping_in_flight) return;  // one at a time

    // Dismiss any lingering result
    if (obj_ping_result_panel) ping_dismiss_result(NULL);

    g_ping_byte_size = 1;

    // Check for duplicate 1-byte prefixes
    int dup_count = mesh->countMatchingPrefixes(g_selected_contact_idx, 1);
    if (dup_count > 1) {
        ContactInfo ci;
        uint8_t prefix = 0;
        if (mesh->getContactByIdx((uint32_t)g_selected_contact_idx, ci)) {
            prefix = ci.id.pub_key[0];
        }
        ping_show_warning(dup_count, prefix);
        return;
    }

    ping_send_now(1);
}

// Long-press handler: show 1/2 byte chooser.
static void on_contact_ping_longpress(lv_event_t *e) {
    (void)e;
    Meck *mesh = meck_get_instance();
    if (!mesh || g_selected_contact_idx < 0) return;
    if (g_ping_in_flight) return;

    // Dismiss any lingering overlays
    if (obj_ping_result_panel) ping_dismiss_result(NULL);
    if (obj_ping_warning_panel) ping_dismiss_warning(NULL);

    if (obj_ping_chooser_panel) {
        lv_obj_del(obj_ping_chooser_panel);
        obj_ping_chooser_panel = NULL;
    }
    if (!scr_contact_detail) return;

    // Get contact prefix info for display
    ContactInfo ci;
    char prefix_1[8] = "??";
    char prefix_2[12] = "????";
    if (mesh->getContactByIdx((uint32_t)g_selected_contact_idx, ci)) {
        snprintf(prefix_1, sizeof(prefix_1), "%02X",
                 (unsigned)ci.id.pub_key[0]);
        snprintf(prefix_2, sizeof(prefix_2), "%02X%02X",
                 (unsigned)ci.id.pub_key[0],
                 (unsigned)ci.id.pub_key[1]);
    }

    obj_ping_chooser_panel = lv_obj_create(scr_contact_detail);
    lv_obj_set_size(obj_ping_chooser_panel, SCREEN_WIDTH - 60, LV_SIZE_CONTENT);
    lv_obj_align(obj_ping_chooser_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(obj_ping_chooser_panel,
        lv_color_make(50, 50, 55), 0);
    lv_obj_set_style_bg_opa(obj_ping_chooser_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(obj_ping_chooser_panel, 16, 0);
    lv_obj_set_style_pad_all(obj_ping_chooser_panel, 20, 0);
    lv_obj_set_style_border_width(obj_ping_chooser_panel, 0, 0);
    lv_obj_clear_flag(obj_ping_chooser_panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(obj_ping_chooser_panel);
    lv_label_set_text(title, "Ping (Zero Hop)");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    meck_set_font(title, &meck_montserrat_22, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 0);

    // 1-byte option
    char lbl_1[48];
    snprintf(lbl_1, sizeof(lbl_1), "Ping <%s>\n1-byte", prefix_1);
    lv_obj_t *btn1 = lv_button_create(obj_ping_chooser_panel);
    lv_obj_set_size(btn1, SCREEN_WIDTH - 120, 70);
    lv_obj_align(btn1, LV_ALIGN_TOP_LEFT, 0, 45);
    lv_obj_set_style_bg_color(btn1, lv_color_make(60, 60, 65), 0);
    lv_obj_set_style_radius(btn1, 8, 0);
    lv_obj_t *l1 = lv_label_create(btn1);
    lv_label_set_text(l1, lbl_1);
    lv_obj_set_style_text_color(l1, lv_color_white(), 0);
    meck_set_font(l1, &meck_montserrat_18, 0);
    lv_obj_align(l1, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_t *arrow1 = lv_label_create(btn1);
    lv_label_set_text(arrow1, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(arrow1, lv_color_white(), 0);
    meck_set_font(arrow1, &meck_montserrat_18, 0);
    lv_obj_align(arrow1, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_add_event_cb(btn1, ping_chooser_select, LV_EVENT_CLICKED,
                        (void*)(uintptr_t)1);

    // 2-byte option
    char lbl_2[48];
    snprintf(lbl_2, sizeof(lbl_2), "Ping <%s>\n2-byte", prefix_2);
    lv_obj_t *btn2 = lv_button_create(obj_ping_chooser_panel);
    lv_obj_set_size(btn2, SCREEN_WIDTH - 120, 70);
    lv_obj_align(btn2, LV_ALIGN_TOP_LEFT, 0, 125);
    lv_obj_set_style_bg_color(btn2, lv_color_make(60, 60, 65), 0);
    lv_obj_set_style_radius(btn2, 8, 0);
    lv_obj_t *l2 = lv_label_create(btn2);
    lv_label_set_text(l2, lbl_2);
    lv_obj_set_style_text_color(l2, lv_color_white(), 0);
    meck_set_font(l2, &meck_montserrat_18, 0);
    lv_obj_align(l2, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_t *arrow2 = lv_label_create(btn2);
    lv_label_set_text(arrow2, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(arrow2, lv_color_white(), 0);
    meck_set_font(arrow2, &meck_montserrat_18, 0);
    lv_obj_align(arrow2, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_add_event_cb(btn2, ping_chooser_select, LV_EVENT_CLICKED,
                        (void*)(uintptr_t)2);

    // Cancel button
    lv_obj_t *btn_cancel = lv_button_create(obj_ping_chooser_panel);
    lv_obj_set_size(btn_cancel, SCREEN_WIDTH - 120, 45);
    lv_obj_align(btn_cancel, LV_ALIGN_TOP_LEFT, 0, 210);
    lv_obj_set_style_bg_opa(btn_cancel, 0, 0);
    lv_obj_t *lc = lv_label_create(btn_cancel);
    lv_label_set_text(lc, "Cancel");
    lv_obj_set_style_text_color(lc, lv_palette_main(LV_PALETTE_BLUE), 0);
    meck_set_font(lc, &meck_montserrat_18, 0);
    lv_obj_align(lc, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_add_event_cb(btn_cancel, [](lv_event_t *ev) {
        (void)ev;
        if (obj_ping_chooser_panel) {
            lv_obj_del(obj_ping_chooser_panel);
            obj_ping_chooser_panel = NULL;
        }
    }, LV_EVENT_CLICKED, NULL);
}

// Chooser selection handler — user picked 1-byte or 2-byte.
static void ping_chooser_select(lv_event_t *e) {
    uint8_t byte_size = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    if (obj_ping_chooser_panel) {
        lv_obj_del(obj_ping_chooser_panel);
        obj_ping_chooser_panel = NULL;
    }

    Meck *mesh = meck_get_instance();
    if (!mesh || g_selected_contact_idx < 0) return;

    g_ping_byte_size = byte_size;

    // For 1-byte, check duplicate prefixes
    if (byte_size == 1) {
        int dup_count = mesh->countMatchingPrefixes(g_selected_contact_idx, 1);
        if (dup_count > 1) {
            ContactInfo ci;
            uint8_t prefix = 0;
            if (mesh->getContactByIdx((uint32_t)g_selected_contact_idx, ci)) {
                prefix = ci.id.pub_key[0];
            }
            ping_show_warning(dup_count, prefix);
            return;
        }
    }

    ping_send_now(byte_size);
}

// ============================================================================

static void on_contact_admin_tap(lv_event_t *e) {
    if (g_selected_contact_idx < 0) return;
    admin_login_screen_enter(g_selected_contact_idx);
}

// ---- Screen construction ----

static void create_admin_login_screen() {
    scr_admin_login = lv_obj_create(NULL);
    lock_screen_scroll(scr_admin_login);
    lv_obj_set_style_bg_color(scr_admin_login, lv_color_black(), 0);
    // Slot 9 — first free slot past the channel-picker / message screens.
    screen_attach_clock_battery(scr_admin_login, 9, &meck_montserrat_22, 30);

    // Back button (top left)
    lv_obj_t *btn_back = lv_button_create(scr_admin_login);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_admin_login_back, LV_EVENT_CLICKED, NULL);

    // Title — short "Login" rather than "Repeater Login" because the
    // TFT camera punch-hole sits along the top-centre of the panel and
    // obscures the second word at this x-position.
    lbl_admin_login_title = lv_label_create(scr_admin_login);
    lv_label_set_text(lbl_admin_login_title, "Login");
    lv_obj_set_style_text_color(lbl_admin_login_title,
                                lv_palette_main(LV_PALETTE_INDIGO), 0);
    meck_set_font(lbl_admin_login_title, &meck_montserrat_24, 0);
    lv_obj_align(lbl_admin_login_title, LV_ALIGN_TOP_LEFT, 120, 20);

    // Subtitle — contact name (filled by admin_login_screen_enter)
    lbl_admin_login_subtitle = lv_label_create(scr_admin_login);
    lv_label_set_text(lbl_admin_login_subtitle, "");
    lv_obj_set_style_text_color(lbl_admin_login_subtitle, lv_color_white(), 0);
    meck_set_font(lbl_admin_login_subtitle, &meck_montserrat_18, 0);
    lv_obj_align(lbl_admin_login_subtitle, LV_ALIGN_TOP_LEFT, 120, 55);

    // Lock icon — large symbol above the password field. Centered.
    lv_obj_t *lock_icon = lv_label_create(scr_admin_login);
    lv_label_set_text(lock_icon, LV_SYMBOL_LOCK);
    lv_obj_set_style_text_color(lock_icon,
                                lv_palette_main(LV_PALETTE_INDIGO), 0);
    meck_set_font(lock_icon, &meck_montserrat_24, 0);
    lv_obj_align(lock_icon, LV_ALIGN_TOP_MID, 0, 110);

    // "Authentication Required" heading
    lv_obj_t *lbl_auth = lv_label_create(scr_admin_login);
    lv_label_set_text(lbl_auth, "Authentication Required");
    lv_obj_set_style_text_color(lbl_auth, lv_color_white(), 0);
    meck_set_font(lbl_auth, &meck_montserrat_22, 0);
    lv_obj_align(lbl_auth, LV_ALIGN_TOP_MID, 0, 160);

    lv_obj_t *lbl_sub = lv_label_create(scr_admin_login);
    lv_label_set_text(lbl_sub, "Please log in to manage this repeater.");
    lv_obj_set_style_text_color(lbl_sub, lv_color_make(180, 180, 180), 0);
    meck_set_font(lbl_sub, &meck_montserrat_16, 0);
    lv_obj_align(lbl_sub, LV_ALIGN_TOP_MID, 0, 200);

    // Password textarea
    ta_admin_password = lv_textarea_create(scr_admin_login);
    lv_obj_set_size(ta_admin_password, SCREEN_WIDTH - 130, 70);
    lv_obj_align(ta_admin_password, LV_ALIGN_TOP_LEFT, 60, 250);
    lv_textarea_set_placeholder_text(ta_admin_password, "Password");
    lv_textarea_set_one_line(ta_admin_password, true);
    meck_set_font(ta_admin_password, &meck_montserrat_22, 0);
    lv_obj_add_event_cb(ta_admin_password, on_admin_password_textarea_change,
                        LV_EVENT_VALUE_CHANGED, NULL);

    // Eye toggle button (right of the textarea). The eye-open / eye-close
    // FontAwesome glyphs (U+F06E / U+F070) are in the Meck Montserrat
    // fonts as of v0.3.5.
    btn_admin_password_eye = lv_button_create(scr_admin_login);
    lv_obj_set_size(btn_admin_password_eye, 50, 70);
    lv_obj_align(btn_admin_password_eye, LV_ALIGN_TOP_RIGHT, -10, 250);
    lv_obj_set_style_bg_color(btn_admin_password_eye,
                              lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_admin_password_eye, 8, 0);
    lv_obj_t *eye_lbl = lv_label_create(btn_admin_password_eye);
    lv_label_set_text(eye_lbl, LV_SYMBOL_EYE_OPEN);
    lv_obj_set_style_text_color(eye_lbl, lv_color_white(), 0);
    meck_set_font(eye_lbl, &meck_montserrat_22, 0);
    lv_obj_center(eye_lbl);
    lv_obj_add_event_cb(btn_admin_password_eye, on_admin_password_eye_tap,
                        LV_EVENT_CLICKED, NULL);

    // Remember Password checkbox
    cb_admin_remember = lv_checkbox_create(scr_admin_login);
    lv_checkbox_set_text(cb_admin_remember, "Remember Password?");
    lv_obj_set_style_text_color(cb_admin_remember, lv_color_white(), 0);
    meck_set_font(cb_admin_remember, &meck_montserrat_18, 0);
    lv_obj_align(cb_admin_remember, LV_ALIGN_TOP_LEFT, 60, 335);
    lv_obj_add_state(cb_admin_remember, LV_STATE_CHECKED);

    // Login button (large, centered below)
    btn_admin_login_submit = lv_button_create(scr_admin_login);
    lv_obj_set_size(btn_admin_login_submit, SCREEN_WIDTH - 120, 70);
    lv_obj_align(btn_admin_login_submit, LV_ALIGN_TOP_LEFT, 60, 395);
    lv_obj_set_style_bg_color(btn_admin_login_submit,
                              lv_palette_darken(LV_PALETTE_INDIGO, 1), 0);
    lv_obj_set_style_radius(btn_admin_login_submit, 8, 0);
    lbl_admin_login_submit = lv_label_create(btn_admin_login_submit);
    lv_label_set_text(lbl_admin_login_submit, "Log In  ·  Flood");
    lv_obj_set_style_text_color(lbl_admin_login_submit, lv_color_white(), 0);
    meck_set_font(lbl_admin_login_submit, &meck_montserrat_22, 0);
    lv_obj_center(lbl_admin_login_submit);
    lv_obj_add_event_cb(btn_admin_login_submit, on_admin_login_submit,
                        LV_EVENT_CLICKED, NULL);

    // Status line (rendered in red on failure, yellow during login,
    // hidden when idle)
    lbl_admin_login_status = lv_label_create(scr_admin_login);
    lv_label_set_text(lbl_admin_login_status, "");
    lv_obj_set_style_text_color(lbl_admin_login_status, lv_color_white(), 0);
    meck_set_font(lbl_admin_login_status, &meck_montserrat_18, 0);
    lv_obj_align(lbl_admin_login_status, LV_ALIGN_TOP_MID, 0, 485);
    lv_label_set_long_mode(lbl_admin_login_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_admin_login_status, SCREEN_WIDTH - 60);

    // Footer hint
    lv_obj_t *footer = lv_label_create(scr_admin_login);
    lv_label_set_text(footer,
                      "Some repeater owners may allow guests to view\n"
                      "metrics without a password.");
    lv_obj_set_style_text_color(footer, lv_color_make(140, 140, 140), 0);
    meck_set_font(footer, &meck_montserrat_16, 0);
    lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, -100);
    lv_label_set_long_mode(footer, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(footer, SCREEN_WIDTH - 80);
    lv_obj_set_style_text_align(footer, LV_TEXT_ALIGN_CENTER, 0);

    // Keyboard (hidden by default — shown when textarea focused).
    // meck_style_keyboard applies the codebase-wide layout (QWERTY /
    // QWERTZ / AZERTY per user prefs), font, padding, and dark/light
    // theme — without it the keyboard reverts to LVGL's default look,
    // which doesn't match the rest of the UI.
    kb_admin_password = lv_keyboard_create(scr_admin_login);
    meck_style_keyboard(kb_admin_password);
    lv_keyboard_set_textarea(kb_admin_password, ta_admin_password);
    lv_obj_add_flag(kb_admin_password, LV_OBJ_FLAG_HIDDEN);
    // Show keyboard when textarea focused, hide on defocus
    lv_obj_add_event_cb(ta_admin_password, on_admin_password_focused,
                        LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(ta_admin_password, on_admin_password_defocused,
                        LV_EVENT_DEFOCUSED, NULL);
}

// ============================================================================
// Admin home (menu list) + subpages — piece 4
// ----------------------------------------------------------------------------
// Replaces the piece-3 "logged in" placeholder. The home screen is a
// scrollable list of menu rows under a persistent banner:
//
//   Status        → scr_admin_status       (full RepeaterStats view)
//   Send Advert   → scr_admin_send_advert  (one-tap `advert` CLI)
//   Cmd Line      → scr_admin_cmd          (placeholder for piece 6)
//   Settings      → scr_admin_settings     (placeholder for piece 5)
//
// Banner colour:
//   - green  = admin session
//   - yellow = guest session
//
// Banner is shown ONLY on the home menu list. Subpages have their own
// title bar and back button; they share scr_admin_home as the back
// destination via on_admin_subpage_back.
// ============================================================================

// Helper: a tappable menu row that fills the full screen width. Same
// pattern as create_settings_row but single-line (just a label + right
// arrow), since admin menu rows are pure navigation with no value.
static lv_obj_t* create_admin_menu_row(lv_obj_t* parent, const char* label,
                                        int y, lv_event_cb_t cb) {
    lv_obj_t* btn = lv_button_create(parent);
    lv_obj_set_size(btn, SCREEN_WIDTH - 40, 70);
    lv_obj_set_pos(btn, 20, y);
    lv_obj_set_style_bg_color(btn, lv_color_make(25, 25, 35), 0);
    lv_obj_set_style_radius(btn, 10, 0);
    lv_obj_set_style_border_width(btn, 1, 0);
    lv_obj_set_style_border_color(btn, lv_color_make(50, 50, 60), 0);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, label);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    meck_set_font(lbl, &meck_montserrat_22, 0);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 10, 0);

    lv_obj_t* arrow = lv_label_create(btn);
    lv_label_set_text(arrow, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(arrow, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_set_font(arrow, &meck_montserrat_18, 0);
    lv_obj_align(arrow, LV_ALIGN_RIGHT_MID, -10, 0);

    return btn;
}

// Standard subpage scaffolding: clock/battery header on a fresh slot,
// black background, back button top-left (returns to admin home),
// title to the right of the back button. Returns the screen handle.
static lv_obj_t* create_admin_subpage(const char* title_text, int slot) {
    lv_obj_t* scr = lv_obj_create(NULL);
    lock_screen_scroll(scr);
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    screen_attach_clock_battery(scr, slot, &meck_montserrat_22, 30);

    // Back button → admin home menu list.
    lv_obj_t* btn_back = lv_button_create(scr);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t* bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_admin_subpage_back, LV_EVENT_CLICKED, NULL);

    // Title — short to clear the camera punch-hole.
    lv_obj_t* title = lv_label_create(scr);
    lv_label_set_text(title, title_text);
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    return scr;
}

static void create_admin_status_screen() {
    scr_admin_status = create_admin_subpage("Status", 11);

    // Refresh button — top-right under the header. No icon (LVGL's
    // LV_SYMBOL_REFRESH points at U+F021 which isn't in our font
    // glyph set — would render as tofu).
    btn_admin_status_refresh = lv_button_create(scr_admin_status);
    lv_obj_set_size(btn_admin_status_refresh, 160, 60);
    lv_obj_align(btn_admin_status_refresh, LV_ALIGN_TOP_RIGHT, -10, 30);
    lv_obj_set_style_bg_color(btn_admin_status_refresh,
                              lv_palette_darken(LV_PALETTE_INDIGO, 1), 0);
    lv_obj_set_style_radius(btn_admin_status_refresh, 8, 0);
    lbl_admin_status_refresh = lv_label_create(btn_admin_status_refresh);
    lv_label_set_text(lbl_admin_status_refresh, "Refresh");
    lv_obj_set_style_text_color(lbl_admin_status_refresh, lv_color_white(), 0);
    meck_set_font(lbl_admin_status_refresh, &meck_montserrat_18, 0);
    lv_obj_center(lbl_admin_status_refresh);
    lv_obj_add_event_cb(btn_admin_status_refresh, on_admin_status_refresh_tap,
                        LV_EVENT_CLICKED, NULL);

    // "Updated Ns ago" / "Refreshing..." line — under the title row.
    lbl_admin_status_updated = lv_label_create(scr_admin_status);
    lv_label_set_text(lbl_admin_status_updated, "Tap Refresh to load");
    lv_obj_set_style_text_color(lbl_admin_status_updated,
                                lv_color_make(180, 180, 180), 0);
    meck_set_font(lbl_admin_status_updated, &meck_montserrat_16, 0);
    lv_obj_align(lbl_admin_status_updated, LV_ALIGN_TOP_LEFT, 20, 110);

    // Status body — scrollable container with the full multi-line
    // RepeaterStats output. Scrollable in case 14+ fields don't fit
    // on screen at the user's font scale.
    lv_obj_t* scroll = lv_obj_create(scr_admin_status);
    lv_obj_set_size(scroll, SCREEN_WIDTH - 40, SCREEN_HEIGHT - 240);
    lv_obj_set_pos(scroll, 20, 150);
    lv_obj_set_style_bg_color(scroll, lv_color_make(10, 10, 15), 0);
    lv_obj_set_style_border_width(scroll, 0, 0);
    lv_obj_set_style_radius(scroll, 8, 0);
    lv_obj_set_style_pad_all(scroll, 15, 0);
    lv_obj_set_scroll_dir(scroll, LV_DIR_VER);

    lbl_admin_status_body = lv_label_create(scroll);
    lv_label_set_text(lbl_admin_status_body,
                      "No status data yet.\n\n"
                      "Tap the Refresh button to request the\n"
                      "repeater's current statistics.");
    lv_obj_set_style_text_color(lbl_admin_status_body,
                                lv_color_make(220, 220, 220), 0);
    meck_set_font(lbl_admin_status_body, &meck_montserrat_18, 0);
    lv_label_set_long_mode(lbl_admin_status_body, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_admin_status_body, SCREEN_WIDTH - 90);
}

static void create_admin_send_advert_screen() {
    scr_admin_send_advert = create_admin_subpage("Send Advert", 12);

    // Explanation text.
    lv_obj_t* explain = lv_label_create(scr_admin_send_advert);
    lv_label_set_text(explain,
                      "Send a flood advert from the repeater so\n"
                      "other nodes learn its current path.");
    lv_obj_set_style_text_color(explain, lv_color_make(200, 200, 200), 0);
    meck_set_font(explain, &meck_montserrat_18, 0);
    lv_label_set_long_mode(explain, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(explain, SCREEN_WIDTH - 40);
    lv_obj_align(explain, LV_ALIGN_TOP_LEFT, 20, 130);

    // The button itself — full-width-ish, centered, prominent.
    btn_admin_send_advert = lv_button_create(scr_admin_send_advert);
    lv_obj_set_size(btn_admin_send_advert, SCREEN_WIDTH - 80, 80);
    lv_obj_align(btn_admin_send_advert, LV_ALIGN_TOP_MID, 0, 230);
    lv_obj_set_style_bg_color(btn_admin_send_advert,
                              lv_palette_darken(LV_PALETTE_INDIGO, 1), 0);
    lv_obj_set_style_radius(btn_admin_send_advert, 10, 0);
    lv_obj_t* btn_lbl = lv_label_create(btn_admin_send_advert);
    lv_label_set_text(btn_lbl, "Send Advert");
    lv_obj_set_style_text_color(btn_lbl, lv_color_white(), 0);
    meck_set_font(btn_lbl, &meck_montserrat_24, 0);
    lv_obj_center(btn_lbl);
    lv_obj_add_event_cb(btn_admin_send_advert, on_admin_send_advert_tap,
                        LV_EVENT_CLICKED, NULL);

    // Status text below the button — "Sending..." / "Sent" / etc.
    lbl_admin_send_advert_status = lv_label_create(scr_admin_send_advert);
    lv_label_set_text(lbl_admin_send_advert_status,
                      "Tap the button to send an advert command.");
    lv_obj_set_style_text_color(lbl_admin_send_advert_status,
                                lv_color_make(180, 180, 180), 0);
    meck_set_font(lbl_admin_send_advert_status, &meck_montserrat_18, 0);
    lv_label_set_long_mode(lbl_admin_send_advert_status, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_admin_send_advert_status, SCREEN_WIDTH - 40);
    lv_obj_align(lbl_admin_send_advert_status, LV_ALIGN_TOP_LEFT, 20, 340);

    // Response text — the actual reply from the repeater's CLI.
    lbl_admin_send_advert_response = lv_label_create(scr_admin_send_advert);
    lv_label_set_text(lbl_admin_send_advert_response, "");
    lv_obj_set_style_text_color(lbl_admin_send_advert_response,
                                lv_color_white(), 0);
    meck_set_font(lbl_admin_send_advert_response, &meck_montserrat_18, 0);
    lv_label_set_long_mode(lbl_admin_send_advert_response, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_admin_send_advert_response, SCREEN_WIDTH - 40);
    lv_obj_align(lbl_admin_send_advert_response, LV_ALIGN_TOP_LEFT, 20, 410);
}

// Cmd Line placeholder — piece 6 builds this out.
// Cmd Line subpage — piece 6. Free-form CLI input with response
// scrollback. The repeater understands commands like `neighbors`,
// `ver`, `clock sync`, `get freq`, `set name <name>`, `reboot`, etc.
//
// Layout:
//   y=0..100   clock/battery header (via create_admin_subpage)
//   y=30..    title "Cmd Line"
//   y=130..   scrollback container — fills the space above the textarea
//   bottom:   textarea + send button (10px margins)
//   keyboard slides up on textarea focus; textarea/send lift above it.
//
// Limits:
//   - Command length: 100 chars (well within MeshCore single-packet
//     payload limits)
//   - Scrollback depth: LVGL handles whatever fits in memory; we trim
//     to ~50 entries on the fly to keep it responsive.
//
// Sharing the CLI dispatch path: meck_admin_cli_dispatch is shared
// with Send Advert. The dispatcher routes on whichever in-flight flag
// is set (g_admin_send_advert_in_flight vs g_admin_cmd_in_flight).

// Append a command/response pair to the scrollback. is_error flips
// the response colour to red for "send failed" feedback.
static void admin_cmd_add_entry(const char* cmd, const char* response,
                                 bool is_error) {
    if (!obj_admin_cmd_scroll) return;

    // Trim the scrollback if it's getting too long. Each entry is a
    // container holding two labels; deleting the container deletes
    // its children. lv_obj_get_child_count walks the immediate
    // children of the scroll only.
    while (lv_obj_get_child_count(obj_admin_cmd_scroll) >= ADMIN_CMD_MAX_ENTRIES) {
        lv_obj_t* oldest = lv_obj_get_child(obj_admin_cmd_scroll, 0);
        if (!oldest) break;
        lv_obj_delete(oldest);
    }

    lv_obj_t* row = lv_obj_create(obj_admin_cmd_scroll);
    lv_obj_set_size(row, SCREEN_WIDTH - 60, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(row, lv_color_make(15, 15, 22), 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_radius(row, 8, 0);
    lv_obj_set_style_pad_all(row, 12, 0);
    lv_obj_set_style_margin_bottom(row, 10, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(row, 6, 0);
    lock_screen_scroll(row);

    if (cmd && cmd[0]) {
        lv_obj_t* cmd_lbl = lv_label_create(row);
        char buf[ADMIN_CMD_MAX_LEN + 8];
        snprintf(buf, sizeof(buf), "> %s", cmd);
        lv_label_set_text(cmd_lbl, buf);
        lv_obj_set_style_text_color(cmd_lbl,
            lv_palette_main(LV_PALETTE_CYAN), 0);
        meck_set_font(cmd_lbl, &meck_montserrat_18, 0);
        lv_label_set_long_mode(cmd_lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(cmd_lbl, SCREEN_WIDTH - 90);
    }

    if (response && response[0]) {
        lv_obj_t* resp_lbl = lv_label_create(row);
        lv_label_set_text(resp_lbl, response);
        lv_obj_set_style_text_color(resp_lbl,
            is_error ? lv_palette_main(LV_PALETTE_RED) : lv_color_white(), 0);
        meck_set_font(resp_lbl, &meck_montserrat_18, 0);
        lv_label_set_long_mode(resp_lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(resp_lbl, SCREEN_WIDTH - 90);
    }

    // Scroll to the bottom so the newest entry is visible.
    lv_obj_scroll_to_view(row, LV_ANIM_OFF);
}

// Issue the CLI command currently in the textarea. Validates non-empty
// and not already in flight; clears the textarea + closes the keyboard
// on success so the user can keep typing or read the response.
static void admin_cmd_send_current() {
    if (!ta_admin_cmd_input) return;
    if (g_admin_login_contact_idx < 0) return;
    if (g_admin_cmd_in_flight) return;
    if (g_admin_send_advert_in_flight) {
        // Another CLI is mid-flight; refuse politely.
        admin_cmd_add_entry(NULL,
            "Another CLI request is in flight. Try again in a moment.",
            true);
        return;
    }

    const char* text = lv_textarea_get_text(ta_admin_cmd_input);
    if (!text || !text[0]) return;

    // Truncate to ADMIN_CMD_MAX_LEN just in case.
    char cmd_buf[ADMIN_CMD_MAX_LEN + 1];
    strncpy(cmd_buf, text, ADMIN_CMD_MAX_LEN);
    cmd_buf[ADMIN_CMD_MAX_LEN] = '\0';

    printf("Admin Cmd: sending \"%s\" to contact[%d]\n",
           cmd_buf, g_admin_login_contact_idx);

    g_admin_cmd_in_flight = true;

    // Append the command immediately with an empty response slot, so
    // the user sees feedback even before the radio reply arrives.
    // The response will be filled in by meck_admin_cli_dispatch.
    admin_cmd_add_entry(cmd_buf, "(waiting...)", false);

    // Clear the textarea and hide the keyboard so focus returns to
    // the scrollback, ready for the user to read the reply.
    lv_textarea_set_text(ta_admin_cmd_input, "");
    if (kb_admin_cmd_input) {
        lv_obj_add_flag(kb_admin_cmd_input, LV_OBJ_FLAG_HIDDEN);
    }
    if (btn_admin_cmd_send) {
        lv_obj_add_state(btn_admin_cmd_send, LV_STATE_DISABLED);
    }

    meck_request_admin_cli(g_admin_login_contact_idx, cmd_buf);
}

static void on_admin_cmd_send_tap(lv_event_t *e) {
    admin_cmd_send_current();
}

// Textarea focus → show the keyboard and reposition the textarea + send
// button above it. Mirrors the compose pattern.
static void on_admin_cmd_input_focused(lv_event_t *e) {
    if (!kb_admin_cmd_input) return;
    lv_keyboard_set_textarea(kb_admin_cmd_input, ta_admin_cmd_input);
    // Hardware keyboard: keep the on-screen keyboard down and the input in
    // its resting position; the binding above stays for Enter/Esc routing.
    if (meck_hw_keyboard_active()) return;
    lv_obj_remove_flag(kb_admin_cmd_input, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(kb_admin_cmd_input);
    if (ta_admin_cmd_input)
        lv_obj_align(ta_admin_cmd_input, LV_ALIGN_BOTTOM_LEFT, 10,
                     -(15 + MECK_KB_HEIGHT));
    if (btn_admin_cmd_send)
        lv_obj_align(btn_admin_cmd_send, LV_ALIGN_BOTTOM_RIGHT, -10,
                     -(15 + MECK_KB_HEIGHT));
    // Shrink scrollback to make room for keyboard + textarea (same
    // pattern as meck_relayout_compose in the channel messages screen)
    if (obj_admin_cmd_scroll)
        lv_obj_set_height(obj_admin_cmd_scroll,
                          SCREEN_HEIGHT - 110 - 90 - MECK_KB_HEIGHT);
}

static void on_admin_cmd_input_defocused(lv_event_t *e) {
    if (!kb_admin_cmd_input) return;
    lv_obj_add_flag(kb_admin_cmd_input, LV_OBJ_FLAG_HIDDEN);
    if (ta_admin_cmd_input)
        lv_obj_align(ta_admin_cmd_input, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    if (btn_admin_cmd_send)
        lv_obj_align(btn_admin_cmd_send, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    // Restore scrollback to full height
    if (obj_admin_cmd_scroll)
        lv_obj_set_height(obj_admin_cmd_scroll, SCREEN_HEIGHT - 200);
}

// Keyboard event → handle enter (send) and cancel (close keyboard).
static void on_admin_cmd_kb_event(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        admin_cmd_send_current();
    } else if (code == LV_EVENT_CANCEL) {
        on_admin_cmd_input_defocused(e);
    }
}

static void create_admin_cmd_screen() {
    scr_admin_cmd = create_admin_subpage("Cmd Line", 13);

    // Bottom row: textarea (left, expanding) + send button (right).
    // Built first so we know its height before positioning the
    // scrollback. Both lift above the keyboard on textarea focus via
    // on_admin_cmd_input_focused.
    btn_admin_cmd_send = lv_button_create(scr_admin_cmd);
    lv_obj_set_size(btn_admin_cmd_send, 130, 70);
    lv_obj_align(btn_admin_cmd_send, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_bg_color(btn_admin_cmd_send,
        lv_palette_darken(LV_PALETTE_INDIGO, 1), 0);
    lv_obj_set_style_radius(btn_admin_cmd_send, 8, 0);
    lv_obj_t* send_lbl = lv_label_create(btn_admin_cmd_send);
    lv_label_set_text(send_lbl, "Send");
    lv_obj_set_style_text_color(send_lbl, lv_color_white(), 0);
    meck_set_font(send_lbl, &meck_montserrat_18, 0);
    lv_obj_center(send_lbl);
    lv_obj_add_event_cb(btn_admin_cmd_send, on_admin_cmd_send_tap,
                        LV_EVENT_CLICKED, NULL);

    ta_admin_cmd_input = lv_textarea_create(scr_admin_cmd);
    lv_obj_set_size(ta_admin_cmd_input, SCREEN_WIDTH - 170, 70);
    lv_obj_align(ta_admin_cmd_input, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_textarea_set_placeholder_text(ta_admin_cmd_input,
        "Command (e.g. neighbors, ver, get freq)");
    lv_textarea_set_one_line(ta_admin_cmd_input, true);
    lv_textarea_set_max_length(ta_admin_cmd_input, ADMIN_CMD_MAX_LEN);
    meck_set_font(ta_admin_cmd_input, &meck_montserrat_18, 0);
    lv_obj_add_event_cb(ta_admin_cmd_input, on_admin_cmd_input_focused,
                        LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(ta_admin_cmd_input, on_admin_cmd_input_defocused,
                        LV_EVENT_DEFOCUSED, NULL);

    // Keyboard — hidden by default, shown when textarea is focused.
    kb_admin_cmd_input = lv_keyboard_create(scr_admin_cmd);
    lv_obj_set_size(kb_admin_cmd_input, SCREEN_WIDTH, MECK_KB_HEIGHT);
    lv_obj_align(kb_admin_cmd_input, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_add_flag(kb_admin_cmd_input, LV_OBJ_FLAG_HIDDEN);
    meck_style_keyboard(kb_admin_cmd_input);
    lv_obj_add_event_cb(kb_admin_cmd_input, on_admin_cmd_kb_event,
                        LV_EVENT_ALL, NULL);

    // Scrollback — sits between the title (y=110) and the textarea
    // (bottom 90px). Reserved 110 top + 90 bottom = 200px outside
    // the scrollback area itself.
    obj_admin_cmd_scroll = lv_obj_create(scr_admin_cmd);
    lv_obj_set_size(obj_admin_cmd_scroll,
                    SCREEN_WIDTH - 20, SCREEN_HEIGHT - 200);
    lv_obj_set_pos(obj_admin_cmd_scroll, 10, 110);
    lv_obj_set_style_bg_color(obj_admin_cmd_scroll,
                              lv_color_make(8, 8, 12), 0);
    lv_obj_set_style_border_width(obj_admin_cmd_scroll, 0, 0);
    lv_obj_set_style_radius(obj_admin_cmd_scroll, 8, 0);
    lv_obj_set_style_pad_all(obj_admin_cmd_scroll, 10, 0);
    lv_obj_set_scroll_dir(obj_admin_cmd_scroll, LV_DIR_VER);
    lv_obj_set_flex_flow(obj_admin_cmd_scroll, LV_FLEX_FLOW_COLUMN);
}

// Settings menu list — piece 5. Subpage with a scroll container of
// rows. Rows are rebuilt on every entry so guest vs admin visibility
// is honoured after a re-login. Each row taps into a shared
// placeholder subpage (scr_admin_setting_placeholder) until the
// individual setting screens get built out.
static void create_admin_settings_screen() {
    scr_admin_settings = create_admin_subpage("Settings", 14);

    // Scroll container for the rows. Sits below the title with
    // bottom margin for the home indicator area.
    obj_admin_settings_list = lv_obj_create(scr_admin_settings);
    lv_obj_set_size(obj_admin_settings_list, SCREEN_WIDTH, SCREEN_HEIGHT - 110);
    lv_obj_set_pos(obj_admin_settings_list, 0, 100);
    lv_obj_set_style_bg_color(obj_admin_settings_list, lv_color_black(), 0);
    lv_obj_set_style_border_width(obj_admin_settings_list, 0, 0);
    lv_obj_set_style_pad_all(obj_admin_settings_list, 10, 0);
    lv_obj_set_scroll_dir(obj_admin_settings_list, LV_DIR_VER);

    // First populate happens in on_admin_menu_settings_tap via
    // admin_settings_rebuild_list, since session role isn't known at
    // creation time.
}

// Generic placeholder subpage for individual settings, reused for all
// 10 settings. The title and body labels get updated on entry to
// reflect which row was tapped.
static void create_admin_setting_placeholder_screen() {
    scr_admin_setting_placeholder = lv_obj_create(NULL);
    lock_screen_scroll(scr_admin_setting_placeholder);
    lv_obj_set_style_bg_color(scr_admin_setting_placeholder, lv_color_black(), 0);
    screen_attach_clock_battery(scr_admin_setting_placeholder, 15,
                                &meck_montserrat_22, 30);

    // Back button — returns to the Settings menu list (not Admin
    // home, which is where on_admin_subpage_back goes).
    lv_obj_t* btn_back = lv_button_create(scr_admin_setting_placeholder);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t* bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_admin_setting_back,
                        LV_EVENT_CLICKED, NULL);

    // Title — populated on entry. Stored in the file-scope handle so
    // on_admin_setting_row_tap can update it.
    lbl_admin_setting_placeholder_title = lv_label_create(scr_admin_setting_placeholder);
    lv_label_set_text(lbl_admin_setting_placeholder_title, "");
    lv_obj_set_style_text_color(lbl_admin_setting_placeholder_title,
                                lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(lbl_admin_setting_placeholder_title, &meck_montserrat_24, 0);
    lv_obj_align(lbl_admin_setting_placeholder_title,
                 LV_ALIGN_TOP_LEFT, 120, 30);

    // Body — populated on entry.
    lbl_admin_setting_placeholder_body = lv_label_create(scr_admin_setting_placeholder);
    lv_label_set_text(lbl_admin_setting_placeholder_body, "");
    lv_obj_set_style_text_color(lbl_admin_setting_placeholder_body,
                                lv_color_make(180, 180, 180), 0);
    meck_set_font(lbl_admin_setting_placeholder_body, &meck_montserrat_18, 0);
    lv_label_set_long_mode(lbl_admin_setting_placeholder_body,
                           LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_admin_setting_placeholder_body, SCREEN_WIDTH - 40);
    lv_obj_align(lbl_admin_setting_placeholder_body,
                 LV_ALIGN_TOP_LEFT, 20, 130);
}

// Back from an individual setting placeholder → Settings menu list
// (not Admin home, which is where on_admin_subpage_back goes).
static void on_admin_setting_back(lv_event_t *e) {
    if (scr_admin_settings) lv_screen_load(scr_admin_settings);
}

// Per-row tap handler. user_data is a static const char* setting
// name. Updates the placeholder screen's title + body and loads it.
static void on_admin_setting_row_tap(lv_event_t *e) {
    const char* name = (const char*)lv_event_get_user_data(e);
    if (!name) return;

    printf("Admin Settings: '%s' tapped\n", name);

    // Firmware Info has its own dedicated subpage. Auto-send "ver" on
    // entry so the user doesn't have to tap Refresh to see the values.
    if (strcmp(name, "Firmware Info") == 0) {
        if (scr_admin_fw_info) {
            lv_screen_load(scr_admin_fw_info);
            admin_fw_info_request();
        }
        return;
    }

    // Neighbours uses the binary REQ_TYPE_GET_NEIGHBOURS path to
    // retrieve the repeater's full neighbour list (paginated), rather
    // than the text CLI 'neighbors' command which is hard-capped at
    // ~8 entries by the repeater's reply buffer. Guest-visible.
    if (strcmp(name, "Neighbours") == 0) {
        if (scr_admin_neighbours) {
            lv_screen_load(scr_admin_neighbours);
            admin_neighbours_request_first_page();
        }
        return;
    }

    if (lbl_admin_setting_placeholder_title) {
        lv_label_set_text(lbl_admin_setting_placeholder_title, name);
    }
    if (lbl_admin_setting_placeholder_body) {
        char body[160];
        snprintf(body, sizeof(body),
                 "%s\n\n"
                 "This setting is part of the admin menu, but the\n"
                 "control screen for it isn't built yet. It'll be\n"
                 "added in a future piece.",
                 name);
        lv_label_set_text(lbl_admin_setting_placeholder_body, body);
    }
    if (scr_admin_setting_placeholder) {
        lv_screen_load(scr_admin_setting_placeholder);
    }
}

// ---- Firmware Info subpage ----
//
// Sends the "ver" CLI command (handled upstream by CommonCLI.cpp's
// `} else if (memcmp(command, "ver", 3) == 0) {` branch). The reply
// format is "<version> (Build: <date>)" with no trailing newline.
// admin_fw_info_parse_and_render splits it into the version and build
// labels.

static void create_admin_fw_info_screen() {
    scr_admin_fw_info = lv_obj_create(NULL);
    lock_screen_scroll(scr_admin_fw_info);
    lv_obj_set_style_bg_color(scr_admin_fw_info, lv_color_black(), 0);
    screen_attach_clock_battery(scr_admin_fw_info, 16,
                                &meck_montserrat_22, 30);

    // Back button — returns to Settings menu list (not Admin home,
    // which is where on_admin_subpage_back goes). Mirrors the
    // placeholder subpage scaffolding for the same reason.
    lv_obj_t* btn_back = lv_button_create(scr_admin_fw_info);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t* bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_admin_fw_info_back,
                        LV_EVENT_CLICKED, NULL);

    // Title — short to clear the camera punch-hole.
    lv_obj_t* title = lv_label_create(scr_admin_fw_info);
    lv_label_set_text(title, "Firmware Info");
    lv_obj_set_style_text_color(title,
                                lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    // Refresh button — top-right under the header. Mirrors Status's
    // layout. No icon (LV_SYMBOL_REFRESH renders as tofu in our font
    // glyph set).
    btn_admin_fw_info_refresh = lv_button_create(scr_admin_fw_info);
    lv_obj_set_size(btn_admin_fw_info_refresh, 160, 60);
    lv_obj_align(btn_admin_fw_info_refresh, LV_ALIGN_TOP_RIGHT, -10, 30);
    lv_obj_set_style_bg_color(btn_admin_fw_info_refresh,
                              lv_palette_darken(LV_PALETTE_INDIGO, 1), 0);
    lv_obj_set_style_radius(btn_admin_fw_info_refresh, 8, 0);
    lv_obj_t* refresh_lbl = lv_label_create(btn_admin_fw_info_refresh);
    lv_label_set_text(refresh_lbl, "Refresh");
    lv_obj_set_style_text_color(refresh_lbl, lv_color_white(), 0);
    meck_set_font(refresh_lbl, &meck_montserrat_18, 0);
    lv_obj_center(refresh_lbl);
    lv_obj_add_event_cb(btn_admin_fw_info_refresh,
                        on_admin_fw_info_refresh_tap,
                        LV_EVENT_CLICKED, NULL);

    // Status line — "Refreshing..." / timeout / send-failed text.
    // Sits under the title row, mirrors the Status subpage's
    // lbl_admin_status_updated.
    lbl_admin_fw_info_status = lv_label_create(scr_admin_fw_info);
    lv_label_set_text(lbl_admin_fw_info_status, "");
    lv_obj_set_style_text_color(lbl_admin_fw_info_status,
                                lv_color_make(180, 180, 180), 0);
    meck_set_font(lbl_admin_fw_info_status, &meck_montserrat_16, 0);
    lv_obj_align(lbl_admin_fw_info_status, LV_ALIGN_TOP_LEFT, 20, 110);

    // "Version" caption.
    lv_obj_t* cap_version = lv_label_create(scr_admin_fw_info);
    lv_label_set_text(cap_version, "Version");
    lv_obj_set_style_text_color(cap_version,
                                lv_color_make(150, 150, 150), 0);
    meck_set_font(cap_version, &meck_montserrat_18, 0);
    lv_obj_align(cap_version, LV_ALIGN_TOP_LEFT, 20, 160);

    // Version value label — populated from the parsed response.
    lbl_admin_fw_info_version = lv_label_create(scr_admin_fw_info);
    lv_label_set_text(lbl_admin_fw_info_version, "");
    lv_obj_set_style_text_color(lbl_admin_fw_info_version,
                                lv_color_white(), 0);
    meck_set_font(lbl_admin_fw_info_version, &meck_montserrat_24, 0);
    lv_label_set_long_mode(lbl_admin_fw_info_version,
                           LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_admin_fw_info_version, SCREEN_WIDTH - 40);
    lv_obj_align(lbl_admin_fw_info_version, LV_ALIGN_TOP_LEFT, 20, 190);

    // "Build" caption.
    lv_obj_t* cap_build = lv_label_create(scr_admin_fw_info);
    lv_label_set_text(cap_build, "Build");
    lv_obj_set_style_text_color(cap_build,
                                lv_color_make(150, 150, 150), 0);
    meck_set_font(cap_build, &meck_montserrat_18, 0);
    lv_obj_align(cap_build, LV_ALIGN_TOP_LEFT, 20, 260);

    // Build value label — populated from the parsed response.
    lbl_admin_fw_info_build = lv_label_create(scr_admin_fw_info);
    lv_label_set_text(lbl_admin_fw_info_build, "");
    lv_obj_set_style_text_color(lbl_admin_fw_info_build,
                                lv_color_white(), 0);
    meck_set_font(lbl_admin_fw_info_build, &meck_montserrat_24, 0);
    lv_label_set_long_mode(lbl_admin_fw_info_build,
                           LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_admin_fw_info_build, SCREEN_WIDTH - 40);
    lv_obj_align(lbl_admin_fw_info_build, LV_ALIGN_TOP_LEFT, 20, 290);
}

// Send "ver" to the logged-in repeater. Sets the in-flight flag,
// stamps the send timestamp for timeout polling, disables Refresh
// while in flight, and clears the value labels so stale text from a
// previous fetch doesn't linger during the round-trip.
static void admin_fw_info_request() {
    if (g_admin_login_contact_idx < 0) return;
    if (g_admin_fw_in_flight) return;  // debounce

    printf("Admin: fw info request for contact[%d]\n",
           g_admin_login_contact_idx);

    g_admin_fw_in_flight = true;
    g_admin_fw_sent_ms   = (unsigned long)esp_log_timestamp();

    if (lbl_admin_fw_info_status) {
        lv_label_set_text(lbl_admin_fw_info_status, "Refreshing...");
        lv_obj_set_style_text_color(lbl_admin_fw_info_status,
                                    lv_palette_main(LV_PALETTE_YELLOW), 0);
    }
    if (lbl_admin_fw_info_version) {
        lv_label_set_text(lbl_admin_fw_info_version, "");
    }
    if (lbl_admin_fw_info_build) {
        lv_label_set_text(lbl_admin_fw_info_build, "");
    }
    if (btn_admin_fw_info_refresh) {
        lv_obj_add_state(btn_admin_fw_info_refresh, LV_STATE_DISABLED);
    }

    meck_request_admin_cli(g_admin_login_contact_idx, "ver");
}

// Parse upstream CommonCLI's `ver` reply:
//   sprintf(reply, "%s (Build: %s)", firmwareVer, buildDate);
// Splits on the literal " (Build: " separator and the trailing ")".
// If the reply doesn't match this shape, fall back to displaying the
// whole raw response in the Version label with a yellow note in the
// status line so the user can see what arrived.
static void admin_fw_info_parse_and_render(const char* response) {
    if (!lbl_admin_fw_info_version || !lbl_admin_fw_info_build ||
        !lbl_admin_fw_info_status) {
        return;
    }

    if (!response || !response[0]) {
        lv_label_set_text(lbl_admin_fw_info_version, "(no reply)");
        lv_label_set_text(lbl_admin_fw_info_build, "");
        lv_label_set_text(lbl_admin_fw_info_status,
                          "Empty reply from repeater.");
        lv_obj_set_style_text_color(lbl_admin_fw_info_status,
                                    lv_palette_main(LV_PALETTE_YELLOW), 0);
        return;
    }

    const char* sep = strstr(response, " (Build: ");
    const char* end = response + strlen(response);
    bool has_trailing_paren = (end > response && *(end - 1) == ')');

    if (sep && has_trailing_paren) {
        // Version: response[0..sep)
        char version[64];
        size_t vlen = (size_t)(sep - response);
        if (vlen >= sizeof(version)) vlen = sizeof(version) - 1;
        memcpy(version, response, vlen);
        version[vlen] = '\0';

        // Build: sep + strlen(" (Build: ") .. end-1 (drop closing ')')
        const char* bstart = sep + strlen(" (Build: ");
        size_t blen = (size_t)(end - 1 - bstart);
        char build[64];
        if (blen >= sizeof(build)) blen = sizeof(build) - 1;
        memcpy(build, bstart, blen);
        build[blen] = '\0';

        lv_label_set_text(lbl_admin_fw_info_version, version);
        lv_label_set_text(lbl_admin_fw_info_build, build);
        lv_label_set_text(lbl_admin_fw_info_status, "");
        lv_obj_set_style_text_color(lbl_admin_fw_info_status,
                                    lv_color_make(180, 180, 180), 0);
    } else {
        // Unexpected format. Surface the raw reply rather than silently
        // failing, so the user (and field debugging) can see what was
        // received.
        lv_label_set_text(lbl_admin_fw_info_version, response);
        lv_label_set_text(lbl_admin_fw_info_build, "");
        lv_label_set_text(lbl_admin_fw_info_status,
                          "Unexpected reply format.");
        lv_obj_set_style_text_color(lbl_admin_fw_info_status,
                                    lv_palette_main(LV_PALETTE_YELLOW), 0);
    }
}

static void on_admin_fw_info_refresh_tap(lv_event_t *e) {
    admin_fw_info_request();
}

// Back from Firmware Info → Settings menu list (not Admin home, which
// is where on_admin_subpage_back goes — Firmware Info is reached via
// Settings, so back should return there).
static void on_admin_fw_info_back(lv_event_t *e) {
    if (scr_admin_settings) lv_screen_load(scr_admin_settings);
}

// ---- Neighbours subpage ----
//
// Uses the binary REQ_TYPE_GET_NEIGHBOURS protocol (upstream MyMesh.cpp
// line ~279) with pagination. The CLI 'neighbors' command is hard-
// capped at ~8 entries by the repeater's 134-byte reply buffer; this
// binary form returns the total count plus a page of entries per
// request, so we iterate with incrementing offset until we have the
// whole list. MECK_ADMIN_NEIGHBOURS_PAGE_SIZE=10 leaves headroom in
// the 130-byte results buffer at MECK_ADMIN_NEIGHBOURS_PREFIX_LEN=4
// (9 bytes per entry, 90 bytes used).
//
// Per-entry display format (per E's spec):
//   <name> (XXXX) <±N.Ndb> <Ns/m/h/d ago>      (name resolved)
//   <XXXXXXXX> <±N.Ndb> <Ns/m/h/d ago>          (name not resolved)
// where XXXX is the first 2 hex bytes of the prefix (4 chars), and
// XXXXXXXX is the full 4-byte prefix as hex (8 chars). Emoji and
// other non-Latin-1 characters are stripped from names via
// strip_unrenderable (the same helper used by the contacts list)
// so resolved names don't render with tofu boxes.

static void create_admin_neighbours_screen() {
    scr_admin_neighbours = lv_obj_create(NULL);
    lock_screen_scroll(scr_admin_neighbours);
    lv_obj_set_style_bg_color(scr_admin_neighbours, lv_color_black(), 0);
    screen_attach_clock_battery(scr_admin_neighbours, 17,
                                &meck_montserrat_22, 30);

    // Back button — returns to Settings menu list. Mirrors the
    // placeholder/fw_info scaffolding for the same reason: Neighbours
    // is reached via Settings, so back should return there (not Admin
    // home, which is where on_admin_subpage_back goes).
    lv_obj_t* btn_back = lv_button_create(scr_admin_neighbours);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t* bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_admin_neighbours_back,
                        LV_EVENT_CLICKED, NULL);

    // Title.
    lv_obj_t* title = lv_label_create(scr_admin_neighbours);
    lv_label_set_text(title, "Neighbours");
    lv_obj_set_style_text_color(title,
                                lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    // Refresh button — top-right under the header.
    btn_admin_neighbours_refresh = lv_button_create(scr_admin_neighbours);
    lv_obj_set_size(btn_admin_neighbours_refresh, 160, 60);
    lv_obj_align(btn_admin_neighbours_refresh, LV_ALIGN_TOP_RIGHT, -10, 30);
    lv_obj_set_style_bg_color(btn_admin_neighbours_refresh,
                              lv_palette_darken(LV_PALETTE_INDIGO, 1), 0);
    lv_obj_set_style_radius(btn_admin_neighbours_refresh, 8, 0);
    lv_obj_t* refresh_lbl = lv_label_create(btn_admin_neighbours_refresh);
    lv_label_set_text(refresh_lbl, "Refresh");
    lv_obj_set_style_text_color(refresh_lbl, lv_color_white(), 0);
    meck_set_font(refresh_lbl, &meck_montserrat_18, 0);
    lv_obj_center(refresh_lbl);
    lv_obj_add_event_cb(btn_admin_neighbours_refresh,
                        on_admin_neighbours_refresh_tap,
                        LV_EVENT_CLICKED, NULL);

    // Status line — "Refreshing...", "N neighbours", timeout text, etc.
    lbl_admin_neighbours_status = lv_label_create(scr_admin_neighbours);
    lv_label_set_text(lbl_admin_neighbours_status, "");
    lv_obj_set_style_text_color(lbl_admin_neighbours_status,
                                lv_color_make(180, 180, 180), 0);
    meck_set_font(lbl_admin_neighbours_status, &meck_montserrat_16, 0);
    lv_obj_align(lbl_admin_neighbours_status, LV_ALIGN_TOP_LEFT, 20, 110);

    // Scrollable list container — rows appended one per entry as
    // pages arrive.
    obj_admin_neighbours_scroll = lv_obj_create(scr_admin_neighbours);
    lv_obj_set_size(obj_admin_neighbours_scroll,
                    SCREEN_WIDTH - 40, SCREEN_HEIGHT - 240);
    lv_obj_set_pos(obj_admin_neighbours_scroll, 20, 150);
    lv_obj_set_style_bg_color(obj_admin_neighbours_scroll,
                              lv_color_make(10, 10, 15), 0);
    lv_obj_set_style_border_width(obj_admin_neighbours_scroll, 0, 0);
    lv_obj_set_style_radius(obj_admin_neighbours_scroll, 8, 0);
    lv_obj_set_style_pad_all(obj_admin_neighbours_scroll, 10, 0);
    lv_obj_set_scroll_dir(obj_admin_neighbours_scroll, LV_DIR_VER);
    lv_obj_set_flex_flow(obj_admin_neighbours_scroll, LV_FLEX_FLOW_COLUMN);
}

// Start a fresh paginated retrieval: clear list/state, set in-flight,
// send offset=0. Used by both subpage entry and the Refresh button.
static void admin_neighbours_request_first_page() {
    if (g_admin_login_contact_idx < 0) return;
    if (g_admin_neighbours_in_flight) return;   // debounce

    printf("Admin: neighbours request_first_page for contact[%d]\n",
           g_admin_login_contact_idx);

    g_admin_neighbours_in_flight   = true;
    g_admin_neighbours_sent_ms     = (unsigned long)esp_log_timestamp();
    g_admin_neighbours_total       = 0;
    g_admin_neighbours_received    = 0;
    g_admin_neighbours_next_offset = 0;

    if (obj_admin_neighbours_scroll) {
        lv_obj_clean(obj_admin_neighbours_scroll);
    }
    if (lbl_admin_neighbours_status) {
        lv_label_set_text(lbl_admin_neighbours_status, "Refreshing...");
        lv_obj_set_style_text_color(lbl_admin_neighbours_status,
                                    lv_palette_main(LV_PALETTE_YELLOW), 0);
    }
    if (btn_admin_neighbours_refresh) {
        lv_obj_add_state(btn_admin_neighbours_refresh, LV_STATE_DISABLED);
    }

    meck_request_admin_neighbours(g_admin_login_contact_idx,
                                   MECK_ADMIN_NEIGHBOURS_PAGE_SIZE,
                                   0,
                                   MECK_ADMIN_NEIGHBOURS_ORDER_BY,
                                   MECK_ADMIN_NEIGHBOURS_PREFIX_LEN);
}

// Send the next page during an active pagination. Called from the
// dispatch handler when received < total. Updates _sent_ms so the
// timeout clock resets at every successful round-trip.
static void admin_neighbours_request_next_page() {
    if (g_admin_login_contact_idx < 0) return;
    if (!g_admin_neighbours_in_flight) return;  // cancelled mid-flight

    printf("Admin: neighbours request_next_page offset=%u\n",
           (unsigned)g_admin_neighbours_next_offset);

    g_admin_neighbours_sent_ms = (unsigned long)esp_log_timestamp();
    meck_request_admin_neighbours(g_admin_login_contact_idx,
                                   MECK_ADMIN_NEIGHBOURS_PAGE_SIZE,
                                   g_admin_neighbours_next_offset,
                                   MECK_ADMIN_NEIGHBOURS_ORDER_BY,
                                   MECK_ADMIN_NEIGHBOURS_PREFIX_LEN);
}

// Resolve a 4-byte pubkey prefix against the contact list. Iterates
// via getContactByIdx so we don't depend on whether
// BaseChatMesh::lookupContactByPubKey is public or protected — every
// other path in MeckUI uses getContactByIdx. Linear scan is fine
// given typical contact-list sizes (low hundreds) and the fact that
// this only runs at paginated-render time, never on hot paths.
//
// Returns true and fills name_out with up to name_sz bytes of the
// matching contact's name (already stripped of unrenderable
// codepoints and leading whitespace) if a match is found. Returns
// false if no contact matches.
static bool admin_neighbours_resolve_prefix(const uint8_t* prefix,
                                              char* name_out,
                                              size_t name_sz) {
    if (!prefix || !name_out || name_sz == 0) return false;
    Meck* mesh = meck_get_instance();
    if (!mesh) return false;

    // getNumContacts isn't an established UI accessor here, so use
    // getContactByIdx with an unbounded loop and bail on first miss.
    // Caps at 1024 as a sanity ceiling.
    for (uint32_t i = 0; i < 1024; i++) {
        ContactInfo ci;
        if (!mesh->getContactByIdx(i, ci)) break;
        if (memcmp(ci.id.pub_key, prefix, 4) == 0) {
            // Strip emoji / non-Latin-1 codepoints into a stack buffer
            // first, then trim leading whitespace before writing to
            // name_out. Many node names are prefixed with an emoji
            // followed by a space (e.g. "🌱 Petersham"); after
            // stripping we get " Petersham" which needs the leading
            // space removed.
            char tmp[64];
            strip_unrenderable(ci.name, tmp, sizeof(tmp));
            const char* p = tmp;
            while (*p == ' ' || *p == '\t') p++;
            strncpy(name_out, p, name_sz - 1);
            name_out[name_sz - 1] = '\0';
            return true;
        }
    }
    return false;
}

// Format SNR*4 as a dB string with one fractional digit when the value
// doesn't divide evenly. Mirrors upstream parseNeighborResponse's
// approach: snr_x4 / 4 for the integer part, ((|snr_x4| % 4) * 10) / 4
// for the fractional part. -21 → "-5.2dB", -20 → "-5dB".
static void admin_neighbours_format_snr(int8_t snr_x4,
                                          char* out, size_t out_sz) {
    if (!out || out_sz == 0) return;
    if (snr_x4 % 4 == 0) {
        snprintf(out, out_sz, "%ddB", snr_x4 / 4);
    } else {
        int int_part  = snr_x4 / 4;
        int frac_part = ((abs((int)snr_x4) % 4) * 10) / 4;
        snprintf(out, out_sz, "%d.%ddB", int_part, frac_part);
    }
}

// Format seconds-ago as a compact relative time. Coarse but consistent
// across magnitudes: secs / mins / hours / days. No upstream
// formatRelativeTime dependency (the upstream helper isn't in scope
// here).
static void admin_neighbours_format_time(uint32_t secs_ago,
                                           char* out, size_t out_sz) {
    if (!out || out_sz == 0) return;
    if (secs_ago < 60) {
        snprintf(out, out_sz, "%us ago", (unsigned)secs_ago);
    } else if (secs_ago < 3600) {
        snprintf(out, out_sz, "%um ago", (unsigned)(secs_ago / 60));
    } else if (secs_ago < 86400) {
        snprintf(out, out_sz, "%uh ago", (unsigned)(secs_ago / 3600));
    } else {
        snprintf(out, out_sz, "%ud ago", (unsigned)(secs_ago / 86400));
    }
}

// Append one row to the scrollable list. Each row is a single label
// whose text is the formatted line. Rows wrap if a long name pushes
// the line past the container width.
static void admin_neighbours_append_row(const uint8_t* prefix,
                                         uint32_t secs_ago,
                                         int8_t snr_x4) {
    if (!obj_admin_neighbours_scroll) return;

    char snr_str[12];
    admin_neighbours_format_snr(snr_x4, snr_str, sizeof(snr_str));

    char time_str[24];
    admin_neighbours_format_time(secs_ago, time_str, sizeof(time_str));

    // Line buffer: name(64) + 2-byte hex prefix(4) + parens + SNR + time
    // + separators, comfortably under 160.
    char line[160];
    char name[64];
    if (admin_neighbours_resolve_prefix(prefix, name, sizeof(name)) &&
        name[0] != '\0') {
        // "Name (3601) -5dB 2m ago"
        snprintf(line, sizeof(line),
                 "%s (%02X%02X) %s %s",
                 name, prefix[0], prefix[1], snr_str, time_str);
    } else {
        // "<3601aabb> -5dB 2m ago"
        snprintf(line, sizeof(line),
                 "<%02X%02X%02X%02X> %s %s",
                 prefix[0], prefix[1], prefix[2], prefix[3],
                 snr_str, time_str);
    }

    lv_obj_t* lbl = lv_label_create(obj_admin_neighbours_scroll);
    lv_label_set_text(lbl, line);
    lv_obj_set_style_text_color(lbl, lv_color_make(220, 220, 220), 0);
    meck_set_font(lbl, &meck_montserrat_18, 0);
    lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl, SCREEN_WIDTH - 80);
}

// Dispatch handler for REQ_TYPE_GET_NEIGHBOURS responses (registered
// via meck_register_admin_neighbours_callback in meck_ui_init). Runs
// on the LVGL task. `entries` is a packed byte blob of 9-byte records
// (4 prefix + 4 secs_ago + 1 snr*4) per the C-API contract in
// target.h.
static void admin_neighbours_dispatch(uint16_t total_count,
                                       uint16_t page_count,
                                       uint16_t offset,
                                       const uint8_t* entries,
                                       int contact_idx) {
    printf("Admin: neighbours dispatch total=%u page=%u offset=%u contact=%d "
           "(state: in_flight=%d total=%u received=%u next_offset=%u)\n",
           (unsigned)total_count, (unsigned)page_count, (unsigned)offset,
           contact_idx,
           g_admin_neighbours_in_flight ? 1 : 0,
           (unsigned)g_admin_neighbours_total,
           (unsigned)g_admin_neighbours_received,
           (unsigned)g_admin_neighbours_next_offset);

    // If the user navigated away mid-pagination, on_admin_neighbours_back
    // cleared the in-flight flag. Late-arriving pages should not append
    // rows to a hidden list or trigger more requests.
    if (!g_admin_neighbours_in_flight) {
        printf("Admin: neighbours dispatch dropped (not in flight)\n");
        return;
    }

    // First page (offset==0) seeds the expected total.
    if (offset == 0) {
        g_admin_neighbours_total = total_count;
    }

    // Append entries from this page.
    if (entries && page_count > 0) {
        const uint8_t* p = entries;
        for (uint16_t i = 0; i < page_count; i++) {
            uint8_t  prefix[4];
            uint32_t secs_ago;
            int8_t   snr_x4;
            memcpy(prefix, p, 4); p += 4;
            memcpy(&secs_ago, p, 4); p += 4;
            snr_x4 = (int8_t)*p; p += 1;
            admin_neighbours_append_row(prefix, secs_ago, snr_x4);
            g_admin_neighbours_received++;
        }
    }

    g_admin_neighbours_next_offset = g_admin_neighbours_received;

    // Decide what to do next.
    bool more_expected = (g_admin_neighbours_received < g_admin_neighbours_total);
    bool repeater_stalled = (page_count == 0 && more_expected);

    if (repeater_stalled) {
        // We asked for more, but the repeater returned an empty page
        // while still claiming there are more entries. Stop here and
        // show what we have.
        printf("Admin: neighbours pagination stalled at %u/%u\n",
               (unsigned)g_admin_neighbours_received,
               (unsigned)g_admin_neighbours_total);
        g_admin_neighbours_in_flight = false;
        if (lbl_admin_neighbours_status) {
            char buf[64];
            snprintf(buf, sizeof(buf),
                     "Partial: %u of %u neighbours",
                     (unsigned)g_admin_neighbours_received,
                     (unsigned)g_admin_neighbours_total);
            lv_label_set_text(lbl_admin_neighbours_status, buf);
            lv_obj_set_style_text_color(lbl_admin_neighbours_status,
                                        lv_palette_main(LV_PALETTE_YELLOW), 0);
        }
        if (btn_admin_neighbours_refresh) {
            lv_obj_clear_state(btn_admin_neighbours_refresh, LV_STATE_DISABLED);
        }
        return;
    }

    if (more_expected) {
        // Update status line with progress and request the next page.
        if (lbl_admin_neighbours_status) {
            char buf[64];
            snprintf(buf, sizeof(buf),
                     "Loading %u of %u...",
                     (unsigned)g_admin_neighbours_received,
                     (unsigned)g_admin_neighbours_total);
            lv_label_set_text(lbl_admin_neighbours_status, buf);
        }
        admin_neighbours_request_next_page();
        return;
    }

    // Done.
    g_admin_neighbours_in_flight = false;
    if (lbl_admin_neighbours_status) {
        char buf[48];
        if (g_admin_neighbours_total == 0) {
            snprintf(buf, sizeof(buf), "No neighbours");
        } else if (g_admin_neighbours_total == 1) {
            snprintf(buf, sizeof(buf), "1 neighbour");
        } else {
            snprintf(buf, sizeof(buf), "%u neighbours",
                     (unsigned)g_admin_neighbours_total);
        }
        lv_label_set_text(lbl_admin_neighbours_status, buf);
        lv_obj_set_style_text_color(lbl_admin_neighbours_status,
                                    lv_palette_main(LV_PALETTE_GREEN), 0);
    }
    if (btn_admin_neighbours_refresh) {
        lv_obj_clear_state(btn_admin_neighbours_refresh, LV_STATE_DISABLED);
    }
}

static void on_admin_neighbours_refresh_tap(lv_event_t *e) {
    admin_neighbours_request_first_page();
}

// Back from Neighbours → Settings menu list. Also clears the
// in-flight flag so any late-arriving pages get dropped by the
// dispatcher rather than appended to a hidden list or kicking off
// more next-page requests. The repeater may still send pages we'd
// asked for; admin_neighbours_dispatch's early-return on
// !g_admin_neighbours_in_flight handles that.
static void on_admin_neighbours_back(lv_event_t *e) {
    g_admin_neighbours_in_flight = false;
    if (scr_admin_settings) lv_screen_load(scr_admin_settings);
}

// Rebuild the rows in obj_admin_settings_list. Called from
// on_admin_menu_settings_tap on every entry so admin vs guest
// visibility is reflected.
//
// Row order is by frequency of use, with most-used at top. Guests
// see only the rows that don't require admin permission:
// Neighbours, Firmware Info.
static void admin_settings_rebuild_list() {
    if (!obj_admin_settings_list) return;

    // Clear existing rows.
    lv_obj_clean(obj_admin_settings_list);

    // (label, admin_only) pairs.
    struct SettingRow {
        const char* name;
        bool admin_only;
    };
    static const SettingRow ROWS[] = {
        { "Neighbours",          false },  // guest-visible
        { "Sync Clock",          true  },
        { "Firmware Info",       false },  // guest-visible
        { "Manage Regions",      true  },
        { "Position",            true  },
        { "Repeat Settings",     true  },
        { "Access Control",      true  },
        { "Admin Password",      true  },
        { "Guest Password",      true  },
        { "Change Identity Key", true  },
    };

    int y = 5;
    for (size_t i = 0; i < sizeof(ROWS) / sizeof(ROWS[0]); i++) {
        if (ROWS[i].admin_only && !g_admin_session_is_admin) continue;

        lv_obj_t* btn = lv_button_create(obj_admin_settings_list);
        lv_obj_set_size(btn, SCREEN_WIDTH - 40, 70);
        lv_obj_set_pos(btn, 10, y);
        lv_obj_set_style_bg_color(btn, lv_color_make(25, 25, 35), 0);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, lv_color_make(50, 50, 60), 0);
        // user_data is the static string literal, used by the tap
        // handler to look up which row was pressed.
        lv_obj_add_event_cb(btn, on_admin_setting_row_tap,
                            LV_EVENT_CLICKED, (void*)ROWS[i].name);

        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text(lbl, ROWS[i].name);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        meck_set_font(lbl, &meck_montserrat_22, 0);
        lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 10, 0);

        lv_obj_t* arrow = lv_label_create(btn);
        lv_label_set_text(arrow, LV_SYMBOL_RIGHT);
        lv_obj_set_style_text_color(arrow,
            lv_palette_main(LV_PALETTE_GREY), 0);
        meck_set_font(arrow, &meck_montserrat_18, 0);
        lv_obj_align(arrow, LV_ALIGN_RIGHT_MID, -10, 0);

        y += 80;
    }
}

static void create_admin_home_screen() {
    scr_admin_home = lv_obj_create(NULL);
    lock_screen_scroll(scr_admin_home);
    lv_obj_set_style_bg_color(scr_admin_home, lv_color_black(), 0);
    // Slot 10 — same as the piece-3 placeholder it replaces.
    screen_attach_clock_battery(scr_admin_home, 10, &meck_montserrat_22, 30);

    // Title — "Admin" only (avoids the camera punch-hole).
    lv_obj_t *title = lv_label_create(scr_admin_home);
    lv_label_set_text(title, "Admin");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    // Persistent banner — green/yellow, full-width, just under the
    // clock/battery header. Updated by admin_home_update_banner on
    // login dispatch. Only present on the home menu list (not on
    // subpages).
    obj_admin_banner = lv_obj_create(scr_admin_home);
    lv_obj_set_size(obj_admin_banner, SCREEN_WIDTH - 40, 60);
    lv_obj_set_pos(obj_admin_banner, 20, 120);
    lv_obj_set_style_bg_color(obj_admin_banner,
                              lv_palette_darken(LV_PALETTE_GREEN, 2), 0);
    lv_obj_set_style_border_width(obj_admin_banner, 0, 0);
    lv_obj_set_style_radius(obj_admin_banner, 8, 0);
    lv_obj_set_style_pad_all(obj_admin_banner, 0, 0);
    lock_screen_scroll(obj_admin_banner);

    lbl_admin_banner = lv_label_create(obj_admin_banner);
    lv_label_set_text(lbl_admin_banner, "Logged in");
    lv_obj_set_style_text_color(lbl_admin_banner, lv_color_white(), 0);
    meck_set_font(lbl_admin_banner, &meck_montserrat_22, 0);
    lv_obj_center(lbl_admin_banner);

    // Menu rows. y=200 below the banner with comfortable spacing.
    // Order: Status, Send Advert (frequent operations), Cmd Line,
    // Settings (less frequent).
    int y = 200;
    create_admin_menu_row(scr_admin_home, "Status",      y, on_admin_menu_status_tap);
    y += 90;
    create_admin_menu_row(scr_admin_home, "Send Advert", y, on_admin_menu_send_advert_tap);
    y += 90;
    create_admin_menu_row(scr_admin_home, "Cmd Line",    y, on_admin_menu_cmd_tap);
    y += 90;
    create_admin_menu_row(scr_admin_home, "Settings",    y, on_admin_menu_settings_tap);

    // Back button — bottom-left. Tears down the session and returns
    // to contact detail.
    lv_obj_t *btn_back = lv_button_create(scr_admin_home);
    lv_obj_set_size(btn_back, 140, 70);
    lv_obj_align(btn_back, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_admin_home_back, LV_EVENT_CLICKED, NULL);

    // Build subpage screens after the home so any references between
    // them are resolved.
    create_admin_status_screen();
    create_admin_send_advert_screen();
    create_admin_cmd_screen();
    create_admin_settings_screen();
    create_admin_setting_placeholder_screen();
    create_admin_fw_info_screen();
    create_admin_neighbours_screen();
}

// ============================================================================
// Room messages screen (Piece A: stub)
// ----------------------------------------------------------------------------
// Stand-in landing screen for room-server contacts after a successful login.
// Has the header (room name, top-right Admin overlay button) and a back
// button. The body is a "Coming soon" placeholder until Piece B wires up
// post storage + rendering and Piece C adds the composer.
//
// Routing into this screen happens from the login dispatch via
// room_messages_screen_enter(). The Admin overlay button is shown only when
// g_admin_session_is_admin is true (admin password used), and tapping it
// loads scr_admin_home (the existing admin menu list).
// ============================================================================

static void on_room_back(lv_event_t *e) {
    printf("Room: back from room messages screen\n");
    meck_admin_clear_session();
    g_admin_ui_state = ADMIN_STATE_IDLE;
    memset(g_admin_pwd_raw, 0, sizeof(g_admin_pwd_raw));
    g_admin_pwd_len = 0;
    g_in_room_mode = false;
    g_active_room_contact_idx = -1;
    if (kb_room_compose) lv_obj_add_flag(kb_room_compose, LV_OBJ_FLAG_HIDDEN);
    if (ta_room_compose) lv_textarea_set_text(ta_room_compose, "");
    if (scr_contact_detail) lv_screen_load(scr_contact_detail);
}

static void on_room_admin_tap(lv_event_t *e) {
    printf("Room: admin overlay tapped\n");
    if (scr_admin_home) lv_screen_load(scr_admin_home);
}

static void room_messages_screen_enter(int contact_idx, const char* contact_name) {
    g_active_room_contact_idx = contact_idx;
    g_in_room_mode = true;
    if (lbl_room_messages_title) {
        lv_label_set_text(lbl_room_messages_title,
                          contact_name && contact_name[0] ? contact_name : "Room");
    }
    if (btn_room_admin) {
        if (g_admin_session_is_admin) {
            lv_obj_clear_flag(btn_room_admin, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(btn_room_admin, LV_OBJ_FLAG_HIDDEN);
        }
    }
    rebuild_room_bubbles(contact_idx);
    if (scr_room_messages) lv_screen_load(scr_room_messages);
}

static void create_room_messages_screen() {
    scr_room_messages = lv_obj_create(NULL);
    lock_screen_scroll(scr_room_messages);
    lv_obj_set_style_bg_color(scr_room_messages, lv_color_black(), 0);
    // Slot 10 — same header slot as scr_admin_home.
    screen_attach_clock_battery(scr_room_messages, 10, &meck_montserrat_22, 30);
    // Move clock to right side (under battery) so long room names don't overlap
    if (lbl_screen_clock[10]) {
        lv_obj_set_style_text_align(lbl_screen_clock[10], LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(lbl_screen_clock[10], LV_ALIGN_TOP_RIGHT, -15, 55);
    }

    // Back button — top-left, mirroring the messages screen so the
    // composer can claim the bottom edge.
    lv_obj_t *btn_back = lv_button_create(scr_room_messages);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t *bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_room_back, LV_EVENT_CLICKED, NULL);

    // Title — room contact name. Populated by room_messages_screen_enter
    // on each entry. Positioned to the right of the back button.
    lbl_room_messages_title = lv_label_create(scr_room_messages);
    lv_label_set_text(lbl_room_messages_title, "Room");
    lv_obj_set_style_text_color(lbl_room_messages_title,
                                lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_set_font(lbl_room_messages_title, &meck_montserrat_24, 0);
    lv_obj_align(lbl_room_messages_title, LV_ALIGN_TOP_LEFT, 120, 30);

    // Top-right Admin overlay button. Visible only when
    // g_admin_session_is_admin is true; toggled in room_messages_screen_enter.
    btn_room_admin = lv_button_create(scr_room_messages);
    lv_obj_set_size(btn_room_admin, 120, 60);
    lv_obj_align(btn_room_admin, LV_ALIGN_TOP_RIGHT, -10, 30);
    lv_obj_set_style_bg_color(btn_room_admin,
        lv_palette_darken(LV_PALETTE_INDIGO, 1), 0);
    lv_obj_set_style_radius(btn_room_admin, 8, 0);
    lv_obj_t *ra_lbl = lv_label_create(btn_room_admin);
    lv_label_set_text(ra_lbl, LV_SYMBOL_SETTINGS " Admin");
    lv_obj_set_style_text_color(ra_lbl, lv_color_white(), 0);
    meck_set_font(ra_lbl, &meck_montserrat_16, 0);
    lv_obj_center(ra_lbl);
    lv_obj_add_event_cb(btn_room_admin, on_room_admin_tap, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(btn_room_admin, LV_OBJ_FLAG_HIDDEN);

    // Bubble scroll container — fills the area between header row and
    // the composer at the bottom. rebuild_room_bubbles populates it on
    // room_messages_screen_enter; meck_relayout_room_compose resizes it
    // when the textarea grows or the keyboard appears.
    obj_room_scroll = lv_obj_create(scr_room_messages);
    lv_obj_set_size(obj_room_scroll, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 180);
    lv_obj_set_pos(obj_room_scroll, 10, 110);
    lv_obj_set_style_bg_color(obj_room_scroll, lv_color_make(10, 10, 15), 0);
    lv_obj_set_style_border_width(obj_room_scroll, 0, 0);
    lv_obj_set_style_radius(obj_room_scroll, 8, 0);
    lv_obj_set_style_pad_all(obj_room_scroll, 10, 0);
    lv_obj_set_scroll_dir(obj_room_scroll, LV_DIR_VER);

    // Composer textarea. Same styling/limits as the DM composer for
    // consistency. The room server in default mode accepts posts from
    // any logged-in user (incl. guests), so the composer is always
    // visible — no gating on g_admin_session_is_admin.
    ta_room_compose = lv_textarea_create(scr_room_messages);
    lv_obj_set_width(ta_room_compose, SCREEN_WIDTH - 100);
    lv_obj_set_height(ta_room_compose, LV_SIZE_CONTENT);
    lv_obj_set_style_min_height(ta_room_compose, 50, 0);
    lv_obj_set_style_max_height(ta_room_compose, 150, 0);
    lv_obj_align(ta_room_compose, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_textarea_set_placeholder_text(ta_room_compose, "Type post...");
    lv_textarea_set_one_line(ta_room_compose, false);
    lv_obj_set_scrollbar_mode(ta_room_compose, LV_SCROLLBAR_MODE_OFF);
    // Cap at MAX_POST_TEXT_LEN-ish so we don't exceed upstream
    // MyMesh::addPost's 151-char ceiling on the wire.
    lv_textarea_set_max_length(ta_room_compose, 150);
    lv_obj_set_style_bg_color(ta_room_compose, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_text_color(ta_room_compose, lv_color_white(), 0);
    meck_set_font(ta_room_compose, &meck_montserrat_16, 0);
    lv_obj_set_style_border_color(ta_room_compose, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_border_width(ta_room_compose, 1, 0);
    lv_obj_set_style_radius(ta_room_compose, 8, 0);
    lv_obj_set_style_border_color(ta_room_compose, lv_color_white(),       LV_PART_CURSOR);
    lv_obj_set_style_border_width(ta_room_compose, 2,                       LV_PART_CURSOR);
    lv_obj_set_style_border_side(ta_room_compose,  LV_BORDER_SIDE_LEFT,     LV_PART_CURSOR);
    lv_obj_set_style_border_opa(ta_room_compose,   LV_OPA_COVER,            LV_PART_CURSOR);
    // Blink the cursor. start_cursor_blink() in LVGL's textarea reads
    // anim_duration on LV_PART_CURSOR: 0 leaves the cursor solid, non-zero
    // drives the on/off animation. Set it here rather than inheriting
    // whatever the theme happens to default to.
    lv_obj_set_style_anim_duration(ta_room_compose, 500, LV_PART_CURSOR);
    lv_obj_add_event_cb(ta_room_compose, on_room_compose_focused,      LV_EVENT_FOCUSED,      NULL);
    lv_obj_add_event_cb(ta_room_compose, on_room_compose_defocused,    LV_EVENT_DEFOCUSED,    NULL);
    lv_obj_add_event_cb(ta_room_compose, on_room_compose_size_changed, LV_EVENT_SIZE_CHANGED, NULL);

    btn_room_send = lv_button_create(scr_room_messages);
    lv_obj_set_size(btn_room_send, 70, 50);
    lv_obj_align(btn_room_send, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_bg_color(btn_room_send, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_radius(btn_room_send, 8, 0);
    lv_obj_t *send_lbl = lv_label_create(btn_room_send);
    lv_label_set_text(send_lbl, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(send_lbl, lv_color_black(), 0);
    meck_set_font(send_lbl, &meck_montserrat_22, 0);
    lv_obj_center(send_lbl);
    lv_obj_add_event_cb(btn_room_send, on_room_send_clicked, LV_EVENT_CLICKED, NULL);

    kb_room_compose = lv_keyboard_create(scr_room_messages);
    meck_style_keyboard(kb_room_compose);
    lv_keyboard_set_textarea(kb_room_compose, ta_room_compose);
    lv_obj_add_flag(kb_room_compose, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(kb_room_compose, on_room_kb_event, LV_EVENT_READY,  NULL);
    lv_obj_add_event_cb(kb_room_compose, on_room_kb_event, LV_EVENT_CANCEL, NULL);
    lv_obj_add_event_cb(kb_room_compose, on_kb_long_press,
                        LV_EVENT_LONG_PRESSED, NULL);
    lv_obj_add_event_cb(kb_room_compose, on_kb_emoji_event,
                        LV_EVENT_PRESSED, NULL);
}

// ============================================================================
// CardKB (M5Stack I2C keyboard) input — direct-write into the active composer
//
// 4B integration: when a message composer is open, CardKB keystrokes are written
// straight into its textarea, and Enter/Esc are replayed as the same
// LV_EVENT_READY / LV_EVENT_CANCEL the on-screen keyboard's OK/close buttons
// emit, so the existing send/cancel logic in on_kb_event / on_room_kb_event runs
// unchanged. Whole-device navigation (arrow keys moving focus across the UI) is
// deferred to 4A and needs an lv_group; here the arrows only move the textarea
// cursor and non-composer keys are ignored.
//
// Runs as an lv_timer, i.e. in the LVGL task context (LVGL is not thread-safe,
// so this must not be polled from a separate task). The I2C read is one
// throttled byte per tick.
// ============================================================================
#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4) && defined(MECK_CARDKB)

static MeckCardKB   g_cardkb;
static lv_timer_t  *g_cardkb_timer = NULL;

#endif // CONFIG_BOARD_TYPE_T_DISPLAY_P4 && MECK_CARDKB

// Shared by the CardKB, BLE-keyboard and P4-keyboard poll paths, so compiled
// if any of them is.
#if (defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4) && defined(MECK_CARDKB)) || MECK_BLE_KEYBOARD_ENABLED || defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)

// The textarea the user is currently typing into belongs to whichever composer
// keyboard is visible. Returns that keyboard, or NULL if no composer is open.
static lv_obj_t *meck_cardkb_active_kb(void) {
    // Active composer = the one whose textarea currently has focus. Keyed on
    // focus rather than on-screen-keyboard visibility, so CardKB routing keeps
    // working after the keyboard is auto-dismissed on the first CardKB key.
    if (ta_compose && lv_obj_has_state(ta_compose, LV_STATE_FOCUSED))
        return kb_compose;
    if (ta_room_compose && lv_obj_has_state(ta_room_compose, LV_STATE_FOCUSED))
        return kb_room_compose;
    return NULL;
}

#endif // (T_DISPLAY_P4 && MECK_CARDKB) || MECK_BLE_KEYBOARD_ENABLED || T_DISPLAY_P4_KEYBOARD

#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4) && defined(MECK_CARDKB)

static void meck_cardkb_poll(lv_timer_t *t) {
    (void)t;
    uint32_t key = g_cardkb.read_key();
    if (key == 0) return;

    lv_obj_t *kb = meck_cardkb_active_kb();
    if (!kb) return;                                // no composer focused
    lv_obj_t *ta = lv_keyboard_get_textarea(kb);
    if (!ta) return;

    // First CardKB keypress dismisses the on-screen keyboard so the message
    // list reclaims its space; the textarea keeps focus so CardKB entry
    // continues. Re-show the touch keyboard by tapping away and back into the
    // field (the existing focus handler brings it up again). Reuses the same
    // hide/relayout the keyboard's own close button uses.
    if (!lv_obj_has_flag(kb, LV_OBJ_FLAG_HIDDEN)) {
        if (kb == kb_compose)           on_compose_defocused(NULL);
        else if (kb == kb_room_compose) on_room_compose_defocused(NULL);
    }

    switch (key) {
        case LV_KEY_ENTER:                          // replay the keyboard OK button
            lv_obj_send_event(kb, LV_EVENT_READY, NULL);
            break;
        case LV_KEY_ESC:                            // replay the keyboard close button
            lv_obj_send_event(kb, LV_EVENT_CANCEL, NULL);
            break;
        case LV_KEY_BACKSPACE:
            lv_textarea_delete_char(ta);
            break;
        case LV_KEY_LEFT:
            lv_textarea_cursor_left(ta);
            break;
        case LV_KEY_RIGHT:
            lv_textarea_cursor_right(ta);
            break;
        // LV_KEY_NEXT / LV_KEY_PREV (Tab and up/down) are deliberately ignored
        // here; whole-UI focus movement comes in 4A.
        default:
            if (key >= 0x20 && key <= 0x7E) {
                char s[2] = { (char)key, 0 };
                lv_textarea_add_text(ta, s);
            }
            break;
    }
}

#endif // CONFIG_BOARD_TYPE_T_DISPLAY_P4 && MECK_CARDKB

// External BLE keyboard -> active composer. Same routing as meck_cardkb_poll;
// reads keys from the C6 BLE central via meck_kbd_read_key() and reuses the
// existing meck_cardkb_active_kb() composer lookup.
#if MECK_BLE_KEYBOARD_ENABLED
static void meck_blekbd_poll(lv_timer_t *t) {
    (void)t;
    uint32_t key = meck_kbd_read_key();
    if (key == 0) return;

    lv_obj_t *kb = meck_cardkb_active_kb();
    if (!kb) return;
    lv_obj_t *ta = lv_keyboard_get_textarea(kb);
    if (!ta) return;

    if (!lv_obj_has_flag(kb, LV_OBJ_FLAG_HIDDEN)) {
        if (kb == kb_compose)           on_compose_defocused(NULL);
        else if (kb == kb_room_compose) on_room_compose_defocused(NULL);
    }

    switch (key) {
        case LV_KEY_ENTER:     lv_obj_send_event(kb, LV_EVENT_READY,  NULL); break;
        case LV_KEY_ESC:       lv_obj_send_event(kb, LV_EVENT_CANCEL, NULL); break;
        case LV_KEY_BACKSPACE: lv_textarea_delete_char(ta);                 break;
        case LV_KEY_LEFT:      lv_textarea_cursor_left(ta);                 break;
        case LV_KEY_RIGHT:     lv_textarea_cursor_right(ta);                break;
        default:
            if (key >= 0x20 && key <= 0x7E) {
                char s[2] = { (char)key, 0 };
                lv_textarea_add_text(ta, s);
            }
            break;
    }
}
#endif // MECK_BLE_KEYBOARD_ENABLED

#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4) && defined(MECK_CARDKB)

// Probe for a CardKB and, if present, start the poll timer. Called from the end
// of meck_ui_init() while the LVGL lock is held.
static void meck_cardkb_init(void) {
    if (g_cardkb.begin()) {
        g_cardkb_timer = lv_timer_create(meck_cardkb_poll, 30, NULL);
        printf("MeckUI: CardKB poll timer started\n");
    } else {
        printf("MeckUI: no CardKB found at 0x%02X\n", CARDKB_IIC_ADDRESS);
    }
}

#endif // CONFIG_BOARD_TYPE_T_DISPLAY_P4 && MECK_CARDKB

// ============================================================================
// T-Display-P4-Keyboard (TCA8418) input -> active composer
//
// Same routing as the CardKB path and reusing its composer lookup. Polled from
// an LVGL timer, so this runs in the LVGL task context and may touch widgets
// directly. MeckP4Keyboard::read_key() already discards key-up events and
// resolves Shift/Caps/Fn, so one call yields one finished character.
// ============================================================================
#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)

static MeckP4Keyboard  g_p4kbd;
static lv_timer_t     *g_p4kbd_timer = NULL;

// Step the home tileview one page left or right, matching what a swipe does.
// Does not wrap: the first tile only allows LV_DIR_RIGHT and the last only
// LV_DIR_LEFT, so stopping at the ends mirrors the gesture behaviour.
static void meck_home_page_step(int delta) {
    if (!g_tileview) return;
    lv_obj_t *act = lv_tileview_get_tile_act(g_tileview);
    int idx = -1;
    for (int i = 0; i < MECK_HOME_PAGE_COUNT; i++) {
        if (g_home_tiles[i] && g_home_tiles[i] == act) { idx = i; break; }
    }
    if (idx < 0) return;
    const int next = idx + delta;
    if ((next < 0) || (next >= MECK_HOME_PAGE_COUNT)) return;
    if (!g_home_tiles[next]) return;
    lv_tileview_set_tile_by_index(g_tileview, (uint32_t)next, 0, LV_ANIM_ON);
}

// Find the focused textarea anywhere beneath `root`, or NULL. Walking the tree
// rather than consulting a fixed list of composer objects means every screen
// carrying a text field is handled -- settings, channel add, region and
// position editors, WiFi, trace and path editors, repeater admin password and
// command entry, the web URL and search bars -- including the Notes editor,
// whose widgets live in another translation unit and are not visible here.
static lv_obj_t *meck_find_focused_textarea(lv_obj_t *root) {
    if (!root) return NULL;
    if (lv_obj_check_type(root, &lv_textarea_class) &&
        lv_obj_has_state(root, LV_STATE_FOCUSED)) {
        return root;
    }
    const uint32_t n = lv_obj_get_child_count(root);
    for (uint32_t i = 0; i < n; i++) {
        lv_obj_t *hit = meck_find_focused_textarea(lv_obj_get_child(root, i));
        if (hit) return hit;
    }
    return NULL;
}

// Find the keyboard bound to `ta`, or NULL if this screen has none. LVGL has no
// reverse lookup from textarea to keyboard, so compare each keyboard's binding.
static lv_obj_t *meck_find_keyboard_for_textarea(lv_obj_t *root, lv_obj_t *ta) {
    if (!root) return NULL;
    if (lv_obj_check_type(root, &lv_keyboard_class) &&
        lv_keyboard_get_textarea(root) == ta) {
        return root;
    }
    const uint32_t n = lv_obj_get_child_count(root);
    for (uint32_t i = 0; i < n; i++) {
        lv_obj_t *hit = meck_find_keyboard_for_textarea(lv_obj_get_child(root, i), ta);
        if (hit) return hit;
    }
    return NULL;
}

static void meck_p4kbd_poll(lv_timer_t *t) {
    (void)t;
    uint32_t key = g_p4kbd.read_key();
    if (key == 0) return;

    // The textarea this key belongs to: whatever is focused on the active
    // screen. Generic by design, so screens built elsewhere are covered too.
    lv_obj_t *ta = meck_find_focused_textarea(lv_screen_active());

    // Type-to-compose: on the message screens a printable key focuses the
    // composer, so typing can begin without tapping into the field first. Only
    // printable keys do this -- arrows, Enter and Esc stay available for moving
    // around the message list without the composer springing open.
    //
    // Sending LV_EVENT_FOCUSED rather than only adding the state matters twice:
    // it runs on_compose_focused, which keeps the on-screen keyboard hidden and
    // relays out the message list, and LVGL's own textarea handler starts the
    // cursor blink on that same event.
    if (!ta && (key >= 0x20) && (key <= 0x7E)) {
        lv_obj_t *scr    = lv_screen_active();
        lv_obj_t *target = NULL;
        if      (scr == scr_messages      && ta_compose)      target = ta_compose;
        else if (scr == scr_room_messages && ta_room_compose) target = ta_room_compose;
        if (target) {
            lv_obj_add_state(target, LV_STATE_FOCUSED);
            lv_obj_send_event(target, LV_EVENT_FOCUSED, NULL);
            ta = target;
        }
    }
    // Home screen: the left and right arrows page the tileview, the same
    // movement a swipe produces. Only when nothing is focused for text entry,
    // so arrows keep their cursor role inside any field.
    if (!ta && (lv_screen_active() == scr_home) &&
        ((key == LV_KEY_LEFT) || (key == LV_KEY_RIGHT))) {
        meck_home_page_step((key == LV_KEY_RIGHT) ? 1 : -1);
        return;
    }

    if (!ta) return;                            // nothing focused to type into

    // The keyboard bound to that textarea, if the screen has one. Enter and Esc
    // are replayed as the events its OK and close buttons emit, so each
    // screen's existing commit/cancel handler runs unchanged. A textarea with
    // no keyboard still takes characters; only Enter and Esc need the object.
    lv_obj_t *kb = meck_find_keyboard_for_textarea(lv_screen_active(), ta);

    switch (key) {
        case LV_KEY_ENTER:     if (kb) lv_obj_send_event(kb, LV_EVENT_READY,  NULL); break;
        case LV_KEY_ESC:       if (kb) lv_obj_send_event(kb, LV_EVENT_CANCEL, NULL); break;
        case LV_KEY_BACKSPACE: lv_textarea_delete_char(ta);                  break;
        case LV_KEY_LEFT:      lv_textarea_cursor_left(ta);                  break;
        case LV_KEY_RIGHT:     lv_textarea_cursor_right(ta);                 break;
        default:
            if (key >= 0x20 && key <= 0x7E) {
                char s[2] = { (char)key, 0 };
                lv_textarea_add_text(ta, s);
            }
            break;
    }
}

// Keyboard backlight slider. Live-applies the duty while dragging (visible
// only while the backlight is on -- the LilyGo key toggles it); persists on
// release, same pattern as the screen brightness slider.
static void on_settings_kb_backlight_slider_event(lv_event_t *e) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    if (!slider_set_kb_backlight) return;

    int pct = (int)lv_slider_get_value(slider_set_kb_backlight);
    if (pct < 5)   pct = 5;
    if (pct > 100) pct = 100;

    g_p4kbd.set_backlight_level_pct((uint8_t)pct);

    if (lbl_set_kb_backlight_pct) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d%%", pct);
        lv_label_set_text(lbl_set_kb_backlight_pct, buf);
    }

    if (lv_event_get_code(e) == LV_EVENT_RELEASED) {
        prefs->kb_backlight_pct = (uint8_t)pct;
        mesh->getDataStore()->savePrefs(*prefs);
        printf("Settings: kb backlight = %d%%\n", pct);
    }
}

// Probe for the keyboard and, if present, start the poll timer. Absence is a
// normal outcome -- the on-screen keyboard then behaves exactly as it does on
// a plain T-Display-P4 build.
static void meck_p4kbd_init(void) {
    if (g_p4kbd.begin()) {
        g_p4kbd_present = true;
        // Apply the persisted backlight level so the LilyGo-key toggle
        // comes on at the chosen brightness (default 25%).
        {
            Meck* mesh = meck_get_instance();
            P4NodePrefs* prefs = mesh ? mesh->getNodePrefs() : NULL;
            uint8_t kpct = (prefs && prefs->kb_backlight_pct)
                               ? prefs->kb_backlight_pct : 25;
            g_p4kbd.set_backlight_level_pct(kpct);
        }
        g_p4kbd_timer = lv_timer_create(meck_p4kbd_poll, 30, NULL);
        printf("MeckUI: P4 keyboard poll timer started\n");
    } else {
        g_p4kbd_present = false;
        printf("MeckUI: no P4 keyboard found, using the on-screen keyboard\n");
    }
}

#endif // CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD

// Build every screen: the home tileview and its pages, all sub-screens, and
// the audio/map/reader modules. Split out of meck_ui_init so the live
// orientation rebuild can recreate them after a teardown. Assumes the caller
// holds lvgl_api_lock and that the desired display rotation is already set, so
// SCREEN_WIDTH/SCREEN_HEIGHT report the right logical dimensions during layout.
static void meck_ui_build_screens() {
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

    g_home_tiles[0] = t_home;
    g_home_tiles[1] = t_recent;
    g_home_tiles[2] = t_radio;
    g_home_tiles[3] = t_advert;
    g_home_tiles[4] = t_gps;
    g_home_tiles[5] = t_battery;
    g_home_tiles[6] = t_shutdown;

    create_page_home(t_home);
    create_page_recent(t_recent);
    create_page_radio(t_radio);
    create_page_advert(t_advert);
    create_page_gps(t_gps);
    create_page_battery(t_battery);
    create_page_shutdown(t_shutdown);

    create_settings_screen();
    create_debug_logs_screen();
    create_settings_contacts_screen();
    create_settings_wifi_screen();
    create_web_screen();
#if MECK_BLE_KEYBOARD_ENABLED
    create_settings_keyboard_screen();
#endif
    create_settings_channels_screen();
    create_channel_detail_screen();
    create_settings_position_screen();
    create_not_implemented_screen();
    create_voice_landing_screen();
    create_voice_inbox_screen();
    create_voice_record_screen();
    voice_inbox_load();
    create_radio_picker_screen();
    create_channel_picker_screen();
    create_messages_screen();
    create_retry_modal();
    create_path_modal();
    create_dm_inbox_screen();
    create_contacts_screen();
    create_contact_detail_screen();
    create_trace_screen();
    create_path_editor_screen();
    create_discover_screen();
    create_admin_login_screen();
    create_admin_home_screen();
    create_room_messages_screen();
    meck_audio_ui_init();
    meck_reader_ui_init();
    meck_notes_ui_init();
    meck_map_ui_init();
}

// Delete every screen and null its global, then tear down the audio/map/reader
// modules. Deleting a screen frees all its child widgets, so the per-screen
// lbl_*/btn_*/ta_*/kb_* globals are freed here and reassigned when
// meck_ui_build_screens() recreates each screen. g_tileview is a child of
// scr_home and dies with it. Screens that were never created (lazily-built
// admin sub-tabs the user never opened) are NULL and skipped — they rebuild on
// next navigation at the current orientation.
static void meck_ui_teardown_screens() {
    lv_obj_t** screens[] = {
        &scr_home, &scr_settings, &scr_debug_logs, &scr_not_implemented,
        &scr_settings_wifi, &scr_settings_channels, &scr_channel_detail,
        &scr_settings_position, &scr_voice_landing, &scr_voice_inbox, &scr_voice,
        &scr_radio_picker, &scr_channel_picker, &scr_dm_inbox, &scr_messages,
        &scr_contacts, &scr_contact_detail, &scr_trace, &scr_path_editor,
        &scr_settings_contacts, &scr_discover, &scr_admin_login, &scr_admin_home,
        &scr_room_messages, &scr_admin_status, &scr_admin_cmd, &scr_admin_settings,
        &scr_admin_send_advert, &scr_admin_setting_placeholder, &scr_admin_fw_info,
        &scr_admin_neighbours,
    };
    for (size_t i = 0; i < sizeof(screens) / sizeof(screens[0]); i++) {
        if (*screens[i]) { lv_obj_delete(*screens[i]); *screens[i] = NULL; }
    }
    g_tileview = NULL;   // was a child of scr_home, already freed above
    for (int i = 0; i < MECK_HOME_PAGE_COUNT; i++) g_home_tiles[i] = NULL;

    meck_audio_ui_teardown();
    meck_reader_ui_teardown();
    meck_notes_ui_teardown();
    meck_map_ui_teardown();
}

// Live screen-orientation rebuild. Invoked via lv_async_call from the Settings
// toggle so it runs between input events (not inside the toggle's own click
// handler, which would delete scr_settings out from under its event) and while
// the LVGL lock is held, so no timer interleaves with the teardown/rebuild.
// Parks on a throwaway blank screen so the screen being deleted is never the
// active one, switches rotation, rebuilds, and loads home.
static void meck_ui_apply_orientation_rebuild(void *unused) {
    (void)unused;
    Meck* mesh = meck_get_instance();
    P4NodePrefs* prefs = mesh ? mesh->getNodePrefs() : nullptr;

    lv_obj_t* blank = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(blank, lv_color_black(), 0);
    lv_screen_load(blank);

    meck_ui_teardown_screens();

    lv_display_set_rotation(lv_display_get_default(),
        (prefs && prefs->orientation == 1)
            ? LV_DISPLAY_ROTATION_90
            : LV_DISPLAY_ROTATION_0);

    meck_ui_build_screens();

    // The rebuilt clock/battery labels start empty. Reset their refresh
    // counters to the period so the next ui_update_timer_cb tick (<=500ms)
    // refills them, rather than leaving them blank until the next wrap (up to
    // 1 min for the clock, 5 min for the battery).
    clock_ticks   = 120;
    battery_ticks = 600;

    lv_screen_load(scr_home);
    lv_obj_delete(blank);

    printf("MeckUI: rebuilt for %s\n",
        (prefs && prefs->orientation == 1) ? "landscape" : "portrait");
}

extern "C" void meck_ui_init() {
    // Apply the saved screen orientation before any screen is built, so the
    // layout reads the correct logical dimensions from the start. 0 =
    // portrait (default), 1 = landscape. The Settings > Orientation toggle
    // saves this pref and rebuilds live; this read is the boot-time apply.
    {
        Meck* mesh = meck_get_instance();
        P4NodePrefs* prefs = mesh ? mesh->getNodePrefs() : nullptr;
        lv_display_set_rotation(lv_display_get_default(),
            (prefs && prefs->orientation == 1)
                ? LV_DISPLAY_ROTATION_90
                : LV_DISPLAY_ROTATION_0);
    }

    printf("MeckUI: building home screen\n");

    // Silence the LEDC fade-install error spam. cpp_bus_driver re-installs
    // the LEDC fade ISR on every set_rm69a10_brightness call; ESP-IDF
    // returns ESP_ERR_INVALID_STATE which the driver ignores (the actual
    // brightness write goes via panel register 0x51, not LEDC). Logging
    // the error each tap is just noise.
    esp_log_level_set("ledc", ESP_LOG_NONE);

    // Route ESP_LOGx output through the debug-log hook so it is captured to
    // the SD session log when logging is active (ESP_LOGx bypasses printf).
    esp_log_set_vprintf(meck_debug_log_vprintf);

    _lock_acquire(&lvgl_api_lock);

    meck_emoji_init();

    meck_ui_build_screens();

    lv_timer_create(ui_update_timer_cb, 500, NULL);

    // Apply user's saved brightness now that prefs and the panel are both
    // up. Sentinel value 0 means "not set" — fall back to 200 so the screen
    // doesn't go black on first boot of an upgraded device.
    {
        Meck* mesh = meck_get_instance();
        if (mesh) {
            P4NodePrefs* prefs = mesh->getNodePrefs();
            if (prefs) {
                uint8_t b = prefs->screen_brightness
                    ? prefs->screen_brightness : 200;
                meck_screen_set_brightness(b);

                // Apply saved GPS gate. 0 = unset (default on), 1 = on,
                // 2 = off. Avoids surprising users whose existing prefs
                // have reserved[] = 0.
                if (prefs->gps_enabled == 2) {
                    meck_gps_set_enabled(false);
                }
            }
        }
    }
    // 4 Hz idle watcher for auto screen-off. The timer is always running
    // — the handler short-circuits when screen_off_minutes is 0, so the
    // cost is negligible. 250 ms period is short enough that pressing the
    // boot button to wake feels instant; the body is just a register read
    // and a couple of compares so the higher rate doesn't matter.
    meck_boot_button_init();
    lv_timer_create(screen_idle_timer_cb, 250, NULL);

    // DM receive dispatcher. Registered after the screens exist so the
    // first incoming DM doesn't fire before rebuild_dm_bubbles can touch
    // its widgets. The bridge in meck_app.cpp gates on this being non-null
    // — pre-registration, queued DMs sit in Meck's ring with a serial log
    // so they're not silently lost.
    meck_register_dm_recv_callback(meck_dm_recv_dispatch);
    meck_register_dm_sent_callback(meck_dm_sent_dispatch);
    meck_register_post_recv_callback(meck_post_recv_dispatch);

    // Repeater admin callbacks. Login + send-result wired in piece 3,
    // status + CLI dispatch wired in piece 4. Telemetry callback lands
    // in piece 5 when the screen that consumes it exists.
    meck_register_admin_send_result_callback(meck_admin_send_result_dispatch);
    meck_register_admin_login_callback(meck_admin_login_dispatch);
    meck_register_admin_status_callback(meck_admin_status_dispatch);
    meck_register_admin_cli_callback(meck_admin_cli_dispatch);
    meck_register_admin_neighbours_callback(admin_neighbours_dispatch);

    // Hydrate per-contact DM rings from /sdcard/meshcore/dms.bin. Runs
    // after the mesh + contacts are loaded so the pub_key prefix demux
    // can find matching contacts. Must run BEFORE the LVGL task starts
    // dispatching events so partial state isn't visible to a user
    // already tapping through to the inbox.
    load_persisted_dms_into_rings();
    // Same for room posts (Piece B) — hydrate per-room rings from
    // /sdcard/meshcore/posts.bin before any LVGL events can reach
    // room_messages_screen_enter.
    load_persisted_posts_into_rings();

#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4) && defined(MECK_CARDKB)
    meck_cardkb_init();
#endif

#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)
    meck_p4kbd_init();
#endif

    // External BLE keyboard: route decoded keys into the active composer. Cheap
    // (just drains the C6 keyboard's key ring); always running.
#if MECK_BLE_KEYBOARD_ENABLED
    lv_timer_create(meck_blekbd_poll, 30, NULL);
#endif

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

/* Same as meck_ui_show_home() but does NOT acquire lvgl_api_lock. For
 * callers that are already running inside the LVGL task (event handlers,
 * timers, async callbacks) — the LVGL task already holds the lock for the
 * full duration of its iteration, so re-acquiring deadlocks silently.
 *
 * Use meck_ui_show_home() from non-LVGL threads (boot, BLE callbacks,
 * radio task, etc.). Use meck_ui_load_home_screen() from inside LVGL
 * event handlers — this matches how goto_settings_from_contacts and
 * the other inter-screen handlers in this file work. */
extern "C" void meck_ui_load_home_screen() {
    if (!scr_home) {
        printf("MeckUI: home screen not initialized, call meck_ui_init() first\n");
        return;
    }
    lv_screen_load(scr_home);
    printf("MeckUI: home screen loaded (unlocked)\n");
}
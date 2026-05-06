/*
 * MeckUI.cpp — LVGL UI for Meck on T-Display P4
 *
 * Builds the 7-tile horizontal tileview ported from the old meshcore_test.cpp.
 * Currently implemented:
 *   - Tile 0 (Home) fully populated with header + 3x2 navigation grid
 *   - Settings sub-screen (read-only) reachable from the Settings tile
 *   - Back button helper used by Settings, will be reused by future screens
 *
 * Tiles 1-6 (Recent, Radio Details, Advert, GPS, Battery, Hibernate) are
 * still title-only stubs; later turns fill them in.
 *
 * Threading
 * ---------
 * LilyGo runs LVGL on `lvgl_ui_task`. We coordinate with their task via
 * their lock (`_lock_t lvgl_api_lock`, declared non-static in main.cpp).
 *
 * The 500ms refresh timer runs inside lvgl_ui_task's own context (lv_timers
 * are dispatched by lv_timer_handler, called from that task), so the timer
 * callback does NOT take the lock.
 */

#include "MeckUI.h"
#include "meck.h"
#include "target.h"
#include "MeckMesh.h"
#include "MeckDataStore.h"

#include <sys/lock.h>
#include "lvgl.h"
#include "t_display_p4_driver.h"

#include <cstdio>
#include <cstring>

// LilyGo's LVGL mutex (declared non-static in main/examples/lvgl_9_ui/main.cpp:253)
extern _lock_t lvgl_api_lock;

// Notch-safe padding (camera at top center of the display)
#define NOTCH_SAFE_X  20
#define NOTCH_SAFE_Y  20

// ============================================================================
// LVGL objects (file-scope so the refresh timer can update them)
// ============================================================================

static lv_obj_t *scr_home          = NULL;
static lv_obj_t *g_tileview        = NULL;

// Header labels on the home tile (refreshed every 500ms)
static lv_obj_t *lbl_home_packets  = NULL;
static lv_obj_t *lbl_home_rssi     = NULL;
static lv_obj_t *lbl_home_unread   = NULL;

// Settings screen
static lv_obj_t *scr_settings      = NULL;
static lv_obj_t *lbl_set_name      = NULL;
static lv_obj_t *lbl_set_radio     = NULL;
static lv_obj_t *lbl_set_txpower   = NULL;
static lv_obj_t *lbl_set_utc       = NULL;
static lv_obj_t *lbl_set_identity  = NULL;

// ============================================================================
// Forward declarations for navigation callbacks
// ============================================================================

static void goto_home(lv_event_t *e);
static void goto_settings(lv_event_t *e);

// ============================================================================
// Tab-button callbacks (stubs — real screens land in later turns)
// ============================================================================

static void cb_todo_messages(lv_event_t* e)  { printf("MeckUI: Messages tile clicked (TODO)\n"); }
static void cb_todo_contacts(lv_event_t* e)  { printf("MeckUI: Contacts tile clicked (TODO)\n"); }
static void cb_todo_reader(lv_event_t* e)    { printf("MeckUI: Reader tile clicked (TODO)\n"); }
static void cb_todo_notes(lv_event_t* e)     { printf("MeckUI: Notes tile clicked (TODO)\n"); }
static void cb_todo_discover(lv_event_t* e)  { printf("MeckUI: Discover tile clicked (TODO)\n"); }

// ============================================================================
// Helper: back button (left-aligned, clear of camera notch)
// Used by every sub-screen. Always navigates back to the home screen.
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

// ============================================================================
// Helper: tile button on the 3x2 home grid
// ============================================================================

static lv_obj_t* create_tile_button(lv_obj_t *parent, const char *label,
                                    lv_event_cb_t cb, int col, int row) {
    int tileW = (SCREEN_WIDTH - 60) / 3;
    int tileH = 100;
    int gapX  = 10;
    int gapY  = 10;
    int gridX = 20;
    int gridY = 140;

    int x = gridX + col * (tileW + gapX);
    int y = gridY + row * (tileH + gapY);

    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_size(btn, tileW, tileH);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_style_bg_color(btn, lv_color_make(30, 30, 40), 0);
    lv_obj_set_style_bg_color(btn, lv_color_make(50, 50, 70), LV_STATE_PRESSED);
    lv_obj_set_style_radius(btn, 12, 0);
    lv_obj_set_style_border_color(btn, lv_color_make(80, 80, 100), 0);
    lv_obj_set_style_border_width(btn, 1, 0);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, label);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
    lv_obj_center(lbl);

    if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    return btn;
}

// ============================================================================
// Helper: settings row (label above, value below, on a rounded panel)
// Returns the value label so the caller can set its text.
// ============================================================================

static lv_obj_t* create_settings_row(lv_obj_t *parent, const char *label, int y) {
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_size(row, SCREEN_WIDTH - 40, 60);
    lv_obj_set_pos(row, 20, y);
    lv_obj_set_style_bg_color(row, lv_color_make(15, 15, 20), 0);
    lv_obj_set_style_radius(row, 10, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 10, 0);

    lv_obj_t *title_lbl = lv_label_create(row);
    lv_label_set_text(title_lbl, label);
    lv_obj_set_style_text_color(title_lbl, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(title_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(title_lbl, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *value_lbl = lv_label_create(row);
    lv_label_set_text(value_lbl, "");
    lv_obj_set_style_text_color(value_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(value_lbl, &lv_font_montserrat_18, 0);
    lv_obj_align(value_lbl, LV_ALIGN_TOP_LEFT, 0, 22);

    return value_lbl;
}

// ============================================================================
// Tile 0: Home — title, header, navigation grid
// ============================================================================

static void create_page_home(lv_obj_t *page) {
    Meck* mesh = meck_get_instance();
    P4NodePrefs* prefs = mesh ? mesh->getNodePrefs() : nullptr;

    // Title (left of camera notch)
    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, "Meck P4");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    // Radio config (right of camera notch)
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

    // Unread message count (left, under title)
    lbl_home_unread = lv_label_create(page);
    lv_label_set_text(lbl_home_unread, "MSG: 0");
    lv_obj_set_style_text_color(lbl_home_unread, lv_color_make(0, 255, 100), 0);
    lv_obj_set_style_text_font(lbl_home_unread, &lv_font_montserrat_18, 0);
    lv_obj_align(lbl_home_unread, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, 60);

    // Packet counter (right)
    lbl_home_packets = lv_label_create(page);
    lv_label_set_text(lbl_home_packets, "RX: 0");
    lv_obj_set_style_text_color(lbl_home_packets, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(lbl_home_packets, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_home_packets, LV_ALIGN_TOP_RIGHT, -10, 60);

    // Last RSSI/SNR (right, below packet counter)
    lbl_home_rssi = lv_label_create(page);
    lv_label_set_text(lbl_home_rssi, "");
    lv_obj_set_style_text_color(lbl_home_rssi, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(lbl_home_rssi, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_home_rssi, LV_ALIGN_TOP_RIGHT, -10, 80);

    // 3x2 navigation tile grid
    create_tile_button(page, LV_SYMBOL_ENVELOPE "\nMessages", cb_todo_messages, 0, 0);
    create_tile_button(page, LV_SYMBOL_LIST "\nContacts",     cb_todo_contacts, 1, 0);
    create_tile_button(page, LV_SYMBOL_SETTINGS "\nSettings", goto_settings,    2, 0);
    create_tile_button(page, LV_SYMBOL_FILE "\nReader",       cb_todo_reader,   0, 1);
    create_tile_button(page, LV_SYMBOL_EDIT "\nNotes",        cb_todo_notes,    1, 1);
    create_tile_button(page, LV_SYMBOL_GPS "\nDiscover",      cb_todo_discover, 2, 1);

    // Swipe hint at bottom
    lv_obj_t *hint = lv_label_create(page);
    lv_label_set_text(hint, "Swipe left for more pages " LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(hint, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -20);
}

// ============================================================================
// Tiles 1-6: stubs (replaced by real content in later turns)
// ============================================================================

static void create_page_stub(lv_obj_t *page, const char *title_text, lv_color_t color) {
    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, title_text);
    lv_obj_set_style_text_color(title, color, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    lv_obj_t *body = lv_label_create(page);
    lv_label_set_text(body, "(coming soon)");
    lv_obj_set_style_text_color(body, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(body, &lv_font_montserrat_16, 0);
    lv_obj_align(body, LV_ALIGN_CENTER, 0, 0);
}

// ============================================================================
// Settings screen (read-only — editing comes in a later turn with a virtual
// keyboard). Reached from the Settings tile on the home screen.
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

    // Scrollable area below the header
    lv_obj_t *scroll = lv_obj_create(scr_settings);
    lv_obj_set_size(scroll, SCREEN_WIDTH, SCREEN_HEIGHT - 60);
    lv_obj_set_pos(scroll, 0, 60);
    lv_obj_set_style_bg_opa(scroll, 0, 0);
    lv_obj_set_style_border_width(scroll, 0, 0);
    lv_obj_set_style_pad_all(scroll, 0, 0);
    lv_obj_set_scroll_dir(scroll, LV_DIR_VER);

    Meck* mesh = meck_get_instance();
    P4NodePrefs* prefs = mesh ? mesh->getNodePrefs() : nullptr;

    int y = 5;

    // Node name
    lbl_set_name = create_settings_row(scroll, "Node Name", y);
    lv_label_set_text(lbl_set_name, prefs ? prefs->node_name : "(no prefs)");
    y += 70;

    // Radio config
    lbl_set_radio = create_settings_row(scroll, "Radio", y);
    if (prefs) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%.3f MHz / BW%.0f / SF%d / CR4/%d",
                 prefs->freq, prefs->bw, prefs->sf, prefs->cr);
        lv_label_set_text(lbl_set_radio, buf);
    } else {
        lv_label_set_text(lbl_set_radio, "(no prefs)");
    }
    y += 70;

    // TX power
    lbl_set_txpower = create_settings_row(scroll, "TX Power", y);
    if (prefs) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%d dBm", (int)prefs->tx_power_dbm);
        lv_label_set_text(lbl_set_txpower, buf);
    } else {
        lv_label_set_text(lbl_set_txpower, "(no prefs)");
    }
    y += 70;

    // UTC offset
    lbl_set_utc = create_settings_row(scroll, "UTC Offset", y);
    if (prefs) {
        char buf[16];
        snprintf(buf, sizeof(buf), "UTC%+d", (int)prefs->utc_offset_hours);
        lv_label_set_text(lbl_set_utc, buf);
    } else {
        lv_label_set_text(lbl_set_utc, "(no prefs)");
    }
    y += 70;

    // Identity (full pubkey, formatted as four hex groups)
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
}

// ============================================================================
// Navigation callbacks
// ============================================================================

static void goto_home(lv_event_t *e) {
    if (scr_home) lv_screen_load(scr_home);
}

static void goto_settings(lv_event_t *e) {
    if (scr_settings) lv_screen_load(scr_settings);
}

// ============================================================================
// 500ms refresh timer (runs inside lvgl_ui_task — no explicit locking)
// ============================================================================

static void ui_update_timer_cb(lv_timer_t *t) {
    Meck* mesh = meck_get_instance();

    // Unread message badge
    if (lbl_home_unread && mesh) {
        int unread = mesh->getUnreadCount();
        lv_label_set_text_fmt(lbl_home_unread, "MSG: %d", unread);
        lv_obj_set_style_text_color(lbl_home_unread,
            unread > 0 ? lv_color_make(0, 255, 100)
                       : lv_palette_main(LV_PALETTE_GREY), 0);
    }

    // Packet counter
    if (lbl_home_packets) {
        uint32_t rx = radio_driver.getPacketsRecv();
        lv_label_set_text_fmt(lbl_home_packets, "RX: %lu", (unsigned long)rx);
    }

    // Last RSSI / SNR (only once we've heard something)
    if (lbl_home_rssi && radio_driver.getPacketsRecv() > 0) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%.0f dBm / %.1f dB",
                 radio_driver.getLastRSSI(), radio_driver.getLastSNR());
        lv_label_set_text(lbl_home_rssi, buf);
    }
}

// ============================================================================
// Public API
// ============================================================================

extern "C" void meck_ui_init() {
    printf("MeckUI: building home screen\n");

    _lock_acquire(&lvgl_api_lock);

    // ---- Home screen with 7-tile horizontal tileview ----
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
    create_page_stub(t_recent,   "Recent Heard",  lv_palette_main(LV_PALETTE_GREEN));
    create_page_stub(t_radio,    "Radio Details", lv_palette_main(LV_PALETTE_YELLOW));
    create_page_stub(t_advert,   "Advert",        lv_palette_main(LV_PALETTE_GREEN));
    create_page_stub(t_gps,      "GPS",           lv_palette_main(LV_PALETTE_GREEN));
    create_page_stub(t_battery,  "Battery Gauge", lv_palette_main(LV_PALETTE_GREEN));
    create_page_stub(t_shutdown, "Hibernate",     lv_palette_main(LV_PALETTE_RED));

    // ---- Sub-screens ----
    create_settings_screen();

    // ---- Periodic refresh ----
    lv_timer_create(ui_update_timer_cb, 500, NULL);

    _lock_release(&lvgl_api_lock);

    printf("MeckUI: home screen + settings ready\n");
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

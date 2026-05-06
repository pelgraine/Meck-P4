/*
 * MeckUI.cpp — LVGL UI for Meck on T-Display P4
 *
 * 7-tile horizontal tileview ported from old meshcore_test.cpp:
 *   - Tile 0: Home (title, freq header, MSG/RX/RSSI counters, 3x2 nav grid)
 *   - Tile 1: Recent Heard (live list of received adverts)
 *   - Tile 2: Radio Details (current freq/BW/SF/CR/TX/sync word)
 *   - Tile 3: Advert (long-press to send manual advert)
 *   - Tile 4: GPS (status + fix info, awaiting GPS hookup)
 *   - Tile 5: Battery Gauge (placeholder, awaiting board accessor)
 *   - Tile 6: Hibernate (long-press handler comes in B.3)
 *
 * Sub-screens reached from home grid:
 *   - Settings (editable: name, preset, TX power, UTC; identity read-only)
 *   - Radio Preset picker (17 community presets, selecting reconfigures radio)
 *
 * Threading
 * ---------
 * LilyGo runs LVGL on lvgl_ui_task. We coordinate via their lock
 * (_lock_t lvgl_api_lock, declared non-static in main.cpp).
 *
 * The 500ms refresh timer runs inside lvgl_ui_task's own context
 * (lv_timers are dispatched from lv_timer_handler in that task), so
 * the timer callback does NOT take the lock.
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
#include <cstdint>
#include <cstdlib>

extern _lock_t lvgl_api_lock;

#define NOTCH_SAFE_X  20
#define NOTCH_SAFE_Y  20

// ============================================================================
// Radio preset table (17 community presets, current as of upstream Meshcore App)
// TX powers: 22 dBm for AU/NZ/USA/VN (915-923 MHz, 22 dBm allowed),
//            14 dBm for EU/UK/CZ/CH/PT868 (868/869 MHz, ETSI 14 dBm cap),
//            10 dBm for 433 MHz ISM (typical conservative default)
// Australia (Deprecated 915.8/SF10/BW250) intentionally excluded - unused.
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
    { "Australia (Narrow)",      916.575f,  62.5f,  7, 8, 22 },
    { "Australia (Mid)",         915.075f, 125.0f,  9, 5, 22 },
    { "Australia: SA, WA",       923.125f,  62.5f,  8, 8, 22 },
    { "Australia: QLD",          923.125f,  62.5f,  8, 5, 22 },
    { "EU/UK (Narrow)",          869.618f,  62.5f,  8, 8, 14 },
    { "EU/UK (Deprecated)",      869.525f, 250.0f, 11, 5, 14 },
    { "Czech Republic (Narrow)", 869.432f,  62.5f,  7, 5, 14 },
    { "EU 433MHz (Long Range)",  433.650f, 250.0f, 11, 5, 10 },
    { "EU 433MHz (Narrow)",      433.650f,  62.5f,  8, 8, 10 },
    { "New Zealand",             917.375f, 250.0f, 11, 5, 22 },
    { "New Zealand (Narrow)",    917.375f,  62.5f,  7, 5, 22 },
    { "Portugal 433",            433.375f,  62.5f,  9, 6, 10 },
    { "Portugal 868",            869.618f,  62.5f,  7, 6, 14 },
    { "Switzerland",             869.618f,  62.5f,  8, 8, 14 },
    { "USA/Canada (Recommended)", 910.525f, 62.5f,  7, 5, 22 },
    { "Vietnam (Narrow)",        920.250f,  62.5f,  8, 5, 22 },
    { "Vietnam (Deprecated)",    920.250f, 250.0f, 11, 5, 22 },
};
#define NUM_RADIO_PRESETS (sizeof(RADIO_PRESETS) / sizeof(RADIO_PRESETS[0]))

static const uint8_t TX_POWER_OPTIONS[] = { 10, 14, 17, 20, 22 };
#define NUM_TX_POWER_OPTIONS 5

// ============================================================================
// LVGL objects (file-scope so the refresh timer can update them)
// ============================================================================

static lv_obj_t *scr_home          = NULL;
static lv_obj_t *g_tileview        = NULL;

// Home tile header labels (refreshed every 500ms)
static lv_obj_t *lbl_home_packets  = NULL;
static lv_obj_t *lbl_home_rssi     = NULL;
static lv_obj_t *lbl_home_unread   = NULL;

// Detail tile labels
static lv_obj_t *lbl_recent_list   = NULL;
static lv_obj_t *lbl_radio_detail  = NULL;
static lv_obj_t *lbl_advert_status = NULL;
static lv_obj_t *lbl_gps_detail    = NULL;
static lv_obj_t *lbl_battery_detail = NULL;

// Settings screen
static lv_obj_t *scr_settings      = NULL;
static lv_obj_t *lbl_set_name      = NULL;
static lv_obj_t *lbl_set_radio     = NULL;
static lv_obj_t *lbl_set_txpower   = NULL;
static lv_obj_t *lbl_set_utc       = NULL;
static lv_obj_t *lbl_set_identity  = NULL;

// Name-edit overlay
static lv_obj_t *obj_name_edit_panel = NULL;
static lv_obj_t *ta_settings_name    = NULL;
static lv_obj_t *kb_settings         = NULL;

// Radio preset picker
static lv_obj_t *scr_radio_picker  = NULL;

// ============================================================================
// Forward declarations
// ============================================================================

static void goto_home(lv_event_t *e);
static void goto_settings(lv_event_t *e);
static void settings_update_labels();
static void update_radio_detail_label();

static void on_settings_name_tap(lv_event_t *e);
static void on_settings_name_save(lv_event_t *e);
static void on_settings_name_cancel(lv_event_t *e);
static void on_settings_kb_event(lv_event_t *e);
static void on_settings_radio_tap(lv_event_t *e);
static void on_settings_txpower_tap(lv_event_t *e);
static void on_settings_utc_tap(lv_event_t *e);
static void on_radio_preset_select(lv_event_t *e);

// ============================================================================
// Home grid tile callbacks (Messages/Contacts/Reader/Notes/Discover still TODO,
// Settings wired to goto_settings)
// ============================================================================

static void cb_todo_messages(lv_event_t* e) { printf("MeckUI: Messages tile clicked (TODO)\n"); }
static void cb_todo_contacts(lv_event_t* e) { printf("MeckUI: Contacts tile clicked (TODO)\n"); }
static void cb_todo_reader(lv_event_t* e)   { printf("MeckUI: Reader tile clicked (TODO)\n"); }
static void cb_todo_notes(lv_event_t* e)    { printf("MeckUI: Notes tile clicked (TODO)\n"); }
static void cb_todo_discover(lv_event_t* e) { printf("MeckUI: Discover tile clicked (TODO)\n"); }

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
// settings_update_labels - re-render settings row values from current prefs
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
        char buf[64];
        snprintf(buf, sizeof(buf), "%.3f MHz / BW%.0f / SF%d / CR4/%d",
                 prefs->freq, prefs->bw, prefs->sf, prefs->cr);
        lv_label_set_text(lbl_set_radio, buf);
    }
    if (lbl_set_txpower) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%d dBm", (int)prefs->tx_power_dbm);
        lv_label_set_text(lbl_set_txpower, buf);
    }
    if (lbl_set_utc) {
        char buf[16];
        snprintf(buf, sizeof(buf), "UTC%+d", (int)prefs->utc_offset_hours);
        lv_label_set_text(lbl_set_utc, buf);
    }
}

// ============================================================================
// update_radio_detail_label - re-render the Radio Details tile from prefs
// (Called at creation and after any radio config change.)
// ============================================================================

static void update_radio_detail_label() {
    if (!lbl_radio_detail) return;
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    char buf[256];
    snprintf(buf, sizeof(buf),
        "Frequency:     %.3f MHz\n"
        "Bandwidth:     %.1f kHz\n"
        "Spread Factor: SF%d\n"
        "Coding Rate:   4/%d\n"
        "TX Power:      %d dBm\n"
        "Sync Word:     0x1424",
        prefs->freq, prefs->bw, prefs->sf, prefs->cr,
        (int)prefs->tx_power_dbm);
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

    // Defer radio reconfig to meck_task to avoid SPI race with mesh.loop()
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

    // Defer radio reconfig to meck_task to avoid SPI race with mesh.loop()
    radio_request_reconfig(prefs->freq, prefs->bw, prefs->sf, prefs->cr, prefs->tx_power_dbm);

    printf("Settings: TX power = %d dBm\n", prefs->tx_power_dbm);
    settings_update_labels();
    update_radio_detail_label();
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

// ============================================================================
// Tile 0: Home
// ============================================================================

static void create_page_home(lv_obj_t *page) {
    Meck* mesh = meck_get_instance();
    P4NodePrefs* prefs = mesh ? mesh->getNodePrefs() : nullptr;

    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, "Meck P4");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

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

    create_tile_button(page, LV_SYMBOL_ENVELOPE "\nMessages", cb_todo_messages, 0, 0);
    create_tile_button(page, LV_SYMBOL_LIST "\nContacts",     cb_todo_contacts, 1, 0);
    create_tile_button(page, LV_SYMBOL_SETTINGS "\nSettings", goto_settings,    2, 0);
    create_tile_button(page, LV_SYMBOL_FILE "\nReader",       cb_todo_reader,   0, 1);
    create_tile_button(page, LV_SYMBOL_EDIT "\nNotes",        cb_todo_notes,    1, 1);
    create_tile_button(page, LV_SYMBOL_GPS "\nDiscover",      cb_todo_discover, 2, 1);

    lv_obj_t *hint = lv_label_create(page);
    lv_label_set_text(hint, "Swipe left for more pages " LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(hint, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -20);
}

// ============================================================================
// Tile 1: Recent Heard (lifted from old:537)
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
// Tile 2: Radio Details (lifted from old:556)
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
// Tile 3: Advert (lifted from old:595, long-press sends manual advert)
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
// Tile 4: GPS (lifted from old:623, static text until GPS data hookup)
// ============================================================================

static void create_page_gps(lv_obj_t *page) {
    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, "GPS");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    lbl_gps_detail = lv_label_create(page);
    // L76K is initialized by LilyGo's device_gps_task but we don't yet have
    // an accessor wired up. Showing static "awaiting hookup" until next turn.
    lv_label_set_text(lbl_gps_detail,
        "Status:     L76K detected\n"
        "Fix:        --\n"
        "Satellites: --\n"
        "Position:   --\n"
        "Altitude:   --\n\n"
        "(GPS data hookup pending)");
    lv_obj_set_style_text_color(lbl_gps_detail, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_gps_detail, &lv_font_montserrat_18, 0);
    lv_label_set_long_mode(lbl_gps_detail, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_gps_detail, SCREEN_WIDTH - 40);
    lv_obj_align(lbl_gps_detail, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, 60);
}

// ============================================================================
// Tile 5: Battery (lifted from old:647, readout pending board accessor wiring)
// ============================================================================

static void create_page_battery(lv_obj_t *page) {
    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, "Battery Gauge");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);

    lbl_battery_detail = lv_label_create(page);
    // BQ27220 is initialized by LilyGo (boot log shows "bq27220 init success")
    // but the readout accessor (board.getBattMilliVolts() in old code) was a
    // file-scope singleton in meshcore_test.cpp. In lvgl_9_ui the chip is
    // owned by LilyGo's main.cpp - the right extern needs identifying.
    lv_label_set_text(lbl_battery_detail,
        "BQ27220:    detected\n"
        "Voltage:    --\n"
        "Percent:    --\n"
        "Time empty: --\n"
        "Avg current: --\n"
        "Avg power:  --\n"
        "Remaining:  --\n"
        "Design cap: --\n"
        "Temp:       --\n\n"
        "(readout pending - see TODO in MeckUI.cpp)");
    lv_obj_set_style_text_color(lbl_battery_detail, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_battery_detail, &lv_font_montserrat_18, 0);
    lv_label_set_long_mode(lbl_battery_detail, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_battery_detail, SCREEN_WIDTH - 40);
    lv_obj_align(lbl_battery_detail, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, 60);
}

// ============================================================================
// Tile 6: Hibernate (lifted from old:666, long-press handler comes in B.3)
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
    create_settings_row(scroll, "UTC Offset (tap to cycle)", &lbl_set_utc,     on_settings_utc_tap,     y);
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

    // Name edit overlay (hidden by default)
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
// Navigation callbacks
// ============================================================================

static void goto_home(lv_event_t *e) {
    if (scr_home) lv_screen_load(scr_home);
}

static void goto_settings(lv_event_t *e) {
    if (scr_settings) lv_screen_load(scr_settings);
}

// ============================================================================
// 500ms refresh timer
// ============================================================================

static void ui_update_timer_cb(lv_timer_t *t) {
    Meck* mesh = meck_get_instance();

    // Home tile: unread message count
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

    // Home tile: last RSSI/SNR
    if (lbl_home_rssi && radio_driver.getPacketsRecv() > 0) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%.0f dBm / %.1f dB",
                 radio_driver.getLastRSSI(), radio_driver.getLastSNR());
        lv_label_set_text(lbl_home_rssi, buf);
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
                int w = snprintf(log_buf + pos, sizeof(log_buf) - pos,
                    "%s  [%02X%02X]\n  RSSI %.0f  SNR %.1f  hops %s\n\n",
                    recent[i].name,
                    recent[i].pub_key_prefix[0], recent[i].pub_key_prefix[1],
                    recent[i].rssi, recent[i].snr,
                    recent[i].path_len == 0xFF ? "direct" : "flood");
                if (w > 0) pos += w;
            }
            log_buf[pos] = '\0';
            lv_label_set_text(lbl_recent_list, log_buf);
        }
    }

    // Battery tile readout - TODO: needs the LilyGo battery accessor.
    // The old meshcore_test.cpp owned its own bq27220 singleton. In lvgl_9_ui
    // the battery chip is initialized by LilyGo (boot log: "bq27220 init success")
    // but we do not yet know the export symbol. To find it run:
    //   grep -rn "BQ27220\|bq27220\|battery" ~/T-Display-P4/main/examples/lvgl_9_ui/main.cpp
    //   grep -rn "extern.*board\|extern.*Battery" ~/T-Display-P4/main/examples/lvgl_9_ui/
    // Once identified, the live readout (mv/pct/avgCur/avgPow/tte/remCap/desCap/temp)
    // can be wired in here. Layout already in place in create_page_battery.
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

    lv_timer_create(ui_update_timer_cb, 500, NULL);

    _lock_release(&lvgl_api_lock);

    printf("MeckUI: home + 6 detail tiles + settings + radio picker ready\n");
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
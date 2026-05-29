/*
 * MeckReaderUI.cpp — LVGL screens for the Meck text reader
 *
 * See MeckReaderUI.h for the module overview. Structured to mirror
 * MeckAudioUI.cpp: a folder browser rooted at /sdcard/books, plus a
 * reading view backed entirely by MeckReader (the paging/resume core).
 *
 * Fonts go through meck_ui_set_font (exposed from MeckUI.cpp) so the
 * reader honours the Settings font-size preference and live-updates with
 * it, the same as the rest of the UI.
 */

#include "MeckReaderUI.h"
#include "MeckReader.h"

#include "lvgl.h"
#include "t_display_p4_driver.h"

#include <dirent.h>
#include <sys/stat.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include <vector>
#include <string>
#include <algorithm>

/* ----- borrowed from MeckUI.cpp -------------------------------------------*/
extern "C" void lock_screen_scroll(lv_obj_t* scr);
extern "C" void meck_ui_load_home_screen(void);
extern "C" void meck_ui_attach_audio_indicator(lv_obj_t* parent, int slot,
        const lv_font_t* font, int32_t x_ofs, int32_t y_ofs);
extern "C" void meck_ui_set_font(lv_obj_t* obj, const lv_font_t* base,
                                 lv_style_selector_t part);

/* Meck custom fonts (defined in meck_montserrat_<size>.c at the component
 * root, declared the same way in MeckUI.cpp). */
extern "C" {
    extern lv_font_t meck_montserrat_18;
    extern lv_font_t meck_montserrat_22;
    extern lv_font_t meck_montserrat_24;
}

/* ============================================================================
 * Constants
 * ==========================================================================*/

static const char*  READER_ROOT  = "/sdcard/books";
static const size_t PATH_MAX_LEN = 256;
static const int    ROW_H        = 90;
static const int    HEADER_H     = 90;
static const int    BREADCRUMB_H = 50;

/* Reading-view geometry: a header for the Back button, side margins, and a
 * footer line for the percentage. */
static const int READER_HEADER_H = 95;
static const int READER_MARGIN   = 15;
static const int READER_FOOTER_H = 45;

static int reader_page_w(void) { return SCREEN_WIDTH - 2 * READER_MARGIN; }
static int reader_page_h(void) {
    return SCREEN_HEIGHT - READER_HEADER_H - READER_FOOTER_H;
}

/* ============================================================================
 * Browser state
 * ==========================================================================*/

struct ReaderEntry {
    std::string name;        /* basename only */
    std::string ext;         /* lowercased extension, empty for dirs */
    bool        is_dir;
    uint64_t    size;        /* bytes; 0 for dirs */
    bool        has_resume;  /* a saved reading position exists */
};

static char g_browser_path[PATH_MAX_LEN] = "";
static std::vector<ReaderEntry> g_entries;

static lv_obj_t* scr_reader_browser = NULL;
static lv_obj_t* scr_reader_view    = NULL;

/* Browser widgets updated on path change. */
static lv_obj_t* lbl_browser_path = NULL;   /* breadcrumb */
static lv_obj_t* btn_browser_up   = NULL;   /* up-one-level */
static lv_obj_t* list_container   = NULL;   /* scrollable list area */

/* Reading-view widgets. */
static lv_obj_t* lbl_reader_text     = NULL;   /* current page text */
static lv_obj_t* lbl_reader_progress = NULL;   /* percentage */

/* The file the user tapped to open. */
static char g_current_file[PATH_MAX_LEN] = "";

/* ============================================================================
 * Forward declarations
 * ==========================================================================*/

static void browser_repopulate(void);
static void on_browser_back_clicked(lv_event_t* e);
static void on_browser_up_clicked(lv_event_t* e);
static void on_dir_row_clicked(lv_event_t* e);
static void on_file_row_clicked(lv_event_t* e);
static void reader_render_current(void);
static void on_reader_prev_tap(lv_event_t* e);
static void on_reader_next_tap(lv_event_t* e);
static void on_reader_view_back(lv_event_t* e);

/* ============================================================================
 * Helpers — path manipulation, extension test
 * ==========================================================================*/

static std::string to_lower_ext(const char* name) {
    const char* dot = strrchr(name, '.');
    if (!dot || dot == name) return "";
    std::string out(dot + 1);
    for (char& c : out) c = (char)tolower((unsigned char)c);
    return out;
}

static bool is_txt_ext(const std::string& ext) {
    return ext == "txt";
}

/* Strip the last path component. Returns true if a strip happened; false if
 * we were already at READER_ROOT. */
static bool path_strip_last(char* path) {
    if (strcmp(path, READER_ROOT) == 0) return false;
    char* slash = strrchr(path, '/');
    if (!slash || slash == path) return false;
    *slash = '\0';
    if (strncmp(path, READER_ROOT, strlen(READER_ROOT)) != 0) {
        snprintf(path, PATH_MAX_LEN, "%s", READER_ROOT);
    }
    return true;
}

static void path_append(char* path, const char* name) {
    size_t l = strlen(path);
    if (l > 0 && path[l - 1] != '/') {
        strncat(path, "/", PATH_MAX_LEN - l - 1);
    }
    strncat(path, name, PATH_MAX_LEN - strlen(path) - 1);
}

/* ============================================================================
 * Directory scan
 * ==========================================================================*/

static void scan_current_dir(void) {
    g_entries.clear();

    DIR* d = opendir(g_browser_path);
    if (!d) return;

    struct dirent* de;
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;   /* dotfiles, .pos sidecars */

        ReaderEntry e;
        e.name = de->d_name;
        e.is_dir = (de->d_type == DT_DIR);
        e.size = 0;
        e.has_resume = false;

        if (!e.is_dir) {
            /* readdir doesn't always set d_type on FAT32; stat on unknown. */
            if (de->d_type == DT_UNKNOWN) {
                char full[PATH_MAX_LEN];
                snprintf(full, sizeof(full), "%s/%s", g_browser_path, de->d_name);
                struct stat st;
                if (stat(full, &st) == 0) {
                    e.is_dir = S_ISDIR(st.st_mode);
                    e.size   = st.st_size;
                }
            }
            if (!e.is_dir) {
                e.ext = to_lower_ext(e.name.c_str());
                if (!is_txt_ext(e.ext)) continue;   /* only folders + .txt */

                char full[PATH_MAX_LEN];
                snprintf(full, sizeof(full), "%s/%s", g_browser_path, de->d_name);
                if (e.size == 0) {
                    struct stat st;
                    if (stat(full, &st) == 0) e.size = st.st_size;
                }
                e.has_resume = meck_reader_has_resume(full);
            }
        }

        g_entries.push_back(std::move(e));
    }
    closedir(d);

    /* Dirs first, then files; alphabetical within each group. */
    std::sort(g_entries.begin(), g_entries.end(),
              [](const ReaderEntry& a, const ReaderEntry& b) {
                  if (a.is_dir != b.is_dir) return a.is_dir > b.is_dir;
                  return strcasecmp(a.name.c_str(), b.name.c_str()) < 0;
              });
}

/* ============================================================================
 * Row factory
 * ==========================================================================*/

static char* dup_str(const char* s) {
    size_t n = strlen(s) + 1;
    char* out = (char*)malloc(n);
    if (out) memcpy(out, s, n);
    return out;
}

static void delete_user_data_event_cb(lv_event_t* e) {
    void* p = lv_event_get_user_data(e);
    if (p) free(p);
}

static void add_entry_row(lv_obj_t* parent, const ReaderEntry& e, int y) {
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_set_size(row, SCREEN_WIDTH - 20, ROW_H);
    lv_obj_set_pos(row, 10, y);
    lv_obj_set_style_bg_color(row, lv_color_make(28, 28, 28), 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, lv_color_make(60, 60, 60), 0);
    lv_obj_set_style_radius(row, 8, 0);
    lv_obj_set_style_pad_all(row, 8, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);

    /* Icon column. */
    lv_obj_t* ico = lv_label_create(row);
    lv_label_set_text(ico,
        e.is_dir         ? LV_SYMBOL_DIRECTORY
        : e.has_resume   ? LV_SYMBOL_PLAY        /* in progress / resume here */
                         : LV_SYMBOL_FILE);
    lv_obj_set_style_text_color(ico,
        e.is_dir ? lv_palette_main(LV_PALETTE_AMBER)
                 : (e.has_resume ? lv_palette_main(LV_PALETTE_GREEN)
                                 : lv_color_white()), 0);
    lv_obj_set_style_text_font(ico, &lv_font_montserrat_24, 0);
    lv_obj_align(ico, LV_ALIGN_LEFT_MID, 4, 0);

    /* Title. */
    lv_obj_t* title = lv_label_create(row);
    lv_label_set_text(title, e.name.c_str());
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    meck_ui_set_font(title, &meck_montserrat_22, 0);
    lv_label_set_long_mode(title, LV_LABEL_LONG_DOT);
    lv_obj_set_width(title, SCREEN_WIDTH - 100);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 44, 6);

    /* Subtitle: size for files, "Folder" for directories. (All books are
     * .txt, so the extension is omitted as redundant.) */
    char sub[64];
    if (e.is_dir) {
        snprintf(sub, sizeof(sub), "Folder");
    } else if (e.size >= 1024 * 1024) {
        snprintf(sub, sizeof(sub), "%.1f MB", (double)e.size / (1024.0 * 1024.0));
    } else {
        snprintf(sub, sizeof(sub), "%.0f KB", (double)e.size / 1024.0);
    }
    lv_obj_t* st = lv_label_create(row);
    lv_label_set_text(st, sub);
    lv_obj_set_style_text_color(st, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_ui_set_font(st, &meck_montserrat_18, 0);
    lv_obj_align(st, LV_ALIGN_BOTTOM_LEFT, 44, -6);

    /* Click handler. user_data is a malloc'd copy of the basename so the
     * callback can rebuild the full path. Freed by the DELETE event. */
    char* name_copy = dup_str(e.name.c_str());
    lv_obj_add_event_cb(row,
        e.is_dir ? on_dir_row_clicked : on_file_row_clicked,
        LV_EVENT_CLICKED, name_copy);
    lv_obj_add_event_cb(row, delete_user_data_event_cb,
        LV_EVENT_DELETE, name_copy);
}

/* ============================================================================
 * Browser repopulate
 * ==========================================================================*/

static void browser_repopulate(void) {
    if (!list_container) return;

    /* Breadcrumb: show the path from /sdcard onward. */
    if (lbl_browser_path) {
        const char* shown = g_browser_path;
        const char* sd = "/sdcard";
        if (strncmp(shown, sd, strlen(sd)) == 0) shown += strlen(sd);
        lv_label_set_text(lbl_browser_path, shown);
    }
    if (btn_browser_up) {
        if (strcmp(g_browser_path, READER_ROOT) == 0) {
            lv_obj_add_state(btn_browser_up, LV_STATE_DISABLED);
        } else {
            lv_obj_clear_state(btn_browser_up, LV_STATE_DISABLED);
        }
    }

    scan_current_dir();
    lv_obj_clean(list_container);

    if (g_entries.empty()) {
        lv_obj_t* msg = lv_label_create(list_container);
        if (strcmp(g_browser_path, READER_ROOT) == 0) {
            lv_label_set_text(msg,
                "No books yet.\n\n"
                "Create a folder on the SD card:\n"
                "  /books/\n\n"
                "Then drop in .txt files, in\n"
                "subfolders if you like.");
        } else {
            lv_label_set_text(msg, "Folder is empty.");
        }
        lv_obj_set_style_text_color(msg, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_ui_set_font(msg, &meck_montserrat_18, 0);
        lv_obj_set_style_text_align(msg, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_width(msg, SCREEN_WIDTH - 40);
        lv_obj_center(msg);
        return;
    }

    int y = 8;
    for (const ReaderEntry& e : g_entries) {
        add_entry_row(list_container, e, y);
        y += ROW_H + 8;
    }
    lv_obj_update_layout(list_container);
}

/* ============================================================================
 * Browser click handlers
 * ==========================================================================*/

static void on_browser_back_clicked(lv_event_t* e) {
    (void)e;
    /* At the root, Back goes home. In a subfolder, Back goes up one level. */
    if (strcmp(g_browser_path, READER_ROOT) == 0) {
        meck_ui_load_home_screen();
    } else if (path_strip_last(g_browser_path)) {
        browser_repopulate();
    } else {
        meck_ui_load_home_screen();
    }
}

static void on_browser_up_clicked(lv_event_t* e) {
    (void)e;
    if (path_strip_last(g_browser_path)) {
        browser_repopulate();
    }
}

static void on_dir_row_clicked(lv_event_t* e) {
    const char* name = (const char*)lv_event_get_user_data(e);
    if (!name) return;
    path_append(g_browser_path, name);
    browser_repopulate();
}

static void on_file_row_clicked(lv_event_t* e) {
    const char* name = (const char*)lv_event_get_user_data(e);
    if (!name) return;

    snprintf(g_current_file, sizeof(g_current_file),
             "%s/%s", g_browser_path, name);
    if (!meck_reader_open(g_current_file)) return;
    meck_reader_ui_show_reader();
    reader_render_current();
}

/* ============================================================================
 * Browser screen build
 * ==========================================================================*/

static void create_browser_screen(void) {
    scr_reader_browser = lv_obj_create(NULL);
    lock_screen_scroll(scr_reader_browser);
    lv_obj_set_style_bg_color(scr_reader_browser, lv_color_black(), 0);

    /* Back button -> up a level / home */
    lv_obj_t* btn_back = lv_button_create(scr_reader_browser);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t* bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_ui_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_browser_back_clicked, LV_EVENT_CLICKED, NULL);

    /* Title */
    lv_obj_t* title = lv_label_create(scr_reader_browser);
    lv_label_set_text(title, "Library");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_CYAN), 0);
    meck_ui_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    /* Up-one-level button (top-right) */
    btn_browser_up = lv_button_create(scr_reader_browser);
    lv_obj_set_size(btn_browser_up, 60, 40);
    lv_obj_align(btn_browser_up, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_bg_color(btn_browser_up, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_browser_up, 8, 0);
    lv_obj_t* ul = lv_label_create(btn_browser_up);
    lv_label_set_text(ul, LV_SYMBOL_UP);
    lv_obj_set_style_text_color(ul, lv_color_white(), 0);
    lv_obj_set_style_text_font(ul, &lv_font_montserrat_18, 0);
    lv_obj_center(ul);
    lv_obj_add_event_cb(btn_browser_up, on_browser_up_clicked, LV_EVENT_CLICKED, NULL);

    /* Breadcrumb (below header) */
    lbl_browser_path = lv_label_create(scr_reader_browser);
    lv_label_set_text(lbl_browser_path, "/books");
    lv_label_set_long_mode(lbl_browser_path, LV_LABEL_LONG_DOT);
    lv_obj_set_width(lbl_browser_path, SCREEN_WIDTH - 40);
    lv_obj_set_style_text_color(lbl_browser_path, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_ui_set_font(lbl_browser_path, &meck_montserrat_18, 0);
    lv_obj_align(lbl_browser_path, LV_ALIGN_TOP_LEFT, 20, HEADER_H);

    /* Scrollable list container */
    list_container = lv_obj_create(scr_reader_browser);
    lv_obj_set_size(list_container, SCREEN_WIDTH,
                    SCREEN_HEIGHT - HEADER_H - BREADCRUMB_H);
    lv_obj_set_pos(list_container, 0, HEADER_H + BREADCRUMB_H);
    lv_obj_set_style_bg_color(list_container, lv_color_black(), 0);
    lv_obj_set_style_border_width(list_container, 0, 0);
    lv_obj_set_style_pad_all(list_container, 0, 0);
    lv_obj_set_scroll_dir(list_container, LV_DIR_VER);

    /* Now-playing ">>" indicator (slot 19), top-right, left of the Up
     * button. Hidden until audio is playing; tap returns to the player. */
    meck_ui_attach_audio_indicator(scr_reader_browser, 19,
                                   &lv_font_montserrat_24, -85, 18);

    snprintf(g_browser_path, sizeof(g_browser_path), "%s", READER_ROOT);
}

/* ============================================================================
 * Reading view
 * ==========================================================================*/

static void reader_render_current(void) {
    if (!lbl_reader_text) return;
    meck_reader_show_page(lbl_reader_text, reader_page_w(), reader_page_h());
    if (lbl_reader_progress) {
        char b[16];
        snprintf(b, sizeof(b), "%d%%", meck_reader_progress_pct());
        lv_label_set_text(lbl_reader_progress, b);
    }
}

static void on_reader_prev_tap(lv_event_t* e) {
    (void)e;
    meck_reader_prev(lbl_reader_text, reader_page_w(), reader_page_h());
    if (lbl_reader_progress) {
        char b[16];
        snprintf(b, sizeof(b), "%d%%", meck_reader_progress_pct());
        lv_label_set_text(lbl_reader_progress, b);
    }
}

static void on_reader_next_tap(lv_event_t* e) {
    (void)e;
    meck_reader_next(lbl_reader_text, reader_page_w(), reader_page_h());
    if (lbl_reader_progress) {
        char b[16];
        snprintf(b, sizeof(b), "%d%%", meck_reader_progress_pct());
        lv_label_set_text(lbl_reader_progress, b);
    }
}

static void on_reader_view_back(lv_event_t* e) {
    (void)e;
    meck_reader_close();
    meck_reader_ui_show_browser();
}

static void create_reader_view_screen(void) {
    scr_reader_view = lv_obj_create(NULL);
    lock_screen_scroll(scr_reader_view);
    lv_obj_set_style_bg_color(scr_reader_view, lv_color_black(), 0);

    /* Back button -> browser */
    lv_obj_t* btn_back = lv_button_create(scr_reader_view);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t* bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_ui_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_reader_view_back, LV_EVENT_CLICKED, NULL);

    /* Page text: width-constrained and wrapped, below the header. */
    lbl_reader_text = lv_label_create(scr_reader_view);
    lv_label_set_long_mode(lbl_reader_text, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_reader_text, reader_page_w());
    lv_obj_set_pos(lbl_reader_text, READER_MARGIN, READER_HEADER_H);
    lv_obj_set_style_text_color(lbl_reader_text, lv_color_white(), 0);
    meck_ui_set_font(lbl_reader_text, &meck_montserrat_22, 0);
    lv_label_set_text(lbl_reader_text, "");

    /* Progress percentage, centered at the bottom. */
    lbl_reader_progress = lv_label_create(scr_reader_view);
    lv_obj_set_style_text_color(lbl_reader_progress, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_ui_set_font(lbl_reader_progress, &meck_montserrat_18, 0);
    lv_label_set_text(lbl_reader_progress, "0%");
    lv_obj_align(lbl_reader_progress, LV_ALIGN_BOTTOM_MID, 0, -10);

    /* Transparent tap zones over the text area: left third = previous page,
     * right two thirds = next page. Created last so they sit above the text. */
    int zone_h = reader_page_h();
    int prev_w = (SCREEN_WIDTH * 35) / 100;

    lv_obj_t* zone_prev = lv_obj_create(scr_reader_view);
    lv_obj_set_size(zone_prev, prev_w, zone_h);
    lv_obj_set_pos(zone_prev, 0, READER_HEADER_H);
    lv_obj_set_style_bg_opa(zone_prev, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(zone_prev, 0, 0);
    lv_obj_add_flag(zone_prev, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(zone_prev, on_reader_prev_tap, LV_EVENT_CLICKED, NULL);

    lv_obj_t* zone_next = lv_obj_create(scr_reader_view);
    lv_obj_set_size(zone_next, SCREEN_WIDTH - prev_w, zone_h);
    lv_obj_set_pos(zone_next, prev_w, READER_HEADER_H);
    lv_obj_set_style_bg_opa(zone_next, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(zone_next, 0, 0);
    lv_obj_add_flag(zone_next, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(zone_next, on_reader_next_tap, LV_EVENT_CLICKED, NULL);
}

/* ============================================================================
 * Public API
 * ==========================================================================*/

extern "C" void meck_reader_ui_init(void) {
    static bool inited = false;
    if (inited) return;

    create_browser_screen();
    create_reader_view_screen();
    browser_repopulate();

    inited = true;
}

extern "C" void meck_reader_ui_show_browser(void) {
    if (!scr_reader_browser) return;
    browser_repopulate();
    lv_screen_load(scr_reader_browser);
}

extern "C" void meck_reader_ui_show_reader(void) {
    if (!scr_reader_view) return;
    lv_screen_load(scr_reader_view);
}
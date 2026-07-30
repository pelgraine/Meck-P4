/*
 * MeckNotesUI.cpp — LVGL screens for Meck notes
 *
 * See MeckNotesUI.h for the overview. Structured to mirror MeckReaderUI.cpp:
 * a file browser rooted at /sdcard/notes, plus a read-only view backed
 * entirely by MeckReader (the paging/resume core). The notes folder is
 * created on init if it doesn't already exist.
 *
 * Stage 1: browse + read. The in-app editor, new-note, rename and delete are
 * later stages and will add a MeckNotes backend for writes.
 *
 * Fonts go through meck_ui_set_font (exposed from MeckUI.cpp) so notes honour
 * the Settings font-size preference and live-update with it, the same as the
 * rest of the UI.
 */

#include "MeckNotesUI.h"
#include "MeckNotes.h"
#include "MeckReader.h"

#include "lvgl.h"
#include "t_display_p4_driver.h"

// Orientation-aware logical dimensions (mirrors MeckUI.cpp). The panel can be
// rotated at runtime; when rotated, LVGL reports a swapped logical resolution.
// t_display_p4_driver.h's SCREEN_WIDTH/SCREEN_HEIGHT are the fixed *physical*
// panel size, so without this notes would lay out into the portrait-width left
// strip when the screen is landscape. Redefine them within this TU to the live
// logical resolution so the page area, list, and rows fill the screen at
// either orientation. Every use here is in a function body, so a
// function-valued macro is safe.
#undef SCREEN_WIDTH
#undef SCREEN_HEIGHT
static inline int32_t meck_logical_w() { return lv_display_get_horizontal_resolution(lv_display_get_default()); }
static inline int32_t meck_logical_h() { return lv_display_get_vertical_resolution(lv_display_get_default()); }
#define SCREEN_WIDTH  (meck_logical_w())
#define SCREEN_HEIGHT (meck_logical_h())

#include <dirent.h>
#include <sys/stat.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "esp_heap_caps.h"

#include <vector>
#include <string>
#include <algorithm>

/* ----- borrowed from MeckUI.cpp -------------------------------------------*/
extern "C" void lock_screen_scroll(lv_obj_t* scr);
extern "C" void meck_ui_load_home_screen(void);
extern "C" void meck_ui_set_font(lv_obj_t* obj, const lv_font_t* base,
                                 lv_style_selector_t part);
/* Styles an lv_keyboard to match the composer (size, theme, layout pref). */
extern "C" void meck_ui_style_keyboard(lv_obj_t* kb);

/* Hardware-keyboard hooks from MeckUI.cpp: on open, focus the field so the
 * TCA8418 poll has a target and hide the on-screen keyboard when that
 * keyboard is present (the return says whether it is, so callers can
 * reclaim the hidden keyboard's space); on close, drop the focus. No-ops on
 * plain t_display_p4 builds. */
extern "C" bool meck_ui_panel_edit_opened(lv_obj_t* ta, lv_obj_t* kb);
extern "C" void meck_ui_panel_edit_closed(lv_obj_t* ta);

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

static const char*  NOTES_ROOT   = MECK_NOTES_DIR;   /* "/sdcard/notes" */
static const size_t PATH_MAX_LEN = 256;
static const int    ROW_H        = 90;
static const int    HEADER_H     = 90;
static const int    BREADCRUMB_H = 50;

/* Read-view geometry: a header for the Back button, side margins, and a
 * footer line for the percentage. */
static const int NOTES_HEADER_H = 95;
static const int NOTES_MARGIN   = 15;
static const int NOTES_FOOTER_H = 45;

static int notes_page_w(void) { return SCREEN_WIDTH - 2 * NOTES_MARGIN; }
static int notes_page_h(void) {
    return SCREEN_HEIGHT - NOTES_HEADER_H - NOTES_FOOTER_H;
}

/* ============================================================================
 * Browser state
 * ==========================================================================*/

struct NotesEntry {
    std::string name;        /* basename only */
    std::string ext;         /* lowercased extension, empty for dirs */
    bool        is_dir;
    uint64_t    size;        /* bytes; 0 for dirs */
    bool        has_resume;  /* a saved reading position exists */
};

static char g_browser_path[PATH_MAX_LEN] = "";
static std::vector<NotesEntry> g_entries;

static lv_obj_t* scr_notes_browser = NULL;
static lv_obj_t* scr_notes_view    = NULL;
static bool      g_notes_ui_inited = false;

/* Browser widgets updated on path change. */
static lv_obj_t* lbl_browser_path = NULL;   /* breadcrumb */
static lv_obj_t* btn_browser_up   = NULL;   /* up-one-level */
static lv_obj_t* list_container   = NULL;   /* scrollable list area */

/* Read-view widgets. */
static lv_obj_t* lbl_notes_text     = NULL;   /* current page text */
static lv_obj_t* lbl_notes_progress = NULL;   /* percentage */

/* The note the user tapped to open. */
static char g_current_file[PATH_MAX_LEN] = "";

/* ============================================================================
 * Forward declarations
 * ==========================================================================*/

static void browser_repopulate(void);
static void on_browser_back_clicked(lv_event_t* e);
static void on_browser_up_clicked(lv_event_t* e);
static void on_dir_row_clicked(lv_event_t* e);
static void on_file_row_clicked(lv_event_t* e);
static void notes_render_current(void);
static void on_notes_prev_tap(lv_event_t* e);
static void on_notes_next_tap(lv_event_t* e);
static void on_notes_view_back(lv_event_t* e);
static void on_new_note_clicked(lv_event_t* e);
static void on_notes_edit_from_read(lv_event_t* e);
static void on_notes_edit_save(lv_event_t* e);
static void on_notes_kb_ready(lv_event_t* e);
static void create_editor_screen(void);
static void on_file_row_long_pressed(lv_event_t* e);
static void on_action_rename(lv_event_t* e);
static void on_action_delete(lv_event_t* e);
static void on_action_cancel(lv_event_t* e);
static void on_delete_confirm(lv_event_t* e);
static void on_notes_rename_save(lv_event_t* e);
static void on_notes_rename_cancel(lv_event_t* e);
static void create_rename_screen(void);
static void notes_open_rename(void);
static void notes_modal_close(void);

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
 * we were already at NOTES_ROOT. */
static bool path_strip_last(char* path) {
    if (strcmp(path, NOTES_ROOT) == 0) return false;
    char* slash = strrchr(path, '/');
    if (!slash || slash == path) return false;
    *slash = '\0';
    if (strncmp(path, NOTES_ROOT, strlen(NOTES_ROOT)) != 0) {
        snprintf(path, PATH_MAX_LEN, "%s", NOTES_ROOT);
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

        NotesEntry e;
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
              [](const NotesEntry& a, const NotesEntry& b) {
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

static void add_entry_row(lv_obj_t* parent, const NotesEntry& e, int y) {
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

    /* Subtitle: size for files, "Folder" for directories. (All notes are
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

    /* Row interaction. user_data is a malloc'd copy of the basename so the
     * callbacks can rebuild the full path. Freed by the DELETE event. Files
     * open on a short click and bring up the action menu on long press; using
     * SHORT_CLICKED (not CLICKED) keeps a long press from also opening the
     * note, since CLICKED still fires on release after a long press. */
    char* name_copy = dup_str(e.name.c_str());
    if (e.is_dir) {
        lv_obj_add_event_cb(row, on_dir_row_clicked, LV_EVENT_CLICKED, name_copy);
    } else {
        lv_obj_add_event_cb(row, on_file_row_clicked, LV_EVENT_SHORT_CLICKED, name_copy);
        lv_obj_add_event_cb(row, on_file_row_long_pressed, LV_EVENT_LONG_PRESSED, name_copy);
    }
    lv_obj_add_event_cb(row, delete_user_data_event_cb,
        LV_EVENT_DELETE, name_copy);
}

/* The "+ New Note" action row. Styled green to set it apart from file/folder
 * rows. Tapping it starts a new note in the current folder. */
static void add_new_note_row(lv_obj_t* parent, int y) {
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_set_size(row, SCREEN_WIDTH - 20, ROW_H);
    lv_obj_set_pos(row, 10, y);
    lv_obj_set_style_bg_color(row, lv_color_make(20, 40, 20), 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_radius(row, 8, 0);
    lv_obj_set_style_pad_all(row, 8, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* ico = lv_label_create(row);
    lv_label_set_text(ico, LV_SYMBOL_PLUS);
    lv_obj_set_style_text_color(ico, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(ico, &lv_font_montserrat_24, 0);
    lv_obj_align(ico, LV_ALIGN_LEFT_MID, 4, 0);

    lv_obj_t* title = lv_label_create(row);
    lv_label_set_text(title, "New Note");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    meck_ui_set_font(title, &meck_montserrat_22, 0);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 44, 0);

    lv_obj_add_event_cb(row, on_new_note_clicked, LV_EVENT_CLICKED, NULL);
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
        if (strcmp(g_browser_path, NOTES_ROOT) == 0) {
            lv_obj_add_state(btn_browser_up, LV_STATE_DISABLED);
        } else {
            lv_obj_clear_state(btn_browser_up, LV_STATE_DISABLED);
        }
    }

    scan_current_dir();
    lv_obj_clean(list_container);

    int y = 8;
    /* "+ New Note" is always the first row, so the first note can be created
     * even when the folder is empty. New notes land in the current folder. */
    add_new_note_row(list_container, y);
    y += ROW_H + 8;

    if (g_entries.empty()) {
        lv_obj_t* msg = lv_label_create(list_container);
        lv_label_set_text(msg,
            "No notes yet.\n\n"
            "Notes are .txt files stored in\n"
            "/notes on the SD card.");
        lv_obj_set_style_text_color(msg, lv_palette_main(LV_PALETTE_GREY), 0);
        meck_ui_set_font(msg, &meck_montserrat_18, 0);
        lv_obj_set_style_text_align(msg, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_width(msg, SCREEN_WIDTH - 40);
        lv_obj_center(msg);
        return;
    }

    for (const NotesEntry& e : g_entries) {
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
    if (strcmp(g_browser_path, NOTES_ROOT) == 0) {
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
    meck_notes_ui_show_reader();
    notes_render_current();
}

/* ============================================================================
 * Browser screen build
 * ==========================================================================*/

static void create_browser_screen(void) {
    scr_notes_browser = lv_obj_create(NULL);
    lock_screen_scroll(scr_notes_browser);
    lv_obj_set_style_bg_color(scr_notes_browser, lv_color_black(), 0);

    /* Back button -> up a level / home */
    lv_obj_t* btn_back = lv_button_create(scr_notes_browser);
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
    lv_obj_t* title = lv_label_create(scr_notes_browser);
    lv_label_set_text(title, "Notes");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_CYAN), 0);
    meck_ui_set_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 120, 30);

    /* Up-one-level button (top-right) */
    btn_browser_up = lv_button_create(scr_notes_browser);
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
    lbl_browser_path = lv_label_create(scr_notes_browser);
    lv_label_set_text(lbl_browser_path, "/notes");
    lv_label_set_long_mode(lbl_browser_path, LV_LABEL_LONG_DOT);
    lv_obj_set_width(lbl_browser_path, SCREEN_WIDTH - 40);
    lv_obj_set_style_text_color(lbl_browser_path, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_ui_set_font(lbl_browser_path, &meck_montserrat_18, 0);
    lv_obj_align(lbl_browser_path, LV_ALIGN_TOP_LEFT, 20, HEADER_H);

    /* Scrollable list container */
    list_container = lv_obj_create(scr_notes_browser);
    lv_obj_set_size(list_container, SCREEN_WIDTH,
                    SCREEN_HEIGHT - HEADER_H - BREADCRUMB_H);
    lv_obj_set_pos(list_container, 0, HEADER_H + BREADCRUMB_H);
    lv_obj_set_style_bg_color(list_container, lv_color_black(), 0);
    lv_obj_set_style_border_width(list_container, 0, 0);
    lv_obj_set_style_pad_all(list_container, 0, 0);
    lv_obj_set_scroll_dir(list_container, LV_DIR_VER);

    snprintf(g_browser_path, sizeof(g_browser_path), "%s", NOTES_ROOT);
}

/* ============================================================================
 * Read view
 * ==========================================================================*/

static void notes_render_current(void) {
    if (!lbl_notes_text) return;
    meck_reader_show_page(lbl_notes_text, notes_page_w(), notes_page_h());
    if (lbl_notes_progress) {
        char b[16];
        snprintf(b, sizeof(b), "%d%%", meck_reader_progress_pct());
        lv_label_set_text(lbl_notes_progress, b);
    }
}

static void on_notes_prev_tap(lv_event_t* e) {
    (void)e;
    meck_reader_prev(lbl_notes_text, notes_page_w(), notes_page_h());
    if (lbl_notes_progress) {
        char b[16];
        snprintf(b, sizeof(b), "%d%%", meck_reader_progress_pct());
        lv_label_set_text(lbl_notes_progress, b);
    }
}

static void on_notes_next_tap(lv_event_t* e) {
    (void)e;
    meck_reader_next(lbl_notes_text, notes_page_w(), notes_page_h());
    if (lbl_notes_progress) {
        char b[16];
        snprintf(b, sizeof(b), "%d%%", meck_reader_progress_pct());
        lv_label_set_text(lbl_notes_progress, b);
    }
}

static void on_notes_view_back(lv_event_t* e) {
    (void)e;
    meck_reader_close();
    meck_notes_ui_show_browser();
}

static void create_notes_view_screen(void) {
    scr_notes_view = lv_obj_create(NULL);
    lock_screen_scroll(scr_notes_view);
    lv_obj_set_style_bg_color(scr_notes_view, lv_color_black(), 0);

    /* Back button -> browser */
    lv_obj_t* btn_back = lv_button_create(scr_notes_view);
    lv_obj_set_size(btn_back, 100, 70);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t* bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    meck_ui_set_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_notes_view_back, LV_EVENT_CLICKED, NULL);

    /* Edit button (top-right) -> open this note in the editor. Sits in the
     * header, above the page tap zones, so it isn't swallowed by them. */
    lv_obj_t* btn_edit = lv_button_create(scr_notes_view);
    lv_obj_set_size(btn_edit, 110, 70);
    lv_obj_align(btn_edit, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_bg_color(btn_edit, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_edit, 8, 0);
    lv_obj_t* el = lv_label_create(btn_edit);
    lv_label_set_text(el, LV_SYMBOL_EDIT " Edit");
    lv_obj_set_style_text_color(el, lv_color_white(), 0);
    meck_ui_set_font(el, &meck_montserrat_18, 0);
    lv_obj_center(el);
    lv_obj_add_event_cb(btn_edit, on_notes_edit_from_read, LV_EVENT_CLICKED, NULL);

    /* Page text: width-constrained and wrapped, below the header. */
    lbl_notes_text = lv_label_create(scr_notes_view);
    lv_label_set_long_mode(lbl_notes_text, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_notes_text, notes_page_w());
    lv_obj_set_pos(lbl_notes_text, NOTES_MARGIN, NOTES_HEADER_H);
    lv_obj_set_style_text_color(lbl_notes_text, lv_color_white(), 0);
    meck_ui_set_font(lbl_notes_text, &meck_montserrat_22, 0);
    lv_label_set_text(lbl_notes_text, "");

    /* Progress percentage, centered at the bottom. */
    lbl_notes_progress = lv_label_create(scr_notes_view);
    lv_obj_set_style_text_color(lbl_notes_progress, lv_palette_main(LV_PALETTE_GREY), 0);
    meck_ui_set_font(lbl_notes_progress, &meck_montserrat_18, 0);
    lv_label_set_text(lbl_notes_progress, "0%");
    lv_obj_align(lbl_notes_progress, LV_ALIGN_BOTTOM_MID, 0, -10);

    /* Transparent tap zones over the text area: left third = previous page,
     * right two thirds = next page. Created last so they sit above the text. */
    int zone_h = notes_page_h();
    int prev_w = (SCREEN_WIDTH * 35) / 100;

    lv_obj_t* zone_prev = lv_obj_create(scr_notes_view);
    lv_obj_set_size(zone_prev, prev_w, zone_h);
    lv_obj_set_pos(zone_prev, 0, NOTES_HEADER_H);
    lv_obj_set_style_bg_opa(zone_prev, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(zone_prev, 0, 0);
    lv_obj_add_flag(zone_prev, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(zone_prev, on_notes_prev_tap, LV_EVENT_CLICKED, NULL);

    lv_obj_t* zone_next = lv_obj_create(scr_notes_view);
    lv_obj_set_size(zone_next, SCREEN_WIDTH - prev_w, zone_h);
    lv_obj_set_pos(zone_next, prev_w, NOTES_HEADER_H);
    lv_obj_set_style_bg_opa(zone_next, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(zone_next, 0, 0);
    lv_obj_add_flag(zone_next, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(zone_next, on_notes_next_tap, LV_EVENT_CLICKED, NULL);
}

/* ============================================================================
 * Editor
 * ==========================================================================*/

static lv_obj_t* scr_notes_editor = NULL;
static lv_obj_t* ta_notes_edit    = NULL;
static lv_obj_t* kb_notes_edit    = NULL;

/* Load/scratch buffer for the text area. 16 KB matches the upstream notes cap
 * and is ample for hand-typed text. Allocated once in PSRAM (not internal BSS)
 * by meck_notes_ui_init; internal RAM is scarce during boot. */
static const size_t NOTES_EDIT_BUF_SZ = 16384;
static char* g_edit_buf = NULL;

/* Write the text area back to g_current_file. Empty text is skipped so a
 * "New Note" that was opened and abandoned doesn't leave a zero-byte file. */
static void notes_save_current(void) {
    if (!ta_notes_edit || g_current_file[0] == '\0') return;
    const char* txt = lv_textarea_get_text(ta_notes_edit);
    if (txt && txt[0] != '\0') meck_notes_save(g_current_file, txt);
}

static void on_notes_edit_save(lv_event_t* e) {
    (void)e;
    notes_save_current();
    meck_notes_ui_show_browser();
}

/* The keyboard's OK/Enter inserts a newline here rather than submitting: notes
 * are multi-line free text, and saving is the explicit Save button. */
static void on_notes_kb_ready(lv_event_t* e) {
    (void)e;
    if (ta_notes_edit) lv_textarea_add_char(ta_notes_edit, '\n');
}

/* Start a fresh note in the current folder and open the editor empty. The file
 * itself is created on Save (skipped if left empty). */
static void on_new_note_clicked(lv_event_t* e) {
    (void)e;
    if (!meck_notes_new_path(g_current_file, sizeof(g_current_file))) return;
    if (ta_notes_edit) lv_textarea_set_text(ta_notes_edit, "");
    meck_notes_ui_show_editor();
}

/* From the read view: close the reader, load the note's text into the editor. */
static void on_notes_edit_from_read(lv_event_t* e) {
    (void)e;
    meck_reader_close();
    int n = meck_notes_load(g_current_file, g_edit_buf, NOTES_EDIT_BUF_SZ);
    if (ta_notes_edit)
        lv_textarea_set_text(ta_notes_edit, n >= 0 ? g_edit_buf : "");
    meck_notes_ui_show_editor();
}

static void create_editor_screen(void) {
    scr_notes_editor = lv_obj_create(NULL);
    lock_screen_scroll(scr_notes_editor);
    lv_obj_set_style_bg_color(scr_notes_editor, lv_color_black(), 0);

    /* Save button -> write + back to browser. Occupies the usual Back slot;
     * there is no separate discard path in this stage. */
    lv_obj_t* btn_save = lv_button_create(scr_notes_editor);
    lv_obj_set_size(btn_save, 120, 70);
    lv_obj_align(btn_save, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_save, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_save, 8, 0);
    lv_obj_t* sl = lv_label_create(btn_save);
    lv_label_set_text(sl, LV_SYMBOL_OK " Save");
    lv_obj_set_style_text_color(sl, lv_color_white(), 0);
    meck_ui_set_font(sl, &meck_montserrat_18, 0);
    lv_obj_center(sl);
    lv_obj_add_event_cb(btn_save, on_notes_edit_save, LV_EVENT_CLICKED, NULL);

    /* On-screen keyboard: the primary input. Styled to match the composer and
     * honour the layout preference via the helper exposed from MeckUI.cpp,
     * which also sizes it to SCREEN_WIDTH x MECK_KB_HEIGHT and bottom-aligns
     * it. Created before the text area so its height is known below. */
    kb_notes_edit = lv_keyboard_create(scr_notes_editor);
    meck_ui_style_keyboard(kb_notes_edit);
    lv_obj_update_layout(kb_notes_edit);

    /* Multi-line text area filling the gap between header and keyboard. */
    ta_notes_edit = lv_textarea_create(scr_notes_editor);
    lv_textarea_set_one_line(ta_notes_edit, false);
    lv_obj_set_style_text_color(ta_notes_edit, lv_color_white(), 0);
    lv_obj_set_style_bg_color(ta_notes_edit, lv_color_make(18, 18, 18), 0);
    lv_obj_set_style_border_width(ta_notes_edit, 0, 0);
    meck_ui_set_font(ta_notes_edit, &meck_montserrat_22, 0);
    int kbh = lv_obj_get_height(kb_notes_edit);
    lv_obj_set_pos(ta_notes_edit, NOTES_MARGIN, NOTES_HEADER_H);
    lv_obj_set_size(ta_notes_edit, SCREEN_WIDTH - 2 * NOTES_MARGIN,
                    SCREEN_HEIGHT - NOTES_HEADER_H - kbh);

    /* Keyboard types into the text area; its OK key inserts a newline. */
    lv_keyboard_set_textarea(kb_notes_edit, ta_notes_edit);
    lv_obj_add_event_cb(kb_notes_edit, on_notes_kb_ready, LV_EVENT_READY, NULL);
}

/* ============================================================================
 * Rename / delete (long-press a note row)
 * ==========================================================================*/

/* Full path of the note the action menu / rename / delete is operating on. */
static char g_action_target[PATH_MAX_LEN] = "";

/* Current modal backdrop (action menu or delete confirm). Deleting it dismisses
 * the modal and all its children. */
static lv_obj_t* g_modal = NULL;

/* Rename screen widgets. */
static lv_obj_t* scr_notes_rename = NULL;
static lv_obj_t* ta_notes_rename  = NULL;
static lv_obj_t* kb_notes_rename  = NULL;

static void notes_modal_close(void) {
    /* Deleted from within a modal button's own CLICKED handler, i.e. while an
     * ancestor of the event target is being processed. Async deletion defers it
     * past the current event dispatch so LVGL doesn't touch freed memory. */
    if (g_modal) { lv_obj_delete_async(g_modal); g_modal = NULL; }
}

/* Dimmed full-screen backdrop with a centered card carrying a title. Buttons
 * are added to the returned card by the caller. */
static lv_obj_t* notes_modal_open(const char* title) {
    notes_modal_close();   /* never stack two */

    lv_obj_t* back = lv_obj_create(scr_notes_browser);
    lv_obj_set_size(back, SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_set_pos(back, 0, 0);
    lv_obj_set_style_bg_color(back, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(back, LV_OPA_70, 0);
    lv_obj_set_style_border_width(back, 0, 0);
    lv_obj_set_style_radius(back, 0, 0);
    lv_obj_set_style_pad_all(back, 0, 0);
    lv_obj_clear_flag(back, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(back, LV_OBJ_FLAG_CLICKABLE);   /* swallow taps on the list behind */

    lv_obj_t* card = lv_obj_create(back);
    lv_obj_set_width(card, SCREEN_WIDTH - 80);
    lv_obj_set_height(card, LV_SIZE_CONTENT);
    lv_obj_center(card);
    lv_obj_set_style_bg_color(card, lv_color_make(28, 28, 28), 0);
    lv_obj_set_style_border_color(card, lv_color_make(80, 80, 80), 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_radius(card, 10, 0);
    lv_obj_set_style_pad_all(card, 16, 0);
    lv_obj_set_style_pad_row(card, 10, 0);
    lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(card, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* t = lv_label_create(card);
    lv_label_set_text(t, title);
    lv_label_set_long_mode(t, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(t, lv_pct(100));
    lv_obj_set_style_text_color(t, lv_color_white(), 0);
    lv_obj_set_style_text_align(t, LV_TEXT_ALIGN_CENTER, 0);
    meck_ui_set_font(t, &meck_montserrat_22, 0);

    g_modal = back;
    return card;
}

static void notes_modal_button(lv_obj_t* card, const char* text,
                               lv_color_t bg, lv_event_cb_t cb) {
    lv_obj_t* b = lv_button_create(card);
    lv_obj_set_width(b, lv_pct(100));
    lv_obj_set_height(b, 64);
    lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_set_style_radius(b, 8, 0);
    lv_obj_t* l = lv_label_create(b);
    lv_label_set_text(l, text);
    lv_obj_set_style_text_color(l, lv_color_white(), 0);
    meck_ui_set_font(l, &meck_montserrat_22, 0);
    lv_obj_center(l);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, NULL);
}

/* Basename of g_action_target (after the last '/'). */
static const char* action_basename(void) {
    const char* slash = strrchr(g_action_target, '/');
    return slash ? slash + 1 : g_action_target;
}

static void on_file_row_long_pressed(lv_event_t* e) {
    const char* name = (const char*)lv_event_get_user_data(e);
    if (!name) return;
    snprintf(g_action_target, sizeof(g_action_target),
             "%s/%s", g_browser_path, name);

    lv_obj_t* card = notes_modal_open(name);
    notes_modal_button(card, LV_SYMBOL_EDIT " Rename",
                       lv_color_make(40, 40, 40), on_action_rename);
    notes_modal_button(card, LV_SYMBOL_TRASH " Delete",
                       lv_palette_darken(LV_PALETTE_RED, 2), on_action_delete);
    notes_modal_button(card, "Cancel",
                       lv_color_make(40, 40, 40), on_action_cancel);
}

static void on_action_cancel(lv_event_t* e) {
    (void)e;
    notes_modal_close();
}

static void on_action_delete(lv_event_t* e) {
    (void)e;
    char title[PATH_MAX_LEN + 16];
    snprintf(title, sizeof(title), "Delete \"%s\"?", action_basename());
    lv_obj_t* card = notes_modal_open(title);   /* replaces the action menu */
    notes_modal_button(card, LV_SYMBOL_TRASH " Delete",
                       lv_palette_darken(LV_PALETTE_RED, 2), on_delete_confirm);
    notes_modal_button(card, "Cancel",
                       lv_color_make(40, 40, 40), on_action_cancel);
}

static void on_delete_confirm(lv_event_t* e) {
    (void)e;
    notes_modal_close();
    meck_notes_delete(g_action_target);
    browser_repopulate();
}

static void on_action_rename(lv_event_t* e) {
    (void)e;
    notes_modal_close();
    notes_open_rename();
}

/* Prefill the rename field with the current name minus its .txt extension and
 * show the rename screen. */
static void notes_open_rename(void) {
    if (!ta_notes_rename) return;
    char stem[PATH_MAX_LEN];
    snprintf(stem, sizeof(stem), "%s", action_basename());
    char* dot = strrchr(stem, '.');
    if (dot) *dot = '\0';
    lv_textarea_set_text(ta_notes_rename, stem);
    lv_screen_load(scr_notes_rename);
    meck_ui_panel_edit_opened(ta_notes_rename, kb_notes_rename);
}

static void on_notes_rename_cancel(lv_event_t* e) {
    (void)e;
    meck_notes_ui_show_browser();
}

static void on_notes_rename_save(lv_event_t* e) {
    (void)e;
    if (ta_notes_rename) {
        const char* stem = lv_textarea_get_text(ta_notes_rename);
        if (stem && stem[0] != '\0') {
            char newp[PATH_MAX_LEN];
            snprintf(newp, sizeof(newp), "%s/%s.txt", g_browser_path, stem);
            meck_notes_rename(g_action_target, newp);
        }
    }
    meck_notes_ui_show_browser();
}

static void create_rename_screen(void) {
    scr_notes_rename = lv_obj_create(NULL);
    lock_screen_scroll(scr_notes_rename);
    lv_obj_set_style_bg_color(scr_notes_rename, lv_color_black(), 0);

    /* Save (top-left) */
    lv_obj_t* btn_save = lv_button_create(scr_notes_rename);
    lv_obj_set_size(btn_save, 120, 70);
    lv_obj_align(btn_save, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_save, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_save, 8, 0);
    lv_obj_t* sl = lv_label_create(btn_save);
    lv_label_set_text(sl, LV_SYMBOL_OK " Save");
    lv_obj_set_style_text_color(sl, lv_color_white(), 0);
    meck_ui_set_font(sl, &meck_montserrat_18, 0);
    lv_obj_center(sl);
    lv_obj_add_event_cb(btn_save, on_notes_rename_save, LV_EVENT_CLICKED, NULL);

    /* Cancel (top-right) */
    lv_obj_t* btn_cancel = lv_button_create(scr_notes_rename);
    lv_obj_set_size(btn_cancel, 120, 70);
    lv_obj_align(btn_cancel, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_bg_color(btn_cancel, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_cancel, 8, 0);
    lv_obj_t* cl = lv_label_create(btn_cancel);
    lv_label_set_text(cl, LV_SYMBOL_CLOSE " Cancel");
    lv_obj_set_style_text_color(cl, lv_color_white(), 0);
    meck_ui_set_font(cl, &meck_montserrat_18, 0);
    lv_obj_center(cl);
    lv_obj_add_event_cb(btn_cancel, on_notes_rename_cancel, LV_EVENT_CLICKED, NULL);

    /* Title */
    lv_obj_t* title = lv_label_create(scr_notes_rename);
    lv_label_set_text(title, "Rename note");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_CYAN), 0);
    meck_ui_set_font(title, &meck_montserrat_18, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);

    /* Keyboard (styled, honours layout pref). Created before the field so its
     * height is known. */
    kb_notes_rename = lv_keyboard_create(scr_notes_rename);
    meck_ui_style_keyboard(kb_notes_rename);
    lv_obj_update_layout(kb_notes_rename);

    /* Single-line name field below the header. The ".txt" extension is implied
     * and re-added on save, so the user edits only the name. */
    ta_notes_rename = lv_textarea_create(scr_notes_rename);
    lv_textarea_set_one_line(ta_notes_rename, true);
    lv_obj_set_style_text_color(ta_notes_rename, lv_color_white(), 0);
    lv_obj_set_style_bg_color(ta_notes_rename, lv_color_make(18, 18, 18), 0);
    lv_obj_set_style_border_width(ta_notes_rename, 0, 0);
    meck_ui_set_font(ta_notes_rename, &meck_montserrat_22, 0);
    lv_obj_set_pos(ta_notes_rename, NOTES_MARGIN, NOTES_HEADER_H);
    lv_obj_set_width(ta_notes_rename, SCREEN_WIDTH - 2 * NOTES_MARGIN);

    /* On a single-line name, the keyboard's OK key commits the rename. */
    lv_keyboard_set_textarea(kb_notes_rename, ta_notes_rename);
    lv_obj_add_event_cb(kb_notes_rename, on_notes_rename_save, LV_EVENT_READY, NULL);
    /* Esc from the hardware keyboard replays LV_EVENT_CANCEL on this kb. */
    lv_obj_add_event_cb(kb_notes_rename, on_notes_rename_cancel, LV_EVENT_CANCEL, NULL);
}

/* ============================================================================
 * Public API
 * ==========================================================================*/

extern "C" void meck_notes_ui_init(void) {
    if (g_notes_ui_inited) return;

    if (!g_edit_buf) {
        g_edit_buf = (char*)heap_caps_malloc(NOTES_EDIT_BUF_SZ, MALLOC_CAP_SPIRAM);
    }
    meck_notes_ensure_dir();   /* create /sdcard/notes on first boot */
    create_browser_screen();
    create_notes_view_screen();
    create_editor_screen();
    create_rename_screen();
    browser_repopulate();

    g_notes_ui_inited = true;
}

// Delete the notes screens and clear the init guard so a later
// meck_notes_ui_init() rebuilds them (used by the live orientation rebuild).
// Deleting each screen frees all its child widgets.
extern "C" void meck_notes_ui_teardown(void) {
    if (scr_notes_browser) { lv_obj_delete(scr_notes_browser); scr_notes_browser = NULL; }
    if (scr_notes_view)    { lv_obj_delete(scr_notes_view);    scr_notes_view    = NULL; }
    if (scr_notes_editor)  { lv_obj_delete(scr_notes_editor);  scr_notes_editor  = NULL; }
    if (scr_notes_rename)  { lv_obj_delete(scr_notes_rename);  scr_notes_rename  = NULL; }
    ta_notes_edit   = NULL;
    kb_notes_edit   = NULL;
    ta_notes_rename = NULL;
    kb_notes_rename = NULL;
    g_modal = NULL;   /* was a child of scr_notes_browser, already deleted above */
    g_notes_ui_inited = false;
}

extern "C" void meck_notes_ui_show_browser(void) {
    if (!scr_notes_browser) return;
    /* Leaving the editor or rename screen: drop any lingering field focus so
     * the hardware-keyboard poll stops routing keys to a hidden screen. */
    meck_ui_panel_edit_closed(ta_notes_edit);
    meck_ui_panel_edit_closed(ta_notes_rename);
    browser_repopulate();
    lv_screen_load(scr_notes_browser);
}

extern "C" void meck_notes_ui_show_reader(void) {
    if (!scr_notes_view) return;
    lv_screen_load(scr_notes_view);
}

extern "C" void meck_notes_ui_show_editor(void) {
    if (!scr_notes_editor) return;
    lv_screen_load(scr_notes_editor);
    /* Hardware keyboard: focus the note so typed keys land, hide the
     * on-screen keyboard, and let the text area reclaim its space. On touch
     * builds this only (re)focuses; the sizing matches create time. */
    bool hw = meck_ui_panel_edit_opened(ta_notes_edit, kb_notes_edit);
    if (ta_notes_edit && kb_notes_edit) {
        int kbh = hw ? 0 : lv_obj_get_height(kb_notes_edit);
        lv_obj_set_height(ta_notes_edit,
                          SCREEN_HEIGHT - NOTES_HEADER_H - kbh);
    }
}
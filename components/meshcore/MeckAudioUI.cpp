/*
 * MeckAudioUI.cpp — LVGL screens for the Meck audio player
 *
 * See MeckAudioUI.h for module overview.
 *
 * This first cut delivers a fully-working browser and a placeholder
 * player. The player is wired to the backend interface (MeckAudio.h)
 * but the backend itself is stubbed in MeckAudio.cpp until the next
 * turn (alongside M4BMetadata.h port and Arduino-as-component build
 * wiring).
 */

#include "MeckAudioUI.h"
#include "MeckAudio.h"
#include "meck.h"

#include "lvgl.h"
#include "t_display_p4_driver.h"

#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>

#include <vector>
#include <string>
#include <algorithm>

/* ============================================================================
 * Constants
 * ==========================================================================*/

static const char* AUDIO_ROOT   = "/sdcard/audio";
static const char* MUSIC_PREFIX = "/sdcard/audio/music";
static const char* BOOKS_PREFIX = "/sdcard/audio/audiobooks";

static const size_t PATH_MAX_LEN = 256;
static const int    ROW_H        = 90;   /* px, per file/folder row */
static const int    ROW_PAD_X    = 12;
static const int    HEADER_H     = 90;
static const int    BREADCRUMB_H = 50;

enum LibraryMode {
    LIB_GENERIC,
    LIB_MUSIC,
    LIB_AUDIOBOOK,
};

/* ============================================================================
 * Browser state
 * ==========================================================================*/

struct AudioEntry {
    std::string name;      /* basename only */
    std::string ext;       /* lowercased extension (no dot), empty for dirs */
    bool        is_dir;
    uint64_t    size;      /* bytes; 0 for dirs */
    bool        has_bookmark;
};

static char g_browser_path[PATH_MAX_LEN] = "";
static std::vector<AudioEntry> g_entries;

static lv_obj_t* scr_audio_browser = NULL;
static lv_obj_t* scr_audio_player  = NULL;

/* Browser-screen widgets we need to update on path change. */
static lv_obj_t* lbl_browser_path = NULL;   /* breadcrumb */
static lv_obj_t* btn_browser_up   = NULL;   /* up-one-level */
static lv_obj_t* list_container   = NULL;   /* scrollable list area */
static lv_obj_t* lbl_browser_empty = NULL;  /* "no audio files" overlay */

/* Player-screen widgets are declared below in the player section so they
 * sit next to the code that uses them. */

/* Currently selected (the file the user tapped to open the player). */
static char g_current_file[PATH_MAX_LEN] = "";

/* ============================================================================
 * Helpers — path manipulation, extension test, library mode inference
 * ==========================================================================*/

static bool str_ends_with_ci(const char* s, const char* suffix) {
    if (!s || !suffix) return false;
    size_t ls = strlen(s);
    size_t lf = strlen(suffix);
    if (lf > ls) return false;
    return strcasecmp(s + ls - lf, suffix) == 0;
}

static std::string to_lower_ext(const char* name) {
    /* Returns the extension (after final dot) in lowercase, no leading dot.
     * Returns "" if the name has no dot. */
    const char* dot = strrchr(name, '.');
    if (!dot || dot == name) return "";
    std::string out(dot + 1);
    for (char& c : out) c = (char)tolower((unsigned char)c);
    return out;
}

static bool is_audio_ext(const std::string& ext) {
    return ext == "wav"  || ext == "mp3"  || ext == "m4a" || ext == "m4b" ||
           ext == "flac" || ext == "aac"  || ext == "ogg" || ext == "opus";
}

static LibraryMode infer_library_mode(const char* path) {
    if (strncmp(path, BOOKS_PREFIX, strlen(BOOKS_PREFIX)) == 0) return LIB_AUDIOBOOK;
    if (strncmp(path, MUSIC_PREFIX, strlen(MUSIC_PREFIX)) == 0) return LIB_MUSIC;
    return LIB_GENERIC;
}

/* Strip the last path component. Returns true if a strip happened; false
 * if we were already at AUDIO_ROOT (in which case no change). */
static bool path_strip_last(char* path) {
    if (strcmp(path, AUDIO_ROOT) == 0) return false;
    char* slash = strrchr(path, '/');
    if (!slash || slash == path) return false;
    *slash = '\0';
    /* If we stripped down to before AUDIO_ROOT, restore. */
    if (strncmp(path, AUDIO_ROOT, strlen(AUDIO_ROOT)) != 0) {
        snprintf(path, PATH_MAX_LEN, "%s", AUDIO_ROOT);
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
 * Bookmark lookup — delegated to the backend's public API so the file
 * naming scheme stays single-sourced (FNV-1a hashed path).
 * ==========================================================================*/

static bool bookmark_exists_for(const char* full_path) {
    return meck_audio_bookmark_load_sec(full_path) > 0;
}

/* ============================================================================
 * Directory scan
 * ==========================================================================*/

static void scan_current_dir() {
    g_entries.clear();

    DIR* d = opendir(g_browser_path);
    if (!d) {
        printf("MeckAudioUI: opendir('%s') failed (errno=%d)\n",
               g_browser_path, errno);
        return;
    }

    struct dirent* de;
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;       /* skip dotfiles, .bookmarks/ */

        AudioEntry e;
        e.name = de->d_name;
        e.is_dir = (de->d_type == DT_DIR);
        e.size = 0;
        e.has_bookmark = false;

        if (!e.is_dir) {
            /* readdir doesn't always set d_type reliably on FAT32; fall
             * back to stat when DT_UNKNOWN. */
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
                /* Filter: only directories or known audio extensions. */
                if (!is_audio_ext(e.ext)) continue;

                if (e.size == 0) {
                    /* size not yet filled — stat now */
                    char full[PATH_MAX_LEN];
                    snprintf(full, sizeof(full), "%s/%s", g_browser_path, de->d_name);
                    struct stat st;
                    if (stat(full, &st) == 0) e.size = st.st_size;
                }

                char full[PATH_MAX_LEN];
                snprintf(full, sizeof(full), "%s/%s", g_browser_path, de->d_name);
                e.has_bookmark = bookmark_exists_for(full);
            }
        }

        g_entries.push_back(std::move(e));
    }
    closedir(d);

    /* Sort: dirs first, then files; alphabetical within each group. */
    std::sort(g_entries.begin(), g_entries.end(),
              [](const AudioEntry& a, const AudioEntry& b) {
                  if (a.is_dir != b.is_dir) return a.is_dir > b.is_dir;
                  return strcasecmp(a.name.c_str(), b.name.c_str()) < 0;
              });

    printf("MeckAudioUI: scanned '%s' — %u entries\n",
           g_browser_path, (unsigned)g_entries.size());
}

/* ============================================================================
 * Row factory
 * ==========================================================================*/

static void on_dir_row_clicked(lv_event_t* e);
static void on_file_row_clicked(lv_event_t* e);

/* Allocate one persistent copy of the name string per row so the callback
 * has stable user_data. Freed implicitly when the list_container is
 * cleaned on next rescan. */
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

static void add_entry_row(lv_obj_t* parent, const AudioEntry& e, int y) {
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
        e.is_dir            ? LV_SYMBOL_DIRECTORY
        : e.has_bookmark    ? LV_SYMBOL_OK         /* "you have a saved position here" */
                            : LV_SYMBOL_AUDIO);
    lv_obj_set_style_text_color(ico,
        e.is_dir ? lv_palette_main(LV_PALETTE_AMBER)
                 : (e.has_bookmark ? lv_palette_main(LV_PALETTE_GREEN)
                                   : lv_color_white()), 0);
    lv_obj_set_style_text_font(ico, &lv_font_montserrat_24, 0);
    lv_obj_align(ico, LV_ALIGN_LEFT_MID, 4, 0);

    /* Title. */
    lv_obj_t* title = lv_label_create(row);
    lv_label_set_text(title, e.name.c_str());
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_label_set_long_mode(title, LV_LABEL_LONG_DOT);
    lv_obj_set_width(title, SCREEN_WIDTH - 100);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 44, 6);

    /* Subtitle: size or "Folder". */
    char sub[64];
    if (e.is_dir) {
        snprintf(sub, sizeof(sub), "Folder");
    } else {
        /* Size in MB if >= 1 MB, else KB. Separator is "  -  " (ASCII)
         * rather than " · " because the middle-dot glyph (U+B7) isn't in
         * the default montserrat font set and would render as a missing-
         * glyph warning on every row. */
        if (e.size >= 1024 * 1024) {
            snprintf(sub, sizeof(sub), "%.1f MB  -  %s",
                     (double)e.size / (1024.0 * 1024.0), e.ext.c_str());
        } else {
            snprintf(sub, sizeof(sub), "%.0f KB  -  %s",
                     (double)e.size / 1024.0, e.ext.c_str());
        }
    }
    lv_obj_t* st = lv_label_create(row);
    lv_label_set_text(st, sub);
    lv_obj_set_style_text_color(st, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(st, &lv_font_montserrat_18, 0);
    lv_obj_align(st, LV_ALIGN_BOTTOM_LEFT, 44, -6);

    /* Click handler. user_data is a malloc'd copy of the basename so the
     * callback can reconstruct the full path. Freed by DELETE event. */
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

static void browser_repopulate() {
    if (!list_container) return;

    /* Update breadcrumb. Show the path from /sdcard/audio onward. */
    if (lbl_browser_path) {
        const char* shown = g_browser_path;
        const char* sd = "/sdcard";
        if (strncmp(shown, sd, strlen(sd)) == 0) shown += strlen(sd);
        lv_label_set_text(lbl_browser_path, shown);
    }
    if (btn_browser_up) {
        if (strcmp(g_browser_path, AUDIO_ROOT) == 0) {
            lv_obj_add_state(btn_browser_up, LV_STATE_DISABLED);
        } else {
            lv_obj_clear_state(btn_browser_up, LV_STATE_DISABLED);
        }
    }

    /* Scan and rebuild rows. */
    scan_current_dir();
    lv_obj_clean(list_container);

    if (g_entries.empty()) {
        /* Show an empty-state message. */
        lv_obj_t* msg = lv_label_create(list_container);
        if (strcmp(g_browser_path, AUDIO_ROOT) == 0) {
            lv_label_set_text(msg,
                "No audio yet.\n\n"
                "Create folders on the SD card:\n"
                "  /audio/music/<artist>/<album>/\n"
                "  /audio/audiobooks/<author>/<book>/\n\n"
                "Then drop in .wav, .mp3, .m4a, .m4b,\n"
                ".flac, .aac, .ogg or .opus files.");
        } else {
            lv_label_set_text(msg, "Folder is empty.");
        }
        lv_obj_set_style_text_color(msg, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_obj_set_style_text_font(msg, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_align(msg, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_width(msg, SCREEN_WIDTH - 40);
        lv_obj_center(msg);
        return;
    }

    int y = 8;
    for (const AudioEntry& e : g_entries) {
        add_entry_row(list_container, e, y);
        y += ROW_H + 8;
    }
    /* Make container's scroll bounds reflect content. */
    lv_obj_update_layout(list_container);
}

/* ============================================================================
 * Click handlers
 * ==========================================================================*/

/* Forward decl from MeckUI.cpp. The locked variant deadlocks when called
 * from an LVGL event handler because the LVGL task already holds the
 * lock for its full iteration. The _load_home_screen variant skips the
 * lock — safe because we're already inside LVGL context. */
extern "C" void meck_ui_load_home_screen(void);

static void on_browser_back_clicked(lv_event_t* e) {
    (void)e;
    /* Debug: confirms the click actually reached us. */
    printf("MeckAudioUI: back tapped, path='%s'\n", g_browser_path);

    /* Context-sensitive: at the audio root, Back goes to the home screen.
     * In any subfolder, Back goes up one level — matches phone-app
     * conventions and avoids two near-duplicate navigation buttons. */
    if (strcmp(g_browser_path, AUDIO_ROOT) == 0) {
        meck_ui_load_home_screen();
    } else if (path_strip_last(g_browser_path)) {
        browser_repopulate();
    } else {
        /* Defensive fallback — path was outside AUDIO_ROOT somehow. */
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
    printf("MeckAudioUI: opening file '%s'\n", g_current_file);
    meck_audio_ui_show_player();
}

/* ============================================================================
 * Browser screen build
 * ==========================================================================*/

extern "C" void lock_screen_scroll(lv_obj_t* scr);  /* from MeckUI.cpp */

static void create_browser_screen() {
    scr_audio_browser = lv_obj_create(NULL);
    lock_screen_scroll(scr_audio_browser);
    lv_obj_set_style_bg_color(scr_audio_browser, lv_color_black(), 0);

    /* Back button → home */
    lv_obj_t* btn_back = lv_button_create(scr_audio_browser);
    lv_obj_set_size(btn_back, 80, 40);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t* bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    lv_obj_set_style_text_font(bl, &lv_font_montserrat_16, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_browser_back_clicked,
                        LV_EVENT_CLICKED, NULL);

    /* Title */
    lv_obj_t* title = lv_label_create(scr_audio_browser);
    lv_label_set_text(title, "Audio");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_AMBER), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 100, 18);

    /* Up-one-level button (top-right) */
    btn_browser_up = lv_button_create(scr_audio_browser);
    lv_obj_set_size(btn_browser_up, 60, 40);
    lv_obj_align(btn_browser_up, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_bg_color(btn_browser_up, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_browser_up, 8, 0);
    lv_obj_t* ul = lv_label_create(btn_browser_up);
    lv_label_set_text(ul, LV_SYMBOL_UP);
    lv_obj_set_style_text_color(ul, lv_color_white(), 0);
    lv_obj_set_style_text_font(ul, &lv_font_montserrat_18, 0);
    lv_obj_center(ul);
    lv_obj_add_event_cb(btn_browser_up, on_browser_up_clicked,
                        LV_EVENT_CLICKED, NULL);

    /* Breadcrumb (below header) */
    lbl_browser_path = lv_label_create(scr_audio_browser);
    lv_label_set_text(lbl_browser_path, "/audio");
    lv_label_set_long_mode(lbl_browser_path, LV_LABEL_LONG_DOT);
    lv_obj_set_width(lbl_browser_path, SCREEN_WIDTH - 40);
    lv_obj_set_style_text_color(lbl_browser_path,
                                lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(lbl_browser_path, &lv_font_montserrat_18, 0);
    lv_obj_align(lbl_browser_path, LV_ALIGN_TOP_LEFT, 20, HEADER_H);

    /* Scrollable list container */
    list_container = lv_obj_create(scr_audio_browser);
    lv_obj_set_size(list_container, SCREEN_WIDTH,
                    SCREEN_HEIGHT - HEADER_H - BREADCRUMB_H);
    lv_obj_set_pos(list_container, 0, HEADER_H + BREADCRUMB_H);
    lv_obj_set_style_bg_color(list_container, lv_color_black(), 0);
    lv_obj_set_style_border_width(list_container, 0, 0);
    lv_obj_set_style_pad_all(list_container, 0, 0);
    lv_obj_set_scroll_dir(list_container, LV_DIR_VER);

    /* Start at AUDIO_ROOT. */
    snprintf(g_browser_path, sizeof(g_browser_path), "%s", AUDIO_ROOT);
}

/* ============================================================================
 * Player screen — full transport, progress, volume, sleep timer
 *
 * Cover art and ID3-tag parsing are deferred to a follow-up polish turn;
 * a placeholder rectangle reserves the cover slot so layout won't shift
 * when art lands. Title is derived from filename minus extension.
 *
 * Refresh timer fires at 250 ms while the player screen is loaded. It
 * polls MeckAudio state and updates widgets. When the player isn't the
 * active screen the timer skips work (early-exit on screen check).
 * ==========================================================================*/

/* Widget handles. */
static lv_obj_t* obj_cover_slot       = NULL;   /* dark rect; image goes here later */
static lv_obj_t* obj_cover_image      = NULL;   /* lv_image widget; created on demand */
static lv_obj_t* obj_cover_icon       = NULL;   /* fallback music-note icon */
static lv_obj_t* lbl_player_title     = NULL;
static lv_obj_t* lbl_player_subtitle  = NULL;
static lv_obj_t* slider_progress      = NULL;
static lv_obj_t* lbl_time_current     = NULL;
static lv_obj_t* lbl_time_total       = NULL;
static lv_obj_t* btn_seek_back        = NULL;
static lv_obj_t* btn_play_pause       = NULL;
static lv_obj_t* lbl_play_pause_icon  = NULL;
static lv_obj_t* btn_seek_fwd         = NULL;
static lv_obj_t* slider_volume        = NULL;
static lv_obj_t* lbl_volume           = NULL;
static lv_obj_t* btn_sleep            = NULL;   /* audiobook-only */
static lv_obj_t* lbl_sleep            = NULL;

static lv_timer_t* player_refresh_timer = NULL;

/* Player session state — lives across browser ↔ player navigation. */
static LibraryMode    g_player_mode       = LIB_GENERIC;
static bool           g_progress_dragging = false;
static uint32_t       g_sleep_minutes     = 0;       /* 0 = off */
static uint32_t       g_sleep_deadline_ms = 0;       /* absolute deadline */

/* Playlist — built when a file is opened. Same-folder *.{wav,mp3,m4a,m4b,flac,aac,ogg,opus},
 * sorted alphabetically. Drives EOF auto-advance. */
static std::vector<std::string> g_playlist;
static int                       g_playlist_idx = -1;

/* ----- helpers -------------------------------------------------------------*/

static void format_time(uint32_t total_sec, char* buf, size_t buf_sz) {
    uint32_t h = total_sec / 3600;
    uint32_t m = (total_sec / 60) % 60;
    uint32_t s = total_sec % 60;
    if (h > 0) snprintf(buf, buf_sz, "%u:%02u:%02u", h, m, s);
    else       snprintf(buf, buf_sz, "%u:%02u", m, s);
}

static void file_title_from_path(const char* path, char* out, size_t out_sz) {
    /* Take basename, strip extension. */
    const char* slash = strrchr(path, '/');
    const char* base  = slash ? slash + 1 : path;
    strncpy(out, base, out_sz);
    out[out_sz - 1] = '\0';
    char* dot = strrchr(out, '.');
    if (dot) *dot = '\0';
}

static void parent_dir_of(const char* path, char* out, size_t out_sz) {
    strncpy(out, path, out_sz);
    out[out_sz - 1] = '\0';
    char* slash = strrchr(out, '/');
    if (slash && slash != out) *slash = '\0';
}

/* Pretty subtitle: "<parent folder> · MUSIC" or "AUDIOBOOK" or "GENERIC". */
static void build_subtitle(const char* path, char* out, size_t out_sz) {
    char dir[PATH_MAX_LEN];
    parent_dir_of(path, dir, sizeof(dir));
    const char* folder = strrchr(dir, '/');
    folder = folder ? folder + 1 : dir;
    const char* mode_str =
        g_player_mode == LIB_MUSIC     ? "MUSIC" :
        g_player_mode == LIB_AUDIOBOOK ? "AUDIOBOOK" :
                                         "AUDIO";
    snprintf(out, out_sz, "%s  ·  %s", folder, mode_str);
}

/* Build the same-folder playlist for EOF auto-advance. */
static void build_playlist_for(const char* file_path) {
    g_playlist.clear();
    g_playlist_idx = -1;

    char dir[PATH_MAX_LEN];
    parent_dir_of(file_path, dir, sizeof(dir));

    DIR* d = opendir(dir);
    if (!d) return;

    const char* base = strrchr(file_path, '/');
    base = base ? base + 1 : file_path;

    struct dirent* de;
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;
        std::string ext = to_lower_ext(de->d_name);
        if (!is_audio_ext(ext)) continue;
        std::string full = std::string(dir) + "/" + de->d_name;

        /* Skip directories that snuck through (FAT32 sometimes reports
         * DT_UNKNOWN; verify with stat). */
        struct stat st;
        if (stat(full.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) continue;

        if (strcmp(de->d_name, base) == 0) {
            g_playlist_idx = (int)g_playlist.size();
        }
        g_playlist.push_back(std::move(full));
    }
    closedir(d);

    std::sort(g_playlist.begin(), g_playlist.end());

    /* Re-resolve index after sort. */
    g_playlist_idx = -1;
    for (size_t i = 0; i < g_playlist.size(); i++) {
        if (g_playlist[i] == file_path) {
            g_playlist_idx = (int)i;
            break;
        }
    }
    printf("MeckAudioUI: playlist size=%u, idx=%d\n",
           (unsigned)g_playlist.size(), g_playlist_idx);
}

/* ----- transport handlers --------------------------------------------------*/

static void on_play_pause_clicked(lv_event_t* e) {
    (void)e;
    MeckAudioState s = meck_audio_get_state();
    if (s == MECK_AUDIO_STATE_PLAYING) {
        meck_audio_pause();
    } else if (s == MECK_AUDIO_STATE_PAUSED) {
        meck_audio_resume();
    } else if (s == MECK_AUDIO_STATE_IDLE || s == MECK_AUDIO_STATE_EOF) {
        /* Re-open the current file (e.g. user finished a track, taps play
         * to replay). Backend re-opens at last bookmark. */
        if (g_current_file[0]) {
            uint32_t resume = meck_audio_bookmark_load_sec(g_current_file);
            meck_audio_play_file(g_current_file, resume);
        }
    }
}

static void on_seek_back_clicked(lv_event_t* e) {
    (void)e;
    meck_audio_seek_relative(-30);
}

static void on_seek_fwd_clicked(lv_event_t* e) {
    (void)e;
    meck_audio_seek_relative(+30);
}

static void on_progress_pressed(lv_event_t* e) {
    (void)e;
    g_progress_dragging = true;
}

static void on_progress_released(lv_event_t* e) {
    g_progress_dragging = false;
    if (!slider_progress) return;
    int32_t target = lv_slider_get_value(slider_progress);
    if (target < 0) target = 0;
    meck_audio_seek_absolute((uint32_t)target);
}

static void on_volume_changed(lv_event_t* e) {
    if (!slider_volume) return;
    int32_t v = lv_slider_get_value(slider_volume);
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    meck_audio_set_volume_pct((uint8_t)v);
    if (lbl_volume) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d%%", (int)v);
        lv_label_set_text(lbl_volume, buf);
    }
}

/* Sleep timer cycles: 0 (off) → 15 → 30 → 45 → 60 → 0. */
static void on_sleep_clicked(lv_event_t* e) {
    (void)e;
    static const uint32_t cycle[] = {0, 15, 30, 45, 60};
    static const int N = sizeof(cycle) / sizeof(cycle[0]);

    int idx = 0;
    for (int i = 0; i < N; i++) if (cycle[i] == g_sleep_minutes) { idx = i; break; }
    idx = (idx + 1) % N;
    g_sleep_minutes = cycle[idx];

    if (g_sleep_minutes == 0) {
        g_sleep_deadline_ms = 0;
        printf("Sleep: OFF\n");
    } else {
        g_sleep_deadline_ms = lv_tick_get() + g_sleep_minutes * 60u * 1000u;
        printf("Sleep: %u min\n", (unsigned)g_sleep_minutes);
    }
}

/* ----- refresh timer -------------------------------------------------------*/

static void player_refresh_cb(lv_timer_t* t) {
    (void)t;
    /* Only update widgets when the player is the active screen. */
    if (lv_screen_active() != scr_audio_player) return;

    MeckAudioState s = meck_audio_get_state();
    uint32_t pos = meck_audio_get_position_sec();
    uint32_t dur = meck_audio_get_duration_sec();

    /* Play/pause icon. */
    if (lbl_play_pause_icon) {
        const char* sym = (s == MECK_AUDIO_STATE_PLAYING) ? LV_SYMBOL_PAUSE
                                                          : LV_SYMBOL_PLAY;
        lv_label_set_text(lbl_play_pause_icon, sym);
    }

    /* Progress slider — don't fight the user's drag. */
    if (slider_progress && !g_progress_dragging) {
        if (dur > 0) {
            lv_slider_set_range(slider_progress, 0, (int32_t)dur);
            lv_slider_set_value(slider_progress, (int32_t)pos, LV_ANIM_OFF);
        }
    }

    /* Time labels. */
    char buf[32];
    if (lbl_time_current) {
        format_time(pos, buf, sizeof(buf));
        lv_label_set_text(lbl_time_current, buf);
    }
    if (lbl_time_total) {
        if (dur > 0) {
            format_time(dur, buf, sizeof(buf));
            lv_label_set_text(lbl_time_total, buf);
        } else {
            lv_label_set_text(lbl_time_total, "--:--");
        }
    }

    /* Sleep-timer countdown. */
    if (lbl_sleep && g_sleep_minutes > 0) {
        uint32_t now = lv_tick_get();
        if (now >= g_sleep_deadline_ms) {
            printf("Sleep: expired — pausing\n");
            meck_audio_pause();
            g_sleep_minutes = 0;
            g_sleep_deadline_ms = 0;
            lv_label_set_text(lbl_sleep, LV_SYMBOL_BELL " Sleep: OFF");
        } else {
            uint32_t remain_s = (g_sleep_deadline_ms - now) / 1000u;
            char b[32];
            uint32_t mm = remain_s / 60, ss = remain_s % 60;
            snprintf(b, sizeof(b), LV_SYMBOL_BELL " Sleep in %u:%02u", mm, ss);
            lv_label_set_text(lbl_sleep, b);
        }
    } else if (lbl_sleep) {
        lv_label_set_text(lbl_sleep, LV_SYMBOL_BELL " Sleep: OFF");
    }

    /* EOF — advance to next playlist track, or stop. */
    if (meck_audio_consume_eof_flag()) {
        if (!g_playlist.empty() && g_playlist_idx >= 0 &&
            g_playlist_idx < (int)g_playlist.size() - 1) {
            g_playlist_idx++;
            const std::string& next = g_playlist[g_playlist_idx];
            strncpy(g_current_file, next.c_str(), sizeof(g_current_file));
            g_current_file[sizeof(g_current_file) - 1] = '\0';
            uint32_t resume = meck_audio_bookmark_load_sec(g_current_file);
            meck_audio_play_file(g_current_file, resume);

            /* Update title labels for the new track. */
            if (lbl_player_title) {
                char title[160];
                file_title_from_path(g_current_file, title, sizeof(title));
                lv_label_set_text(lbl_player_title, title);
            }
            if (lbl_player_subtitle) {
                char sub[200];
                build_subtitle(g_current_file, sub, sizeof(sub));
                lv_label_set_text(lbl_player_subtitle, sub);
            }
            printf("MeckAudioUI: EOF — advanced to '%s'\n", g_current_file);
        } else {
            printf("MeckAudioUI: EOF — end of playlist\n");
        }
    }
}

/* ----- screen build (replaces placeholder) ---------------------------------*/

static void on_player_back_clicked(lv_event_t* e) {
    (void)e;
    if (scr_audio_browser) lv_screen_load(scr_audio_browser);
}

/* Helper: round button with icon glyph. */
static lv_obj_t* make_round_button(lv_obj_t* parent, const char* glyph,
                                   int size, lv_event_cb_t cb,
                                   lv_obj_t** glyph_label_out = NULL) {
    lv_obj_t* btn = lv_button_create(parent);
    lv_obj_set_size(btn, size, size);
    lv_obj_set_style_radius(btn, size / 2, 0);
    lv_obj_set_style_bg_color(btn, lv_color_make(50, 50, 50), 0);
    lv_obj_set_style_shadow_width(btn, 0, 0);
    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, glyph);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl,
        size >= 110 ? &lv_font_montserrat_36 : &lv_font_montserrat_24, 0);
    lv_obj_center(lbl);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    if (glyph_label_out) *glyph_label_out = lbl;
    return btn;
}

/* ============================================================================
 * Album art
 *
 * Looks for cover art alongside the playing audio file. Standard cover
 * filenames in priority order: cover.jpg, cover.png, folder.jpg,
 * folder.png, front.jpg, front.png, album.jpg, album.png. Matched
 * case-insensitively so cover.JPG from a Windows ripper works too.
 *
 * Requires LVGL filesystem + image decoder support in lv_conf.h:
 *   #define LV_USE_FS_POSIX   1
 *   #define LV_FS_POSIX_LETTER 'A'    (or whatever letter is unused)
 *   #define LV_USE_TJPGD       1      (JPG support, software, ~5KB)
 *   #define LV_USE_LODEPNG     1      (PNG support, software)
 *
 * If those aren't enabled, lv_image_set_src() silently fails and the
 * placeholder music-note icon remains visible — no crash.
 *
 * The cpp_bus_driver / LilyGo base setup may already register a POSIX
 * FS driver under a different letter. If covers don't appear after
 * enabling the configs above, grep your lv_conf.h for LV_FS_POSIX_LETTER
 * and adjust LVGL_FS_LETTER below to match.
 * ==========================================================================*/

#define LVGL_FS_LETTER 'A'

/* Case-insensitive strcmp for filename matching on FAT (where case is
 * preserved on disk but should match user expectations). */
static int cover_strcasecmp(const char* a, const char* b) {
    while (*a && *b) {
        int ca = tolower((unsigned char)*a);
        int cb = tolower((unsigned char)*b);
        if (ca != cb) return ca - cb;
        a++; b++;
    }
    return (unsigned char)*a - (unsigned char)*b;
}

/* Strip filename from a full path, leaving just the directory portion.
 * Modifies the buffer in place: "/sdcard/audio/foo/bar.mp3" → "/sdcard/audio/foo".
 * Returns true on success. */
static bool dir_of(char* path) {
    char* last_slash = strrchr(path, '/');
    if (!last_slash) return false;
    *last_slash = '\0';
    return true;
}

/* Search `dir` for any of the standard cover filenames. On hit, write
 * the full path (dir + "/" + filename) to `out` and return true. */
static bool find_cover_in_dir(const char* dir, char* out, size_t out_len) {
    static const char* candidates[] = {
        "cover.jpg", "cover.png",
        "folder.jpg", "folder.png",
        "front.jpg", "front.png",
        "album.jpg", "album.png",
    };
    static const size_t n_candidates = sizeof(candidates) / sizeof(candidates[0]);

    DIR* d = opendir(dir);
    if (!d) return false;

    bool found = false;
    struct dirent* de;
    while ((de = readdir(d)) != NULL) {
        if (de->d_name[0] == '.') continue;
        for (size_t i = 0; i < n_candidates; i++) {
            if (cover_strcasecmp(de->d_name, candidates[i]) == 0) {
                snprintf(out, out_len, "%s/%s", dir, de->d_name);
                found = true;
                break;
            }
        }
        if (found) break;
    }
    closedir(d);
    return found;
}

/* Load album art for the currently-playing track. Looks in the same
 * directory as the audio file. Falls back to the placeholder icon if no
 * cover is found, or if the image fails to decode.
 *
 * Safe to call repeatedly — the LVGL image widget is created on first
 * call and reused on subsequent calls (just src is swapped). */
static void load_album_art(const char* audio_path) {
    if (!audio_path || !obj_cover_slot) return;

    char dir[256];
    strncpy(dir, audio_path, sizeof(dir));
    dir[sizeof(dir) - 1] = '\0';
    if (!dir_of(dir)) {
        printf("MeckAudioUI: load_album_art: no directory in '%s'\n", audio_path);
        return;
    }

    char cover_path[300];
    if (!find_cover_in_dir(dir, cover_path, sizeof(cover_path))) {
        /* No cover file present. Make sure the placeholder is visible
         * and any prior cover image is hidden. */
        if (obj_cover_image) {
            lv_obj_add_flag(obj_cover_image, LV_OBJ_FLAG_HIDDEN);
        }
        if (obj_cover_icon) {
            lv_obj_clear_flag(obj_cover_icon, LV_OBJ_FLAG_HIDDEN);
        }
        printf("MeckAudioUI: no cover art found in '%s'\n", dir);
        return;
    }

    /* Build LVGL filesystem path: "A:/sdcard/audio/.../cover.jpg".
     * LVGL needs the drive-letter prefix to dispatch to the right FS
     * driver. The actual letter is set in lv_conf.h via
     * LV_FS_POSIX_LETTER (or LV_FS_FATFS_LETTER if using FATFS bridge). */
    char lvgl_path[320];
    snprintf(lvgl_path, sizeof(lvgl_path), "%c:%s", LVGL_FS_LETTER, cover_path);

    /* Pre-flight check: ask LVGL whether a decoder can claim this source
     * BEFORE we swap widget visibility. If FS or JPG decoder aren't
     * configured in lv_conf.h, get_info returns failure and we keep the
     * placeholder icon visible rather than hiding it over a blank slot. */
    lv_image_header_t hdr;
    if (lv_image_decoder_get_info(lvgl_path, &hdr) != LV_RESULT_OK) {
        printf("MeckAudioUI: cover art decode unavailable for '%s' — "
               "check lv_conf.h has LV_USE_FS_POSIX, LV_FS_POSIX_LETTER, "
               "and LV_USE_TJPGD/LV_USE_LODEPNG enabled, and that "
               "LV_FS_POSIX_LETTER matches LVGL_FS_LETTER ('%c').\n",
               lvgl_path, LVGL_FS_LETTER);
        if (obj_cover_image) {
            lv_obj_add_flag(obj_cover_image, LV_OBJ_FLAG_HIDDEN);
        }
        if (obj_cover_icon) {
            lv_obj_clear_flag(obj_cover_icon, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }

    /* Lazy-create the image widget on first call. After that we just
     * swap the src so we're not creating/destroying objects per track. */
    if (!obj_cover_image) {
        obj_cover_image = lv_image_create(obj_cover_slot);
        lv_obj_center(obj_cover_image);
        /* Inner-image-only — don't let an oversized source overflow the
         * slot's rounded border. LVGL 9 honours the parent clip rect. */
        lv_obj_clear_flag(obj_cover_image, LV_OBJ_FLAG_SCROLLABLE);
    }

    lv_image_set_src(obj_cover_image, lvgl_path);
    lv_obj_clear_flag(obj_cover_image, LV_OBJ_FLAG_HIDDEN);
    if (obj_cover_icon) {
        lv_obj_add_flag(obj_cover_icon, LV_OBJ_FLAG_HIDDEN);
    }
    printf("MeckAudioUI: loaded cover art '%s' (%lux%lu)\n",
           lvgl_path, (unsigned long)hdr.w, (unsigned long)hdr.h);
}


static void create_player_screen() {
    scr_audio_player = lv_obj_create(NULL);
    lock_screen_scroll(scr_audio_player);
    lv_obj_set_style_bg_color(scr_audio_player, lv_color_black(), 0);

    /* Back button */
    lv_obj_t* btn_back = lv_button_create(scr_audio_player);
    lv_obj_set_size(btn_back, 80, 40);
    lv_obj_align(btn_back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_bg_color(btn_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_back, 8, 0);
    lv_obj_t* bl = lv_label_create(btn_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    lv_obj_set_style_text_font(bl, &lv_font_montserrat_16, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_back, on_player_back_clicked,
                        LV_EVENT_CLICKED, NULL);

    /* Title in header */
    lv_obj_t* hdr_title = lv_label_create(scr_audio_player);
    lv_label_set_text(hdr_title, "Now Playing");
    lv_obj_set_style_text_color(hdr_title, lv_palette_main(LV_PALETTE_AMBER), 0);
    lv_obj_set_style_text_font(hdr_title, &lv_font_montserrat_24, 0);
    lv_obj_align(hdr_title, LV_ALIGN_TOP_LEFT, 100, 18);

    /* Cover slot — 280×280 placeholder until cover art lands.
     * Centered horizontally, near the top of content area. */
    const int COVER_W = 280;
    const int COVER_H = 280;
    obj_cover_slot = lv_obj_create(scr_audio_player);
    lv_obj_set_size(obj_cover_slot, COVER_W, COVER_H);
    lv_obj_align(obj_cover_slot, LV_ALIGN_TOP_MID, 0, 80);
    lv_obj_set_style_bg_color(obj_cover_slot, lv_color_make(20, 20, 20), 0);
    lv_obj_set_style_border_width(obj_cover_slot, 2, 0);
    lv_obj_set_style_border_color(obj_cover_slot,
                                  lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_radius(obj_cover_slot, 12, 0);
    lv_obj_clear_flag(obj_cover_slot, LV_OBJ_FLAG_SCROLLABLE);
    /* Audio glyph centered in the placeholder. Kept as a fallback so the
     * slot doesn't look empty when no cover.jpg is found in the album
     * folder. Hidden by load_album_art() when a real image takes over. */
    obj_cover_icon = lv_label_create(obj_cover_slot);
    lv_label_set_text(obj_cover_icon, LV_SYMBOL_AUDIO);
    lv_obj_set_style_text_color(obj_cover_icon, lv_palette_main(LV_PALETTE_GREY), 0);
    /* lv_font_montserrat_48 isn't in the default LVGL font set; use the
     * largest standard size (36) here. */
    lv_obj_set_style_text_font(obj_cover_icon, &lv_font_montserrat_36, 0);
    lv_obj_center(obj_cover_icon);

    /* Title label */
    lbl_player_title = lv_label_create(scr_audio_player);
    lv_label_set_text(lbl_player_title, "");
    lv_label_set_long_mode(lbl_player_title, LV_LABEL_LONG_DOT);
    lv_obj_set_width(lbl_player_title, SCREEN_WIDTH - 40);
    lv_obj_set_style_text_align(lbl_player_title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(lbl_player_title, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_player_title, &lv_font_montserrat_24, 0);
    lv_obj_align(lbl_player_title, LV_ALIGN_TOP_MID, 0, 380);

    /* Subtitle: folder · MODE */
    lbl_player_subtitle = lv_label_create(scr_audio_player);
    lv_label_set_text(lbl_player_subtitle, "");
    lv_label_set_long_mode(lbl_player_subtitle, LV_LABEL_LONG_DOT);
    lv_obj_set_width(lbl_player_subtitle, SCREEN_WIDTH - 40);
    lv_obj_set_style_text_align(lbl_player_subtitle, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(lbl_player_subtitle,
                                lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(lbl_player_subtitle, &lv_font_montserrat_16, 0);
    lv_obj_align(lbl_player_subtitle, LV_ALIGN_TOP_MID, 0, 420);

    /* Progress slider */
    slider_progress = lv_slider_create(scr_audio_player);
    lv_obj_set_size(slider_progress, SCREEN_WIDTH - 80, 8);
    lv_obj_align(slider_progress, LV_ALIGN_TOP_MID, 0, 470);
    lv_slider_set_range(slider_progress, 0, 1);
    lv_slider_set_value(slider_progress, 0, LV_ANIM_OFF);
    lv_obj_add_event_cb(slider_progress, on_progress_pressed,
                        LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(slider_progress, on_progress_released,
                        LV_EVENT_RELEASED, NULL);

    /* Time labels */
    lbl_time_current = lv_label_create(scr_audio_player);
    lv_label_set_text(lbl_time_current, "0:00");
    lv_obj_set_style_text_color(lbl_time_current,
                                lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(lbl_time_current, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_time_current, LV_ALIGN_TOP_LEFT, 40, 490);

    lbl_time_total = lv_label_create(scr_audio_player);
    lv_label_set_text(lbl_time_total, "--:--");
    lv_obj_set_style_text_color(lbl_time_total,
                                lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(lbl_time_total, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_time_total, LV_ALIGN_TOP_RIGHT, -40, 490);

    /* Transport row: -30, play/pause, +30 — large tap targets, centered */
    const int TRANSPORT_Y = 550;
    btn_seek_back  = make_round_button(scr_audio_player, LV_SYMBOL_LEFT,   80,
                                       on_seek_back_clicked);
    lv_obj_align(btn_seek_back, LV_ALIGN_TOP_MID, -130, TRANSPORT_Y);

    btn_play_pause = make_round_button(scr_audio_player, LV_SYMBOL_PLAY, 120,
                                       on_play_pause_clicked,
                                       &lbl_play_pause_icon);
    lv_obj_align(btn_play_pause, LV_ALIGN_TOP_MID, 0, TRANSPORT_Y - 20);

    btn_seek_fwd   = make_round_button(scr_audio_player, LV_SYMBOL_RIGHT,  80,
                                       on_seek_fwd_clicked);
    lv_obj_align(btn_seek_fwd, LV_ALIGN_TOP_MID, 130, TRANSPORT_Y);

    /* Tiny "-30s" / "+30s" labels under the seek buttons */
    lv_obj_t* lbl_b = lv_label_create(scr_audio_player);
    lv_label_set_text(lbl_b, "-30s");
    lv_obj_set_style_text_color(lbl_b, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(lbl_b, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_b, LV_ALIGN_TOP_MID, -130, TRANSPORT_Y + 90);

    lv_obj_t* lbl_f = lv_label_create(scr_audio_player);
    lv_label_set_text(lbl_f, "+30s");
    lv_obj_set_style_text_color(lbl_f, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_text_font(lbl_f, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_f, LV_ALIGN_TOP_MID, 130, TRANSPORT_Y + 90);

    /* Volume row */
    const int VOL_Y = 730;
    lv_obj_t* vol_icon = lv_label_create(scr_audio_player);
    lv_label_set_text(vol_icon, LV_SYMBOL_VOLUME_MAX);
    lv_obj_set_style_text_color(vol_icon, lv_color_white(), 0);
    lv_obj_set_style_text_font(vol_icon, &lv_font_montserrat_18, 0);
    lv_obj_align(vol_icon, LV_ALIGN_TOP_LEFT, 30, VOL_Y);

    slider_volume = lv_slider_create(scr_audio_player);
    lv_obj_set_size(slider_volume, SCREEN_WIDTH - 160, 8);
    lv_obj_align(slider_volume, LV_ALIGN_TOP_LEFT, 70, VOL_Y + 8);
    lv_slider_set_range(slider_volume, 0, 100);
    lv_slider_set_value(slider_volume, 50, LV_ANIM_OFF);
    lv_obj_add_event_cb(slider_volume, on_volume_changed,
                        LV_EVENT_VALUE_CHANGED, NULL);

    lbl_volume = lv_label_create(scr_audio_player);
    lv_label_set_text(lbl_volume, "50%");
    lv_obj_set_style_text_color(lbl_volume, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_volume, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_volume, LV_ALIGN_TOP_RIGHT, -30, VOL_Y);

    /* Sleep-timer chip (visible only in audiobook mode) */
    btn_sleep = lv_button_create(scr_audio_player);
    lv_obj_set_size(btn_sleep, SCREEN_WIDTH - 80, 50);
    lv_obj_align(btn_sleep, LV_ALIGN_TOP_MID, 0, 800);
    lv_obj_set_style_bg_color(btn_sleep, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_sleep, 25, 0);
    lbl_sleep = lv_label_create(btn_sleep);
    lv_label_set_text(lbl_sleep, LV_SYMBOL_BELL " Sleep: OFF");
    lv_obj_set_style_text_color(lbl_sleep, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_sleep, &lv_font_montserrat_16, 0);
    lv_obj_center(lbl_sleep);
    lv_obj_add_event_cb(btn_sleep, on_sleep_clicked, LV_EVENT_CLICKED, NULL);

    /* 4 Hz refresh while the player is the active screen. */
    player_refresh_timer = lv_timer_create(player_refresh_cb, 250, NULL);
}

/* ============================================================================
 * Public API
 * ==========================================================================*/

/* Create the canonical audio folder tree on first boot if it isn't there.
 * /sdcard/audio + /sdcard/audio/music + /sdcard/audio/audiobooks.
 *
 * EEXIST is fine — we only care about the end state, not who created it.
 * Any other error (e.g. SD card not mounted, full, write-protected) is
 * logged but not fatal: the browser screen will still load and just show
 * its "no audio yet" message instead of an empty file list.
 */
static void ensure_audio_dirs(void) {
    const char* paths[] = { AUDIO_ROOT, MUSIC_PREFIX, BOOKS_PREFIX };
    for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); i++) {
        const char* p = paths[i];
        if (mkdir(p, 0777) == 0) {
            printf("MeckAudioUI: created '%s'\n", p);
        } else if (errno != EEXIST) {
            printf("MeckAudioUI: mkdir('%s') failed: %s\n",
                   p, strerror(errno));
        }
        /* errno == EEXIST: directory was already there, nothing to do. */
    }
}

extern "C" void meck_audio_ui_init(void) {
    static bool inited = false;
    if (inited) return;

    ensure_audio_dirs();   /* TEMP DISABLED for audio-hang debugging */
    create_browser_screen();
    create_player_screen();
    browser_repopulate();

    inited = true;
    printf("MeckAudioUI: browser + player screens built\n");
}

extern "C" void meck_audio_ui_show_browser(void) {
    if (!scr_audio_browser) {
        printf("MeckAudioUI: show_browser before init — ignored\n");
        return;
    }
    browser_repopulate();
    lv_screen_load(scr_audio_browser);
}

extern "C" void meck_audio_ui_show_player(void) {
    if (!scr_audio_player) return;

    g_player_mode = infer_library_mode(g_current_file);

    /* Title + subtitle */
    if (lbl_player_title) {
        char title[160];
        file_title_from_path(g_current_file, title, sizeof(title));
        lv_label_set_text(lbl_player_title, title);
    }
    if (lbl_player_subtitle) {
        char sub[200];
        build_subtitle(g_current_file, sub, sizeof(sub));
        lv_label_set_text(lbl_player_subtitle, sub);
    }

    /* Album art — looks for cover.jpg/png/folder.jpg/etc. in the same
     * directory as the audio file. Falls back to the placeholder icon
     * if no cover is found or the decoder isn't enabled. Safe no-op if
     * LVGL FS / image decoders aren't configured. */
    load_album_art(g_current_file);

    /* Sleep-timer chip visibility */
    if (btn_sleep) {
        if (g_player_mode == LIB_AUDIOBOOK) {
            lv_obj_clear_flag(btn_sleep, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(btn_sleep, LV_OBJ_FLAG_HIDDEN);
            g_sleep_minutes = 0;
            g_sleep_deadline_ms = 0;
        }
    }

    /* Reset progress slider for new file */
    if (slider_progress) {
        lv_slider_set_range(slider_progress, 0, 1);
        lv_slider_set_value(slider_progress, 0, LV_ANIM_OFF);
    }

    /* Sync volume slider with backend's persisted value */
    if (slider_volume) {
        uint8_t v = meck_audio_get_volume_pct();
        lv_slider_set_value(slider_volume, (int32_t)v, LV_ANIM_OFF);
        if (lbl_volume) {
            char b[16];
            snprintf(b, sizeof(b), "%u%%", (unsigned)v);
            lv_label_set_text(lbl_volume, b);
        }
    }

    /* Build the same-folder playlist and kick off playback. */
    build_playlist_for(g_current_file);

    uint32_t resume = meck_audio_bookmark_load_sec(g_current_file);
    printf("MeckAudioUI: opening file (resume=%u): %s\n", resume, g_current_file);
    meck_audio_play_file(g_current_file, resume);

    lv_screen_load(scr_audio_player);
}
/*
 * MeckMapScreen.cpp — LVGL map screen implementation
 *
 * Session 1 scope (this file at this revision):
 *   - Screen scaffolding: back button, title, toolbar with disabled
 *     Recenter / Zoom-out / Zoom-in / Filter buttons
 *   - Slippy map math (latLonToTileXY, etc.) ported from upstream
 *   - SD scan to detect available zoom range
 *   - Static tile rendering: compute viewport tiles, place lv_image
 *     widgets at the right positions, set src to A:/sdcard paths
 *   - Initial center from P4NodePrefs (or Sydney CBD fallback)
 *
 * Session 2 (later): touch drag pan, zoom button bodies, last-viewed
 * persistence.
 *
 * Session 3 (later): GPS dot, repeater markers, filter modal.
 *
 * The tile pool is a small fixed array of lv_image widgets. Session 1
 * creates exactly the number of widgets needed to fill the visible
 * grid on the first render and leaves them there — no recycling yet.
 */

#include "MeckMapScreen.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/stat.h>

#include "lvgl.h"
#include "esp_log.h"
#include "t_display_p4_driver.h"

#include "target.h"
#include "MeckMesh.h"
#include "MeckDataStore.h"
#include "MeckUI.h"

// Debug Logs: rewrites printf -> meck_debug_log_printf so calls below
// land in the SD log file when Settings > Debug Logs > Start is active.
// See meck_log.h for the macro mechanism.
#include "meck_log.h"

// ----------------------------------------------------------------------------
// External symbols from MeckUI.cpp.
// meck_get_instance() is declared in target.h (included via MeckMesh.h
// below), so we don't redeclare it here — doing so caused a linkage
// conflict (target.h declares it C++, redeclaring as extern "C" is a
// mismatch). meck_ui_show_home() is declared in MeckUI.h (included
// below). SCREEN_WIDTH / SCREEN_HEIGHT come in via the LilyGo driver
// headers (HI8561 / RM69A10 panel constants); they're pulled in
// transitively through MeckMesh.h's include chain. NOTCH_SAFE_X/Y are
// MeckUI's convention for keeping content away from the camera
// punch-hole.
// ----------------------------------------------------------------------------

extern const lv_font_t meck_montserrat_16;
extern const lv_font_t meck_montserrat_18;
extern const lv_font_t meck_montserrat_22;
extern const lv_font_t meck_montserrat_24;

#ifndef NOTCH_SAFE_X
#define NOTCH_SAFE_X 20
#endif
#ifndef NOTCH_SAFE_Y
#define NOTCH_SAFE_Y 20
#endif

// ----------------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------------

// Standard OSM tile dimensions (slippy map convention).
#define MECK_MAP_TILE_PX            256

// SD card tile root (POSIX path used for stat()).
#define MECK_MAP_TILE_ROOT_FS       "/sdcard/tiles"

// LVGL-flavoured tile root (drive letter prefix selects the POSIX FS
// driver registered via LV_FS_POSIX_LETTER in lv_conf.h; the audio UI
// uses the same 'A' letter for cover art).
#define MECK_MAP_TILE_ROOT_LVGL     "A:/sdcard/tiles"

// Sydney CBD fallback when no persisted center is available. The lat
// is approximately Town Hall.
#define MECK_MAP_DEFAULT_LAT        (-33.8688)
#define MECK_MAP_DEFAULT_LON        (151.2093)
#define MECK_MAP_DEFAULT_ZOOM       13

// Scan range when detecting which /sdcard/tiles/{z}/ directories
// actually exist. Bounded to keep the boot scan cheap; OSM tile sets
// rarely go higher than 18.
#define MECK_MAP_SCAN_ZOOM_MIN      1
#define MECK_MAP_SCAN_ZOOM_MAX      18

// Maximum number of tile lv_image widgets we'll create. Sized to cover
// a viewport up to ~1024px on the long dimension with a 1-tile margin
// on each side: (1024/256 + 2) ^2 ≈ 36. 48 leaves headroom and is
// still a modest PSRAM cost for the widget structs themselves
// (decoded image data is held in LVGL's image cache, not the widgets).
#define MECK_MAP_MAX_TILE_WIDGETS   48

// Toolbar and header are fixed-height strips at the top and bottom of
// the screen. Tile viewport occupies what's left.
#define MECK_MAP_HEADER_H           80
#define MECK_MAP_TOOLBAR_H          90

// Marker pool size. One pair of widgets (circle + optional label) per
// contact visible in the current viewport. 64 is generous for the
// foreseeable zoom levels; at low zooms many contacts may fall inside
// but they cluster pixel-wise, so the cap really only affects extreme
// edge cases. Excess contacts are silently skipped with a one-line log.
#define MECK_MAP_MAX_MARKERS        64

// ADV_TYPE constants come from upstream MeshCore. Guarded include so we
// don't double-define if the upstream header is already in scope.
#ifndef ADV_TYPE_REPEATER
  #define ADV_TYPE_NONE       0
  #define ADV_TYPE_CHAT       1
  #define ADV_TYPE_REPEATER   2
  #define ADV_TYPE_ROOM       3
  #define ADV_TYPE_SENSOR     4
#endif

// Filter bitmask stored in P4NodePrefs::map_filter_mask. Bit 7 is the
// "value has been written" sentinel — if it's clear, treat the whole
// byte as unset and fall back to the repeater-only default. Bit 0 in
// user-facing UI is labelled "Companion" (the wire protocol still calls
// it CHAT — same advert type, different label).
#define MECK_MAP_FILTER_COMPANION   (1 << 0)
#define MECK_MAP_FILTER_REPEATER    (1 << 1)
#define MECK_MAP_FILTER_ROOM        (1 << 2)
#define MECK_MAP_FILTER_SENSOR      (1 << 3)
#define MECK_MAP_FILTER_SET         (1 << 7)
#define MECK_MAP_FILTER_DEFAULT     (MECK_MAP_FILTER_REPEATER | MECK_MAP_FILTER_SET)

// ----------------------------------------------------------------------------
// Screen state (single instance — there is only one map screen)
// ----------------------------------------------------------------------------

static bool       g_map_inited                 = false;
static lv_obj_t*  scr_map                      = NULL;
static lv_obj_t*  obj_map_tiles_container      = NULL;
static lv_obj_t*  lbl_map_status               = NULL;

// Toolbar handles (created in session 1, bodies wired in session 2/3)
static lv_obj_t*  btn_map_back                 = NULL;
static lv_obj_t*  btn_map_recenter             = NULL;
static lv_obj_t*  btn_map_zoom_out             = NULL;
static lv_obj_t*  btn_map_zoom_in              = NULL;
static lv_obj_t*  btn_map_filter               = NULL;

// Tile widget pool. Each visible tile uses one widget. Session 1
// creates them on first show and leaves them; session 2 will recycle.
static lv_obj_t*  g_tile_widgets[MECK_MAP_MAX_TILE_WIDGETS] = {NULL};
static int        g_tile_widget_count = 0;

// Map viewport state.
static double     g_map_center_lat       = MECK_MAP_DEFAULT_LAT;
static double     g_map_center_lon       = MECK_MAP_DEFAULT_LON;
static int        g_map_zoom             = MECK_MAP_DEFAULT_ZOOM;
static int        g_map_zoom_min         = MECK_MAP_DEFAULT_ZOOM;
static int        g_map_zoom_max         = MECK_MAP_DEFAULT_ZOOM;
static bool       g_map_zoom_range_known = false;

// Per-show flag so persisted state load + initial render happen lazily
// only once (on the first show after init).
static bool       g_map_first_show       = true;

// Dirty flag for persistence — set when pan or zoom changes the
// viewport, cleared after savePrefs runs on screen exit. Avoids
// hammering NVS on every interaction; one write per visit to the map
// screen at most.
static bool       g_map_dirty            = false;

// Pan-by-drag state. Captured on LV_EVENT_PRESSED, used by
// LV_EVENT_PRESSING to translate tile widgets, finalised on
// LV_EVENT_RELEASED to update the geographic center.
static bool       g_drag_active          = false;
static lv_point_t g_drag_start_point     = {0, 0};
static int        g_tile_orig_x[MECK_MAP_MAX_TILE_WIDGETS] = {0};
static int        g_tile_orig_y[MECK_MAP_MAX_TILE_WIDGETS] = {0};

// GPS "you are here" dot. Lives as a child of obj_map_tiles_container
// so it clips to the viewport, sits above tile widgets but below the
// toolbar. update_gps_dot positions it from the latest snapshot and
// shows/hides it based on fix validity and on-screen visibility.
// g_gps_dot_orig_{x,y} mirror g_tile_orig_x/y so pan-by-drag can
// translate the dot in lockstep with tiles.
static lv_obj_t*  g_gps_dot              = NULL;
static int        g_gps_dot_orig_x       = 0;
static int        g_gps_dot_orig_y       = 0;
static lv_timer_t* g_gps_timer           = NULL;

// Contact markers (repeaters by default). One circle widget per visible
// contact, plus an optional label widget at zoom >= 14. Both widgets
// for marker i live at index i in their respective pools. Translated
// alongside tiles during pan, then fully rebuilt by render_markers()
// at every render_tiles() call (which fires on pan-release, zoom,
// recenter, and initial show).
static lv_obj_t*  g_marker_circles[MECK_MAP_MAX_MARKERS]    = {NULL};
static lv_obj_t*  g_marker_labels[MECK_MAP_MAX_MARKERS]     = {NULL};
static int        g_marker_count                            = 0;
static int        g_marker_orig_x[MECK_MAP_MAX_MARKERS]     = {0};
static int        g_marker_orig_y[MECK_MAP_MAX_MARKERS]     = {0};
static int        g_marker_label_orig_x[MECK_MAP_MAX_MARKERS] = {0};
static int        g_marker_label_orig_y[MECK_MAP_MAX_MARKERS] = {0};

// strip_unrenderable lives in MeckUI.cpp (drops UTF-8 codepoints above
// U+00FF — emoji, CJK, etc. — that the Montserrat fonts can't render).
// Shared via extern, same convention as lock_screen_scroll.
extern "C" void strip_unrenderable(const char *src, char *dst, size_t dst_sz);

// Filter modal. A semi-transparent backdrop covers the screen and
// blocks taps to the map; a centered card holds the 4 type toggles
// (with colour-coded dots), a Clear button, and Apply / Close buttons.
// Built hidden in create_map_screen, shown by the Filter toolbar
// button, dismissed by Close or tap-outside.
//
// g_filter_switches[i] corresponds to ADV_TYPE (i+1):
//   [0] = ADV_TYPE_CHAT (UI label: "Companion")
//   [1] = ADV_TYPE_REPEATER
//   [2] = ADV_TYPE_ROOM
//   [3] = ADV_TYPE_SENSOR
static lv_obj_t*  g_filter_backdrop       = NULL;
static lv_obj_t*  g_filter_card           = NULL;
static lv_obj_t*  g_filter_switches[4]    = {NULL};

// ----------------------------------------------------------------------------
// Forward declarations
// ----------------------------------------------------------------------------

static void create_map_screen(void);
static void detect_zoom_range(void);
static void load_persisted_state(void);
static void render_tiles(void);
static void clear_tile_widgets(void);
static void update_status_label(void);

static void on_map_back(lv_event_t* e);
static void on_map_recenter(lv_event_t* e);
static void on_map_zoom_out(lv_event_t* e);
static void on_map_zoom_in(lv_event_t* e);
static void on_map_filter(lv_event_t* e);

static void latLonToTileXY(double lat, double lon, int zoom,
                            int* tileX, int* tileY,
                            int* pixelX, int* pixelY);
static void pixelToLatLon(double world_px, double world_py, int zoom,
                           double* lat, double* lon);
static void on_tile_container_event(lv_event_t* e);
static void save_state_if_dirty(void);
static void update_gps_dot(void);
static void gps_timer_cb(lv_timer_t* t);
static void clear_markers(void);
static void render_markers(void);
static uint8_t resolve_filter_mask(void);
static lv_color_t color_for_adv_type(uint8_t adv_type);
static void create_filter_modal(void);
static void show_filter_modal(void);
static void hide_filter_modal(void);
static void on_filter_apply(lv_event_t* e);
static void on_filter_clear(lv_event_t* e);
static void on_filter_close(lv_event_t* e);
static void on_filter_backdrop_click(lv_event_t* e);

// ============================================================================
// Slippy map math (pure functions, ported from upstream Mapscreen.h)
// ============================================================================

// Convert lat/lon to tile X,Y at the given zoom, plus the sub-tile
// pixel offset within that tile. tileX/tileY = which tile, pixelX/Y
// = where inside that tile (0..255).
static void latLonToTileXY(double lat, double lon, int zoom,
                            int* tileX, int* tileY,
                            int* pixelX, int* pixelY) {
    int n = 1 << zoom;

    // Longitude is linear across the world: lon=-180 -> x=0,
    // lon=+180 -> x=n.
    double x = (lon + 180.0) / 360.0 * (double)n;
    *tileX  = (int)floor(x);
    *pixelX = (int)((x - *tileX) * (double)MECK_MAP_TILE_PX);

    // Latitude uses Web Mercator. Standard slippy formula.
    double latRad = lat * M_PI / 180.0;
    double y = (1.0 - log(tan(latRad) + 1.0 / cos(latRad)) / M_PI) / 2.0 * (double)n;
    *tileY  = (int)floor(y);
    *pixelY = (int)((y - *tileY) * (double)MECK_MAP_TILE_PX);
}

// Inverse of latLonToTileXY. Given a world-pixel position (i.e.
// position in the global "all tiles laid out edge-to-edge" coordinate
// system at the given zoom), return the corresponding geographic
// coordinate. Used to convert pan-drag pixel deltas back into a new
// map center.
static void pixelToLatLon(double world_px, double world_py, int zoom,
                           double* lat, double* lon) {
    double n = (double)(1 << zoom);
    double tile_x = world_px / (double)MECK_MAP_TILE_PX;
    double tile_y = world_py / (double)MECK_MAP_TILE_PX;
    *lon = (tile_x / n) * 360.0 - 180.0;
    double lat_rad = atan(sinh(M_PI * (1.0 - 2.0 * tile_y / n)));
    *lat = lat_rad * 180.0 / M_PI;
}

// ============================================================================
// Zoom range detection
// ============================================================================

// Scan /sdcard/tiles/{N}/ for N in [SCAN_ZOOM_MIN..SCAN_ZOOM_MAX] and
// record the min and max that actually exist as directories. Cached
// across sessions via g_map_zoom_range_known so we only stat 18
// directories once per boot.
static void detect_zoom_range(void) {
    if (g_map_zoom_range_known) return;

    int found_min = MECK_MAP_SCAN_ZOOM_MAX + 1;
    int found_max = MECK_MAP_SCAN_ZOOM_MIN - 1;

    char path[64];
    for (int z = MECK_MAP_SCAN_ZOOM_MIN; z <= MECK_MAP_SCAN_ZOOM_MAX; z++) {
        snprintf(path, sizeof(path), "%s/%d", MECK_MAP_TILE_ROOT_FS, z);
        struct stat st;
        if (stat(path, &st) == 0 && S_ISDIR(st.st_mode)) {
            if (z < found_min) found_min = z;
            if (z > found_max) found_max = z;
        }
    }

    if (found_min > found_max) {
        // No tile directories on the card. Leave defaults so the
        // status label can surface the situation; render_tiles will
        // try to load tiles anyway and produce empty squares for
        // missing files, making the problem visually obvious.
        g_map_zoom_min = MECK_MAP_DEFAULT_ZOOM;
        g_map_zoom_max = MECK_MAP_DEFAULT_ZOOM;
        printf("MeckMapScreen: no tile dirs under %s\n", MECK_MAP_TILE_ROOT_FS);
    } else {
        g_map_zoom_min = found_min;
        g_map_zoom_max = found_max;
        printf("MeckMapScreen: detected zoom range %d..%d\n",
               g_map_zoom_min, g_map_zoom_max);
    }

    g_map_zoom_range_known = true;
}

// ============================================================================
// Persisted state load (from P4NodePrefs)
// ============================================================================

// Pull last-viewed center from prefs if set, otherwise leave the
// Sydney CBD defaults in place. Clamps the loaded zoom to the
// detected range so a card swap with a different tile set doesn't
// leave us pointing at a missing zoom level.
static void load_persisted_state(void) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    // 0/0 e7 is the "not set" sentinel — fall back to Sydney CBD.
    if (prefs->map_last_lat_e7 != 0 || prefs->map_last_lon_e7 != 0) {
        g_map_center_lat = (double)prefs->map_last_lat_e7 / 1.0e7;
        g_map_center_lon = (double)prefs->map_last_lon_e7 / 1.0e7;
        printf("MeckMapScreen: restored center %.6f, %.6f from prefs\n",
               g_map_center_lat, g_map_center_lon);
    } else {
        printf("MeckMapScreen: no persisted center, using Sydney CBD fallback\n");
    }

    // 0 zoom = not set -> pick middle of detected range, or the
    // default if no range was found.
    if (prefs->map_last_zoom != 0) {
        g_map_zoom = prefs->map_last_zoom;
    } else if (g_map_zoom_range_known && g_map_zoom_max >= g_map_zoom_min) {
        g_map_zoom = (g_map_zoom_min + g_map_zoom_max) / 2;
    } else {
        g_map_zoom = MECK_MAP_DEFAULT_ZOOM;
    }

    // Clamp into the detected range so a stale persisted zoom doesn't
    // point outside available tiles.
    if (g_map_zoom_range_known) {
        if (g_map_zoom < g_map_zoom_min) g_map_zoom = g_map_zoom_min;
        if (g_map_zoom > g_map_zoom_max) g_map_zoom = g_map_zoom_max;
    }
    printf("MeckMapScreen: starting at zoom %d\n", g_map_zoom);
}

// ============================================================================
// Tile rendering — static layout for session 1
// ============================================================================

// Remove and delete all tile widgets we previously created, leaving
// the container empty. Called before each render so we don't leak
// widgets (and so a future pan/zoom rebuild is straightforward).
static void clear_tile_widgets(void) {
    for (int i = 0; i < g_tile_widget_count; i++) {
        if (g_tile_widgets[i]) {
            lv_obj_delete(g_tile_widgets[i]);
            g_tile_widgets[i] = NULL;
        }
    }
    g_tile_widget_count = 0;
}

// Render the tile grid for the current center + zoom. Computes which
// tiles fall within the viewport bounds, creates an lv_image widget
// per tile, and points each at the corresponding A:/sdcard/tiles
// path. Missing tiles surface as transparent gaps — LVGL silently
// fails the load and the slot just shows the container background.
static void render_tiles(void) {
    if (!obj_map_tiles_container) return;

    clear_tile_widgets();

    // Viewport dimensions in pixels. We use compile-time SCREEN_*
    // macros (same value used to size the container at creation time)
    // rather than lv_obj_get_width/height on the container, because in
    // LVGL 9 those return 0 until after a layout pass has run — and
    // render_tiles is called immediately after lv_screen_load, before
    // the next frame.
    int viewport_w = SCREEN_WIDTH;
    int viewport_h = SCREEN_HEIGHT - MECK_MAP_HEADER_H - MECK_MAP_TOOLBAR_H;
    if (viewport_w <= 0 || viewport_h <= 0) {
        printf("MeckMapScreen: render_tiles aborted, bad viewport %dx%d\n",
               viewport_w, viewport_h);
        return;
    }

    // Find which tile the center point lives in, plus its sub-tile
    // pixel offset. The center pixel of the viewport should land at
    // (centerPixelX, centerPixelY) within tile (centerTileX, centerTileY).
    int centerTileX, centerTileY, centerPixelX, centerPixelY;
    latLonToTileXY(g_map_center_lat, g_map_center_lon, g_map_zoom,
                   &centerTileX, &centerTileY, &centerPixelX, &centerPixelY);

    int viewport_cx = viewport_w / 2;
    int viewport_cy = viewport_h / 2;

    // Screen position where the center tile's (0,0) corner should be
    // placed, such that the geographic center ends up at viewport
    // center.
    int baseScreenX = viewport_cx - centerPixelX;
    int baseScreenY = viewport_cy - centerPixelY;

    // Find tile grid range needed to cover the viewport. Expand
    // outward from the center until we've gone past every edge.
    int startDX = 0, startDY = 0, endDX = 0, endDY = 0;
    while (baseScreenX + startDX * MECK_MAP_TILE_PX > 0)             startDX--;
    while (baseScreenY + startDY * MECK_MAP_TILE_PX > 0)             startDY--;
    while (baseScreenX + (endDX + 1) * MECK_MAP_TILE_PX < viewport_w) endDX++;
    while (baseScreenY + (endDY + 1) * MECK_MAP_TILE_PX < viewport_h) endDY++;

    int maxTile = (1 << g_map_zoom) - 1;
    int created = 0;
    int skipped_cap = 0;

    for (int dy = startDY; dy <= endDY; dy++) {
        for (int dx = startDX; dx <= endDX; dx++) {
            int tx = centerTileX + dx;
            int ty = centerTileY + dy;

            // Longitude wraps; latitude doesn't.
            if (tx < 0)         tx += (1 << g_map_zoom);
            if (tx > maxTile)   tx -= (1 << g_map_zoom);
            if (ty < 0 || ty > maxTile) continue;

            // Bail if we've hit the widget pool cap. Should not happen
            // at sensible viewport sizes and zoom levels — log if it
            // does so the cap can be tuned.
            if (g_tile_widget_count >= MECK_MAP_MAX_TILE_WIDGETS) {
                skipped_cap++;
                continue;
            }

            int screenX = baseScreenX + dx * MECK_MAP_TILE_PX;
            int screenY = baseScreenY + dy * MECK_MAP_TILE_PX;

            char src[96];
            snprintf(src, sizeof(src), "%s/%d/%d/%d.png",
                     MECK_MAP_TILE_ROOT_LVGL, g_map_zoom, tx, ty);

            lv_obj_t* img = lv_image_create(obj_map_tiles_container);
            lv_obj_set_pos(img, screenX, screenY);
            lv_image_set_src(img, src);
            lv_obj_clear_flag(img, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_clear_flag(img, LV_OBJ_FLAG_CLICKABLE);

            g_tile_widgets[g_tile_widget_count++] = img;
            created++;
        }
    }

    printf("MeckMapScreen: render_tiles z=%d center=%.6f,%.6f -> %d tiles "
           "(viewport %dx%d, grid dx=[%d..%d] dy=[%d..%d])%s\n",
           g_map_zoom, g_map_center_lat, g_map_center_lon, created,
           viewport_w, viewport_h, startDX, endDX, startDY, endDY,
           skipped_cap > 0 ? " WIDGET CAP HIT" : "");

    // Contact markers go on top of tiles, beneath the GPS dot.
    render_markers();

    // Tile widgets just got recreated. Lift the GPS dot back above
    // them in z-order and reposition it for the new center/zoom.
    if (g_gps_dot) {
        lv_obj_move_foreground(g_gps_dot);
        update_gps_dot();
    }
}

// ============================================================================
// Status label
// ============================================================================

// Debug strip near the top of the screen showing current zoom and
// center. Useful during session 1 testing; will probably stay as a
// small overlay in later sessions too.
static void update_status_label(void) {
    if (!lbl_map_status) return;
    char buf[80];
    snprintf(buf, sizeof(buf), "z%d  %.4f, %.4f",
             g_map_zoom, g_map_center_lat, g_map_center_lon);
    lv_label_set_text(lbl_map_status, buf);
}

// ============================================================================
// Event handlers
// ============================================================================

// meck_ui_show_home() acquires lvgl_api_lock for callers on non-LVGL
// threads. From inside an LVGL event handler the LVGL task already
// holds that lock for the full event-dispatch iteration, so calling
// the locked variant deadlocks. lv_async_call doesn't help — the next
// LVGL tick also runs with the lock held. The correct entry point is
// meck_ui_load_home_screen(), MeckUI's unlocked variant intended for
// in-LVGL-context callers (same pattern MeckAudioUI uses for its back
// button — see MeckAudioUI.cpp).
extern "C" void meck_ui_load_home_screen(void);

static void on_map_back(lv_event_t* e) {
    printf("MeckMapScreen: back tap\n");
    if (g_gps_timer) {
        lv_timer_delete(g_gps_timer);
        g_gps_timer = NULL;
    }
    save_state_if_dirty();
    meck_ui_load_home_screen();
}

// Toolbar handlers — placeholder bodies for session 1. Session 2
// fills these in. Logging makes it visible during testing that the
// button is wired and the tap is firing.
static void on_map_recenter(lv_event_t* e) {
    // Snap the viewport center to the current GPS position. Silent
    // no-op if there's no fix — same convention as the zoom buttons
    // at their limits.
    MeckGpsSnapshot snap;
    meck_gps_get_snapshot(&snap);
    if (!snap.init_ok || !snap.fix_valid) {
        printf("MeckMapScreen: recenter tap ignored (no GPS fix)\n");
        return;
    }
    g_map_center_lat = (double)snap.lat_e7 / 1.0e7;
    g_map_center_lon = (double)snap.lon_e7 / 1.0e7;
    g_map_dirty = true;
    update_status_label();
    render_tiles();
}
static void on_map_zoom_out(lv_event_t* e) {
    // Silent no-op at the bottom of the detected range so repeated taps
    // at the limit don't spam the log.
    if (g_map_zoom_range_known && g_map_zoom <= g_map_zoom_min) return;
    g_map_zoom--;
    g_map_dirty = true;
    update_status_label();
    render_tiles();
}
static void on_map_zoom_in(lv_event_t* e) {
    if (g_map_zoom_range_known && g_map_zoom >= g_map_zoom_max) return;
    g_map_zoom++;
    g_map_dirty = true;
    update_status_label();
    render_tiles();
}
static void on_map_filter(lv_event_t* e) {
    show_filter_modal();
}

// Pan-by-drag handler attached to the tile container. The container is
// made clickable in create_map_screen; LVGL then routes PRESSED /
// PRESSING / RELEASED events here.
//
// PRESSED:  snapshot the touch position and each tile widget's current
//           position so PRESSING can translate them from a known base.
// PRESSING: translate every tile widget by the cumulative pixel delta.
//           No PNG decode happens here — we're just moving existing
//           widgets, which is cheap.
// RELEASED: convert the final pixel delta into a lat/lon shift via
//           inverse slippy math, update g_map_center_lat/lon, mark the
//           viewport dirty for persistence on screen exit, then call
//           render_tiles to rebuild with proper edge tiles loaded.
static void on_tile_container_event(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_PRESSED &&
        code != LV_EVENT_PRESSING &&
        code != LV_EVENT_RELEASED &&
        code != LV_EVENT_PRESS_LOST) {
        return;
    }
    lv_indev_t* indev = lv_indev_active();
    if (!indev) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    if (code == LV_EVENT_PRESSED) {
        g_drag_active = true;
        g_drag_start_point = p;
        for (int i = 0; i < g_tile_widget_count && i < MECK_MAP_MAX_TILE_WIDGETS; i++) {
            if (g_tile_widgets[i]) {
                g_tile_orig_x[i] = lv_obj_get_x(g_tile_widgets[i]);
                g_tile_orig_y[i] = lv_obj_get_y(g_tile_widgets[i]);
            }
        }
        if (g_gps_dot && !lv_obj_has_flag(g_gps_dot, LV_OBJ_FLAG_HIDDEN)) {
            g_gps_dot_orig_x = lv_obj_get_x(g_gps_dot);
            g_gps_dot_orig_y = lv_obj_get_y(g_gps_dot);
        }
        for (int i = 0; i < g_marker_count && i < MECK_MAP_MAX_MARKERS; i++) {
            if (g_marker_circles[i]) {
                g_marker_orig_x[i] = lv_obj_get_x(g_marker_circles[i]);
                g_marker_orig_y[i] = lv_obj_get_y(g_marker_circles[i]);
            }
            if (g_marker_labels[i]) {
                g_marker_label_orig_x[i] = lv_obj_get_x(g_marker_labels[i]);
                g_marker_label_orig_y[i] = lv_obj_get_y(g_marker_labels[i]);
            }
        }
        return;
    }

    if (!g_drag_active) return;

    int dx = p.x - g_drag_start_point.x;
    int dy = p.y - g_drag_start_point.y;

    if (code == LV_EVENT_PRESSING) {
        for (int i = 0; i < g_tile_widget_count && i < MECK_MAP_MAX_TILE_WIDGETS; i++) {
            if (g_tile_widgets[i]) {
                lv_obj_set_pos(g_tile_widgets[i],
                                g_tile_orig_x[i] + dx,
                                g_tile_orig_y[i] + dy);
            }
        }
        if (g_gps_dot && !lv_obj_has_flag(g_gps_dot, LV_OBJ_FLAG_HIDDEN)) {
            lv_obj_set_pos(g_gps_dot,
                            g_gps_dot_orig_x + dx,
                            g_gps_dot_orig_y + dy);
        }
        for (int i = 0; i < g_marker_count && i < MECK_MAP_MAX_MARKERS; i++) {
            if (g_marker_circles[i]) {
                lv_obj_set_pos(g_marker_circles[i],
                                g_marker_orig_x[i] + dx,
                                g_marker_orig_y[i] + dy);
            }
            if (g_marker_labels[i]) {
                lv_obj_set_pos(g_marker_labels[i],
                                g_marker_label_orig_x[i] + dx,
                                g_marker_label_orig_y[i] + dy);
            }
        }
        return;
    }

    // RELEASED or PRESS_LOST: finalise the pan.
    g_drag_active = false;
    if (dx == 0 && dy == 0) return;

    // Convert current center to absolute world-pixel coords at this zoom,
    // shift by the drag delta (dragging right = map content moves right
    // = geographic center shifts LEFT), then convert back to lat/lon.
    double n = (double)(1 << g_map_zoom);
    double world_size = n * (double)MECK_MAP_TILE_PX;

    int tx, ty, px, py;
    latLonToTileXY(g_map_center_lat, g_map_center_lon, g_map_zoom,
                    &tx, &ty, &px, &py);
    double world_px = (double)tx * MECK_MAP_TILE_PX + (double)px - (double)dx;
    double world_py = (double)ty * MECK_MAP_TILE_PX + (double)py - (double)dy;

    // Wrap longitude across the antimeridian, clamp latitude at the poles.
    while (world_px <  0.0)        world_px += world_size;
    while (world_px >= world_size) world_px -= world_size;
    if    (world_py <  0.0)        world_py = 0.0;
    if    (world_py >= world_size) world_py = world_size - 1.0;

    pixelToLatLon(world_px, world_py, g_map_zoom,
                  &g_map_center_lat, &g_map_center_lon);

    g_map_dirty = true;
    update_status_label();
    render_tiles();
}

// Persist current viewport (center + zoom) to P4NodePrefs via the
// data store. Called from on_map_back when leaving the map screen.
// Cheap no-op if nothing has changed since the last save.
static void save_state_if_dirty(void) {
    if (!g_map_dirty) return;
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;
    prefs->map_last_lat_e7 = (int32_t)(g_map_center_lat * 1.0e7);
    prefs->map_last_lon_e7 = (int32_t)(g_map_center_lon * 1.0e7);
    prefs->map_last_zoom   = (uint8_t)g_map_zoom;
    mesh->savePrefs();
    g_map_dirty = false;
    printf("MeckMapScreen: persisted center=%.6f,%.6f zoom=%d\n",
           g_map_center_lat, g_map_center_lon, g_map_zoom);
}

// Read the latest GPS snapshot and position the "you are here" dot,
// or hide it if there's no fix or the user's position is outside the
// current viewport. Cheap to call — the snapshot copy is a few dozen
// bytes and the math is the same world-pixel calculation used for
// pan release.
static void update_gps_dot(void) {
    if (!g_gps_dot || !obj_map_tiles_container) return;

    MeckGpsSnapshot snap;
    meck_gps_get_snapshot(&snap);
    if (!snap.init_ok || !snap.fix_valid) {
        lv_obj_add_flag(g_gps_dot, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    // Snapshot stores lat/lon as int32 * 1e7. Contacts store * 1e6.
    // Don't conflate the two.
    double user_lat = (double)snap.lat_e7 / 1.0e7;
    double user_lon = (double)snap.lon_e7 / 1.0e7;

    int u_tx, u_ty, u_px, u_py;
    int c_tx, c_ty, c_px, c_py;
    latLonToTileXY(user_lat, user_lon, g_map_zoom,
                    &u_tx, &u_ty, &u_px, &u_py);
    latLonToTileXY(g_map_center_lat, g_map_center_lon, g_map_zoom,
                    &c_tx, &c_ty, &c_px, &c_py);

    double user_world_x   = (double)u_tx * MECK_MAP_TILE_PX + (double)u_px;
    double user_world_y   = (double)u_ty * MECK_MAP_TILE_PX + (double)u_py;
    double center_world_x = (double)c_tx * MECK_MAP_TILE_PX + (double)c_px;
    double center_world_y = (double)c_ty * MECK_MAP_TILE_PX + (double)c_py;

    int container_w = lv_obj_get_width(obj_map_tiles_container);
    int container_h = lv_obj_get_height(obj_map_tiles_container);
    int dot_size = 42;

    int dot_x = container_w / 2 + (int)(user_world_x - center_world_x) - dot_size / 2;
    int dot_y = container_h / 2 + (int)(user_world_y - center_world_y) - dot_size / 2;

    // Hide if entirely outside the container (saves a draw pass and
    // avoids confusion when the dot would otherwise pin to an edge).
    if (dot_x + dot_size < 0 || dot_x > container_w ||
        dot_y + dot_size < 0 || dot_y > container_h) {
        lv_obj_add_flag(g_gps_dot, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_set_pos(g_gps_dot, dot_x, dot_y);
    lv_obj_clear_flag(g_gps_dot, LV_OBJ_FLAG_HIDDEN);
}

static void gps_timer_cb(lv_timer_t* t) {
    (void)t;
    update_gps_dot();
}

// Read the persisted filter mask. The "set" sentinel (bit 7) lets us
// distinguish "never written, fall back to default" from "user has
// explicitly cleared all filter bits to hide every marker". Returns the
// raw mask (still includes the SET bit — callers only look at the
// low-nibble type bits).
static uint8_t resolve_filter_mask(void) {
    Meck* mesh = meck_get_instance();
    if (!mesh) return MECK_MAP_FILTER_DEFAULT;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return MECK_MAP_FILTER_DEFAULT;
    if ((prefs->map_filter_mask & MECK_MAP_FILTER_SET) == 0) {
        return MECK_MAP_FILTER_DEFAULT;
    }
    return prefs->map_filter_mask;
}

// Marker fill colour by advert type. The GPS dot uses cyan-blue with a
// white border, so none of these collide visually.
static lv_color_t color_for_adv_type(uint8_t adv_type) {
    switch (adv_type) {
        case ADV_TYPE_CHAT:      return lv_color_make( 80, 220, 120); // companion: green
        case ADV_TYPE_REPEATER:  return lv_color_make( 90, 200, 200); // cyan-teal
        case ADV_TYPE_ROOM:      return lv_color_make(110,  60, 180); // dark purple
        case ADV_TYPE_SENSOR:    return lv_color_make(240, 220,  80); // yellow
        default:                 return lv_color_make(160, 160, 160); // grey fallback
    }
}

// Tear down all marker widgets created by the previous render.
static void clear_markers(void) {
    for (int i = 0; i < g_marker_count; i++) {
        if (g_marker_labels[i])  { lv_obj_delete(g_marker_labels[i]);  g_marker_labels[i]  = NULL; }
        if (g_marker_circles[i]) { lv_obj_delete(g_marker_circles[i]); g_marker_circles[i] = NULL; }
    }
    g_marker_count = 0;
}

// Render all contact markers that pass the filter and fall inside the
// current viewport. Called from the tail of render_tiles so markers
// always reflect the current center and zoom. Labels appear at
// zoom >= 14 (any closer in and they crowd; any further out and the
// font can't render at a useful size).
static void render_markers(void) {
    clear_markers();
    if (!obj_map_tiles_container) return;

    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    int n = mesh->getNumContacts();
    if (n <= 0) return;

    uint8_t mask = resolve_filter_mask();

    // Map ADV_TYPE_* (1..4) to filter bits (0..3).
    // ADV_TYPE_CHAT=1 -> bit 0, REPEATER=2 -> bit 1, ROOM=3 -> bit 2, SENSOR=4 -> bit 3.

    // Current center as world-pixel coords, computed once for the loop.
    int c_tx, c_ty, c_px, c_py;
    latLonToTileXY(g_map_center_lat, g_map_center_lon, g_map_zoom,
                    &c_tx, &c_ty, &c_px, &c_py);
    double center_world_x = (double)c_tx * MECK_MAP_TILE_PX + (double)c_px;
    double center_world_y = (double)c_ty * MECK_MAP_TILE_PX + (double)c_py;

    int container_w = lv_obj_get_width(obj_map_tiles_container);
    int container_h = lv_obj_get_height(obj_map_tiles_container);
    int half_w = container_w / 2;
    int half_h = container_h / 2;

    const int circle_size  = 16;
    const int label_offset = circle_size + 2;   // label sits just below the circle
    const bool show_labels = (g_map_zoom >= 14);

    int skipped_no_gps   = 0;
    int skipped_filter   = 0;
    int skipped_viewport = 0;
    int skipped_cap      = 0;
    int created          = 0;

    for (int i = 0; i < n; i++) {
        ContactInfo ci;
        if (!mesh->getContactByIdx((uint32_t)i, ci)) continue;

        // Type filter. Map ADV_TYPE_* (1..4) to bit position (0..3).
        if (ci.type < 1 || ci.type > 4) { skipped_filter++; continue; }
        uint8_t type_bit = (uint8_t)(1 << (ci.type - 1));
        if ((mask & type_bit) == 0) { skipped_filter++; continue; }

        if (ci.gps_lat == 0 && ci.gps_lon == 0) { skipped_no_gps++; continue; }

        // ContactInfo stores lat/lon as int32 * 1e6 (six decimal places).
        // GPS snapshot uses 1e7. Don't conflate.
        double c_lat = (double)ci.gps_lat / 1.0e6;
        double c_lon = (double)ci.gps_lon / 1.0e6;

        int m_tx, m_ty, m_px, m_py;
        latLonToTileXY(c_lat, c_lon, g_map_zoom, &m_tx, &m_ty, &m_px, &m_py);
        double marker_world_x = (double)m_tx * MECK_MAP_TILE_PX + (double)m_px;
        double marker_world_y = (double)m_ty * MECK_MAP_TILE_PX + (double)m_py;

        int circle_x = half_w + (int)(marker_world_x - center_world_x) - circle_size / 2;
        int circle_y = half_h + (int)(marker_world_y - center_world_y) - circle_size / 2;

        if (circle_x + circle_size < 0 || circle_x > container_w ||
            circle_y + circle_size < 0 || circle_y > container_h) {
            skipped_viewport++;
            continue;
        }

        if (g_marker_count >= MECK_MAP_MAX_MARKERS) {
            skipped_cap++;
            continue;
        }

        // ---- Circle ----
        lv_obj_t* circle = lv_obj_create(obj_map_tiles_container);
        lv_obj_set_size(circle, circle_size, circle_size);
        lv_obj_set_pos(circle, circle_x, circle_y);
        lv_obj_set_style_radius(circle, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(circle, color_for_adv_type(ci.type), 0);
        lv_obj_set_style_bg_opa(circle, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(circle, lv_color_make(20, 20, 20), 0);
        lv_obj_set_style_border_width(circle, 1, 0);
        lv_obj_set_style_pad_all(circle, 0, 0);
        lv_obj_clear_flag(circle, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(circle, LV_OBJ_FLAG_SCROLLABLE);
        g_marker_circles[g_marker_count] = circle;

        // ---- Label (zoom >= 14 only) ----
        lv_obj_t* label = NULL;
        int label_x = 0, label_y = 0;
        if (show_labels) {
            // Strip emoji / higher-plane Unicode, then truncate to 20 chars.
            char clean[64];
            strip_unrenderable(ci.name, clean, sizeof(clean));
            if (strlen(clean) > 20) clean[20] = '\0';

            label = lv_label_create(obj_map_tiles_container);
            lv_label_set_text(label, clean);
            lv_obj_set_style_text_color(label, lv_color_white(), 0);
            lv_obj_set_style_bg_color(label, lv_color_make(0, 0, 0), 0);
            lv_obj_set_style_bg_opa(label, LV_OPA_60, 0);
            lv_obj_set_style_pad_hor(label, 3, 0);
            lv_obj_set_style_pad_ver(label, 1, 0);
            lv_obj_set_style_radius(label, 3, 0);
            lv_obj_clear_flag(label, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_clear_flag(label, LV_OBJ_FLAG_SCROLLABLE);

            // Position below the circle, horizontally centred. The label
            // hasn't been laid out yet so width queries return 0; use a
            // small left offset and let LVGL size it on first draw.
            label_x = circle_x + circle_size / 2 - 30;  // approximate centring
            label_y = circle_y + label_offset;
            lv_obj_set_pos(label, label_x, label_y);
        }
        g_marker_labels[g_marker_count] = label;

        g_marker_count++;
        created++;
    }

    printf("MeckMapScreen: render_markers z=%d mask=0x%02X -> %d markers"
           " (skipped: %d no-gps, %d filter, %d off-screen%s)\n",
           g_map_zoom, mask, created,
           skipped_no_gps, skipped_filter, skipped_viewport,
           skipped_cap > 0 ? ", CAP HIT" : "");
}

// ============================================================================
// Filter modal
// ============================================================================
//
// Built once in create_map_screen (after the toolbar so it lands on top
// in z-order), kept hidden via LV_OBJ_FLAG_HIDDEN. Filter tap calls
// show_filter_modal() which re-syncs the switches from prefs before
// revealing the modal. Apply writes the toggle state back to prefs +
// re-renders markers. Close (or backdrop tap) dismisses without saving.
// Clear resets toggles to the default (Repeater only) without applying
// or closing — user still has to tap Apply to commit it.

static void create_filter_modal(void) {
    if (g_filter_backdrop) return;
    if (!scr_map) return;

    int sw = SCREEN_WIDTH;
    int sh = SCREEN_HEIGHT;

    // Backdrop: full-screen, semi-transparent, swallows taps that miss
    // the card so the map underneath doesn't react.
    g_filter_backdrop = lv_obj_create(scr_map);
    lv_obj_set_size(g_filter_backdrop, sw, sh);
    lv_obj_set_pos(g_filter_backdrop, 0, 0);
    lv_obj_set_style_bg_color(g_filter_backdrop, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(g_filter_backdrop, LV_OPA_60, 0);
    lv_obj_set_style_border_width(g_filter_backdrop, 0, 0);
    lv_obj_set_style_radius(g_filter_backdrop, 0, 0);
    lv_obj_set_style_pad_all(g_filter_backdrop, 0, 0);
    lv_obj_add_flag(g_filter_backdrop, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(g_filter_backdrop, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(g_filter_backdrop, on_filter_backdrop_click,
                        LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(g_filter_backdrop, LV_OBJ_FLAG_HIDDEN);

    // Card: centered, holds the title, four toggle rows, and the
    // button strip. Sized to fit comfortably in the 540 x 1168 viewport
    // with generous touch targets.
    int card_w = 460;
    int card_h = 640;
    g_filter_card = lv_obj_create(g_filter_backdrop);
    lv_obj_set_size(g_filter_card, card_w, card_h);
    lv_obj_align(g_filter_card, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(g_filter_card, lv_color_make(35, 35, 40), 0);
    lv_obj_set_style_bg_opa(g_filter_card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_filter_card, lv_color_make(80, 80, 90), 0);
    lv_obj_set_style_border_width(g_filter_card, 1, 0);
    lv_obj_set_style_radius(g_filter_card, 12, 0);
    lv_obj_set_style_pad_all(g_filter_card, 20, 0);
    lv_obj_clear_flag(g_filter_card, LV_OBJ_FLAG_SCROLLABLE);
    // Stop card clicks from bubbling to the backdrop's CLICKED handler.
    lv_obj_add_flag(g_filter_card, LV_OBJ_FLAG_CLICKABLE);

    // Title.
    lv_obj_t* title = lv_label_create(g_filter_card);
    lv_label_set_text(title, "Filter markers");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &meck_montserrat_22, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

    // Four toggle rows. Each row: coloured dot at left, label, switch at
    // right. Iteration matches ADV_TYPE order: CHAT/Companion(1),
    // REPEATER(2), ROOM(3), SENSOR(4).
    const char* row_labels[4] = {
        "Companion", "Repeater", "Room", "Sensor"
    };
    const uint8_t row_adv_types[4] = {
        ADV_TYPE_CHAT, ADV_TYPE_REPEATER, ADV_TYPE_ROOM, ADV_TYPE_SENSOR
    };
    int row_y_start = 60;
    int row_h       = 56;
    int row_gap     = 8;

    for (int i = 0; i < 4; i++) {
        int y = row_y_start + i * (row_h + row_gap);

        // Coloured dot — same colour the marker uses on the map.
        lv_obj_t* dot = lv_obj_create(g_filter_card);
        lv_obj_set_size(dot, 24, 24);
        lv_obj_set_pos(dot, 0, y + (row_h - 24) / 2);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(dot, color_for_adv_type(row_adv_types[i]), 0);
        lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(dot, lv_color_make(20, 20, 20), 0);
        lv_obj_set_style_border_width(dot, 1, 0);
        lv_obj_set_style_pad_all(dot, 0, 0);
        lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);

        // Label.
        lv_obj_t* row_lbl = lv_label_create(g_filter_card);
        lv_label_set_text(row_lbl, row_labels[i]);
        lv_obj_set_style_text_color(row_lbl, lv_color_white(), 0);
        lv_obj_set_style_text_font(row_lbl, &meck_montserrat_18, 0);
        lv_obj_set_pos(row_lbl, 40, y + (row_h - 22) / 2);

        // Switch.
        lv_obj_t* sw_obj = lv_switch_create(g_filter_card);
        lv_obj_set_size(sw_obj, 70, 38);
        // Right-align inside the card content area (card has 20 px padding,
        // content area width = card_w - 40, switch width 70).
        lv_obj_set_pos(sw_obj, card_w - 40 - 70, y + (row_h - 38) / 2);
        g_filter_switches[i] = sw_obj;
    }

    // Button strip at the bottom of the card. Three buttons: Clear (left,
    // smaller / less prominent), then Apply and Close on the right.
    int btn_y = card_h - 40 - 60;
    int btn_h = 60;

    // Clear (left): resets toggle states only — user still taps Apply to
    // commit. Styled less prominently than Apply.
    lv_obj_t* btn_clear = lv_button_create(g_filter_card);
    lv_obj_set_size(btn_clear, 120, btn_h);
    lv_obj_set_pos(btn_clear, 0, btn_y);
    lv_obj_set_style_bg_color(btn_clear, lv_color_make(60, 60, 70), 0);
    lv_obj_set_style_radius(btn_clear, 8, 0);
    lv_obj_add_event_cb(btn_clear, on_filter_clear, LV_EVENT_CLICKED, NULL);
    lv_obj_t* lbl_clear = lv_label_create(btn_clear);
    lv_label_set_text(lbl_clear, "Clear");
    lv_obj_set_style_text_color(lbl_clear, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_clear, &meck_montserrat_18, 0);
    lv_obj_center(lbl_clear);

    // Apply (right of center). Green-ish to read as the primary action.
    lv_obj_t* btn_apply = lv_button_create(g_filter_card);
    lv_obj_set_size(btn_apply, 120, btn_h);
    lv_obj_set_pos(btn_apply, card_w - 40 - 120 - 130, btn_y);
    lv_obj_set_style_bg_color(btn_apply, lv_color_make(60, 130, 80), 0);
    lv_obj_set_style_radius(btn_apply, 8, 0);
    lv_obj_add_event_cb(btn_apply, on_filter_apply, LV_EVENT_CLICKED, NULL);
    lv_obj_t* lbl_apply = lv_label_create(btn_apply);
    lv_label_set_text(lbl_apply, "Apply");
    lv_obj_set_style_text_color(lbl_apply, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_apply, &meck_montserrat_18, 0);
    lv_obj_center(lbl_apply);

    // Close (rightmost). Neutral dark.
    lv_obj_t* btn_close = lv_button_create(g_filter_card);
    lv_obj_set_size(btn_close, 120, btn_h);
    lv_obj_set_pos(btn_close, card_w - 40 - 120, btn_y);
    lv_obj_set_style_bg_color(btn_close, lv_color_make(50, 50, 55), 0);
    lv_obj_set_style_radius(btn_close, 8, 0);
    lv_obj_add_event_cb(btn_close, on_filter_close, LV_EVENT_CLICKED, NULL);
    lv_obj_t* lbl_close = lv_label_create(btn_close);
    lv_label_set_text(lbl_close, "Close");
    lv_obj_set_style_text_color(lbl_close, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl_close, &meck_montserrat_18, 0);
    lv_obj_center(lbl_close);
}

// Sync each switch from the current prefs mask, then reveal the modal.
static void show_filter_modal(void) {
    if (!g_filter_backdrop) return;
    uint8_t mask = resolve_filter_mask();
    const uint8_t bit_for_row[4] = {
        MECK_MAP_FILTER_COMPANION,
        MECK_MAP_FILTER_REPEATER,
        MECK_MAP_FILTER_ROOM,
        MECK_MAP_FILTER_SENSOR,
    };
    for (int i = 0; i < 4; i++) {
        if (!g_filter_switches[i]) continue;
        if (mask & bit_for_row[i]) {
            lv_obj_add_state(g_filter_switches[i], LV_STATE_CHECKED);
        } else {
            lv_obj_remove_state(g_filter_switches[i], LV_STATE_CHECKED);
        }
    }
    lv_obj_move_foreground(g_filter_backdrop);
    lv_obj_clear_flag(g_filter_backdrop, LV_OBJ_FLAG_HIDDEN);
}

static void hide_filter_modal(void) {
    if (!g_filter_backdrop) return;
    lv_obj_add_flag(g_filter_backdrop, LV_OBJ_FLAG_HIDDEN);
}

// Apply: read switch states, build mask (with SET bit so we don't
// fall back to default on next read), assign to prefs, mark dirty,
// re-render markers. Modal stays open so user can keep tweaking.
static void on_filter_apply(lv_event_t* e) {
    (void)e;
    Meck* mesh = meck_get_instance();
    if (!mesh) return;
    P4NodePrefs* prefs = mesh->getNodePrefs();
    if (!prefs) return;

    const uint8_t bit_for_row[4] = {
        MECK_MAP_FILTER_COMPANION,
        MECK_MAP_FILTER_REPEATER,
        MECK_MAP_FILTER_ROOM,
        MECK_MAP_FILTER_SENSOR,
    };
    uint8_t new_mask = MECK_MAP_FILTER_SET;
    for (int i = 0; i < 4; i++) {
        if (g_filter_switches[i] &&
            lv_obj_has_state(g_filter_switches[i], LV_STATE_CHECKED)) {
            new_mask |= bit_for_row[i];
        }
    }
    if (prefs->map_filter_mask != new_mask) {
        prefs->map_filter_mask = new_mask;
        g_map_dirty = true;
    }
    printf("MeckMapScreen: filter applied mask=0x%02X\n", new_mask);
    render_markers();
}

// Clear: reset switch UI state to default (Repeater only). Does NOT
// apply or close — user still taps Apply to commit, or Close to
// discard.
static void on_filter_clear(lv_event_t* e) {
    (void)e;
    for (int i = 0; i < 4; i++) {
        if (!g_filter_switches[i]) continue;
        if (i == 1) {  // Repeater row
            lv_obj_add_state(g_filter_switches[i], LV_STATE_CHECKED);
        } else {
            lv_obj_remove_state(g_filter_switches[i], LV_STATE_CHECKED);
        }
    }
}

static void on_filter_close(lv_event_t* e) {
    (void)e;
    hide_filter_modal();
}

static void on_filter_backdrop_click(lv_event_t* e) {
    // Tap-outside-card dismiss. Only fires if the tap target was the
    // backdrop itself, not a click bubbling up from a child (the card
    // also has LV_OBJ_FLAG_CLICKABLE, which absorbs taps on its area).
    if (lv_event_get_target(e) != g_filter_backdrop) return;
    hide_filter_modal();
}

// ============================================================================
// Screen construction
// ============================================================================

// Helper to create a toolbar button with a centred text label. Used
// for all 4 toolbar buttons so spacing and styling stay consistent.
static lv_obj_t* make_toolbar_button(lv_obj_t* parent, int x, int y,
                                      int w, int h,
                                      const char* text,
                                      lv_event_cb_t cb) {
    lv_obj_t* btn = lv_button_create(parent);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_style_bg_color(btn, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn, 8, 0);

    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(lbl, &meck_montserrat_18, 0);
    lv_obj_center(lbl);

    if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    return btn;
}

static void create_map_screen(void) {
    if (scr_map) return;  // idempotent

    scr_map = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_map, lv_color_black(), 0);
    lv_obj_clear_flag(scr_map, LV_OBJ_FLAG_SCROLLABLE);

    // Use the compile-time panel dimensions (from the LilyGo driver
    // headers, brought in transitively). lv_obj_get_width/height on
    // scr_map returns 0 here because the layout pass hasn't run yet,
    // and that was making all the widgets pile up at (10,10).
    int sw = SCREEN_WIDTH;
    int sh = SCREEN_HEIGHT;

    // ---- Header: back button (left) and title (centered-ish) ----
    btn_map_back = lv_button_create(scr_map);
    lv_obj_set_size(btn_map_back, 100, 70);
    lv_obj_align(btn_map_back, LV_ALIGN_TOP_LEFT, NOTCH_SAFE_X, NOTCH_SAFE_Y);
    lv_obj_set_style_bg_color(btn_map_back, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_radius(btn_map_back, 8, 0);
    lv_obj_t* bl = lv_label_create(btn_map_back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_set_style_text_color(bl, lv_color_white(), 0);
    lv_obj_set_style_text_font(bl, &meck_montserrat_18, 0);
    lv_obj_center(bl);
    lv_obj_add_event_cb(btn_map_back, on_map_back, LV_EVENT_CLICKED, NULL);

    lv_obj_t* title = lv_label_create(scr_map);
    lv_label_set_text(title, "Maps");
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(title, &meck_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 130, 20);

    // Debug status label (zoom + lat/lon).
    lbl_map_status = lv_label_create(scr_map);
    lv_label_set_text(lbl_map_status, "");
    lv_obj_set_style_text_color(lbl_map_status, lv_color_make(180, 180, 180), 0);
    lv_obj_set_style_text_font(lbl_map_status, &meck_montserrat_16, 0);
    lv_obj_align(lbl_map_status, LV_ALIGN_TOP_RIGHT, -20, 30);

    // ---- Tile viewport container ----
    // Fills the space between header and toolbar. Clips overflow so
    // pan-by-translation in session 2 won't spill onto the toolbars.
    obj_map_tiles_container = lv_obj_create(scr_map);
    lv_obj_set_size(obj_map_tiles_container,
                    sw, sh - MECK_MAP_HEADER_H - MECK_MAP_TOOLBAR_H);
    lv_obj_set_pos(obj_map_tiles_container, 0, MECK_MAP_HEADER_H);
    lv_obj_set_style_bg_color(obj_map_tiles_container,
                              lv_color_make(20, 20, 25), 0);
    lv_obj_set_style_border_width(obj_map_tiles_container, 0, 0);
    lv_obj_set_style_radius(obj_map_tiles_container, 0, 0);
    lv_obj_set_style_pad_all(obj_map_tiles_container, 0, 0);
    lv_obj_clear_flag(obj_map_tiles_container, LV_OBJ_FLAG_SCROLLABLE);
    // (LVGL 9 clips children to parent by default for our use case;
    // explicit overflow handling will revisit in session 2 if pan
    // shows tiles spilling outside the container.)

    // Enable input events on the tile container so the pan-by-drag
    // handler receives PRESSED / PRESSING / RELEASED. Tile widgets
    // themselves are non-clickable (set in render_tiles), so taps
    // and drags fall through to the container.
    lv_obj_add_flag(obj_map_tiles_container, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(obj_map_tiles_container, on_tile_container_event,
                        LV_EVENT_ALL, NULL);

    // ---- GPS "you are here" dot ----
    // Lives inside the tile container so it clips to the viewport. Sits
    // above tile widgets in z-order because it's created after them
    // logically (render_tiles re-creates tiles, but it parents them to
    // the same container with default z-order — the dot, created here
    // at screen-build time, ends up below). Z-order is fixed below
    // after the first render with lv_obj_move_foreground.
    g_gps_dot = lv_obj_create(obj_map_tiles_container);
    lv_obj_set_size(g_gps_dot, 42, 42);
    lv_obj_set_style_radius(g_gps_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(g_gps_dot, lv_color_make(80, 180, 255), 0);
    lv_obj_set_style_bg_opa(g_gps_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(g_gps_dot, lv_color_white(), 0);
    lv_obj_set_style_border_width(g_gps_dot, 2, 0);
    lv_obj_set_style_pad_all(g_gps_dot, 0, 0);
    lv_obj_clear_flag(g_gps_dot, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(g_gps_dot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_gps_dot, LV_OBJ_FLAG_HIDDEN);  // shown when fix arrives

    // ---- Toolbar (bottom) ----
    // Layout: Recenter / Zoom- / Zoom+ / Filter, evenly spaced.
    // Inner padding is 5 px top + 5 px bottom so button height fills
    // most of the 90 px strip — gives a noticeably larger touch target
    // than the original 60 px.
    int tb_y = sh - MECK_MAP_TOOLBAR_H + 5;
    int tb_h = MECK_MAP_TOOLBAR_H - 10;
    int tb_w = (sw - 50) / 4;   // 4 buttons + 5 gaps of ~10px
    int tb_gap = 10;
    int tb_x = tb_gap;

    btn_map_recenter = make_toolbar_button(scr_map, tb_x, tb_y, tb_w, tb_h,
                                            "Recenter", on_map_recenter);
    tb_x += tb_w + tb_gap;
    btn_map_zoom_out = make_toolbar_button(scr_map, tb_x, tb_y, tb_w, tb_h,
                                            "Zoom -", on_map_zoom_out);
    tb_x += tb_w + tb_gap;
    btn_map_zoom_in = make_toolbar_button(scr_map, tb_x, tb_y, tb_w, tb_h,
                                           "Zoom +", on_map_zoom_in);
    tb_x += tb_w + tb_gap;
    btn_map_filter = make_toolbar_button(scr_map, tb_x, tb_y, tb_w, tb_h,
                                          "Filter", on_map_filter);

    // Filter modal: built last so it sits on top in z-order. Hidden
    // until the Filter button is tapped.
    create_filter_modal();
}

// ============================================================================
// Public API
// ============================================================================

void meck_map_ui_init(void) {
    if (g_map_inited) return;
    create_map_screen();
    g_map_inited = true;
    printf("MeckMapScreen: init complete\n");
}

void meck_map_ui_show(void) {
    if (!g_map_inited || !scr_map) {
        printf("MeckMapScreen: show called before init\n");
        return;
    }

    // Lazy first-time work: detect zoom range and load persisted
    // state. Cheap to do on every show but the scan is the more
    // expensive of the two so we gate it.
    if (g_map_first_show) {
        detect_zoom_range();
        load_persisted_state();
        g_map_first_show = false;
    }

    lv_screen_load(scr_map);

    // Render after the screen is loaded so the container has its
    // actual laid-out width and height by the time render_tiles
    // queries them. Without this, the first render can compute
    // viewport_w=0 (container not yet positioned) and bail.
    update_status_label();
    render_tiles();

    // GPS dot: keep it above the tiles in z-order (render_tiles
    // recreates tile widgets after the dot was created), then start a
    // 2 s refresh timer. Immediate update so the dot doesn't wait up
    // to 2 s after first show to appear.
    if (g_gps_dot) lv_obj_move_foreground(g_gps_dot);
    update_gps_dot();
    if (!g_gps_timer) {
        g_gps_timer = lv_timer_create(gps_timer_cb, 2000, NULL);
    }
}
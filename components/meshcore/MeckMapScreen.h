/*
 * MeckMapScreen.h — LVGL map screen for Meck on T-Display P4
 *
 * Renders OSM "slippy map" PNG tiles from /sdcard/tiles/{z}/{x}/{y}.png
 * via LVGL's image widget + LV_USE_LODEPNG. Markers overlay user GPS
 * position (when GPS is enabled) and contact positions filtered by
 * type (repeaters by default).
 *
 * Tile path on SD: /sdcard/tiles/{z}/{x}/{y}.png
 * Tile path as seen by LVGL: A:/sdcard/tiles/{z}/{x}/{y}.png
 *   (the "A:" prefix selects the POSIX FS driver — same letter the
 *   audio UI uses for cover art at A:/sdcard/audio/...)
 *
 * Session 1 deliverable: tapping the Maps home tile opens the screen
 * and shows static tiles around either the last-viewed center or the
 * Sydney CBD fallback. Pan, zoom, GPS dot, and markers are session 2
 * and 3.
 *
 * Back button returns to the home screen.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Build the map screen. Called once from meck_ui_init() AFTER
 * lvgl_api_lock is acquired. Idempotent; subsequent calls no-op.
 *
 * Does NOT load tiles or detect zoom range — both happen lazily on
 * the first call to meck_map_ui_show() so the cost is only paid when
 * the user actually opens the screen.
 */
void meck_map_ui_init(void);

/*
 * Tear down the map screen and reset its state (pools, lazy flags, GPS
 * timer) so a subsequent meck_map_ui_init() rebuilds it. Used by the live
 * screen-orientation rebuild.
 */
void meck_map_ui_teardown(void);

/*
 * Load the map screen. Wired to the home grid's Maps tile.
 * On first call: detects available zoom range from SD, loads
 * persisted center from prefs (or Sydney CBD fallback), renders
 * tiles. Subsequent calls just re-show the screen with current state.
 */
void meck_map_ui_show(void);

#ifdef __cplusplus
}
#endif
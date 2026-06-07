/*
 * MeckReaderUI.h — LVGL screens for the Meck text reader
 *
 * Two screens, mirroring the audio module's split:
 *   scr_reader_browser — folder navigator rooted at /sdcard/books
 *   scr_reader_view    — reading view (one page at a time, progress, taps)
 *
 * The browser handles:
 *   - directory listing with subfolder navigation
 *   - breadcrumb path display
 *   - .txt filtering
 *   - a resume indicator per file (a saved position exists)
 *
 * The reading view handles:
 *   - windowed page-at-a-time rendering
 *   - left third of the text taps back a page, right two thirds forward
 *   - percentage progress
 *   - Back returns to the browser; the browser's Back goes up a level,
 *     or home at the /sdcard/books root
 *
 * Both screens use MeckReader (declared in MeckReader.h) as the only paging
 * and resume backend, the same way the audio screens use MeckAudio.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Build the browser and reading-view screens. Called once from
 * meck_ui_init(). Idempotent; subsequent calls no-op.
 */
void meck_reader_ui_init(void);

/* Tear down the reader screens (frees child widgets) and reset the init
 * guard so a subsequent meck_reader_ui_init() rebuilds them. Used by the
 * live screen-orientation rebuild. */
void meck_reader_ui_teardown(void);

/* Load the browser screen. Wired to the home grid's Reader tile. */
void meck_reader_ui_show_browser(void);

/* Load the reading view. Called internally when a file is opened. */
void meck_reader_ui_show_reader(void);

#ifdef __cplusplus
}
#endif
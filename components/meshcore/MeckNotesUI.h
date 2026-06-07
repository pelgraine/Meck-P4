/*
 * MeckNotesUI.h — LVGL screens for Meck notes
 *
 * Mirrors the reader module's split:
 *   scr_notes_browser — file navigator rooted at /sdcard/notes
 *   scr_notes_view    — read-only view (one page at a time, progress, taps)
 *
 * Stage 1 scope: browse and read existing .txt notes. Reading reuses
 * MeckReader (the paging/resume core), the same way the reader screens do.
 * The in-app editor, new-note, rename and delete arrive in later stages.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Build the notes browser and read-view screens, and ensure the notes folder
 * exists on the SD card. Called once from meck_ui_init(). Idempotent;
 * subsequent calls no-op.
 */
void meck_notes_ui_init(void);

/* Tear down the notes screens (frees child widgets) and reset the init guard
 * so a subsequent meck_notes_ui_init() rebuilds them. Used by the live
 * screen-orientation rebuild. */
void meck_notes_ui_teardown(void);

/* Load the browser screen. Wired to the home grid's Notes tile. */
void meck_notes_ui_show_browser(void);

/* Load the read view. Called internally when a note is opened. */
void meck_notes_ui_show_reader(void);

/* Load the editor screen. Called internally for "New Note" and for the read
 * view's Edit button. */
void meck_notes_ui_show_editor(void);

#ifdef __cplusplus
}
#endif
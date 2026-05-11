/*
 * MeckAudioUI.h — LVGL screens for the Meck audio player
 *
 * Two screens:
 *   scr_audio_browser — file/folder navigator rooted at /sdcard/audio
 *   scr_audio_player  — Now Playing view (cover, transport, progress)
 *
 * The browser handles:
 *   - directory listing with subfolder navigation
 *   - breadcrumb path display
 *   - file-type filtering (.wav .mp3 .m4a .m4b .flac .aac .ogg .opus)
 *   - bookmark indicator per file
 *   - "library mode" inference based on top-level path
 *     (audio/music vs audio/audiobooks subtrees)
 *
 * The player handles:
 *   - transport row (-30s, play/pause, +30s)
 *   - progress bar + time labels
 *   - volume control
 *   - audiobook-only widgets (chapter chips, sleep timer) gated on
 *     library mode + presence of chapter metadata
 *
 * Both screens use MeckAudio (declared in MeckAudio.h) as the only
 * backend interface — they have no direct dependency on the Arduino
 * audio library.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Build the audio browser and player LVGL screens. Called once from
 * meck_ui_init() AFTER lvgl_api_lock is acquired. Idempotent; subsequent
 * calls no-op.
 *
 * Does NOT call meck_audio_init() — that happens lazily on the first
 * file open so the codec stays cold until needed.
 */
void meck_audio_ui_init(void);

/* Load the browser screen. Wired to the home grid's Audio tile. */
void meck_audio_ui_show_browser(void);

/* Load the player screen. Called internally when a file is opened. */
void meck_audio_ui_show_player(void);

#ifdef __cplusplus
}
#endif
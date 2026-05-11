/*
 * MeckAudio.h — Audio backend public C API for Meck on T-Display P4
 *
 * Implementation lives in MeckAudio.cpp, built on:
 *   - chmorgan/esp-audio-player  (WAV + MP3 decode, state machine)
 *   - espressif/es8311           (codec register driver)
 *   - ESP-IDF i2s_std            (I²S output channel)
 *
 * No Arduino dependencies anywhere. SD card paths are passed directly to
 * fopen() via the VFS — LilyGo's existing /sdcard mount works as-is.
 *
 * Threading: chmorgan's decode task runs internally. UI calls into this
 * module are non-blocking where possible (open/play queues a command;
 * pause/resume/volume are immediate). Getters take a brief mutex.
 *
 * Format support in v0.2:
 *   - WAV: sample-accurate seek (header parsing for byte offset)
 *   - MP3: approximate seek (CBR byte offset; VBR approximate)
 *   FLAC, M4A, OGG: not in v0.2. chmorgan only does WAV + MP3.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* State enum                                                          */
/* ------------------------------------------------------------------ */

typedef enum {
    MECK_AUDIO_STATE_IDLE = 0,      /* nothing loaded / nothing playing */
    MECK_AUDIO_STATE_LOADING,       /* file open in flight */
    MECK_AUDIO_STATE_PLAYING,
    MECK_AUDIO_STATE_PAUSED,
    MECK_AUDIO_STATE_EOF,            /* end of file reached this turn */
    MECK_AUDIO_STATE_ERROR,
} MeckAudioState;

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

/*
 * Lazy-initialise the audio backend. Sets up the I²S channel, configures
 * the ES8311 codec, and starts chmorgan's decode task. Idempotent — safe
 * to call from meck_audio_play_file().
 *
 * Returns true on success, false if any of the underlying inits failed.
 * Side effect on first call: ~250 ms of setup work; codec power-on may
 * produce a brief click (much less than LilyGo's boot-time init).
 */
bool meck_audio_init(void);

/* Whether init has completed successfully. */
bool meck_audio_ready(void);

/* ------------------------------------------------------------------ */
/* Playback control                                                    */
/* ------------------------------------------------------------------ */

/*
 * Open a file and start playing it. resume_sec > 0 will fseek the file
 * before playback starts (bookmark resume). Returns true if the play
 * request was queued; false if the path is empty or the backend can't
 * be initialised.
 *
 * If another file is already playing it's stopped and its bookmark is
 * saved before the new file opens.
 */
bool meck_audio_play_file(const char *path, uint32_t resume_sec);

void meck_audio_pause(void);
void meck_audio_resume(void);
void meck_audio_toggle_pause(void);
void meck_audio_stop(void);

/* Skip ±N seconds. Causes a brief audible blip (stop → reopen → play).
 * Sample-accurate for WAV, approximate for MP3. */
void meck_audio_seek_relative(int32_t seconds);

/* Jump to an absolute time. Same blip-on-seek characteristics. */
void meck_audio_seek_absolute(uint32_t position_sec);

/* ------------------------------------------------------------------ */
/* Volume                                                              */
/* ------------------------------------------------------------------ */

/* UI exposes 0..100%; this maps directly to ES8311 hardware volume. */
uint8_t meck_audio_get_volume_pct(void);
void    meck_audio_set_volume_pct(uint8_t pct);

/* ------------------------------------------------------------------ */
/* Status queries                                                      */
/* ------------------------------------------------------------------ */

MeckAudioState meck_audio_get_state(void);
uint32_t       meck_audio_get_position_sec(void);
uint32_t       meck_audio_get_duration_sec(void);
uint32_t       meck_audio_get_bitrate(void);
const char    *meck_audio_get_current_path(void);

/* ------------------------------------------------------------------ */
/* EOF flag (set when current track completes)                         */
/* ------------------------------------------------------------------ */

/* Returns true ONCE per EOF event — atomically reads and clears the
 * flag. UI polls this from the screen refresh timer; when it sees true
 * it advances to the next track in the playlist (or stops). */
bool meck_audio_consume_eof_flag(void);

/* ------------------------------------------------------------------ */
/* Bookmark store                                                      */
/* ------------------------------------------------------------------ */

/*
 * Look up the saved playback position (seconds) for the given file path,
 * or 0 if none exists. Caller uses this when opening a file via
 * meck_audio_play_file() to pass as resume_sec.
 *
 * Safe to call from the UI thread; reads a small file from SD.
 */
uint32_t meck_audio_bookmark_load_sec(const char *path);

/*
 * Forget the bookmark for a file (e.g. user explicitly restarts from 0).
 * No-op if no bookmark exists. Safe from UI thread.
 */
void meck_audio_bookmark_clear(const char *path);

/* ------------------------------------------------------------------ */
/* Codec passthrough (for screens that want direct DAC control)       */
/* ------------------------------------------------------------------ */

/* Soft mute the DAC immediately. Used by the screen-off timer so the
 * device doesn't continue playing into a pocket. Does NOT pause —
 * playback continues silently. Pair with set_dac_mute(false) to restore. */
void meck_audio_set_dac_mute(bool muted);

#ifdef __cplusplus
}
#endif
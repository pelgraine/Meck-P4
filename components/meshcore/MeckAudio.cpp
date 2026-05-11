/*
 * MeckAudio.cpp — Audio backend for Meck on T-Display P4 (pure ESP-IDF)
 *
 * Stack:
 *   chmorgan/esp-audio-player  — decode (WAV + MP3 via libhelix-mp3) + state
 *                                machine + decode task
 *   espressif/es8311           — codec register driver
 *   ESP-IDF i2s_std            — I²S output channel
 *
 * The decode task is owned by chmorgan; it calls our write_fn callback
 * (in meck_es8311.c) with PCM samples. We watch chmorgan's state-change
 * callback for EOF, error, etc.
 *
 * No Arduino dependencies. Replaces the previous Arduino-as-component
 * version that conflicted with LilyGo's pinned esp_hosted / esp_wifi_remote.
 *
 * Position tracking: chmorgan doesn't expose current position natively.
 * We accumulate bytes pushed through write_fn (counter lives in
 * meck_es8311.c) and divide by rate × channels × bytes_per_sample.
 *
 * Seek workflow: chmorgan has no native seek either. We:
 *   1. stop the player
 *   2. reopen the file
 *   3. fseek to the computed byte offset
 *   4. start playing again
 * Causes a brief audible blip but no other side effects. Sample-accurate
 * for WAV (header parsing); approximate for MP3 (byte ratio).
 */

#include "MeckAudio.h"

/* ---- ESP-IDF ---- */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

/* ---- chmorgan/esp-audio-player ---- */
#include "audio_player.h"

/* ---- POSIX ---- */
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>     /* for unlink() in meck_audio_bookmark_clear */
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "MeckAudio";

/* ---- Externs from meck_es8311.c ---- */
extern "C" {
    bool     meck_audio_es8311_setup(int sda, int scl, uint32_t rate);
    void     meck_audio_es8311_set_volume(uint8_t pct);
    uint8_t  meck_audio_es8311_get_volume(void);
    void     meck_audio_es8311_set_mute(bool muted);
    bool     meck_audio_es8311_is_ready(void);

    /* chmorgan/esp-audio-player mute callback. The library calls this
     * during audio_player_new() before the first decode and at every
     * play/stop boundary. mute_fn = nullptr in the config crashes
     * audio_player_new with PC=0; this stub returns ESP_OK and tracks
     * the requested state. Real codec mute is wired through
     * ES8311->set_dac_mute() in the cpp_bus_driver pass. */
    esp_err_t meck_audio_es8311_mute_cb(AUDIO_PLAYER_MUTE_SETTING setting);

    esp_err_t meck_audio_i2s_write(void *buf, size_t len,
                                   size_t *bytes_written, uint32_t timeout);
    esp_err_t meck_audio_i2s_reconfig(uint32_t rate, uint32_t bps,
                                      i2s_slot_mode_t channels);
    void     meck_audio_i2s_reset_counter(void);
    uint64_t meck_audio_i2s_bytes_written(void);
    uint32_t meck_audio_i2s_current_rate(void);
    uint8_t  meck_audio_i2s_current_chans(void);
    uint8_t  meck_audio_i2s_current_bps(void);
}

/* ============================================================================
 * State (file scope)
 * ==========================================================================*/

static SemaphoreHandle_t g_state_mu = nullptr;
static bool              g_inited   = false;

/* Public-API state (under g_state_mu). */
static MeckAudioState g_state           = MECK_AUDIO_STATE_IDLE;
static uint32_t       g_duration_sec    = 0;
static uint32_t       g_bitrate_bps     = 0;
static uint8_t        g_volume_pct      = 50;
static char           g_current_path[256] = "";

/* The "seek-start" offset: when we seek to T seconds, we restart playback
 * from byte offset N and tell ourselves "this is at T seconds". The
 * write_fn byte counter resets and counts from there. Position is
 * computed as g_seek_start_sec + (bytes_written / bytes_per_second). */
static uint32_t g_seek_start_sec = 0;

/* The currently-open FILE* — we own it across the player's lifetime so
 * we can reopen for seek. chmorgan reads from the position we leave it
 * at when audio_player_play() is called. */
static FILE *g_fp = nullptr;

/* EOF flag — set in callback, drained by UI. */
static volatile uint8_t g_eof_flag = 0;

/* ============================================================================
 * Bookmark store (FNV-1a hash of path → /sdcard/audio/.bookmarks/<hex>.bm)
 *
 * Identical to the previous Arduino version's scheme so existing
 * bookmarks survive the migration.
 * ==========================================================================*/

static const char *BOOKMARK_DIR = "/sdcard/audio/.bookmarks";

static uint32_t fnv1a_hash(const char *s)
{
    uint32_t h = 2166136261u;
    for (const unsigned char *p = (const unsigned char *)s; *p; ++p) {
        h ^= *p;
        h *= 16777619u;
    }
    return h;
}

static void bookmark_path_for(const char *full_path, char *out, size_t out_sz)
{
    snprintf(out, out_sz, "%s/%08x.bm", BOOKMARK_DIR, fnv1a_hash(full_path));
}

static bool ensure_bookmark_dir(void)
{
    struct stat st;
    if (stat(BOOKMARK_DIR, &st) == 0) return true;
    if (mkdir(BOOKMARK_DIR, 0755) == 0) return true;
    if (errno == EEXIST) return true;
    ESP_LOGE(TAG, "mkdir('%s') failed (errno=%d)", BOOKMARK_DIR, errno);
    return false;
}

extern "C" uint32_t meck_audio_bookmark_load_sec(const char *path)
{
    if (!path || !*path) return 0;
    char bm[300];
    bookmark_path_for(path, bm, sizeof(bm));
    FILE *f = fopen(bm, "rb");
    if (!f) return 0;
    uint32_t pos = 0;
    size_t got = fread(&pos, 1, sizeof(pos), f);
    fclose(f);
    return (got == sizeof(pos)) ? pos : 0;
}

static void bookmark_save_for(const char *path, uint32_t pos_sec)
{
    if (!path || !*path) return;
    if (!ensure_bookmark_dir()) return;
    char bm[300];
    bookmark_path_for(path, bm, sizeof(bm));
    FILE *f = fopen(bm, "wb");
    if (!f) {
        ESP_LOGE(TAG, "bookmark save fopen('%s') failed (errno=%d)",
                 bm, errno);
        return;
    }
    fwrite(&pos_sec, 1, sizeof(pos_sec), f);
    fclose(f);
}

extern "C" void meck_audio_bookmark_clear(const char *path)
{
    if (!path || !*path) return;
    char bm[300];
    bookmark_path_for(path, bm, sizeof(bm));
    if (unlink(bm) != 0 && errno != ENOENT) {
        ESP_LOGE(TAG, "bookmark clear unlink('%s') failed (errno=%d)",
                 bm, errno);
    }
}

/* ============================================================================
 * WAV header parsing (for sample-accurate seek)
 *
 * Reads the RIFF/WAVE header to extract sample_rate, channels, bits_per_sample,
 * data_offset, and data_size. Skips unknown chunks (e.g. LIST, fact) to find
 * the 'data' chunk reliably.
 * ==========================================================================*/

typedef struct {
    uint32_t sample_rate;
    uint16_t channels;
    uint16_t bits_per_sample;
    uint32_t data_offset;
    uint32_t data_size;
} WavInfo;

static bool wav_parse_header(FILE *f, WavInfo *out)
{
    uint8_t hdr[12];
    rewind(f);
    if (fread(hdr, 1, 12, f) != 12) return false;
    if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) {
        return false;
    }

    bool fmt_seen = false;
    while (true) {
        uint8_t chunk_hdr[8];
        if (fread(chunk_hdr, 1, 8, f) != 8) return false;
        uint32_t size = (uint32_t)chunk_hdr[4]
                      | ((uint32_t)chunk_hdr[5] << 8)
                      | ((uint32_t)chunk_hdr[6] << 16)
                      | ((uint32_t)chunk_hdr[7] << 24);

        if (memcmp(chunk_hdr, "fmt ", 4) == 0) {
            uint8_t fmt[16];
            if (size < 16) return false;
            if (fread(fmt, 1, 16, f) != 16) return false;
            out->channels        = (uint16_t)fmt[2] | ((uint16_t)fmt[3] << 8);
            out->sample_rate     = (uint32_t)fmt[4]
                                  | ((uint32_t)fmt[5] << 8)
                                  | ((uint32_t)fmt[6] << 16)
                                  | ((uint32_t)fmt[7] << 24);
            out->bits_per_sample = (uint16_t)fmt[14] | ((uint16_t)fmt[15] << 8);
            fmt_seen = true;
            /* Skip any extra fmt bytes. */
            if (size > 16) fseek(f, size - 16, SEEK_CUR);
        } else if (memcmp(chunk_hdr, "data", 4) == 0) {
            if (!fmt_seen) return false;
            out->data_size   = size;
            out->data_offset = (uint32_t)ftell(f);
            return true;
        } else {
            /* Unknown chunk — skip. */
            fseek(f, size, SEEK_CUR);
        }
    }
}

static uint32_t wav_byte_offset_for_sec(const WavInfo *w, uint32_t target_sec)
{
    uint32_t bps = w->bits_per_sample / 8;
    uint32_t bytes_per_sec = w->sample_rate * w->channels * bps;
    uint64_t off = (uint64_t)bytes_per_sec * target_sec;
    /* Align down to a frame boundary so we don't tear samples. */
    uint32_t frame = w->channels * bps;
    off -= (off % frame);
    if (off > w->data_size) off = w->data_size;
    return w->data_offset + (uint32_t)off;
}

static uint32_t wav_duration_sec(const WavInfo *w)
{
    uint32_t bps = w->bits_per_sample / 8;
    uint32_t bytes_per_sec = w->sample_rate * w->channels * bps;
    if (bytes_per_sec == 0) return 0;
    return w->data_size / bytes_per_sec;
}

/* ============================================================================
 * MP3 duration / seek (approximate)
 *
 * No frame-by-frame index — we estimate via file size and bitrate. CBR
 * gives near-perfect results; VBR seeking will drift but stays within a
 * few seconds. Good enough for ±30s skip and bookmark resume on
 * audiobook MP3s.
 * ==========================================================================*/

static bool mp3_estimate_duration(const char *path, uint32_t *out_dur_sec,
                                  uint32_t *out_bitrate)
{
    /* Cheap approach: open the file, scan first ~4KB for a sync frame,
     * extract bitrate from the header. Use file size + bitrate for
     * duration. Good enough; if it fails we fall back to 0/unknown. */
    FILE *f = fopen(path, "rb");
    if (!f) return false;

    uint8_t buf[4096];
    size_t got = fread(buf, 1, sizeof(buf), f);
    if (got < 4) { fclose(f); return false; }

    /* MPEG audio frame sync: bytes [FF Ex] or [FF Fx]. */
    static const uint16_t bitrate_table_v1l3[16] = {
        0,32,40,48,56,64,80,96,112,128,160,192,224,256,320,0
    };
    static const uint32_t sr_table_v1[4] = { 44100, 48000, 32000, 0 };

    uint32_t bitrate_kbps = 0;
    for (size_t i = 0; i + 3 < got; i++) {
        if (buf[i] != 0xFF) continue;
        if ((buf[i + 1] & 0xE0) != 0xE0) continue;
        uint8_t br_idx = (buf[i + 2] >> 4) & 0x0F;
        uint8_t sr_idx = (buf[i + 2] >> 2) & 0x03;
        if (br_idx == 0 || br_idx == 15) continue;
        bitrate_kbps = bitrate_table_v1l3[br_idx];
        (void)sr_table_v1[sr_idx];
        break;
    }
    if (bitrate_kbps == 0) { fclose(f); return false; }

    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fclose(f);
    if (file_size <= 0) return false;

    uint32_t bytes_per_sec = (bitrate_kbps * 1000u) / 8u;
    if (bytes_per_sec == 0) return false;

    if (out_dur_sec) *out_dur_sec = (uint32_t)(file_size / bytes_per_sec);
    if (out_bitrate) *out_bitrate = bitrate_kbps * 1000u;
    return true;
}

static uint32_t mp3_byte_offset_for_sec(const char *path, uint32_t target_sec,
                                        uint32_t duration_sec)
{
    if (duration_sec == 0) return 0;
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fclose(f);
    if (sz <= 0) return 0;
    /* Linear approximation. Skip any ID3v2 header by aligning to a
     * reasonable byte; the chmorgan decoder resyncs on first valid frame
     * after our fseek. */
    uint64_t off = (uint64_t)sz * target_sec / duration_sec;
    return (uint32_t)off;
}

/* ============================================================================
 * File type helpers
 * ==========================================================================*/

typedef enum { FT_UNKNOWN, FT_WAV, FT_MP3 } FileType;

static FileType detect_type(const char *path)
{
    const char *dot = strrchr(path, '.');
    if (!dot) return FT_UNKNOWN;
    if (strcasecmp(dot, ".wav") == 0) return FT_WAV;
    if (strcasecmp(dot, ".mp3") == 0) return FT_MP3;
    return FT_UNKNOWN;
}

/* ============================================================================
 * chmorgan event callback
 *
 * Fires when the player transitions states. We watch for IDLE-after-playing
 * (= track completed) to set the EOF flag and let the UI auto-advance.
 * ==========================================================================*/

static void player_event_cb(audio_player_cb_ctx_t *ctx)
{
    /* The cb_ctx struct exposes the new state in ctx->audio_event.
     * If chmorgan's signature differs in your installed version, this
     * is the one spot to tweak — see audio_player.h. */
    audio_player_callback_event_t evt = ctx->audio_event;

    switch (evt) {
        case AUDIO_PLAYER_CALLBACK_EVENT_IDLE:
            ESP_LOGI(TAG, "callback: IDLE (track ended or stopped)");
            g_eof_flag = 1;
            if (g_state_mu && xSemaphoreTake(g_state_mu, pdMS_TO_TICKS(10))) {
                /* Only flip to EOF if we were playing (not from an
                 * explicit stop, which sets state directly). */
                if (g_state == MECK_AUDIO_STATE_PLAYING) {
                    g_state = MECK_AUDIO_STATE_EOF;
                }
                xSemaphoreGive(g_state_mu);
            }
            break;
        case AUDIO_PLAYER_CALLBACK_EVENT_PLAYING:
            if (g_state_mu && xSemaphoreTake(g_state_mu, pdMS_TO_TICKS(10))) {
                g_state = MECK_AUDIO_STATE_PLAYING;
                xSemaphoreGive(g_state_mu);
            }
            break;
        case AUDIO_PLAYER_CALLBACK_EVENT_PAUSE:
            if (g_state_mu && xSemaphoreTake(g_state_mu, pdMS_TO_TICKS(10))) {
                g_state = MECK_AUDIO_STATE_PAUSED;
                xSemaphoreGive(g_state_mu);
            }
            break;
        case AUDIO_PLAYER_CALLBACK_EVENT_COMPLETED_PLAYING_NEXT:
            /* Track-to-track transition without an intervening idle. */
            break;
        case AUDIO_PLAYER_CALLBACK_EVENT_SHUTDOWN:
        case AUDIO_PLAYER_CALLBACK_EVENT_UNKNOWN_FILE_TYPE:
        case AUDIO_PLAYER_CALLBACK_EVENT_UNKNOWN:
        default:
            ESP_LOGW(TAG, "callback: event %d", (int)evt);
            break;
    }
}

/* ============================================================================
 * Internal helpers
 * ==========================================================================*/

static void set_state(MeckAudioState s)
{
    if (!g_state_mu) return;
    xSemaphoreTake(g_state_mu, portMAX_DELAY);
    g_state = s;
    xSemaphoreGive(g_state_mu);
}

/*
 * Once we've handed a FILE* to chmorgan via audio_player_play(), the audio
 * task owns it — when playback ends (EOF, STOP request, or transition to
 * a new PLAY request) the task fcloses the file itself. If we also fclose
 * here we double-close: at best a benign no-op on an already-closed fp,
 * at worst heap corruption that surfaces much later (e.g. a queue handle
 * with garbage uxItemSize causing xSemaphoreTake to assert on the next
 * play call). So this helper just clears our tracking pointer; the only
 * place we fclose ourselves is the error path where audio_player_play
 * returned non-OK and ownership never transferred.
 */
static void forget_current_file(void)
{
    g_fp = nullptr;
}

/*
 * Open file, fseek to target seconds, stash duration/bitrate in state,
 * and hand the FILE* to chmorgan. Returns true if play started.
 */
static bool open_and_play(const char *path, uint32_t target_sec)
{
    /* Old fp (if any) belongs to chmorgan now and will be closed by its
     * task when the new PLAY event preempts it. Just drop our reference. */
    forget_current_file();
    meck_audio_i2s_reset_counter();
    g_seek_start_sec = 0;

    g_fp = fopen(path, "rb");
    if (!g_fp) {
        ESP_LOGE(TAG, "fopen('%s') failed (errno=%d)", path, errno);
        set_state(MECK_AUDIO_STATE_ERROR);
        return false;
    }

    uint32_t duration = 0;
    uint32_t bitrate  = 0;

    FileType ft = detect_type(path);
    if (ft == FT_WAV) {
        WavInfo w;
        if (wav_parse_header(g_fp, &w)) {
            duration = wav_duration_sec(&w);
            bitrate  = w.sample_rate * w.channels * w.bits_per_sample;
            if (target_sec > 0 && target_sec < duration) {
                uint32_t off = wav_byte_offset_for_sec(&w, target_sec);
                fseek(g_fp, off, SEEK_SET);
                g_seek_start_sec = target_sec;
            } else {
                /* Reset to data offset — header was just read. */
                fseek(g_fp, w.data_offset, SEEK_SET);
            }
        } else {
            ESP_LOGW(TAG, "WAV header parse failed; playing from start");
            rewind(g_fp);
        }
    } else if (ft == FT_MP3) {
        mp3_estimate_duration(path, &duration, &bitrate);
        if (target_sec > 0 && duration > 0 && target_sec < duration) {
            uint32_t off = mp3_byte_offset_for_sec(path, target_sec, duration);
            fseek(g_fp, off, SEEK_SET);
            g_seek_start_sec = target_sec;
        } else {
            rewind(g_fp);
        }
    } else {
        ESP_LOGW(TAG, "Unknown file type for '%s'; trying anyway", path);
        rewind(g_fp);
    }

    /* Publish file state. */
    if (g_state_mu && xSemaphoreTake(g_state_mu, pdMS_TO_TICKS(10))) {
        strncpy(g_current_path, path, sizeof(g_current_path));
        g_current_path[sizeof(g_current_path) - 1] = '\0';
        g_duration_sec = duration;
        g_bitrate_bps  = bitrate;
        xSemaphoreGive(g_state_mu);
    }

    esp_err_t err = audio_player_play(g_fp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "audio_player_play failed: %s", esp_err_to_name(err));
        /* audio_player_play returned non-OK: ownership did NOT transfer
         * to chmorgan. We still own this fp, so we close it here. */
        if (g_fp) { fclose(g_fp); g_fp = nullptr; }
        set_state(MECK_AUDIO_STATE_ERROR);
        return false;
    }
    return true;
}

/* ============================================================================
 * Lifecycle
 * ==========================================================================*/

extern "C" bool meck_audio_init(void)
{
    if (g_inited) return true;

    ESP_LOGI(TAG, "meck_audio_init: starting");

    g_state_mu = xSemaphoreCreateMutex();
    if (!g_state_mu) {
        ESP_LOGE(TAG, "mutex alloc failed");
        return false;
    }

    /* Codec + I2S first. */
    if (!meck_audio_es8311_setup(20, 21, 44100)) {
        ESP_LOGE(TAG, "ES8311/I2S setup failed");
        return false;
    }

    /* chmorgan player. The exact config struct member names may vary
     * by chmorgan version — if compile fails here, check the installed
     * audio_player.h. v1.0.7 fields shown below. */
    audio_player_config_t cfg = {};
    cfg.mute_fn      = meck_audio_es8311_mute_cb;
    cfg.write_fn     = meck_audio_i2s_write;
    cfg.clk_set_fn   = meck_audio_i2s_reconfig;
    cfg.priority     = 5;
    cfg.coreID       = 1;             /* core 1; LVGL is on core 0 */
    /* If your chmorgan version uses a different field name (e.g.
     * stack_size vs stack_in_psram), adjust here. */

    esp_err_t err = audio_player_new(cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "audio_player_new failed: %s", esp_err_to_name(err));
        return false;
    }

    err = audio_player_callback_register(player_event_cb, nullptr);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "callback_register failed: %s — proceeding anyway",
                 esp_err_to_name(err));
    }

    /* Apply persisted volume. */
    meck_audio_es8311_set_volume(g_volume_pct);

    g_inited = true;
    set_state(MECK_AUDIO_STATE_IDLE);
    ESP_LOGI(TAG, "meck_audio_init: ready");
    return true;
}

extern "C" bool meck_audio_ready(void) { return g_inited; }

/* ============================================================================
 * Playback control
 * ==========================================================================*/

extern "C" bool meck_audio_play_file(const char *path, uint32_t resume_sec)
{
    if (!path || !*path) return false;
    if (!g_inited && !meck_audio_init()) return false;

    /* Save bookmark for the outgoing track if we were playing. */
    if (g_fp && g_current_path[0]) {
        uint32_t cur = meck_audio_get_position_sec();
        if (cur > 5) bookmark_save_for(g_current_path, cur);
    }

    audio_player_stop();
    set_state(MECK_AUDIO_STATE_LOADING);

    /* Use rewind-3-seconds for context, like the previous version. */
    uint32_t target = (resume_sec > 3) ? (resume_sec - 3) : 0;
    if (!open_and_play(path, target)) {
        return false;
    }

    /* Apply current volume (codec may have lost it during a reconfig). */
    meck_audio_es8311_set_volume(g_volume_pct);

    ESP_LOGI(TAG, "playing '%s' from %us (bookmark resume=%us)",
             path, (unsigned)target, (unsigned)resume_sec);
    return true;
}

extern "C" void meck_audio_pause(void)
{
    if (!g_inited) return;
    audio_player_pause();
    /* Callback will flip state; set proactively for snappy UI. */
    set_state(MECK_AUDIO_STATE_PAUSED);
}

extern "C" void meck_audio_resume(void)
{
    if (!g_inited) return;
    audio_player_resume();
    set_state(MECK_AUDIO_STATE_PLAYING);
}

extern "C" void meck_audio_toggle_pause(void)
{
    MeckAudioState s = meck_audio_get_state();
    if (s == MECK_AUDIO_STATE_PLAYING) meck_audio_pause();
    else if (s == MECK_AUDIO_STATE_PAUSED) meck_audio_resume();
}

extern "C" void meck_audio_stop(void)
{
    if (!g_inited) return;
    uint32_t cur = meck_audio_get_position_sec();
    if (g_current_path[0] && cur > 5) bookmark_save_for(g_current_path, cur);
    audio_player_stop();
    /* chmorgan's audio_task will fclose the fp when it processes the
     * STOP event. We just drop our reference. */
    forget_current_file();
    meck_audio_i2s_reset_counter();
    g_seek_start_sec = 0;
    set_state(MECK_AUDIO_STATE_IDLE);
}

/*
 * Seek = stop → reopen file → fseek → replay. Brief audible blip; this
 * is the documented chmorgan-friendly seek pattern. We re-use the
 * open_and_play() path so all the same WAV/MP3 logic applies.
 */
static void seek_to_absolute_locked(uint32_t target_sec)
{
    if (!g_current_path[0]) return;

    /* Snapshot path before we touch state. */
    char path[256];
    strncpy(path, g_current_path, sizeof(path));
    path[sizeof(path) - 1] = '\0';

    audio_player_stop();
    /* Don't save bookmark here — the seek is user-driven, not a
     * pause/stop event. */
    open_and_play(path, target_sec);
    meck_audio_es8311_set_volume(g_volume_pct);
}

extern "C" void meck_audio_seek_relative(int32_t seconds)
{
    if (!g_inited || !g_current_path[0]) return;
    int64_t cur    = (int64_t)meck_audio_get_position_sec();
    int64_t target = cur + seconds;
    if (target < 0) target = 0;
    uint32_t dur = meck_audio_get_duration_sec();
    if (dur > 0 && (uint64_t)target > dur) target = dur;
    seek_to_absolute_locked((uint32_t)target);
}

extern "C" void meck_audio_seek_absolute(uint32_t position_sec)
{
    if (!g_inited || !g_current_path[0]) return;
    uint32_t dur = meck_audio_get_duration_sec();
    if (dur > 0 && position_sec > dur) position_sec = dur;
    seek_to_absolute_locked(position_sec);
}

/* ============================================================================
 * Volume
 * ==========================================================================*/

extern "C" uint8_t meck_audio_get_volume_pct(void)
{
    uint8_t v = 50;
    if (g_state_mu && xSemaphoreTake(g_state_mu, pdMS_TO_TICKS(5))) {
        v = g_volume_pct;
        xSemaphoreGive(g_state_mu);
    }
    return v;
}

extern "C" void meck_audio_set_volume_pct(uint8_t pct)
{
    if (pct > 100) pct = 100;
    if (g_state_mu && xSemaphoreTake(g_state_mu, pdMS_TO_TICKS(5))) {
        g_volume_pct = pct;
        xSemaphoreGive(g_state_mu);
    }
    meck_audio_es8311_set_volume(pct);
}

extern "C" void meck_audio_set_dac_mute(bool muted)
{
    meck_audio_es8311_set_mute(muted);
}

/* ============================================================================
 * Status queries
 *
 * Position is derived from the byte counter in meck_es8311.c plus the
 * seek-start offset. UI calls this from its refresh timer at ~4 Hz.
 * ==========================================================================*/

extern "C" MeckAudioState meck_audio_get_state(void)
{
    MeckAudioState s = MECK_AUDIO_STATE_IDLE;
    if (g_state_mu && xSemaphoreTake(g_state_mu, pdMS_TO_TICKS(5))) {
        s = g_state;
        xSemaphoreGive(g_state_mu);
    }
    return s;
}

extern "C" uint32_t meck_audio_get_position_sec(void)
{
    uint64_t bytes = meck_audio_i2s_bytes_written();
    uint32_t rate  = meck_audio_i2s_current_rate();
    uint8_t  chans = meck_audio_i2s_current_chans();
    uint8_t  bps   = meck_audio_i2s_current_bps();
    if (rate == 0 || chans == 0 || bps == 0) return g_seek_start_sec;
    uint32_t bytes_per_sec = rate * chans * (bps / 8);
    if (bytes_per_sec == 0) return g_seek_start_sec;
    return g_seek_start_sec + (uint32_t)(bytes / bytes_per_sec);
}

extern "C" uint32_t meck_audio_get_duration_sec(void)
{
    uint32_t v = 0;
    if (g_state_mu && xSemaphoreTake(g_state_mu, pdMS_TO_TICKS(5))) {
        v = g_duration_sec;
        xSemaphoreGive(g_state_mu);
    }
    return v;
}

extern "C" uint32_t meck_audio_get_bitrate(void)
{
    uint32_t v = 0;
    if (g_state_mu && xSemaphoreTake(g_state_mu, pdMS_TO_TICKS(5))) {
        v = g_bitrate_bps;
        xSemaphoreGive(g_state_mu);
    }
    return v;
}

extern "C" const char *meck_audio_get_current_path(void)
{
    return g_current_path;
}

extern "C" bool meck_audio_consume_eof_flag(void)
{
    if (g_eof_flag) {
        g_eof_flag = 0;
        return true;
    }
    return false;
}
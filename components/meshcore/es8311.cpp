/*
 * es8311.cpp -- Real audio output via cpp_bus_driver
 *
 * Replaces the diagnostic stub. PCM goes to ES8311->write_data() on the
 * same I2S channel LilyGo's ES8311_Init() configured at boot, sharing the
 * I2C/I2S buses with the rest of the codebase (no driver conflict).
 *
 * Public API stays plain C so MeckAudio.cpp's extern "C" block links
 * unchanged. ES8311 codec calls happen inside C++ scope.
 *
 * Notes:
 *   - The codec object (ES8311) is owned by main.cpp at line 396, created
 *     as a unique_ptr<Cpp_Bus_Driver::Es8311>. We extern-reference it.
 *   - ES8311_Init() runs at boot before our setup() is ever called, so the
 *     codec is configured for 44.1 kHz / 16-bit by the time we get here.
 *   - write_data() blocks until DMA accepts the data, providing natural
 *     back-pressure to chmorgan's decode task. No vTaskDelay needed.
 *   - Volume is hardware (set_dac_volume 0-255). Mute is implemented as
 *     volume=0 / restore-on-unmute because no set_dac_mute is used in
 *     LilyGo's main.cpp and I haven't verified the method exists.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <memory>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "audio_player.h"
#include "cpp_bus_driver_library.h"

/* Codec object owned by main.cpp. Same instance ES8311_Init() configures
 * at boot — shares the I2C and I2S buses with the rest of the device. */
extern std::unique_ptr<Cpp_Bus_Driver::Es8311> ES8311;

static const char *TAG = "meck_es8311";

static bool     g_ready          = false;
static uint64_t g_bytes_written_total = 0;
static uint32_t g_current_rate   = 44100;
static uint8_t  g_current_bps    = 16;
static uint8_t  g_current_chans  = 2;
static uint8_t  g_current_vol    = 50;
static bool     g_muted          = false;

/* Helper: apply the current volume to the codec. Used by both
 * set_volume() and unmute paths so the curve is in one place. */
static void apply_hw_volume(uint8_t pct)
{
    if (!ES8311) return;
    /* Linear map 0..100% to ES8311's 0..255 register range. The codec's
     * internal curve is already roughly logarithmic, so a linear UI feel
     * comes out about right at this stage. */
    uint8_t hw = (uint8_t)((uint32_t)pct * 255u / 100u);
    ES8311->set_dac_volume(hw);
}

extern "C" {

bool meck_audio_es8311_setup(int sda_pin_unused, int scl_pin_unused, uint32_t rate)
{
    (void)sda_pin_unused;
    (void)scl_pin_unused;
    g_current_rate = rate ? rate : 44100;

    if (!ES8311) {
        ESP_LOGE(TAG, "setup: ES8311 global not initialized; was ES8311_Init() called?");
        return false;
    }

    /* libhelix-mp3 logs at ERROR per failed frame sync. ID3v2 tags at the
     * head of MP3 files cause thousands of these per second until the
     * decoder finds the first real frame. ESP_LOG_NONE silences the
     * channel entirely; player-level errors come through MeckAudio's own
     * logging, not libhelix's, so we don't lose useful diagnostics by
     * silencing this one tag. */
    esp_log_level_set("mp3", ESP_LOG_NONE);

    g_ready = true;
    ESP_LOGI(TAG, "setup ok (rate=%lu) — routing PCM to ES8311->write_data",
             (unsigned long)g_current_rate);
    return true;
}

void meck_audio_i2s_reset_counter(void)        { g_bytes_written_total = 0; }
uint64_t meck_audio_i2s_bytes_written(void)    { return g_bytes_written_total; }
uint32_t meck_audio_i2s_current_rate(void)     { return g_current_rate; }
uint8_t  meck_audio_i2s_current_chans(void)    { return g_current_chans; }
uint8_t  meck_audio_i2s_current_bps(void)      { return g_current_bps; }
bool     meck_audio_es8311_is_ready(void)      { return g_ready; }

/* write_fn for chmorgan/esp-audio-player. write_data blocks on DMA, which
 * paces the decode task naturally; no explicit throttle needed.
 *
 * Diagnostic instrumentation (May 2026): logs first call, every 500th call,
 * and any zero-byte return. Also yields one tick every 500 calls to keep
 * IDLE1 alive in case the underlying DMA queue is full — without the yield
 * a stuck codec causes the task watchdog to fire after 5s. */
esp_err_t meck_audio_i2s_write(void *audio_buffer, size_t len,
                               size_t *bytes_written, uint32_t timeout_ticks)
{
    (void)timeout_ticks;  /* cpp_bus_driver write_data has no timeout arg */
    if (!ES8311 || !audio_buffer || !bytes_written) {
        if (bytes_written) *bytes_written = 0;
        return ESP_ERR_INVALID_STATE;
    }

    static uint32_t s_call_count   = 0;
    static uint32_t s_zero_returns = 0;
    s_call_count++;

    if (s_call_count == 1) {
        ESP_LOGI(TAG, "i2s_write: first call (len=%u)", (unsigned)len);
    }

    size_t written = ES8311->write_data((const char *)audio_buffer, len);
    *bytes_written = written;
    g_bytes_written_total += written;

    if (written == 0) {
        s_zero_returns++;
        if (s_zero_returns == 1 || (s_zero_returns % 100) == 0) {
            ESP_LOGW(TAG, "i2s_write: write_data returned 0 (count=%u, "
                          "total_calls=%u, total_bytes=%llu)",
                     (unsigned)s_zero_returns,
                     (unsigned)s_call_count,
                     (unsigned long long)g_bytes_written_total);
        }
    }

    if ((s_call_count % 500) == 0) {
        ESP_LOGI(TAG, "i2s_write: %u calls, %llu bytes, %u zero-returns",
                 (unsigned)s_call_count,
                 (unsigned long long)g_bytes_written_total,
                 (unsigned)s_zero_returns);
        /* Yield one tick to keep IDLE1 alive. Without this a stuck codec
         * (DMA queue never drains because hardware isn't consuming) starves
         * IDLE1 for >5s and fires the task watchdog. The yield costs ~10ms
         * per 500 frames which is inaudible. */
        vTaskDelay(1);
    }

    return ESP_OK;
}

/* clk_set_fn — chmorgan calls this when a new track opens. Push the new
 * rate / bit depth to the codec so the I2S clock matches the source. */
esp_err_t meck_audio_i2s_reconfig(uint32_t rate, uint32_t bps,
                                  i2s_slot_mode_t channels)
{
    if (!ES8311) return ESP_ERR_INVALID_STATE;

    g_current_rate  = rate;
    g_current_bps   = (uint8_t)bps;
    g_current_chans = (channels == I2S_SLOT_MODE_STEREO) ? 2 : 1;

    ESP_LOGI(TAG, "reconfig: rate=%lu bps=%lu chans=%d",
             (unsigned long)rate, (unsigned long)bps, (int)g_current_chans);

    /* Match LilyGo's ES8311_Init: clock_coeff takes the same MCLK_MULTIPLE
     * enum used at boot. Hardcoded 256x — chmorgan/libhelix always outputs
     * at the file's native rate so we only ever need to change the rate. */
    ES8311->set_clock_coeff(i2s_mclk_multiple_t::I2S_MCLK_MULTIPLE_256, rate);

    /* Bit depth: libhelix-mp3 always decodes to 16-bit; WAV can be 16 or
     * 24. Only push if it actually changed from the boot default (16). */
    if (bps == 24) {
        ES8311->set_sdp_data_bit_length(
            Cpp_Bus_Driver::Es8311::Sdp::DAC,
            Cpp_Bus_Driver::Es8311::Bits_Per_Sample::DATA_24BIT);
    } else {
        ES8311->set_sdp_data_bit_length(
            Cpp_Bus_Driver::Es8311::Sdp::DAC,
            Cpp_Bus_Driver::Es8311::Bits_Per_Sample::DATA_16BIT);
    }

    return ESP_OK;
}

void meck_audio_es8311_set_volume(uint8_t pct)
{
    g_current_vol = pct;
    if (g_muted) return;     /* let unmute restore the value */
    apply_hw_volume(pct);
    ESP_LOGD(TAG, "set_volume=%u%%", (unsigned)pct);
}

void meck_audio_es8311_set_mute(bool muted)
{
    g_muted = muted;
    if (!ES8311) return;
    if (muted) {
        ES8311->set_dac_volume(0);
    } else {
        apply_hw_volume(g_current_vol);
    }
}

esp_err_t meck_audio_es8311_mute_cb(AUDIO_PLAYER_MUTE_SETTING setting)
{
    meck_audio_es8311_set_mute(setting == AUDIO_PLAYER_MUTE);
    return ESP_OK;
}

} /* extern "C" */
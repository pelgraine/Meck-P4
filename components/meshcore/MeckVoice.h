/*
 * MeckVoice.h -- Voice message recording, encoding, and VE3 protocol for P4
 *
 * Codec2 1200bps voice codec over MeshCore's PAYLOAD_TYPE_RAW_CUSTOM.
 * Wire-compatible with upstream Meck (T-Deck Pro) VoiceMessageScreen.h.
 *
 * Architecture:
 *   es8311.cpp mic functions  -->  this file (buffer + Codec2)  -->  MeckMesh.h (send/recv)
 *                                                                      |
 *   es8311.cpp DAC playback  <--  this file (decode + WAV)     <--  MeckUI.cpp (voice screen)
 *
 * Threading: all functions are called from the LVGL task unless noted.
 * The mic read loop runs in a short burst (up to 12s) blocking the UI,
 * which is acceptable for a hold-to-record interaction.
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"

// Codec2 -- if not available yet, guard with #ifdef HAVE_CODEC2
#ifdef HAVE_CODEC2
#include <codec2.h>
#endif

// Debug Logs
#include "meck_log.h"

// ---- ES8311 mic capture (defined in es8311.cpp) ----
extern "C" {
    bool   meck_audio_mic_start(uint32_t sample_rate);
    size_t meck_audio_mic_read(void *buf, size_t len);
    void   meck_audio_mic_stop(void);
    bool   meck_audio_mic_is_active(void);
}

// ---- SD card file I/O (from MeckDataStore / MeckSDCard.h) ----
// p4_sdcard_is_mounted() is already visible via MeckSDCard.h included
// through MeckDataStore.h -> MeckMesh.h -> MeckUI.cpp. No redeclaration needed.

// ============================================================================
// VE3 Voice Protocol Constants (must match upstream Meck)
// ============================================================================

#define VOICE_PKT_MAGIC       0x56  // 'V' -- voice data packet
#define VOICE_FETCH_MAGIC     0x72  // 'r' -- voice fetch request
#define VOICE_PKT_HDR_SIZE    6     // magic(1) + sessionID(4) + index(1)
#define VOICE_SESSION_TTL_MS  900000  // 15 minutes cache TTL
#define VOICE_C2_MODE_ID      1     // Codec2 1200bps mode ID for VE3 protocol

// ============================================================================
// Recording Configuration
// ============================================================================

#define VOICE_MAX_SECONDS     12
#define VOICE_SAMPLE_RATE     44100  // Native I2S rate (don't change the bus clock)
#define VOICE_BITS            16
#define VOICE_CHANNELS        1

// Codec2 encoding config
#define VOICE_C2_RATE         8000   // Codec2 native sample rate
#define VOICE_C2_FRAME_MS     40     // Frame duration at 1200bps
#define VOICE_C2_FRAME_SAM    320    // Samples per frame (8kHz x 40ms)
#define VOICE_C2_FRAME_BYTES  6      // Encoded bytes per frame (48 bits)

// Max encoded size: 12 seconds = 300 frames x 6 bytes = 1800 bytes
#define VOICE_C2_MAX_BYTES    ((VOICE_MAX_SECONDS * 1000 / VOICE_C2_FRAME_MS) * VOICE_C2_FRAME_BYTES)

// Usable codec2 data per raw voice packet.
// Keep under ~150 to avoid MAX_PACKET_PAYLOAD (184) boundary issues.
#define VOICE_MESH_PAYLOAD    150

// Buffer: 44100Hz stereo x 12s = 1,058,400 int16_t values during capture.
// After deinterleave in stopRecording, halved to mono.
#define VOICE_BUF_SAMPLES     (VOICE_SAMPLE_RATE * VOICE_MAX_SECONDS * 2)
#define VOICE_BUF_BYTES       (VOICE_BUF_SAMPLES * sizeof(int16_t))

// SD card voice folder
#define VOICE_FOLDER          "/sdcard/voice"

// ============================================================================
// WAV header writer (44-byte RIFF/WAVE PCM header)
// ============================================================================

static inline void meck_voice_write_wav_header(FILE *f, uint32_t dataBytes,
                                                uint32_t sampleRate,
                                                uint16_t bitsPerSample,
                                                uint16_t channels)
{
    uint32_t byteRate   = sampleRate * channels * (bitsPerSample / 8);
    uint16_t blockAlign = channels * (bitsPerSample / 8);
    uint32_t chunkSize  = 36 + dataBytes;
    uint16_t audioFmt   = 1;  // PCM
    uint32_t fmtSize    = 16;

    fwrite("RIFF", 1, 4, f);
    fwrite(&chunkSize, 4, 1, f);
    fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f);
    fwrite(&fmtSize, 4, 1, f);
    fwrite(&audioFmt, 2, 1, f);
    fwrite(&channels, 2, 1, f);
    fwrite(&sampleRate, 4, 1, f);
    fwrite(&byteRate, 4, 1, f);
    fwrite(&blockAlign, 2, 1, f);
    fwrite(&bitsPerSample, 2, 1, f);
    fwrite("data", 1, 4, f);
    fwrite(&dataBytes, 4, 1, f);
}

// ============================================================================
// VE3 Base36 helpers (compact wire format, matches upstream)
// ============================================================================

static inline int meck_voice_to_base36(uint32_t val, char *buf, int bufLen)
{
    if (bufLen < 2) return 0;
    if (val == 0) { buf[0] = '0'; buf[1] = '\0'; return 1; }
    char tmp[16];
    int pos = 0;
    while (val > 0 && pos < 15) {
        uint8_t d = val % 36;
        tmp[pos++] = d < 10 ? ('0' + d) : ('a' + d - 10);
        val /= 36;
    }
    int len = pos < bufLen - 1 ? pos : bufLen - 1;
    for (int i = 0; i < len; i++) buf[i] = tmp[pos - 1 - i];
    buf[len] = '\0';
    return len;
}

static inline uint32_t meck_voice_from_base36(const char *s)
{
    uint32_t val = 0;
    while (*s) {
        val *= 36;
        char c = *s++;
        if (c >= '0' && c <= '9') val += c - '0';
        else if (c >= 'a' && c <= 'z') val += 10 + c - 'a';
        else if (c >= 'A' && c <= 'Z') val += 10 + c - 'A';
    }
    return val;
}

// ============================================================================
// MeckVoice -- voice recording buffer, Codec2, WAV save
// ============================================================================

class MeckVoice {
public:
    MeckVoice() {
        memset(&_outSession, 0, sizeof(_outSession));
        memset(&_inSession, 0, sizeof(_inSession));
    }

    // ---- Recording buffer ----

    bool ensureRecBuffer() {
        if (_recBuffer) return true;
        _recBuffer = (int16_t *)heap_caps_calloc(
            VOICE_BUF_SAMPLES, sizeof(int16_t), MALLOC_CAP_SPIRAM);
        if (!_recBuffer) {
            printf("MeckVoice: PSRAM alloc failed for rec buffer (%d bytes)\n",
                   (int)VOICE_BUF_BYTES);
            return false;
        }
        printf("MeckVoice: allocated rec buffer (%d bytes in PSRAM)\n",
               (int)VOICE_BUF_BYTES);
        return true;
    }

    // ---- Recording ----

    bool startRecording() {
        if (_recording) return false;
        if (!ensureRecBuffer()) return false;

        // Stop any active playback before capturing
        extern void meck_audio_stop(void);
        meck_audio_stop();

        if (!meck_audio_mic_start(VOICE_SAMPLE_RATE)) {
            printf("MeckVoice: mic start failed\n");
            return false;
        }

        _recSamples = 0;
        _recording = true;
        _recStartMs = (uint32_t)(esp_timer_get_time() / 1000ULL);
        printf("MeckVoice: recording started\n");
        return true;
    }

    // Call this from the UI timer while recording.
    // Reads audio in a time-bounded loop (~150ms max) to stay responsive
    // to touch events. Some data may be lost to DMA overflow between
    // ticks, but voice is tolerant of small gaps.
    uint32_t recordTick() {
        if (!_recording || !_recBuffer) return _recSamples;

        // Check time limit
        uint32_t elapsed = (uint32_t)(esp_timer_get_time() / 1000ULL) - _recStartMs;
        if (elapsed >= VOICE_MAX_SECONDS * 1000) {
            stopRecording();
            return _recSamples;
        }

        uint32_t remaining = VOICE_BUF_SAMPLES - _recSamples;
        if (remaining == 0) {
            stopRecording();
            return _recSamples;
        }

        // Read in a time-bounded loop: max ~400ms blocking per tick.
        // This captures ~80% of the stereo data (vs 100% at full blocking
        // but with an unresponsive UI). The ~100ms gap between ticks lets
        // LVGL process touch events so the stop button responds within
        // one tick cycle. Some DMA overflow is acceptable for voice.
        uint32_t tickStartUs = (uint32_t)(esp_timer_get_time());
        uint32_t maxBlockUs = 400000;  // 400ms

        while (remaining > 0) {
            size_t chunkBytes = 2048;
            if (chunkBytes > remaining * sizeof(int16_t))
                chunkBytes = remaining * sizeof(int16_t);

            size_t got = meck_audio_mic_read(
                &_recBuffer[_recSamples], chunkBytes);
            if (got == 0) break;
            uint32_t gotSamples = got / sizeof(int16_t);
            _recSamples += gotSamples;
            remaining -= gotSamples;

            // Check if we've spent enough time in this tick
            uint32_t nowUs = (uint32_t)(esp_timer_get_time());
            if (nowUs - tickStartUs >= maxBlockUs) break;
        }

        return _recSamples;
    }

    void stopRecording() {
        if (!_recording) return;
        _recording = false;
        meck_audio_mic_stop();

        // The I2S bus runs in stereo mode. read_data() returns interleaved
        // L/R pairs: [L0, R0, L1, R1, ...]. The mic audio is on the L
        // channel. Deinterleave in-place, keeping only L samples, and
        // halve the sample count. The effective rate stays at 44100Hz.
        if (_recBuffer && _recSamples > 1) {
            uint32_t monoCount = _recSamples / 2;
            for (uint32_t i = 0; i < monoCount; i++) {
                _recBuffer[i] = _recBuffer[i * 2];  // keep L, skip R
            }
            printf("MeckVoice: deinterleaved stereo %lu -> %lu mono samples\n",
                   (unsigned long)_recSamples, (unsigned long)monoCount);
            _recSamples = monoCount;
        }

        printf("MeckVoice: recording stopped, %lu samples (%.1fs)\n",
               (unsigned long)_recSamples,
               _recSamples / (float)VOICE_SAMPLE_RATE);
    }

    bool isRecording() const { return _recording; }
    uint32_t getRecSamples() const { return _recSamples; }
    float getRecDurationSec() const {
        if (_recording) {
            // During recording, use wall clock (raw samples are stereo)
            return (uint32_t)(esp_timer_get_time() / 1000ULL - _recStartMs) / 1000.0f;
        }
        // After deinterleave, _recSamples is mono count
        return _recSamples / (float)VOICE_SAMPLE_RATE;
    }

    // ---- Audio normalization ----

    void normalizeRecording() {
        if (!_recBuffer || _recSamples == 0) return;

        int16_t peak = 0;
        for (uint32_t i = 0; i < _recSamples; i++) {
            int16_t s = _recBuffer[i];
            int16_t absS = (s < 0) ? -s : s;
            if (absS > peak) peak = absS;
        }
        if (peak < 100) {
            printf("MeckVoice: near-silent, skipping normalization\n");
            return;
        }

        int32_t target = 29491;  // 0.9 * 32767
        int32_t gain16 = (target << 16) / peak;

        for (uint32_t i = 0; i < _recSamples; i++) {
            int32_t amplified = ((int32_t)_recBuffer[i] * gain16) >> 16;
            if (amplified > 32767) amplified = 32767;
            if (amplified < -32768) amplified = -32768;
            _recBuffer[i] = (int16_t)amplified;
        }
        printf("MeckVoice: normalized (peak=%d, gain=%.1fx)\n",
               peak, gain16 / 65536.0f);
    }

    // ---- Codec2 encoding ----

#ifdef HAVE_CODEC2
    void encodeCodec2() {
        _c2Bytes = 0;
        _c2Frames = 0;
        _c2Valid = false;

        if (_recSamples < 4000) {
            printf("MeckVoice: too few samples for Codec2\n");
            return;
        }

        struct CODEC2 *c2 = codec2_create(CODEC2_MODE_1200);
        if (!c2) {
            printf("MeckVoice: codec2_create failed\n");
            return;
        }

        int frameSamples = codec2_samples_per_frame(c2);  // 320 at 8kHz
        int frameBytes = (codec2_bits_per_frame(c2) + 7) / 8;  // 6 at 1200bps

        // Downsample ratio: 44100/8000 = 5.5125
        float dsRatio = (float)VOICE_SAMPLE_RATE / (float)VOICE_C2_RATE;
        // Source samples consumed per Codec2 frame
        int srcPerFrame = (int)(frameSamples * dsRatio + 0.5f);  // ~1764

        int16_t frameBuf[VOICE_C2_FRAME_SAM];
        uint32_t srcPos = 0;

        while (srcPos + srcPerFrame <= _recSamples &&
               _c2Bytes + frameBytes <= VOICE_C2_MAX_BYTES) {
            // Downsample this frame using linear interpolation
            for (int i = 0; i < frameSamples; i++) {
                float srcIdx = srcPos + i * dsRatio;
                uint32_t idx = (uint32_t)srcIdx;
                float frac = srcIdx - idx;
                if (idx + 1 < _recSamples) {
                    frameBuf[i] = (int16_t)((1.0f - frac) * _recBuffer[idx]
                                          + frac * _recBuffer[idx + 1]);
                } else if (idx < _recSamples) {
                    frameBuf[i] = _recBuffer[idx];
                } else {
                    frameBuf[i] = 0;
                }
            }

            codec2_encode(c2, &_c2Data[_c2Bytes], frameBuf);
            _c2Bytes += frameBytes;
            _c2Frames++;
            srcPos += srcPerFrame;
        }

        codec2_destroy(c2);
        _c2Valid = (_c2Frames > 0);

        printf("MeckVoice: Codec2 encoded %lu frames, %lu bytes (%.1fs, %d packets)\n",
               (unsigned long)_c2Frames, (unsigned long)_c2Bytes,
               _c2Frames * VOICE_C2_FRAME_MS / 1000.0f,
               (_c2Bytes + VOICE_MESH_PAYLOAD - 1) / VOICE_MESH_PAYLOAD);
    }

    // Decode Codec2 data into the recording buffer for playback
    bool decodeCodec2(const uint8_t *c2data, uint32_t c2bytes) {
        if (!ensureRecBuffer()) return false;

        struct CODEC2 *c2 = codec2_create(CODEC2_MODE_1200);
        if (!c2) return false;

        int frameSamples = codec2_samples_per_frame(c2);
        int frameBytes = (codec2_bits_per_frame(c2) + 7) / 8;

        // Upsample ratio: 44100/8000 = 5.5125
        float usRatio = (float)VOICE_SAMPLE_RATE / (float)VOICE_C2_RATE;
        int outPerFrame = (int)(frameSamples * usRatio + 0.5f);  // ~1764

        int16_t frameBuf[VOICE_C2_FRAME_SAM];
        uint32_t srcPos = 0;
        _recSamples = 0;

        while (srcPos + frameBytes <= c2bytes &&
               _recSamples + outPerFrame <= VOICE_BUF_SAMPLES) {
            codec2_decode(c2, frameBuf, &c2data[srcPos]);
            // Upsample 8kHz->44100Hz using linear interpolation
            float dsRatio = (float)VOICE_C2_RATE / (float)VOICE_SAMPLE_RATE;
            for (int o = 0; o < outPerFrame; o++) {
                float srcIdx = o * dsRatio;
                int idx = (int)srcIdx;
                float frac = srcIdx - idx;
                if (idx + 1 < frameSamples) {
                    _recBuffer[_recSamples++] = (int16_t)(
                        (1.0f - frac) * frameBuf[idx] + frac * frameBuf[idx + 1]);
                } else if (idx < frameSamples) {
                    _recBuffer[_recSamples++] = frameBuf[idx];
                }
            }
            srcPos += frameBytes;
        }

        codec2_destroy(c2);
        printf("MeckVoice: decoded %lu bytes -> %lu samples (%.1fs)\n",
               (unsigned long)c2bytes, (unsigned long)_recSamples,
               _recSamples / (float)VOICE_SAMPLE_RATE);
        return _recSamples > 0;
    }
#endif  // HAVE_CODEC2

    bool isCodec2Valid() const { return _c2Valid; }
    uint32_t getC2Bytes() const { return _c2Bytes; }
    const uint8_t *getC2Data() const { return _c2Data; }

    // ---- WAV load from SD (for re-encoding previous recordings) ----

    bool loadFromWav(const char *path) {
        if (!ensureRecBuffer()) return false;

        FILE *f = fopen(path, "rb");
        if (!f) {
            printf("MeckVoice: loadFromWav: can't open %s\n", path);
            return false;
        }

        // Read and validate WAV header
        uint8_t hdr[44];
        if (fread(hdr, 1, 44, f) != 44) {
            printf("MeckVoice: loadFromWav: header too short\n");
            fclose(f);
            return false;
        }
        if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) {
            printf("MeckVoice: loadFromWav: not a WAV file\n");
            fclose(f);
            return false;
        }

        uint16_t channels = hdr[22] | (hdr[23] << 8);
        uint32_t sampleRate = hdr[24] | (hdr[25] << 8) | (hdr[26] << 16) | (hdr[27] << 24);
        uint16_t bitsPerSample = hdr[34] | (hdr[35] << 8);
        uint32_t dataSize = hdr[40] | (hdr[41] << 8) | (hdr[42] << 16) | (hdr[43] << 24);

        printf("MeckVoice: loadFromWav: %luHz %dch %dbit %lu bytes\n",
               (unsigned long)sampleRate, channels, bitsPerSample,
               (unsigned long)dataSize);

        if (bitsPerSample != 16) {
            printf("MeckVoice: loadFromWav: only 16-bit supported\n");
            fclose(f);
            return false;
        }

        // Read samples, de-interleave stereo to mono (keep L channel)
        _recSamples = 0;
        uint32_t totalFrames = dataSize / (channels * sizeof(int16_t));
        int16_t frame[2];  // max 2 channels

        for (uint32_t i = 0; i < totalFrames && _recSamples < VOICE_BUF_SAMPLES; i++) {
            if (fread(frame, sizeof(int16_t), channels, f) != channels) break;
            _recBuffer[_recSamples++] = frame[0];  // L channel only
        }

        fclose(f);
        printf("MeckVoice: loadFromWav: loaded %lu mono samples (%.1fs)\n",
               (unsigned long)_recSamples,
               _recSamples / (float)VOICE_SAMPLE_RATE);
        return _recSamples > 0;
    }

    // ---- WAV save to SD ----

    bool saveToWav(const char *filename) {
        if (!_recBuffer || _recSamples < VOICE_SAMPLE_RATE / 4) {
            printf("MeckVoice: too short to save\n");
            return false;
        }
        if (!p4_sdcard_is_mounted()) {
            printf("MeckVoice: SD not mounted\n");
            return false;
        }

        struct stat st;
        if (stat(VOICE_FOLDER, &st) != 0) {
            mkdir(VOICE_FOLDER, 0755);
        }

        char path[96];
        snprintf(path, sizeof(path), "%s/%s", VOICE_FOLDER, filename);

        normalizeRecording();

        FILE *f = fopen(path, "wb");
        if (!f) {
            printf("MeckVoice: fopen(%s) failed\n", path);
            return false;
        }

        // Write as stereo at the native capture rate. The ES8311 I2S
        // bus is stereo-only, so duplicate each mono sample to L+R.
        uint16_t outChannels = 2;
        uint32_t dataBytes = _recSamples * sizeof(int16_t) * outChannels;
        meck_voice_write_wav_header(f, dataBytes, VOICE_SAMPLE_RATE,
                                    VOICE_BITS, outChannels);

        // Write in bulk chunks for speed (sample-at-a-time fwrite is slow)
        int16_t chunk[512];
        uint32_t pos = 0;
        while (pos < _recSamples) {
            int ci = 0;
            while (ci < 512 && pos < _recSamples) {
                chunk[ci++] = _recBuffer[pos];  // L
                chunk[ci++] = _recBuffer[pos];  // R
                pos++;
            }
            fwrite(chunk, sizeof(int16_t), ci, f);
        }
        fclose(f);

        printf("MeckVoice: saved %s (%lu bytes, %luHz stereo, %.1fs)\n",
               path, (unsigned long)(44 + dataBytes),
               (unsigned long)VOICE_SAMPLE_RATE,
               _recSamples / (float)VOICE_SAMPLE_RATE);
        return true;
    }

    // ---- VE3 envelope formatting ----

    void formatEnvelope(char *buf, int bufLen, uint32_t sessionId) {
        char sid[12], mode[4], total[4], dur[4];
        int totalPackets = (_c2Bytes + VOICE_MESH_PAYLOAD - 1) / VOICE_MESH_PAYLOAD;
        int durationSec = (int)(_c2Frames * VOICE_C2_FRAME_MS / 1000);

        meck_voice_to_base36(sessionId, sid, sizeof(sid));
        meck_voice_to_base36(VOICE_C2_MODE_ID, mode, sizeof(mode));
        meck_voice_to_base36(totalPackets, total, sizeof(total));
        meck_voice_to_base36(durationSec, dur, sizeof(dur));
        snprintf(buf, bufLen, "VE3:%s:%s:%s:%s", sid, mode, total, dur);
    }

    // Parse a VE3 envelope string. Returns true if valid.
    static bool parseVE3(const char *text, uint32_t *sessionId,
                         uint8_t *totalPackets, uint8_t *durationSec) {
        if (strncmp(text, "VE3:", 4) != 0) return false;
        const char *p = text + 4;

        // sessionId
        const char *sep = strchr(p, ':');
        if (!sep) return false;
        char field[16];
        int len = sep - p;
        if (len <= 0 || len >= 16) return false;
        memcpy(field, p, len); field[len] = '\0';
        *sessionId = meck_voice_from_base36(field);
        p = sep + 1;

        // mode (skip)
        sep = strchr(p, ':');
        if (!sep) return false;
        p = sep + 1;

        // totalPackets
        sep = strchr(p, ':');
        if (!sep) return false;
        len = sep - p;
        if (len <= 0 || len >= 16) return false;
        memcpy(field, p, len); field[len] = '\0';
        *totalPackets = (uint8_t)meck_voice_from_base36(field);
        p = sep + 1;

        // durationSec
        *durationSec = (uint8_t)meck_voice_from_base36(p);
        return true;
    }

    // ---- Voice packet building (for send) ----

    // Build a voice data packet: [0x56][sessionId:4B][index:1B][codec2 data...]
    int buildVoicePacket(uint8_t *buf, int bufLen, uint32_t sessionId, int pktIndex) {
        if (!_c2Valid || pktIndex < 0) return 0;

        int offset = pktIndex * VOICE_MESH_PAYLOAD;
        if ((uint32_t)offset >= _c2Bytes) return 0;

        int dataLen = _c2Bytes - offset;
        if (dataLen > VOICE_MESH_PAYLOAD) dataLen = VOICE_MESH_PAYLOAD;

        int totalLen = VOICE_PKT_HDR_SIZE + dataLen;
        if (totalLen > bufLen) return 0;

        buf[0] = VOICE_PKT_MAGIC;
        memcpy(&buf[1], &sessionId, 4);
        buf[5] = (uint8_t)pktIndex;
        memcpy(&buf[6], &_c2Data[offset], dataLen);

        return totalLen;
    }

    int getPacketCount() const {
        if (!_c2Valid || _c2Bytes == 0) return 0;
        return (_c2Bytes + VOICE_MESH_PAYLOAD - 1) / VOICE_MESH_PAYLOAD;
    }

    // ---- Outgoing session cache (for serving fetch requests) ----

    struct OutSession {
        uint32_t      sessionId;
        uint8_t       data[VOICE_C2_MAX_BYTES];
        uint32_t      dataBytes;
        uint8_t       totalPackets;
        unsigned long cachedAt;
        bool          active;
    };

    void cacheOutSession(uint32_t sessionId) {
        _outSession.sessionId = sessionId;
        memcpy(_outSession.data, _c2Data, _c2Bytes);
        _outSession.dataBytes = _c2Bytes;
        _outSession.totalPackets = (uint8_t)getPacketCount();
        _outSession.cachedAt = (unsigned long)(esp_timer_get_time() / 1000ULL);
        _outSession.active = true;
        printf("MeckVoice: session 0x%08lX cached (%lu bytes, %d pkts)\n",
               (unsigned long)sessionId, (unsigned long)_c2Bytes, _outSession.totalPackets);
    }

    bool hasValidOutSession() const {
        return _outSession.active &&
               ((unsigned long)(esp_timer_get_time() / 1000ULL) - _outSession.cachedAt
                < VOICE_SESSION_TTL_MS);
    }

    uint32_t getOutSessionId() const { return _outSession.sessionId; }
    const OutSession &getOutSession() const { return _outSession; }

    // ---- Incoming session (reassembly from received packets) ----

    struct InSession {
        uint32_t sessionId;
        uint8_t  data[VOICE_C2_MAX_BYTES];
        uint16_t pktOffset[16];
        uint16_t pktSize[16];
        uint8_t  totalPackets;
        uint16_t receivedBitmap;
        uint8_t  receivedCount;
        uint32_t dataBytes;
        char     senderName[32];
        unsigned long startedAt;
        bool     active;
        bool     complete;
    };

    // Called when a VE3 envelope DM arrives (sets up expected packet count)
    void onVE3Received(const char *senderName, const char *ve3Text) {
        uint32_t sid = 0;
        uint8_t total = 0, dur = 0;
        if (!parseVE3(ve3Text, &sid, &total, &dur)) {
            printf("MeckVoice: invalid VE3: %s\n", ve3Text);
            return;
        }

        _inSession.sessionId = sid;
        _inSession.totalPackets = total;
        _inSession.receivedBitmap = 0;
        _inSession.receivedCount = 0;
        _inSession.dataBytes = 0;
        _inSession.complete = false;
        _inSession.active = true;
        _inSession.startedAt = (unsigned long)(esp_timer_get_time() / 1000ULL);
        memset(_inSession.pktOffset, 0, sizeof(_inSession.pktOffset));
        memset(_inSession.pktSize, 0, sizeof(_inSession.pktSize));
        strncpy(_inSession.senderName, senderName, 31);
        _inSession.senderName[31] = '\0';

        printf("MeckVoice: VE3 from %s: session=0x%08lX, %d pkts, %ds\n",
               senderName, (unsigned long)sid, total, dur);
    }

    // Called when a 0x56 voice data packet arrives
    void onVoicePacketReceived(const uint8_t *payload, uint8_t len) {
        if (len <= VOICE_PKT_HDR_SIZE) return;

        uint32_t sid;
        memcpy(&sid, &payload[1], 4);
        uint8_t idx = payload[5];

        if (!_inSession.active || sid != _inSession.sessionId) {
            printf("MeckVoice: voice pkt for unknown session 0x%08lX\n", (unsigned long)sid);
            return;
        }
        if (idx >= 16 || idx >= _inSession.totalPackets) return;
        if (_inSession.receivedBitmap & (1 << idx)) return;  // duplicate

        int dataLen = len - VOICE_PKT_HDR_SIZE;
        if (_inSession.dataBytes + dataLen > VOICE_C2_MAX_BYTES) return;

        _inSession.pktOffset[idx] = (uint16_t)_inSession.dataBytes;
        _inSession.pktSize[idx] = (uint16_t)dataLen;
        memcpy(&_inSession.data[_inSession.dataBytes], &payload[6], dataLen);
        _inSession.dataBytes += dataLen;
        _inSession.receivedBitmap |= (1 << idx);
        _inSession.receivedCount++;

        printf("MeckVoice: pkt %d/%d received (%d bytes, %d/%d total)\n",
               idx + 1, _inSession.totalPackets, dataLen,
               _inSession.receivedCount, _inSession.totalPackets);

        if (_inSession.receivedCount >= _inSession.totalPackets) {
            _inSession.complete = true;
            printf("MeckVoice: session 0x%08lX complete!\n", (unsigned long)sid);
        }
    }

    bool isIncomingComplete() const {
        return _inSession.active && _inSession.complete;
    }

    const InSession &getInSession() const { return _inSession; }

    // Reassemble received packets in order for decoding
    uint32_t reassembleIncoming(uint8_t *dest, uint32_t maxLen) const {
        if (!_inSession.complete) return 0;
        uint32_t pos = 0;
        for (int p = 0; p < _inSession.totalPackets && p < 16; p++) {
            if (_inSession.pktSize[p] > 0 &&
                pos + _inSession.pktSize[p] <= maxLen) {
                memcpy(&dest[pos],
                       &_inSession.data[_inSession.pktOffset[p]],
                       _inSession.pktSize[p]);
                pos += _inSession.pktSize[p];
            }
        }
        return pos;
    }

    void clearIncoming() {
        _inSession.active = false;
        _inSession.complete = false;
    }

private:
    // Recording buffer (PSRAM)
    int16_t  *_recBuffer    = nullptr;
    uint32_t  _recSamples   = 0;
    bool      _recording    = false;
    uint32_t  _recStartMs   = 0;

    // Codec2 encoded data
    uint8_t   _c2Data[VOICE_C2_MAX_BYTES] = {};
    uint32_t  _c2Bytes  = 0;
    uint32_t  _c2Frames = 0;
    bool      _c2Valid  = false;

    // Session state
    OutSession _outSession;
    InSession  _inSession;
};
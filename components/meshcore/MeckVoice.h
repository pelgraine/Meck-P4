/*
 * MeckVoice.h -- Voice message recording, encoding, and VE3 protocol for P4
 *
 * Codec2 1200bps voice codec over MeshCore's PAYLOAD_TYPE_RAW_CUSTOM.
 * Wire-compatible with upstream Meck (T-Deck Pro) VoiceMessageScreen.h and
 * with Meck Watch (WatchVoiceScreen.h): VE3 envelope DM, 0x56 voice packets
 * of up to 150 bytes of Codec2 1200 data, at most 12 s / 12 packets.
 *
 * Threading (Nov 2026, brought in line with Meck Watch's voice notes):
 *   - LVGL task: startRecording() / requestStop(), selects notes for send,
 *     and collects finished results (takeRecordResult / takeInboxResult).
 *   - "voice_cap" task: reads the ES8311 mic into PSRAM for as long as the
 *     recording runs, so the UI never waits on the mic. Stop (button, key
 *     or the 12 s limit) ends the same loop, and every stop goes the same way.
 *   - "voice_work" task: filters, encodes, decodes and writes the WAV files
 *     (own recordings and incoming notes), off the UI and radio tasks.
 *   - meck_task: VE3 send (prepareOutSession / buildVoicePacket), fetch
 *     serving, and incoming packet reassembly (onVE3Received /
 *     onVoicePacketReceived / pollIncomingHandoff).
 *   Everything shared between those tasks is guarded by _lock.
 *
 * Audio path (Meck Watch "version B", run at the P4's rates):
 *   44.1 kHz stereo capture -> louder channel -> 6 kHz low-pass and resample
 *   to 16 kHz -> DC removal, normalise to 90% of full scale, 150 Hz high-pass,
 *   31-tap 3.6 kHz low-pass and halve to 8 kHz -> Codec2 1200.
 *   Playback: Codec2 -> 8 kHz -> same 3.6 kHz low-pass, doubled to 16 kHz ->
 *   resampled to 44.1 kHz (the I2S bus rate) -> stereo WAV on the SD card.
 *   A recording's WAV holds the decoded Codec2 audio, so Play gives what the
 *   other end will hear. Its Codec2 data is kept beside it as <name>.c2, so a
 *   previous recording is sent exactly as it was recorded.
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <atomic>
#include <sys/stat.h>
#include <unistd.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"   // xTaskCreatePinnedToCoreWithCaps

// Codec2 -- if not available yet, guard with #ifdef HAVE_CODEC2
#ifdef HAVE_CODEC2
#include <codec2.h>
#endif

// Debug Logs
#include "meck_log.h"
#include "meck.h"        // meck_audio_codec_wake/sleep, meck_clock_get_utc
#include "MeckAudio.h"   // meck_audio_stop

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
#define VOICE_MAX_PACKETS     16    // receivers track packets in a 16-bit map

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

// Capture buffer: 44100Hz stereo x 12s = 1,058,400 int16_t values.
#define VOICE_BUF_SAMPLES     (VOICE_SAMPLE_RATE * VOICE_MAX_SECONDS * 2)
#define VOICE_BUF_BYTES       (VOICE_BUF_SAMPLES * sizeof(int16_t))

// Filter chain buffers (Meck Watch rates)
#define VOICE_RATE_16K        16000
#define VOICE_16K_SAMPLES     (VOICE_RATE_16K * VOICE_MAX_SECONDS)       // 192000
#define VOICE_8K_SAMPLES      (VOICE_16K_SAMPLES / 2)                    // 96000
#define VOICE_44K_SAMPLES     (VOICE_SAMPLE_RATE * VOICE_MAX_SECONDS)    // 529200 mono
#define VOICE_HPF_HZ          150.0f
#define VOICE_LPF_HZ          3600.0f
#define VOICE_FIR_TAPS        31
#define VOICE_AA_HZ           6000.0f   // 44.1 kHz -> 16 kHz anti-alias
#define VOICE_AA_TAPS         31

// Mic start-up output dropped at the start of every recording (as the watch)
#define VOICE_SKIP_START_MS   250
// Under half a second of audio is not kept (as the watch)
#define VOICE_MIN_16K         (VOICE_RATE_16K / 2)

// SD card voice folders
#define VOICE_FOLDER          "/sdcard/voice"
#define VOICE_RX_FOLDER       "/sdcard/voice/inbox"

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
// MeckVoice
// ============================================================================

class MeckVoice {
public:
    // Recording pipeline state, as the record screen shows it.
    enum RecState : int { REC_IDLE = 0, REC_RECORDING = 1, REC_SAVING = 2 };

    // Outcome of one recording, collected by the UI with takeRecordResult().
    enum RecOutcome : int {
        REC_OK = 0,          // encoded; WAV and .c2 written
        REC_OK_NOT_SAVED,    // encoded (can be sent) but the SD write failed
        REC_TOO_SHORT,       // under half a second
        REC_MIC_FAILED,      // the mic returned no audio
        REC_ENCODE_FAILED,
        REC_PREPARED,        // a previous recording's Codec2 data is ready
        REC_PREPARE_FAILED,
    };

    struct RecordResult {
        int   outcome;
        float seconds;
        char  wavPath[112];
    };

    struct InboxResult {
        char     senderName[32];
        char     wavPath[112];
        uint32_t timestamp;
        uint8_t  durationSec;
        bool     saved;
    };

    MeckVoice() {
        memset(&_out, 0, sizeof(_out));
        memset(&_in, 0, sizeof(_in));
        memset(&_rxJob, 0, sizeof(_rxJob));
        memset(&_recResult, 0, sizeof(_recResult));
        memset(&_inboxResult, 0, sizeof(_inboxResult));
        _prepPath[0] = '\0';
    }

    // Creates the lock and the two voice tasks. Called once from
    // meck_app_start(); safe to call again (does nothing the second time).
    void begin() {
        if (_lock) return;
        _lock = xSemaphoreCreateMutex();
        firDesign(_fir, VOICE_FIR_TAPS, VOICE_LPF_HZ / VOICE_RATE_16K);
        firDesign(_aa, VOICE_AA_TAPS, VOICE_AA_HZ / VOICE_SAMPLE_RATE);
        // Capture: mostly blocked in the I2S read, so a high priority costs
        // nothing and keeps the DMA drained. Work: below the UI and radio.
        // Both stacks are in PSRAM, as MeckGBC's task: plain xTaskCreate
        // takes the stack from internal RAM, which has no 32 KB block left
        // in the full firmware (the work task failed to start on hardware).
        // Neither task does flash operations (the restriction on PSRAM
        // stacks); the SD driver copies non-DMA buffers itself.
        if (xTaskCreatePinnedToCoreWithCaps(captureTaskEntry, "voice_cap", 6 * 1024, this, 4,
                                            &_capTask, tskNO_AFFINITY, MALLOC_CAP_SPIRAM) != pdPASS) {
            _capTask = nullptr;
        }
        if (xTaskCreatePinnedToCoreWithCaps(workTaskEntry, "voice_work", 32 * 1024, this, 1,
                                            &_workTask, tskNO_AFFINITY, MALLOC_CAP_SPIRAM) != pdPASS) {
            _workTask = nullptr;
        }
        printf("MeckVoice: tasks started (cap=%p work=%p)%s\n",
               (void *)_capTask, (void *)_workTask,
               tasksReady() ? "" : " -- TASK CREATE FAILED, voice notes off");
    }

    // Both voice tasks are running (recording, saving and receiving need them).
    bool tasksReady() const { return _capTask != nullptr && _workTask != nullptr; }

    // ========================================================================
    // Recording (LVGL task)
    // ========================================================================

    RecState recState() const { return (RecState)_recState.load(); }
    bool isRecording() const { return _recState.load() == REC_RECORDING; }
    bool isSaving() const    { return _recState.load() == REC_SAVING; }

    // Seconds of audio captured so far (during recording).
    float recordedSeconds() const {
        return _capValues.load() / (float)(VOICE_SAMPLE_RATE * 2);
    }

    // Starts a recording. Returns false (nothing started) if a recording is
    // already running or being saved, or the mic could not be started.
    bool startRecording() {
        if (!_lock) begin();
        if (!tasksReady()) return false;
        if (!ensureCaptureBuffer()) return false;

        // The buffer is also used to save incoming notes: only take it when
        // the work task is not using it (isBusy()).
        xSemaphoreTake(_lock, portMAX_DELAY);
        bool free = (_recState.load() == REC_IDLE) && !_bufBusy.load();
        if (free) _recState = REC_RECORDING;
        xSemaphoreGive(_lock);
        if (!free) return false;

        // Stop any active playback before capturing; that also puts the
        // codec to sleep, with both I2S channels disabled.
        meck_audio_stop();
        // Wake the codec: re-enables the I2S channels (the receive channel
        // must run for the mic read to return audio).
        meck_audio_codec_wake();
        if (!meck_audio_mic_start(VOICE_SAMPLE_RATE)) {
            printf("MeckVoice: mic start failed\n");
            meck_audio_codec_sleep();
            _recState = REC_IDLE;
            xTaskNotifyGive(_workTask);   // anything that waited for the buffer
            return false;
        }

        _capValues = 0;
        _stopReq = false;
        xTaskNotifyGive(_capTask);
        printf("MeckVoice: recording started\n");
        return true;
    }

    // The work task is saving an incoming note (or preparing an older
    // recording) in the shared buffer: a recording cannot start just now.
    bool isBusy() const { return _bufBusy.load(); }

    // Stops the recording (Stop button, keyboard). The capture task ends its
    // loop within one read and hands the audio to the work task, exactly as
    // when it stops by itself at 12 s.
    void requestStop() {
        if (_recState.load() == REC_RECORDING) _stopReq = true;
    }

    // Finished recording (or prepared previous recording). True once per result.
    bool takeRecordResult(RecordResult &out) {
        if (!_lock) return false;
        bool got = false;
        xSemaphoreTake(_lock, portMAX_DELAY);
        if (_recResultReady) {
            out = _recResult;
            _recResultReady = false;
            got = true;
        }
        xSemaphoreGive(_lock);
        return got;
    }

    // ========================================================================
    // Choosing what Send sends (LVGL task)
    // ========================================================================

    // Codec2 data of the selected note is ready to send.
    bool isCodec2Valid() const { return _c2Valid.load(); }

    int getPacketCount() const {
        uint32_t b = _c2Bytes.load();
        if (!_c2Valid.load() || b == 0) return 0;
        return (b + VOICE_MESH_PAYLOAD - 1) / VOICE_MESH_PAYLOAD;
    }

    // Makes the recording at wavPath the one Send sends. Reads its .c2 file
    // when it has one (returns true: ready now). A recording made before .c2
    // files existed is encoded from its WAV on the work task (returns false;
    // takeRecordResult() later gives REC_PREPARED or REC_PREPARE_FAILED).
    bool selectRecording(const char *wavPath) {
        if (!_lock) begin();
        uint8_t buf[VOICE_C2_MAX_BYTES];
        char c2Path[120];
        sidecarPath(wavPath, c2Path, sizeof(c2Path));
        uint32_t n = 0;
        FILE *f = fopen(c2Path, "rb");
        if (f) {
            n = (uint32_t)fread(buf, 1, sizeof(buf), f);
            fclose(f);
        }
        n -= n % VOICE_C2_FRAME_BYTES;
        if (n > 0) {
            setCodec2(buf, n);
            printf("MeckVoice: selected %s (%lu Codec2 bytes)\n",
                   wavPath, (unsigned long)n);
            return true;
        }
        // No .c2: clear the current data so a Send now cannot send a
        // different note, then encode this one from its WAV.
        xSemaphoreTake(_lock, portMAX_DELAY);
        _c2Valid = false;
        _c2Bytes = 0;
        strncpy(_prepPath, wavPath, sizeof(_prepPath) - 1);
        _prepPath[sizeof(_prepPath) - 1] = '\0';
        _jobPrepare = (_workTask != nullptr);
        xSemaphoreGive(_lock);
        if (!_workTask) return false;
        xTaskNotifyGive(_workTask);
        printf("MeckVoice: %s has no .c2, encoding it from the WAV\n", wavPath);
        return false;
    }

    // The note was deleted: nothing to send.
    void clearSelection() {
        if (!_lock) return;
        xSemaphoreTake(_lock, portMAX_DELAY);
        _c2Valid = false;
        _c2Bytes = 0;
        xSemaphoreGive(_lock);
    }

    // The <name>.c2 file kept beside a recording's <name>.wav.
    static void sidecarPath(const char *wavPath, char *out, size_t outLen) {
        snprintf(out, outLen, "%s", wavPath);
        char *dot = strrchr(out, '.');
        if (dot && (size_t)(dot - out) + 4 <= outLen) strcpy(dot, ".c2");
    }

    // ========================================================================
    // Send (meck_task)
    // ========================================================================

    // Copies the selected note into the outgoing session (kept 15 minutes
    // for fetch requests). Returns the packet count, 0 if nothing to send.
    int prepareOutSession(uint32_t sessionId) {
        if (!_lock) return 0;
        int total = 0;
        xSemaphoreTake(_lock, portMAX_DELAY);
        uint32_t b = _c2Bytes.load();
        if (_c2Valid.load() && b > 0) {
            _out.sessionId = sessionId;
            memcpy(_out.data, _c2, b);
            _out.dataBytes = b;
            _out.totalPackets = (uint8_t)((b + VOICE_MESH_PAYLOAD - 1) / VOICE_MESH_PAYLOAD);
            _out.cachedAt = nowMs();
            _out.active = true;
            total = _out.totalPackets;
        }
        xSemaphoreGive(_lock);
        if (total > 0) {
            printf("MeckVoice: session 0x%08lX cached (%lu bytes, %d pkts)\n",
                   (unsigned long)sessionId, (unsigned long)b, total);
        }
        return total;
    }

    // VE3:{sid}:{mode}:{total}:{durS} for the outgoing session.
    void formatEnvelope(char *buf, int bufLen, uint32_t sessionId) {
        uint32_t bytes = 0;
        if (_lock) {
            xSemaphoreTake(_lock, portMAX_DELAY);
            bytes = (_out.active && _out.sessionId == sessionId) ? _out.dataBytes : 0;
            xSemaphoreGive(_lock);
        }
        char sid[12], mode[4], total[4], dur[4];
        int totalPackets = (bytes + VOICE_MESH_PAYLOAD - 1) / VOICE_MESH_PAYLOAD;
        int durationSec = (int)((bytes / VOICE_C2_FRAME_BYTES) * VOICE_C2_FRAME_MS / 1000);
        meck_voice_to_base36(sessionId, sid, sizeof(sid));
        meck_voice_to_base36(VOICE_C2_MODE_ID, mode, sizeof(mode));
        meck_voice_to_base36(totalPackets, total, sizeof(total));
        meck_voice_to_base36(durationSec, dur, sizeof(dur));
        snprintf(buf, bufLen, "VE3:%s:%s:%s:%s", sid, mode, total, dur);
    }

    // Build a voice data packet: [0x56][sessionId:4B][index:1B][codec2 data...]
    // from the outgoing session.
    int buildVoicePacket(uint8_t *buf, int bufLen, uint32_t sessionId, int pktIndex) {
        if (!_lock || pktIndex < 0) return 0;
        int totalLen = 0;
        xSemaphoreTake(_lock, portMAX_DELAY);
        if (_out.active && _out.sessionId == sessionId) {
            uint32_t offset = (uint32_t)pktIndex * VOICE_MESH_PAYLOAD;
            if (offset < _out.dataBytes) {
                uint32_t dataLen = _out.dataBytes - offset;
                if (dataLen > VOICE_MESH_PAYLOAD) dataLen = VOICE_MESH_PAYLOAD;
                if ((int)(VOICE_PKT_HDR_SIZE + dataLen) <= bufLen) {
                    buf[0] = VOICE_PKT_MAGIC;
                    memcpy(&buf[1], &sessionId, 4);
                    buf[5] = (uint8_t)pktIndex;
                    memcpy(&buf[6], &_out.data[offset], dataLen);
                    totalLen = VOICE_PKT_HDR_SIZE + dataLen;
                }
            }
        }
        xSemaphoreGive(_lock);
        return totalLen;
    }

    int outPacketCount() {
        if (!_lock) return 0;
        xSemaphoreTake(_lock, portMAX_DELAY);
        int n = _out.active ? _out.totalPackets : 0;
        xSemaphoreGive(_lock);
        return n;
    }

    // An outgoing session with this ID is cached and under 15 minutes old.
    bool hasOutSession(uint32_t sessionId) {
        if (!_lock) return false;
        xSemaphoreTake(_lock, portMAX_DELAY);
        bool ok = _out.active && _out.sessionId == sessionId &&
                  (nowMs() - _out.cachedAt) < VOICE_SESSION_TTL_MS;
        xSemaphoreGive(_lock);
        return ok;
    }

    // ========================================================================
    // Receive (meck_task). _in is only touched on meck_task.
    // ========================================================================

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

    // Called when a VE3 envelope DM arrives (sets up expected packet count)
    void onVE3Received(const char *senderName, const char *ve3Text) {
        uint32_t sid = 0;
        uint8_t total = 0, dur = 0;
        if (!parseVE3(ve3Text, &sid, &total, &dur)) {
            printf("MeckVoice: invalid VE3: %s\n", ve3Text);
            return;
        }
        if (total == 0 || total > VOICE_MAX_PACKETS) {
            printf("MeckVoice: VE3 invalid packet count %d\n", total);
            return;
        }
        // The sender retries the envelope until it is acknowledged: a repeat
        // of the session already being received keeps the packets so far.
        if (_in.active && _in.sessionId == sid) {
            printf("MeckVoice: VE3 repeat for session 0x%08lX\n", (unsigned long)sid);
            return;
        }

        memset(&_in, 0, sizeof(_in));
        _in.sessionId = sid;
        _in.totalPackets = total;
        _in.active = true;
        _in.startedAt = nowMs();
        strncpy(_in.senderName, senderName, 31);
        _in.senderName[31] = '\0';

        printf("MeckVoice: VE3 from %s: session=0x%08lX, %d pkts, %ds\n",
               senderName, (unsigned long)sid, total, dur);
    }

    // Called when a 0x56 voice data packet arrives
    void onVoicePacketReceived(const uint8_t *payload, uint8_t len) {
        if (len <= VOICE_PKT_HDR_SIZE) return;

        uint32_t sid;
        memcpy(&sid, &payload[1], 4);
        uint8_t idx = payload[5];

        if (!_in.active || sid != _in.sessionId) {
            printf("MeckVoice: voice pkt for unknown session 0x%08lX\n", (unsigned long)sid);
            return;
        }
        if (idx >= VOICE_MAX_PACKETS || idx >= _in.totalPackets) return;
        if (_in.receivedBitmap & (1 << idx)) return;  // duplicate

        int dataLen = len - VOICE_PKT_HDR_SIZE;
        if (_in.dataBytes + dataLen > VOICE_C2_MAX_BYTES) return;

        _in.pktOffset[idx] = (uint16_t)_in.dataBytes;
        _in.pktSize[idx] = (uint16_t)dataLen;
        memcpy(&_in.data[_in.dataBytes], &payload[6], dataLen);
        _in.dataBytes += dataLen;
        _in.receivedBitmap |= (1 << idx);
        _in.receivedCount++;

        printf("MeckVoice: pkt %d/%d received (%d bytes, %d/%d total)\n",
               idx + 1, _in.totalPackets, dataLen,
               _in.receivedCount, _in.totalPackets);

        if (_in.receivedCount >= _in.totalPackets) {
            _in.complete = true;
            printf("MeckVoice: session 0x%08lX complete!\n", (unsigned long)sid);
        }
    }

    // Called every meck_task pass: a complete note is reassembled in packet
    // order and handed to the work task, which decodes it and saves it to the
    // inbox. Waits (keeps the note) while the work task still holds one.
    void pollIncomingHandoff() {
        if (!_lock || !_workTask || !_in.active || !_in.complete) return;
        xSemaphoreTake(_lock, portMAX_DELAY);
        bool handed = false;
        if (!_rxJob.pending) {
            uint32_t pos = 0;
            for (int p = 0; p < _in.totalPackets && p < VOICE_MAX_PACKETS; p++) {
                if (_in.pktSize[p] > 0 && pos + _in.pktSize[p] <= VOICE_C2_MAX_BYTES) {
                    memcpy(&_rxJob.data[pos], &_in.data[_in.pktOffset[p]], _in.pktSize[p]);
                    pos += _in.pktSize[p];
                }
            }
            _rxJob.dataBytes = pos;
            strncpy(_rxJob.senderName, _in.senderName, 31);
            _rxJob.senderName[31] = '\0';
            _rxJob.pending = true;
            handed = true;
        }
        xSemaphoreGive(_lock);
        if (handed) {
            _in.active = false;
            _in.complete = false;
            xTaskNotifyGive(_workTask);
        }
    }

    // A received note decoded and saved (LVGL task). True once per note.
    bool takeInboxResult(InboxResult &out) {
        if (!_lock) return false;
        bool got = false;
        xSemaphoreTake(_lock, portMAX_DELAY);
        if (_inboxResultReady) {
            out = _inboxResult;
            _inboxResultReady = false;
            got = true;
        }
        xSemaphoreGive(_lock);
        return got;
    }

private:
    struct OutSession {
        uint32_t      sessionId;
        uint8_t       data[VOICE_C2_MAX_BYTES];
        uint32_t      dataBytes;
        uint8_t       totalPackets;
        uint32_t      cachedAt;
        bool          active;
    };

    struct InSession {
        uint32_t sessionId;
        uint8_t  data[VOICE_C2_MAX_BYTES];
        uint16_t pktOffset[VOICE_MAX_PACKETS];
        uint16_t pktSize[VOICE_MAX_PACKETS];
        uint8_t  totalPackets;
        uint16_t receivedBitmap;
        uint8_t  receivedCount;
        uint32_t dataBytes;
        char     senderName[32];
        uint32_t startedAt;
        bool     active;
        bool     complete;
    };

    // A complete incoming note waiting for the work task.
    struct RxJob {
        uint8_t  data[VOICE_C2_MAX_BYTES];
        uint32_t dataBytes;
        char     senderName[32];
        bool     pending;
    };

    static uint32_t nowMs() { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

    void setCodec2(const uint8_t *data, uint32_t len) {
        xSemaphoreTake(_lock, portMAX_DELAY);
        memcpy(_c2, data, len);
        _c2Bytes = len;
        _c2Valid = (len > 0);
        xSemaphoreGive(_lock);
    }

    // ---- The one PSRAM buffer (allocated on first use, kept) ----
    // 2.1 MB: the stereo capture, 44.1 kHz x 2 x 12 s. Once a recording has
    // been reduced to mono, only its first half holds audio, so the filter
    // chain works in the second half: _pcm16 (16 kHz) and _pcm8k (8 kHz)
    // point there. Saving incoming notes uses the same two areas; _recState
    // and _bufBusy (under _lock) keep the capture task and the work task
    // from using the buffer at the same time.
    static_assert(VOICE_44K_SAMPLES + VOICE_16K_SAMPLES + VOICE_8K_SAMPLES <= VOICE_BUF_SAMPLES,
                  "filter areas must fit after the mono audio in the capture buffer");

    bool ensureCaptureBuffer() {
        xSemaphoreTake(_lock, portMAX_DELAY);
        if (!_recBuffer) {
            _recBuffer = (int16_t *)heap_caps_calloc(
                VOICE_BUF_SAMPLES, sizeof(int16_t), MALLOC_CAP_SPIRAM);
            if (_recBuffer) {
                _pcm16 = _recBuffer + VOICE_44K_SAMPLES;
                _pcm8k = _pcm16 + VOICE_16K_SAMPLES;
            } else {
                printf("MeckVoice: PSRAM alloc failed for voice buffer (%d bytes; "
                       "PSRAM free %u, largest block %u)\n",
                       (int)VOICE_BUF_BYTES,
                       (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                       (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
            }
        }
        bool ok = (_recBuffer != nullptr);
        xSemaphoreGive(_lock);
        return ok;
    }

    // Work task: take the buffer for an incoming note or a prepare, unless a
    // recording has it. releaseForWork() hands it back.
    bool claimForWork() {
        xSemaphoreTake(_lock, portMAX_DELAY);
        bool ok = (_recState.load() == REC_IDLE) && !_bufBusy.load();
        if (ok) _bufBusy = true;
        xSemaphoreGive(_lock);
        return ok;
    }
    void releaseForWork() { _bufBusy = false; }

    // ========================================================================
    // Capture task
    // ========================================================================

    static void captureTaskEntry(void *arg) { ((MeckVoice *)arg)->captureTask(); }

    void captureTask() {
        for (;;) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            if (_recState.load() != REC_RECORDING) continue;

            const int64_t skipUntil = esp_timer_get_time() + VOICE_SKIP_START_MS * 1000LL;
            uint32_t n = 0;           // int16 values captured (stereo)
            int zeroReads = 0;
            bool micFailed = false;
            while (!_stopReq.load()) {
                size_t want = 4096;
                if (n * sizeof(int16_t) + want > VOICE_BUF_BYTES)
                    want = (VOICE_BUF_SAMPLES - n) * sizeof(int16_t);
                if (want == 0) break;                      // 12 s: buffer full
                size_t got = meck_audio_mic_read(&_recBuffer[n], want);
                if (got == 0) {
                    // Each failed read has already waited up to 1 s. With
                    // audio already captured (something else stopped the
                    // codec mid-recording), keep it as an ordinary stop.
                    if (++zeroReads >= 3) { micFailed = (n == 0); break; }
                    continue;
                }
                zeroReads = 0;
                if (esp_timer_get_time() < skipUntil) continue;   // start-up output
                n += got / sizeof(int16_t);
                _capValues = n;
            }
            meck_audio_mic_stop();
            meck_audio_codec_sleep();
            printf("MeckVoice: recording stopped, %lu values (%.1fs)%s\n",
                   (unsigned long)n, n / (float)(VOICE_SAMPLE_RATE * 2),
                   micFailed ? ", mic returned no audio" : "");

            if (micFailed || n / 2 < (uint32_t)(VOICE_SAMPLE_RATE / 2)) {
                RecordResult r = {};
                r.outcome = micFailed ? REC_MIC_FAILED : REC_TOO_SHORT;
                r.seconds = n / (float)(VOICE_SAMPLE_RATE * 2);
                postRecordResult(r);
                _recState = REC_IDLE;
                xTaskNotifyGive(_workTask);   // anything that waited for the buffer
                continue;
            }
            _capFinal = n;
            _recState = REC_SAVING;
            _jobRecord = true;
            xTaskNotifyGive(_workTask);
        }
    }

    // ========================================================================
    // Work task
    // ========================================================================

    static void workTaskEntry(void *arg) { ((MeckVoice *)arg)->workTask(); }

    void workTask() {
        for (;;) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            bool again = true;
            while (again) {
                again = false;
                if (_jobRecord.load()) {
                    _jobRecord = false;
                    doRecording();
                    again = true;
                }
                bool rx = false, prep = false;
                xSemaphoreTake(_lock, portMAX_DELAY);
                rx = _rxJob.pending;
                prep = _jobPrepare;
                xSemaphoreGive(_lock);
                // While a recording holds the buffer these wait; the
                // recording's end wakes this task again.
                if (rx && claimForWork())   { doIncoming(); releaseForWork(); again = true; }
                if (prep && claimForWork()) { doPrepare();  releaseForWork(); again = true; }
            }
        }
    }

    void postRecordResult(const RecordResult &r) {
        xSemaphoreTake(_lock, portMAX_DELAY);
        _recResult = r;
        _recResultReady = true;
        xSemaphoreGive(_lock);
    }

    static void yieldNow() { vTaskDelay(1); }

    // Own recording: _recBuffer (stereo 44.1k, _capFinal values) -> encode,
    // decoded WAV + .c2 on the SD card, selected for Send.
    void doRecording() {
        RecordResult r = {};
        uint32_t nStereo = _capFinal;
        uint8_t c2[VOICE_C2_MAX_BYTES];
        uint32_t c2Bytes = 0;

        uint32_t n44 = pickLouderChannel(_recBuffer, nStereo);
        uint32_t n16 = resample44to16(_recBuffer, n44, _pcm16);
        c2Bytes = encode16k(n16, c2);
        if (c2Bytes == 0) {
            r.outcome = REC_ENCODE_FAILED;
            postRecordResult(r);
            _recState = REC_IDLE;
            return;
        }
        r.seconds = (c2Bytes / VOICE_C2_FRAME_BYTES) * VOICE_C2_FRAME_MS / 1000.0f;
        setCodec2(c2, c2Bytes);

        char path[112];
        bool saved = makeName(VOICE_FOLDER, "voice_", path, sizeof(path)) &&
                     writeDecodedWav(c2, c2Bytes, path) &&
                     writeSidecar(path, c2, c2Bytes);
        r.outcome = saved ? REC_OK : REC_OK_NOT_SAVED;
        if (saved) {
            strncpy(r.wavPath, path, sizeof(r.wavPath) - 1);
        }
        printf("MeckVoice: recording encoded, %lu bytes (%.1fs, %d pkts), %s %s\n",
               (unsigned long)c2Bytes, r.seconds,
               (int)((c2Bytes + VOICE_MESH_PAYLOAD - 1) / VOICE_MESH_PAYLOAD),
               saved ? "saved" : "NOT saved", saved ? path : "");
        postRecordResult(r);
        _recState = REC_IDLE;
    }

    // Incoming note: decode and save as a WAV in the inbox folder.
    void doIncoming() {
        RxJob job;
        xSemaphoreTake(_lock, portMAX_DELAY);
        memcpy(&job, &_rxJob, sizeof(job));
        _rxJob.pending = false;
        xSemaphoreGive(_lock);

        InboxResult r = {};
        strncpy(r.senderName, job.senderName, sizeof(r.senderName) - 1);
        uint32_t ts = meck_clock_get_utc();
        r.timestamp = ts;
        r.durationSec = (uint8_t)((job.dataBytes / VOICE_C2_FRAME_BYTES) * VOICE_C2_FRAME_MS / 1000);

        bool saved = false;
        if (ensureCaptureBuffer()) {
            struct stat st;
            if (stat(VOICE_FOLDER, &st) != 0) mkdir(VOICE_FOLDER, 0755);
            if (stat(VOICE_RX_FOLDER, &st) != 0) mkdir(VOICE_RX_FOLDER, 0755);
            saved = makeName(VOICE_RX_FOLDER, "voice_rx_", r.wavPath, sizeof(r.wavPath)) &&
                    writeDecodedWav(job.data, job.dataBytes, r.wavPath);
        }
        r.saved = saved;
        printf("MeckVoice: incoming note from %s, %lu bytes (%ds): %s %s\n",
               r.senderName, (unsigned long)job.dataBytes, r.durationSec,
               saved ? "saved" : "NOT saved", saved ? r.wavPath : "");
        if (!saved) r.wavPath[0] = '\0';
        xSemaphoreTake(_lock, portMAX_DELAY);
        _inboxResult = r;
        _inboxResultReady = true;
        xSemaphoreGive(_lock);
    }

    // Previous recording without a .c2 file: encode it from its WAV, loaded
    // into the first half of the buffer (claimed: no recording is running).
    void doPrepare() {
        char path[112];
        xSemaphoreTake(_lock, portMAX_DELAY);
        strncpy(path, _prepPath, sizeof(path) - 1);
        path[sizeof(path) - 1] = '\0';
        _jobPrepare = false;
        xSemaphoreGive(_lock);

        RecordResult r = {};
        r.outcome = REC_PREPARE_FAILED;
        strncpy(r.wavPath, path, sizeof(r.wavPath) - 1);
        uint8_t c2[VOICE_C2_MAX_BYTES];
        uint32_t c2Bytes = 0;
        if (ensureCaptureBuffer()) {
            uint32_t n44 = loadWavMono(path, _recBuffer, VOICE_44K_SAMPLES);
            if (n44 > 0) {
                uint32_t n16 = resample44to16(_recBuffer, n44, _pcm16);
                c2Bytes = encode16k(n16, c2);
            }
        }
        if (c2Bytes > 0) {
            // Only if the user has not picked something else meanwhile
            xSemaphoreTake(_lock, portMAX_DELAY);
            bool still = (strcmp(_prepPath, path) == 0) && !_jobPrepare;
            if (still) {
                memcpy(_c2, c2, c2Bytes);
                _c2Bytes = c2Bytes;
                _c2Valid = true;
            }
            xSemaphoreGive(_lock);
            writeSidecar(path, c2, c2Bytes);
            r.outcome = REC_PREPARED;
            r.seconds = (c2Bytes / VOICE_C2_FRAME_BYTES) * VOICE_C2_FRAME_MS / 1000.0f;
        }
        printf("MeckVoice: prepared %s: %lu Codec2 bytes\n", path, (unsigned long)c2Bytes);
        postRecordResult(r);
    }

    // ========================================================================
    // DSP (filters as Meck Watch version B)
    // ========================================================================

    static inline int16_t clip16(float v) {
        if (v > 32767.0f) return 32767;
        if (v < -32768.0f) return -32768;
        return (int16_t)v;
    }

    // Hamming-windowed sinc low-pass, normalised to unity gain at DC.
    // fc = cutoff / sample rate.
    static void firDesign(float *h, int taps, float fc) {
        const int M = (taps - 1) / 2;
        float sum = 0;
        for (int n = 0; n < taps; n++) {
            int k = n - M;
            float sinc = (k == 0) ? 2.0f * fc
                                  : sinf(2.0f * (float)M_PI * fc * k) / ((float)M_PI * k);
            float win = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * n / (taps - 1));
            h[n] = sinc * win;
            sum += h[n];
        }
        for (int n = 0; n < taps; n++) h[n] /= sum;
    }

    // Stereo interleaved -> mono in place, keeping whichever channel holds
    // the mic (the louder one; which channel the ES8311 uses is not known).
    // Returns the mono sample count.
    static uint32_t pickLouderChannel(int16_t *buf, uint32_t nStereo) {
        uint32_t frames = nStereo / 2;
        uint64_t sumL = 0, sumR = 0;
        for (uint32_t i = 0; i < frames; i++) {
            int32_t l = buf[2 * i], r = buf[2 * i + 1];
            sumL += (uint32_t)(l < 0 ? -l : l);
            sumR += (uint32_t)(r < 0 ? -r : r);
        }
        int ch = (sumR > sumL) ? 1 : 0;
        for (uint32_t i = 0; i < frames; i++) buf[i] = buf[2 * i + ch];
        printf("MeckVoice: mic on %s channel (mean |L| %lu, |R| %lu)\n",
               ch ? "right" : "left",
               (unsigned long)(frames ? sumL / frames : 0),
               (unsigned long)(frames ? sumR / frames : 0));
        return frames;
    }

    // 44.1 kHz mono -> 16 kHz: 6 kHz anti-alias low-pass, evaluated at the
    // two input samples either side of each output position, then linear
    // interpolation between them. Returns the 16 kHz sample count.
    uint32_t resample44to16(const int16_t *in, uint32_t n44, int16_t *out16) {
        const int M = (VOICE_AA_TAPS - 1) / 2;
        uint32_t n16 = (uint32_t)((uint64_t)n44 * VOICE_RATE_16K / VOICE_SAMPLE_RATE);
        if (n16 > VOICE_16K_SAMPLES) n16 = VOICE_16K_SAMPLES;
        const float step = (float)VOICE_SAMPLE_RATE / VOICE_RATE_16K;   // 2.75625
        for (uint32_t k = 0; k < n16; k++) {
            float t = k * step;
            int32_t i = (int32_t)t;
            float f = t - i;
            float y[2];
            for (int s = 0; s < 2; s++) {
                int32_t c = i + s;
                float acc = 0;
                for (int tp = 0; tp < VOICE_AA_TAPS; tp++) {
                    int32_t idx = c + M - tp;
                    if (idx >= 0 && idx < (int32_t)n44) acc += _aa[tp] * in[idx];
                }
                y[s] = acc;
            }
            out16[k] = clip16((1.0f - f) * y[0] + f * y[1]);
            if ((k & 0x3FFF) == 0x3FFF) yieldNow();
        }
        return n16;
    }

    // _pcm16 (n16 samples) -> DC removal + normalise (in place), high-pass
    // (in place), then low-pass + decimate into _pcm8k, then Codec2 1200 into
    // c2. Identical to the watch's prepareForEncode() + encodeStep().
    // Returns the Codec2 byte count (0 on failure).
    uint32_t encode16k(uint32_t n, uint8_t *c2) {
#ifdef HAVE_CODEC2
        if (n < VOICE_MIN_16K) return 0;
        int64_t sum = 0;
        for (uint32_t i = 0; i < n; i++) sum += _pcm16[i];
        int32_t dc = (int32_t)(sum / (int64_t)n);
        int32_t peak = 0;
        for (uint32_t i = 0; i < n; i++) {
            int32_t v = _pcm16[i] - dc;
            if (v > 32767) v = 32767;
            if (v < -32768) v = -32768;
            _pcm16[i] = (int16_t)v;
            int32_t a = v < 0 ? -v : v;
            if (a > peak) peak = a;
        }
        if (peak >= 100) {
            int32_t gain16 = (29491 << 16) / peak;   // 90% of full scale
            for (uint32_t i = 0; i < n; i++) {
                int32_t v = ((int32_t)_pcm16[i] * gain16) >> 16;
                if (v > 32767) v = 32767;
                if (v < -32768) v = -32768;
                _pcm16[i] = (int16_t)v;
            }
        }
        printf("MeckVoice: %lu samples at 16 kHz, dc %ld, peak %ld\n",
               (unsigned long)n, (long)dc, (long)peak);

        // 2nd-order Butterworth high-pass (RBJ biquad)
        const float w0 = 2.0f * (float)M_PI * VOICE_HPF_HZ / VOICE_RATE_16K;
        const float alpha = sinf(w0) / (2.0f * 0.7071f);
        const float cw = cosf(w0);
        const float a0 = 1.0f + alpha;
        const float b0 = ((1.0f + cw) / 2.0f) / a0, b1 = (-(1.0f + cw)) / a0, b2 = b0;
        const float a1 = (-2.0f * cw) / a0, a2 = (1.0f - alpha) / a0;
        float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
        for (uint32_t i = 0; i < n; i++) {
            float x = _pcm16[i];
            float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            x2 = x1; x1 = x; y2 = y1; y1 = y;
            _pcm16[i] = clip16(y);
        }

        const int M = (VOICE_FIR_TAPS - 1) / 2;
        uint32_t n8 = n / 2;
        for (uint32_t k = 0; k < n8; k++) {
            int32_t centre = (int32_t)(k * 2);
            float acc = 0;
            for (int t = 0; t < VOICE_FIR_TAPS; t++) {
                int32_t idx = centre + M - t;
                if (idx >= 0 && idx < (int32_t)n) acc += _fir[t] * _pcm16[idx];
            }
            _pcm8k[k] = clip16(acc);
        }
        yieldNow();

        uint32_t frames = n8 / VOICE_C2_FRAME_SAM;
        struct CODEC2 *codec = codec2_create(CODEC2_MODE_1200);
        if (!codec) {
            printf("MeckVoice: codec2_create failed\n");
            return 0;
        }
        uint32_t bytes = 0;
        for (uint32_t fr = 0; fr < frames && bytes + VOICE_C2_FRAME_BYTES <= VOICE_C2_MAX_BYTES; fr++) {
            codec2_encode(codec, &c2[bytes], &_pcm8k[fr * VOICE_C2_FRAME_SAM]);
            bytes += VOICE_C2_FRAME_BYTES;
            if ((fr % 10) == 9) yieldNow();
        }
        codec2_destroy(codec);
        return bytes;
#else
        (void)n; (void)c2;
        return 0;
#endif
    }

    // Codec2 -> _pcm8k -> _pcm16 (zero-stuff + the 3.6 kHz low-pass, x2, as
    // the watch's upsampleForPlay). Returns the 16 kHz sample count; the
    // WAV writer resamples it to 44.1 kHz as it writes.
    uint32_t decodeTo16k(const uint8_t *c2, uint32_t c2Bytes) {
#ifdef HAVE_CODEC2
        struct CODEC2 *codec = codec2_create(CODEC2_MODE_1200);
        if (!codec) return 0;
        uint32_t frames = c2Bytes / VOICE_C2_FRAME_BYTES;
        uint32_t n8 = 0;
        for (uint32_t fr = 0; fr < frames && n8 + VOICE_C2_FRAME_SAM <= VOICE_8K_SAMPLES; fr++) {
            codec2_decode(codec, &_pcm8k[n8], &c2[fr * VOICE_C2_FRAME_BYTES]);
            n8 += VOICE_C2_FRAME_SAM;
            if ((fr % 10) == 9) yieldNow();
        }
        codec2_destroy(codec);

        const int M = (VOICE_FIR_TAPS - 1) / 2;
        uint32_t n16 = n8 * 2;
        for (uint32_t m = 0; m < n16; m++) {
            float acc = 0;
            for (int t = 0; t < VOICE_FIR_TAPS; t++) {
                int32_t idx = (int32_t)m + M - t;
                if (idx >= 0 && idx < (int32_t)n16 && (idx & 1) == 0) acc += _fir[t] * _pcm8k[idx / 2];
            }
            _pcm16[m] = clip16(2.0f * acc);
        }
        yieldNow();
        return n16;
#else
        (void)c2; (void)c2Bytes;
        return 0;
#endif
    }

    // ========================================================================
    // Files
    // ========================================================================

    // <folder>/<prefix><UTC seconds>.wav, or <prefix>u<uptime s>.wav with no
    // clock; -1..-9 added if the name is taken.
    static bool makeName(const char *folder, const char *prefix, char *out, size_t outLen) {
        struct stat st;
        if (stat(VOICE_FOLDER, &st) != 0) mkdir(VOICE_FOLDER, 0755);
        if (stat(folder, &st) != 0) mkdir(folder, 0755);
        char base[48];
        uint32_t utc = meck_clock_get_utc();
        if (utc >= 1700000000UL) {
            snprintf(base, sizeof(base), "%s%010lu", prefix, (unsigned long)utc);
        } else {
            snprintf(base, sizeof(base), "%su%07lu", prefix,
                     (unsigned long)((esp_timer_get_time() / 1000000ULL) % 10000000ULL));
        }
        snprintf(out, outLen, "%s/%s.wav", folder, base);
        for (int k = 1; k <= 9 && stat(out, &st) == 0; k++) {
            snprintf(out, outLen, "%s/%s-%d.wav", folder, base, k);
        }
        return stat(out, &st) != 0;
    }

    // Decodes c2 and writes it as a 44.1 kHz stereo WAV (the I2S bus is
    // stereo-only, so each sample goes to both channels). The 16 kHz audio is
    // resampled (linear) a chunk at a time as it is written, so no 44.1 kHz
    // copy of the note is held in memory.
    bool writeDecodedWav(const uint8_t *c2, uint32_t c2Bytes, const char *path) {
        uint32_t n16 = decodeTo16k(c2, c2Bytes);
        if (n16 == 0) return false;
        uint32_t n44 = (uint32_t)((uint64_t)n16 * VOICE_SAMPLE_RATE / VOICE_RATE_16K);
        FILE *f = fopen(path, "wb");
        if (!f) {
            printf("MeckVoice: fopen(%s) failed\n", path);
            return false;
        }
        uint32_t dataBytes = n44 * sizeof(int16_t) * 2;
        meck_voice_write_wav_header(f, dataBytes, VOICE_SAMPLE_RATE, VOICE_BITS, 2);
        const float step = (float)VOICE_RATE_16K / VOICE_SAMPLE_RATE;
        int16_t chunk[1024];
        uint32_t m = 0;
        bool ok = true;
        while (m < n44 && ok) {
            int ci = 0;
            while (ci < 1024 && m < n44) {
                float s = m * step;
                uint32_t j = (uint32_t)s;
                float fr = s - j;
                float a = (j < n16) ? _pcm16[j] : 0.0f;
                float b = (j + 1 < n16) ? _pcm16[j + 1] : a;
                int16_t v = clip16((1.0f - fr) * a + fr * b);
                chunk[ci++] = v;
                chunk[ci++] = v;
                m++;
            }
            ok = fwrite(chunk, sizeof(int16_t), ci, f) == (size_t)ci;
        }
        fclose(f);
        if (!ok) {
            printf("MeckVoice: write to %s failed\n", path);
            remove(path);
        }
        return ok;
    }

    static bool writeSidecar(const char *wavPath, const uint8_t *c2, uint32_t c2Bytes) {
        char p[120];
        sidecarPath(wavPath, p, sizeof(p));
        FILE *f = fopen(p, "wb");
        if (!f) return false;
        bool ok = fwrite(c2, 1, c2Bytes, f) == c2Bytes;
        fclose(f);
        return ok;
    }

    // Reads a 16-bit WAV (any channel count; first channel kept) into out.
    // Returns the sample count.
    static uint32_t loadWavMono(const char *path, int16_t *out, uint32_t maxSamples) {
        FILE *f = fopen(path, "rb");
        if (!f) {
            printf("MeckVoice: loadWav: can't open %s\n", path);
            return 0;
        }
        uint8_t hdr[44];
        if (fread(hdr, 1, 44, f) != 44 ||
            memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) {
            printf("MeckVoice: loadWav: not a WAV file\n");
            fclose(f);
            return 0;
        }
        uint16_t channels = hdr[22] | (hdr[23] << 8);
        uint16_t bitsPerSample = hdr[34] | (hdr[35] << 8);
        uint32_t dataSize = hdr[40] | (hdr[41] << 8) | (hdr[42] << 16) | ((uint32_t)hdr[43] << 24);
        if (bitsPerSample != 16 || channels == 0 || channels > 2) {
            printf("MeckVoice: loadWav: only 16-bit mono/stereo supported\n");
            fclose(f);
            return 0;
        }
        uint32_t frames = dataSize / (channels * sizeof(int16_t));
        uint32_t n = 0;
        int16_t frame[2];
        while (n < frames && n < maxSamples &&
               fread(frame, sizeof(int16_t), channels, f) == channels) {
            out[n++] = frame[0];
        }
        fclose(f);
        return n;
    }

    // ---- State ----
    SemaphoreHandle_t _lock = nullptr;
    TaskHandle_t _capTask = nullptr;
    TaskHandle_t _workTask = nullptr;

    // Recording (capture task writes _recBuffer while REC_RECORDING; the work
    // task reads it while REC_SAVING)
    int16_t *_recBuffer = nullptr;
    std::atomic<int>      _recState{REC_IDLE};
    std::atomic<bool>     _stopReq{false};
    std::atomic<uint32_t> _capValues{0};
    uint32_t              _capFinal = 0;
    std::atomic<bool>     _jobRecord{false};
    std::atomic<bool>     _bufBusy{false};   // work task has the buffer (under _lock)

    // Filter areas inside _recBuffer (see ensureCaptureBuffer)
    int16_t *_pcm16 = nullptr;
    int16_t *_pcm8k = nullptr;
    float    _fir[VOICE_FIR_TAPS];
    float    _aa[VOICE_AA_TAPS];

    // Selected note for Send (under _lock)
    uint8_t               _c2[VOICE_C2_MAX_BYTES] = {};
    std::atomic<uint32_t> _c2Bytes{0};
    std::atomic<bool>     _c2Valid{false};
    char                  _prepPath[112];
    bool                  _jobPrepare = false;

    // Results for the UI (under _lock)
    RecordResult _recResult;
    bool         _recResultReady = false;
    InboxResult  _inboxResult;
    bool         _inboxResultReady = false;

    // Sessions
    OutSession _out;     // under _lock (meck_task, LVGL task)
    InSession  _in;      // meck_task only
    RxJob      _rxJob;   // under _lock (meck_task -> work task)
};
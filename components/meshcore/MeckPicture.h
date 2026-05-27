#pragma once
// ============================================================================
// MeckPicture.h — Picture transfer over LoRa via channel messages
// ============================================================================
// Chunks a file into base64-encoded channel messages with format:
//   [PD:SSSSSSSS:NN/TT:base64data]
// where S=session(hex), NN=chunk(dec), TT=total(dec), base64data=payload.
//
// Channel messages CAN flood through repeaters (unlike RAW_CUSTOM),
// so this works even without a direct path to the recipient.
//
// Max file size: ~4KB (practical limit for ~30 messages at 1s spacing).
// ============================================================================

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>

#define PIC_CHUNK_MARKER     "[PD:"
#define PIC_CHUNK_MARKER_LEN 4
#define PIC_MAX_FILE_BYTES   4096
#define PIC_SEND_INTERVAL_MS 2000   // ms between chunk messages
#define PIC_MAX_CHUNKS       40

// Base64 encode/decode (minimal, self-contained)
static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static int b64_encode(const uint8_t *in, int in_len, char *out, int out_max) {
    int o = 0;
    for (int i = 0; i < in_len && o + 4 < out_max; i += 3) {
        uint32_t v = (uint32_t)in[i] << 16;
        if (i + 1 < in_len) v |= (uint32_t)in[i + 1] << 8;
        if (i + 2 < in_len) v |= (uint32_t)in[i + 2];
        out[o++] = b64_table[(v >> 18) & 0x3F];
        out[o++] = b64_table[(v >> 12) & 0x3F];
        out[o++] = (i + 1 < in_len) ? b64_table[(v >> 6) & 0x3F] : '=';
        out[o++] = (i + 2 < in_len) ? b64_table[v & 0x3F] : '=';
    }
    out[o] = '\0';
    return o;
}

static int b64_decode_char(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

static int b64_decode(const char *in, int in_len, uint8_t *out, int out_max) {
    int o = 0;
    for (int i = 0; i + 3 < in_len && o < out_max; i += 4) {
        int a = b64_decode_char(in[i]);
        int b = b64_decode_char(in[i + 1]);
        int c = (in[i + 2] != '=') ? b64_decode_char(in[i + 2]) : 0;
        int d = (in[i + 3] != '=') ? b64_decode_char(in[i + 3]) : 0;
        if (a < 0 || b < 0) break;
        uint32_t v = ((uint32_t)a << 18) | ((uint32_t)b << 12)
                   | ((uint32_t)c << 6) | (uint32_t)d;
        if (o < out_max) out[o++] = (v >> 16) & 0xFF;
        if (in[i + 2] != '=' && o < out_max) out[o++] = (v >> 8) & 0xFF;
        if (in[i + 3] != '=' && o < out_max) out[o++] = v & 0xFF;
    }
    return o;
}

// ============================================================================
// Picture send state
// ============================================================================

struct MeckPictureSend {
    bool     active;
    uint8_t  channel_idx;
    uint32_t session_id;
    uint8_t  data[PIC_MAX_FILE_BYTES];
    int      data_len;
    int      total_chunks;
    int      next_chunk;
    uint32_t last_send_ms;
    char     filename[48];

    // Raw bytes per chunk: channel msg max ~180 chars.
    // Header "[PD:SSSSSSSS:NN/TT:" = ~20 chars + "]" = ~21 chars.
    // Leaves ~159 chars for base64 = 119 raw bytes per chunk.
    static constexpr int RAW_PER_CHUNK = 119;

    void start(uint8_t ch_idx, const char *filepath) {
        FILE *f = fopen(filepath, "rb");
        if (!f) {
            printf("PicSend: failed to open %s\n", filepath);
            return;
        }
        fseek(f, 0, SEEK_END);
        data_len = (int)ftell(f);
        fseek(f, 0, SEEK_SET);
        if (data_len > PIC_MAX_FILE_BYTES) {
            printf("PicSend: file too large (%d > %d)\n", data_len, PIC_MAX_FILE_BYTES);
            fclose(f);
            return;
        }
        if (data_len <= 0) {
            printf("PicSend: empty file\n");
            fclose(f);
            return;
        }
        fread(data, 1, data_len, f);
        fclose(f);

        channel_idx = ch_idx;
        session_id = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
        total_chunks = (data_len + RAW_PER_CHUNK - 1) / RAW_PER_CHUNK;
        next_chunk = 0;
        last_send_ms = 0;
        active = true;

        const char *slash = strrchr(filepath, '/');
        strncpy(filename, slash ? slash + 1 : filepath, sizeof(filename) - 1);
        filename[sizeof(filename) - 1] = '\0';

        printf("PicSend: %s (%d bytes, %d chunks, session 0x%08lX) on ch[%d]\n",
               filename, data_len, total_chunks,
               (unsigned long)session_id, channel_idx);
    }

    // Build the next chunk message. Returns the message text, or NULL if done.
    // Caller sends this as a channel message.
    const char* buildNextChunk(char *buf, int buf_len) {
        if (!active || next_chunk >= total_chunks) {
            active = false;
            return NULL;
        }

        int offset = next_chunk * RAW_PER_CHUNK;
        int chunk_len = data_len - offset;
        if (chunk_len > RAW_PER_CHUNK) chunk_len = RAW_PER_CHUNK;

        char b64[200];
        b64_encode(&data[offset], chunk_len, b64, sizeof(b64));

        // First chunk includes filename after the base64
        if (next_chunk == 0) {
            snprintf(buf, buf_len, "[PD:%08lX:%d/%d:%s|%s]",
                     (unsigned long)session_id,
                     next_chunk, total_chunks, b64, filename);
        } else {
            snprintf(buf, buf_len, "[PD:%08lX:%d/%d:%s]",
                     (unsigned long)session_id,
                     next_chunk, total_chunks, b64);
        }

        next_chunk++;
        last_send_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        printf("PicSend: chunk %d/%d (%d bytes, %d b64 chars)\n",
               next_chunk, total_chunks, chunk_len, (int)strlen(b64));

        if (next_chunk >= total_chunks) {
            active = false;
            printf("PicSend: all chunks queued\n");
        }
        return buf;
    }

    bool isReady() const {
        if (!active) return false;
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
        return (now - last_send_ms) >= PIC_SEND_INTERVAL_MS;
    }
};

// ============================================================================
// Picture receive state
// ============================================================================

struct MeckPictureRecv {
    bool     active;
    uint32_t session_id;
    int      total_chunks;
    int      received_count;
    bool     chunk_received[PIC_MAX_CHUNKS];
    uint8_t  data[PIC_MAX_FILE_BYTES];
    int      chunk_offsets[PIC_MAX_CHUNKS];  // byte offset of each chunk in data[]
    int      chunk_lengths[PIC_MAX_CHUNKS];  // byte length of each chunk
    int      data_len;
    char     filename[48];
    char     sender_name[32];
    uint32_t last_recv_ms;

    void reset() {
        active = false;
        session_id = 0;
        total_chunks = 0;
        received_count = 0;
        data_len = 0;
        filename[0] = '\0';
        sender_name[0] = '\0';
        memset(chunk_received, 0, sizeof(chunk_received));
        memset(chunk_offsets, 0, sizeof(chunk_offsets));
        memset(chunk_lengths, 0, sizeof(chunk_lengths));
    }

    // Parse and store a received chunk. Returns true if the picture is complete.
    bool onChunkReceived(const char *msg, const char *sender) {
        // Parse: [PD:SSSSSSSS:NN/TT:base64data] or [PD:SSSSSSSS:NN/TT:base64|filename]
        if (strncmp(msg, PIC_CHUNK_MARKER, PIC_CHUNK_MARKER_LEN) != 0) return false;

        const char *p = msg + PIC_CHUNK_MARKER_LEN;

        // Session ID (8 hex chars)
        uint32_t sid = 0;
        for (int i = 0; i < 8 && *p; i++, p++) {
            sid <<= 4;
            if (*p >= '0' && *p <= '9') sid |= (*p - '0');
            else if (*p >= 'a' && *p <= 'f') sid |= (*p - 'a' + 10);
            else if (*p >= 'A' && *p <= 'F') sid |= (*p - 'A' + 10);
        }
        if (*p != ':') return false;
        p++;

        // Chunk index
        int chunk_idx = 0;
        while (*p >= '0' && *p <= '9') { chunk_idx = chunk_idx * 10 + (*p - '0'); p++; }
        if (*p != '/') return false;
        p++;

        // Total chunks
        int total = 0;
        while (*p >= '0' && *p <= '9') { total = total * 10 + (*p - '0'); p++; }
        if (*p != ':') return false;
        p++;

        if (total <= 0 || total > PIC_MAX_CHUNKS || chunk_idx < 0 || chunk_idx >= total)
            return false;

        // Find end of base64 data (either | for filename or ] for end)
        const char *data_start = p;
        const char *data_end = p;
        const char *fname = NULL;
        while (*data_end && *data_end != ']' && *data_end != '|') data_end++;
        if (*data_end == '|') {
            fname = data_end + 1;
        }
        int b64_len = (int)(data_end - data_start);

        // New session or different session?
        if (!active || sid != session_id) {
            reset();
            active = true;
            session_id = sid;
            total_chunks = total;
            strncpy(sender_name, sender ? sender : "?", sizeof(sender_name) - 1);
            sender_name[sizeof(sender_name) - 1] = '\0';
            printf("PicRecv: new session 0x%08lX from %s (%d chunks)\n",
                   (unsigned long)sid, sender_name, total);
        }

        // Extract filename from first chunk
        if (fname && chunk_idx == 0) {
            const char *fend = fname;
            while (*fend && *fend != ']') fend++;
            int flen = (int)(fend - fname);
            if (flen > 0 && flen < (int)sizeof(filename)) {
                memcpy(filename, fname, flen);
                filename[flen] = '\0';
            }
        }

        // Decode base64 chunk
        if (!chunk_received[chunk_idx]) {
            uint8_t decoded[200];
            int decoded_len = b64_decode(data_start, b64_len, decoded, sizeof(decoded));
            if (decoded_len > 0) {
                // Store at the correct offset
                int offset = chunk_idx * MeckPictureSend::RAW_PER_CHUNK;
                if (offset + decoded_len <= PIC_MAX_FILE_BYTES) {
                    memcpy(&data[offset], decoded, decoded_len);
                    chunk_offsets[chunk_idx] = offset;
                    chunk_lengths[chunk_idx] = decoded_len;
                    chunk_received[chunk_idx] = true;
                    received_count++;
                    // Track total assembled length
                    if (offset + decoded_len > data_len)
                        data_len = offset + decoded_len;
                }
            }
            printf("PicRecv: chunk %d/%d (%d bytes), %d/%d received\n",
                   chunk_idx + 1, total_chunks, decoded_len,
                   received_count, total_chunks);
        }

        last_recv_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        // Check if complete
        if (received_count >= total_chunks) {
            printf("PicRecv: complete! %d bytes, filename='%s'\n",
                   data_len, filename);
            return true;
        }
        return false;
    }

    // Save the reassembled picture to SD card.
    // Returns the full path, or empty string on failure.
    bool saveToSD(char *out_path, int out_path_len) {
        if (!active || received_count < total_chunks) return false;

        struct stat st;
        if (stat("/sdcard/pictures", &st) != 0) {
            mkdir("/sdcard/pictures", 0755);
        }

        // Use original filename or generate one
        char fname[64];
        if (filename[0] != '\0') {
            snprintf(fname, sizeof(fname), "%s", filename);
        } else {
            snprintf(fname, sizeof(fname), "pic_%08lX.jpg",
                     (unsigned long)session_id);
        }

        snprintf(out_path, out_path_len, "/sdcard/pictures/%s", fname);

        FILE *f = fopen(out_path, "wb");
        if (!f) {
            printf("PicRecv: failed to save %s\n", out_path);
            return false;
        }
        fwrite(data, 1, data_len, f);
        fclose(f);

        printf("PicRecv: saved %s (%d bytes)\n", out_path, data_len);
        reset();
        return true;
    }
};
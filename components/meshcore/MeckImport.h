/*
 * MeckImport.h — One-shot import of MeshCore app config from SD card.
 *
 * Looks for /sdcard/meshcore/import.json on boot. If present, parses the
 * JSON exported by the MeshCore mobile/desktop app and applies its
 * contents to Meck's persisted state before the rest of meck_app_init
 * loads anything from SD or NVS.
 *
 * Behaviour:
 *   - identity (private_key + public_key)  → REPLACE _main.id on SD
 *   - channels                              → MERGE into channels.bin
 *                                             (existing channels kept;
 *                                              new ones from import added
 *                                              to free slots; duplicates
 *                                              by secret are skipped)
 *   - contacts                              → MERGE into contacts.bin
 *                                             (existing contacts kept;
 *                                              new ones added; duplicates
 *                                              by pub_key are skipped)
 *   - name + radio_settings                 → REPLACE corresponding fields
 *                                             in P4NodePrefs (the caller
 *                                             passes a reference to the
 *                                             prefs struct, this writes
 *                                             back in-place and to SD)
 *
 * On any parse/IO failure, the import.json file is left in place and a
 * warning is logged so the user can inspect and re-attempt.
 *
 * On full success, the file is moved to
 *   /sdcard/meshcore/import.history/import-<unix_epoch>.json
 * so it doesn't re-apply on the next boot and so the user has an audit
 * trail of what was imported.
 *
 * Called from meck_app_init() between dataStore.begin()/restoreFromSD()
 * and loadPrefs() — early enough that the imported state is what gets
 * loaded by the normal init sequence on the same boot.
 */

#pragma once

#include "MeckDataStore.h"
#include "MeckSDCard.h"
#include "cJSON.h"
#include "esp_log.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <sys/stat.h>
#include <sys/unistd.h>

static constexpr const char* MECK_IMPORT_PATH       = "/sdcard/meshcore/import.json";
static constexpr const char* MECK_IMPORT_HISTORY_DIR = "/sdcard/meshcore/import.history";

// Strip non-ASCII bytes from src into dst, copying at most dst_size-1
// bytes and always null-terminating. Anything above 0x7E (printable
// ASCII upper bound) is dropped, which removes emoji, accented Latin,
// and other multi-byte UTF-8 sequences. Used on imported names until
// Meck-P4 has proper emoji rendering — otherwise emojis would either
// truncate mid-codepoint or render as boxes/mojibake.
static void meck_import_strip_to_ascii(const char* src, char* dst, size_t dst_size) {
    if (dst_size == 0) return;
    size_t out = 0;
    if (src) {
        for (size_t i = 0; src[i] != '\0' && out + 1 < dst_size; i++) {
            unsigned char c = (unsigned char)src[i];
            // Keep printable ASCII (0x20-0x7E) plus tab and newline.
            // Drop everything else (control chars below 0x20 except
            // those two, DEL, and the entire upper half 0x80-0xFF
            // which is where multi-byte UTF-8 lives).
            if ((c >= 0x20 && c <= 0x7E) || c == '\t' || c == '\n') {
                dst[out++] = (char)c;
            }
        }
    }
    dst[out] = '\0';
    // Trim trailing spaces left behind after stripping emojis at the
    // end of the string (e.g. "Bridge W🛣" -> "Bridge W"). Leading
    // spaces from "🌱Camperdown" -> "Camperdown" are also trimmed.
    while (out > 0 && dst[out - 1] == ' ') {
        dst[--out] = '\0';
    }
    size_t start = 0;
    while (dst[start] == ' ') start++;
    if (start > 0) {
        memmove(dst, dst + start, out - start + 1);
    }
}

// Decode a hex string of expected_len*2 chars into out_buf (expected_len
// bytes). Returns true on success, false on malformed input.
static bool meck_import_hex_decode(const char* hex, uint8_t* out_buf, size_t expected_len) {
    if (!hex || !out_buf) return false;
    if (strlen(hex) != expected_len * 2) return false;
    for (size_t i = 0; i < expected_len; i++) {
        char c1 = hex[i * 2];
        char c2 = hex[i * 2 + 1];
        uint8_t v1, v2;
        if      (c1 >= '0' && c1 <= '9') v1 = c1 - '0';
        else if (c1 >= 'a' && c1 <= 'f') v1 = c1 - 'a' + 10;
        else if (c1 >= 'A' && c1 <= 'F') v1 = c1 - 'A' + 10;
        else return false;
        if      (c2 >= '0' && c2 <= '9') v2 = c2 - '0';
        else if (c2 >= 'a' && c2 <= 'f') v2 = c2 - 'a' + 10;
        else if (c2 >= 'A' && c2 <= 'F') v2 = c2 - 'A' + 10;
        else return false;
        out_buf[i] = (v1 << 4) | v2;
    }
    return true;
}

// Read the entire import.json file into a freshly-allocated heap buffer.
// Caller owns the returned pointer and must free() it. Returns nullptr
// on any failure (file missing, too large, read error).
static char* meck_import_read_file() {
    FILE* f = fopen(MECK_IMPORT_PATH, "rb");
    if (!f) return nullptr;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return nullptr; }
    long len = ftell(f);
    if (len <= 0 || len > 256 * 1024) { fclose(f); return nullptr; }
    rewind(f);
    char* buf = (char*)malloc((size_t)len + 1);
    if (!buf) { fclose(f); return nullptr; }
    size_t rd = fread(buf, 1, (size_t)len, f);
    fclose(f);
    if (rd != (size_t)len) { free(buf); return nullptr; }
    buf[len] = '\0';
    return buf;
}

// Move import.json to import.history/import-<epoch>.json on success.
// Best-effort: a failure here doesn't fail the import (the data is
// already applied), but it does mean the user may see the import run
// again on next boot.
static void meck_import_archive_file() {
    mkdir(MECK_IMPORT_HISTORY_DIR, 0755);  // ignore EEXIST
    time_t now = time(nullptr);
    char dest[160];
    snprintf(dest, sizeof(dest), "%s/import-%ld.json",
             MECK_IMPORT_HISTORY_DIR, (long)now);
    if (rename(MECK_IMPORT_PATH, dest) != 0) {
        ESP_LOGW("import", "archive rename failed (errno=%d), unlinking instead", errno);
        unlink(MECK_IMPORT_PATH);
    } else {
        ESP_LOGI("import", "moved %s -> %s", MECK_IMPORT_PATH, dest);
    }
}

// Write the 96-byte identity (priv || pub) to _main.id. Atomic via tmp+rename.
static bool meck_import_write_identity(const uint8_t* priv64, const uint8_t* pub32) {
    mkdir("/sdcard/meshcore", 0755);
    mkdir("/sdcard/meshcore/identity", 0755);

    const char* path     = "/sdcard/meshcore/identity/_main.id";
    const char* tmp_path = "/sdcard/meshcore/identity/_main.id.tmp";

    FILE* f = fopen(tmp_path, "wb");
    if (!f) {
        ESP_LOGW("import", "identity write: fopen(%s) failed (errno=%d)", tmp_path, errno);
        return false;
    }
    size_t wrote = 0;
    wrote += fwrite(priv64, 1, 64, f);
    wrote += fwrite(pub32,  1, 32, f);
    int closed = fclose(f);
    if (wrote != 96 || closed != 0) {
        unlink(tmp_path);
        ESP_LOGW("import", "identity write incomplete (wrote=%u, close=%d)",
                 (unsigned)wrote, closed);
        return false;
    }
    unlink(path);
    if (rename(tmp_path, path) != 0) {
        ESP_LOGW("import", "identity rename(%s,%s) failed (errno=%d)",
                 tmp_path, path, errno);
        unlink(tmp_path);
        return false;
    }
    ESP_LOGI("import", "wrote %s (96 bytes)", path);
    return true;
}

// Merge channels from the import.json into channels.bin on SD. Existing
// channels (loaded via P4DataStore) are preserved; new channels (by
// secret) are appended into free slots up to MAX_GROUP_CHANNELS. Returns
// number of channels added. Returns -1 on hard error.
static int meck_import_merge_channels(P4DataStore& store, cJSON* arr) {
    if (!arr || !cJSON_IsArray(arr)) return 0;

    P4ChannelRecord existing[MAX_GROUP_CHANNELS];
    int existing_count = 0;
    store.loadChannels(existing, MAX_GROUP_CHANNELS, existing_count);

    int added = 0;
    int n = cJSON_GetArraySize(arr);
    for (int i = 0; i < n; i++) {
        cJSON* item = cJSON_GetArrayItem(arr, i);
        if (!item) continue;
        cJSON* j_name   = cJSON_GetObjectItem(item, "name");
        cJSON* j_secret = cJSON_GetObjectItem(item, "secret");
        if (!cJSON_IsString(j_name) || !cJSON_IsString(j_secret)) continue;

        uint8_t secret16[16];
        if (!meck_import_hex_decode(j_secret->valuestring, secret16, 16)) {
            ESP_LOGW("import", "channel '%s': bad secret hex, skipping",
                     j_name->valuestring);
            continue;
        }

        // Skip if a channel with this secret already exists.
        bool dup = false;
        for (int e = 0; e < existing_count; e++) {
            if (memcmp(existing[e].secret, secret16, 16) == 0) {
                dup = true;
                break;
            }
        }
        if (dup) continue;

        if (existing_count >= MAX_GROUP_CHANNELS) {
            ESP_LOGW("import", "channel '%s': no free slot (cap %d), skipping",
                     j_name->valuestring, (int)MAX_GROUP_CHANNELS);
            continue;
        }

        // Append into next slot. The on-disk format reserves
        // P4_CHANNEL_SECRET_LEN bytes (32) for the secret; only the
        // first 16 are used (AES-128), upper 16 stay zeroed.
        P4ChannelRecord& slot = existing[existing_count];
        memset(&slot, 0, sizeof(slot));
        meck_import_strip_to_ascii(j_name->valuestring, slot.name, P4_CHANNEL_NAME_MAX);
        memcpy(slot.secret, secret16, 16);
        slot.active = 1;
        existing_count++;
        added++;
    }

    if (added > 0) {
        store.saveChannels(existing, MAX_GROUP_CHANNELS);
    }
    return added;
}

// Merge contacts from import.json into contacts.bin on SD. Existing
// contacts preserved; new ones (by pub_key) appended. Returns number
// of contacts added, or -1 on hard error.
static int meck_import_merge_contacts(P4DataStore& store, cJSON* arr) {
    if (!arr || !cJSON_IsArray(arr)) return 0;

    // Load existing contacts.
    typedef P4DataStore::ContactRecord CR;
    CR* existing = (CR*)malloc(sizeof(CR) * MAX_CONTACTS);
    if (!existing) return -1;
    int existing_count = store.loadContacts(existing, MAX_CONTACTS);
    if (existing_count < 0) existing_count = 0;

    int added = 0;
    int n = cJSON_GetArraySize(arr);
    for (int i = 0; i < n; i++) {
        cJSON* item = cJSON_GetArrayItem(arr, i);
        if (!item) continue;
        cJSON* j_pub  = cJSON_GetObjectItem(item, "public_key");
        if (!cJSON_IsString(j_pub)) continue;

        uint8_t pub32[32];
        if (!meck_import_hex_decode(j_pub->valuestring, pub32, 32)) {
            ESP_LOGW("import", "contact: bad public_key hex, skipping");
            continue;
        }

        // Dedup by pub_key.
        bool dup = false;
        for (int e = 0; e < existing_count; e++) {
            if (memcmp(existing[e].pub_key, pub32, 32) == 0) {
                dup = true;
                break;
            }
        }
        if (dup) continue;

        if (existing_count >= MAX_CONTACTS) {
            ESP_LOGW("import", "contacts: no free slot (cap %d), stopping",
                     (int)MAX_CONTACTS);
            break;
        }

        // Build a record. Prefer custom_name if present, else name.
        cJSON* j_custom = cJSON_GetObjectItem(item, "custom_name");
        cJSON* j_name   = cJSON_GetObjectItem(item, "name");
        const char* name_src = nullptr;
        if (cJSON_IsString(j_custom) && j_custom->valuestring[0] != '\0') {
            name_src = j_custom->valuestring;
        } else if (cJSON_IsString(j_name)) {
            name_src = j_name->valuestring;
        }

        CR& slot = existing[existing_count];
        memset(&slot, 0, sizeof(slot));
        memcpy(slot.pub_key, pub32, 32);
        if (name_src) {
            meck_import_strip_to_ascii(name_src, slot.name, sizeof(slot.name));
        }

        cJSON* j_type  = cJSON_GetObjectItem(item, "type");
        if (cJSON_IsNumber(j_type)) slot.type = (uint8_t)j_type->valueint;

        cJSON* j_flags = cJSON_GetObjectItem(item, "flags");
        if (cJSON_IsNumber(j_flags)) slot.flags = (uint8_t)j_flags->valueint;

        // lat/lon arrive as strings in the JSON (e.g. "-33.878016").
        // Convert to e7 integer.
        cJSON* j_lat = cJSON_GetObjectItem(item, "latitude");
        cJSON* j_lon = cJSON_GetObjectItem(item, "longitude");
        if (cJSON_IsString(j_lat)) {
            double d = atof(j_lat->valuestring);
            slot.gps_lat = (int32_t)(d * 1e7);
        }
        if (cJSON_IsString(j_lon)) {
            double d = atof(j_lon->valuestring);
            slot.gps_lon = (int32_t)(d * 1e7);
        }

        cJSON* j_lastadv = cJSON_GetObjectItem(item, "last_advert");
        if (cJSON_IsNumber(j_lastadv)) {
            slot.last_advert_timestamp = (uint32_t)j_lastadv->valuedouble;
        }
        cJSON* j_lastmod = cJSON_GetObjectItem(item, "last_modified");
        if (cJSON_IsNumber(j_lastmod)) {
            slot.lastmod = (uint32_t)j_lastmod->valuedouble;
        }

        existing_count++;
        added++;
    }

    if (added > 0) {
        store.saveContacts(existing, existing_count);
    }
    free(existing);
    return added;
}

// Apply radio_settings + name from the JSON to a P4NodePrefs struct in
// place. Caller is responsible for persisting via savePrefs() afterwards.
static void meck_import_apply_prefs(cJSON* root, P4NodePrefs& prefs) {
    cJSON* j_name = cJSON_GetObjectItem(root, "name");
    if (cJSON_IsString(j_name) && j_name->valuestring[0] != '\0') {
        meck_import_strip_to_ascii(j_name->valuestring,
                                   prefs.node_name,
                                   sizeof(prefs.node_name));
    }

    cJSON* radio = cJSON_GetObjectItem(root, "radio_settings");
    if (radio) {
        cJSON* j_freq = cJSON_GetObjectItem(radio, "frequency");
        cJSON* j_bw   = cJSON_GetObjectItem(radio, "bandwidth");
        cJSON* j_sf   = cJSON_GetObjectItem(radio, "spreading_factor");
        cJSON* j_cr   = cJSON_GetObjectItem(radio, "coding_rate");
        cJSON* j_tx   = cJSON_GetObjectItem(radio, "tx_power");
        // JSON frequency is kHz, P4NodePrefs::freq is MHz.
        if (cJSON_IsNumber(j_freq)) prefs.freq = (float)(j_freq->valuedouble / 1000.0);
        // JSON bandwidth is Hz, P4NodePrefs::bw is kHz.
        if (cJSON_IsNumber(j_bw))   prefs.bw   = (float)(j_bw->valuedouble / 1000.0);
        if (cJSON_IsNumber(j_sf))   prefs.sf   = (uint8_t)j_sf->valueint;
        if (cJSON_IsNumber(j_cr))   prefs.cr   = (uint8_t)j_cr->valueint;
        if (cJSON_IsNumber(j_tx))   prefs.tx_power_dbm = (uint8_t)j_tx->valueint;
    }
}

// Entry point. Returns true if an import was applied (caller should
// persist prefs after this call). Returns false if there was no file,
// or if parsing failed (file left in place for user inspection).
static bool meck_import_from_sd(P4DataStore& store, P4NodePrefs& prefs) {
    if (!p4_sdcard_is_mounted()) return false;
    if (!p4_sdcard_file_exists(MECK_IMPORT_PATH)) return false;

    ESP_LOGI("import", "found %s, attempting import", MECK_IMPORT_PATH);

    char* text = meck_import_read_file();
    if (!text) {
        ESP_LOGW("import", "could not read %s (errno=%d), leaving in place",
                 MECK_IMPORT_PATH, errno);
        return false;
    }

    cJSON* root = cJSON_Parse(text);
    if (!root) {
        const char* err_at = cJSON_GetErrorPtr();
        size_t err_offset = (err_at && err_at >= text) ? (size_t)(err_at - text) : 0;
        free(text);
        ESP_LOGW("import", "parse failed at offset %zu, leaving file in place",
                 err_offset);
        return false;
    }
    free(text);

    // Identity is mandatory — without it the import isn't meaningful.
    cJSON* j_priv = cJSON_GetObjectItem(root, "private_key");
    cJSON* j_pub  = cJSON_GetObjectItem(root, "public_key");
    if (!cJSON_IsString(j_priv) || !cJSON_IsString(j_pub)) {
        ESP_LOGW("import", "missing private_key or public_key, leaving file in place");
        cJSON_Delete(root);
        return false;
    }

    uint8_t priv64[64];
    uint8_t pub32[32];
    if (!meck_import_hex_decode(j_priv->valuestring, priv64, 64)) {
        ESP_LOGW("import", "private_key: bad hex (expected 128 chars), leaving file");
        cJSON_Delete(root);
        return false;
    }
    if (!meck_import_hex_decode(j_pub->valuestring, pub32, 32)) {
        ESP_LOGW("import", "public_key: bad hex (expected 64 chars), leaving file");
        cJSON_Delete(root);
        return false;
    }

    if (!meck_import_write_identity(priv64, pub32)) {
        ESP_LOGW("import", "identity write failed, leaving file in place");
        cJSON_Delete(root);
        return false;
    }

    int ch_added = meck_import_merge_channels(store, cJSON_GetObjectItem(root, "channels"));
    int ct_added = meck_import_merge_contacts(store, cJSON_GetObjectItem(root, "contacts"));
    meck_import_apply_prefs(root, prefs);

    ESP_LOGI("import", "applied identity + %d channels + %d contacts + prefs",
             ch_added, ct_added);

    cJSON_Delete(root);
    meck_import_archive_file();
    return true;
}
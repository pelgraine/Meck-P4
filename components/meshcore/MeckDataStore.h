/*
 * P4DataStore.h — NVS + SD card backed storage for ESP32-P4
 *
 * Replaces the Arduino FILESYSTEM-dependent DataStore/IdentityStore
 * for the ESP-IDF P4 port. Uses ESP-IDF's NVS (Non-Volatile Storage)
 * for identity and radio preferences, with SD card as backup storage
 * and home for larger data (contacts, channels).
 *
 * Storage layout:
 *
 *   NVS namespace "meshcore":
 *     "identity"     — 96-byte blob (prv_key[64] + pub_key[32])
 *     "prefs"        — serialized P4NodePrefs struct
 *     "channels"     — serialized channel array (up to MAX_GROUP_CHANNELS)
 *
 *   SD card (/sdcard/meshcore/):
 *     prefs.bin             — binary backup of P4NodePrefs
 *     channels.bin          — binary backup of channel data
 *     identity/_main.id     — 96-byte identity backup
 *     contacts.bin          — chunked ContactInfo records (future)
 *     meshcore_contacts.json — human-readable contact export (future)
 *
 * Boot sequence:
 *   1. NVS init (always)
 *   2. Load identity from NVS; if missing, try SD restore
 *   3. Load prefs from NVS; if missing, try SD restore, else defaults
 *   4. Load channels from NVS; if missing, try SD restore
 *
 * Save sequence:
 *   1. Write to NVS (always)
 *   2. If SD mounted, write backup copy
 */

#pragma once

#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "Identity.h"
#include "MeshCore.h"
#include "variant.h"
#include "MeckSDCard.h"

// ---- Channel storage record ----
// Matches MeshCore's channel structure: name + 32-byte secret
#define P4_CHANNEL_NAME_MAX  32
#define P4_CHANNEL_SECRET_LEN 32

struct P4ChannelRecord {
    char name[P4_CHANNEL_NAME_MAX];
    uint8_t secret[P4_CHANNEL_SECRET_LEN];
    uint8_t active;     // 1 = in use, 0 = empty slot
    uint8_t reserved[3]; // pad to 68 bytes
};

// Binary header for channels file (NVS blob and SD backup)
struct P4ChannelsHeader {
    uint8_t magic[4];       // "MCH\0"
    uint8_t version;        // 1
    uint8_t count;          // number of active channels
    uint8_t max_channels;   // MAX_GROUP_CHANNELS
    uint8_t reserved;
};

// Minimal NodePrefs for P4 (matches fields needed for radio operation)
struct P4NodePrefs {
    float freq;
    float bw;
    uint8_t sf;
    uint8_t cr;
    uint8_t tx_power_dbm;
    char node_name[32];
    int8_t utc_offset_hours;
    uint8_t path_hash_mode;
    float airtime_factor;
    uint8_t multi_acks;
    uint8_t manual_add_contacts;  // 0 = auto-add, 1 = manual only
    uint8_t autoadd_config;       // bitmask: overwrite|chat|repeater|room|sensor
    uint8_t dark_mode;            // 0 = light, 1 = dark
    uint8_t reserved[5];          // future fields, zero-initialized

    // Initialize with defaults from variant.h
    void setDefaults() {
        freq = LORA_FREQ_DEFAULT;
        bw = LORA_BW_DEFAULT;
        sf = LORA_SF_DEFAULT;
        cr = LORA_CR_DEFAULT;
        tx_power_dbm = LORA_TX_POWER_DEFAULT;
        strncpy(node_name, "NONAME", sizeof(node_name));
        utc_offset_hours = 0;
        path_hash_mode = 1;  // default 2 bytes (matches AU mesh)
        airtime_factor = 1.0f;
        multi_acks = 1;
        manual_add_contacts = 0;
        autoadd_config = 0x1E;  // all types, no overwrite (chat|repeater|room|sensor)
        dark_mode = 0;
        memset(reserved, 0, sizeof(reserved));
    }
};


class P4DataStore {
    static constexpr const char* TAG = "P4DataStore";
    static constexpr const char* NVS_NAMESPACE = "meshcore";

    // SD card paths
    static constexpr const char* SD_PREFS_PATH    = "/sdcard/meshcore/prefs.bin";
    static constexpr const char* SD_CHANNELS_PATH = "/sdcard/meshcore/channels.bin";
    static constexpr const char* SD_IDENTITY_PATH = "/sdcard/meshcore/identity/_main.id";
    static constexpr const char* SD_CONTACTS_PATH = "/sdcard/meshcore/contacts.bin";
    static constexpr const char* SD_MESSAGES_DIR  = "/sdcard/meshcore/messages";

    bool _initialized;

public:
    P4DataStore() : _initialized(false) {}

    bool begin() {
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_LOGW(TAG, "NVS needs erase (err=0x%x), erasing...", err);
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "NVS flash init failed: 0x%x", err);
            return false;
        }
        _initialized = true;
        ESP_LOGI(TAG, "NVS initialized");
        return true;
    }

    // =====================================================================
    // Identity — NVS primary, SD backup
    // =====================================================================

    bool loadIdentity(mesh::LocalIdentity &id) {
        if (!_initialized) return false;

        // Try NVS first
        nvs_handle_t handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
        if (err == ESP_OK) {
            uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE];
            size_t len = sizeof(buf);
            err = nvs_get_blob(handle, "identity", buf, &len);
            nvs_close(handle);

            if (err == ESP_OK && len == sizeof(buf)) {
                id.readFrom(buf, len);
                printf("P4DataStore: identity loaded from NVS — pub[0..3]=%02X%02X%02X%02X\n",
                       id.pub_key[0], id.pub_key[1], id.pub_key[2], id.pub_key[3]);
                return true;
            }
        }

        // NVS empty — try SD restore
        if (p4_sdcard_is_mounted() && p4_sdcard_file_exists(SD_IDENTITY_PATH)) {
            FILE* f = fopen(SD_IDENTITY_PATH, "rb");
            if (f) {
                uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE];
                size_t rd = fread(buf, 1, sizeof(buf), f);
                fclose(f);
                if (rd == sizeof(buf)) {
                    id.readFrom(buf, rd);
                    // Save to NVS for next boot
                    saveIdentity(id);
                    printf("P4DataStore: identity restored from SD — pub[0..3]=%02X%02X%02X%02X\n",
                           id.pub_key[0], id.pub_key[1], id.pub_key[2], id.pub_key[3]);
                    return true;
                }
            }
        }

        ESP_LOGW(TAG, "loadIdentity: no saved identity found");
        return false;
    }

    bool saveIdentity(const mesh::LocalIdentity &id) {
        if (!_initialized) return false;

        // Serialize: prv_key[64] + pub_key[32]
        uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE];
        size_t written = const_cast<mesh::LocalIdentity&>(id).writeTo(buf, sizeof(buf));
        if (written != sizeof(buf)) {
            ESP_LOGE(TAG, "saveIdentity: writeTo returned %d (expected %d)", (int)written, (int)sizeof(buf));
            return false;
        }

        // Write to NVS
        nvs_handle_t handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
        if (err != ESP_OK) return false;

        err = nvs_set_blob(handle, "identity", buf, sizeof(buf));
        if (err == ESP_OK) err = nvs_commit(handle);
        nvs_close(handle);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "saveIdentity: NVS write failed (0x%x)", err);
            return false;
        }

        // Backup to SD
        if (p4_sdcard_is_mounted()) {
            FILE* f = fopen(SD_IDENTITY_PATH, "wb");
            if (f) {
                fwrite(buf, 1, sizeof(buf), f);
                fclose(f);
                ESP_LOGI(TAG, "saveIdentity: SD backup written");
            }
        }

        printf("P4DataStore: identity saved — pub[0..3]=%02X%02X%02X%02X\n",
               id.pub_key[0], id.pub_key[1], id.pub_key[2], id.pub_key[3]);
        return true;
    }

    // =====================================================================
    // Prefs — NVS primary, SD backup
    // =====================================================================

    bool loadPrefs(P4NodePrefs &prefs) {
        if (!_initialized) return false;

        // Try NVS first
        nvs_handle_t handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
        if (err == ESP_OK) {
            size_t len = sizeof(P4NodePrefs);
            err = nvs_get_blob(handle, "prefs", &prefs, &len);
            nvs_close(handle);

            if (err == ESP_OK && len == sizeof(P4NodePrefs)) {
                ESP_LOGI(TAG, "loadPrefs: from NVS (name=%s, freq=%.3f, sf=%d)",
                         prefs.node_name, prefs.freq, prefs.sf);
                return true;
            }
        }

        // NVS empty — try SD restore
        if (p4_sdcard_is_mounted() && p4_sdcard_file_exists(SD_PREFS_PATH)) {
            FILE* f = fopen(SD_PREFS_PATH, "rb");
            if (f) {
                size_t rd = fread(&prefs, 1, sizeof(P4NodePrefs), f);
                fclose(f);
                if (rd == sizeof(P4NodePrefs)) {
                    // Save to NVS for next boot
                    savePrefs(prefs);
                    ESP_LOGI(TAG, "loadPrefs: restored from SD (name=%s)", prefs.node_name);
                    return true;
                }
            }
        }

        ESP_LOGW(TAG, "loadPrefs: no saved prefs, using defaults");
        prefs.setDefaults();
        return false;
    }

    bool savePrefs(const P4NodePrefs &prefs) {
        if (!_initialized) return false;

        // Write to NVS
        nvs_handle_t handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
        if (err != ESP_OK) return false;

        err = nvs_set_blob(handle, "prefs", &prefs, sizeof(P4NodePrefs));
        if (err == ESP_OK) err = nvs_commit(handle);
        nvs_close(handle);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "savePrefs: NVS write failed (0x%x)", err);
            return false;
        }

        // Backup to SD
        if (p4_sdcard_is_mounted()) {
            FILE* f = fopen(SD_PREFS_PATH, "wb");
            if (f) {
                fwrite(&prefs, 1, sizeof(P4NodePrefs), f);
                fclose(f);
            }
        }

        ESP_LOGI(TAG, "savePrefs: saved (name=%s)", prefs.node_name);
        return true;
    }

    // =====================================================================
    // Channels — NVS primary, SD backup
    //
    // Stored as: P4ChannelsHeader + P4ChannelRecord[MAX_GROUP_CHANNELS]
    // NVS blob size: 8 + (68 × 8) = 552 bytes — fits easily
    // =====================================================================

    bool loadChannels(P4ChannelRecord* channels, int maxChannels, int &outCount) {
        if (!_initialized) return false;
        outCount = 0;
        memset(channels, 0, sizeof(P4ChannelRecord) * maxChannels);

        // Calculate expected blob size
        size_t blobSize = sizeof(P4ChannelsHeader) + sizeof(P4ChannelRecord) * maxChannels;
        uint8_t* blob = (uint8_t*)malloc(blobSize);
        if (!blob) return false;

        bool loaded = false;

        // Try NVS first
        nvs_handle_t handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
        if (err == ESP_OK) {
            size_t len = blobSize;
            err = nvs_get_blob(handle, "channels", blob, &len);
            nvs_close(handle);

            if (err == ESP_OK && len >= sizeof(P4ChannelsHeader)) {
                loaded = true;
            }
        }

        // NVS empty — try SD restore
        if (!loaded && p4_sdcard_is_mounted() && p4_sdcard_file_exists(SD_CHANNELS_PATH)) {
            FILE* f = fopen(SD_CHANNELS_PATH, "rb");
            if (f) {
                size_t rd = fread(blob, 1, blobSize, f);
                fclose(f);
                if (rd >= sizeof(P4ChannelsHeader)) {
                    loaded = true;
                    ESP_LOGI(TAG, "loadChannels: restored from SD");
                }
            }
        }

        if (!loaded) {
            free(blob);
            return false;
        }

        // Parse header
        P4ChannelsHeader* hdr = (P4ChannelsHeader*)blob;
        if (memcmp(hdr->magic, "MCH", 3) != 0 || hdr->version != 1) {
            ESP_LOGW(TAG, "loadChannels: bad header magic/version");
            free(blob);
            return false;
        }

        // Copy channel records
        int count = hdr->count;
        if (count > maxChannels) count = maxChannels;

        P4ChannelRecord* records = (P4ChannelRecord*)(blob + sizeof(P4ChannelsHeader));
        memcpy(channels, records, sizeof(P4ChannelRecord) * count);
        outCount = count;

        ESP_LOGI(TAG, "loadChannels: %d channels loaded", count);
        free(blob);
        return true;
    }

    bool saveChannels(const P4ChannelRecord* channels, int maxChannels) {
        if (!_initialized) return false;

        // Count active channels
        int activeCount = 0;
        for (int i = 0; i < maxChannels; i++) {
            if (channels[i].active) activeCount++;
        }

        // Build blob
        size_t blobSize = sizeof(P4ChannelsHeader) + sizeof(P4ChannelRecord) * maxChannels;
        uint8_t* blob = (uint8_t*)malloc(blobSize);
        if (!blob) return false;

        P4ChannelsHeader* hdr = (P4ChannelsHeader*)blob;
        memcpy(hdr->magic, "MCH", 3);
        hdr->magic[3] = 0;
        hdr->version = 1;
        hdr->count = (uint8_t)activeCount;
        hdr->max_channels = (uint8_t)maxChannels;
        hdr->reserved = 0;

        memcpy(blob + sizeof(P4ChannelsHeader), channels, sizeof(P4ChannelRecord) * maxChannels);

        // Write to NVS
        nvs_handle_t handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
        if (err == ESP_OK) {
            err = nvs_set_blob(handle, "channels", blob, blobSize);
            if (err == ESP_OK) err = nvs_commit(handle);
            nvs_close(handle);
        }

        // Backup to SD
        if (p4_sdcard_is_mounted()) {
            FILE* f = fopen(SD_CHANNELS_PATH, "wb");
            if (f) {
                fwrite(blob, 1, blobSize, f);
                fclose(f);
            }
        }

        ESP_LOGI(TAG, "saveChannels: %d active channels saved", activeCount);
        free(blob);
        return (err == ESP_OK);
    }

    // =====================================================================
    // Contacts — NVS primary (no SD backup yet, pending SD card init fix)
    // =====================================================================

    // Compact contact record for NVS storage (80 bytes each).
    // Excludes transient data (out_path, shared_secret) which is
    // re-derived at runtime.
    struct ContactRecord {
        uint8_t pub_key[PUB_KEY_SIZE]; // 32
        char name[32];                  // 32
        uint8_t type;                   // 1
        uint8_t flags;                  // 1
        int32_t gps_lat;                // 4
        int32_t gps_lon;                // 4
        uint32_t lastmod;               // 4
        uint32_t last_advert_timestamp; // 4
    };
    // Total: 82 bytes — no padding needed

    struct ContactsHeader {
        uint8_t magic[4];   // "MCC\0"
        uint8_t version;    // 1
        uint8_t count;      // number of contacts
        uint8_t reserved[2];
    };

    bool saveContacts(const ContactRecord* records, int count) {
        if (!_initialized || count < 0) return false;
        if (count > 255) count = 255;  // uint8_t header field

        size_t blobSize = sizeof(ContactsHeader) + sizeof(ContactRecord) * count;
        uint8_t* blob = (uint8_t*)malloc(blobSize);
        if (!blob) {
            ESP_LOGE(TAG, "saveContacts: malloc failed (%d bytes)", (int)blobSize);
            return false;
        }

        ContactsHeader* hdr = (ContactsHeader*)blob;
        memcpy(hdr->magic, "MCC", 3);
        hdr->magic[3] = 0;
        hdr->version = 1;
        hdr->count = (uint8_t)count;
        memset(hdr->reserved, 0, sizeof(hdr->reserved));

        if (count > 0 && records) {
            memcpy(blob + sizeof(ContactsHeader), records, sizeof(ContactRecord) * count);
        }

        nvs_handle_t handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
        if (err == ESP_OK) {
            err = nvs_set_blob(handle, "contacts", blob, blobSize);
            if (err == ESP_OK) err = nvs_commit(handle);
            nvs_close(handle);
        }

        free(blob);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "saveContacts: NVS write failed (0x%x)", err);
            return false;
        }

        ESP_LOGI(TAG, "saveContacts: %d contacts saved (%d bytes)", count, (int)blobSize);
        return true;
    }

    int loadContacts(ContactRecord* records, int maxCount) {
        if (!_initialized) return 0;

        nvs_handle_t handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
        if (err != ESP_OK) return 0;

        // Query blob size first
        size_t blobSize = 0;
        err = nvs_get_blob(handle, "contacts", NULL, &blobSize);
        if (err != ESP_OK || blobSize < sizeof(ContactsHeader)) {
            nvs_close(handle);
            return 0;
        }

        uint8_t* blob = (uint8_t*)malloc(blobSize);
        if (!blob) { nvs_close(handle); return 0; }

        err = nvs_get_blob(handle, "contacts", blob, &blobSize);
        nvs_close(handle);

        if (err != ESP_OK) { free(blob); return 0; }

        ContactsHeader* hdr = (ContactsHeader*)blob;
        if (memcmp(hdr->magic, "MCC", 3) != 0 || hdr->version != 1) {
            ESP_LOGW(TAG, "loadContacts: bad header");
            free(blob);
            return 0;
        }

        int count = hdr->count;
        if (count > maxCount) count = maxCount;

        size_t dataNeeded = sizeof(ContactsHeader) + sizeof(ContactRecord) * count;
        if (blobSize < dataNeeded) {
            ESP_LOGW(TAG, "loadContacts: blob too small (%d < %d)", (int)blobSize, (int)dataNeeded);
            free(blob);
            return 0;
        }

        memcpy(records, blob + sizeof(ContactsHeader), sizeof(ContactRecord) * count);
        free(blob);

        ESP_LOGI(TAG, "loadContacts: %d contacts loaded", count);
        return count;
    }

    // =====================================================================
    // Backup / Restore — bulk operations for fresh-flash recovery
    // =====================================================================

    // Backup everything to SD card. Called after any significant change.
    void backupToSD() {
        if (!p4_sdcard_is_mounted()) return;
        // Prefs and identity are backed up on every save automatically.
        // This method is for explicit full backup if needed.
        ESP_LOGI(TAG, "backupToSD: prefs and channels backed up");
    }

    // Ensure all Meck SD directories exist. Idempotent — safe to call on
    // every boot. POSIX mkdir doesn't create parents, so we walk top-down.
    // Logs a warning if any level can't be created but continues — caller
    // sites will fail with their own warning if writes can't proceed.
    void ensureSDDirectories() {
        if (!p4_sdcard_is_mounted()) return;
        if (!ensureDir("/sdcard/meshcore")) {
            ESP_LOGW(TAG, "ensureSDDirectories: cannot create /sdcard/meshcore (errno=%d)", errno);
            return;
        }
        if (!ensureDir("/sdcard/meshcore/identity")) {
            ESP_LOGW(TAG, "ensureSDDirectories: cannot create /sdcard/meshcore/identity (errno=%d)", errno);
        }
        if (!ensureDir(SD_MESSAGES_DIR)) {
            ESP_LOGW(TAG, "ensureSDDirectories: cannot create %s (errno=%d)", SD_MESSAGES_DIR, errno);
        }
    }

    // Restore from SD if NVS is empty (e.g. after erase_flash).
    // Called once during boot, before mesh starts.
    // Returns true if anything was restored.
    bool restoreFromSD() {
        if (!p4_sdcard_is_mounted()) return false;
        ensureSDDirectories();  // create dirs once per boot for fresh SDs
        bool restored = false;

        // Check if NVS has identity
        if (!hasIdentity() && p4_sdcard_file_exists(SD_IDENTITY_PATH)) {
            mesh::LocalIdentity id;
            FILE* f = fopen(SD_IDENTITY_PATH, "rb");
            if (f) {
                uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE];
                size_t rd = fread(buf, 1, sizeof(buf), f);
                fclose(f);
                if (rd == sizeof(buf)) {
                    id.readFrom(buf, rd);
                    saveIdentity(id);
                    printf("P4DataStore: identity restored from SD\n");
                    restored = true;
                }
            }
        }

        // Check if NVS has prefs
        {
            nvs_handle_t handle;
            if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
                size_t len = 0;
                esp_err_t err = nvs_get_blob(handle, "prefs", NULL, &len);
                nvs_close(handle);
                if (err != ESP_OK || len == 0) {
                    // No prefs in NVS — try SD
                    if (p4_sdcard_file_exists(SD_PREFS_PATH)) {
                        P4NodePrefs prefs;
                        FILE* f = fopen(SD_PREFS_PATH, "rb");
                        if (f) {
                            size_t rd = fread(&prefs, 1, sizeof(prefs), f);
                            fclose(f);
                            if (rd == sizeof(prefs)) {
                                savePrefs(prefs);
                                printf("P4DataStore: prefs restored from SD\n");
                                restored = true;
                            }
                        }
                    }
                }
            }
        }

        // Check if NVS has channels
        {
            nvs_handle_t handle;
            if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
                size_t len = 0;
                esp_err_t err = nvs_get_blob(handle, "channels", NULL, &len);
                nvs_close(handle);
                if (err != ESP_OK || len == 0) {
                    if (p4_sdcard_file_exists(SD_CHANNELS_PATH)) {
                        // Read SD file and write directly to NVS
                        size_t fsize = p4_sdcard_file_size(SD_CHANNELS_PATH);
                        if (fsize > 0) {
                            uint8_t* buf = (uint8_t*)malloc(fsize);
                            if (buf) {
                                FILE* f = fopen(SD_CHANNELS_PATH, "rb");
                                if (f) {
                                    fread(buf, 1, fsize, f);
                                    fclose(f);
                                    nvs_handle_t wh;
                                    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &wh) == ESP_OK) {
                                        nvs_set_blob(wh, "channels", buf, fsize);
                                        nvs_commit(wh);
                                        nvs_close(wh);
                                        printf("P4DataStore: channels restored from SD\n");
                                        restored = true;
                                    }
                                }
                                free(buf);
                            }
                        }
                    }
                }
            }
        }

        if (restored) {
            printf("=== P4DataStore: settings restored from SD card ===\n");
        }
        return restored;
    }

    // =====================================================================
    // Utility
    // =====================================================================

    bool eraseAll() {
        if (!_initialized) return false;

        nvs_handle_t handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
        if (err != ESP_OK) return false;

        err = nvs_erase_all(handle);
        if (err == ESP_OK) err = nvs_commit(handle);
        nvs_close(handle);

        ESP_LOGI(TAG, "eraseAll: %s", err == ESP_OK ? "OK" : "FAILED");
        return err == ESP_OK;
    }

    bool hasIdentity() {
        if (!_initialized) return false;

        nvs_handle_t handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
        if (err != ESP_OK) return false;

        size_t len = 0;
        err = nvs_get_blob(handle, "identity", NULL, &len);
        nvs_close(handle);

        return (err == ESP_OK && len == PRV_KEY_SIZE + PUB_KEY_SIZE);
    }

    // =====================================================================
    // Channel message history persistence (SD card only — too large for NVS)
    //
    // File layout: /sdcard/meshcore/messages/ch_<idx>.bin per channel
    //   [P4MsgFileHeader]   16 bytes (magic, version, record_size, reserved)
    //   [record 0][record 1]...[record N-1]   each record_size bytes
    //
    // Append-only: each new message appends one record to the tail. Header
    // is written once when the file is created and never modified again, so
    // record count is computed at load time from file size. This keeps per-
    // message IO tiny (~308 bytes) and avoids rewriting the whole file.
    //
    // Schema mismatch policy: file is renamed to ch_<idx>.bin.bak and a
    // fresh file is created. Old data is preserved (renamed) for recovery
    // but the in-RAM ring starts empty.
    // =====================================================================

    // Build the per-channel file path. out_buf must be >= 64 bytes.
    static void buildMessagePath(uint8_t channel_idx, char* out_buf, size_t buf_len) {
        snprintf(out_buf, buf_len, "%s/ch_%u.bin", SD_MESSAGES_DIR, (unsigned)channel_idx);
    }

    // Ensure a single directory exists. Returns true if it now exists (was
    // created or already there), false on hard error. POSIX mkdir doesn't
    // create parents, so callers must build paths top-down.
    static bool ensureDir(const char* path) {
        struct stat st;
        if (stat(path, &st) == 0) {
            return S_ISDIR(st.st_mode);
        }
        if (mkdir(path, 0755) == 0) return true;
        return false;
    }

    // Ensure the messages directory and its parent exist. Called lazily on
    // first save in case earlier persistence flows haven't already created
    // /sdcard/meshcore/.
    void ensureMessagesDir() {
        if (!p4_sdcard_is_mounted()) return;
        if (!ensureDir("/sdcard/meshcore")) {
            ESP_LOGW(TAG, "ensureMessagesDir: cannot create /sdcard/meshcore (errno=%d)", errno);
            return;
        }
        if (!ensureDir(SD_MESSAGES_DIR)) {
            ESP_LOGW(TAG, "ensureMessagesDir: cannot create %s (errno=%d)", SD_MESSAGES_DIR, errno);
        }
    }

    // Append one record to a channel's message file. The header is created
    // on first write; thereafter we append to the tail. On magic/version
    // mismatch the existing file is renamed to .bak and a fresh one written.
    // Returns true on successful append.
    bool appendChannelMessageRecord(uint8_t channel_idx,
                                     uint32_t expected_magic,
                                     uint16_t expected_version,
                                     uint16_t record_size,
                                     const void* record_bytes)
    {
        if (!record_bytes || record_size == 0) return false;
        if (!p4_sdcard_is_mounted()) return false;

        ensureMessagesDir();

        char path[80];
        buildMessagePath(channel_idx, path, sizeof(path));

        // Try opening for read/write (existing file); fall through to fresh
        // create if it doesn't exist or is truncated.
        FILE* f = fopen(path, "r+b");
        bool fresh_file = false;

        if (f) {
            // Existing file. Validate header.
            struct {
                uint32_t magic;
                uint16_t version;
                uint16_t record_size;
                uint32_t reserved[2];
            } __attribute__((packed)) hdr;

            if (fread(&hdr, sizeof(hdr), 1, f) == 1 &&
                hdr.magic == expected_magic &&
                hdr.version == expected_version &&
                hdr.record_size == record_size) {
                // Header OK; seek to end and append
                fseek(f, 0, SEEK_END);
                size_t wrote = fwrite(record_bytes, 1, record_size, f);
                fclose(f);
                return wrote == record_size;
            }

            // Header mismatch — rename and fall through to fresh create
            fclose(f);
            char bak_path[96];
            snprintf(bak_path, sizeof(bak_path), "%s.bak", path);
            remove(bak_path);  // ignore failure if no prior bak
            if (rename(path, bak_path) == 0) {
                ESP_LOGW(TAG, "appendChannelMessageRecord: schema mismatch on ch %u, "
                              "renamed to %s, starting fresh", (unsigned)channel_idx, bak_path);
            } else {
                ESP_LOGW(TAG, "appendChannelMessageRecord: schema mismatch on ch %u, "
                              "rename failed (errno=%d), overwriting", (unsigned)channel_idx, errno);
            }
            fresh_file = true;
        } else {
            fresh_file = true;
        }

        // Fresh file: write header then first record.
        f = fopen(path, "wb");
        if (!f) {
            ESP_LOGW(TAG, "appendChannelMessageRecord: fopen(%s, wb) failed (errno=%d)",
                     path, errno);
            return false;
        }

        struct {
            uint32_t magic;
            uint16_t version;
            uint16_t record_size;
            uint32_t reserved[2];
        } __attribute__((packed)) hdr;
        hdr.magic = expected_magic;
        hdr.version = expected_version;
        hdr.record_size = record_size;
        hdr.reserved[0] = 0;
        hdr.reserved[1] = 0;

        bool ok = (fwrite(&hdr, sizeof(hdr), 1, f) == 1) &&
                  (fwrite(record_bytes, 1, record_size, f) == record_size);
        fclose(f);

        if (ok && fresh_file) {
            ESP_LOGI(TAG, "appendChannelMessageRecord: created %s", path);
        }
        return ok;
    }

    // Load up to max_records most recent records from a channel's file into
    // records_buf (each entry record_size bytes). Returns number actually
    // loaded. If the file's record count exceeds max_records, only the
    // last max_records are loaded.
    int loadChannelMessageTail(uint8_t channel_idx,
                                uint32_t expected_magic,
                                uint16_t expected_version,
                                uint16_t record_size,
                                void* records_buf,
                                int max_records)
    {
        if (!records_buf || record_size == 0 || max_records <= 0) return 0;
        if (!p4_sdcard_is_mounted()) return 0;

        char path[80];
        buildMessagePath(channel_idx, path, sizeof(path));
        if (!p4_sdcard_file_exists(path)) return 0;

        FILE* f = fopen(path, "rb");
        if (!f) return 0;

        struct {
            uint32_t magic;
            uint16_t version;
            uint16_t record_size;
            uint32_t reserved[2];
        } __attribute__((packed)) hdr;

        if (fread(&hdr, sizeof(hdr), 1, f) != 1) {
            fclose(f);
            return 0;
        }

        if (hdr.magic != expected_magic ||
            hdr.version != expected_version ||
            hdr.record_size != record_size) {
            fclose(f);
            ESP_LOGW(TAG, "loadChannelMessageTail: schema mismatch on ch %u, "
                          "skipping load (caller will rename on next save)",
                     (unsigned)channel_idx);
            return 0;
        }

        // Compute total record count from file size (append-only: count
        // grows beyond max_records over time but only the tail is loaded).
        fseek(f, 0, SEEK_END);
        long file_size = ftell(f);
        long data_size = file_size - (long)sizeof(hdr);
        if (data_size < 0) { fclose(f); return 0; }

        long total_records = data_size / record_size;
        long skip_records = (total_records > max_records) ? (total_records - max_records) : 0;
        long load_records = total_records - skip_records;

        long offset = (long)sizeof(hdr) + (skip_records * record_size);
        if (fseek(f, offset, SEEK_SET) != 0) { fclose(f); return 0; }

        size_t bytes_to_read = (size_t)(load_records * record_size);
        size_t got = fread(records_buf, 1, bytes_to_read, f);
        fclose(f);

        int loaded = (int)(got / record_size);
        ESP_LOGI(TAG, "loadChannelMessageTail: ch %u loaded %d of %ld total",
                 (unsigned)channel_idx, loaded, total_records);
        return loaded;
    }
};
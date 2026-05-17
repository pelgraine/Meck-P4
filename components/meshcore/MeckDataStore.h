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
 *     "contacts"     — DEPRECATED. Kept for one-shot migration on the first
 *                      boot after upgrading from NVS-contacts storage. The
 *                      blob is read into memory and rewritten to SD, then
 *                      the key is erased to free NVS space. Subsequent
 *                      saves are SD-only.
 *
 *   SD card (/sdcard/meshcore/):
 *     prefs.bin             — binary backup of P4NodePrefs
 *     channels.bin          — binary backup of channel data
 *     identity/_main.id     — 96-byte identity backup
 *     contacts.bin          — PRIMARY contact store (v2 header, uint16_t count,
 *                             written atomically via .tmp + rename)
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
    // Screen control. Both fields use 0 as "not set / use default" so
    // existing NVS blobs (which had reserved[] zeroed) keep working —
    // setDefaults below maps 0 → a sane initial value at first boot.
    uint8_t screen_brightness;    // 0 = default (200), else 1..255
    uint8_t screen_off_minutes;   // 0 = never, else minutes until dim
    // GPS power gate. 0 = use default (on); 1 = explicitly on; 2 = off.
    // The 3-state encoding lets us distinguish "user has actively turned
    // GPS off" from "fresh prefs, never set" — useful if we ever change
    // the default. Existing NVS blobs read as 0 → default-on, no surprise.
    uint8_t gps_enabled;
    // Keyboard preferences. Both fields default to 0, which is what
    // existing NVS blobs read as (previously this was reserved[2]). So
    // existing installs come up with Meck's dark keyboard theme + QWERTY
    // layout on first boot after the upgrade, no migration step needed.
    uint8_t kb_dark_mode;         // 0 = dark (default), 1 = light
    uint8_t kb_layout;            // 0 = QWERTY (default), 1 = AZERTY, 2 = QWERTZ
    uint8_t font_scale;           // 0 = Classic, 1 = Larger

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
        screen_brightness = 200;
        screen_off_minutes = 5;
        gps_enabled = 1;        // on by default
        kb_dark_mode = 0;       // dark theme (matches the rest of Meck's UI)
        kb_layout = 0;          // QWERTY
        font_scale = 0;         // Classic
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

    // Deferred SD-backup state for identity. If loadIdentity loads from
    // NVS but the SD card isn't mounted yet (or the write fails for any
    // other reason), the identity blob is cached here and the flag is
    // raised. Meck::loop() calls ensureIdentityOnSD() periodically, which
    // retries the write and clears the flag on success. This closes the
    // race between SD mount and Meck::begin()'s loadIdentity call, which
    // otherwise leaves a node with identity in NVS only — vulnerable to
    // a future reflash regenerating a fresh pub key.
    bool _identity_sd_backup_pending = false;
    uint8_t _pending_identity_buf[PRV_KEY_SIZE + PUB_KEY_SIZE] = {0};

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

        // SD-first policy. SD is the authoritative identity backup, NVS
        // is the secondary cache. On boot:
        //   1. Try SD first. If found, load it AND mirror to NVS so the
        //      two sources agree.
        //   2. If SD has nothing, fall back to NVS. If found, load it
        //      AND immediately back it up to SD so the SD copy exists
        //      from this boot onward.
        //   3. If neither has anything, return false and let the caller
        //      generate a new identity (which then gets saved to both
        //      via saveIdentity).
        //
        // Rationale: NVS can be wiped or regenerated by accident — a
        // failed save, a partition-table change, a low-level erase. If
        // SD always wins, the device's identity survives those events
        // and a reflash without erasing the SD card preserves the same
        // pub key. The user's contacts on other nodes don't have to
        // re-add them.

        // -- Step 1: SD --
        if (p4_sdcard_is_mounted() && p4_sdcard_file_exists(SD_IDENTITY_PATH)) {
            FILE* f = fopen(SD_IDENTITY_PATH, "rb");
            if (f) {
                uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE];
                size_t rd = fread(buf, 1, sizeof(buf), f);
                fclose(f);
                if (rd == sizeof(buf)) {
                    id.readFrom(buf, rd);
                    printf("P4DataStore: identity loaded from SD — pub[0..3]=%02X%02X%02X%02X\n",
                           id.pub_key[0], id.pub_key[1], id.pub_key[2], id.pub_key[3]);
                    // Mirror to NVS unconditionally — if NVS already
                    // holds the same identity this is a no-op rewrite;
                    // if NVS was empty or holds a different (likely
                    // regenerated) identity, this realigns it with the
                    // SD-authoritative copy.
                    saveIdentityToNVS(id);
                    _identity_sd_backup_pending = false;  // SD copy confirmed
                    return true;
                }
                ESP_LOGW(TAG, "loadIdentity: SD identity file size mismatch (%u != %u)",
                         (unsigned)rd, (unsigned)sizeof(buf));
            }
        }

        // -- Step 2: NVS fallback --
        nvs_handle_t handle;
        if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
            uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE];
            size_t len = sizeof(buf);
            esp_err_t err = nvs_get_blob(handle, "identity", buf, &len);
            nvs_close(handle);

            if (err == ESP_OK && len == sizeof(buf)) {
                id.readFrom(buf, len);
                printf("P4DataStore: identity loaded from NVS — pub[0..3]=%02X%02X%02X%02X\n",
                       id.pub_key[0], id.pub_key[1], id.pub_key[2], id.pub_key[3]);
                // Promote to SD as the authoritative backup. If SD isn't
                // mounted yet, queue a retry so Meck::loop()'s periodic
                // ensureIdentityOnSD() call can complete the backup once
                // the card is ready. This closes the window where a
                // device boots with NVS identity but the SD mount hasn't
                // finished — without the retry, the SD backup would
                // never happen and a future reflash would orphan the
                // identity.
                if (saveIdentityToSD(id)) {
                    printf("P4DataStore: identity backed up to SD from NVS\n");
                    _identity_sd_backup_pending = false;
                } else {
                    printf("P4DataStore: SD not ready at identity load, "
                           "backup deferred (will retry from main loop)\n");
                    memcpy(_pending_identity_buf, buf, sizeof(buf));
                    _identity_sd_backup_pending = true;
                }
                return true;
            }
        }

        // -- Step 3: neither source has anything --
        ESP_LOGW(TAG, "loadIdentity: no saved identity found (caller will generate new)");
        return false;
    }

    // Retry the deferred SD backup of the identity if loadIdentity took
    // the NVS-fallback path while the SD card wasn't mounted yet.
    // Idempotent: returns true the moment the SD copy exists (or
    // succeeds at writing it), false otherwise. Called periodically from
    // Meck::loop() until it returns true.
    //
    // Also covers the SD-inserted-later case: if the user boots without
    // an SD card and inserts one minutes later, the next loop tick
    // catches the now-mounted card and writes the backup.
    bool ensureIdentityOnSD() {
        if (!_identity_sd_backup_pending) return true;  // nothing to do
        if (!p4_sdcard_is_mounted()) return false;       // try again later

        // Rebuild the LocalIdentity from the cached blob and write it.
        // Holding the bytes (rather than a LocalIdentity reference) keeps
        // ownership in the data store and avoids any lifetime concern if
        // the caller's local variable goes out of scope between
        // loadIdentity and the retry.
        mesh::LocalIdentity id;
        id.readFrom(_pending_identity_buf, PRV_KEY_SIZE + PUB_KEY_SIZE);
        if (saveIdentityToSD(id)) {
            printf("P4DataStore: deferred identity SD backup succeeded — "
                   "pub[0..3]=%02X%02X%02X%02X\n",
                   id.pub_key[0], id.pub_key[1], id.pub_key[2], id.pub_key[3]);
            _identity_sd_backup_pending = false;
            // Wipe the cached blob now that it's on disk; no reason to
            // keep the private key sitting around in RAM as a copy.
            memset(_pending_identity_buf, 0, sizeof(_pending_identity_buf));
            return true;
        }
        return false;
    }

    // ---- Internal helpers split out from saveIdentity ----
    //
    // Splitting the per-destination writes makes the SD-first load path
    // possible: when loadIdentity finds an identity on SD, we want to
    // mirror to NVS without touching SD; when loadIdentity finds an
    // identity in NVS, we want to back up to SD without rewriting NVS.

private:
    bool saveIdentityToNVS(const mesh::LocalIdentity &id) {
        uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE];
        size_t written = const_cast<mesh::LocalIdentity&>(id).writeTo(buf, sizeof(buf));
        if (written != sizeof(buf)) return false;

        nvs_handle_t handle;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return false;
        esp_err_t err = nvs_set_blob(handle, "identity", buf, sizeof(buf));
        if (err == ESP_OK) err = nvs_commit(handle);
        nvs_close(handle);
        return err == ESP_OK;
    }

    bool saveIdentityToSD(const mesh::LocalIdentity &id) {
        if (!p4_sdcard_is_mounted()) return false;
        if (!ensureDir("/sdcard/meshcore/identity")) return false;

        uint8_t buf[PRV_KEY_SIZE + PUB_KEY_SIZE];
        size_t written = const_cast<mesh::LocalIdentity&>(id).writeTo(buf, sizeof(buf));
        if (written != sizeof(buf)) return false;

        FILE* f = fopen(SD_IDENTITY_PATH, "wb");
        if (!f) return false;
        size_t w = fwrite(buf, 1, sizeof(buf), f);
        fclose(f);
        return w == sizeof(buf);
    }

public:

    bool saveIdentity(const mesh::LocalIdentity &id) {
        if (!_initialized) return false;

        // SD first (authoritative backup), NVS second. If the SD write
        // fails (e.g. card not mounted or full), we still attempt the NVS
        // write so the identity is at least in volatile-but-fast storage;
        // the next saveIdentity will retry the SD write.
        bool sd_ok = saveIdentityToSD(id);
        if (sd_ok) {
            ESP_LOGI(TAG, "saveIdentity: SD backup written");
        } else {
            ESP_LOGW(TAG, "saveIdentity: SD backup not written "
                          "(card not mounted or write failed)");
        }

        bool nvs_ok = saveIdentityToNVS(id);
        if (!nvs_ok) {
            ESP_LOGE(TAG, "saveIdentity: NVS write failed");
        }

        if (sd_ok || nvs_ok) {
            printf("P4DataStore: identity saved — pub[0..3]=%02X%02X%02X%02X "
                   "(SD=%s, NVS=%s)\n",
                   id.pub_key[0], id.pub_key[1], id.pub_key[2], id.pub_key[3],
                   sd_ok ? "ok" : "fail", nvs_ok ? "ok" : "fail");
        }
        // Treat success as "at least one of the two backed up".
        return sd_ok || nvs_ok;
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
            // Older firmware may have stored a shorter struct (before fields
            // like font_scale were added). Zero-fill prefs, then read using
            // the on-disk blob size so the missing tail comes up as zero —
            // which setDefaults already maps to sensible values for every
            // field. Avoids wiping users' saved prefs on every struct grow.
            memset(&prefs, 0, sizeof(P4NodePrefs));
            size_t len = 0;
            err = nvs_get_blob(handle, "prefs", NULL, &len);
            if (err == ESP_OK && len > 0 && len <= sizeof(P4NodePrefs)) {
                err = nvs_get_blob(handle, "prefs", &prefs, &len);
            }
            nvs_close(handle);

            if (err == ESP_OK && len > 0 && len <= sizeof(P4NodePrefs)) {
                ESP_LOGI(TAG, "loadPrefs: from NVS (name=%s, freq=%.3f, sf=%d)",
                         prefs.node_name, prefs.freq, prefs.sf);
                return true;
            }
        }

        // NVS empty — try SD restore
        if (p4_sdcard_is_mounted() && p4_sdcard_file_exists(SD_PREFS_PATH)) {
            FILE* f = fopen(SD_PREFS_PATH, "rb");
            if (f) {
                // Same shorter-blob tolerance as the NVS path above.
                memset(&prefs, 0, sizeof(P4NodePrefs));
                size_t rd = fread(&prefs, 1, sizeof(P4NodePrefs), f);
                fclose(f);
                if (rd > 0 && rd <= sizeof(P4NodePrefs)) {
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
    // Contacts — SD card primary (v2), NVS legacy fallback (v1)
    //
    // Contacts outgrew the 24 KB NVS partition once we crossed ~160 entries
    // (each ContactRecord is 82 bytes; NVS also keeps an old-blob copy
    // during atomic writes so effective capacity is roughly half).
    // Moving to SD removes the cap entirely.
    //
    // File format (contacts.bin):
    //   [ContactsHeaderV2]  8 bytes — magic "MCC\0", version 2, uint16_t count
    //   [ContactRecord 0][ContactRecord 1]…[ContactRecord N-1]
    //
    // Atomic save: writes to contacts.bin.tmp first, then rename() over
    // contacts.bin. POSIX rename within the same directory is atomic on
    // FAT, so a power loss either leaves the old file intact or the new
    // file fully written — never a truncated mess.
    //
    // Migration: on first load after upgrade, if SD is empty but NVS has
    // a legacy "contacts" v1 blob, we read it from NVS, write it to SD,
    // and erase the NVS key. The next save is SD-only.
    // =====================================================================

    // Compact contact record for persistence (82 bytes each).
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

    // Legacy v1 header — kept for one-shot migration of existing NVS blobs.
    struct ContactsHeader {
        uint8_t magic[4];   // "MCC\0"
        uint8_t version;    // 1
        uint8_t count;      // uint8_t (cap 255)
        uint8_t reserved[2];
    };

    // Current v2 header — used by all new writes. Same total size (8 bytes)
    // as v1, just with the count field promoted to uint16_t. Layout uses
    // explicit bytes via __attribute__((packed)) so the on-disk format is
    // platform-independent (RISC-V is little-endian so memcpy works for
    // both fields; the packed attribute just prevents any compiler from
    // adding padding around `count`).
    struct __attribute__((packed)) ContactsHeaderV2 {
        uint8_t magic[4];   // "MCC\0"
        uint8_t version;    // 2
        uint8_t flags;      // reserved, 0
        uint16_t count;     // uint16_t (cap 65535)
    };

    // Atomic write helper: write `data` of length `len` to `final_path` by
    // first writing to `final_path.tmp`, fsync (best-effort), then rename.
    // Returns true on success. Logs but doesn't return errors for unlink
    // of stale tmp (cleanup is best-effort).
    bool atomicWriteToSD(const char* final_path, const void* data, size_t len) {
        char tmp_path[96];
        snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", final_path);

        FILE* f = fopen(tmp_path, "wb");
        if (!f) {
            ESP_LOGW(TAG, "atomicWriteToSD: fopen(%s, wb) failed (errno=%d)",
                     tmp_path, errno);
            return false;
        }
        size_t wrote = fwrite(data, 1, len, f);
        int closed = fclose(f);
        if (wrote != len || closed != 0) {
            ESP_LOGW(TAG, "atomicWriteToSD: write/close failed "
                          "(wrote=%d, expected=%d, close=%d)",
                     (int)wrote, (int)len, closed);
            unlink(tmp_path);
            return false;
        }

        // rename on POSIX overwrites; rename on FAT does not (errno 17
        // EEXIST). ESP-IDF's FATFS follows the FAT semantics, so we need
        // to unlink the destination first. This breaks true atomicity:
        // there's a microsecond window where neither file exists. If
        // power is lost in that window, contacts.bin.tmp survives and
        // loadContacts() will pick it up via the recovery path.
        unlink(final_path);  // ignore error: file may not exist on first save

        if (rename(tmp_path, final_path) != 0) {
            ESP_LOGW(TAG, "atomicWriteToSD: rename(%s, %s) failed (errno=%d)",
                     tmp_path, final_path, errno);
            unlink(tmp_path);
            return false;
        }
        return true;
    }

    bool saveContacts(const ContactRecord* records, int count) {
        if (!_initialized || count < 0) return false;
        if (count > 65535) count = 65535;  // uint16_t header field

        if (!p4_sdcard_is_mounted()) {
            ESP_LOGE(TAG, "saveContacts: SD not mounted — contacts not persisted");
            return false;
        }
        ensureSDDirectories();

        size_t blobSize = sizeof(ContactsHeaderV2) + sizeof(ContactRecord) * count;
        uint8_t* blob = (uint8_t*)malloc(blobSize);
        if (!blob) {
            ESP_LOGE(TAG, "saveContacts: malloc failed (%d bytes)", (int)blobSize);
            return false;
        }

        ContactsHeaderV2* hdr = (ContactsHeaderV2*)blob;
        memcpy(hdr->magic, "MCC", 3);
        hdr->magic[3] = 0;
        hdr->version = 2;
        hdr->flags = 0;
        hdr->count = (uint16_t)count;

        if (count > 0 && records) {
            memcpy(blob + sizeof(ContactsHeaderV2), records,
                   sizeof(ContactRecord) * count);
        }

        bool ok = atomicWriteToSD(SD_CONTACTS_PATH, blob, blobSize);
        free(blob);

        if (!ok) {
            ESP_LOGE(TAG, "saveContacts: SD write failed");
            return false;
        }

        // One-shot NVS cleanup: if a legacy contacts blob is still sitting
        // in NVS, erase it now that we have a successful SD copy. This
        // frees up partition space and prevents the next failed write at
        // the size limit. nvs_erase_key is a no-op if the key is absent.
        {
            nvs_handle_t handle;
            if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
                esp_err_t derr = nvs_erase_key(handle, "contacts");
                if (derr == ESP_OK) {
                    nvs_commit(handle);
                    ESP_LOGI(TAG, "saveContacts: erased legacy NVS contacts blob");
                }
                // ESP_ERR_NVS_NOT_FOUND is the steady-state case — silent.
                nvs_close(handle);
            }
        }

        ESP_LOGI(TAG, "saveContacts: %d contacts saved to SD (%d bytes)",
                 count, (int)blobSize);
        return true;
    }

    // Parse a contacts blob and copy records into the caller's buffer.
    // Handles both v1 (uint8_t count) and v2 (uint16_t count) headers.
    // Returns number of records copied; 0 on header/format error.
    int parseContactsBlob(const uint8_t* blob, size_t blobSize,
                          ContactRecord* records, int maxCount) {
        if (!blob || blobSize < 8) return 0;
        if (memcmp(blob, "MCC", 3) != 0) {
            ESP_LOGW(TAG, "parseContactsBlob: bad magic");
            return 0;
        }
        uint8_t version = blob[4];
        int count = 0;
        size_t header_size = 0;

        if (version == 1) {
            // v1: byte 5 is uint8_t count
            count = blob[5];
            header_size = sizeof(ContactsHeader);
        } else if (version == 2) {
            // v2: bytes 6-7 are little-endian uint16_t count
            count = (int)blob[6] | ((int)blob[7] << 8);
            header_size = sizeof(ContactsHeaderV2);
        } else {
            ESP_LOGW(TAG, "parseContactsBlob: unsupported version %u",
                     (unsigned)version);
            return 0;
        }

        if (count > maxCount) count = maxCount;
        size_t need = header_size + (size_t)count * sizeof(ContactRecord);
        if (blobSize < need) {
            ESP_LOGW(TAG, "parseContactsBlob: blob too small (%d < %d)",
                     (int)blobSize, (int)need);
            return 0;
        }
        memcpy(records, blob + header_size,
               sizeof(ContactRecord) * count);
        return count;
    }

    int loadContacts(ContactRecord* records, int maxCount) {
        if (!_initialized) return 0;

        // Recovery: if a prior save was interrupted between unlink(.bin)
        // and rename(.tmp → .bin), only the .tmp file exists. Promote it
        // before the normal load path runs. This is silent in the happy
        // case (no .tmp present, nothing to do).
        if (p4_sdcard_is_mounted()) {
            char tmp_path[96];
            snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", SD_CONTACTS_PATH);
            if (p4_sdcard_file_exists(tmp_path) &&
                !p4_sdcard_file_exists(SD_CONTACTS_PATH)) {
                if (rename(tmp_path, SD_CONTACTS_PATH) == 0) {
                    ESP_LOGW(TAG, "loadContacts: recovered interrupted save "
                                  "(promoted %s → %s)", tmp_path, SD_CONTACTS_PATH);
                } else {
                    ESP_LOGW(TAG, "loadContacts: found orphan %s but rename "
                                  "to %s failed (errno=%d)",
                             tmp_path, SD_CONTACTS_PATH, errno);
                }
            }
        }

        // Primary path: SD card.
        if (p4_sdcard_is_mounted() && p4_sdcard_file_exists(SD_CONTACTS_PATH)) {
            size_t fsize = p4_sdcard_file_size(SD_CONTACTS_PATH);
            if (fsize >= 8) {
                uint8_t* blob = (uint8_t*)malloc(fsize);
                if (blob) {
                    FILE* f = fopen(SD_CONTACTS_PATH, "rb");
                    if (f) {
                        size_t rd = fread(blob, 1, fsize, f);
                        fclose(f);
                        if (rd == fsize) {
                            int count = parseContactsBlob(blob, fsize, records, maxCount);
                            free(blob);
                            ESP_LOGI(TAG, "loadContacts: %d contacts loaded from SD", count);
                            return count;
                        }
                    }
                    free(blob);
                }
            }
            ESP_LOGW(TAG, "loadContacts: SD file present but unreadable, "
                          "falling back to legacy NVS");
        }

        // Migration path: legacy NVS blob from before SD storage. Read it,
        // copy to caller, then queue an SD write on the next save. We
        // explicitly DO NOT delete the NVS key here — saveContacts handles
        // that once it confirms the SD write succeeded.
        size_t blobSize = 0;
        nvs_handle_t handle;
        if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
            esp_err_t err = nvs_get_blob(handle, "contacts", NULL, &blobSize);
            if (err == ESP_OK && blobSize >= 8) {
                uint8_t* blob = (uint8_t*)malloc(blobSize);
                if (blob) {
                    err = nvs_get_blob(handle, "contacts", blob, &blobSize);
                    if (err == ESP_OK) {
                        int count = parseContactsBlob(blob, blobSize, records, maxCount);
                        free(blob);
                        nvs_close(handle);
                        ESP_LOGI(TAG, "loadContacts: %d contacts loaded from "
                                      "legacy NVS — will migrate to SD on next save",
                                 count);
                        return count;
                    }
                    free(blob);
                }
            }
            nvs_close(handle);
        }

        ESP_LOGI(TAG, "loadContacts: no saved contacts (fresh install)");
        return 0;
    }

    // =====================================================================
    // Backup / Restore — bulk operations for fresh-flash recovery
    // =====================================================================

    // Force a full write-out of every persisted blob from NVS to the SD
    // card. Useful for a manual "Backup now" UI button — every individual
    // save path already writes through to SD, but this catches anything
    // that may have failed an SD write earlier (card not mounted, write
    // error, etc.). Returns the number of blobs successfully written.
    int backupToSD() {
        if (!p4_sdcard_is_mounted()) {
            ESP_LOGW(TAG, "backupToSD: SD not mounted");
            return 0;
        }
        ensureSDDirectories();
        int written = 0;

        // Identity. Read the NVS blob and write it to the SD path.
        {
            nvs_handle_t handle;
            if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
                size_t len = 0;
                if (nvs_get_blob(handle, "identity", NULL, &len) == ESP_OK && len > 0) {
                    uint8_t* buf = (uint8_t*)malloc(len);
                    if (buf && nvs_get_blob(handle, "identity", buf, &len) == ESP_OK) {
                        FILE* f = fopen(SD_IDENTITY_PATH, "wb");
                        if (f) {
                            fwrite(buf, 1, len, f);
                            fclose(f);
                            written++;
                        }
                    }
                    if (buf) free(buf);
                }
                nvs_close(handle);
            }
        }

        // Generic blob copier: read from NVS, write to the given SD path.
        // Used for prefs, channels and contacts which all live as opaque
        // blobs under NVS_NAMESPACE.
        auto copy_blob = [&](const char* nvs_key, const char* sd_path) {
            nvs_handle_t handle;
            if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return;
            size_t len = 0;
            esp_err_t err = nvs_get_blob(handle, nvs_key, NULL, &len);
            if (err == ESP_OK && len > 0) {
                uint8_t* buf = (uint8_t*)malloc(len);
                if (buf && nvs_get_blob(handle, nvs_key, buf, &len) == ESP_OK) {
                    FILE* f = fopen(sd_path, "wb");
                    if (f) {
                        fwrite(buf, 1, len, f);
                        fclose(f);
                        written++;
                    }
                }
                if (buf) free(buf);
            }
            nvs_close(handle);
        };
        copy_blob("prefs",    SD_PREFS_PATH);
        copy_blob("channels", SD_CHANNELS_PATH);
        // Note: contacts are SD-native since v2 — no NVS blob to copy.
        // The legacy NVS "contacts" key is erased by saveContacts on the
        // first successful SD write after upgrade.

        ESP_LOGI(TAG, "backupToSD: %d blobs written", written);
        return written;
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

        // Check if NVS has contacts. Same pattern: if NVS lacks them but
        // SD has a backup, blit the SD blob into NVS and let loadContacts
        // pick it up via the normal path on next call.
        {
            nvs_handle_t handle;
            if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
                size_t len = 0;
                esp_err_t err = nvs_get_blob(handle, "contacts", NULL, &len);
                nvs_close(handle);
                if (err != ESP_OK || len == 0) {
                    if (p4_sdcard_file_exists(SD_CONTACTS_PATH)) {
                        size_t fsize = p4_sdcard_file_size(SD_CONTACTS_PATH);
                        if (fsize > 0) {
                            uint8_t* buf = (uint8_t*)malloc(fsize);
                            if (buf) {
                                FILE* f = fopen(SD_CONTACTS_PATH, "rb");
                                if (f) {
                                    fread(buf, 1, fsize, f);
                                    fclose(f);
                                    nvs_handle_t wh;
                                    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &wh) == ESP_OK) {
                                        nvs_set_blob(wh, "contacts", buf, fsize);
                                        nvs_commit(wh);
                                        nvs_close(wh);
                                        printf("P4DataStore: contacts restored from SD\n");
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
    // on first write; thereafter we append to the tail.
    //
    // Schema migration: if the file's header version is older than
    // expected_version but the record_size matches, the header is bumped
    // in-place (legal because schema v1 records are byte-compatible with
    // v2 — v1 always wrote zero to the byte that v2 calls heard_count).
    // On magic mismatch or record_size mismatch the file is renamed to
    // .bak and a fresh one written.
    //
    // out_offset (optional): if non-null, receives the byte offset within
    // the file at which this record was placed. The caller stores this on
    // the in-memory message so subsequent in-place rewrites of the same
    // record (e.g. heard_count updates) can seek directly without scanning.
    //
    // Returns true on successful append.
    bool appendChannelMessageRecord(uint8_t channel_idx,
                                     uint32_t expected_magic,
                                     uint16_t expected_version,
                                     uint16_t record_size,
                                     const void* record_bytes,
                                     uint32_t* out_offset = nullptr)
    {
        if (out_offset) *out_offset = 0;
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
                hdr.record_size == record_size &&
                hdr.version <= expected_version) {

                // Layout-compatible. If older, bump version in place so the
                // header reflects the schema actually in use. Writes a
                // single 16-bit field at offset 4 of the file (header is
                // [u32 magic][u16 version][u16 record_size][u32 reserved x 2]).
                if (hdr.version < expected_version) {
                    ESP_LOGI(TAG, "appendChannelMessageRecord: upgrading ch %u "
                                  "header v%u -> v%u in place",
                             (unsigned)channel_idx,
                             (unsigned)hdr.version, (unsigned)expected_version);
                    if (fseek(f, (long)sizeof(uint32_t), SEEK_SET) == 0) {
                        uint16_t v = expected_version;
                        fwrite(&v, sizeof(v), 1, f);
                    }
                }

                // Header OK; seek to end and append. Capture offset first
                // so the caller can write it back to the in-memory record.
                fseek(f, 0, SEEK_END);
                long append_at = ftell(f);
                size_t wrote = fwrite(record_bytes, 1, record_size, f);
                fclose(f);
                bool ok = (wrote == record_size);
                if (ok && out_offset && append_at >= 0) {
                    *out_offset = (uint32_t)append_at;
                }
                return ok;
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
        long append_at = (long)sizeof(hdr);  // first record sits right after the header
        fclose(f);

        if (ok) {
            if (out_offset) *out_offset = (uint32_t)append_at;
            if (fresh_file) {
                ESP_LOGI(TAG, "appendChannelMessageRecord: created %s", path);
            }
        }
        return ok;
    }

    // Rewrite a single record at a known file offset. Used to update
    // heard_count on a sent message in place as additional flood echoes
    // arrive, without appending a duplicate. The offset is the byte
    // position returned by the original appendChannelMessageRecord call
    // (or computed by loadChannelMessageTail). On any mismatch (file
    // missing, header bad, offset out of range) the rewrite is skipped
    // and false returned — the in-RAM state is still correct, just not
    // synced to disk this time around.
    bool rewriteChannelMessageRecord(uint8_t channel_idx,
                                      uint32_t expected_magic,
                                      uint16_t expected_version,
                                      uint32_t file_offset,
                                      uint16_t record_size,
                                      const void* record_bytes)
    {
        if (!record_bytes || record_size == 0) return false;
        if (file_offset == 0) return false;  // 0 means "never appended"
        if (!p4_sdcard_is_mounted()) return false;

        char path[80];
        buildMessagePath(channel_idx, path, sizeof(path));
        if (!p4_sdcard_file_exists(path)) return false;

        FILE* f = fopen(path, "r+b");
        if (!f) return false;

        struct {
            uint32_t magic;
            uint16_t version;
            uint16_t record_size;
            uint32_t reserved[2];
        } __attribute__((packed)) hdr;

        if (fread(&hdr, sizeof(hdr), 1, f) != 1 ||
            hdr.magic != expected_magic ||
            hdr.record_size != record_size ||
            hdr.version > expected_version) {
            fclose(f);
            ESP_LOGW(TAG, "rewriteChannelMessageRecord: header check failed on ch %u",
                     (unsigned)channel_idx);
            return false;
        }

        // Sanity: offset must be on a record boundary past the header.
        if (file_offset < sizeof(hdr) ||
            ((file_offset - sizeof(hdr)) % record_size) != 0) {
            fclose(f);
            ESP_LOGW(TAG, "rewriteChannelMessageRecord: offset %u not on record boundary",
                     (unsigned)file_offset);
            return false;
        }

        if (fseek(f, (long)file_offset, SEEK_SET) != 0) {
            fclose(f);
            return false;
        }

        size_t wrote = fwrite(record_bytes, 1, record_size, f);
        fclose(f);
        return wrote == record_size;
    }

    // Delete the entire message file for a channel slot. Used when a
    // channel is removed from the picker, so the slot can be re-used
    // for a fresh channel without inheriting the old channel's history.
    // Returns true if the file was removed (or didn't exist to begin
    // with); false on a hard error.
    bool deleteChannelMessageFile(uint8_t channel_idx) {
        if (!p4_sdcard_is_mounted()) return false;
        char path[80];
        buildMessagePath(channel_idx, path, sizeof(path));
        if (!p4_sdcard_file_exists(path)) return true;
        if (unlink(path) == 0) {
            ESP_LOGI(TAG, "deleteChannelMessageFile: removed %s", path);
            return true;
        }
        ESP_LOGW(TAG, "deleteChannelMessageFile: unlink(%s) failed (errno=%d)",
                 path, errno);
        return false;
    }

    // Rename a channel's message file when its slot index changes (e.g.
    // delete-and-compact shifts channels down to fill a gap). Returns
    // true if the rename succeeded, or if there was no source file to
    // rename (a slot that never had any messages persisted). False on
    // a hard error.
    bool renameChannelMessageFile(uint8_t from_idx, uint8_t to_idx) {
        if (!p4_sdcard_is_mounted()) return false;
        if (from_idx == to_idx) return true;
        char from_path[80], to_path[80];
        buildMessagePath(from_idx, from_path, sizeof(from_path));
        buildMessagePath(to_idx,   to_path,   sizeof(to_path));
        if (!p4_sdcard_file_exists(from_path)) return true;
        // Clear any stale file at the destination so rename doesn't fail.
        if (p4_sdcard_file_exists(to_path)) unlink(to_path);
        if (rename(from_path, to_path) == 0) {
            ESP_LOGI(TAG, "renameChannelMessageFile: %s -> %s", from_path, to_path);
            return true;
        }
        ESP_LOGW(TAG, "renameChannelMessageFile: rename(%s, %s) failed (errno=%d)",
                 from_path, to_path, errno);
        return false;
    }


    // Load up to max_records most recent records from a channel's file into
    // records_buf (each entry record_size bytes). Returns number actually
    // loaded. If the file's record count exceeds max_records, only the
    // last max_records are loaded.
    //
    // Accepts schema versions <= expected_version (older layouts that share
    // the same record_size are byte-compatible with the current schema; see
    // P4_MSG_FILE_VERSION_MIN in MeckMesh.h). Newer-than-expected versions
    // are refused — they may have layouts this build can't safely interpret.
    //
    // offsets_buf (optional): if non-null, must hold at least max_records
    // uint32_t entries; on return offsets_buf[i] is the byte offset within
    // the file at which records_buf[i] was read. Callers use this to
    // populate file_offset on each in-memory message so subsequent
    // in-place rewrites can target the correct disk location.
    int loadChannelMessageTail(uint8_t channel_idx,
                                uint32_t expected_magic,
                                uint16_t expected_version,
                                uint16_t record_size,
                                void* records_buf,
                                uint32_t* offsets_buf,
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
            hdr.record_size != record_size ||
            hdr.version > expected_version) {
            fclose(f);
            ESP_LOGW(TAG, "loadChannelMessageTail: schema mismatch on ch %u "
                          "(file v%u, expected <= v%u, size %u vs %u), skipping",
                     (unsigned)channel_idx,
                     (unsigned)hdr.version, (unsigned)expected_version,
                     (unsigned)hdr.record_size, (unsigned)record_size);
            return 0;
        }
        if (hdr.version < expected_version) {
            ESP_LOGI(TAG, "loadChannelMessageTail: ch %u loading legacy v%u "
                          "(layout-compatible with v%u)",
                     (unsigned)channel_idx,
                     (unsigned)hdr.version, (unsigned)expected_version);
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

        long first_offset = (long)sizeof(hdr) + (skip_records * record_size);
        if (fseek(f, first_offset, SEEK_SET) != 0) { fclose(f); return 0; }

        size_t bytes_to_read = (size_t)(load_records * record_size);
        size_t got = fread(records_buf, 1, bytes_to_read, f);
        fclose(f);

        int loaded = (int)(got / record_size);

        // Fill per-record offsets in the same order as records_buf.
        if (offsets_buf) {
            for (int i = 0; i < loaded; i++) {
                offsets_buf[i] = (uint32_t)(first_offset + (long)i * (long)record_size);
            }
        }

        ESP_LOGI(TAG, "loadChannelMessageTail: ch %u loaded %d of %ld total",
                 (unsigned)channel_idx, loaded, total_records);
        return loaded;
    }
};
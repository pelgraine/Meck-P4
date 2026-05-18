/*
 * MeckExport.h — User-triggered export of current state to a MeshCore
 * app-compatible JSON config on SD.
 *
 * Pair to MeckImport.h. Writes /sdcard/meshcore/export-<unix_epoch>.json
 * via tmp+rename. The output format matches what the MeshCore mobile and
 * desktop apps emit on export and what meck_import_from_sd reads back in,
 * so a Meck-exported file is portable to another Meck device or into the
 * MeshCore apps directly.
 *
 * Section selection via the flags bitmask lets the user choose what to
 * include — useful when, for example, you want to share contacts and
 * channels with someone else without handing over your private key. The
 * "name" field is always emitted (it's the human-readable title of the
 * config and harmless either way).
 *
 * The optional sub-objects the MeshCore apps sometimes emit
 * (position_settings, other_settings, auto_add_settings) are deliberately
 * omitted: Meck doesn't store those fields and the importer ignores them,
 * and a minimal sample from the desktop app confirms they're optional in
 * the schema.
 *
 * Round-trip caveats:
 *   - contacts[].custom_name is always null on output. Meck collapses
 *     custom_name and name into a single stored name at import time.
 *   - contacts[].out_path_list is always null on output. Meck doesn't
 *     expose the cached path.
 *   - contact names are post-ASCII-strip (Meck-P4's Montserrat fonts only
 *     cover Latin-1 so emojis are stripped at import). Names round-trip
 *     in their already-stripped form.
 *
 * Called from the Settings screen's "Export Config" modal. Not boot-time.
 */

#pragma once

#include "MeckDataStore.h"
#include "MeckSDCard.h"
#include "meck.h"        // for meck_clock_get_utc
#include "cJSON.h"
#include "esp_log.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <sys/stat.h>
#include <sys/unistd.h>

// Bit flags selecting which sections of the JSON to populate.
// All four together = MECK_EXPORT_ALL produces a fully re-importable file.
// These are mirrored in target.h so UI code can build the flags bitmask
// without including this header; the #ifndef guards keep both translation
// units (meck_app.cpp includes both) compatible.
#ifndef MECK_EXPORT_IDENTITY
#define MECK_EXPORT_IDENTITY  (1u << 0)
#define MECK_EXPORT_CHANNELS  (1u << 1)
#define MECK_EXPORT_CONTACTS  (1u << 2)
#define MECK_EXPORT_RADIO     (1u << 3)
#define MECK_EXPORT_ALL       (MECK_EXPORT_IDENTITY | MECK_EXPORT_CHANNELS | \
                               MECK_EXPORT_CONTACTS | MECK_EXPORT_RADIO)
#endif

static constexpr const char* MECK_EXPORT_DIR = "/sdcard/meshcore";

// Hex-encode `len` bytes as lowercase ASCII. out must have at least
// 2*len+1 bytes. Matches the lowercase hex the MeshCore app uses in its
// JSON exports (so byte-for-byte symmetric with the importer's
// meck_import_hex_decode).
static void meck_export_hex_encode(const uint8_t* src, size_t len, char* out) {
    static const char digits[] = "0123456789abcdef";
    for (size_t i = 0; i < len; i++) {
        out[i * 2]     = digits[(src[i] >> 4) & 0x0F];
        out[i * 2 + 1] = digits[src[i] & 0x0F];
    }
    out[len * 2] = '\0';
}

// Format an e7-integer lat/lon back to the variable-precision decimal
// string the MeshCore app uses (e.g. "-33.878016", "-33.89223"). Strips
// trailing zeros after the decimal point but keeps at least one digit
// (so "-33.0" stays as "-33.0", not "-33."). Buffer size 16 is enough
// for the full range plus sign plus null terminator.
static void meck_export_e7_to_decimal(int32_t e7, char* out, size_t out_size) {
    if (out_size == 0) return;
    bool neg = (e7 < 0);
    // Use uint32_t for the magnitude so abs(INT32_MIN) doesn't overflow.
    uint32_t mag = neg ? (uint32_t)(-(int64_t)e7) : (uint32_t)e7;
    uint32_t whole = mag / 10000000u;
    uint32_t frac  = mag % 10000000u;

    // Always emit 7 fractional digits first, then trim trailing zeros.
    char tmp[24];
    int n = snprintf(tmp, sizeof(tmp), "%s%u.%07u",
                     neg ? "-" : "", (unsigned)whole, (unsigned)frac);
    if (n < 0 || (size_t)n >= sizeof(tmp)) {
        // Shouldn't happen for valid e7 values, but truncate safely.
        snprintf(out, out_size, "0.0");
        return;
    }
    // Trim trailing zeros from the fractional part, leaving at least one
    // digit after the decimal point.
    while (n > 1 && tmp[n - 1] == '0' && tmp[n - 2] != '.') {
        tmp[--n] = '\0';
    }
    strncpy(out, tmp, out_size - 1);
    out[out_size - 1] = '\0';
}

// Read /sdcard/meshcore/identity/_main.id back into split priv/pub
// buffers. Returns true if the file exists and is exactly 96 bytes
// (priv64 || pub32). False on any error.
static bool meck_export_read_identity(uint8_t* out_priv64, uint8_t* out_pub32) {
    const char* path = "/sdcard/meshcore/identity/_main.id";
    if (!p4_sdcard_file_exists(path)) return false;
    if (p4_sdcard_file_size(path) != 96) return false;
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    size_t rd_priv = fread(out_priv64, 1, 64, f);
    size_t rd_pub  = fread(out_pub32,  1, 32, f);
    fclose(f);
    return (rd_priv == 64 && rd_pub == 32);
}

// Build the JSON tree as cJSON. Caller owns the returned root and must
// call cJSON_Delete on it. Returns nullptr only on allocation failure.
// Identity errors (file missing/wrong size with MECK_EXPORT_IDENTITY set)
// are reported by the caller after this returns nullptr-or-incomplete.
static cJSON* meck_export_build_json(P4DataStore& store,
                                     const P4NodePrefs& prefs,
                                     uint32_t flags,
                                     bool* identity_ok_out) {
    if (identity_ok_out) *identity_ok_out = true;

    cJSON* root = cJSON_CreateObject();
    if (!root) return nullptr;

    // "name" — always emitted.
    cJSON_AddStringToObject(root, "name", prefs.node_name);

    // Identity. If requested but unreadable, mark identity_ok=false so the
    // caller can fail fast with a clear error before writing the file.
    if (flags & MECK_EXPORT_IDENTITY) {
        uint8_t priv64[64];
        uint8_t pub32[32];
        if (!meck_export_read_identity(priv64, pub32)) {
            ESP_LOGW("export", "identity requested but _main.id missing/invalid");
            if (identity_ok_out) *identity_ok_out = false;
            // Fall through and emit the rest; the caller decides what
            // to do with the incomplete tree.
        } else {
            char pub_hex[65];
            char priv_hex[129];
            meck_export_hex_encode(pub32,  32, pub_hex);
            meck_export_hex_encode(priv64, 64, priv_hex);
            cJSON_AddStringToObject(root, "public_key",  pub_hex);
            cJSON_AddStringToObject(root, "private_key", priv_hex);
        }
    }

    // radio_settings sub-object.
    if (flags & MECK_EXPORT_RADIO) {
        cJSON* radio = cJSON_CreateObject();
        if (radio) {
            // freq is in MHz internally, JSON wants kHz.
            cJSON_AddNumberToObject(radio, "frequency",
                (double)prefs.freq * 1000.0);
            // bw is in kHz internally, JSON wants Hz.
            cJSON_AddNumberToObject(radio, "bandwidth",
                (double)prefs.bw * 1000.0);
            cJSON_AddNumberToObject(radio, "spreading_factor", prefs.sf);
            cJSON_AddNumberToObject(radio, "coding_rate",      prefs.cr);
            cJSON_AddNumberToObject(radio, "tx_power",         prefs.tx_power_dbm);
            cJSON_AddItemToObject(root, "radio_settings", radio);
        }
    }

    // channels array. Always emit (possibly empty) when requested, to
    // match the minimal sample export from the desktop app which still
    // includes "channels": [] when there are none.
    if (flags & MECK_EXPORT_CHANNELS) {
        cJSON* arr = cJSON_CreateArray();
        if (arr) {
            P4ChannelRecord channels[MAX_GROUP_CHANNELS];
            int count = 0;
            if (store.loadChannels(channels, MAX_GROUP_CHANNELS, count)) {
                for (int i = 0; i < count; i++) {
                    cJSON* item = cJSON_CreateObject();
                    if (!item) break;
                    cJSON_AddStringToObject(item, "name", channels[i].name);
                    // JSON secret is 16 bytes (AES-128), encoded as 32 hex
                    // chars. The on-disk slot reserves 32 bytes but only
                    // the first 16 are used — see MeckImport.h channel
                    // merge for the symmetric read side.
                    char sec_hex[33];
                    meck_export_hex_encode(channels[i].secret, 16, sec_hex);
                    cJSON_AddStringToObject(item, "secret", sec_hex);
                    cJSON_AddItemToArray(arr, item);
                }
            }
            cJSON_AddItemToObject(root, "channels", arr);
        }
    }

    // contacts array. Same "always emit when requested" rule.
    if (flags & MECK_EXPORT_CONTACTS) {
        cJSON* arr = cJSON_CreateArray();
        if (arr) {
            typedef P4DataStore::ContactRecord CR;
            CR* contacts = (CR*)malloc(sizeof(CR) * MAX_CONTACTS);
            if (contacts) {
                int n = store.loadContacts(contacts, MAX_CONTACTS);
                if (n < 0) n = 0;
                for (int i = 0; i < n; i++) {
                    cJSON* item = cJSON_CreateObject();
                    if (!item) break;
                    cJSON_AddNumberToObject(item, "type", contacts[i].type);
                    cJSON_AddStringToObject(item, "name", contacts[i].name);
                    // custom_name always null: Meck doesn't store one
                    // separately from name.
                    cJSON_AddNullToObject(item, "custom_name");
                    char pub_hex[65];
                    meck_export_hex_encode(contacts[i].pub_key, 32, pub_hex);
                    cJSON_AddStringToObject(item, "public_key", pub_hex);
                    cJSON_AddNumberToObject(item, "flags", contacts[i].flags);
                    char lat_str[16], lon_str[16];
                    meck_export_e7_to_decimal(contacts[i].gps_lat,
                                              lat_str, sizeof(lat_str));
                    meck_export_e7_to_decimal(contacts[i].gps_lon,
                                              lon_str, sizeof(lon_str));
                    cJSON_AddStringToObject(item, "latitude",  lat_str);
                    cJSON_AddStringToObject(item, "longitude", lon_str);
                    cJSON_AddNumberToObject(item, "last_advert",
                                            (double)contacts[i].last_advert_timestamp);
                    cJSON_AddNumberToObject(item, "last_modified",
                                            (double)contacts[i].lastmod);
                    // out_path_list always null: Meck doesn't store the
                    // cached path persistently.
                    cJSON_AddNullToObject(item, "out_path_list");
                    cJSON_AddItemToArray(arr, item);
                }
                free(contacts);
            }
            cJSON_AddItemToObject(root, "contacts", arr);
        }
    }

    return root;
}

// Atomic write of `json` text to `final_path` via final_path.tmp + rename.
// Returns true on success. Local copy of P4DataStore::atomicWriteToSD's
// pattern, kept here because that method is private to the class.
static bool meck_export_atomic_write(const char* final_path,
                                     const char* text, size_t len) {
    char tmp_path[160];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", final_path);

    FILE* f = fopen(tmp_path, "wb");
    if (!f) {
        ESP_LOGW("export", "fopen(%s, wb) failed (errno=%d)", tmp_path, errno);
        return false;
    }
    size_t wrote = fwrite(text, 1, len, f);
    int closed = fclose(f);
    if (wrote != len || closed != 0) {
        ESP_LOGW("export", "write/close failed (wrote=%u expected=%u close=%d)",
                 (unsigned)wrote, (unsigned)len, closed);
        unlink(tmp_path);
        return false;
    }

    // rename on FAT does not overwrite (errno EEXIST), so clear the
    // destination first. Best-effort; if there's no stale file the
    // unlink fails harmlessly.
    unlink(final_path);
    if (rename(tmp_path, final_path) != 0) {
        ESP_LOGW("export", "rename(%s, %s) failed (errno=%d)",
                 tmp_path, final_path, errno);
        unlink(tmp_path);
        return false;
    }
    return true;
}

// Public entry point — build the JSON and write it to SD. See the file
// header for behaviour. out_path (if non-null) receives just the
// basename of the resulting file (no directory prefix), so callers can
// display it without leading "/sdcard/meshcore/" clutter.
static bool meck_export_to_sd(P4DataStore& store,
                              const P4NodePrefs& prefs,
                              uint32_t flags,
                              char* out_path, size_t out_path_size) {
    if (out_path && out_path_size > 0) out_path[0] = '\0';

    if (!p4_sdcard_is_mounted()) {
        ESP_LOGW("export", "SD not mounted");
        return false;
    }
    mkdir(MECK_EXPORT_DIR, 0755);  // ignore EEXIST

    bool identity_ok = true;
    cJSON* root = meck_export_build_json(store, prefs, flags, &identity_ok);
    if (!root) {
        ESP_LOGW("export", "cJSON allocation failed");
        return false;
    }
    if ((flags & MECK_EXPORT_IDENTITY) && !identity_ok) {
        cJSON_Delete(root);
        return false;
    }

    // Log when the user explicitly excludes identity, so the partial
    // export shows up in the serial trace as deliberate rather than a
    // bug.
    if ((flags & MECK_EXPORT_IDENTITY) == 0) {
        ESP_LOGI("export", "exporting WITHOUT identity (not re-importable as a new node)");
    }

    char* text = cJSON_Print(root);
    cJSON_Delete(root);
    if (!text) {
        ESP_LOGW("export", "cJSON_Print failed");
        return false;
    }
    size_t text_len = strlen(text);

    // Pick a filename. Prefer UTC epoch when the soft RTC has been set;
    // fall back to monotonic uptime so the filename is still unique-ish
    // pre-GPS-fix. cJSON_free (== free) the rendered text after writing.
    char filename[64];
    uint32_t now = meck_clock_get_utc();
    if (now != 0) {
        snprintf(filename, sizeof(filename), "export-%u.json", (unsigned)now);
    } else {
        time_t fallback = time(nullptr);
        if (fallback > 0) {
            snprintf(filename, sizeof(filename), "export-%ld.json",
                     (long)fallback);
        } else {
            snprintf(filename, sizeof(filename), "export-uptime-%lu.json",
                     (unsigned long)(esp_log_timestamp() / 1000));
        }
    }
    char full_path[160];
    snprintf(full_path, sizeof(full_path), "%s/%s", MECK_EXPORT_DIR, filename);

    bool ok = meck_export_atomic_write(full_path, text, text_len);
    cJSON_free(text);

    if (!ok) return false;

    if (out_path && out_path_size > 0) {
        strncpy(out_path, filename, out_path_size - 1);
        out_path[out_path_size - 1] = '\0';
    }
    ESP_LOGI("export", "wrote %s (%u bytes, flags=0x%X)",
             full_path, (unsigned)text_len, (unsigned)flags);
    return true;
}

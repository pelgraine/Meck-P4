/*
 * P4Mesh.h — MeshCore BaseChatMesh subclass for T-Display P4
 *
 * Standalone mesh node (no BLE, no WiFi). Receives channel messages and
 * adverts, stores them in thread-safe ring buffers for LVGL display.
 *
 * Replaces the raw SX1262 polling loop with MeshCore's full protocol
 * stack: Dispatcher → Mesh → BaseChatMesh → P4Mesh callbacks → LVGL UI.
 *
 * Channel and prefs persistence via P4DataStore (NVS + SD card).
 */

#pragma once

#include "arduino_compat.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#include <helpers/BaseChatMesh.h>
#pragma GCC diagnostic pop
#include <helpers/AdvertDataHelpers.h>
#include <StaticPoolPacketManager.h>
#include <SimpleMeshTables.h>
#include "MeckDataStore.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "mbedtls/sha256.h"
#include "mbedtls/base64.h"
#include <strings.h>   // strcasecmp for case-insensitive name match in migrateChannelSecrets

// Forward decl for the deferred SD-save queue, defined in meck_app.cpp.
// Allows ring-write call sites in this header to enqueue without including
// target.h (which would pull P4SX1262Radio.h into hot paths). ring_idx is
// the position of the just-written message in the channel's ring; the
// drain uses it to write the resulting file_offset back to the same slot
// after an initial append, and to look up the current heard_count for any
// in-place rewrite triggered by a subsequent echo.
struct P4ChannelMessage;
extern "C" void meck_request_save_message(uint8_t channel_idx, int ring_idx,
                                           const P4ChannelMessage* msg);

// ---- ESP-IDF clock/RNG implementations ----

// ============================================================================
// P4MeshTables — flood-relay dedup with our-own-packet pass-through
// ----------------------------------------------------------------------------
// Mesh::sendFlood marks every packet it sends in _tables (see upstream
// Mesh.cpp), so that the upstream GRP_TXT receive path
//
//     } else if (!_tables->hasSeen(pkt)) {
//         // ... decrypt + dispatch onGroupDataRecv → onChannelMessageRecv
//
// skips re-broadcasts of our own packet. For repeaters this prevents
// re-forwarding loops. For a companion that wants to count "Heard N
// Repeats" by listening for flood relays of its own send, the upstream
// behaviour silently drops every echo before our isOurOwnEcho check
// ever runs, so heard_count stays at zero.
//
// P4MeshTables keeps a small ring of recent outgoing packet hashes. On
// hasSeen() we check the ring first:
//   - In "marking outgoing" mode (set by Meck around its own send call):
//     capture this packet's hash into the ring and return false WITHOUT
//     adding to the underlying SimpleMeshTables. The send-side caller
//     ignores the return value but the side effect (no marking) is what
//     matters.
//   - On any receive whose hash matches our outgoing ring: return false
//     WITHOUT touching the underlying table. Every distinct repeater
//     relay of our packet therefore reaches onChannelMessageRecv, and
//     isOurOwnEcho counts each one.
//   - Everything else: defer to SimpleMeshTables for normal dedup.
//
// Safe for companion / standalone builds that never forward packets
// (allowPacketForward returns false). Do NOT use on a repeater build —
// it would re-forward its own packets indefinitely.
// ============================================================================

class P4MeshTables : public SimpleMeshTables {
public:
    // Flag toggled around our own sendChannelMessage call. While true,
    // the next hasSeen() invocation is treated as the sendFlood marking
    // step for our outgoing packet.
    void beginMarkingOurOutgoing() { _marking_outgoing = true; }
    void endMarkingOurOutgoing()   { _marking_outgoing = false; }

    bool hasSeen(const mesh::Packet* pkt) override {
        uint8_t h[MAX_HASH_SIZE];
        pkt->calculatePacketHash(h);

        if (_marking_outgoing) {
            // Capture this packet hash so future re-broadcast arrivals
            // can be recognised and passed through.
            memcpy(_outgoing[_outgoing_idx], h, MAX_HASH_SIZE);
            _outgoing_idx = (_outgoing_idx + 1) % OUR_OUTGOING_RING;
            // Do NOT call SimpleMeshTables::hasSeen() — that would add
            // the hash to the upstream dedup table and re-introduce the
            // very filtering we're trying to bypass.
            return false;
        }

        // Receive path: if this incoming packet's hash matches one we
        // sent recently, let it through every time without ever
        // marking it seen, so subsequent repeater relays (each
        // delivering the same hash from a different angle) also fire
        // onChannelMessageRecv and get counted.
        for (int i = 0; i < OUR_OUTGOING_RING; i++) {
            if (memcmp(_outgoing[i], h, MAX_HASH_SIZE) == 0) {
                return false;
            }
        }

        return SimpleMeshTables::hasSeen(pkt);
    }

private:
    static const int OUR_OUTGOING_RING = 8;
    uint8_t _outgoing[OUR_OUTGOING_RING][MAX_HASH_SIZE] = {{0}};
    int _outgoing_idx = 0;
    bool _marking_outgoing = false;
};

// Forward decl for begin/end markers exposed by meck_app.cpp so MeckMesh
// can flip P4MeshTables into "marking outgoing" mode around its own
// sendGroupMessage call without taking a direct dependency on the
// globally-instantiated tables.
extern "C" void meck_tables_begin_outgoing();
extern "C" void meck_tables_end_outgoing();


class P4MillisecondClock : public mesh::MillisecondClock {
public:
    unsigned long getMillis() override { return millis(); }
};

class P4RTCClock : public mesh::RTCClock {
    uint32_t base_time;
    uint64_t accumulator;
    unsigned long prev_millis;

    // Compile-time fallback epoch. Used as the boot-time clock value
    // until something more authoritative (advert, GPS, or explicit
    // setCurrentTime) overrides it. This way pre-sync timestamps fall
    // close to "now" rather than at a baked-in 2025 date that ages out
    // over time. Recomputes automatically every flash; no manual
    // maintenance needed.
    //
    // Two sources, in priority order:
    //   1. MECK_BUILD_EPOCH (preferred) — UTC epoch injected at build
    //      time by CMakeLists.txt via `string(TIMESTAMP ... %s UTC)`.
    //      This is the only way to get a true UTC value at build time;
    //      __DATE__ / __TIME__ are LOCAL time on the build machine, so
    //      relying on them alone leaves the device displaying
    //      local_time + utc_offset, off by the build machine's offset.
    //   2. __DATE__ / __TIME__ parse — fallback if MECK_BUILD_EPOCH is
    //      not defined (e.g. the CMakeLists snippet wasn't applied).
    //      The result is treated as UTC even though it's really local,
    //      which on a +10 build machine displays 10 hours late.
    //
    // __DATE__ is "Mmm dd yyyy" (11 chars + NUL), __TIME__ is "hh:mm:ss".
    // The parser is constexpr-friendly but kept as a static helper for
    // clarity rather than constexpr-correctness; the resulting epoch is
    // computed once in the constructor and stored.
    static uint32_t compileTimeEpoch() {
#ifdef MECK_BUILD_EPOCH
        return (uint32_t)MECK_BUILD_EPOCH;
#else
        const char* d = __DATE__;
        const char* t = __TIME__;
        int year = (d[7] - '0') * 1000 + (d[8] - '0') * 100
                 + (d[9] - '0') * 10   + (d[10] - '0');
        const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
        int month = 0;
        for (int i = 0; i < 12; i++) {
            if (d[0] == months[i*3] && d[1] == months[i*3+1] && d[2] == months[i*3+2]) {
                month = i; break;
            }
        }
        int day = (d[4] == ' ' ? 0 : (d[4] - '0') * 10) + (d[5] - '0');
        int hour = (t[0] - '0') * 10 + (t[1] - '0');
        int minute = (t[3] - '0') * 10 + (t[4] - '0');
        int second = (t[6] - '0') * 10 + (t[7] - '0');

        // Days-since-epoch using Howard Hinnant's date algorithm
        // (https://howardhinnant.github.io/date_algorithms.html). Treats
        // year/month/day as a date in the proleptic Gregorian calendar and
        // returns days since 1970-01-01. Works for all years we'll see.
        int y = year - (month < 2 ? 1 : 0);
        int m = month + 1;  // algorithm wants 1-indexed
        int era = (y >= 0 ? y : y - 399) / 400;
        unsigned yoe = (unsigned)(y - era * 400);
        unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + day - 1;
        unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
        long days_since_epoch = (long)era * 146097 + (long)doe - 719468;

        return (uint32_t)(days_since_epoch * 86400 + hour * 3600 + minute * 60 + second);
#endif
    }

public:
    P4RTCClock() : RTCClock() {
        base_time = compileTimeEpoch();
        accumulator = 0;
        prev_millis = millis();
    }
    uint32_t getCurrentTime() override { return base_time + (uint32_t)(accumulator / 1000); }
    void setCurrentTime(uint32_t time) override {
        base_time = time;
        accumulator = 0;
        prev_millis = millis();
    }
    void tick() override {
        unsigned long now = millis();
        accumulator += (now - prev_millis);
        prev_millis = now;
    }
};

class P4RNG : public mesh::RNG {
public:
    void random(uint8_t* dest, size_t sz) override {
        for (size_t i = 0; i < sz; i += 4) {
            uint32_t r = esp_random();
            size_t n = (sz - i < 4) ? (sz - i) : 4;
            memcpy(dest + i, &r, n);
        }
    }
};

// ---- Channel message ring buffer ----

#define P4_MSG_PER_CHANNEL  500
#define P4_MSG_TEXT_LEN      300

struct P4ChannelMessage {
    char text[P4_MSG_TEXT_LEN];   // "sender: message"
    uint32_t timestamp;
    uint8_t channel_idx;
    uint8_t path_len;
    // heard_count: for our own outgoing messages, the number of flood-echo
    // copies we've heard come back through the mesh. Each distinct repeater
    // that re-broadcasts our packet causes onChannelMessageRecv to fire on
    // the isOurOwnEcho path and bump this counter. UI uses it to display a
    // "✓ Heard by N" indicator on sent bubbles — the same affordance the
    // MeshCore mobile app shows as "Heard N Repeats". For received bubbles
    // this stays 0 (the field is meaningful only for our own sends).
    // Persisted to disk in schema v2 onward so a "Heard 3 Repeats" footer
    // survives a reboot.
    uint8_t heard_count;
    bool valid;
    // file_offset: byte offset of this record within its channel's ch_<idx>.bin
    // file. Zero means "not yet persisted" (initial save still pending or this
    // message predates persistence). Once non-zero, future state changes
    // (e.g. heard_count increment) rewrite the record in-place at this offset
    // rather than appending a duplicate. In-memory only — not part of the
    // on-disk record, recomputed at load time from file position.
    uint32_t file_offset;
};

// ---- On-disk format for channel message persistence ----
//
// File path: /sdcard/meshcore/messages/ch_<idx>.bin (one per channel)
//
// Schema v1: timestamp / channel / path_len / text / valid, with reserved
//            bytes for SNR, hop path, DM peer hash, heard_count.
// Schema v2: adds heard_count (1 byte stolen from the path[] reserved
//            region). Layout is byte-compatible with v1 because v1 always
//            wrote zero to the stolen byte, so a v2 reader interprets v1
//            records as v2 records with heard_count=0 (semantically correct
//            — pre-v2 records simply never had echo counts persisted).
//            Migration is a header version bump, no record data movement.
//
// Magic 'MCMS' (Meck Channel Message Store) — distinct from upstream Meck's
// 'MCHS' since the storage strategy differs (per-channel vs single file).

#define P4_MSG_FILE_MAGIC           0x4D434D53U  // 'MCMS' little-endian
#define P4_MSG_FILE_VERSION         2
#define P4_MSG_FILE_VERSION_MIN     1  // oldest accepted on load (in-place upgraded on first write)

struct __attribute__((packed)) P4MsgFileRecord {
    uint32_t timestamp;
    uint32_t dm_peer_hash;       // reserved (0) — DM persistence pending
    uint8_t  channel_idx;
    uint8_t  path_len;
    int8_t   snr_x4;             // reserved (0) — bumped to v3 when wired
    uint8_t  flags;              // bit 0 = valid; rest reserved
    uint8_t  heard_count;        // v2: flood-echo count for our own sends
    uint8_t  path[7];            // reserved zeros — hop hashes pending (was [8] in v1)
    char     text[P4_MSG_TEXT_LEN];
};
// Total: 4 + 4 + 1 + 1 + 1 + 1 + 1 + 7 + 300 = 320 bytes (identical to v1)
static_assert(sizeof(P4MsgFileRecord) == 320, "P4MsgFileRecord size must stay 320 bytes for v1 compatibility");

// ---- Recent heard ring buffer ----

#define P4_RECENT_HEARD_SIZE  16

struct P4RecentHeard {
    char name[32];
    uint8_t pub_key_prefix[4];
    // Full 32-byte Ed25519 public key. Required to build a ContactInfo
    // capable of crypto (the shared-secret cache, encrypted DMs, etc.).
    // pub_key_prefix is kept around for backward compatibility with
    // existing UI sites that only render a 2-byte short ID; new code
    // (Discover screen, contact-add-from-heard) reads pub_key instead.
    uint8_t pub_key[32];
    float rssi;
    float snr;
    uint8_t path_len;
    uint32_t timestamp;
    uint8_t adv_type;
    bool valid;
};

// ---- Active Discovery ----
//
// Discovery is an active scan, not a passive list-of-cached-adverts. The
// model (per MeshCore issue #1027 and the "enhanced zero-hop neighbour
// discovery" roadmap item, marked done upstream):
//
//   1. We transmit a zero-hop self-advert.
//   2. Repeaters / chat nodes within radio range that have zero-hop
//      response enabled (set advert.interval > 0 on a repeater) reply
//      with their own zero-hop advert.
//   3. Any other adverts that happen to land in the scan window are
//      also captured.
//   4. After the window closes (P4_DISCOVERY_WINDOW_MS), the list
//      freezes until the next manual scan.
//
// Stored separately from _recent[] so a scan doesn't perturb the
// general heard-list, and so cached entries (carried over from
// _recent[] at scan start) can be visually distinguished from
// freshly-heard zero-hop responses:
//   snr_x4 != 0  →  heard during this scan, snr reading is real
//   snr_x4 == 0  →  pre-seeded from _recent[] cache, show hop count
//
// The DiscoveredNode shape is informed by the T-Deck Pro firmware's
// DiscoveryScreen.h (uploaded reference) but flattened — our P4 LVGL
// renderer reads fields directly rather than going via a ContactInfo
// nested struct.

#define P4_DISCOVERED_SIZE       32
#define P4_DISCOVERY_WINDOW_MS   30000   // 30s scan window

struct DiscoveredNode {
    bool valid;
    char name[32];
    uint8_t pub_key[32];
    uint8_t adv_type;        // 1=chat, 2=repeater, 3=room, 4=sensor
    int8_t  snr_x4;          // SNR × 4. 0 = pre-seeded cache, not heard live.
    uint8_t path_len;        // For cached/flood entries; 0 for zero-hop.
    uint32_t heard_ms;       // millis() at most-recent capture in this scan
    bool already_in_contacts;
};

// ============================================================
// P4Mesh — standalone mesh for T-Display P4
// ============================================================

class Meck : public BaseChatMesh {
public:
    Meck(mesh::Radio& radio, P4RNG& rng, P4RTCClock& rtc, SimpleMeshTables& tables)
        : BaseChatMesh(radio, *new P4MillisecondClock(), rng, rtc,
                        *new StaticPoolPacketManager(16), tables),
          _rtc_ref(rtc), _store(nullptr), _prefs(nullptr)
    {
        // Per-channel message rings, each P4_MSG_PER_CHANNEL entries in PSRAM.
        // 8 channels × 500 msgs × ~308 bytes ≈ 1.2 MB total. PSRAM has plenty.
        size_t per_ring = P4_MSG_PER_CHANNEL * sizeof(P4ChannelMessage);
        for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
            _msgs_ch[i] = (P4ChannelMessage*)heap_caps_malloc(per_ring, MALLOC_CAP_SPIRAM);
            if (_msgs_ch[i]) {
                memset(_msgs_ch[i], 0, per_ring);
            } else {
                printf("Meck: PSRAM alloc FAILED for channel %d ring (%zu bytes)\n",
                       i, per_ring);
            }
            _msg_count_ch[i]  = 0;
            _msg_newest_ch[i] = -1;
        }
        printf("Meck: allocated %d channel msg rings × %d entries (%zu KB total)\n",
               MAX_GROUP_CHANNELS, P4_MSG_PER_CHANNEL,
               (size_t)(MAX_GROUP_CHANNELS * per_ring) / 1024);

        memset((void*)_msg_unread_ch, 0, sizeof(_msg_unread_ch));
        _msg_dirty = false;

        _recent_count = 0;
        _recent_newest = -1;
        _recent_dirty = false;
        memset(_recent, 0, sizeof(_recent));

        _discovered_count = 0;
        _discovery_active = false;
        _discovery_start_ms = 0;
        _discovery_tag = 0;
        _discovery_dirty = false;
        memset(_discovered, 0, sizeof(_discovered));

        _advert_enabled = false;
        _next_advert_ms = 0;
        _advert_interval_ms = 0;  // No auto-advert — companion nodes advert manually only

        _contacts_save_pending = false;
        _contacts_save_at = 0;

        _mutex = xSemaphoreCreateMutex();
    }

    // ---- Initialization ----

    void begin(P4DataStore& store, P4NodePrefs& prefs) {
        _store = &store;
        _prefs = &prefs;

        initContacts();

        if (!store.loadIdentity(self_id)) {
            printf("Meck: generating new identity...\n");
            P4RNG rng;
            self_id = mesh::LocalIdentity(&rng);
            int retries = 0;
            while (retries < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) {
                self_id = mesh::LocalIdentity(&rng);
                retries++;
            }
            store.saveIdentity(self_id);
        }

        Mesh::begin();

        // Load channels from NVS/SD — fall back to defaults if nothing stored
        if (!loadChannels()) {
            setupDefaultChannels();
            saveChannels();  // persist defaults so they load next time
        }

        // Self-healing migration for any channel records carrying the old
        // hash-of-name secret for Public (which broke decryption against the
        // canonical network secret). Runs every boot, no-op if already
        // correct, no NVS schema bump so custom channels survive.
        migrateChannelSecrets();

        // Load contacts from NVS
        loadContactsFromStore();

        // Load channel message history from SD (per-channel ring tail)
        loadMessagesFromStore();

        printf("Meck: ready — %02X%02X%02X%02X, name='%s'\n",
               self_id.pub_key[0], self_id.pub_key[1],
               self_id.pub_key[2], self_id.pub_key[3],
               _prefs->node_name);
    }

    // ---- Main loop (call frequently from mesh task) ----

    void loop() {
        _rtc_ref.tick();
        BaseChatMesh::loop();

        // Lazy contact save — 3s delay to batch rapid changes
        if (_contacts_save_pending && millis() >= _contacts_save_at) {
            _contacts_save_pending = false;
            saveContactsToStore();
        }

        // Periodic retry of the SD-backup of the device identity. Runs
        // every 5 seconds while pending. The data store's ensureIdentity-
        // OnSD() is cheap: a flag check + (when due) an SD-mount probe.
        // Once the backup succeeds, the flag clears and subsequent calls
        // are a single bool test. Handles two scenarios:
        //   1. SD wasn't mounted when loadIdentity ran during begin().
        //   2. SD card was inserted after boot.
        // Both end with the identity safely mirrored to SD before any
        // future reflash could orphan the NVS-only copy.
        if (_store) {
            unsigned long now = millis();
            if (now - _last_identity_sd_check >= 5000) {
                _last_identity_sd_check = now;
                _store->ensureIdentityOnSD();
            }
        }

        if (_advert_enabled && _advert_interval_ms > 0) {
            unsigned long now = millis();
            if (now >= _next_advert_ms) {
                mesh::Packet* adv = createSelfAdvert(_prefs->node_name);
                if (adv) {
                    sendFlood(adv);
                    printf("Meck: sent advert as '%s'\n", _prefs->node_name);
                }
                _next_advert_ms = now + _advert_interval_ms;
            }
        }

        // Active discovery auto-stop. We don't actually need to do
        // anything on timeout — isDiscoveryActive() is computed from
        // (now - _discovery_start_ms) so it'll start returning false
        // automatically. We just flip the flag once and mark dirty so
        // the UI redraws its header ("Scanning..." → "Scan done").
        if (_discovery_active) {
            unsigned long now = millis();
            if (now - _discovery_start_ms >= P4_DISCOVERY_WINDOW_MS) {
                _discovery_active = false;
                _discovery_dirty = true;
                printf("Meck: discovery window closed — %d nodes found\n",
                       _discovered_count);
            }
        }
    }

    // ---- Send a message on a channel (with local echo) ----

    bool sendChannelMessage(uint8_t channel_idx, const char* text) {
        ChannelDetails ch;
        if (!getChannel(channel_idx, ch)) return false;
        uint32_t ts = _rtc_ref.getCurrentTime();

        // Flip P4MeshTables into "marking outgoing" mode for the duration
        // of the send. The Mesh layer's sendFlood will call hasSeen() on
        // the just-created packet to mark it; our override captures the
        // packet hash for echo-recognition without adding to the seen
        // table. The marker is cleared as soon as sendGroupMessage
        // returns so unrelated background packet processing reverts to
        // normal dedup.
        meck_tables_begin_outgoing();
        bool ok = sendGroupMessage(ts, ch.channel, _prefs->node_name, text, strlen(text));
        meck_tables_end_outgoing();
        if (ok) {
            // Mark this (timestamp, channel) so flood-relayed copies of our
            // own packet that bounce back via the mesh don't get re-stashed
            // by onChannelMessageRecv as fresh incoming messages.
            _recent_outgoing_ts[_recent_outgoing_idx] = ts;
            _recent_outgoing_ch[_recent_outgoing_idx] = channel_idx;
            _recent_outgoing_idx = (_recent_outgoing_idx + 1) % RECENT_OUTGOING_RING;

            // Local echo — add to message history so it shows on screen
            char echo[P4_MSG_TEXT_LEN];
            snprintf(echo, sizeof(echo), "%s: %s", _prefs->node_name, text);

            xSemaphoreTake(_mutex, portMAX_DELAY);
            P4ChannelMessage* ring = (channel_idx < MAX_GROUP_CHANNELS)
                                       ? _msgs_ch[channel_idx] : nullptr;
            P4ChannelMessage saved_copy;
            int wrote_idx = -1;
            if (ring) {
                _msg_newest_ch[channel_idx] = (_msg_newest_ch[channel_idx] + 1) % P4_MSG_PER_CHANNEL;
                P4ChannelMessage& m = ring[_msg_newest_ch[channel_idx]];
                m.timestamp = ts;
                m.channel_idx = channel_idx;
                m.path_len = 0;  // local
                m.heard_count = 0;  // no echoes yet — increments as repeaters re-flood
                m.valid = true;
                m.file_offset = 0;  // not yet persisted — queue drain sets this after append
                strncpy(m.text, echo, P4_MSG_TEXT_LEN - 1);
                m.text[P4_MSG_TEXT_LEN - 1] = '\0';
                if (_msg_count_ch[channel_idx] < P4_MSG_PER_CHANNEL) _msg_count_ch[channel_idx]++;
                _msg_dirty = true;
                saved_copy = m;
                wrote_idx = _msg_newest_ch[channel_idx];
            }
            xSemaphoreGive(_mutex);

            // Queue the SD save outside the mutex (deferred to meck_task).
            // ring_idx lets the drain write file_offset back to this exact
            // slot once the initial append returns its position on disk.
            if (wrote_idx >= 0) {
                meck_request_save_message(channel_idx, wrote_idx, &saved_copy);
            }
        }
        return ok;
    }

    // =====================================================================
    // Channel management — add, delete, save, load
    // =====================================================================

    // Add a hashtag channel by name. SHA-256s the name, uses first 16 bytes
    // as the 128-bit channel secret. Matches Meck and all MeshCore nodes.
    void addHashChannel(const char* name) {
        // SHA-256 the channel name → first 16 bytes become the secret
        ChannelDetails newCh;
        memset(&newCh, 0, sizeof(newCh));
        strncpy(newCh.name, name, sizeof(newCh.name) - 1);
        newCh.name[sizeof(newCh.name) - 1] = '\0';

        uint8_t hash[32];
        mesh::Utils::sha256(hash, 32, (const uint8_t*)name, strlen(name));
        memcpy(newCh.channel.secret, hash, 16);
        // Upper 16 bytes left as zero → setChannel uses 128-bit mode

        // Find next empty slot
        for (uint8_t i = 0; i < MAX_GROUP_CHANNELS; i++) {
            ChannelDetails existing;
            if (!getChannel(i, existing) || existing.name[0] == '\0') {
                if (setChannel(i, newCh)) {
                    saveChannels();
                    printf("Meck: added channel '%s' at idx %d\n", name, i);
                }
                return;
            }
        }
        printf("Meck: no empty slot for channel '%s'\n", name);
    }

    // Delete a channel by index, compact remaining channels down.
    void deleteChannel(uint8_t idx) {
        // Find total channel count
        int total = 0;
        for (uint8_t i = 0; i < MAX_GROUP_CHANNELS; i++) {
            ChannelDetails ch;
            if (getChannel(i, ch) && ch.name[0] != '\0') {
                total = i + 1;
            } else {
                break;
            }
        }

        // Drop the message history file for the slot being deleted, then
        // shift the remaining files down so each ch_<N>.bin stays aligned
        // with the channel that ends up at slot N after the compact.
        if (_store) {
            _store->deleteChannelMessageFile(idx);
            for (int i = idx; i < total - 1; i++) {
                _store->renameChannelMessageFile((uint8_t)(i + 1), (uint8_t)i);
            }
        }

        // Compact the in-RAM message rings the same way: shift each
        // surviving ring down one slot, then clear the now-vacated tail
        // slot. getMessages reads from the ring exclusively, so without
        // this step the new channel that ends up at slot N would inherit
        // the deleted channel's messages from RAM even though its file
        // on SD is now correct.
        xSemaphoreTake(_mutex, portMAX_DELAY);
        for (int i = idx; i < total - 1; i++) {
            if (_msgs_ch[i] && _msgs_ch[i + 1]) {
                memcpy(_msgs_ch[i], _msgs_ch[i + 1],
                       P4_MSG_PER_CHANNEL * sizeof(P4ChannelMessage));
            }
            _msg_count_ch[i]  = _msg_count_ch[i + 1];
            _msg_newest_ch[i] = _msg_newest_ch[i + 1];
            _msg_unread_ch[i] = _msg_unread_ch[i + 1];
        }
        if (total - 1 >= 0 && _msgs_ch[total - 1]) {
            memset(_msgs_ch[total - 1], 0,
                   P4_MSG_PER_CHANNEL * sizeof(P4ChannelMessage));
        }
        if (total - 1 >= 0) {
            _msg_count_ch[total - 1]  = 0;
            _msg_newest_ch[total - 1] = -1;
            _msg_unread_ch[total - 1] = 0;
        }
        xSemaphoreGive(_mutex);

        // Shift channels down to fill the gap
        for (int i = idx; i < total - 1; i++) {
            ChannelDetails next;
            if (getChannel(i + 1, next)) {
                setChannel(i, next);
            }
        }

        // Clear the last slot
        ChannelDetails empty;
        memset(&empty, 0, sizeof(empty));
        setChannel(total - 1, empty);
        saveChannels();
        printf("Meck: deleted channel idx %d, compacted %d channels\n", idx, total);
    }

    // Count active channels
    int getActiveChannelCount() {
        int count = 0;
        for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
            ChannelDetails ch;
            if (getChannel(i, ch) && ch.name[0] != '\0') {
                count++;
            } else {
                break;  // channels are contiguous
            }
        }
        return count;
    }

    // Save all channels to P4DataStore (NVS + SD backup)
    void saveChannels() {
        if (!_store) return;

        P4ChannelRecord records[MAX_GROUP_CHANNELS];
        memset(records, 0, sizeof(records));

        for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
            ChannelDetails ch;
            if (getChannel(i, ch) && ch.name[0] != '\0') {
                strncpy(records[i].name, ch.name, P4_CHANNEL_NAME_MAX - 1);
                memcpy(records[i].secret, ch.channel.secret, P4_CHANNEL_SECRET_LEN);
                records[i].active = 1;
            }
        }
        _store->saveChannels(records, MAX_GROUP_CHANNELS);
    }

    // Load channels from P4DataStore. Returns true if channels were loaded.
    bool loadChannels() {
        if (!_store) return false;

        P4ChannelRecord records[MAX_GROUP_CHANNELS];
        int count = 0;

        if (!_store->loadChannels(records, MAX_GROUP_CHANNELS, count)) {
            return false;
        }

        // Apply loaded channels via setChannel (computes hashes)
        int loaded = 0;
        for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
            if (records[i].active && records[i].name[0] != '\0') {
                ChannelDetails ch;
                memset(&ch, 0, sizeof(ch));
                strncpy(ch.name, records[i].name, sizeof(ch.name) - 1);
                memcpy(ch.channel.secret, records[i].secret, 32);
                setChannel(i, ch);
                loaded++;
            }
        }
        printf("Meck: loaded %d channels from store\n", loaded);
        return (loaded > 0);
    }

    // Save prefs to P4DataStore (NVS + SD backup)
    void savePrefs() {
        if (_store && _prefs) {
            _store->savePrefs(*_prefs);
        }
    }

    // ---- Contact persistence ----

    // Schedule a deferred contact save (3s delay to batch rapid adds)
    void scheduleLazyContactSave() {
        _contacts_save_pending = true;
        _contacts_save_at = millis() + 3000;
    }

    // Save all contacts to NVS via P4DataStore
    void saveContactsToStore() {
        if (!_store) return;
        int n = getNumContacts();
        if (n == 0) {
            // Save empty set to clear old data
            _store->saveContacts(nullptr, 0);
            return;
        }

        P4DataStore::ContactRecord* records =
            (P4DataStore::ContactRecord*)malloc(sizeof(P4DataStore::ContactRecord) * n);
        if (!records) {
            printf("Meck: saveContacts malloc failed\n");
            return;
        }

        int saved = 0;
        for (int i = 0; i < n; i++) {
            ContactInfo ci;
            if (!getContactByIdx(i, ci)) continue;
            P4DataStore::ContactRecord& r = records[saved];
            memcpy(r.pub_key, ci.id.pub_key, PUB_KEY_SIZE);
            strncpy(r.name, ci.name, sizeof(r.name) - 1);
            r.name[sizeof(r.name) - 1] = '\0';
            r.type = ci.type;
            r.flags = ci.flags;
            r.gps_lat = ci.gps_lat;
            r.gps_lon = ci.gps_lon;
            r.lastmod = ci.lastmod;
            r.last_advert_timestamp = ci.last_advert_timestamp;
            saved++;
        }

        _store->saveContacts(records, saved);
        free(records);
        printf("Meck: %d contacts saved to NVS\n", saved);
    }

    // Load contacts from NVS at startup
    void loadContactsFromStore() {
        if (!_store) return;

        // Allocate in PSRAM heap. MAX_CONTACTS * sizeof(ContactRecord) is ~126KB,
        // which would overflow any reasonable task stack.
        P4DataStore::ContactRecord* records = (P4DataStore::ContactRecord*)
            heap_caps_malloc(sizeof(P4DataStore::ContactRecord) * MAX_CONTACTS,
                             MALLOC_CAP_SPIRAM);
        if (!records) {
            printf("Meck: loadContactsFromStore malloc failed\n");
            return;
        }
        int count = _store->loadContacts(records, MAX_CONTACTS);
        if (count == 0) { free(records); return; }

        int loaded = 0;
        for (int i = 0; i < count; i++) {
            ContactInfo ci;
            memset(&ci, 0, sizeof(ci));
            memcpy(ci.id.pub_key, records[i].pub_key, PUB_KEY_SIZE);
            strncpy(ci.name, records[i].name, sizeof(ci.name) - 1);
            ci.name[sizeof(ci.name) - 1] = '\0';
            ci.type = records[i].type;
            ci.flags = records[i].flags;
            ci.out_path_len = OUT_PATH_UNKNOWN;  // paths are transient
            ci.shared_secret_valid = false;
            ci.gps_lat = records[i].gps_lat;
            ci.gps_lon = records[i].gps_lon;
            ci.lastmod = records[i].lastmod;
            ci.last_advert_timestamp = records[i].last_advert_timestamp;
            ci.sync_since = 0;
            if (addContact(ci)) loaded++;
        }
        free(records);
        printf("Meck: %d contacts loaded from NVS\n", loaded);
    }

    // ---- Channel message persistence (load from SD on boot) ----
    //
    // For each channel, opens /sdcard/meshcore/messages/ch_<idx>.bin and
    // loads up to P4_MSG_PER_CHANNEL most recent records into the in-RAM
    // ring. The file may have grown beyond the ring size; only the tail
    // is loaded. Sets _msg_count_ch and _msg_newest_ch to match.
    void loadMessagesFromStore() {
        if (!_store) return;

        // Buffer for one channel's tail. ~160KB on stack would overflow,
        // so allocate from PSRAM heap (we already have plenty there).
        size_t buf_size = P4_MSG_PER_CHANNEL * sizeof(P4MsgFileRecord);
        P4MsgFileRecord* buf = (P4MsgFileRecord*)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
        // Parallel offsets array so each loaded record knows where it
        // lives on disk — needed for future in-place rewrites of heard_count
        // when echoes arrive (though in practice loaded records won't see
        // echoes since the recent-outgoing ring is rebuilt at boot).
        uint32_t* offs = (uint32_t*)heap_caps_malloc(
            P4_MSG_PER_CHANNEL * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
        if (!buf || !offs) {
            printf("Meck: loadMessagesFromStore malloc failed\n");
            if (buf) free(buf);
            if (offs) free(offs);
            return;
        }

        int total_loaded = 0;
        for (uint8_t ch = 0; ch < MAX_GROUP_CHANNELS; ch++) {
            P4ChannelMessage* ring = _msgs_ch[ch];
            if (!ring) continue;

            int n = _store->loadChannelMessageTail(
                ch,
                P4_MSG_FILE_MAGIC, P4_MSG_FILE_VERSION,
                (uint16_t)sizeof(P4MsgFileRecord),
                buf, offs, P4_MSG_PER_CHANNEL);
            if (n <= 0) continue;

            // Records are in chronological order (oldest first). Copy into
            // the ring at indices 0..n-1; newest sits at index n-1.
            xSemaphoreTake(_mutex, portMAX_DELAY);
            for (int i = 0; i < n; i++) {
                P4ChannelMessage& m = ring[i];
                m.timestamp = buf[i].timestamp;
                m.channel_idx = buf[i].channel_idx;
                m.path_len = buf[i].path_len;
                // heard_count restored from schema v2 (or zero on a v1
                // record, since v1's byte at this offset was reserved zero).
                m.heard_count = buf[i].heard_count;
                m.valid = (buf[i].flags & 0x01) != 0;
                m.file_offset = offs[i];
                memcpy(m.text, buf[i].text, P4_MSG_TEXT_LEN);
                m.text[P4_MSG_TEXT_LEN - 1] = '\0';
            }
            _msg_count_ch[ch]  = n;
            _msg_newest_ch[ch] = n - 1;
            _msg_dirty = true;
            xSemaphoreGive(_mutex);
            total_loaded += n;
        }

        free(buf);
        free(offs);
        printf("Meck: loadMessagesFromStore — %d total messages loaded across %d channels\n",
               total_loaded, MAX_GROUP_CHANNELS);
    }

    // ---- Accessors ----

    const char* getNodeName() const { return _prefs ? _prefs->node_name : "NONAME"; }
    P4NodePrefs* getNodePrefs() { return _prefs; }

    // ---- Contact flag mutation ----
    //
    // BaseChatMesh::addContact() appends a new slot — it does NOT upsert by
    // pubkey. To mutate an existing contact we must removeContact() first,
    // then addContact() the modified copy. The contact ends up at the tail
    // of the table (entries after it shift down by one); for our use case
    // (favourite toggle) the change in row order is acceptable.
    //
    // Pair with scheduleLazyContactSave so the change makes it to NVS within
    // ~3 s.
    bool setContactFlags(int idx, uint8_t new_flags) {
        ContactInfo ci;
        if (!getContactByIdx(idx, ci)) return false;
        if (ci.flags == new_flags) return true;          // no-op
        ci.flags = new_flags;

        if (!removeContact(ci)) {
            printf("Meck: setContactFlags removeContact failed for '%s'\n",
                   ci.name);
            return false;
        }
        if (!addContact(ci)) {
            // Should be impossible — we just freed a slot. But if it does
            // happen the contact is now lost from the table; warn loudly.
            printf("Meck: setContactFlags addContact failed for '%s' "
                   "(contact lost!)\n", ci.name);
            return false;
        }

        scheduleLazyContactSave();
        printf("Meck: contact '%s' flags = 0x%02X\n", ci.name, new_flags);
        return true;
    }

    // Convenience: flip bit 0 (favourite) for a contact by table index.
    bool toggleContactFavourite(int idx) {
        ContactInfo ci;
        if (!getContactByIdx(idx, ci)) return false;
        return setContactFlags(idx, ci.flags ^ 0x01);
    }

    // Delete a contact from the in-memory table by index. Persists via the
    // lazy-save queue. Useful for clearing duplicates that were created by
    // earlier (broken) toggleContactFavourite paths.
    bool deleteContactByIdx(int idx) {
        ContactInfo ci;
        if (!getContactByIdx(idx, ci)) return false;
        if (!removeContact(ci)) {
            printf("Meck: deleteContactByIdx failed for '%s'\n", ci.name);
            return false;
        }
        scheduleLazyContactSave();
        printf("Meck: deleted contact '%s'\n", ci.name);
        return true;
    }
    P4DataStore* getDataStore() { return _store; }

    // ---- Thread-safe accessors for LVGL UI ----

    bool isMessageDirty() {
        bool d = _msg_dirty;
        _msg_dirty = false;
        return d;
    }

    bool isRecentDirty() {
        bool d = _recent_dirty;
        _recent_dirty = false;
        return d;
    }

    int getUnreadCount(int ch = -1) const {
        if (ch >= 0 && ch < MAX_GROUP_CHANNELS) return _msg_unread_ch[ch];
        int total = 0;
        for (int i = 0; i < MAX_GROUP_CHANNELS; i++) total += _msg_unread_ch[i];
        return total;
    }
    void clearUnread(int ch = -1) {
        if (ch >= 0 && ch < MAX_GROUP_CHANNELS) { _msg_unread_ch[ch] = 0; return; }
        memset((void*)_msg_unread_ch, 0, sizeof(_msg_unread_ch));
    }

    int getMessages(P4ChannelMessage* dest, int max_count, uint8_t channel_idx) {
        if (channel_idx >= MAX_GROUP_CHANNELS) return 0;
        P4ChannelMessage* ring = _msgs_ch[channel_idx];
        if (!ring) return 0;

        xSemaphoreTake(_mutex, portMAX_DELAY);
        int newest = _msg_newest_ch[channel_idx];
        int count  = _msg_count_ch[channel_idx];
        int out = 0;
        for (int i = 0; i < count && out < max_count; i++) {
            int idx = (newest - i + P4_MSG_PER_CHANNEL) % P4_MSG_PER_CHANNEL;
            if (!ring[idx].valid) continue;
            dest[out++] = ring[idx];
        }
        xSemaphoreGive(_mutex);
        return out;
    }

    int getRecentHeard(P4RecentHeard* dest, int max_count) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        int out = 0;
        for (int i = 0; i < P4_RECENT_HEARD_SIZE && out < max_count; i++) {
            int idx = (_recent_newest - i + P4_RECENT_HEARD_SIZE) % P4_RECENT_HEARD_SIZE;
            if (!_recent[idx].valid) continue;
            dest[out++] = _recent[idx];
        }
        xSemaphoreGive(_mutex);
        return out;
    }

    // =====================================================================
    // Discover support (repeaters-only, mirroring Meck T-Deck Pro behaviour)
    // ---------------------------------------------------------------------
    // The Discover screen surfaces nearby repeaters from the same _recent[]
    // ring the Last Heard list reads, filtered to ADV_TYPE_REPEATER. No
    // separate "active scan" buffer is needed — adverts are received
    // passively and the ring already de-dupes by pubkey on each
    // onAdvertRecv. The screen's Rescan button calls sendDiscoveryProbe()
    // below, which floods a self-advert; repeaters that hear it will
    // typically not re-advert in response (MeshCore repeaters advert on
    // their own schedule), but it announces our presence so subsequent
    // peer-initiated traffic includes us as a known node, and it gives
    // the user a clear "I did something" affordance.
    // =====================================================================

    // Returns only entries whose advert type is REPEATER (ADV_TYPE_REPEATER
    // == 2). Sort order matches getRecentHeard: newest first via the
    // _recent_newest cursor walk.
    int getRecentRepeaters(P4RecentHeard* dest, int max_count) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        int out = 0;
        for (int i = 0; i < P4_RECENT_HEARD_SIZE && out < max_count; i++) {
            int idx = (_recent_newest - i + P4_RECENT_HEARD_SIZE) % P4_RECENT_HEARD_SIZE;
            if (!_recent[idx].valid) continue;
            if (_recent[idx].adv_type != 2 /* ADV_TYPE_REPEATER */) continue;
            dest[out++] = _recent[idx];
        }
        xSemaphoreGive(_mutex);
        return out;
    }

    // O(N) scan of the contact table looking for a matching pub_key.
    // Used by the Discover screen to draw "[+]" against entries already
    // in the local contact list. Returns true on first match.
    //
    // Compares the full 32-byte key, not the 4-byte prefix — the prefix
    // can collide between unrelated nodes and would produce false "[+]"
    // markers on the Discover screen.
    bool hasContactWithPubKey(const uint8_t* pub_key) {
        int n = getNumContacts();
        for (int i = 0; i < n; i++) {
            ContactInfo ci;
            if (!getContactByIdx(i, ci)) continue;
            if (memcmp(ci.id.pub_key, pub_key, PUB_KEY_SIZE) == 0) return true;
        }
        return false;
    }

    // Build a ContactInfo from a previously-heard advert snapshot and
    // append it to the contact table. Returns true on success.
    //
    // Notes:
    //  - shared_secret is left uncomputed (shared_secret_valid=false).
    //    The first encrypted exchange will compute and cache it on
    //    demand via the standard BaseChatMesh path.
    //  - out_path_len is OUT_PATH_UNKNOWN so the first DM to the contact
    //    will fall back to flood routing until a path return arrives.
    //  - flags = 0 (no favourite). The user can toggle that later from
    //    the contact detail screen.
    //  - last_advert_timestamp is taken from the snapshot so the contact
    //    appears at the appropriate position in "recently heard" sorts.
    //  - addContact() is the appropriate API: it appends if a free slot
    //    exists. Mirrors the same call path loadContactsFromStore uses.
    bool addContactFromRecent(const P4RecentHeard& r) {
        if (hasContactWithPubKey(r.pub_key)) return true;  // already in
        ContactInfo ci;
        memset(&ci, 0, sizeof(ci));
        memcpy(ci.id.pub_key, r.pub_key, PUB_KEY_SIZE);
        strncpy(ci.name, r.name, sizeof(ci.name) - 1);
        ci.name[sizeof(ci.name) - 1] = '\0';
        ci.type = r.adv_type;
        ci.flags = 0;
        ci.out_path_len = OUT_PATH_UNKNOWN;
        ci.shared_secret_valid = false;
        ci.gps_lat = 0;
        ci.gps_lon = 0;
        ci.lastmod = r.timestamp;
        ci.last_advert_timestamp = r.timestamp;
        ci.sync_since = 0;
        if (!addContact(ci)) {
            printf("Meck: addContactFromRecent: addContact failed "
                   "(table full?) for '%s'\n", r.name);
            return false;
        }
        scheduleLazyContactSave();
        printf("Meck: added contact from heard advert '%s' [%02X%02X%02X%02X]\n",
               ci.name, ci.id.pub_key[0], ci.id.pub_key[1],
               ci.id.pub_key[2], ci.id.pub_key[3]);
        return true;
    }

    // Same as addContactFromRecent but takes a DiscoveredNode (the
    // active-scan equivalent of P4RecentHeard). DiscoveredNode doesn't
    // carry a timestamp — we use the current RTC time for lastmod and
    // last_advert_timestamp since we *just* heard the advert.
    bool addContactFromDiscovered(const DiscoveredNode& d) {
        if (hasContactWithPubKey(d.pub_key)) return true;
        ContactInfo ci;
        memset(&ci, 0, sizeof(ci));
        memcpy(ci.id.pub_key, d.pub_key, PUB_KEY_SIZE);
        strncpy(ci.name, d.name, sizeof(ci.name) - 1);
        ci.name[sizeof(ci.name) - 1] = '\0';
        ci.type = d.adv_type;
        ci.flags = 0;
        ci.out_path_len = OUT_PATH_UNKNOWN;
        ci.shared_secret_valid = false;
        ci.gps_lat = 0;
        ci.gps_lon = 0;
        uint32_t now_ts = _rtc_ref.getCurrentTime();
        ci.lastmod = now_ts;
        ci.last_advert_timestamp = now_ts;
        ci.sync_since = 0;
        if (!addContact(ci)) {
            printf("Meck: addContactFromDiscovered: addContact failed "
                   "(table full?) for '%s'\n", d.name);
            return false;
        }
        scheduleLazyContactSave();
        printf("Meck: added contact from discovery '%s' [%02X%02X%02X%02X]\n",
               ci.name, ci.id.pub_key[0], ci.id.pub_key[1],
               ci.id.pub_key[2], ci.id.pub_key[3]);
        return true;
    }

    // ---- Active Discovery API ----
    //
    // startDiscovery() is the active scan. It:
    //   1. Clears the _discovered[] ring.
    //   2. Pre-seeds it with our cached _recent[] entries (marked snr_x4=0
    //      so the UI shows them as hop-count cached rather than fresh).
    //   3. Transmits a zero-hop self-advert (path_len=0, ROUTE_TYPE_DIRECT)
    //      via sendDirect(adv, nullptr, 0) — this asks for replies from any
    //      neighbour with zero-hop response enabled.
    //   4. Sets _discovery_active for P4_DISCOVERY_WINDOW_MS; during that
    //      window onAdvertRecv writes incoming adverts to _discovered[].
    //
    // Per the MeshCore wiki "set advert.interval {minutes}" CLI command,
    // a repeater only sends a zero-hop response advert if its operator has
    // set that interval > 0. Many repeaters default to 0 (Switzerland docs
    // call out that explicitly). So a "no response" outcome is normal and
    // doesn't necessarily mean nothing's there — operators have to opt in.
    void startDiscovery() {
        if (!_prefs) return;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        memset(_discovered, 0, sizeof(_discovered));
        _discovered_count = 0;

        _discovery_active = true;
        _discovery_start_ms = millis();
        _discovery_dirty = true;
        xSemaphoreGive(_mutex);

        // PAYLOAD_TYPE_CONTROL (0x0B) DISCOVER_REQ. Wire format
        // (matches upstream Meck MyMesh.cpp:3539-3565 — the published
        // DeepWiki bit numbering is misleading, the upstream source
        // is canonical):
        //
        //   [0]: flags = 0x80
        //         upper nibble 0x8 = DISCOVER_REQ subtype, lower
        //         nibble = prefix_only flag (0 = full 32-byte pubkey
        //         in response, 1 = 8-byte prefix only).
        //   [1]: type_filter = (1 << ADV_TYPE_REPEATER) | (1 << ADV_TYPE_ROOM)
        //         = (1 << 2) | (1 << 3) = 0x0C.
        //         CRITICAL: the bit position is the literal ADV_TYPE_*
        //         value (chat=1 → bit 1, repeater=2 → bit 2, room=3
        //         → bit 3, sensor=4 → bit 4), NOT 0-indexed. This is
        //         the same convention as our shouldAutoAddContactType
        //         bits and as upstream's AUTO_ADD_REPEATER (1 << 2).
        //         Setting type_filter=0x02 (bit 1) means "I want chat
        //         nodes only", and a repeater receiving such a REQ
        //         silently drops it.
        //   [2..5]: tag (uint32 LE) — random per-scan correlation ID.
        //           Upstream uses RNG range [1, 0xFFFFFFFF) so we
        //           never emit tag=0.
        //   [6..9]: since (uint32 LE) — zero. Marked optional in the
        //           docs but upstream firmware sends it; older
        //           repeaters may require the fixed 10-byte length.
        _discovery_tag = (uint32_t)getRNG()->nextInt(1, 0x7FFFFFFF);
        uint8_t ctl_payload[10];
        ctl_payload[0] = 0x80;  // DISCOVER_REQ, prefix_only=0
        ctl_payload[1] = (1u << 2) | (1u << 3);  // repeaters + rooms
        memcpy(&ctl_payload[2], &_discovery_tag, 4);
        uint32_t since = 0;
        memcpy(&ctl_payload[6], &since, 4);

        mesh::Packet* pkt = createControlData(ctl_payload, sizeof(ctl_payload));
        if (pkt) {
            sendZeroHop(pkt);
            printf("Meck: discovery started — DISCOVER_REQ tag=%08X, "
                   "type_filter=0x%02X (repeaters+rooms), "
                   "%d cached pre-seeded, %d ms window\n",
                   (unsigned)_discovery_tag, (unsigned)ctl_payload[1],
                   _discovered_count, (int)P4_DISCOVERY_WINDOW_MS);
        } else {
            printf("Meck: discovery — createControlData failed, pool full?\n");
        }
    }

    void stopDiscovery() {
        if (_discovery_active) {
            _discovery_active = false;
            _discovery_dirty = true;
        }
    }

    bool isDiscoveryActive() const {
        if (!_discovery_active) return false;
        // Belt-and-braces — loop() flips this off when the window
        // expires, but UI consumers reading without waiting for loop()
        // tick still get a correct answer.
        return (millis() - _discovery_start_ms) < P4_DISCOVERY_WINDOW_MS;
    }

    int getDiscoveredCount() const { return _discovered_count; }

    const DiscoveredNode& getDiscovered(int i) const {
        // Caller must check getDiscoveredCount() first. Returning by ref
        // to match the T-Deck pattern in DiscoveryScreen.h.
        return _discovered[i];
    }

    // Returns true if the UI should redraw (any change since last poll).
    // Resets the dirty flag.
    bool consumeDiscoveryDirty() {
        if (!_discovery_dirty) return false;
        _discovery_dirty = false;
        return true;
    }

    // Backwards-compat shim. Older call sites referenced sendDiscoveryProbe
    // (a flood advert with no follow-up). Forward to the new active scan
    // so existing UI buttons keep working — same semantic intent, better
    // implementation.
    void sendDiscoveryProbe() { startDiscovery(); }

    // Write back the on-disk file offset for a freshly-appended message
    // record. Called by the save-queue drain after appendChannelMessageRecord
    // returns the position where it placed the bytes. The expected_timestamp
    // guards against the rare race where the ring slot has been overwritten
    // by a newer message between queue and drain — if the timestamps don't
    // match, we silently skip rather than clobber the new entry's offset.
    bool setMessageFileOffset(uint8_t channel_idx, int ring_idx,
                               uint32_t expected_timestamp, uint32_t file_offset) {
        if (channel_idx >= MAX_GROUP_CHANNELS) return false;
        if (ring_idx < 0 || ring_idx >= P4_MSG_PER_CHANNEL) return false;
        P4ChannelMessage* ring = _msgs_ch[channel_idx];
        if (!ring) return false;
        bool ok = false;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        P4ChannelMessage& m = ring[ring_idx];
        if (m.valid && m.timestamp == expected_timestamp) {
            m.file_offset = file_offset;
            ok = true;
        }
        xSemaphoreGive(_mutex);
        return ok;
    }

    // Snapshot a ring slot's current state into both an on-disk record and
    // the slot's file_offset. Called by the save-queue drain so it sees the
    // latest in-memory state (including any heard_count or file_offset
    // changes made between enqueue and drain), rather than trusting the
    // possibly-stale copy captured at enqueue time.
    //
    // expected_timestamp guards against ring-slot reuse: if the slot has
    // been overwritten by a newer message with a different timestamp, the
    // snapshot fails and the caller skips this drain entry.
    //
    // Returns true if the slot is valid and timestamp matches.
    bool buildMessageRecordSnapshot(uint8_t channel_idx, int ring_idx,
                                     uint32_t expected_timestamp,
                                     P4MsgFileRecord* out_rec,
                                     uint32_t* out_file_offset) {
        if (!out_rec || !out_file_offset) return false;
        if (channel_idx >= MAX_GROUP_CHANNELS) return false;
        if (ring_idx < 0 || ring_idx >= P4_MSG_PER_CHANNEL) return false;
        P4ChannelMessage* ring = _msgs_ch[channel_idx];
        if (!ring) return false;
        bool ok = false;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        P4ChannelMessage& m = ring[ring_idx];
        if (m.valid && m.timestamp == expected_timestamp) {
            memset(out_rec, 0, sizeof(*out_rec));
            out_rec->timestamp    = m.timestamp;
            out_rec->dm_peer_hash = 0;                        // reserved
            out_rec->channel_idx  = m.channel_idx;
            out_rec->path_len     = m.path_len;
            out_rec->snr_x4       = 0;                        // reserved (schema v3+)
            out_rec->flags        = m.valid ? 0x01 : 0x00;
            out_rec->heard_count  = m.heard_count;
            // out_rec->path[] left zeroed (reserved for schema v3)
            memcpy(out_rec->text, m.text, P4_MSG_TEXT_LEN);
            *out_file_offset = m.file_offset;
            ok = true;
        }
        xSemaphoreGive(_mutex);
        return ok;
    }

    void setAdvertEnabled(bool en) { _advert_enabled = en; }
    bool isAdvertEnabled() const { return _advert_enabled; }
    const mesh::LocalIdentity& getIdentity() const { return self_id; }
    bool isClockSynced() const { return _rtc_ref.getCurrentTime() > 1750000000; }

protected:
    // ---- BaseChatMesh pure virtual overrides ----

    void onChannelMessageRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt,
                               uint32_t timestamp, const char* text) override {
        uint8_t ch_idx = findChannelIdx(channel);

        // Self-echo dedup: if this is our own packet bouncing back via flood,
        // drop it. We already wrote a local-echo entry at send time. Update
        // that entry's path_len to reflect the real flood-relay path so the
        // user sees their message was successfully relayed, AND bump
        // heard_count so we accumulate a "heard by N repeaters" indicator
        // as additional repeaters re-flood the same packet. Each increment
        // also queues a save so the count is rewritten in-place at the
        // record's existing file_offset on SD (set on the initial append).
        if (isOurOwnEcho(timestamp, ch_idx)) {
            uint8_t real_path_len = pkt->isRouteFlood() ? pkt->path_len : 0xFE;
            // Debug: lets you confirm in serial whether a sent packet bounced
            // back through any repeaters at all, and via how many hops. If
            // nothing prints in the seconds after a send, no repeater in
            // earshot picked it up.
            printf("Meck: heard own echo ch[%d] ts=%u path_len=0x%02X flood=%d\n",
                   (int)ch_idx, (unsigned)timestamp, (unsigned)real_path_len,
                   (int)pkt->isRouteFlood());
            xSemaphoreTake(_mutex, portMAX_DELAY);
            P4ChannelMessage* ring = (ch_idx < MAX_GROUP_CHANNELS) ? _msgs_ch[ch_idx] : nullptr;
            P4ChannelMessage saved_copy;
            int matched_idx = -1;
            if (ring) {
                int newest = _msg_newest_ch[ch_idx];
                int count  = _msg_count_ch[ch_idx];
                for (int i = 0; i < count; i++) {
                    int idx = (newest + P4_MSG_PER_CHANNEL - i) % P4_MSG_PER_CHANNEL;
                    P4ChannelMessage& existing = ring[idx];
                    if (existing.valid && existing.timestamp == timestamp) {
                        // First echo: capture real path_len. Subsequent
                        // echoes don't overwrite — we keep the first echo's
                        // path as representative.
                        if (existing.path_len == 0) {
                            existing.path_len = real_path_len;
                        }
                        // Every echo from a distinct repeater bumps the
                        // heard counter. Saturate at 255 so we never wrap.
                        if (existing.heard_count < 0xFF) {
                            existing.heard_count++;
                        }
                        printf("Meck:   heard_count now %u (file_offset=%u)\n",
                               (unsigned)existing.heard_count,
                               (unsigned)existing.file_offset);
                        _msg_dirty = true;
                        saved_copy = existing;
                        matched_idx = idx;
                        break;
                    }
                }
            }
            xSemaphoreGive(_mutex);

            // Queue an in-place rewrite. The drain sees file_offset != 0 on
            // the captured copy and seeks to that offset rather than
            // appending a duplicate. If file_offset is still 0 (initial
            // append hasn't drained yet), the drain coalesces and the
            // single eventual append captures the updated heard_count.
            if (matched_idx >= 0) {
                meck_request_save_message(ch_idx, matched_idx, &saved_copy);
            }
            return;
        }

        uint8_t path_len = pkt->isRouteFlood() ? pkt->path_len : 0xFF;

        printf("Meck: channel[%d] msg: %s\n", ch_idx, text);

        xSemaphoreTake(_mutex, portMAX_DELAY);
        P4ChannelMessage* ring = (ch_idx < MAX_GROUP_CHANNELS) ? _msgs_ch[ch_idx] : nullptr;
        P4ChannelMessage saved_copy;
        int wrote_idx = -1;
        if (ring) {
            _msg_newest_ch[ch_idx] = (_msg_newest_ch[ch_idx] + 1) % P4_MSG_PER_CHANNEL;
            P4ChannelMessage& m = ring[_msg_newest_ch[ch_idx]];
            m.timestamp = timestamp;
            m.channel_idx = ch_idx;
            m.path_len = path_len;
            m.heard_count = 0;  // unused for incoming, but keep deterministic
            m.valid = true;
            m.file_offset = 0;  // queue drain sets this after initial append
            strncpy(m.text, text, P4_MSG_TEXT_LEN - 1);
            m.text[P4_MSG_TEXT_LEN - 1] = '\0';
            if (_msg_count_ch[ch_idx] < P4_MSG_PER_CHANNEL) _msg_count_ch[ch_idx]++;
            _msg_unread_ch[ch_idx]++;
            _msg_dirty = true;
            saved_copy = m;
            wrote_idx = _msg_newest_ch[ch_idx];
        }
        xSemaphoreGive(_mutex);

        // Queue the SD save outside the mutex (deferred to meck_task)
        if (wrote_idx >= 0) {
            meck_request_save_message(ch_idx, wrote_idx, &saved_copy);
        }
    }

    void onMessageRecv(const ContactInfo& from, mesh::Packet* pkt,
                        uint32_t sender_timestamp, const char* text) override {
        printf("Meck: DM from %s: %s\n", from.name, text);
    }

    void onCommandDataRecv(const ContactInfo& from, mesh::Packet* pkt,
                            uint32_t sender_timestamp, const char* text) override {
    }

    void onSignedMessageRecv(const ContactInfo& from, mesh::Packet* pkt,
                              uint32_t sender_timestamp,
                              const uint8_t* sender_prefix, const char* text) override {
        printf("Meck: signed msg from %s: %s\n", from.name, text);
    }

    uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override {
        return 10000 + (6 * pkt_airtime_millis);
    }

    uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override {
        uint8_t hops = path_len & 63;
        return 10000 + ((pkt_airtime_millis * 4 + 1000) * (hops + 1));
    }

    void onSendTimeout() override {}

    uint8_t onContactRequest(const ContactInfo& contact, uint32_t sender_timestamp,
                              const uint8_t* data, uint8_t len, uint8_t* reply) override {
        return 0;
    }

    void onContactResponse(const ContactInfo& contact,
                            const uint8_t* data, uint8_t len) override {}

    // ---- Advert handling (update recent heard + clock sync) ----

    void onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id,
                       uint32_t timestamp, const uint8_t* app_data, size_t app_data_len) override {
        BaseChatMesh::onAdvertRecv(packet, id, timestamp, app_data, app_data_len);

        // Clock sync: first valid sync wins, then we trust our own clock.
        // Window is deliberately wide so we don't reject the entire mesh
        // because of a missed annual constant bump. Real-world clock drift
        // / mis-set time on a peer is the main thing this catches.
        //
        // 1730419200 = 1 Nov 2025 00:00 UTC  (floor)
        // 1956528000 = 1 Jan 2032 00:00 UTC  (ceiling — covers ~5 years of
        //   forward drift; bump again well before 2031).
        const uint32_t MIN_VALID = 1730419200U;
        const uint32_t MAX_VALID = 1956528000U;
        const uint32_t SYNC_THRESHOLD = 1750000000U;  // matches isClockSynced()
        uint32_t our_time = _rtc_ref.getCurrentTime();
        bool synced = our_time >= SYNC_THRESHOLD;
        if (timestamp < MIN_VALID || timestamp >= MAX_VALID) {
            // Rogue, stale, or peer-clock-misconfigured advert. Print every
            // time so peer clock issues are visible — this sometimes catches
            // a phone whose date is set wrong, or a companion firmware bug
            // computing the wrong epoch. Include both sides for diagnosis.
            int32_t skew = (int32_t)((int64_t)timestamp - (int64_t)our_time);
            printf("Meck: advert ts out of range — advert=%lu our=%lu "
                   "skew=%+ld s, valid=%lu..%lu\n",
                   (unsigned long)timestamp, (unsigned long)our_time,
                   (long)skew,
                   (unsigned long)MIN_VALID, (unsigned long)MAX_VALID);
        } else if (!synced) {
            _rtc_ref.setCurrentTime(timestamp);
            printf("Meck: clock synced to %lu (was %lu)\n",
                   (unsigned long)timestamp, (unsigned long)our_time);
        }
        // After first sync, ignore all further timestamps until reboot or a
        // higher-authority source (GPS, NTP) lands.

        // Extract name from advert data for UI
        char name[32] = "???";
        uint8_t adv_type = 0;
        if (app_data_len > 0) {
            AdvertDataParser parser(app_data, app_data_len);
            const char* n = parser.getName();
            if (n && n[0]) {
                strncpy(name, n, sizeof(name) - 1);
                name[sizeof(name) - 1] = '\0';
            }
            adv_type = parser.getType();
        }

        float rssi = packet->getSNR() > -100 ? _radio->getLastRSSI() : 0;
        float snr = _radio->getLastSNR();
        uint8_t path_len = packet->isRouteFlood() ? packet->path_len : 0xFF;

        printf("Meck: advert from '%s' [%02X%02X] RSSI=%.0f SNR=%.1f\n",
               name, id.pub_key[0], id.pub_key[1], rssi, snr);

        xSemaphoreTake(_mutex, portMAX_DELAY);

        // Dedup: invalidate any existing entry for this node so it
        // moves to the newest slot with fresh RSSI/SNR/timestamp
        for (int i = 0; i < P4_RECENT_HEARD_SIZE; i++) {
            if (_recent[i].valid &&
                memcmp(_recent[i].pub_key_prefix, id.pub_key, 4) == 0) {
                _recent[i].valid = false;
                break;
            }
        }

        _recent_newest = (_recent_newest + 1) % P4_RECENT_HEARD_SIZE;
        P4RecentHeard& r = _recent[_recent_newest];
        strncpy(r.name, name, sizeof(r.name) - 1);
        r.name[sizeof(r.name) - 1] = '\0';
        memcpy(r.pub_key_prefix, id.pub_key, 4);
        memcpy(r.pub_key, id.pub_key, sizeof(r.pub_key));
        r.rssi = rssi;
        r.snr = snr;
        r.path_len = path_len;
        r.timestamp = timestamp;
        r.adv_type = adv_type;
        r.valid = true;
        if (_recent_count < P4_RECENT_HEARD_SIZE) _recent_count++;
        _recent_dirty = true;

        // Secondary capture path: while a discovery scan is open, any
        // flood advert from a repeater that happens to land in the
        // window is also tracked. The PRIMARY discovery signal is
        // DISCOVER_RESP control packets (see onControlDataRecv below);
        // this branch is a fallback for older repeater firmware that
        // doesn't yet implement zero-hop discovery responses but does
        // periodically flood-advert. Filtered to ADV_TYPE_REPEATER (2)
        // — Discover is for finding repeaters specifically. Other
        // types still hit _recent[] above; they just don't appear in
        // the Discover screen.
        //
        // snr_x4 is signed int8; clamp to that range. We dedup by
        // full pub_key so a node that re-adverts mid-window updates
        // in place rather than appearing twice — and so DISCOVER_RESP
        // and advert capture for the same node converge on one entry
        // (the more recent of the two wins on the SNR/name fields).
        if (_discovery_active &&
            (adv_type == 2 /* ADV_TYPE_REPEATER */ ||
             adv_type == 3 /* ADV_TYPE_ROOM */)) {
            int snr_clamped = (int)(snr * 4.0f);
            if (snr_clamped > 127) snr_clamped = 127;
            if (snr_clamped < -128) snr_clamped = -128;
            // Sentinel: 0 means "cached, no live read". If a live read
            // happens to round to 0, nudge by 1 so the UI still treats
            // it as live data.
            int8_t snr_x4 = (snr_clamped == 0) ? 1 : (int8_t)snr_clamped;

            int slot = -1;
            for (int i = 0; i < _discovered_count; i++) {
                if (_discovered[i].valid &&
                    memcmp(_discovered[i].pub_key, id.pub_key, 32) == 0) {
                    slot = i;
                    break;
                }
            }
            if (slot < 0) {
                if (_discovered_count < P4_DISCOVERED_SIZE) {
                    slot = _discovered_count++;
                } else {
                    // Full — evict. Prefer a cached entry (heard_ms == 0)
                    // if any exist, since those are stale; otherwise evict
                    // the oldest live entry. P4_DISCOVERED_SIZE is 32 so
                    // we should rarely hit this in practice.
                    slot = 0;
                    for (int i = 0; i < P4_DISCOVERED_SIZE; i++) {
                        if (_discovered[i].heard_ms == 0) { slot = i; break; }
                    }
                    if (_discovered[slot].heard_ms != 0) {
                        // No cached entries to evict — find the oldest live one.
                        unsigned long oldest = _discovered[0].heard_ms;
                        for (int i = 1; i < P4_DISCOVERED_SIZE; i++) {
                            if (_discovered[i].heard_ms < oldest) {
                                slot = i;
                                oldest = _discovered[i].heard_ms;
                            }
                        }
                    }
                }
            }

            DiscoveredNode& d = _discovered[slot];
            d.valid = true;
            strncpy(d.name, name, sizeof(d.name) - 1);
            d.name[sizeof(d.name) - 1] = '\0';
            memcpy(d.pub_key, id.pub_key, 32);
            d.adv_type  = adv_type;
            d.snr_x4    = snr_x4;
            // path_len here is 0xFF for non-flood (direct/zero-hop) in our
            // capture above; normalise that to 0 so the UI can read it as
            // "zero hops" cleanly.
            d.path_len  = (path_len == 0xFF) ? 0 : path_len;
            d.heard_ms  = millis();
            d.already_in_contacts = hasContactWithPubKey(id.pub_key);
            _discovery_dirty = true;
        }
        xSemaphoreGive(_mutex);
    }

    // ---- Other overrides ----

    void onDiscoveredContact(ContactInfo& contact, bool is_new,
                              uint8_t path_len, const uint8_t* path) override {
        if (is_new) {
            printf("Meck: new contact '%s' [%02X%02X] hops=%d\n",
                   contact.name, contact.id.pub_key[0], contact.id.pub_key[1], path_len);
            scheduleLazyContactSave();
        }
    }

    void onContactPathUpdated(const ContactInfo& contact) override {}
    bool onContactPathRecv(ContactInfo& from, uint8_t* in_path, uint8_t in_path_len,
                            uint8_t* out_path, uint8_t out_path_len,
                            uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override {
        return true;
    }

    // PAYLOAD_TYPE_CONTROL (0x0B) handler. The base Mesh class delivers
    // control packets here for application-level processing. We use it
    // to catch DISCOVER_RESP replies to the DISCOVER_REQ we sent at
    // startDiscovery(). Wire format per docs/payloads.md:
    //
    //   DISCOVER_RESP:
    //     [0]: flags  — upper nibble 0x9 (subtype), lower nibble = node_type
    //                   (1=chat, 2=repeater, 3=room, 4=sensor)
    //     [1]: snr    — SNR × 4, signed byte (responder's measured SNR
    //                   when receiving our DISCOVER_REQ)
    //     [2..5]: tag (uint32 LE) — echoes our request tag
    //     [6..]: pubkey — 8 bytes if our REQ had prefix_only=1, else 32
    //
    // We sent prefix_only=0, so we expect 32-byte pubkeys. We also
    // filtered our REQ to type_filter=0x02 (repeaters only), so any
    // well-behaved responder will be a repeater — we still sanity-check
    // node_type==2 in case of a stale REQ ID collision.
    //
    // Note: name isn't carried in DISCOVER_RESP — the protocol only
    // returns identity (pubkey) and signal info. If the responder is
    // already a contact we use that name; otherwise we display the
    // pubkey prefix and the user can tap Add to fetch a full identity
    // later via the normal advert path.
    void onControlDataRecv(mesh::Packet* pkt) override {
        BaseChatMesh::onControlDataRecv(pkt);

        if (!pkt) return;

        // UNCONDITIONAL DIAGNOSTIC: log every control packet that arrives,
        // before any subtype/tag/active filtering, so we can verify:
        //   1. that the radio is hearing CONTROL packets at all on the
        //      currently-configured channel
        //   2. that our onControlDataRecv override is being called
        //   3. the actual on-wire byte layout (any responder firmware
        //      version skew, tag byte-order issues, etc., become visible)
        //
        // Once discovery is observed working, this block should be
        // removed or gated behind a verbose-logging flag.
        {
            uint8_t flags = (pkt->payload_len > 0) ? pkt->payload[0] : 0;
            uint8_t subtype = (flags >> 4) & 0x0F;
            printf("Meck: onControlDataRecv pl_len=%u subtype=0x%X "
                   "route=%s path_len=%u snr=%.1fdB raw=",
                   (unsigned)pkt->payload_len, (unsigned)subtype,
                   pkt->isRouteFlood() ? "flood" : "direct",
                   (unsigned)pkt->path_len, pkt->getSNR());
            int dump = pkt->payload_len < 32 ? pkt->payload_len : 32;
            for (int i = 0; i < dump; i++) printf("%02X", pkt->payload[i]);
            printf("\n");
        }

        if (pkt->payload_len < 6) return;
        uint8_t flags = pkt->payload[0];
        uint8_t subtype = (flags >> 4) & 0x0F;
        if (subtype != 0x9) return;  // not DISCOVER_RESP

        uint8_t node_type = flags & 0x0F;
        int8_t  snr_x4    = (int8_t)pkt->payload[1];
        uint32_t rsp_tag = 0;
        memcpy(&rsp_tag, &pkt->payload[2], 4);

        if (!_discovery_active) {
            printf("Meck: DISCOVER_RESP arrived but discovery not active\n");
            return;
        }
        if (rsp_tag != _discovery_tag) {
            printf("Meck: DISCOVER_RESP tag mismatch — got %08X expected %08X\n",
                   (unsigned)rsp_tag, (unsigned)_discovery_tag);
            return;
        }
        if (node_type != 2 /* repeater */ && node_type != 3 /* room */) {
            printf("Meck: DISCOVER_RESP node_type=%u (not repeater/room), skipping\n",
                   (unsigned)node_type);
            return;
        }

        // pubkey starts at offset 6; remaining bytes determine 8 vs 32.
        uint16_t pk_len = pkt->payload_len - 6;
        if (pk_len < 8) return;
        uint8_t pubkey[32];
        memset(pubkey, 0, sizeof(pubkey));
        memcpy(pubkey, &pkt->payload[6], pk_len < 32 ? pk_len : 32);

        // Look up name from existing contacts (DISCOVER_RESP doesn't
        // carry it). If unknown, render a short pubkey prefix as the
        // name placeholder.
        char name[32] = "";
        int n = getNumContacts();
        for (int i = 0; i < n; i++) {
            ContactInfo ci;
            if (!getContactByIdx(i, ci)) continue;
            if (memcmp(ci.id.pub_key, pubkey, pk_len < 32 ? pk_len : 32) == 0) {
                strncpy(name, ci.name, sizeof(name) - 1);
                name[sizeof(name) - 1] = '\0';
                break;
            }
        }
        if (!name[0]) {
            const char* type_prefix = (node_type == 3) ? "Room" : "Rptr";
            snprintf(name, sizeof(name), "%s %02X%02X%02X%02X",
                     type_prefix,
                     pubkey[0], pubkey[1], pubkey[2], pubkey[3]);
        }

        printf("Meck: DISCOVER_RESP from %02X%02X%02X%02X type=%u "
               "snr=%.1fdB tag=%08X (name='%s')\n",
               pubkey[0], pubkey[1], pubkey[2], pubkey[3],
               (unsigned)node_type, snr_x4 / 4.0f,
               (unsigned)rsp_tag, name);

        xSemaphoreTake(_mutex, portMAX_DELAY);

        // Dedup by pubkey prefix (4 bytes is enough; we accept 8 or 32
        // byte responses but prefixes are unique within radio range).
        int slot = -1;
        for (int i = 0; i < _discovered_count; i++) {
            if (_discovered[i].valid &&
                memcmp(_discovered[i].pub_key, pubkey, 4) == 0) {
                slot = i;
                break;
            }
        }
        if (slot < 0) {
            if (_discovered_count < P4_DISCOVERED_SIZE) {
                slot = _discovered_count++;
            } else {
                // Full — evict cached entry if any, else oldest live.
                slot = 0;
                for (int i = 0; i < P4_DISCOVERED_SIZE; i++) {
                    if (_discovered[i].heard_ms == 0) { slot = i; break; }
                }
                if (_discovered[slot].heard_ms != 0) {
                    unsigned long oldest = _discovered[0].heard_ms;
                    for (int i = 1; i < P4_DISCOVERED_SIZE; i++) {
                        if (_discovered[i].heard_ms < oldest) {
                            slot = i;
                            oldest = _discovered[i].heard_ms;
                        }
                    }
                }
            }
        }

        DiscoveredNode& d = _discovered[slot];
        d.valid = true;
        strncpy(d.name, name, sizeof(d.name) - 1);
        d.name[sizeof(d.name) - 1] = '\0';
        memcpy(d.pub_key, pubkey, 32);
        d.adv_type  = node_type;
        // Sentinel: 0 means "cached, no live read". If a live read
        // happens to be exactly 0 SNR (rare), nudge by 1 so the UI
        // still treats it as live data.
        d.snr_x4    = (snr_x4 == 0) ? 1 : snr_x4;
        d.path_len  = 0;  // zero-hop response by definition
        d.heard_ms  = millis();
        d.already_in_contacts = hasContactWithPubKey(pubkey);
        _discovery_dirty = true;
        xSemaphoreGive(_mutex);
    }

    // ---- Auto-add config (reads from prefs) ----

    bool isAutoAddEnabled() const override {
        return _prefs ? (_prefs->manual_add_contacts == 0) : true;
    }

    bool shouldAutoAddContactType(uint8_t type) const override {
        if (!_prefs) return true;
        if (_prefs->manual_add_contacts != 0) return false;
        // Check per-type bits in autoadd_config
        uint8_t cfg = _prefs->autoadd_config;
        switch (type) {
            case 1: return (cfg & 0x02) != 0;  // ADV_TYPE_CHAT
            case 2: return (cfg & 0x04) != 0;  // ADV_TYPE_REPEATER
            case 3: return (cfg & 0x08) != 0;  // ADV_TYPE_ROOM
            case 4: return (cfg & 0x10) != 0;  // ADV_TYPE_SENSOR
            default: return true;
        }
    }

    bool shouldOverwriteWhenFull() const override {
        return _prefs ? ((_prefs->autoadd_config & 0x01) != 0) : true;
    }

    void onContactsFull() override {}
    void onContactOverwrite(const uint8_t* pub_key) override {}
    uint8_t getAutoAddMaxHops() const override { return 4; }

    ContactInfo* processAck(const uint8_t* data) override { return NULL; }

    uint8_t getPathHashSize() const override {
        // path_hash_mode is a 0-indexed mode per the MeshCore companion
        // protocol: mode 0 = 1 byte, mode 1 = 2 bytes, mode 2 = 3 bytes.
        // Default is mode 0 (= 1 byte), which is the safe interop choice
        // until firmware >= 1.14 reaches critical mass on the network.
        return (_prefs ? _prefs->path_hash_mode : 0) + 1;
    }

private:
    P4RTCClock& _rtc_ref;
    P4DataStore* _store;
    P4NodePrefs* _prefs;

    P4ChannelMessage* _msgs_ch[MAX_GROUP_CHANNELS];
    int _msg_count_ch[MAX_GROUP_CHANNELS];
    int _msg_newest_ch[MAX_GROUP_CHANNELS];
    volatile int _msg_unread_ch[MAX_GROUP_CHANNELS];
    volatile bool _msg_dirty;

    P4RecentHeard _recent[P4_RECENT_HEARD_SIZE];
    int _recent_count;
    int _recent_newest;
    volatile bool _recent_dirty;

    // Active discovery ring + window state. _discovered[] is a flat
    // array (not a circular ring like _recent[]); we just walk it and
    // overwrite the oldest entry when full. _discovery_active flips
    // false either when the window timer expires (loop()) or stopDiscovery()
    // is called explicitly. _discovery_dirty lets the UI poll for "should
    // I redraw?" rather than refresh on every loop tick. _discovery_tag
    // is the per-scan random ID echoed back by responders so we can
    // ignore stale replies from previous scans.
    DiscoveredNode _discovered[P4_DISCOVERED_SIZE];
    int _discovered_count;
    bool _discovery_active;
    unsigned long _discovery_start_ms;
    uint32_t _discovery_tag;
    volatile bool _discovery_dirty;

    bool _advert_enabled;
    unsigned long _next_advert_ms;
    unsigned long _advert_interval_ms;

    bool _contacts_save_pending;
    unsigned long _contacts_save_at;

    // Last millis() at which Meck::loop() called ensureIdentityOnSD().
    // Spaced 5 seconds apart so the periodic SD-mount probe doesn't run
    // on every loop iteration. Once the data store reports backup done
    // (its internal pending flag clears), the call becomes a cheap bool
    // check and the throttle stops mattering.
    unsigned long _last_identity_sd_check = 0;

    SemaphoreHandle_t _mutex;

    // ---- Self-echo dedup ----
    // When we send a channel message, the dispatcher hands the packet to the
    // radio but does NOT seed the seen-table with our originated packet hash.
    // Other nodes relay our message via flood; those flood-relayed copies
    // bounce back to us with different packet hashes (path field changes at
    // each hop), so dispatcher dedup misses them. Without UI-level dedup we
    // see our own message multiple times in the channel history.
    //
    // Track recent outgoing (timestamp, channel) pairs and drop matches in
    // onChannelMessageRecv. Collision risk: another node sends on same
    // channel at same epoch second, their message gets dropped. Acceptable
    // at our mesh size.
    static const int RECENT_OUTGOING_RING = 8;
    uint32_t _recent_outgoing_ts[RECENT_OUTGOING_RING] = {0};
    uint8_t  _recent_outgoing_ch[RECENT_OUTGOING_RING] = {0};
    int      _recent_outgoing_idx = 0;

    bool isOurOwnEcho(uint32_t ts, uint8_t ch_idx) const {
        for (int i = 0; i < RECENT_OUTGOING_RING; i++) {
            if (_recent_outgoing_ts[i] == ts && _recent_outgoing_ch[i] == ch_idx) {
                return true;
            }
        }
        return false;
    }

    // ---- Default channels (used on first boot only) ----

    // Canonical fixed secret for the network-wide Public channel. Used by
    // every MeshCore-compatible node — DO NOT derive it from the channel
    // name. Hashing "#public" or "public" produces a unique-to-this-device
    // secret that no peer can decrypt.
    static const uint8_t* publicChannelSecret() {
        static const uint8_t PUBLIC_SECRET[16] = {
            0x8b, 0x33, 0x87, 0xe9, 0xc5, 0xcd, 0xea, 0x6a,
            0xc9, 0xe5, 0xed, 0xba, 0xa1, 0x15, 0xcd, 0x72,
        };
        return PUBLIC_SECRET;
    }

    void setupDefaultChannels() {
        printf("Meck: no stored channels — creating defaults\n");

        // Slot 0: Public — literal canonical secret, NOT a hash.
        {
            ChannelDetails ch;
            memset(&ch, 0, sizeof(ch));
            strncpy(ch.name, "Public", sizeof(ch.name) - 1);
            memcpy(ch.channel.secret, publicChannelSecret(), 16);
            setChannel(0, ch);
        }

        // Slots 1+: hashtag channels — first 16 bytes of SHA-256(name) is
        // the secret, with the leading '#' included. Matches addHashChannel()
        // and the rest of the AU mesh.
        const char* hashed_defaults[] = { "#test", "#sydney" };
        const int n_hashed = (int)(sizeof(hashed_defaults) / sizeof(hashed_defaults[0]));
        for (int i = 0; i < n_hashed; i++) {
            ChannelDetails ch;
            memset(&ch, 0, sizeof(ch));
            strncpy(ch.name, hashed_defaults[i], sizeof(ch.name) - 1);

            uint8_t hash[32];
            mesh::Utils::sha256(hash, 32,
                (const uint8_t*)hashed_defaults[i], strlen(hashed_defaults[i]));
            memcpy(ch.channel.secret, hash, 16);

            setChannel(i + 1, ch);
        }
    }

    // One-shot self-healing migration. Call from begin() after loadChannels()
    // so existing devices with the old (broken) Public secret get fixed in
    // place without losing custom channels like #ivy. Idempotent — safe to
    // run every boot.
    void migrateChannelSecrets() {
        bool dirty = false;
        for (uint8_t i = 0; i < MAX_GROUP_CHANNELS; i++) {
            ChannelDetails ch;
            if (!getChannel(i, ch) || ch.name[0] == '\0') continue;

            // Match "public" or "#public", case-insensitive — covers both
            // legacy hashed-name layout and the new literal-secret one.
            if (strcasecmp(ch.name, "public")  == 0 ||
                strcasecmp(ch.name, "#public") == 0) {
                if (memcmp(ch.channel.secret, publicChannelSecret(), 16) != 0) {
                    memcpy(ch.channel.secret, publicChannelSecret(), 16);
                    memset(ch.channel.secret + 16, 0, 16);  // 128-bit mode
                    setChannel(i, ch);
                    printf("Meck: migrated Public channel secret at slot %d "
                           "(name='%s')\n", i, ch.name);
                    dirty = true;
                }
            }
        }
        if (dirty) saveChannels();
    }
};
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

// ---- ESP-IDF clock/RNG implementations ----

class P4MillisecondClock : public mesh::MillisecondClock {
public:
    unsigned long getMillis() override { return millis(); }
};

class P4RTCClock : public mesh::RTCClock {
    uint32_t base_time;
    uint64_t accumulator;
    unsigned long prev_millis;
public:
    P4RTCClock() : RTCClock() {
        base_time = 1715770351;  // 15 May 2024 as fallback
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

#define P4_MSG_HISTORY_SIZE  32
#define P4_MSG_TEXT_LEN      300

struct P4ChannelMessage {
    char text[P4_MSG_TEXT_LEN];   // "sender: message"
    uint32_t timestamp;
    uint8_t channel_idx;
    uint8_t path_len;
    bool valid;
};

// ---- Recent heard ring buffer ----

#define P4_RECENT_HEARD_SIZE  16

struct P4RecentHeard {
    char name[32];
    uint8_t pub_key_prefix[4];
    float rssi;
    float snr;
    uint8_t path_len;
    uint32_t timestamp;
    uint8_t adv_type;
    bool valid;
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
        _msg_count = 0;
        _msg_newest = -1;
        memset((void*)_msg_unread_ch, 0, sizeof(_msg_unread_ch));
        _msg_dirty = false;
        memset(_messages, 0, sizeof(_messages));

        _recent_count = 0;
        _recent_newest = -1;
        _recent_dirty = false;
        memset(_recent, 0, sizeof(_recent));

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

        // Load contacts from NVS
        loadContactsFromStore();

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
    }

    // ---- Send a message on a channel (with local echo) ----

    bool sendChannelMessage(uint8_t channel_idx, const char* text) {
        ChannelDetails ch;
        if (!getChannel(channel_idx, ch)) return false;
        uint32_t ts = _rtc_ref.getCurrentTime();
        bool ok = sendGroupMessage(ts, ch.channel, _prefs->node_name, text, strlen(text));
        if (ok) {
            // Local echo — add to message history so it shows on screen
            char echo[P4_MSG_TEXT_LEN];
            snprintf(echo, sizeof(echo), "%s: %s", _prefs->node_name, text);

            xSemaphoreTake(_mutex, portMAX_DELAY);
            _msg_newest = (_msg_newest + 1) % P4_MSG_HISTORY_SIZE;
            P4ChannelMessage& m = _messages[_msg_newest];
            m.timestamp = ts;
            m.channel_idx = channel_idx;
            m.path_len = 0;  // local
            m.valid = true;
            strncpy(m.text, echo, P4_MSG_TEXT_LEN - 1);
            m.text[P4_MSG_TEXT_LEN - 1] = '\0';
            if (_msg_count < P4_MSG_HISTORY_SIZE) _msg_count++;
            _msg_dirty = true;
            xSemaphoreGive(_mutex);
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

    // ---- Accessors ----

    const char* getNodeName() const { return _prefs ? _prefs->node_name : "NONAME"; }
    P4NodePrefs* getNodePrefs() { return _prefs; }
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
        xSemaphoreTake(_mutex, portMAX_DELAY);
        int out = 0;
        for (int i = 0; i < P4_MSG_HISTORY_SIZE && out < max_count; i++) {
            int idx = (_msg_newest - i + P4_MSG_HISTORY_SIZE) % P4_MSG_HISTORY_SIZE;
            if (!_messages[idx].valid) continue;
            if (_messages[idx].channel_idx != channel_idx) continue;
            dest[out++] = _messages[idx];
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

    void setAdvertEnabled(bool en) { _advert_enabled = en; }
    bool isAdvertEnabled() const { return _advert_enabled; }
    const mesh::LocalIdentity& getIdentity() const { return self_id; }
    bool isClockSynced() const { return _rtc_ref.getCurrentTime() > 1750000000; }

protected:
    // ---- BaseChatMesh pure virtual overrides ----

    void onChannelMessageRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt,
                               uint32_t timestamp, const char* text) override {
        uint8_t ch_idx = findChannelIdx(channel);
        uint8_t path_len = pkt->isRouteFlood() ? pkt->path_len : 0xFF;

        printf("Meck: channel[%d] msg: %s\n", ch_idx, text);

        xSemaphoreTake(_mutex, portMAX_DELAY);
        _msg_newest = (_msg_newest + 1) % P4_MSG_HISTORY_SIZE;
        P4ChannelMessage& m = _messages[_msg_newest];
        m.timestamp = timestamp;
        m.channel_idx = ch_idx;
        m.path_len = path_len;
        m.valid = true;
        strncpy(m.text, text, P4_MSG_TEXT_LEN - 1);
        m.text[P4_MSG_TEXT_LEN - 1] = '\0';
        if (_msg_count < P4_MSG_HISTORY_SIZE) _msg_count++;
        _msg_unread_ch[ch_idx]++;
        _msg_dirty = true;
        xSemaphoreGive(_mutex);
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

        // Clock sync: if received timestamp is newer than ours, adopt it
        uint32_t our_time = _rtc_ref.getCurrentTime();
        if (timestamp > our_time && timestamp < 2000000000) {
            _rtc_ref.setCurrentTime(timestamp);
            printf("Meck: clock synced to %lu (was %lu)\n",
                   (unsigned long)timestamp, (unsigned long)our_time);
        }

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
        r.rssi = rssi;
        r.snr = snr;
        r.path_len = path_len;
        r.timestamp = timestamp;
        r.adv_type = adv_type;
        r.valid = true;
        if (_recent_count < P4_RECENT_HEARD_SIZE) _recent_count++;
        _recent_dirty = true;
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
        return _prefs ? _prefs->path_hash_mode : 1;
    }

private:
    P4RTCClock& _rtc_ref;
    P4DataStore* _store;
    P4NodePrefs* _prefs;

    P4ChannelMessage _messages[P4_MSG_HISTORY_SIZE];
    int _msg_count;
    int _msg_newest;
    volatile int _msg_unread_ch[MAX_GROUP_CHANNELS];
    volatile bool _msg_dirty;

    P4RecentHeard _recent[P4_RECENT_HEARD_SIZE];
    int _recent_count;
    int _recent_newest;
    volatile bool _recent_dirty;

    bool _advert_enabled;
    unsigned long _next_advert_ms;
    unsigned long _advert_interval_ms;

    bool _contacts_save_pending;
    unsigned long _contacts_save_at;

    SemaphoreHandle_t _mutex;

    // ---- Default channels (used on first boot only) ----

    void setupDefaultChannels() {
        printf("Meck: no stored channels — creating defaults\n");

        const char* defaults[] = { "#public", "#test", "#sydney" };
        for (int i = 0; i < 3; i++) {
            ChannelDetails ch;
            memset(&ch, 0, sizeof(ch));
            strncpy(ch.name, defaults[i], sizeof(ch.name) - 1);

            uint8_t hash[32];
            mesh::Utils::sha256(hash, 32, (const uint8_t*)defaults[i], strlen(defaults[i]));
            memcpy(ch.channel.secret, hash, 16);

            setChannel(i, ch);
        }
    }
};
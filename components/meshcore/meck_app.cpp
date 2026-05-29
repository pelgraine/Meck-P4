/*
 * meck_app.cpp — Meck mesh stack lifecycle for T-Display P4
 *
 * Holds the_mesh (Meck instance), the data store, the prefs struct, and
 * the FreeRTOS task that drives the protocol loop. Equivalent to the
 * `static P4Mesh* the_mesh` + `mesh_task` block from the old repo's
 * meshcore_test.cpp, encapsulated here in the meshcore component so
 * main.cpp doesn't need to know any of these types.
 *
 * Public API (in meck.h):
 *   meck_app_init()  — NVS init, identity, channels, contacts
 *   meck_app_start() — spawn meck_task
 *
 * Internal accessor (target.h):
 *   meck_get_instance() — UI code uses this later to read messages,
 *                         contacts, recent heard, etc.
 */

#include "meck.h"
#include "target.h"
#include "MeckMesh.h"
#include "MeckDataStore.h"
#include "MeckImport.h"
#include "MeckExport.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>

// Debug Logs: rewrites printf -> meck_debug_log_printf so calls below
// land in the SD log file when Settings > Debug Logs > Start is active.
// See meck_log.h for the macro mechanism.
#include "meck_log.h"
#include "BundledSounds.h"
// #include "BundledPictures.h"  // disabled for v0.3.8
#include "NotifSounds.h"
#include "SerialC6BLEInterface.h"
#include "MeckCompanion.h"
#include "esp_random.h"

// ---- Static instances ----
static P4DataStore g_dataStore;
static P4NodePrefs g_node_prefs;
static P4RNG g_rng;
static P4RTCClock g_rtc;
// P4MeshTables replaces upstream SimpleMeshTables so that flood-relayed
// copies of our own outgoing packets aren't filtered before reaching
// onChannelMessageRecv. Without this swap the upstream Mesh.cpp pre-
// marks every outgoing packet in the seen-table at sendFlood time, which
// silently drops every repeater echo of our own send and prevents
// heard_count from ever advancing past zero. See P4MeshTables in
// MeckMesh.h for the full rationale.
static P4MeshTables g_mesh_tables;
static Meck* g_the_mesh = nullptr;
NotifSounds g_notif_sounds;

// ---- C6 AT pointer (shared by BLE and WiFi) ----
static Cpp_Bus_Driver::Esp_At* g_c6_at = nullptr;

#if MECK_BLE_ENABLED
// ---- BLE companion transport via ESP32-C6 AT over SDIO ----
static SerialC6BLEInterface g_ble_interface;
static MeckCompanion g_companion;
#endif

// ---- WiFi companion transport via ESP32-C6 AT over SDIO ----
#include "SerialC6WiFiInterface.h"
static SerialC6WiFiInterface g_wifi_interface;
static MeckCompanion g_wifi_companion;

#if MECK_BLE_ENABLED
// Deferred BLE enable/disable
static volatile int g_ble_pending_action = 0;
#endif

extern "C" void meck_ble_bind(void* esp_at_ptr) {
    g_c6_at = (Cpp_Bus_Driver::Esp_At*)esp_at_ptr;
    printf("meck_ble_bind: C6 AT pointer bound\n");
}

// ---- Battery gauge ----
static meck_battery_fn g_battery_get_mv  = nullptr;
static meck_battery_fn g_battery_get_pct = nullptr;

extern "C" void meck_battery_bind(meck_battery_fn get_mv, meck_battery_fn get_pct) {
    g_battery_get_mv  = get_mv;
    g_battery_get_pct = get_pct;
    printf("meck_battery_bind: gauge bound\n");
}

extern "C" uint16_t meck_battery_get_mv() {
    return g_battery_get_mv ? g_battery_get_mv() : 0;
}

extern "C" uint16_t meck_battery_get_pct() {
    return g_battery_get_pct ? g_battery_get_pct() : 0;
}

// ---- Companion push wrappers (forward to active companions) ----

extern "C" void meck_companion_push_channel_msg(uint8_t ch_idx, uint8_t path_len,
                                                  uint32_t timestamp, int8_t snr_x4,
                                                  const char* text) {
#if MECK_BLE_ENABLED
    g_companion.pushChannelMessage(ch_idx, path_len, timestamp, snr_x4, text);
#endif
    g_wifi_companion.pushChannelMessage(ch_idx, path_len, timestamp, snr_x4, text);
}

extern "C" void meck_companion_push_contact_msg(const uint8_t* pub_key_prefix,
                                                  uint8_t path_len, uint8_t txt_type,
                                                  uint32_t timestamp, int8_t snr_x4,
                                                  const uint8_t* extra, int extra_len,
                                                  const char* text) {
#if MECK_BLE_ENABLED
    g_companion.pushContactMessage(pub_key_prefix, path_len, txt_type, timestamp, snr_x4, extra, extra_len, text);
#endif
    g_wifi_companion.pushContactMessage(pub_key_prefix, path_len, txt_type, timestamp, snr_x4, extra, extra_len, text);
}

extern "C" void meck_companion_push_send_confirmed(const uint8_t* ack_hash, uint32_t trip_time) {
#if MECK_BLE_ENABLED
    g_companion.pushSendConfirmed(ack_hash, trip_time);
#endif
    g_wifi_companion.pushSendConfirmed(ack_hash, trip_time);
}

extern "C" void meck_companion_push_rx_log(int8_t snr_x4, int8_t rssi,
                                            const uint8_t* raw, int len) {
#if MECK_BLE_ENABLED
    g_companion.pushRxLog(snr_x4, rssi, raw, len);
#endif
    g_wifi_companion.pushRxLog(snr_x4, rssi, raw, len);
}

extern "C" void meck_companion_push_new_advert(const void* contact_info) {
    const ContactInfo* ci = (const ContactInfo*)contact_info;
#if MECK_BLE_ENABLED
    g_companion.pushNewAdvert(*ci);
#endif
    g_wifi_companion.pushNewAdvert(*ci);
}

extern "C" void meck_companion_push_advert(const uint8_t* pub_key) {
#if MECK_BLE_ENABLED
    g_companion.pushAdvert(pub_key);
#endif
    g_wifi_companion.pushAdvert(pub_key);
}

extern "C" void meck_companion_push_path_updated(const uint8_t* pub_key) {
#if MECK_BLE_ENABLED
    g_companion.pushPathUpdated(pub_key);
#endif
    g_wifi_companion.pushPathUpdated(pub_key);
}

extern "C" void meck_companion_push_login_success(const uint8_t* pub_key,
                                                   uint32_t server_clock,
                                                   uint8_t is_admin,
                                                   uint8_t acl_permissions,
                                                   uint8_t fw_ver_level) {
#if MECK_BLE_ENABLED
    g_companion.pushLoginSuccess(pub_key, server_clock, is_admin,
                                 acl_permissions, fw_ver_level);
#endif
    g_wifi_companion.pushLoginSuccess(pub_key, server_clock, is_admin,
                                      acl_permissions, fw_ver_level);
}

extern "C" void meck_companion_push_login_fail(const uint8_t* pub_key) {
#if MECK_BLE_ENABLED
    g_companion.pushLoginFail(pub_key);
#endif
    g_wifi_companion.pushLoginFail(pub_key);
}

extern "C" void meck_companion_push_status_response(const uint8_t* pub_key,
                                                     const uint8_t* payload,
                                                     uint8_t payload_len) {
#if MECK_BLE_ENABLED
    g_companion.pushStatusResponse(pub_key, payload, payload_len);
#endif
    g_wifi_companion.pushStatusResponse(pub_key, payload, payload_len);
}

extern "C" void meck_companion_push_cli_reply(const uint8_t* pub_key,
                                               uint8_t path_len,
                                               uint32_t timestamp,
                                               int8_t snr_x4,
                                               const char* text) {
#if MECK_BLE_ENABLED
    g_companion.pushCliReply(pub_key, path_len, timestamp, snr_x4, text);
#endif
    g_wifi_companion.pushCliReply(pub_key, path_len, timestamp, snr_x4, text);
}

extern "C" void meck_companion_push_binary_response(uint32_t tag,
                                                     const uint8_t* payload,
                                                     uint8_t payload_len) {
#if MECK_BLE_ENABLED
    g_companion.pushBinaryResponse(tag, payload, payload_len);
#endif
    g_wifi_companion.pushBinaryResponse(tag, payload, payload_len);
}

#if MECK_BLE_ENABLED
extern "C" void meck_ble_set_enabled(bool enabled) {
    g_ble_pending_action = enabled ? 1 : -1;
    printf("meck_ble_set_enabled: queued %s\n", enabled ? "ON" : "OFF");
}

extern "C" bool meck_ble_is_enabled() {
    return g_ble_interface.isEnabled();
}

// Process deferred BLE enable/disable — called from meck_task only.
static void meck_apply_pending_ble() {
    int action = g_ble_pending_action;
    if (action == 0) return;
    g_ble_pending_action = 0;

    if (action > 0 && !g_ble_interface.isEnabled() && g_c6_at) {
        // Disable WiFi first (mutual exclusivity — shared SDIO bus)
        if (g_wifi_interface.isEnabled()) {
            g_wifi_interface.disable();
            printf("meck_apply_pending_ble: disabled WiFi (mutual excl)\n");
        }
        Meck* mesh = meck_get_instance();
        const char* name = mesh ? mesh->getNodeName() : "Meck-P4";
        g_ble_interface.begin(g_c6_at, name);
        g_ble_interface.setPin(g_node_prefs.ble_pin);
        g_ble_interface.enable();
        printf("meck_apply_pending_ble: enabled\n");
    } else if (action < 0 && g_ble_interface.isEnabled()) {
        g_ble_interface.disable();
        printf("meck_apply_pending_ble: disabled\n");
    }
}
#endif

// ---- Deferred WiFi enable/disable (same SDIO contention pattern) ----
static volatile int g_wifi_pending_action = 0;

extern "C" void meck_wifi_set_enabled(bool enabled) {
    g_wifi_pending_action = enabled ? 1 : -1;
    printf("meck_wifi_set_enabled: queued %s\n", enabled ? "ON" : "OFF");
}

extern "C" bool meck_wifi_is_enabled() {
    return g_wifi_interface.isEnabled();
}

extern "C" bool meck_wifi_is_connected() {
    return g_wifi_interface.isWiFiConnected();
}

extern "C" const char* meck_wifi_get_ip() {
    return g_wifi_interface.getIP();
}

static void meck_apply_pending_wifi() {
    int action = g_wifi_pending_action;
    if (action == 0) return;
    g_wifi_pending_action = 0;

    if (action > 0 && !g_wifi_interface.isEnabled() && g_c6_at) {
#if MECK_BLE_ENABLED
        // Disable BLE first (mutual exclusivity — shared SDIO bus)
        if (g_ble_interface.isEnabled()) {
            g_ble_interface.disable();
            printf("meck_apply_pending_wifi: disabled BLE (mutual excl)\n");
        }
#endif
        g_wifi_interface.begin(g_c6_at);
        g_wifi_interface.setCredentials(g_node_prefs.wifi_ssid, g_node_prefs.wifi_password);
        g_wifi_interface.enable();
        if (g_wifi_interface.isEnabled()) {
            printf("meck_apply_pending_wifi: enabled (IP: %s)\n", g_wifi_interface.getIP());
        } else {
            printf("meck_apply_pending_wifi: enable FAILED\n");
        }
    } else if (action < 0 && g_wifi_interface.isEnabled()) {
        g_wifi_interface.disable();
        printf("meck_apply_pending_wifi: disabled\n");
    }
}

// ---- Internal accessor (declared in target.h) ----
Meck* meck_get_instance() { return g_the_mesh; }

// ---- P4MeshTables outgoing-mode markers ----
// Called by Meck::sendChannelMessage around its sendGroupMessage call so
// the hash of our just-built packet is captured (not marked seen) and
// relayed copies can pass through hasSeen on every arrival. Implemented
// here (rather than as static methods on P4MeshTables) so MeckMesh.h
// doesn't need to know which P4MeshTables instance is the active one.
extern "C" void meck_tables_begin_outgoing() { g_mesh_tables.beginMarkingOurOutgoing(); }
extern "C" void meck_tables_end_outgoing()   { g_mesh_tables.endMarkingOurOutgoing(); }

// Push a UTC epoch into the MeshCore P4RTCClock. Called from
// meck_globals.cpp's meck_clock_set_utc bridge so that GPS-sourced time
// reaches the RTC the Meck class was constructed with (and therefore
// every MeshCore-internal timestamp call plus the UI's top-bar clock,
// which read via mesh->getRTCClock()). Without this, only the
// SoftRtcClock in meck_globals.cpp was updated and g_rtc stayed at its
// boot-time compileTimeEpoch() value.
extern "C" void meck_app_rtc_set(uint32_t epoch) {
    g_rtc.setCurrentTime(epoch);
}

// ---- App init ----
extern "C" bool meck_app_init() {
    printf("meck_app_init: starting\n");

    // 0. BQ27220 fuel gauge calibration. LilyGo's main.cpp already wrote
    //    design_capacity=1000 via the wrapper's minimal path; this runs
    //    the full TI procedure to force FCC recalculation. Self-gated, so
    //    after the first successful run it returns in milliseconds.
    meck_battery_calibrate();

    // 1. NVS / DataStore
    if (!g_dataStore.begin()) {
        printf("meck_app_init: dataStore.begin() failed\n");
        return false;
    }

    // 2. Try to restore everything from SD if NVS was wiped (e.g. fresh flash)
    g_dataStore.restoreFromSD();

    // 3. Load prefs (or apply variant.h defaults)
    if (!g_dataStore.loadPrefs(g_node_prefs)) {
        printf("meck_app_init: no prefs in NVS, using variant.h defaults\n");
        g_node_prefs.setDefaults();
        g_dataStore.savePrefs(g_node_prefs);
    }

    // 3a. One-shot import from /sdcard/meshcore/import.json if present.
    // Runs after loadPrefs so the import can overlay its name and
    // radio_settings onto the loaded prefs rather than zeroes. If
    // anything was applied, persist the modified prefs back to NVS+SD
    // so the loaded copy reflects the import.
    if (meck_import_from_sd(g_dataStore, g_node_prefs)) {
        g_dataStore.savePrefs(g_node_prefs);
        printf("meck_app_init: import applied, prefs persisted\n");
    }

    // ---- Apply prefs to the live radio ----
    // meck_radio_attach() runs from app_main() before prefs are
    // available, so it boots the radio on variant.h defaults
    // (which are the AU Narrow preset). This is where the user's
    // actual saved preset finally takes effect.
    radio_set_params(g_node_prefs.freq, g_node_prefs.bw,
                     g_node_prefs.sf,   g_node_prefs.cr);
    radio_set_tx_power(g_node_prefs.tx_power_dbm);
    printf("meck_app_init: applied prefs to radio: "
           "%.3f MHz, BW=%.1f kHz, SF%u, CR=%u, TX=%d dBm\n",
           (double)g_node_prefs.freq, (double)g_node_prefs.bw,
           (unsigned)g_node_prefs.sf, (unsigned)g_node_prefs.cr,
           (int)g_node_prefs.tx_power_dbm); 

    // 4. Construct the Meck mesh
    g_the_mesh = new Meck(radio_driver, g_rng, g_rtc, g_mesh_tables);
    if (!g_the_mesh) {
        printf("meck_app_init: failed to allocate Meck\n");
        return false;
    }

    // 5. Initialize Meck (loads or generates identity, loads channels, contacts)
    g_the_mesh->begin(g_dataStore, g_node_prefs);

    // 6. Copy bundled notification tones to SD and load tone config
    copyBundledSoundsToSD();
    // copyBundledPicturesToSD();  // disabled for v0.3.8
    g_notif_sounds.begin();

    // 7. Initialize companion transports via C6
    if (g_c6_at) {
#if MECK_BLE_ENABLED
        g_ble_interface.begin(g_c6_at, g_node_prefs.node_name);

        // Probe C6 for OTA and WiFi capability (diagnostic, runs once at boot)
        g_ble_interface.probeOTA();

        // Generate a 6-digit BLE pairing PIN on first use
        if (g_node_prefs.ble_pin == 0) {
            g_node_prefs.ble_pin = 100000 + (esp_random() % 900000);
            if (g_the_mesh) g_the_mesh->getDataStore()->savePrefs(g_node_prefs);
            printf("meck_app_init: generated BLE PIN (masked in log)\n");
        }
        g_ble_interface.setPin(g_node_prefs.ble_pin);

        if (g_node_prefs.ble_enabled != 0) {
            g_ble_interface.enable();
            printf("meck_app_init: BLE companion enabled\n");
        } else {
            printf("meck_app_init: BLE companion interface OFF (pref)\n");
        }
        g_companion.begin(g_the_mesh, &g_ble_interface);
        printf("meck_app_init: BLE companion protocol handler ready\n");
#endif

        // 8. Initialize WiFi companion transport via C6
        g_wifi_interface.begin(g_c6_at);
        g_wifi_interface.setCredentials(g_node_prefs.wifi_ssid, g_node_prefs.wifi_password);
        g_wifi_companion.begin(g_the_mesh, &g_wifi_interface);

        if (g_node_prefs.wifi_enabled != 0 && g_node_prefs.wifi_ssid[0] != '\0') {
            g_wifi_interface.enable();
            printf("meck_app_init: WiFi companion enabled (IP: %s, port 5000)\n",
                   g_wifi_interface.getIP());
        } else {
            printf("meck_app_init: WiFi companion OFF (pref)\n");
        }
    } else {
        printf("meck_app_init: WARNING — meck_ble_bind not called, BLE disabled\n");
    }

    printf("meck_app_init: Meck stack ready\n");
    return true;
}

// ---- Forward declarations for functions defined later ----
static void meck_apply_pending_voice_send(void);
static void meck_apply_pending_picture_send(void);

// ---- Mesh task ----
static void meck_task(void* arg) {
    printf("meck_task: started\n");
    while (true) {
        radio_apply_pending_reconfig();
        meck_apply_pending_send();
        meck_apply_pending_send_dm();
        meck_apply_pending_admin_login();
        meck_apply_pending_admin_status();
        meck_apply_pending_admin_cli();
        meck_apply_pending_admin_telemetry();
        meck_apply_pending_admin_neighbours();
        // Voice and picture transfer disabled for v0.3.8
        // meck_apply_pending_voice_send();
        // meck_apply_pending_picture_send();
        meck_apply_pending_save();
#if MECK_BLE_ENABLED
        meck_apply_pending_ble();
#endif
        meck_apply_pending_wifi();
#if MECK_BLE_ENABLED
        // BLE companion protocol: drain SDIO, handle app commands
        g_companion.check();
#endif
        // WiFi companion protocol: same, via TCP
        g_wifi_companion.check();
        if (g_the_mesh) {
            g_the_mesh->loop();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void meck_app_start() {
    if (!g_the_mesh) {
        printf("meck_app_start: Meck not initialized, skipping task spawn\n");
        return;
    }
    xTaskCreate(meck_task, "meck_task", 16 * 1024, NULL, 3, NULL);
}

// ============================================================================
// Deferred send-message queue (avoids SPI race between LVGL task and meck_task)
// ============================================================================
static volatile bool    g_send_pending     = false;
static volatile uint8_t g_send_pending_ch  = 0;
static char             g_send_pending_text[200] = {};

extern "C" void meck_request_send_text(uint8_t channel_idx, const char* text) {
    if (!text) return;
    g_send_pending_ch = channel_idx;
    strncpy(g_send_pending_text, text, sizeof(g_send_pending_text) - 1);
    g_send_pending_text[sizeof(g_send_pending_text) - 1] = '\0';
    g_send_pending = true;
    printf("meck_request_send_text: queued ch[%d] msg='%s'\n",
           (int)channel_idx, g_send_pending_text);
}

extern "C" void meck_apply_pending_send() {
    if (!g_send_pending) return;
    g_send_pending = false;
    if (!g_the_mesh) return;
    if (g_the_mesh->sendChannelMessage(g_send_pending_ch, g_send_pending_text)) {
        printf(">>> Sent on ch[%d]: %s\n", (int)g_send_pending_ch, g_send_pending_text);
    } else {
        printf(">>> FAILED to send on ch[%d]: %s\n", (int)g_send_pending_ch, g_send_pending_text);
    }
}

// ============================================================================
// Direct-message send bridge
// ----------------------------------------------------------------------------
// LVGL task enqueues a DM via meck_request_send_dm(); meck_task drains via
// meck_apply_pending_send_dm() at the top of its loop. Same SPI-race
// avoidance pattern as meck_request_send_text — the LVGL handler must
// never call Meck::sendDirectMessage directly because that touches the
// radio bus.
//
// One-deep queue (matching the channel-send pattern). DMs are rare enough
// that a deeper queue would just add complexity for no gain; if the user
// taps send twice in quick succession the second tap overwrites the first
// pending entry. That's an edge case where the user is best served by
// seeing only the more recent send.
// ============================================================================
static volatile bool g_dm_send_pending      = false;
static volatile int  g_dm_send_pending_idx  = -1;
static char          g_dm_send_pending_text[200] = {};

// Pending DM send completion — populated by meck_apply_pending_send_dm
// after a successful send, drained by meck_drain_pending_dm_sends()
// from the LVGL task so the UI can write expected_ack back to the
// matching ring slot. One-deep mirrors the send queue; if two sends
// happen between drain cycles the second one overwrites the first's
// pending notification (the actual sends both happened on the radio,
// only the UI status writeback is at risk — and the lookupDMAckStatus
// fallback handles "no expected_ack on bubble" gracefully).
static volatile bool     g_dm_sent_pending      = false;
static volatile int      g_dm_sent_contact_idx  = -1;
static volatile uint32_t g_dm_sent_expected_ack = 0;
static volatile uint32_t g_dm_sent_est_timeout  = 0;

extern "C" void meck_request_send_dm(int contact_idx, const char* text) {
    if (!text) return;
    g_dm_send_pending_idx = contact_idx;
    strncpy(g_dm_send_pending_text, text, sizeof(g_dm_send_pending_text) - 1);
    g_dm_send_pending_text[sizeof(g_dm_send_pending_text) - 1] = '\0';
    g_dm_send_pending = true;
    printf("meck_request_send_dm: queued contact[%d] msg='%s'\n",
           contact_idx, g_dm_send_pending_text);
}

extern "C" void meck_apply_pending_send_dm() {
    if (!g_dm_send_pending) return;
    g_dm_send_pending = false;
    if (!g_the_mesh) return;
    int contact_idx = g_dm_send_pending_idx;
    uint32_t expected_ack = 0;
    // sendDirectMessage's overload takes &est_timeout indirectly via the
    // ack table; we read the captured timeout back from the table after
    // the send returns. Simpler than threading another out-parameter
    // through the public signature.
    if (g_the_mesh->sendDirectMessage(contact_idx, g_dm_send_pending_text,
                                      &expected_ack)) {
        // Capture est_timeout by looking up the entry we just populated.
        // The ack table is small and the entry is the newest write, so
        // an exact lookup gives us the est_timeout_ms.
        bool dummy_acked;
        unsigned long dummy_sent;
        uint32_t est_timeout = 0;
        g_the_mesh->lookupDMAckStatus(expected_ack, dummy_acked, dummy_sent, est_timeout);

        g_dm_sent_contact_idx  = contact_idx;
        g_dm_sent_expected_ack = expected_ack;
        g_dm_sent_est_timeout  = est_timeout;
        g_dm_sent_pending      = true;

        printf(">>> Sent DM to contact[%d]: %s (ack=0x%08X)\n",
               contact_idx, g_dm_send_pending_text, (unsigned)expected_ack);
    } else {
        printf(">>> FAILED to send DM to contact[%d]: %s\n",
               contact_idx, g_dm_send_pending_text);
    }
}

// Callback type: invoked on the LVGL task when a DM send has completed
// on the mesh task. The UI uses this to write expected_ack back into
// the matching outgoing DMMessage in the per-contact ring so future
// renders can poll lookupDMAckStatus and show "Delivered" once the
// ACK lands. contact_idx + expected_ack identify which bubble; the
// UI scans its own ring (newest outgoing message for that contact with
// expected_ack == 0) and fills it in.
typedef void (*meck_dm_sent_cb_t)(int contact_idx,
                                  uint32_t expected_ack,
                                  uint32_t est_timeout_ms);

static meck_dm_sent_cb_t g_dm_sent_cb = nullptr;

extern "C" void meck_register_dm_sent_callback(meck_dm_sent_cb_t cb) {
    g_dm_sent_cb = cb;
    printf("meck_register_dm_sent_callback: %s\n", cb ? "registered" : "cleared");
}

extern "C" void meck_drain_pending_dm_sends() {
    if (!g_dm_sent_pending) return;
    g_dm_sent_pending = false;
    int      idx     = g_dm_sent_contact_idx;
    uint32_t ack     = g_dm_sent_expected_ack;
    uint32_t timeout = g_dm_sent_est_timeout;
    if (g_dm_sent_cb) {
        g_dm_sent_cb(idx, ack, timeout);
    } else {
        printf("meck_drain_pending_dm_sends: no callback for contact[%d]\n", idx);
    }
}

// ============================================================================
// Direct-message receive bridge
// ----------------------------------------------------------------------------
// Pull-based: meck_task fills Meck's pending-DM ring inside onMessageRecv
// (mesh task). The LVGL task calls meck_drain_pending_dms() periodically
// from ui_update_timer_cb, which pops queued DMs and dispatches them to
// the registered callback. The callback therefore runs on the LVGL task
// and can safely touch LVGL state.
//
// One callback slot — the UI registers a single dispatcher and routes
// internally to the right conversation view / inbox unread counter.
// Multiple registrations would just complicate ordering for no benefit.
// ============================================================================
static meck_dm_recv_cb_t g_dm_recv_cb = nullptr;

extern "C" void meck_register_dm_recv_callback(meck_dm_recv_cb_t cb) {
    g_dm_recv_cb = cb;
    printf("meck_register_dm_recv_callback: %s\n", cb ? "registered" : "cleared");
}

extern "C" void meck_drain_pending_dms() {
    if (!g_the_mesh) return;
    // Drain whatever is queued. Each iteration is one ring pop + one
    // callback invocation; cheap enough to run every UI tick (500ms).
    // The UI callback decides whether to redraw, update unread counts,
    // persist, etc.
    Meck::PendingDMRecv dm;
    while (g_the_mesh->drainPendingDM(dm)) {
        if (g_dm_recv_cb) {
            g_dm_recv_cb(dm.from_pub_key, dm.from_name, dm.text,
                         dm.sender_timestamp, dm.path_len, dm.snr_x4);
        } else {
            // No callback registered yet (UI not initialised, or someone
            // unregistered). Log so the message isn't silently lost.
            printf("meck_drain_pending_dms: no callback, dropping DM from %s: %s\n",
                   dm.from_name, dm.text);
        }
    }
}

// ============================================================================
// Room post receive bridge (Piece B)
// ----------------------------------------------------------------------------
// Same pattern as the DM receive bridge. meck_task fills Meck's pending
// post ring inside onSignedMessageRecv; the LVGL task drains here via
// meck_drain_pending_posts() and invokes the registered callback.
// ============================================================================
static meck_post_recv_cb_t g_post_recv_cb = nullptr;

extern "C" void meck_register_post_recv_callback(meck_post_recv_cb_t cb) {
    g_post_recv_cb = cb;
    printf("meck_register_post_recv_callback: %s\n", cb ? "registered" : "cleared");
}

extern "C" void meck_drain_pending_posts() {
    if (!g_the_mesh) return;
    Meck::PendingPostRecv post;
    while (g_the_mesh->drainPendingPost(post)) {
        if (g_post_recv_cb) {
            g_post_recv_cb(post.room_pub_key, post.room_name, post.sender_prefix,
                           post.text, post.sender_timestamp,
                           post.path_len, post.snr_x4);
        } else {
            printf("meck_drain_pending_posts: no callback, dropping post from %s: %s\n",
                   post.room_name, post.text);
        }
    }
}

// ============================================================================
// Repeater Admin bridge
// ----------------------------------------------------------------------------
// Send-side: four one-deep queues, one per request type. LVGL queues via
// meck_request_admin_*; meck_task drains via meck_apply_pending_admin_*
// at the top of its loop. Each apply calls the corresponding Meck::ui*
// method which actually touches the radio.
//
// Send-result notification: a single shared g_admin_send_result_*
// slot is populated after each send attempt (success or failure). The
// LVGL task picks it up on its next tick via the drain dispatcher.
//
// Receive-side: meck_task's onContactResponse / onCommandDataRecv push
// onto Meck's per-type pending rings. meck_drain_pending_admin_responses
// pops each ring and fires the matching registered callback. All
// callbacks run on the LVGL task.
// ============================================================================

// ---- Send queues (one-deep each) ----
static volatile bool g_admin_login_pending     = false;
static volatile int  g_admin_login_contact_idx = -1;
static char          g_admin_login_password[64] = {};

static volatile bool g_admin_status_pending     = false;
static volatile int  g_admin_status_contact_idx = -1;

static volatile bool g_admin_cli_pending     = false;
static volatile int  g_admin_cli_contact_idx = -1;
static char          g_admin_cli_command[160] = {};

static volatile bool g_admin_telemetry_pending     = false;
static volatile int  g_admin_telemetry_contact_idx = -1;

static volatile bool     g_admin_neighbours_pending             = false;
static volatile int      g_admin_neighbours_contact_idx         = -1;
static volatile uint8_t  g_admin_neighbours_count               = 0;
static volatile uint16_t g_admin_neighbours_offset              = 0;
static volatile uint8_t  g_admin_neighbours_order_by            = 0;
static volatile uint8_t  g_admin_neighbours_pubkey_prefix_length = 0;

// ---- Send-result notification slot (one-deep, shared across all types) ----
// Populated by meck_apply_pending_admin_* after each send attempt; drained
// by meck_drain_pending_admin_responses on the LVGL task. If two send
// attempts complete between two LVGL ticks the second overwrites the
// first's notification — acceptable because the UI flow is sequential
// (user can't tap a second admin button before the first lands).
static volatile bool                 g_admin_send_result_pending  = false;
static volatile meck_admin_req_type_t g_admin_send_result_type    = MECK_ADMIN_REQ_LOGIN;
static volatile bool                 g_admin_send_result_success  = false;
static volatile uint32_t             g_admin_send_result_timeout  = 0;

// ---- Callback slots ----
static meck_admin_send_result_cb_t g_admin_send_result_cb = nullptr;
static meck_admin_login_cb_t       g_admin_login_cb       = nullptr;
static meck_admin_status_cb_t      g_admin_status_cb      = nullptr;
static meck_admin_cli_cb_t         g_admin_cli_cb         = nullptr;
static meck_admin_telemetry_cb_t   g_admin_telemetry_cb   = nullptr;
static meck_admin_neighbours_cb_t  g_admin_neighbours_cb  = nullptr;

// ---- Send-queue producers (called by LVGL task) ----

extern "C" void meck_request_admin_login(int contact_idx, const char* password) {
    if (!password) return;
    g_admin_login_contact_idx = contact_idx;
    strncpy(g_admin_login_password, password, sizeof(g_admin_login_password) - 1);
    g_admin_login_password[sizeof(g_admin_login_password) - 1] = '\0';
    g_admin_login_pending = true;
    printf("meck_request_admin_login: queued contact[%d]\n", contact_idx);
}

extern "C" void meck_request_admin_status(int contact_idx) {
    g_admin_status_contact_idx = contact_idx;
    g_admin_status_pending = true;
    printf("meck_request_admin_status: queued contact[%d]\n", contact_idx);
}

extern "C" void meck_request_admin_cli(int contact_idx, const char* command) {
    if (!command) return;
    g_admin_cli_contact_idx = contact_idx;
    strncpy(g_admin_cli_command, command, sizeof(g_admin_cli_command) - 1);
    g_admin_cli_command[sizeof(g_admin_cli_command) - 1] = '\0';
    g_admin_cli_pending = true;
    printf("meck_request_admin_cli: queued contact[%d] cmd='%s'\n",
           contact_idx, g_admin_cli_command);
}

extern "C" void meck_request_admin_telemetry(int contact_idx) {
    g_admin_telemetry_contact_idx = contact_idx;
    g_admin_telemetry_pending = true;
    printf("meck_request_admin_telemetry: queued contact[%d]\n", contact_idx);
}

extern "C" void meck_request_admin_neighbours(int contact_idx,
                                              uint8_t count,
                                              uint16_t offset,
                                              uint8_t order_by,
                                              uint8_t pubkey_prefix_length) {
    g_admin_neighbours_contact_idx          = contact_idx;
    g_admin_neighbours_count                = count;
    g_admin_neighbours_offset               = offset;
    g_admin_neighbours_order_by             = order_by;
    g_admin_neighbours_pubkey_prefix_length = pubkey_prefix_length;
    g_admin_neighbours_pending              = true;
    printf("meck_request_admin_neighbours: queued contact[%d] count=%u "
           "offset=%u order=%u prefix_len=%u\n",
           contact_idx, (unsigned)count, (unsigned)offset,
           (unsigned)order_by, (unsigned)pubkey_prefix_length);
}

// Helper: record a send-result for the LVGL task to pick up on next drain.
// Last-write-wins if two completions happen in the same meck_task loop
// (rare, single-session policy).
static void admin_post_send_result(meck_admin_req_type_t type,
                                    bool success,
                                    uint32_t est_timeout) {
    g_admin_send_result_type    = type;
    g_admin_send_result_success = success;
    g_admin_send_result_timeout = est_timeout;
    g_admin_send_result_pending = true;
}

// ---- Send-queue drains (called by meck_task) ----

extern "C" void meck_apply_pending_admin_login() {
    if (!g_admin_login_pending) return;
    g_admin_login_pending = false;
    if (!g_the_mesh) {
        admin_post_send_result(MECK_ADMIN_REQ_LOGIN, false, 0);
        return;
    }

    int contact_idx = g_admin_login_contact_idx;
    uint32_t est_timeout = 0;
    bool ok = g_the_mesh->uiLoginToRepeater(contact_idx, g_admin_login_password,
                                             est_timeout);

    // Wipe the password buffer post-send. Belt and braces — the buffer
    // is in DRAM and would otherwise hang around indefinitely as a
    // plaintext copy of a credential.
    memset(g_admin_login_password, 0, sizeof(g_admin_login_password));

    admin_post_send_result(MECK_ADMIN_REQ_LOGIN, ok, est_timeout);
    printf(">>> %s admin login to contact[%d], est_timeout=%ums\n",
           ok ? "Sent" : "FAILED",
           contact_idx, (unsigned)est_timeout);
}

extern "C" void meck_apply_pending_admin_status() {
    if (!g_admin_status_pending) return;
    g_admin_status_pending = false;
    if (!g_the_mesh) {
        admin_post_send_result(MECK_ADMIN_REQ_STATUS, false, 0);
        return;
    }

    int contact_idx = g_admin_status_contact_idx;
    uint32_t est_timeout = 0;
    bool ok = g_the_mesh->uiSendStatusRequest(contact_idx, est_timeout);

    admin_post_send_result(MECK_ADMIN_REQ_STATUS, ok, est_timeout);
    printf(">>> %s admin status request to contact[%d], est_timeout=%ums\n",
           ok ? "Sent" : "FAILED",
           contact_idx, (unsigned)est_timeout);
}

extern "C" void meck_apply_pending_admin_cli() {
    if (!g_admin_cli_pending) return;
    g_admin_cli_pending = false;
    if (!g_the_mesh) {
        admin_post_send_result(MECK_ADMIN_REQ_CLI, false, 0);
        return;
    }

    int contact_idx = g_admin_cli_contact_idx;
    uint32_t est_timeout = 0;
    bool ok = g_the_mesh->uiSendCliCommand(contact_idx, g_admin_cli_command,
                                            est_timeout);

    admin_post_send_result(MECK_ADMIN_REQ_CLI, ok, est_timeout);
    printf(">>> %s admin CLI to contact[%d] '%s', est_timeout=%ums\n",
           ok ? "Sent" : "FAILED",
           contact_idx, g_admin_cli_command, (unsigned)est_timeout);
}

extern "C" void meck_apply_pending_admin_telemetry() {
    if (!g_admin_telemetry_pending) return;
    g_admin_telemetry_pending = false;
    if (!g_the_mesh) {
        admin_post_send_result(MECK_ADMIN_REQ_TELEMETRY, false, 0);
        return;
    }

    int contact_idx = g_admin_telemetry_contact_idx;
    uint32_t est_timeout = 0;
    bool ok = g_the_mesh->uiSendTelemetryRequest(contact_idx, est_timeout);

    admin_post_send_result(MECK_ADMIN_REQ_TELEMETRY, ok, est_timeout);
    printf(">>> %s admin telemetry request to contact[%d], est_timeout=%ums\n",
           ok ? "Sent" : "FAILED",
           contact_idx, (unsigned)est_timeout);
}

extern "C" void meck_apply_pending_admin_neighbours() {
    if (!g_admin_neighbours_pending) return;
    g_admin_neighbours_pending = false;
    if (!g_the_mesh) {
        admin_post_send_result(MECK_ADMIN_REQ_NEIGHBOURS, false, 0);
        return;
    }

    int      contact_idx = g_admin_neighbours_contact_idx;
    uint8_t  count       = g_admin_neighbours_count;
    uint16_t offset      = g_admin_neighbours_offset;
    uint8_t  order_by    = g_admin_neighbours_order_by;
    uint8_t  prefix_len  = g_admin_neighbours_pubkey_prefix_length;
    uint32_t est_timeout = 0;
    bool ok = g_the_mesh->uiSendNeighboursRequest(contact_idx, count, offset,
                                                   order_by, prefix_len,
                                                   est_timeout);

    admin_post_send_result(MECK_ADMIN_REQ_NEIGHBOURS, ok, est_timeout);
    printf(">>> %s admin neighbours request to contact[%d] count=%u offset=%u "
           "order=%u prefix_len=%u, est_timeout=%ums\n",
           ok ? "Sent" : "FAILED",
           contact_idx, (unsigned)count, (unsigned)offset,
           (unsigned)order_by, (unsigned)prefix_len,
           (unsigned)est_timeout);
}

extern "C" void meck_admin_clear_session() {
    if (!g_the_mesh) return;
    g_the_mesh->clearAdminSession();
    // Also wipe any pending requests so a stale queue can't trigger
    // sends to a contact the UI thinks it's no longer admin'd on.
    g_admin_login_pending      = false;
    g_admin_status_pending     = false;
    g_admin_cli_pending        = false;
    g_admin_telemetry_pending  = false;
    g_admin_neighbours_pending = false;
    memset(g_admin_login_password, 0, sizeof(g_admin_login_password));
    printf("meck_admin_clear_session: cleared\n");
}

// ---- Callback registration ----

extern "C" void meck_register_admin_send_result_callback(meck_admin_send_result_cb_t cb) {
    g_admin_send_result_cb = cb;
    printf("meck_register_admin_send_result_callback: %s\n", cb ? "registered" : "cleared");
}

extern "C" void meck_register_admin_login_callback(meck_admin_login_cb_t cb) {
    g_admin_login_cb = cb;
    printf("meck_register_admin_login_callback: %s\n", cb ? "registered" : "cleared");
}

extern "C" void meck_register_admin_status_callback(meck_admin_status_cb_t cb) {
    g_admin_status_cb = cb;
    printf("meck_register_admin_status_callback: %s\n", cb ? "registered" : "cleared");
}

extern "C" void meck_register_admin_cli_callback(meck_admin_cli_cb_t cb) {
    g_admin_cli_cb = cb;
    printf("meck_register_admin_cli_callback: %s\n", cb ? "registered" : "cleared");
}

extern "C" void meck_register_admin_telemetry_callback(meck_admin_telemetry_cb_t cb) {
    g_admin_telemetry_cb = cb;
    printf("meck_register_admin_telemetry_callback: %s\n", cb ? "registered" : "cleared");
}

extern "C" void meck_register_admin_neighbours_callback(meck_admin_neighbours_cb_t cb) {
    g_admin_neighbours_cb = cb;
    printf("meck_register_admin_neighbours_callback: %s\n", cb ? "registered" : "cleared");
}

// ---- Response drain (LVGL task, called from ui_update_timer_cb) ----

extern "C" void meck_drain_pending_admin_responses() {
    if (!g_the_mesh) return;

    // Send-result notification first — UI uses this to dismiss
    // "Sending..." spinners and start countdowns. Single-shot per
    // tick; if two completions happened between ticks, only the most
    // recent surfaces (acceptable for sequential admin flow).
    if (g_admin_send_result_pending) {
        g_admin_send_result_pending = false;
        meck_admin_req_type_t type = g_admin_send_result_type;
        bool                  ok   = g_admin_send_result_success;
        uint32_t              t    = g_admin_send_result_timeout;
        if (g_admin_send_result_cb) {
            g_admin_send_result_cb(type, ok, t);
        }
    }

    // Login responses
    {
        Meck::PendingAdminLogin r;
        while (g_the_mesh->drainPendingAdminLogin(r)) {
            if (g_admin_login_cb) {
                g_admin_login_cb(r.success, r.is_admin, r.permissions,
                                 r.fw_ver_level, r.clock_tag, r.contact_idx);
            } else {
                printf("meck_drain_pending_admin_responses: no login callback, "
                       "dropping result success=%d contact=%d\n",
                       r.success ? 1 : 0, r.contact_idx);
            }
        }
    }

    // Status responses
    {
        Meck::PendingAdminStatus r;
        while (g_the_mesh->drainPendingAdminStatus(r)) {
            if (g_admin_status_cb) {
                g_admin_status_cb(&r.stats, r.clock_tag, r.contact_idx);
            } else {
                printf("meck_drain_pending_admin_responses: no status callback, "
                       "dropping result contact=%d\n", r.contact_idx);
            }
        }
    }

    // CLI responses
    {
        Meck::PendingAdminCli r;
        while (g_the_mesh->drainPendingAdminCli(r)) {
            if (g_admin_cli_cb) {
                g_admin_cli_cb(r.text, r.contact_idx);
            } else {
                printf("meck_drain_pending_admin_responses: no CLI callback, "
                       "dropping response contact=%d\n", r.contact_idx);
            }
        }
    }

    // Telemetry responses
    {
        Meck::PendingAdminTelemetry r;
        while (g_the_mesh->drainPendingAdminTelemetry(r)) {
            if (g_admin_telemetry_cb) {
                g_admin_telemetry_cb(r.lpp, r.lpp_len, r.clock_tag, r.contact_idx);
            } else {
                printf("meck_drain_pending_admin_responses: no telemetry callback, "
                       "dropping response contact=%d\n", r.contact_idx);
            }
        }
    }

    // Neighbours responses. The PendingAdminNeighbours.entries field is
    // a NeighbourEntry struct array on Meck's side, but the C-API
    // exposes the page as a packed byte blob (prefix[4] + secs_ago[4]
    // + snr[1] per entry = 9 bytes) so callers don't need to depend
    // on Meck's internal struct layout. Pack on the way out.
    {
        Meck::PendingAdminNeighbours r;
        while (g_the_mesh->drainPendingAdminNeighbours(r)) {
            if (g_admin_neighbours_cb) {
                uint8_t packed[Meck::MECK_NEIGHBOURS_MAX_PAGE * 9];
                uint8_t* p = packed;
                for (uint16_t i = 0; i < r.page_count; i++) {
                    const Meck::NeighbourEntry& e = r.entries[i];
                    memcpy(p, e.prefix, 4); p += 4;
                    memcpy(p, &e.secs_ago, 4); p += 4;
                    *p = (uint8_t)e.snr_x4; p += 1;
                }
                g_admin_neighbours_cb(r.total_count, r.page_count, r.offset,
                                      packed, r.contact_idx);
            } else {
                printf("meck_drain_pending_admin_responses: no neighbours callback, "
                       "dropping response contact=%d total=%u page=%u offset=%u\n",
                       r.contact_idx,
                       (unsigned)r.total_count,
                       (unsigned)r.page_count,
                       (unsigned)r.offset);
            }
        }
    }
}

// ============================================================================
// Deferred SD-save queue for channel messages
// ----------------------------------------------------------------------------
// Ring-write call sites in MeckMesh enqueue completed messages; meck_task
// drains the queue and writes each record to SD via P4DataStore. SD writes
// never block LVGL or receive paths.
//
// Each entry carries (channel_idx, ring_idx, msg_copy):
//   - msg_copy holds the in-memory state captured at enqueue time, which
//     includes timestamp, heard_count, file_offset, and the text.
//   - ring_idx is the index of the source ring slot, used to write the
//     resulting file_offset back to the live ring after an initial append.
//
// On drain, the action depends on msg_copy.file_offset:
//   - file_offset == 0  → initial append. Capture the position returned
//                         by appendChannelMessageRecord and write it back
//                         to the live ring slot (guarded by timestamp so a
//                         ring-overwrite race can't clobber a newer entry).
//   - file_offset != 0  → in-place rewrite at that offset. Used when a
//                         flood echo bumps heard_count on an already-
//                         persisted message; avoids appending duplicates.
//
// Queue size 16: at typical channel-message arrival rates (~1/sec) the
// drain stays empty most of the time. On overflow we drop the oldest
// pending save and log it; the message is still in the in-RAM ring and
// visible until reboot, just not persisted.
// ============================================================================
#define MECK_SAVE_QUEUE_SIZE 16

struct PendingSaveEntry {
    uint8_t channel_idx;
    int     ring_idx;        // location in the source ring (for offset writeback)
    P4ChannelMessage msg;    // captured state (timestamp, heard_count, file_offset, text)
};

static PendingSaveEntry g_save_queue[MECK_SAVE_QUEUE_SIZE];
static volatile int g_save_head = 0;  // next slot to write (producer)
static volatile int g_save_tail = 0;  // next slot to read  (consumer)

extern "C" void meck_request_save_message(uint8_t channel_idx, int ring_idx,
                                          const P4ChannelMessage* msg) {
    if (!msg) return;

    // Dedup: if an entry for this (channel, ring slot) is already pending,
    // skip — the drain will read fresh state via buildMessageRecordSnapshot
    // and pick up whatever the latest heard_count is at drain time. This
    // matters for sent messages: each flood echo would otherwise enqueue
    // another save, and a busy mesh could trivially overflow a 16-deep
    // queue when several repeaters re-broadcast the same packet.
    for (int i = g_save_tail; i != g_save_head; i = (i + 1) % MECK_SAVE_QUEUE_SIZE) {
        if (g_save_queue[i].channel_idx == channel_idx &&
            g_save_queue[i].ring_idx    == ring_idx) {
            return;
        }
    }

    int next_head = (g_save_head + 1) % MECK_SAVE_QUEUE_SIZE;
    if (next_head == g_save_tail) {
        // Queue full — drop oldest pending save by advancing tail.
        g_save_tail = (g_save_tail + 1) % MECK_SAVE_QUEUE_SIZE;
        printf("meck_save: queue full, dropping oldest pending save\n");
    }

    g_save_queue[g_save_head].channel_idx = channel_idx;
    g_save_queue[g_save_head].ring_idx    = ring_idx;
    g_save_queue[g_save_head].msg         = *msg;
    g_save_head = next_head;
}

extern "C" void meck_apply_pending_save() {
    if (g_save_tail == g_save_head) return;  // queue empty
    if (!g_the_mesh) return;
    P4DataStore* store = g_the_mesh->getDataStore();
    if (!store) return;

    // Drain everything pending. Each iteration is one fopen/fwrite/fclose
    // (~1-3 ms on a healthy SD card), so draining 16 entries is well under
    // a 100ms budget.
    while (g_save_tail != g_save_head) {
        PendingSaveEntry& e = g_save_queue[g_save_tail];

        // Snapshot the live ring slot at drain time. This captures the
        // latest heard_count and the current file_offset (which may have
        // been written back by a previous drain pass for the same slot).
        // The expected_timestamp (taken from the enqueue-time copy) guards
        // against ring-slot reuse: if the slot has been overwritten by a
        // newer message with a different timestamp, snapshot fails and we
        // skip this entry rather than persist stale or wrong-slot data.
        P4MsgFileRecord rec;
        uint32_t file_offset = 0;
        bool ok_snap = g_the_mesh->buildMessageRecordSnapshot(
            e.channel_idx, e.ring_idx, e.msg.timestamp, &rec, &file_offset);

        if (!ok_snap) {
            printf("meck_save: ring slot ch[%u] idx[%d] no longer holds ts=%u, "
                   "skipping persist\n",
                   (unsigned)e.channel_idx, e.ring_idx,
                   (unsigned)e.msg.timestamp);
            g_save_tail = (g_save_tail + 1) % MECK_SAVE_QUEUE_SIZE;
            continue;
        }

        if (file_offset != 0) {
            // In-place rewrite — message has been appended before. Most
            // common reason to reach this branch is a heard_count bump
            // triggered by a flood echo of one of our own sends.
            store->rewriteChannelMessageRecord(
                e.channel_idx,
                P4_MSG_FILE_MAGIC, P4_MSG_FILE_VERSION,
                file_offset,
                (uint16_t)sizeof(P4MsgFileRecord),
                &rec);
        } else {
            // Initial append. Capture the offset where the record was
            // placed, then write it back to the live ring slot so future
            // updates can target the same record in-place.
            uint32_t new_offset = 0;
            bool ok = store->appendChannelMessageRecord(
                e.channel_idx,
                P4_MSG_FILE_MAGIC, P4_MSG_FILE_VERSION,
                (uint16_t)sizeof(P4MsgFileRecord),
                &rec,
                &new_offset);
            if (ok && new_offset != 0) {
                // Guarded by expected timestamp inside setMessageFileOffset:
                // if the ring slot has been overwritten between snapshot
                // and writeback, the update is silently skipped.
                g_the_mesh->setMessageFileOffset(
                    e.channel_idx, e.ring_idx,
                    e.msg.timestamp, new_offset);
            }
        }

        g_save_tail = (g_save_tail + 1) % MECK_SAVE_QUEUE_SIZE;
    }
}

// ============================================================================
// Config export bridge for UI thread
// ----------------------------------------------------------------------------
// MeckUI.cpp doesn't directly see g_dataStore or g_node_prefs (both are
// static to this translation unit), so this extern "C" wrapper passes them
// through to meck_export_to_sd. Same pattern as meck_get_instance() for
// the mesh.
// ============================================================================

// ============================================================================
// Voice over LoRa bridge
// ----------------------------------------------------------------------------
// Receive-side: the LVGL task calls meck_drain_pending_voice() periodically,
// which pops VE3 envelopes and 0x56 voice data packets from the Meck pending
// rings and feeds them to the global MeckVoice instance.
//
// Send-side: the LVGL task queues a voice send via meck_request_voice_send().
// meck_task drains via meck_apply_pending_voice_send(), which sends the VE3
// envelope DM followed by staggered raw voice data packets. The Dispatcher's
// delay parameter handles the stagger timing: first data packet at 3s (after
// VE3 + ACK settle), subsequent packets every 3s.
//
// Fetch-serve: when a 0x72 fetch request arrives, the LVGL task responds
// by re-sending the requested packets from the cached outgoing session.
// ============================================================================

#include "MeckVoice.h"

static MeckVoice g_meck_voice;

MeckVoice* meck_get_voice_instance() { return &g_meck_voice; }

// Voice send status for UI polling
#define VOICE_SEND_IDLE           0
#define VOICE_SEND_WAITING_ACK    1
#define VOICE_SEND_PACKETS_QUEUED 2
#define VOICE_SEND_ACK_TIMEOUT    3
#define VOICE_SEND_NO_PATH        4
static volatile int g_voice_send_status = VOICE_SEND_IDLE;
extern "C" int meck_voice_send_get_status() { return g_voice_send_status; }

// ---- Receive drain (called from LVGL task via ui_update_timer_cb) ----
extern "C" void meck_drain_pending_voice() {
    if (!g_the_mesh) return;

    // Drain VE3 envelopes
    Meck::PendingVoiceEnvelope ve;
    while (g_the_mesh->drainPendingVoiceEnvelope(ve)) {
        g_meck_voice.onVE3Received(ve.senderName, ve.ve3Text);
    }

    // Drain voice data packets
    Meck::PendingVoicePacket pkt;
    while (g_the_mesh->drainPendingVoicePacket(pkt)) {
        uint8_t magic = pkt.data[0];
        if (magic == VOICE_PKT_MAGIC && pkt.len > 6) {
            g_meck_voice.onVoicePacketReceived(pkt.data, pkt.len);
        } else if (magic == VOICE_FETCH_MAGIC && pkt.len >= 6) {
            // Fetch request — serve from cached outgoing session
            uint32_t sessionId;
            memcpy(&sessionId, &pkt.data[1], 4);
            if (g_meck_voice.hasValidOutSession() &&
                g_meck_voice.getOutSessionId() == sessionId) {
                printf("Voice: serving fetch for session 0x%08lX\n", (unsigned long)sessionId);
                // Re-send all packets from cache
                // Note: fetch serving requires sendDirect which touches
                // the radio. We'd need to queue this for meck_task. For
                // now, log it — full fetch support comes in a later pass.
                printf("Voice: fetch serve deferred (TODO: queue for meck_task)\n");
            }
        }
    }
}

// ---- Voice send queue (LVGL -> meck_task) ----
// Two-phase send:
//   Phase 1: Send VE3 envelope DM (floods normally, gets ACKed)
//   Phase 2: Poll for ACK. Once ACK arrives (establishing the return path),
//            send raw voice packets via sendDirect ONLY.
//            RAW_CUSTOM packets cannot flood in MeshCore (Mesh.cpp has flood
//            routing commented out for raw packets).

static volatile bool g_voice_send_pending       = false;
static volatile int  g_voice_send_contact_idx   = -1;

// Phase 2 state: waiting for DM ACK before sending raw packets
static bool     g_voice_ack_waiting     = false;
static uint32_t g_voice_ack_expected    = 0;
static uint32_t g_voice_ack_session_id  = 0;
static int      g_voice_ack_contact_idx = -1;
static uint32_t g_voice_ack_sent_ms     = 0;
static uint32_t g_voice_ack_timeout_ms  = 15000;  // per-attempt timeout
static int      g_voice_ack_retries     = 0;
static const int VOICE_DM_MAX_RETRIES   = 5;      // retry DM to establish path

extern "C" void meck_request_voice_send(int contact_idx) {
    g_voice_send_contact_idx = contact_idx;
    g_voice_send_pending = true;
    printf("meck_request_voice_send: queued to contact[%d]\n", contact_idx);
}

// Send raw voice packets via sendDirect (called after path is confirmed)
static void meck_voice_send_direct(int contact_idx, uint32_t sessionId,
                                    ContactInfo* recipient) {
    int totalPkts = g_meck_voice.getPacketCount();
    int sentPkts = 0;

    ContactInfo ci;
    g_the_mesh->getContactByIdx((uint32_t)contact_idx, ci);

    printf("Voice: sending %d packets DIRECT to %s (path_len=%d)\n",
           totalPkts, ci.name, (int)recipient->out_path_len);

    for (int p = 0; p < totalPkts; p++) {
        uint8_t pktBuf[184];
        int pktLen = g_meck_voice.buildVoicePacket(
            pktBuf, sizeof(pktBuf), sessionId, p);
        if (pktLen <= 0) continue;

        mesh::Packet* raw = g_the_mesh->createRawData(pktBuf, pktLen);
        if (!raw) continue;

        // First packet at 1.5s after path confirmed, subsequent 3s apart
        uint32_t delayMs = 1500 + (uint32_t)p * 3000;
        g_the_mesh->sendDirect(raw, recipient->out_path,
                               recipient->out_path_len, delayMs);
        sentPkts++;
        printf("Voice: queued pkt %d/%d (direct, delay %lums)\n",
               p + 1, totalPkts, (unsigned long)delayMs);
    }

    printf("Voice: sent %d/%d packets to %s\n",
           sentPkts, totalPkts, ci.name);
}

// Phase 1: Send VE3 envelope DM, start waiting for ACK
static void meck_voice_send_phase1() {
    if (!g_voice_send_pending) return;
    g_voice_send_pending = false;
    if (!g_the_mesh) return;

    int contact_idx = g_voice_send_contact_idx;
    if (!g_meck_voice.isCodec2Valid()) {
        printf("Voice: no valid Codec2 data to send\n");
        return;
    }

    // Generate session ID
    uint32_t sessionId = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);

    // Cache outgoing session for fetch requests
    g_meck_voice.cacheOutSession(sessionId);

    // Format and send VE3 envelope as a DM
    char envelope[64];
    g_meck_voice.formatEnvelope(envelope, sizeof(envelope), sessionId);
    uint32_t expected_ack = 0;
    bool dmOk = g_the_mesh->sendDirectMessage(contact_idx, envelope, &expected_ack);
    printf("Voice: VE3 DM '%s' to idx %d: %s (expected_ack=0x%08lX)\n",
           envelope, contact_idx, dmOk ? "OK" : "FAIL",
           (unsigned long)expected_ack);

    if (!dmOk) {
        printf("Voice: envelope send failed, aborting\n");
        return;
    }

    // Check if we already have a direct path (from a previous exchange)
    ContactInfo ci;
    g_the_mesh->getContactByIdx((uint32_t)contact_idx, ci);
    ContactInfo* recipient = g_the_mesh->lookupContactByPubKey(
        ci.id.pub_key, PUB_KEY_SIZE);
    bool hasDirect = (recipient && recipient->out_path_len != OUT_PATH_UNKNOWN);

    if (hasDirect) {
        // Path already known, send immediately
        printf("Voice: path already known (path_len=%d), sending direct\n",
               (int)recipient->out_path_len);
        meck_voice_send_direct(contact_idx, sessionId, recipient);
        g_voice_send_status = VOICE_SEND_PACKETS_QUEUED;
    } else {
        // Enter phase 2: wait for ACK to establish path
        g_voice_ack_waiting = true;
        g_voice_ack_expected = expected_ack;
        g_voice_ack_session_id = sessionId;
        g_voice_ack_contact_idx = contact_idx;
        g_voice_ack_sent_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        g_voice_ack_timeout_ms = 15000;
        g_voice_ack_retries = 0;
        g_voice_send_status = VOICE_SEND_WAITING_ACK;
        printf("Voice: waiting for DM ACK (attempt 1/%d)...\n",
               VOICE_DM_MAX_RETRIES);
    }
}

// Phase 2: Poll for ACK (called every meck_task tick, ~10ms)
static void meck_voice_send_poll_ack() {
    if (!g_voice_ack_waiting) return;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    uint32_t elapsed = now_ms - g_voice_ack_sent_ms;

    // Check ACK status
    bool acked = false;
    unsigned long msg_sent_millis = 0;
    uint32_t est_timeout = 0;

    if (g_the_mesh->lookupDMAckStatus(g_voice_ack_expected,
                                       acked, msg_sent_millis, est_timeout)) {
        if (acked) {
            // ACK received! Path should now be known.
            g_voice_ack_waiting = false;

            ContactInfo ci;
            g_the_mesh->getContactByIdx((uint32_t)g_voice_ack_contact_idx, ci);
            ContactInfo* recipient = g_the_mesh->lookupContactByPubKey(
                ci.id.pub_key, PUB_KEY_SIZE);

            if (recipient && recipient->out_path_len != OUT_PATH_UNKNOWN) {
                printf("Voice: ACK received after %lums, path established (path_len=%d)\n",
                       (unsigned long)elapsed, (int)recipient->out_path_len);
                meck_voice_send_direct(g_voice_ack_contact_idx,
                                        g_voice_ack_session_id, recipient);
                g_voice_send_status = VOICE_SEND_PACKETS_QUEUED;
            } else {
                g_voice_send_status = VOICE_SEND_NO_PATH;
                printf("Voice: ACK received but path still unknown! Cannot send.\n");
            }
            return;
        }
    }

    // Check timeout — retry if attempts remain
    if (elapsed >= g_voice_ack_timeout_ms) {
        g_voice_ack_retries++;
        if (g_voice_ack_retries < VOICE_DM_MAX_RETRIES) {
            // Retry: re-send the VE3 envelope DM
            printf("Voice: DM ACK timeout after %lums, retrying (attempt %d/%d)...\n",
                   (unsigned long)elapsed,
                   g_voice_ack_retries + 1, VOICE_DM_MAX_RETRIES);

            char envelope[64];
            g_meck_voice.formatEnvelope(envelope, sizeof(envelope),
                                         g_voice_ack_session_id);
            uint32_t new_ack = 0;
            bool ok = g_the_mesh->sendDirectMessage(
                g_voice_ack_contact_idx, envelope, &new_ack);
            if (ok && new_ack != 0) {
                g_voice_ack_expected = new_ack;
                g_voice_ack_sent_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
                printf("Voice: retry DM sent (expected_ack=0x%08lX)\n",
                       (unsigned long)new_ack);
            } else {
                printf("Voice: retry DM send failed\n");
                g_voice_ack_waiting = false;
                g_voice_send_status = VOICE_SEND_ACK_TIMEOUT;
            }
        } else {
            // All retries exhausted
            g_voice_ack_waiting = false;
            g_voice_send_status = VOICE_SEND_ACK_TIMEOUT;
            printf("Voice: DM ACK timeout after %d attempts. No path established.\n",
                   VOICE_DM_MAX_RETRIES);
        }
    }
}

static void meck_apply_pending_voice_send() {
    meck_voice_send_phase1();
    meck_voice_send_poll_ack();
}

// ============================================================================
// Picture transfer over channel messages
// ============================================================================

#include "MeckPicture.h"

static MeckPictureSend g_pic_send;
static MeckPictureRecv g_pic_recv;

// Drain pending picture chunks from MeckMesh ring (called from LVGL task)
extern "C" void meck_drain_pending_pictures() {
    if (!g_the_mesh) return;

    Meck::PendingPicChunk pc;
    while (g_the_mesh->drainPendingPicChunk(pc)) {
        bool complete = g_pic_recv.onChunkReceived(pc.text, pc.sender);
        if (complete) {
            char pic_path[128] = {};
            if (g_pic_recv.saveToSD(pic_path, sizeof(pic_path))) {
                printf("PicRecv: complete, injecting [PIC:%s] on ch[%d]\n",
                       pic_path, pc.channel_idx);
                // Inject a single [PIC:path] message into the channel ring
                char pic_msg[160];
                snprintf(pic_msg, sizeof(pic_msg), "[PIC:%s]", pic_path);
                g_the_mesh->injectChannelMessage(pc.channel_idx, pic_msg);
            }
        }
    }
}

// Request a picture send (called from LVGL task)
static volatile bool g_pic_send_pending = false;
static volatile uint8_t g_pic_send_ch_idx = 0;
static char g_pic_send_path[128] = {};

extern "C" void meck_request_picture_send(uint8_t ch_idx, const char *filepath) {
    strncpy(g_pic_send_path, filepath, sizeof(g_pic_send_path) - 1);
    g_pic_send_path[sizeof(g_pic_send_path) - 1] = '\0';
    g_pic_send_ch_idx = ch_idx;
    g_pic_send_pending = true;
    printf("meck_request_picture_send: queued %s on ch[%d]\n", filepath, ch_idx);
}

// Tick: start pending send or send next chunk
static void meck_apply_pending_picture_send() {
    // Start new send
    if (g_pic_send_pending && !g_pic_send.active) {
        g_pic_send_pending = false;
        g_pic_send.start(g_pic_send_ch_idx, g_pic_send_path);
    }

    // Send next chunk if ready
    if (g_pic_send.active && g_pic_send.isReady() && g_the_mesh) {
        bool was_active = g_pic_send.active;
        char msg[200];
        const char *chunk = g_pic_send.buildNextChunk(msg, sizeof(msg));
        if (chunk) {
            g_the_mesh->sendChannelMessage(g_pic_send.channel_idx, chunk);
        }
        // All chunks sent — inject a single [PIC:path] into the sender's
        // own channel ring so they see the image in their chat.
        if (was_active && !g_pic_send.active && g_the_mesh) {
            char pic_msg[160];
            snprintf(pic_msg, sizeof(pic_msg), "[PIC:%s]", g_pic_send_path);
            g_the_mesh->injectChannelMessage(g_pic_send_ch_idx, pic_msg);
            printf("PicSend: complete, injected local [PIC:%s]\n", g_pic_send_path);
        }
    }
}

// Process an incoming channel message for picture chunks.
// Returns true if the message is a picture chunk (caller can suppress display).
extern "C" bool meck_picture_on_channel_msg(const char *text, const char *sender,
                                             char *pic_path_out, int pic_path_len) {
    if (!text || strncmp(text, PIC_CHUNK_MARKER, PIC_CHUNK_MARKER_LEN) != 0)
        return false;

    bool complete = g_pic_recv.onChunkReceived(text, sender);
    if (complete) {
        g_pic_recv.saveToSD(pic_path_out, pic_path_len);
    } else {
        pic_path_out[0] = '\0';
    }
    return true;
}

extern "C" bool meck_picture_is_sending() {
    return g_pic_send.active;
}

extern "C" int meck_picture_send_progress_pct() {
    if (!g_pic_send.active) return 100;
    return g_pic_send.next_chunk * 100 / g_pic_send.total_chunks;
}

extern "C" bool meck_export_to_sd_with_flags(uint32_t flags,
                                             char* out_path,
                                             size_t out_path_size) {
    return meck_export_to_sd(g_dataStore, g_node_prefs, flags,
                             out_path, out_path_size);
}
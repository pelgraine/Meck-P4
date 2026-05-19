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

    printf("meck_app_init: Meck stack ready\n");
    return true;
}

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
        meck_apply_pending_save();
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

extern "C" void meck_admin_clear_session() {
    if (!g_the_mesh) return;
    g_the_mesh->clearAdminSession();
    // Also wipe any pending requests so a stale queue can't trigger
    // sends to a contact the UI thinks it's no longer admin'd on.
    g_admin_login_pending     = false;
    g_admin_status_pending    = false;
    g_admin_cli_pending       = false;
    g_admin_telemetry_pending = false;
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
extern "C" bool meck_export_to_sd_with_flags(uint32_t flags,
                                             char* out_path,
                                             size_t out_path_size) {
    return meck_export_to_sd(g_dataStore, g_node_prefs, flags,
                             out_path, out_path_size);
}
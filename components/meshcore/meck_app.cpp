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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>

// ---- Static instances ----
static P4DataStore g_dataStore;
static P4NodePrefs g_node_prefs;
static P4RNG g_rng;
static P4RTCClock g_rtc;
static SimpleMeshTables g_mesh_tables;
static Meck* g_the_mesh = nullptr;

// ---- Internal accessor (declared in target.h) ----
Meck* meck_get_instance() { return g_the_mesh; }

// ---- App init ----
extern "C" bool meck_app_init() {
    printf("meck_app_init: starting\n");

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
// Deferred SD-save queue for channel messages
// ----------------------------------------------------------------------------
// Ring-write call sites in MeckMesh enqueue completed messages. The
// meck_task drains the queue and writes each record to SD via the
// P4DataStore append helper. SD writes never block LVGL or receive paths.
//
// Queue size 16: at typical channel-message arrival rates (~1/sec) the
// drain stays empty most of the time. On overflow we drop the oldest
// pending save and log it; the message is still in the in-RAM ring and
// visible until reboot, just not persisted.
// ============================================================================
#define MECK_SAVE_QUEUE_SIZE 16

struct PendingSaveEntry {
    uint8_t channel_idx;
    P4ChannelMessage msg;
};

static PendingSaveEntry g_save_queue[MECK_SAVE_QUEUE_SIZE];
static volatile int g_save_head = 0;  // next slot to write (producer)
static volatile int g_save_tail = 0;  // next slot to read  (consumer)

extern "C" void meck_request_save_message(uint8_t channel_idx, const P4ChannelMessage* msg) {
    if (!msg) return;

    int next_head = (g_save_head + 1) % MECK_SAVE_QUEUE_SIZE;
    if (next_head == g_save_tail) {
        // Queue full — drop oldest pending save by advancing tail.
        g_save_tail = (g_save_tail + 1) % MECK_SAVE_QUEUE_SIZE;
        printf("meck_save: queue full, dropping oldest pending save\n");
    }

    g_save_queue[g_save_head].channel_idx = channel_idx;
    g_save_queue[g_save_head].msg = *msg;
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

        // Convert in-memory P4ChannelMessage to packed on-disk record.
        P4MsgFileRecord rec = {};
        rec.timestamp    = e.msg.timestamp;
        rec.dm_peer_hash = 0;                        // reserved for DM use
        rec.channel_idx  = e.msg.channel_idx;
        rec.path_len     = e.msg.path_len;
        rec.snr_x4       = 0;                        // reserved (schema v2)
        rec.flags        = e.msg.valid ? 0x01 : 0x00;
        memset(rec.path, 0, sizeof(rec.path));       // reserved (schema v2)
        memcpy(rec.text, e.msg.text, P4_MSG_TEXT_LEN);

        store->appendChannelMessageRecord(
            e.channel_idx,
            P4_MSG_FILE_MAGIC, P4_MSG_FILE_VERSION,
            (uint16_t)sizeof(P4MsgFileRecord),
            &rec);

        g_save_tail = (g_save_tail + 1) % MECK_SAVE_QUEUE_SIZE;
    }
}
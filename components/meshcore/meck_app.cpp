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

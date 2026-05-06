/*
 * meck.h — public API for the Meck mesh stack on T-Display P4
 *
 * This is the only header to include from main.cpp. Internal implementation
 * details (radio adapter, mesh class, data store) are hidden behind these
 * three functions and live in target.h / Meck.h / MeckDataStore.h.
 *
 * Boot sequence in main.cpp's app_main():
 *
 *   1. (LilyGo's existing init: XL9535, SX1262, screen, touch, audio, SD,
 *      GPS, battery gauge, LVGL, all xTaskCreate calls EXCEPT device_rf_task
 *      which we replace.)
 *
 *   2. meck_radio_attach()    — wrap LilyGo's SX1262, set Sydney mesh params
 *   3. meck_app_init()        — NVS init, identity load/generate, channels,
 *                                contacts, full Meck stack ready
 *   4. meck_app_start()       — spawn meck_task, mesh starts processing RX
 *   5. System_Startup_Message_Init();   (LilyGo's modal popups for any
 *                                         failed init steps)
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Wrap LilyGo's already-running SX1262 with our mesh::Radio adapter and
// reconfigure to MeshCore's Australia Narrow preset. Idempotent.
bool meck_radio_attach();

// Initialize the Meck mesh stack. Must be called AFTER meck_radio_attach().
// Initializes NVS, loads or generates the node identity (persisted to SD),
// loads or creates default channels, loads stored contacts.
// Returns true on success.
bool meck_app_init();

// Spawn the meck_task that drives the mesh protocol loop. Must be called
// AFTER meck_app_init(). Runs forever.
void meck_app_start();

#ifdef __cplusplus
}
#endif

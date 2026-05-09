/*
 * meck.h — public API for the Meck mesh stack on T-Display P4
 *
 * This is the only header to include from main.cpp. Internal implementation
 * details (radio adapter, mesh class, data store, UI) are hidden behind
 * these functions and live in target.h / MeckMesh.h / MeckDataStore.h /
 * MeckUI.h.
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
 *   6. meck_ui_init()         — build LVGL home screen + 7-tile structure
 *   7. meck_ui_show_home()    — switch the active screen to ours
 */

#pragma once

#include <stdint.h>

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

// Build all LVGL objects for the home screen and start the 500ms refresh
// timer. Must be called AFTER meck_app_init() so the Meck instance exists,
// and AFTER System_Startup_Message_Init() so any popups have finished.
void meck_ui_init();

// Switch the active LVGL screen to the Meck home screen. Must be called
// AFTER meck_ui_init().
void meck_ui_show_home();

// Push a UTC epoch (seconds since 1970-01-01 00:00 UTC) into the soft RTC.
// Called from main.cpp's device_gps_task whenever a fresh, sane RMC time
// is parsed. Cheap: just stores _epoch_offset = utc - boot_seconds, so
// safe to call at the GPS sentence rate (~1 Hz).
void meck_clock_set_utc(uint32_t epoch);

// Read the soft RTC. Returns 0 if it has never been set, otherwise the
// current UTC epoch in seconds.
uint32_t meck_clock_get_utc(void);

#ifdef __cplusplus
}
#endif
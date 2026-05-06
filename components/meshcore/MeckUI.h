/*
 * MeckUI.h — public LVGL UI API for Meck on T-Display P4
 *
 * The UI is structured as a horizontal tileview with 7 tiles:
 *   0 Home        — title, freq/SF/RSSI/RX header, 3x2 navigation grid
 *   1 Recent Heard
 *   2 Radio Details
 *   3 Advert (long-press to send)
 *   4 GPS
 *   5 Battery Gauge
 *   6 Hibernate
 *
 * In the current build only tile 0 is fully populated; tiles 1-6 are
 * title-only stubs ("coming soon"). They get filled in screen-by-screen.
 *
 * Tab buttons (Messages, Contacts, Settings, Reader, Notes, Discover) on
 * the home grid currently print TODO lines to serial when tapped. Real
 * sub-screen navigation comes later.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Build all LVGL objects for the home screen and start the 500ms refresh
// timer. Must be called after meck_app_init() so the Meck instance exists,
// and after System_Startup_Message_Init() so any popups from LilyGo's
// factory init have finished.
void meck_ui_init();

// Switch the active LVGL screen to the Meck home screen. Must be called
// after meck_ui_init().
void meck_ui_show_home();

#ifdef __cplusplus
}
#endif

/*
 * MeckGBC.h -- Game Boy Color emulator module, public API
 * ----------------------------------------------------------------------------
 * Games menu + ROM browser + emulator screen, built on the Peanut-GB core
 * (tvecera gbc-rtc-fix branch, vendored as peanut_gb.h). Keyboard (K270)
 * builds only: on the plain T-Display P4 board type these calls are no-op
 * stubs and the Games tile keeps its placeholder.
 *
 * Screens are created on show and deleted on exit, so they are always laid
 * out for the current orientation and never join the orientation-rebuild
 * teardown list.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Open the Games menu (the Games home tile's target on keyboard builds).
// Must be called from LVGL task context (a widget event callback or timer).
void meck_gbc_show_menu(void);

// True while the emulator task is running a ROM.
bool meck_gbc_running(void);

#ifdef __cplusplus
}
#endif

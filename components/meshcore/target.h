/*
 * target.h — MeshCore hardware target for T-Display P4 (Meck)
 *
 * Trimmed for the LilyGo lvgl_9_ui-based build:
 *   LilyGo's app_main() initialises the radio hardware (XL9535, SPI, SX1262
 *   power/reset, sync_word=0x1424). We attach an adapter on top and
 *   reconfigure the LoRa params to MeshCore's Australia Narrow preset.
 *
 *   P4Board.h and P4SDCard.h are intentionally NOT included — board
 *   peripherals and SD are handled by LilyGo's init.
 */

#pragma once

#include "variant.h"
#include "P4SX1262Radio.h"

// MeshCore radio adapter (defined in target.cpp)
extern P4SX1262Radio radio_driver;

// cpp_bus_driver SX1262 instance — duplicate client object pointing at the
// same hardware LilyGo already initialised. Defined in target.cpp.
extern std::unique_ptr<Cpp_Bus_Driver::Sx126x> SX1262;

// MeshCore target interface
//
// meck_radio_attach() — call ONCE from app_main() AFTER LilyGo's init has
// completed. Constructs the cpp_bus_driver client object, applies MeshCore
// LoRa parameters, and puts the radio into RX mode.
bool meck_radio_attach();

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr);
void radio_set_tx_power(uint8_t dbm);
uint32_t radio_get_rng_seed();

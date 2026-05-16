/*
 * target.h — INTERNAL header for the meshcore component.
 *
 * This is included by target.cpp / meck_globals.cpp / future Meck code
 * within the meshcore component. main.cpp deliberately does NOT include
 * target.h — it includes the much smaller meck.h instead.
 *
 * Reason: target.h pulls in P4SX1262Radio.h (and via that, MeshCore protocol
 * headers and the SX1262 extern). Including all of that from main.cpp
 * triggers -Wreorder errors under main's stricter compile flags, and the
 * SX1262 extern collides with LilyGo's `auto SX1262 = ...` global.
 */

#pragma once

#include "variant.h"
#include "P4SX1262Radio.h"
#include "meck.h"   // for meck_radio_attach() declaration

// MeshCore radio adapter (defined in target.cpp)
extern P4SX1262Radio radio_driver;

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr);
void radio_set_tx_power(uint8_t dbm);
uint32_t radio_get_rng_seed();

// Internal accessor — UI code uses this to read messages, contacts, recent heard
class Meck;
Meck* meck_get_instance();

// Deferred radio reconfig. Call radio_request_reconfig() from any task to
// queue a config change. meck_task picks it up at the top of its loop via
// radio_apply_pending_reconfig() and applies it in its own context, avoiding
// SPI bus contention with mesh.loop().
extern "C" void radio_request_reconfig(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t tx_power);
extern "C" void radio_apply_pending_reconfig();

// Deferred send-message. LVGL task calls meck_request_send_text() to queue;
// meck_task picks up via meck_apply_pending_send() at the top of its loop.
// Avoids the same SPI race as radio_request_reconfig.
extern "C" void meck_request_send_text(uint8_t channel_idx, const char* text);
extern "C" void meck_apply_pending_send();

// Deferred SD-save for channel messages. ring-write call sites enqueue;
// meck_task drains via meck_apply_pending_save() and writes to SD without
// blocking LVGL or message receive paths. ring_idx is the slot's position
// in the channel's ring buffer; the drain uses it to write the resulting
// file_offset back into the live ring after an initial append, and to
// snapshot the latest in-memory state (heard_count, file_offset) at drain
// time so multiple coalesced echoes resolve to a single in-place rewrite.
struct P4ChannelMessage;
extern "C" void meck_request_save_message(uint8_t channel_idx, int ring_idx,
                                          const P4ChannelMessage* msg);
extern "C" void meck_apply_pending_save();

// Sets SKY13453 VCTL to a known antenna port (default HIGH = antenna A).
// Should be called once at boot during meck_radio_attach.
extern "C" void meck_set_antenna_default();

// ---- Boot button (ESP32-P4 strapping pin, direct GPIO) ----
//
// PIN_BOOT_BTN (GPIO 35) doubles as the BOOT-0 strapping pin used by the
// bootloader to enter download mode. It has an external pull-up on the
// LilyGo board; the physical button shorts it to GND when pressed. So:
// pressed == level 0, released == level 1.
//
// Init configures the pin as plain input with no internal pull resistors
// (the external pull-up is sufficient and we don't want to fight it).
// Safe to call once at boot from the LVGL task.
extern "C" void meck_boot_button_init();
extern "C" bool meck_boot_button_pressed();

// ---- Battery readout (BQ27220 fuel gauge owned by main.cpp) ----
//
// Voltage is always trustworthy. SoC% from BQ27220 depends on cell
// calibration in the chip's data flash, which may drift; treat as
// advisory and compare against meck_battery_pct_from_voltage(mv).
extern "C" bool     meck_battery_available();
extern "C" uint16_t meck_battery_voltage_mv();
extern "C" int16_t  meck_battery_current_ma();
extern "C" uint8_t  meck_battery_pct_from_chip();
extern "C" int8_t   meck_battery_temp_c();
extern "C" uint16_t meck_battery_remaining_mah();
extern "C" uint16_t meck_battery_full_charge_mah();
extern "C" uint16_t meck_battery_time_to_empty_min();
extern "C" uint8_t  meck_battery_pct_from_voltage(uint16_t mv);

// ---- GPS readout (L76K module owned by main.cpp) ----
//
// LilyGo's device_gps_task in main.cpp parses RMC + GGA into Sys_Status.l76k
// at ~1Hz. These accessors copy a snapshot out so UI code (which runs in
// the LVGL task) can read it without seeing main.cpp's System_Status type.
//
// Lat/lon are stored as signed degrees * 1e7 (atomic 32-bit on the P4,
// matches the e7 format used by Meck contact records). Position values
// are only meaningful when fix_valid is true.
extern "C" {
    struct MeckGpsSnapshot {
        bool     init_ok;
        bool     fix_valid;
        char     status;            // 'A' active, 'V' void, 'D' differential
        uint8_t  gps_mode;          // 0 none, 1 GPS, 2 DGPS, 6 DR
        uint8_t  sats_used;
        uint8_t  sats_visible;      // GSV total across all constellations
        float    hdop;
        int32_t  lat_e7;
        int32_t  lon_e7;
        float    altitude_m;
        bool     altitude_valid;
        bool     time_valid;
        uint16_t year;
        uint8_t  month, day, hour, minute;
        float    second;
        uint32_t time_to_first_fix_s;
        uint32_t sentence_count;
        uint32_t last_sentence_ms;
    };
    void meck_gps_get_snapshot(MeckGpsSnapshot *out);
}

// Sets SKY13453 VCTL to a known antenna port (HIGH = RF1 internal, default).
// Should be called once at boot during meck_radio_attach.
extern "C" void meck_set_antenna_default();
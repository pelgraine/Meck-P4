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

// ---- Direct message bridge (LVGL <-> mesh task) ----
//
// Send path: LVGL calls meck_request_send_dm() which queues the request;
// meck_task drains via meck_apply_pending_send_dm() at the top of its loop.
// Mirrors meck_request_send_text exactly — same SPI-race avoidance applies.
//
// Receive path: pull-based. meck_task fills Meck's pending-DM ring inside
// onMessageRecv. The LVGL task calls meck_drain_pending_dms() periodically
// (from ui_update_timer_cb), which pops queued DMs and invokes the
// callback registered via meck_register_dm_recv_callback. The callback
// thereby runs on the LVGL task, safe to touch LVGL state.
//
// The callback is given the sender's pub_key (32 bytes), sender name,
// message text, sender_timestamp, path_len (0xFF for direct route, raw
// hop count for flooded), and SNR×4. Lifetimes: the pointers are only
// valid for the duration of the callback — the UI must copy anything it
// wants to retain.
typedef void (*meck_dm_recv_cb_t)(const uint8_t* from_pub_key,
                                  const char* from_name,
                                  const char* text,
                                  uint32_t sender_timestamp,
                                  uint8_t path_len,
                                  int8_t snr_x4);

extern "C" void meck_request_send_dm(int contact_idx, const char* text);
extern "C" void meck_apply_pending_send_dm();
extern "C" void meck_register_dm_recv_callback(meck_dm_recv_cb_t cb);
extern "C" void meck_drain_pending_dms();

// ---- DM send completion bridge (mesh task -> LVGL) ----
//
// When meck_apply_pending_send_dm successfully transmits a DM, it
// records the expected_ack value (matching the entry the mesh placed
// in the ACK table) plus the estimated round-trip timeout. The UI
// thread pulls this on its periodic tick via meck_drain_pending_dm_sends
// and invokes the registered callback, which writes the expected_ack
// onto the matching outgoing DMMessage in the per-contact ring so the
// bubble can later poll Meck::lookupDMAckStatus and render "Delivered"
// / "Sending..." / "Failed".
//
// Same threading rationale as the receive path — callback runs on the
// LVGL task, safe to touch LVGL state.
typedef void (*meck_dm_sent_cb_t)(int contact_idx,
                                  uint32_t expected_ack,
                                  uint32_t est_timeout_ms);

extern "C" void meck_register_dm_sent_callback(meck_dm_sent_cb_t cb);
extern "C" void meck_drain_pending_dm_sends();

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

// ---- Config export (user-triggered, MeshCore app-compatible JSON) ----
//
// UI bridge to MeckExport.h. UI code (MeckUI.cpp) doesn't see P4DataStore
// or cJSON directly — this function reaches the static instances in
// meck_app.cpp and passes them through. flags is a bitmask of
// MECK_EXPORT_* values (mirrored here so UI code doesn't have to include
// MeckExport.h). On success, the resulting filename basename (e.g.
// "export-1737253200.json") is copied into out_path. Safe to call inline
// from an LVGL handler — the I/O is bounded (single JSON serialise +
// one file write).
#define MECK_EXPORT_IDENTITY  (1u << 0)
#define MECK_EXPORT_CHANNELS  (1u << 1)
#define MECK_EXPORT_CONTACTS  (1u << 2)
#define MECK_EXPORT_RADIO     (1u << 3)
#define MECK_EXPORT_ALL       (MECK_EXPORT_IDENTITY | MECK_EXPORT_CHANNELS | \
                               MECK_EXPORT_CONTACTS | MECK_EXPORT_RADIO)

extern "C" bool meck_export_to_sd_with_flags(uint32_t flags,
                                             char* out_path,
                                             size_t out_path_size);

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

// ---- Battery calibration (BQ27220 fuel gauge, full TI procedure) ----
//
// LilyGo's main.cpp calls BQ27220->set_design_capacity(1000) on every boot
// via the Cpp_Bus_Driver wrapper, but the wrapper's minimal implementation
// skips several steps the TI TRM (SLUUBD4) requires for the change to take
// effect: it doesn't write Design Energy at 0x92A1, it exits CFG_UPDATE
// with 0x0092 (no REINIT) instead of 0x0091, and it doesn't issue a SEAL
// or RESET. The chip's reported FCC therefore stays pinned at the factory
// 3000 mAh until a full charge/discharge cycle triggers a natural relearn,
// which is why meck_battery_full_charge_mah() keeps logging
// "chip FCC=3000 > design=1000, capping" on every boot.
//
// meck_battery_calibrate() runs the full procedure proven on T-Deck Pro:
// Unseal + Full Access, Enter CFG_UPDATE, write Design Energy / Qmax /
// stored FCC via MAC differential checksum, Exit with REINIT, Seal, RESET.
// Self-gated: returns immediately when DC is correct and FCC is already
// inside [target-100, target+100]. The first run after this lands takes
// about 2 seconds because of the post-RESET settling delay; later boots
// short-circuit at the FCC band check and return in milliseconds.
extern "C" void meck_battery_calibrate();

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
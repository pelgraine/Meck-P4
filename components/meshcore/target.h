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

// ---- Repeater Admin bridge (LVGL <-> mesh task) ----
//
// Same threading model as the DM bridges: LVGL queues requests via
// meck_request_admin_*; meck_task drains via meck_apply_pending_admin_*
// at the top of its loop and invokes the appropriate Meck::ui*
// method. Responses arrive on meck_task (inside onContactResponse /
// onCommandDataRecv) which fills per-type pending rings inside Meck;
// the LVGL task drains those rings via meck_drain_pending_admin_responses
// and dispatches each entry to the matching registered callback.
//
// Single-session policy is enforced by Meck (calling uiLoginToRepeater
// for a new contact tears down the prior session). The bridge here is
// fire-and-forget: the LVGL task doesn't need to track which contact a
// queued request was for — each response carries the contact_idx.
//
// Four request types: login (needs password), status (parameter-free),
// CLI (needs command string), telemetry (parameter-free). One-deep
// queue each. If the user double-taps the second tap overwrites the
// first pending entry (consistent with the DM send queue).
//
// Send-result callback: fires after every admin-send-attempt on the
// LVGL task with the request type, success flag, and est_timeout_ms.
// Used by the UI for two things:
//   - immediate failure UI when the send itself failed (contact
//     unreachable, MSG_SEND_FAILED) so the user doesn't sit in
//     "Sending..." forever
//   - sizing a countdown timer for the response-pending state
//
// Response callbacks: fire once a matching response lands. The data
// pointed to is only valid for the duration of the callback — the UI
// must copy anything it wants to retain.

typedef enum {
    MECK_ADMIN_REQ_LOGIN      = 0,
    MECK_ADMIN_REQ_STATUS     = 1,
    MECK_ADMIN_REQ_CLI        = 2,
    MECK_ADMIN_REQ_TELEMETRY  = 3,
    MECK_ADMIN_REQ_NEIGHBOURS = 4,
} meck_admin_req_type_t;

// Send-result callback: fires after every admin send attempt.
typedef void (*meck_admin_send_result_cb_t)(meck_admin_req_type_t type,
                                            bool success,
                                            uint32_t est_timeout_ms);

// Response callbacks — one per response type.
//
// Login: success indicates whether the repeater accepted the password.
// is_admin distinguishes admin (1) from guest (0). permissions is the
// raw permissions byte. fw_ver_level gates UI features that require
// newer repeater firmware (e.g. owner.info on >= 2). clock_tag is the
// repeater's clock at login time (use as "Clock · At Login"
// display). contact_idx identifies which repeater the result belongs
// to.
typedef void (*meck_admin_login_cb_t)(bool success, uint8_t is_admin,
                                       uint8_t permissions,
                                       uint8_t fw_ver_level,
                                       uint32_t clock_tag,
                                       int contact_idx);

// Status: full 56-byte RepeaterStats copy. clock_tag is the response
// timestamp (refreshed clock readout on each request).
struct RepeaterStats;  // forward-decl, defined in MeckMesh.h
typedef void (*meck_admin_status_cb_t)(const RepeaterStats* stats,
                                        uint32_t clock_tag,
                                        int contact_idx);

// CLI: plain text response from a `neighbors` / `ver` / `get foo` /
// etc. command. Up to 255 chars (truncated, NUL-terminated).
typedef void (*meck_admin_cli_cb_t)(const char* response,
                                     int contact_idx);

// Telemetry: CayenneLPP buffer. Variable length, capped at 160 bytes.
typedef void (*meck_admin_telemetry_cb_t)(const uint8_t* lpp,
                                           uint8_t lpp_len,
                                           uint32_t clock_tag,
                                           int contact_idx);

// Neighbours: one page of the repeater's neighbour list. total_count is
// the total number of neighbours the repeater has; page_count is how
// many are in this `entries` blob. offset is the offset this page was
// requested with (so the caller can correlate when iterating).
// entries is a packed blob of `page_count` records, each laid out as
// 4 bytes pubkey prefix + 4 bytes seconds-ago + 1 byte SNR*4. The
// pointer is only valid for the duration of the callback — copy out
// anything you want to retain.
typedef void (*meck_admin_neighbours_cb_t)(uint16_t total_count,
                                            uint16_t page_count,
                                            uint16_t offset,
                                            const uint8_t* entries,
                                            int contact_idx);

// Send-side: LVGL queues, meck_task drains. Each request type has its
// own pending slot; calling the same request twice in a row before the
// drain runs overwrites the first request.
extern "C" void meck_request_admin_login(int contact_idx, const char* password);
extern "C" void meck_request_admin_status(int contact_idx);
extern "C" void meck_request_admin_cli(int contact_idx, const char* command);
extern "C" void meck_request_admin_telemetry(int contact_idx);
extern "C" void meck_request_admin_neighbours(int contact_idx,
                                              uint8_t count,
                                              uint16_t offset,
                                              uint8_t order_by,
                                              uint8_t pubkey_prefix_length);

extern "C" void meck_apply_pending_admin_login();
extern "C" void meck_apply_pending_admin_status();
extern "C" void meck_apply_pending_admin_cli();
extern "C" void meck_apply_pending_admin_telemetry();
extern "C" void meck_apply_pending_admin_neighbours();

// Tear down the active admin session from the LVGL side. Mirrors
// Meck::clearAdminSession but callable from UI without needing to
// touch the mesh instance directly. Safe to call any time.
extern "C" void meck_admin_clear_session();

// Callback registration. The UI registers each callback once at boot
// and routes internally by request type.
extern "C" void meck_register_admin_send_result_callback(meck_admin_send_result_cb_t cb);
extern "C" void meck_register_admin_login_callback(meck_admin_login_cb_t cb);
extern "C" void meck_register_admin_status_callback(meck_admin_status_cb_t cb);
extern "C" void meck_register_admin_cli_callback(meck_admin_cli_cb_t cb);
extern "C" void meck_register_admin_telemetry_callback(meck_admin_telemetry_cb_t cb);
extern "C" void meck_register_admin_neighbours_callback(meck_admin_neighbours_cb_t cb);

// Drain all four response rings and invoke registered callbacks. Called
// from ui_update_timer_cb on the LVGL task. Cheap when nothing is
// queued (early return on empty ring).
extern "C" void meck_drain_pending_admin_responses();

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
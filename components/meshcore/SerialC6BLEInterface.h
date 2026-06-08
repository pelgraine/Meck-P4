/*
 * SerialC6BLEInterface.h -- BLE HID keyboard host via ESP32-C6 AT over SDIO
 *
 * REPURPOSED for Meck-P4 v0.6: this file previously implemented a BLE
 * *companion* transport (device acting as a GATT server/peripheral so a
 * phone could connect). That companion role moved to WiFi, so this is now
 * a BLE *central* that connects to an external Bluetooth Low Energy
 * keyboard (HID-over-GATT) and feeds keystrokes into the UI.
 *
 * The ESP32-P4 has no radio; BLE is the ESP32-C6 coprocessor running
 * ESP-AT firmware (v4.1.0.0_dev slave in firmware/), reached over SDIO via
 * the shared Cpp_Bus_Driver::Esp_At driver. The C6 acts as the GATT
 * client. We drive it with AT+BLEINIT=1 (client), AT+BLESCAN, AT+BLECONN,
 * AT+BLEGATTC* discovery, and decode +NOTIFY input reports.
 *
 * Pairing model (Option B): the UI opens a pairing sub-screen, calls
 * startBrowse() to scan, reads the discovered list via scanCount()/getScan(),
 * and calls connectTo() when the user taps a keyboard. The chosen address is
 * persisted by the caller (prefs.kbd_addr); on a later enable() with that
 * address set, the keyboard reconnects directly without browsing.
 *
 * Filename / include path unchanged so CMakeLists and the meck_app.cpp
 * include need no edits; only the class name changed (SerialC6BLEInterface
 * -> C6BleKeyboard).
 *
 * Threading model:
 *   - poll(), enable(), disable(), startBrowse(), stopBrowse(), connectTo()
 *     all touch the C6 over SDIO and MUST be called only from meck_task.
 *   - read_key(), scanCount(), getScan(), state(), isConnected() are called
 *     from the LVGL/UI task. read_key() drains a lock-free SPSC ring; the
 *     scan list is append-only on the producer side with a volatile count
 *     published last, so UI reads are coherent.
 *
 * Key encoding: read_key() returns 0 when nothing is pending, raw printable
 * ASCII (0x20..0x7E), or an LV_KEY_* code for nav/control keys -- the same
 * convention MeckCardKB::read_key() uses, so composer routing is unchanged.
 */

#pragma once

#include "arduino_compat.h"
#include "lvgl.h"
#include <vector>
#include <cstdint>
#include <cstring>

namespace Cpp_Bus_Driver { class Esp_At; }

class C6BleKeyboard {
public:
    C6BleKeyboard();

    enum class State : uint8_t {
        OFF = 0,        // disabled, BLE not initialised
        IDLE,           // enabled, BLE up, no keyboard, not browsing
        BROWSING,       // scanning to populate the picker list
        SCANNING,       // scanning to reconnect a known keyboard
        CONNECTING,     // link-layer connection in progress
        SECURING,       // pairing / encryption in progress
        DISCOVERING,    // GATT service/characteristic discovery
        SUBSCRIBING,    // enabling input-report notifications
        READY           // keyboard connected, reports flowing
    };

    struct ScanDev {
        char    addr[18];     // "aa:bb:cc:dd:ee:ff"
        char    name[24];     // advertised name, or "" if none
        int8_t  rssi;
        uint8_t addr_type;    // 0 = public, 1 = random (from +BLESCAN)
    };

    void begin(Cpp_Bus_Driver::Esp_At* at);

    // On/off. enable() inits the C6 as a BLE client; if a target address has
    // been set it reconnects to that keyboard, otherwise it sits IDLE waiting
    // for the picker. disable() disconnects and de-inits BLE.
    void enable();
    void disable();
    bool  isEnabled()   const { return _enabled; }
    bool  isConnected() const { return _state == State::READY; }
    State state()       const { return _state; }

    const char* pairedAddr() const { return _paired_addr; }

    // Reconnect target. Empty = none (IDLE until the picker connects one).
    // Call before enable() to auto-reconnect a previously paired keyboard.
    void setTargetAddr(const char* addr);

    // ---- Picker (Option B) ----
    // Begin/stop scanning to populate the discovered-keyboard list. Requires
    // the host to be enabled (BLE client initialised).
    void startBrowse();
    void stopBrowse();
    // Snapshot of what's been seen this browse. Safe to call from the UI task.
    int  scanCount() const { return _scan_count; }
    bool getScan(int i, ScanDev* out) const;
    // Connect to a chosen keyboard (also sets it as the reconnect target).
    void connectTo(const char* addr);

    // meck_task: drive SDIO + state machine + report decode.
    void poll();
    // UI task: pop one decoded key (0 = none).
    uint32_t read_key();

private:
    bool atCmd(const char* cmd, int timeout_ms = 2000);
    bool gattcWrite(int srv_index, int char_index, int desc_index,
                    const uint8_t* data, size_t len, int timeout_ms = 2000);

    bool pollSDIO();
    void drainLines();
    void processLine(const char* line, int len);

    bool startScanRaw();    // AT+BLESCAN=1
    bool stopScanRaw();     // AT+BLESCAN=0
    void onScanResult(const char* line);
    void connectToInternal(const char* addr);
    void beginDiscovery();
    void onPrimSrv(const char* line);
    void onChar(const char* line);
    void finishSubscribe();
    void onNotify(const char* line, int len);
    void resetLink();

    void decodeBootReport(const uint8_t* rpt, int len);
    void pushKey(uint32_t k);
    void addScanDev(const char* addr, const char* name, int8_t rssi, uint8_t addr_type);

    // ---- State ----
    Cpp_Bus_Driver::Esp_At* _at;
    bool  _begun;
    bool  _enabled;
    State _state;

    char  _target_addr[18];
    char  _paired_addr[18];
    int   _conn_index;

    // Event-driven connect sequence: processLine() sets these; poll() acts on
    // them at the top level so AT commands are never issued from inside the
    // parser (which would recurse atCmd -> drainLines -> processLine -> atCmd).
    bool  _want_connect;        // a connect was requested from an event handler
    char  _connect_addr[18];    // address for that deferred connect
    bool  _step_done;           // the AT command for the current step was sent
    uint8_t _conn_addr_type;    // address type to use for the next AT+BLECONN

    int _hid_srv_index;
    int _proto_char_index;
    int _input_char_index;
    int _input_cccd_index;

    // Probe: every Report characteristic (0x2A4D) that has a 0x2902 CCCD, so we
    // can subscribe them all and see which one notifies on a keypress.
    static constexpr int MAX_REPORTS = 8;
    struct ReportInput { int char_idx; int cccd_idx; };
    ReportInput _report_inputs[MAX_REPORTS];
    int  _report_input_count;
    bool _last_char_is_report;   // was the just-enumerated char a 0x2A4D?
    int  _last_report_char_idx;  // its char_index (to match its descriptors)

    unsigned long _state_deadline;
    unsigned long _rescan_at;

    uint8_t _prev_keys[6];

    // ---- SDIO receive line buffer ----
    static constexpr int RX_BUF_SIZE = 1024;
    char _rx_buf[RX_BUF_SIZE];
    int  _rx_len;

    volatile bool _got_ok;
    volatile bool _got_error;
    volatile bool _got_prompt;

    // ---- SPSC key ring (producer = poll/meck_task, consumer = read_key/UI) -
    static constexpr int KEY_FIFO = 32;
    volatile uint32_t _key_ring[KEY_FIFO];
    volatile uint16_t _key_head;
    volatile uint16_t _key_tail;

    // ---- Browse scan list (append-only by producer; count published last) --
    static constexpr int MAX_SCAN = 12;
    ScanDev          _scan[MAX_SCAN];
    volatile int     _scan_count;

    static constexpr uint16_t UUID_HID_SERVICE   = 0x1812;
    static constexpr uint16_t UUID_PROTOCOL_MODE = 0x2A4E;
    static constexpr uint16_t UUID_BOOT_KB_INPUT = 0x2A22;
    static constexpr uint16_t UUID_REPORT        = 0x2A4D;
    static constexpr uint16_t UUID_CCCD          = 0x2902;

    static constexpr int STEP_TIMEOUT_MS = 8000;
    static constexpr int RESCAN_DELAY_MS = 1500;
    static constexpr int PASSKEY_TIMEOUT_MS = 60000;  // time to read + type the passkey
};
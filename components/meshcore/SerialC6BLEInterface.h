/*
 * SerialC6BLEInterface.h -- BLE companion transport via ESP32-C6 AT over SDIO
 *
 * Implements BaseSerialInterface for the T-Display P4, where BLE is not
 * available on the main ESP32-P4 chip.  BLE is routed through an ESP32-C6
 * coprocessor running ESP-AT firmware, connected to the P4 via SDIO.
 *
 * Uses the stock Espressif SPP GATT service (UUID 0xA002) that ships in
 * the C6's ble_data partition:
 *
 *   C304 (Write)  -- phone writes companion frames here  (RX)
 *   C305 (Notify) -- device notifies companion frames here (TX)
 *
 * The MeshCore companion app must recognise these UUIDs alongside the
 * standard NUS UUIDs to connect.
 *
 * Threading: all public methods are called from meck_task only (the mesh
 * loop).  No mutex is needed; the SDIO bus is used exclusively from this
 * task while BLE is active.
 */

#pragma once

#include "arduino_compat.h"
#include <helpers/BaseSerialInterface.h>
#include <vector>
#include <cstdint>
#include <cstring>

// Forward declaration -- avoids pulling in the full LilyGo SDK headers.
namespace Cpp_Bus_Driver { class Esp_At; }

class SerialC6BLEInterface : public BaseSerialInterface {
public:
    SerialC6BLEInterface();

    /**
     * Bind to the C6 AT driver and set the BLE device name.
     * Must be called once before enable().
     *
     * @param at        pointer to the global Esp_At instance (ESP32C6_AT)
     * @param dev_name  advertised BLE device name (max 29 chars)
     */
    void begin(Cpp_Bus_Driver::Esp_At* at, const char* dev_name);

    // ---- BaseSerialInterface ----
    void enable() override;
    void disable() override;
    bool isEnabled() const override { return _enabled; }
    bool isConnected() const override { return _connected; }
    bool isWriteBusy() const override;
    size_t writeFrame(const uint8_t src[], size_t len) override;
    size_t checkRecvFrame(uint8_t dest[]) override;

    // Set the static BLE pairing PIN (6-digit). Call before enable().
    void setPin(uint32_t pin) { _ble_pin = pin; }

    // Probe C6 for OTA and WiFi capability. Sends diagnostic AT commands
    // and prints results to serial. Non-destructive, call after begin().
    void probeOTA();

private:
    // ---- AT command helpers ----

    // Send an AT command string and wait for OK / ERROR.
    // Returns true on OK, false on ERROR or timeout.
    bool atCmd(const char* cmd, int timeout_ms = 2000);

    // Send an AT command that returns a data prompt ('>') before payload.
    // After '>' is detected, sends `data` of `data_len` raw bytes, then
    // waits for the final OK/ERROR.
    bool atCmdWithData(const char* cmd, const uint8_t* data, size_t data_len,
                       int timeout_ms = 2000);

    // Poll SDIO for one chunk of data from the C6.  Appends to _rx_buf.
    // Returns true if new data was read.
    bool pollSDIO();

    // Consume complete lines from _rx_buf and dispatch to processLine().
    void drainLines();

    // Handle one complete line from the C6 (unsolicited or response).
    void processLine(const char* line, int len);

    // Build the hex-encoded scan response data (the device name).
    void buildScanRspHex(char* hex_out, size_t hex_out_sz);

    // ---- BLE advertising ----
    bool startAdvertising();

    // ---- State ----
    Cpp_Bus_Driver::Esp_At* _at;
    char _dev_name[30];
    bool _begun;
    bool _enabled;
    bool _connected;
    uint32_t _ble_pin;
    bool _probe_mode;
    unsigned long _adv_restart_time;
    unsigned long _last_write_ms;

    // ---- SDIO receive line buffer ----
    // AT responses and unsolicited notifications arrive as \r\n-delimited
    // text over SDIO.  We accumulate bytes here and extract complete lines.
    static constexpr int RX_BUF_SIZE = 1024;
    char _rx_buf[RX_BUF_SIZE];
    int  _rx_len;

    // ---- Frame queues (same shape as SerialBLEInterface) ----
    struct Frame {
        uint8_t len;
        uint8_t buf[MAX_FRAME_SIZE];
    };
    static constexpr int FRAME_QUEUE_SIZE = 8;

    Frame _recv_queue[FRAME_QUEUE_SIZE];
    int   _recv_queue_len;

    Frame _send_queue[FRAME_QUEUE_SIZE];
    int   _send_queue_len;

    void clearBuffers() { _recv_queue_len = 0; _send_queue_len = 0; }

    // ---- AT response state (used by atCmd / atCmdWithData) ----
    // These are set by processLine() during synchronous waits.
    volatile bool _got_ok;
    volatile bool _got_error;
    volatile bool _got_prompt;   // '>' for data-mode commands

    // ---- GATT characteristic indices (stock Espressif SPP table) ----
    // These are the srv_index / char_index values used in AT+BLEGATTSNTFY
    // and matched in +WRITE unsolicited messages.  Determined empirically
    // from the stock ble_data partition.
    static constexpr int SPP_SRV_INDEX = 1;   // service 0xA002
    static constexpr int SPP_TX_CHAR   = 6;   // C305 (Notify)
    static constexpr int SPP_RX_CHAR   = 5;   // C304 (Write)

    // ---- Timing ----
    static constexpr int WRITE_MIN_INTERVAL_MS = 15;
    static constexpr int ADV_RESTART_DELAY_MS  = 1000;
};
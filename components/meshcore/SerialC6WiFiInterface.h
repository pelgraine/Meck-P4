/*
 * SerialC6WiFiInterface.h -- WiFi companion transport via ESP32-C6 AT commands
 *
 * Implements BaseSerialInterface using the C6's TCP stack (AT+CIPSERVER)
 * so the MeshCore companion app can connect over WiFi. Uses the same
 * frame protocol as upstream SerialWifiInterface:
 *   App->Radio: '<' + length(2 bytes LE) + payload
 *   Radio->App: '>' + length(2 bytes LE) + payload
 */

#pragma once

#include "arduino_compat.h"
#include <meshcore_src/helpers/BaseSerialInterface.h>

namespace Cpp_Bus_Driver { class Esp_At; }

class SerialC6WiFiInterface : public BaseSerialInterface {
public:
    SerialC6WiFiInterface();

    void begin(Cpp_Bus_Driver::Esp_At* at);

    // Configure WiFi credentials. Call before enable().
    void setCredentials(const char* ssid, const char* password);

    // Set TCP server port (default 5555). Call before enable().
    void setPort(int port) { _port = port; }

    // BaseSerialInterface methods
    void enable() override;
    void disable() override;
    bool isEnabled() const override { return _enabled; }

    bool isConnected() const override { return _client_connected; }
    bool isWriteBusy() const override;
    size_t writeFrame(const uint8_t src[], size_t len) override;
    size_t checkRecvFrame(uint8_t dest[]) override;

    // Get the assigned IP address (valid after enable + WiFi connect)
    const char* getIP() const { return _ip_addr; }
    bool isWiFiConnected() const { return _wifi_connected; }

private:
    // AT command helpers
    bool atCmd(const char* cmd, int timeout_ms = 2000);
    bool pollSDIO();
    void drainLines();
    void processLine(const char* line, int len);
    void parseStreamFrames();

    bool connectWiFi();
    bool startServer();
    void fetchIP();

    // State
    Cpp_Bus_Driver::Esp_At* _at;
    bool _begun;
    bool _enabled;
    bool _wifi_connected;
    bool _client_connected;
    int  _client_id;
    int  _port;

    char _ssid[33];
    char _password[65];
    char _ip_addr[20];

    // AT response flags
    volatile bool _got_ok;
    volatile bool _got_error;
    volatile bool _got_prompt;

    // SDIO read buffer (raw bytes from C6)
    static constexpr int RX_BUF_SIZE = 2048;
    char _rx_buf[RX_BUF_SIZE];
    int  _rx_len;

    // Byte-level parser state for +IPD handling.
    // +IPD data is binary (not newline-terminated), so we parse
    // byte-by-byte with an explicit state machine.
    enum ParseState { PS_LINE, PS_IPD_DATA };
    ParseState _parse_state;

    // Line accumulation buffer (for non-IPD AT responses)
    char _line_buf[512];
    int  _line_len;

    // +IPD binary accumulation
    int  _ipd_remain;

    // Stream buffer: accumulates raw TCP payload (the WiFi framing
    // protocol: '<' + len_lo + len_hi + companion_frame)
    uint8_t _stream_buf[MAX_FRAME_SIZE + 8];
    int     _stream_len;

    // Frame queues
    struct Frame {
        uint8_t len;
        uint8_t buf[MAX_FRAME_SIZE];
    };
    static constexpr int FRAME_QUEUE_SIZE = 16;
    int _recv_queue_len;
    Frame _recv_queue[FRAME_QUEUE_SIZE];
    int _send_queue_len;
    Frame _send_queue[FRAME_QUEUE_SIZE];

    void clearBuffers() {
        _recv_queue_len = 0;
        _send_queue_len = 0;
        _stream_len = 0;
        _ipd_remain = 0;
        _parse_state = PS_LINE;
        _line_len = 0;
    }

    static constexpr int WRITE_MIN_INTERVAL_MS = 2;
    unsigned long _last_write_ms;
};
/*
 * SerialC6WiFiInterface.cpp -- WiFi companion transport via ESP32-C6 AT cmds
 *
 * The C6 coprocessor runs Espressif AT firmware and communicates with the
 * P4 host over SDIO.  This class drives the C6's WiFi + TCP stack via AT
 * commands to present a BaseSerialInterface to MeckCompanion.
 *
 * WiFi framing protocol (same as upstream SerialWifiInterface):
 *   App->Radio: '<' (0x3C) + length (2 bytes LE) + payload
 *   Radio->App: '>' (0x3E) + length (2 bytes LE) + payload
 */

#include "SerialC6WiFiInterface.h"
#include <arduino_cpp_bus_driver_library.h>
#include <cstring>
#include <cstdlib>
#include <vector>

#define C6WIFI_DEBUG 1
#if C6WIFI_DEBUG
  #define C6WIFI_LOG(fmt, ...) printf("C6WiFi: " fmt "\n", ##__VA_ARGS__)
#else
  #define C6WIFI_LOG(...) do {} while(0)
#endif

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

SerialC6WiFiInterface::SerialC6WiFiInterface()
    : _at(nullptr), _begun(false), _enabled(false),
      _wifi_connected(false), _client_connected(false),
      _client_id(-1), _port(5000),
      _got_ok(false), _got_error(false), _got_prompt(false),
      _rx_len(0), _parse_state(PS_LINE), _line_len(0),
      _ipd_remain(0), _stream_len(0),
      _recv_queue_len(0), _send_queue_len(0),
      _last_write_ms(0)
{
    _ssid[0] = '\0';
    _password[0] = '\0';
    _ip_addr[0] = '\0';
}

// ---------------------------------------------------------------------------
// begin / setCredentials
// ---------------------------------------------------------------------------

void SerialC6WiFiInterface::begin(Cpp_Bus_Driver::Esp_At* at) {
    _at = at;
    _begun = (at != nullptr);
    C6WIFI_LOG("begin: at=%p", at);
}

void SerialC6WiFiInterface::setCredentials(const char* ssid, const char* password) {
    strncpy(_ssid, ssid, sizeof(_ssid) - 1);
    _ssid[sizeof(_ssid) - 1] = '\0';
    strncpy(_password, password, sizeof(_password) - 1);
    _password[sizeof(_password) - 1] = '\0';
}

// ---------------------------------------------------------------------------
// AT command infrastructure (mirrors SerialC6BLEInterface)
// ---------------------------------------------------------------------------

bool SerialC6WiFiInterface::pollSDIO() {
    if (!_at) return false;

    uint32_t flag = _at->get_irq_flag();
    if (!_at->assert_rx_new_packet_flag(flag)) return false;
    _at->clear_irq_flag(flag);

    std::vector<uint8_t> buf;
    if (!_at->receive_packet(buf)) return false;
    if (buf.empty()) return false;

    int space = RX_BUF_SIZE - _rx_len;
    int copy = (int)buf.size() < space ? (int)buf.size() : space;
    if (copy > 0) {
        memcpy(_rx_buf + _rx_len, buf.data(), copy);
        _rx_len += copy;
    }
    return true;
}

bool SerialC6WiFiInterface::atCmd(const char* cmd, int timeout_ms) {
    if (!_at) return false;

    _got_ok = false;
    _got_error = false;
    _got_prompt = false;

    char buf[256];
    int n = snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    _at->send_packet(buf, n);
    C6WIFI_LOG(">> %s", cmd);

    // Longer commands (CWJAP) can take 10-15 seconds. Yield generously
    // to keep IDLE0 alive and prevent the task watchdog from firing.
    int delay_ms = (timeout_ms > 5000) ? 100 : 10;

    unsigned long t0 = millis();
    while ((long)(millis() - t0) < timeout_ms) {
        pollSDIO();
        drainLines();
        if (_got_ok)    return true;
        if (_got_error) return false;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    C6WIFI_LOG("AT timeout: %s", cmd);
    return false;
}

// ---------------------------------------------------------------------------
// Byte-level SDIO parser
//
// +IPD data is binary (not newline-terminated), so we process the SDIO
// buffer byte-by-byte with two modes:
//   PS_LINE     -- accumulate text until \n, then processLine()
//   PS_IPD_DATA -- copy exactly _ipd_remain binary bytes to _stream_buf
// ---------------------------------------------------------------------------

void SerialC6WiFiInterface::drainLines() {
    for (int i = 0; i < _rx_len; i++) {
        uint8_t c = (uint8_t)_rx_buf[i];

        // ---- Mode: binary IPD data ----
        if (_parse_state == PS_IPD_DATA) {
            if (_stream_len < (int)sizeof(_stream_buf)) {
                _stream_buf[_stream_len++] = c;
            }
            if (--_ipd_remain <= 0) {
                _parse_state = PS_LINE;
                parseStreamFrames();
            }
            continue;
        }

        // ---- Mode: line accumulation ----
        if (c == '\n') {
            _line_buf[_line_len] = '\0';
            if (_line_len > 0 && _line_buf[_line_len - 1] == '\r') {
                _line_buf[--_line_len] = '\0';
            }
            if (_line_len > 0) {
                processLine(_line_buf, _line_len);
            }
            _line_len = 0;
            continue;
        }

        // Detect '>' prompt (AT+CIPSEND data-mode). The prompt may arrive
        // without a trailing newline, so check as soon as we see it at the
        // start of a fresh line.
        if (c == '>' && _line_len == 0) {
            _got_prompt = true;
            continue;
        }

        if (_line_len < (int)sizeof(_line_buf) - 1) {
            _line_buf[_line_len++] = (char)c;
        }

        // Detect completed +IPD header: "+IPD,<id>,<len>:"
        // The colon terminates the header; bytes after it are binary data.
        if (c == ':' && _line_len >= 7 && strncmp(_line_buf, "+IPD,", 5) == 0) {
            _line_buf[_line_len] = '\0';
            const char* p = _line_buf + 5;
            _client_id = strtol(p, (char**)&p, 10);
            if (*p == ',') p++;
            int data_len = strtol(p, (char**)&p, 10);

            C6WIFI_LOG("+IPD: client=%d len=%d", _client_id, data_len);

            _ipd_remain = data_len;
            _parse_state = PS_IPD_DATA;
            _line_len = 0;

            if (!_client_connected) {
                _client_connected = true;
                C6WIFI_LOG("client connected (via +IPD)");
            }
        }
    }
    _rx_len = 0;
}

// ---------------------------------------------------------------------------
// processLine -- handle AT responses and unsolicited events
// ---------------------------------------------------------------------------

void SerialC6WiFiInterface::processLine(const char* line, int len) {
    // AT response codes
    if (strcmp(line, "OK") == 0) {
        _got_ok = true;
        return;
    }
    if (strcmp(line, "ERROR") == 0 || strncmp(line, "ERR CODE:", 9) == 0) {
        _got_error = true;
        C6WIFI_LOG("AT error: %s", line);
        return;
    }
    if (strcmp(line, ">") == 0) {
        _got_prompt = true;
        return;
    }
    if (strcmp(line, "SEND OK") == 0) {
        return;
    }
    if (strcmp(line, "SEND FAIL") == 0) {
        C6WIFI_LOG("SEND FAIL");
        return;
    }

    // WiFi state events
    if (strcmp(line, "WIFI CONNECTED") == 0) {
        C6WIFI_LOG("WiFi associated");
        return;
    }
    if (strcmp(line, "WIFI GOT IP") == 0) {
        _wifi_connected = true;
        C6WIFI_LOG("WiFi got IP");
        return;
    }
    if (strcmp(line, "WIFI DISCONNECT") == 0) {
        _wifi_connected = false;
        _client_connected = false;
        _ip_addr[0] = '\0';
        C6WIFI_LOG("WiFi disconnected");
        return;
    }

    // TCP connection events: "<link_id>,CONNECT" / "<link_id>,CLOSED"
    if (len >= 9) {
        // e.g. "0,CONNECT"
        if (len >= 2 && line[1] == ',') {
            int id = line[0] - '0';
            const char* evt = line + 2;
            if (strcmp(evt, "CONNECT") == 0) {
                _client_connected = true;
                _client_id = id;
                _stream_len = 0;  // reset stream for new connection
                C6WIFI_LOG("TCP client %d connected", id);
                return;
            }
            if (strcmp(evt, "CLOSED") == 0) {
                if (id == _client_id) {
                    _client_connected = false;
                    _client_id = -1;
                    _stream_len = 0;
                    C6WIFI_LOG("TCP client %d disconnected", id);
                }
                return;
            }
        }
    }

    // +CIPSTA / +CIFSR responses (IP address)
    // +CIFSR:STAIP,"192.168.1.100"
    if (strncmp(line, "+CIFSR:STAIP,\"", 14) == 0) {
        const char* ip_start = line + 14;
        const char* ip_end = strchr(ip_start, '"');
        if (ip_end) {
            int ip_len = ip_end - ip_start;
            if (ip_len < (int)sizeof(_ip_addr)) {
                memcpy(_ip_addr, ip_start, ip_len);
                _ip_addr[ip_len] = '\0';
                C6WIFI_LOG("IP: %s", _ip_addr);
            }
        }
        return;
    }

    // Log anything else
    if (len > 0) {
        C6WIFI_LOG("[C6] %s", line);
    }
}

// ---------------------------------------------------------------------------
// parseStreamFrames -- extract companion frames from the TCP stream buffer
//
// Framing: '<' + len_lo + len_hi + payload (app-to-radio)
// Multiple frames may be concatenated in _stream_buf.
// ---------------------------------------------------------------------------

void SerialC6WiFiInterface::parseStreamFrames() {
    int pos = 0;
    while (pos < _stream_len) {
        // Need at least 3 bytes for a frame header
        if (_stream_len - pos < 3) break;

        uint8_t type = _stream_buf[pos];
        uint16_t frame_len = _stream_buf[pos + 1] | (_stream_buf[pos + 2] << 8);

        // Validate
        if (type != '<') {
            C6WIFI_LOG("parseStream: unexpected type 0x%02x, discarding", type);
            pos++;
            continue;
        }
        if (frame_len > MAX_FRAME_SIZE) {
            C6WIFI_LOG("parseStream: frame too big (%d), skipping", frame_len);
            pos += 3 + frame_len;
            continue;
        }

        // Need full payload
        if (_stream_len - pos < 3 + (int)frame_len) break;

        // Enqueue the frame
        if (_recv_queue_len < FRAME_QUEUE_SIZE) {
            Frame& f = _recv_queue[_recv_queue_len];
            f.len = (uint8_t)frame_len;
            memcpy(f.buf, _stream_buf + pos + 3, frame_len);
            _recv_queue_len++;
            C6WIFI_LOG("recv frame: %d bytes", frame_len);
        } else {
            C6WIFI_LOG("recv queue full, dropping frame");
        }

        pos += 3 + frame_len;
    }

    // Shift any remaining partial data to front
    if (pos > 0 && pos < _stream_len) {
        memmove(_stream_buf, _stream_buf + pos, _stream_len - pos);
        _stream_len -= pos;
    } else if (pos >= _stream_len) {
        _stream_len = 0;
    }
}

// ---------------------------------------------------------------------------
// WiFi + server setup
// ---------------------------------------------------------------------------

bool SerialC6WiFiInterface::connectWiFi() {
    if (_ssid[0] == '\0') {
        C6WIFI_LOG("no SSID configured");
        return false;
    }

    // Station mode
    if (!atCmd("AT+CWMODE=1", 2000)) return false;
    // Brief settle — the C6 sometimes fires async WiFi events (from a
    // previously saved AP) immediately after CWMODE, which can cause
    // "busy p..." if CWJAP arrives while those are still in flight.
    vTaskDelay(pdMS_TO_TICKS(200));

    // Join AP (can take 5-10 seconds)
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", _ssid, _password);
    if (!atCmd(cmd, 15000)) {
        C6WIFI_LOG("CWJAP failed");
        return false;
    }

    _wifi_connected = true;
    return true;
}

void SerialC6WiFiInterface::fetchIP() {
    _ip_addr[0] = '\0';
    // The C6 may still be flushing async WiFi events (WIFI CONNECTED,
    // WIFI GOT IP) after CWJAP succeeds. Give it a moment to settle,
    // then retry CIFSR a few times if the IP comes back empty.
    for (int attempt = 0; attempt < 3; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(500));
        atCmd("AT+CIFSR", 3000);
        if (_ip_addr[0] != '\0') return;
        C6WIFI_LOG("CIFSR: IP empty, retry %d/3", attempt + 1);
    }
}

bool SerialC6WiFiInterface::startServer() {
    // Enable multiple connections (required for CIPSERVER)
    if (!atCmd("AT+CIPMUX=1", 2000)) return false;

    // Start TCP server
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CIPSERVER=1,%d", _port);
    if (!atCmd(cmd, 3000)) return false;

    C6WIFI_LOG("TCP server on port %d", _port);
    return true;
}

// ---------------------------------------------------------------------------
// enable / disable
// ---------------------------------------------------------------------------

void SerialC6WiFiInterface::enable() {
    if (_enabled) return;
    if (!_begun || !_at) return;

    clearBuffers();
    _client_connected = false;
    _client_id = -1;
    _ip_addr[0] = '\0';

    if (!connectWiFi()) {
        C6WIFI_LOG("enable FAILED: WiFi connect failed");
        return;
    }

    fetchIP();

    if (!startServer()) {
        C6WIFI_LOG("enable FAILED: server start failed");
        return;
    }

    _enabled = true;
    C6WIFI_LOG("enabled (IP: %s, port %d)", _ip_addr, _port);
}

void SerialC6WiFiInterface::disable() {
    if (!_enabled) return;
    _enabled = false;

    // Close client connection if any
    if (_client_connected && _client_id >= 0) {
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%d", _client_id);
        atCmd(cmd, 1000);
    }

    // Stop server
    atCmd("AT+CIPSERVER=0", 1000);

    // Disconnect WiFi
    atCmd("AT+CWQAP", 2000);

    _client_connected = false;
    _client_id = -1;
    _wifi_connected = false;
    _ip_addr[0] = '\0';
    clearBuffers();
    C6WIFI_LOG("disabled");
}

// ---------------------------------------------------------------------------
// BaseSerialInterface: writeFrame / isWriteBusy / checkRecvFrame
// ---------------------------------------------------------------------------

bool SerialC6WiFiInterface::isWriteBusy() const {
    return (long)(millis() - _last_write_ms) < WRITE_MIN_INTERVAL_MS;
}

size_t SerialC6WiFiInterface::writeFrame(const uint8_t src[], size_t len) {
    if (len == 0 || len > MAX_FRAME_SIZE) return 0;
    if (!_client_connected) return 0;

    if (_send_queue_len >= FRAME_QUEUE_SIZE) {
        C6WIFI_LOG("writeFrame: send queue full");
        return 0;
    }

    Frame& f = _send_queue[_send_queue_len];
    f.len = (uint8_t)len;
    memcpy(f.buf, src, len);
    _send_queue_len++;
    return len;
}

size_t SerialC6WiFiInterface::checkRecvFrame(uint8_t dest[]) {
    if (!_enabled || !_at) return 0;

    // Poll SDIO and process any data
    pollSDIO();
    drainLines();

    // Send queued outgoing frames — up to 4 per cycle to keep sync fast.
    // We don't block waiting for SEND OK; it's caught by drainLines()
    // on the next poll cycle.
    int sends = 0;
    while (_client_connected && _client_id >= 0 && _send_queue_len > 0 && sends < 2) {
        Frame& f = _send_queue[0];
        int total = 3 + f.len;   // '>' + len_lo + len_hi + payload

        // Build the framed packet
        uint8_t pkt[3 + MAX_FRAME_SIZE];
        pkt[0] = '>';
        pkt[1] = (f.len & 0xFF);
        pkt[2] = (f.len >> 8);
        memcpy(pkt + 3, f.buf, f.len);

        // AT+CIPSEND=<link_id>,<len>
        char cmd[48];
        int cmd_n = snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d,%d\r\n", _client_id, total);

        _got_prompt = false;
        _got_error = false;
        _at->send_packet(cmd, cmd_n);

        // Wait for '>' prompt. The C6's SDIO TX buffer can overflow during
        // burst sends (e.g. 610-contact export), causing send_packet to
        // silently fail. Retry a few times with increasing backoff before
        // giving up.
        bool got_it = false;
        for (int retry = 0; retry < 3 && !got_it; retry++) {
            if (retry > 0) {
                // Backoff: give the C6 time to flush its TCP buffers
                vTaskDelay(pdMS_TO_TICKS(50 * retry));
                _got_prompt = false;
                _got_error = false;
                _at->send_packet(cmd, cmd_n);
            }
            unsigned long t0 = millis();
            while ((long)(millis() - t0) < 500) {
                pollSDIO();
                drainLines();
                if (_got_prompt) { got_it = true; break; }
                if (_got_error)  break;
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }

        if (got_it) {
            _at->send_packet((const char*)pkt, total);
            _last_write_ms = millis();
            // Give the C6 time to flush TCP before the next CIPSEND
            vTaskDelay(pdMS_TO_TICKS(5));
            pollSDIO();
            drainLines();
        } else {
            C6WIFI_LOG("CIPSEND: failed, client likely disconnected");
            _client_connected = false;
            _client_id = -1;
            _send_queue_len = 0;  // flush remaining sends
            _stream_len = 0;
            break;
        }

        // Shift send queue
        _send_queue_len--;
        for (int i = 0; i < _send_queue_len; i++) {
            _send_queue[i] = _send_queue[i + 1];
        }
        sends++;
    }

    // Return a received frame if available
    if (_recv_queue_len > 0) {
        Frame& f = _recv_queue[0];
        int n = f.len;
        memcpy(dest, f.buf, n);

        _recv_queue_len--;
        for (int i = 0; i < _recv_queue_len; i++) {
            _recv_queue[i] = _recv_queue[i + 1];
        }
        return n;
    }

    return 0;
}
/*
 * MeckWebFetch.cpp -- plain-HTTP page fetch over the ESP32-C6 (Stage 1)
 *
 * See MeckWebFetch.h for the overview. This file is the only place the C6
 * bus-driver header is pulled in, so its macros never reach meck_app.cpp.
 *
 * The AT mechanics (pollSDIO / atCmd / byte parser) mirror the proven ones in
 * SerialC6WiFiInterface.cpp. The C6 runs in CIPMUX=1 with one client link, and
 * the response is read in passive mode (CIPRECVMODE=1 + CIPRECVDATA) so each
 * SDIO read stays small.
 */

#include "MeckWebFetch.h"
#include <arduino_cpp_bus_driver_library.h>
#include "arduino_compat.h"          // millis()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include "MeckWebHtml.h"             // lifted HTML reader-mode parser (Stage 2)
// Routes printf below into the SD debug log when it is running. Included
// last so its printf macro does not affect the headers above.
#include "meck_log.h"

// Stage 1 capture cap. Enough to hold a small plain-http test page in full;
// anything larger is consumed but not stored.
static constexpr int WEBFETCH_RESP_CAP = 32768;

// ---------------------------------------------------------------------------
// URL parsing (plain http only)
// ---------------------------------------------------------------------------

// Splits "http://host[:port][/path]" into host, port, path. Returns false on
// an https url (not handled in Stage 1) or if no host could be parsed.
static bool parse_http_url(const char* url, char* host, int host_sz,
                           int* port, char* path, int path_sz, bool* is_https) {
    *port = 80;
    *is_https = false;
    const char* p = url;
    if (strncmp(p, "https://", 8) == 0) {
        *is_https = true;
        *port = 443;
        p += 8;
    } else if (strncmp(p, "http://", 7) == 0) {
        p += 7;
    }
    int hi = 0;
    while (*p && *p != ':' && *p != '/') {
        if (hi < host_sz - 1) host[hi++] = *p;
        p++;
    }
    host[hi] = '\0';
    if (*p == ':') {
        p++;
        *port = atoi(p);
        while (*p && *p != '/') p++;
    }
    if (*p == '/') {
        strncpy(path, p, path_sz - 1);
        path[path_sz - 1] = '\0';
    } else {
        path[0] = '/';
        path[1] = '\0';
    }
    return hi > 0;
}

// ---------------------------------------------------------------------------
// C6 AT plumbing
// ---------------------------------------------------------------------------

bool MeckWebFetch::poll() {
    if (!_at) return false;

    uint32_t flag = _at->get_irq_flag();
    if (!_at->assert_rx_new_packet_flag(flag)) return false;
    _at->clear_irq_flag(flag);

    std::vector<uint8_t> buf;
    if (!_at->receive_packet(buf)) return false;
    if (buf.empty()) return false;

    // Copy the whole packet, draining in place if it is larger than _rx so no
    // bytes are ever dropped. Dropping mid-stream desyncs the +IPD framing (the
    // cause of the stray "+IPD,...:" header and "CLOSED" captured into the body,
    // the short capture, and the read timeout in Stage 1's first device run).
    size_t off = 0;
    while (off < buf.size()) {
        int space = (int)sizeof(_rx) - _rx_len;
        if (space <= 0) {
            drain();                          // frees _rx; parser state carries over
            space = (int)sizeof(_rx) - _rx_len;
        }
        int avail = (int)(buf.size() - off);
        int copy = avail < space ? avail : space;
        memcpy(_rx + _rx_len, buf.data() + off, copy);
        _rx_len += copy;
        off += (size_t)copy;
    }
    return true;
}

void MeckWebFetch::onLine(const char* line, int len) {
    if (strcmp(line, "OK") == 0)                              { _got_ok = true;    return; }
    if (strcmp(line, "ERROR") == 0 ||
        strncmp(line, "ERR CODE:", 9) == 0)                  { _got_error = true; return; }
    if (strcmp(line, ">") == 0)                              { _got_prompt = true; return; }
    if (strcmp(line, "SEND OK") == 0)                        { return; }
    if (strcmp(line, "SEND FAIL") == 0)                      { return; }
    // In single-connection mode the close event is a bare "CLOSED"; tolerate
    // a "<id>,CLOSED" form too in case CIPMUX state lingered.
    if (strcmp(line, "CLOSED") == 0 || strstr(line, ",CLOSED")) { _closed = true; return; }
    // Passive-mode data-available notice: "+IPD,<id>,<len>" with no payload
    // (the bytes are pulled separately via CIPRECVDATA).
    if (strncmp(line, "+IPD,", 5) == 0) { _recv_pending = true; return; }
    // Resolved IP from AT+CIPDOMAIN, quoted ("1.2.3.4") or bare.
    if (strncmp(line, "+CIPDOMAIN:", 11) == 0) {
        const char* p = line + 11;
        if (*p == '"') p++;
        int k = 0;
        while (*p && *p != '"' && k < (int)sizeof(_resolved_ip) - 1) {
            _resolved_ip[k++] = *p++;
        }
        _resolved_ip[k] = '\0';
        return;
    }
    // WIFI CONNECTED / WIFI GOT IP / WIFI DISCONNECT and any other lines are
    // not needed here.
    (void)len;
}

void MeckWebFetch::drain() {
    for (int i = 0; i < _rx_len; i++) {
        uint8_t c = (uint8_t)_rx[i];

        // ---- Binary +IPD payload ----
        if (_state == PS_IPD) {
            if (_resp && _resp_len < _resp_cap) {
                _resp[_resp_len++] = (char)c;
            }
            if (--_ipd_remain <= 0) {
                _state = PS_LINE;
            }
            continue;
        }

        // ---- Text line accumulation ----
        if (c == '\n') {
            _line[_line_len] = '\0';
            if (_line_len > 0 && _line[_line_len - 1] == '\r') {
                _line[--_line_len] = '\0';
            }
            if (_line_len > 0) onLine(_line, _line_len);
            _line_len = 0;
            continue;
        }

        // '>' data-mode prompt can arrive without a trailing newline.
        if (c == '>' && _line_len == 0) {
            _got_prompt = true;
            continue;
        }

        if (_line_len < (int)sizeof(_line) - 1) {
            _line[_line_len++] = (char)c;
        }

        // Completed binary-data header; switch to capturing <len> bytes.
        // Passive pull replies are "+CIPRECVDATA:<len>,<data>" -- colon after
        // the keyword, comma after the length, so the header ends at that
        // comma. Active pushes are "+IPD,...,<len>:<data>" and end at the colon
        // (kept for safety).
        if (c == ',' && _line_len >= 14 && strncmp(_line, "+CIPRECVDATA:", 13) == 0) {
            int data_len = (int)strtol(_line + 13, nullptr, 10);
            if (data_len >= 0) {
                _ipd_remain = data_len;
                _state = (data_len > 0) ? PS_IPD : PS_LINE;
                _line_len = 0;
            }
        } else if (c == ':' && _line_len >= 6 && strncmp(_line, "+IPD,", 5) == 0) {
            const char* p = _line + 5;
            char* end = nullptr;
            long first = strtol(p, &end, 10);
            int data_len = (end && *end == ',') ? (int)strtol(end + 1, nullptr, 10)
                                                : (int)first;
            _ipd_remain = data_len;
            _state = (data_len > 0) ? PS_IPD : PS_LINE;
            _line_len = 0;
        }
    }
    _rx_len = 0;
}

bool MeckWebFetch::cmd(const char* c, int timeout_ms) {
    if (!_at) return false;

    _got_ok = false;
    _got_error = false;
    _got_prompt = false;

    char buf[256];
    int n = snprintf(buf, sizeof(buf), "%s\r\n", c);
    _at->send_packet(buf, n);
    if (strncmp(c, "AT+CWJAP=", 9) == 0) {
        printf("WebFetch: >> AT+CWJAP=...\n");   // mask credentials
    } else {
        printf("WebFetch: >> %s\n", c);
    }

    // Long commands (CWJAP) need generous yielding to keep IDLE alive.
    int delay_ms = (timeout_ms > 5000) ? 100 : 10;

    unsigned long t0 = millis();
    while ((long)(millis() - t0) < timeout_ms) {
        poll();
        drain();
        if (_got_ok)    return true;
        if (_got_error) return false;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    printf("WebFetch: timeout on %s\n", c);
    return false;
}

void MeckWebFetch::teardown() {
    // Both are best-effort; an ERROR here (e.g. already closed) is harmless.
    cmd("AT+CIPCLOSE=0", 2000);
    cmd("AT+CWQAP", 3000);
    if (_resp) {
        heap_caps_free(_resp);
        _resp = nullptr;
    }
}

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

bool MeckWebFetch::fetch(Cpp_Bus_Driver::Esp_At* at,
                         const char* ssid, const char* password, const char* url,
                         char* textOut, int textCap,
                         WebLink* links, int maxLinks,
                         WebForm* forms, int maxForms,
                         ParseResult* out) {
    _at = at;
    _rx_len = 0;
    _line_len = 0;
    _got_ok = _got_error = _got_prompt = false;
    _closed = false;
    _resolved_ip[0] = '\0';
    _recv_pending = false;
    _state = PS_LINE;
    _ipd_remain = 0;
    _resp = nullptr;
    _resp_len = 0;
    _resp_cap = 0;

    if (!_at) {
        printf("WebFetch: no C6 AT driver\n");
        return false;
    }
    if (!ssid || ssid[0] == '\0') {
        printf("WebFetch: no WiFi SSID configured\n");
        return false;
    }

    char host[128];
    char path[256];
    int  port = 80;
    bool is_https = false;
    if (!parse_http_url(url, host, sizeof(host), &port, path, sizeof(path), &is_https)) {
        printf("WebFetch: unparseable url: %s\n", url);
        return false;
    }
    printf("WebFetch: fetching %s  (host=%s port=%d path=%s)\n", url, host, port, path);

    _resp_cap = WEBFETCH_RESP_CAP;
    _resp = (char*)heap_caps_malloc(_resp_cap, MALLOC_CAP_SPIRAM);
    if (!_resp) {
        printf("WebFetch: PSRAM alloc failed (%d bytes)\n", _resp_cap);
        _resp_cap = 0;
        return false;
    }

    // 1. Station mode.
    if (!cmd("AT+CWMODE=1", 2000)) { printf("WebFetch: CWMODE failed\n"); teardown(); return false; }
    vTaskDelay(pdMS_TO_TICKS(200));   // let any stale async WiFi events settle

    // 2. Join AP.
    char join[160];
    snprintf(join, sizeof(join), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    printf("WebFetch: joining \"%s\"...\n", ssid);
    if (!cmd(join, 20000)) { printf("WebFetch: CWJAP failed\n"); teardown(); return false; }

    // 2a. Let DHCP/DNS settle before any network op. The companion's enable()
    //     does the same via fetchIP()'s CIFSR poll; without this settle a
    //     CIPSTART issued the instant CWJAP returns gets rejected outright
    //     (the failure seen in Stage 1's first device run).
    for (int i = 0; i < 3; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
        cmd("AT+CIFSR", 3000);
    }

    // 3. Multiple-connection mode with an explicit link id, matching how the
    //    companion drives the C6 (it never switches CIPMUX). Switching to
    //    single-connection mode was the one thing this fetch did that the
    //    proven companion path never does, and CIPSTART failed right after it;
    //    staying in CIPMUX=1 removes that variable.
    if (!cmd("AT+CIPMUX=1", 2000)) { printf("WebFetch: CIPMUX failed\n"); teardown(); return false; }

    // Passive receive: the C6 buffers incoming TCP data and notifies us rather
    // than pushing it, so we can pull it in small chunks (step 7). A large
    // active-mode push was failing the bus driver's block read on the response.
    if (!cmd("AT+CIPRECVMODE=1", 2000)) { printf("WebFetch: CIPRECVMODE failed\n"); teardown(); return false; }

    // 3a. Resolve the host explicitly. CIPSTART by hostname returns a bare
    //     ERROR from a cold-booted C6 (its resolver isn't primed yet), so we
    //     resolve via CIPDOMAIN and connect to the IP, keeping the hostname
    //     only in the HTTP Host header. If resolution fails we fall back to a
    //     hostname connect so the C6's error stays visible.
    char dom[160];
    snprintf(dom, sizeof(dom), "AT+CIPDOMAIN=\"%s\"", host);
    for (int i = 0; i < 3 && _resolved_ip[0] == '\0'; i++) {
        if (i > 0) vTaskDelay(pdMS_TO_TICKS(400));
        cmd(dom, 8000);
    }
    const char* connect_host = (_resolved_ip[0] != '\0') ? _resolved_ip : host;
    if (_resolved_ip[0] != '\0') {
        printf("WebFetch: resolved %s -> %s\n", host, _resolved_ip);
    } else {
        printf("WebFetch: DNS resolve failed; trying hostname connect\n");
    }

    // 4. For HTTPS, configure the TLS client first: auth_mode 0 (no server
    //    certificate verification -- no CA bundle on the C6 yet) and SNI set to
    //    the hostname so the server serves the right cert. Then open with the
    //    "SSL" socket type by hostname (SNI / cert hostname match) rather than
    //    "TCP" by IP. The TLS handshake is slow, so SSL gets a longer ceiling.
    if (is_https) {
        cmd("AT+CIPSSLCCONF=0,0", 2000);
        char sni[160];
        snprintf(sni, sizeof(sni), "AT+CIPSSLCSNI=0,\"%s\"", host);
        cmd(sni, 2000);
    }
    char start[200];
    snprintf(start, sizeof(start), "AT+CIPSTART=0,\"%s\",\"%s\",%d",
             is_https ? "SSL" : "TCP",
             is_https ? host : connect_host,
             port);
    bool connected = false;
    int start_timeout = is_https ? 15000 : 8000;
    for (int attempt = 0; attempt < 3 && !connected; attempt++) {
        if (attempt > 0) {
            printf("WebFetch: CIPSTART retry %d/3\n", attempt);
            vTaskDelay(pdMS_TO_TICKS(400));
        }
        connected = cmd(start, start_timeout);
    }
    if (!connected) { printf("WebFetch: CIPSTART failed\n"); teardown(); return false; }

    // 5. Build the HTTP/1.0 request (Connection: close so the server closes
    //    when done, which is our read-loop terminator).
    char req[512];
    int reqn = snprintf(req, sizeof(req),
        "GET %s HTTP/1.0\r\n"
        "Host: %s\r\n"
        "User-Agent: Meck-P4\r\n"
        "Connection: close\r\n"
        "\r\n",
        path, host);

    // 6. CIPSEND: announce the length, wait for the '>' prompt, send bytes.
    _got_prompt = false;
    _got_error = false;
    char cs[40];
    int csn = snprintf(cs, sizeof(cs), "AT+CIPSEND=0,%d\r\n", reqn);
    _at->send_packet(cs, csn);
    printf("WebFetch: >> AT+CIPSEND=0,%d\n", reqn);

    bool prompt = false;
    unsigned long tp = millis();
    while ((long)(millis() - tp) < 3000) {
        poll();
        drain();
        if (_got_prompt) { prompt = true; break; }
        if (_got_error)  break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (!prompt) { printf("WebFetch: no CIPSEND '>' prompt\n"); teardown(); return false; }

    _at->send_packet(req, reqn);

    // 7. Passive receive. The C6 buffers incoming data and notifies us with
    //    "+IPD,0,<len>" (no payload); we pull it in small CIPRECVDATA chunks,
    //    then drain any remainder after the connection closes.
    //
    //    CHUNK is kept small on purpose: each CIPRECVDATA reply (header + data
    //    + OK) must stay under the bus driver's 512-byte block threshold
    //    (ESP_AT_MAX_TRANSMIT_BLOCK_BUFFER_SIZE). At or above it, receive_packet
    //    takes a read_block() path that fails on this hardware; below it only
    //    the working read() path runs (the same one the companion's small
    //    frames use). 256 leaves headroom for the ~24 bytes of framing.
    const int CHUNK = 256;
    char recvcmd[32];
    snprintf(recvcmd, sizeof(recvcmd), "AT+CIPRECVDATA=0,%d", CHUNK);

    unsigned long t0 = millis();
    unsigned long last_rx = t0;
    while ((long)(millis() - t0) < 45000) {
        poll();
        drain();   // sets _recv_pending on notify, _closed on close, captures data

        if (_recv_pending) {
            int before = _resp_len;
            cmd(recvcmd, 3000);            // +CIPRECVDATA payload captured by drain
            if (_resp_len > before) {
                last_rx = millis();        // more may remain; keep pulling
            } else {
                _recv_pending = false;     // buffer drained for now
            }
            continue;
        }

        if (_closed) {
            // Drain anything still buffered after the close notification.
            int before = _resp_len;
            cmd(recvcmd, 3000);
            if (_resp_len == before) break;
            last_rx = millis();
            continue;
        }

        if ((long)(millis() - last_rx) > 3000) break;   // quiet-gap fallback
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // 8. Report and dump.
    printf("WebFetch: received %d byte(s)%s\n",
           _resp_len, _closed ? " (connection closed)" : " (read timeout)");
    if (_resp_len >= _resp_cap) {
        printf("WebFetch: (capture truncated to %d-byte cap)\n", _resp_cap);
    }
    // Split off the HTTP headers at the blank line, then run the reader-mode
    // parser over the body into the caller's buffers.
    const char* body = _resp;
    int body_len = _resp_len;
    for (int i = 0; _resp && i + 3 < _resp_len; i++) {
        if (_resp[i] == '\r' && _resp[i + 1] == '\n' &&
            _resp[i + 2] == '\r' && _resp[i + 3] == '\n') {
            body = _resp + i + 4;
            body_len = _resp_len - (i + 4);
            break;
        }
    }

    // --- TEMP diagnostic: dump the response head so we can see the status
    //     line + headers (Content-Encoding / Content-Type / Content-Length)
    //     and the first body bytes (clean HTML vs compressed). Remove once the
    //     receive/parse path is sorted.
    {
        int hn = (_resp_len < 512) ? _resp_len : 512;
        printf("WebFetch: --- response head (%d of %d captured) ---\n", hn, _resp_len);
        for (int i = 0; i < hn; i++) {
            unsigned char ch = (unsigned char)_resp[i];
            if (ch == '\n')               putchar('\n');
            else if (ch == '\r')          { /* CR dropped; LF shows breaks */ }
            else if (ch >= 32 && ch < 127) putchar((int)ch);
            else                           printf("\\x%02X", ch);
        }
        putchar('\n');
        int bn = (body_len < 64) ? body_len : 64;
        printf("WebFetch: --- body first %d bytes (hex; 1F 8B == gzip) ---\n", bn);
        for (int i = 0; i < bn; i++) printf("%02X ", (unsigned char)body[i]);
        putchar('\n');
        printf("WebFetch: --- end response head ---\n");
        fflush(stdout);
    }

    ParseResult res = {0, 0, 0};
    if (textOut && textCap > 0) {
        res = parseHtml(body, body_len, textOut, textCap,
                        links, maxLinks, forms, maxForms, url);
    }
    if (out) *out = res;
    printf("WebFetch: parsed %d chars, %d links, %d forms\n",
           res.textLen, res.linkCount, res.formCount);

    bool got = (_resp_len > 0);

    // 9. Close and drop WiFi. Everything is left off; re-enable a companion
    //    manually.
    teardown();
    printf("WebFetch: done (WiFi left off)\n");
    return got;
}
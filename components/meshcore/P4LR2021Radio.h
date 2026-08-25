/*
 * P4LR2021Radio.h -- mesh::Radio backend for the Semtech LR2021 on the
 * LilyGo T-Display P4 LR2021 variant.
 *
 * Built on RadioLib's LR2021 driver (RadioLib >= 7.7.1-43, the commit
 * upstream MeshCore pins for its own LR2021 support) through RadioLib's
 * ESP-IDF HAL. Selected with CONFIG_MECK_RADIO_LR2021; the SX1262 build
 * (P4SX1262Radio, LilyGo cpp_bus_driver) is untouched when it is off.
 *
 * Shape and behaviour mirror P4SX1262Radio:
 *   - the IRQ register is polled over SPI. On this board family the radio's
 *     IRQ line is routed through the XL9535 expander, which is unreliable as
 *     an interrupt source, so nothing is ever attached to it;
 *   - RX is continuous (RX_TIMEOUT_INF) and re-armed after every packet or
 *     failed reception; TX completion is polled;
 *   - isReceiving() uses the PREAMBLE_DETECTED / LORA_HEADER_VALID bits with
 *     stuck-flag deadlines -- the logic of upstream MeshCore's CustomLR2021
 *     (PRs #3115 / #3146), deadlines re-derived from RadioLib's time-on-air;
 *   - noise floor, packet scoring and airtime follow the SX1262 backend.
 *
 * BOARD CONSTANTS TO CONFIRM AT BRING-UP. LilyGo had not published the
 * variant's configuration when this was written; the values below assume
 * the SX1262 footprint's routing (same SPI bus, CS, BUSY, and the XL9535
 * reset / IRQ lines). Each is a single constant:
 *   LR2021_PIN_CS, LR2021_PIN_BUSY        CS = SX1262_CS (24), BUSY = SX1262_BUSY (6)
 *   LR2021_XL_RST, LR2021_XL_IRQ          XL9535 IO16 / IO17
 *   LR2021_TCXO_VOLTAGE                   3.3 V (LilyGo's LR2021 example); falls
 *                                         back to 0 V if the chip rejects it
 *                                         (-706/-707), as upstream does
 *   LR2021_SPI_HZ                         SPI clock for the HAL device.
 *                                         The bus itself is SPI2_HOST, the
 *                                         same host LilyGo's main.cpp uses
 *                                         for the SX1262 (and the keyboard's
 *                                         CC1101/nRF24); RadioLib's EspHal
 *                                         tolerates a pre-initialised bus
 *                                         (ESP_ERR_INVALID_STATE accepted,
 *                                         EspHal.cpp) and only adds its own
 *                                         CS device, so sharing is safe
 *   LR2021_RF_SWITCH_*                    set from LilyGo's LR2021 example
 *                                         (LR2021_RF_SWITCH_TABLE below); the
 *                                         LR2021 drives the switch from its
 *                                         DIOs and RadioLib picks the path from
 *                                         the frequency, so 2.4 GHz needs no
 *                                         GPIO action
 * Not yet ported for this backend: MECK_RX_DUTY_CYCLE (the SX1262 path's
 * listen/sleep duty cycling) and rx-boosted gain.
 */
#pragma once

#include <Dispatcher.h>   // mesh::Radio
#include <MeshCore.h>     // MAX_TRANS_UNIT
#include <RadioLib.h>
#include "hal/ESP-IDF/EspHal.h"
#include "cpp_bus_driver_library.h"
#include "t_display_p4_config.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "meck_log.h"
#include <memory>
#include <stdio.h>

// LilyGo's main.cpp constructs the XL9535 expander at file scope.
extern std::unique_ptr<Cpp_Bus_Driver::Xl95x5> XL9535;

// ---- board constants (see header comment) ----
#define LR2021_PIN_CS        SX1262_CS
#define LR2021_PIN_BUSY      SX1262_BUSY
#define LR2021_XL_RST        XL9535_SX1262_RST     // Cpp_Bus_Driver::Xl95x5::Pin::IO16
#define LR2021_XL_IRQ        XL9535_SX1262_DIO1    // Cpp_Bus_Driver::Xl95x5::Pin::IO17
#ifndef LR2021_TCXO_VOLTAGE
// LilyGo's LR2021 example (radiolib_lr2021_send_receive) uses 3.3 V.
#define LR2021_TCXO_VOLTAGE  3.3f
#endif
#ifndef LR2021_SPI_HZ
#define LR2021_SPI_HZ        8000000
#endif

// Virtual pin numbers for the two radio lines that live on the XL9535. The
// HAL maps them; RadioLib only ever sees plain pin numbers.
static constexpr uint32_t LR2021_VPIN_RST = 0x100;
static constexpr uint32_t LR2021_VPIN_IRQ = 0x101;

// RadioLib's ESP-IDF HAL, with reset and IRQ redirected to the XL9535.
// LR2021 antenna (RF) switch, from LilyGo's radiolib_lr2021_send_receive
// example for this board. The LR2021 drives the switch from its own DIO
// lines; RadioLib selects the mode automatically from the frequency (the
// sub-GHz MODE_RX/TX vs the 2.4 GHz MODE_RX_HF/TX_HF, chosen by the driver's
// highFreq flag). So tuning to a 2.4 GHz frequency routes the antenna to the
// 2.4 GHz port with no GPIO action. Installed once in init() via
// setRfSwitchTable(). Values are transcribed verbatim from LilyGo's example.
static const uint32_t LR2021_RF_SWITCH_PINS[] = {
    RADIOLIB_NC,
    RADIOLIB_LR2021_DIO6,
    RADIOLIB_LR2021_DIO7,
    RADIOLIB_LR2021_DIO8,
    RADIOLIB_LR2021_DIO10,
};
static const Module::RfSwitchMode_t LR2021_RF_SWITCH_TABLE[] = {
    // mode              NC   DIO6 DIO7 DIO8 DIO10
    { LR2021::MODE_STBY,  { 0, 0, 0, 0, 0 } },
    { LR2021::MODE_RX,    { 0, 0, 0, 1, 0 } },   // sub-GHz RX
    { LR2021::MODE_TX,    { 0, 0, 0, 1, 0 } },   // sub-GHz TX
    { LR2021::MODE_RX_HF, { 0, 1, 0, 0, 1 } },   // 2.4 GHz RX
    { LR2021::MODE_TX_HF, { 0, 0, 1, 0, 1 } },   // 2.4 GHz TX
    END_OF_MODE_TABLE,
};

class P4RadioHal : public EspHal {
public:
    P4RadioHal() : EspHal(SX1262_SCLK, SX1262_MISO, SX1262_MOSI, SPI2_HOST, LR2021_SPI_HZ) {}

    void pinMode(uint32_t pin, uint32_t mode) override {
        if (pin == LR2021_VPIN_RST || pin == LR2021_VPIN_IRQ) {
            if (!XL9535) return;
            XL9535->pin_mode(pin == LR2021_VPIN_RST ? LR2021_XL_RST : LR2021_XL_IRQ,
                             (pin == LR2021_VPIN_RST) ? Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT
                                                      : Cpp_Bus_Driver::Xl95x5::Mode::INPUT);
            return;
        }
        EspHal::pinMode(pin, mode);
    }
    void digitalWrite(uint32_t pin, uint32_t value) override {
        if (pin == LR2021_VPIN_RST || pin == LR2021_VPIN_IRQ) {
            if (!XL9535 || pin != LR2021_VPIN_RST) return;
            XL9535->pin_write(LR2021_XL_RST, value ? Cpp_Bus_Driver::Xl95x5::Value::HIGH
                                                   : Cpp_Bus_Driver::Xl95x5::Value::LOW);
            return;
        }
        EspHal::digitalWrite(pin, value);
    }
    uint32_t digitalRead(uint32_t pin) override {
        if (pin == LR2021_VPIN_IRQ) return (XL9535 && XL9535->pin_read(LR2021_XL_IRQ) == 1) ? 1 : 0;
        if (pin == LR2021_VPIN_RST) return 0;
        return EspHal::digitalRead(pin);
    }
    // The IRQ line is polled through the IRQ register; never attach to it.
    void attachInterrupt(uint32_t interruptNum, void (*cb)(void), uint32_t mode) override {
        if (interruptNum == LR2021_VPIN_IRQ || interruptNum == LR2021_VPIN_RST) return;
        EspHal::attachInterrupt(interruptNum, cb, mode);
    }
    void detachInterrupt(uint32_t interruptNum) override {
        if (interruptNum == LR2021_VPIN_IRQ || interruptNum == LR2021_VPIN_RST) return;
        EspHal::detachInterrupt(interruptNum);
    }
    uint32_t pinToInterrupt(uint32_t pin) override {
        return pin;
    }
};

class P4LR2021Radio : public mesh::Radio {
public:
    P4LR2021Radio()
        : _hal(),
          _mod(&_hal, LR2021_PIN_CS, LR2021_VPIN_IRQ, LR2021_VPIN_RST, LR2021_PIN_BUSY),
          _radio(&_mod),
          _inReceiveMode(false), _sending(false), _initOk(false),
          _txStartUs(0), _txTimeoutUs(0),
          _lastRSSI(0), _lastSNR(0), _noiseFloor(-100), _pktRecv(0), _pktSent(0),
          _lastFloorSampleUs(0),
          _currentFreq(0), _currentBW(0), _currentSF(0), _currentCR(0), _txPower(0),
          _preambleMillis(66), _maxPayloadMillis(3934), _activityAtUs(0), _headerSeen(false) {}

    // ---- bring-up (called once from meck_radio_attach) ----
    // Full chip init: reset through the XL9535 (RadioLib drives it via the
    // HAL's virtual pin), begin() with the given LoRa parameters and the
    // MeshCore sync word, CRC on, explicit header, then continuous RX.
    bool init(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t tx_power) {
        _currentFreq = freq; _currentBW = bw; _currentSF = sf; _currentCR = cr; _txPower = tx_power;
        float tcxo = LR2021_TCXO_VOLTAGE;
        int16_t st = _radio.begin(freq, bw, sf, cr, RADIOLIB_LR2021_LORA_SYNC_WORD_PRIVATE,
                                  (int8_t)tx_power, preambleFor(sf), tcxo);
        if (st == RADIOLIB_ERR_SPI_CMD_FAILED || st == RADIOLIB_ERR_SPI_CMD_INVALID) {
            // Same fallback as upstream: a module without a TCXO rejects the
            // TCXO command; retry with it off.
            tcxo = 0.0f;
            st = _radio.begin(freq, bw, sf, cr, RADIOLIB_LR2021_LORA_SYNC_WORD_PRIVATE,
                              (int8_t)tx_power, preambleFor(sf), tcxo);
        }
        if (st != RADIOLIB_ERR_NONE) {
            printf("P4LR2021Radio: begin() failed: %d\n", (int)st);
            _initOk = false;
            return false;
        }
        _radio.setCRC(2);
        _radio.explicitHeader();
        // Antenna switch: LR2021 drives it from its DIOs, RadioLib selects the
        // path from the frequency (2.4 GHz -> the HF modes). See the table above.
        _radio.setRfSwitchTable(LR2021_RF_SWITCH_PINS, LR2021_RF_SWITCH_TABLE);
        printf("P4LR2021Radio: init ok (tcxo=%.1fV) %.3f MHz SF%u BW%.1f CR4/%u TX %u dBm\n",
               (double)tcxo, (double)freq, (unsigned)sf, (double)bw, (unsigned)cr, (unsigned)tx_power);
        computeIrqDeadlines();
        _initOk = true;
        startRx();
        return true;
    }

    // ---- mesh::Radio ----
    void begin() override {
        if (_initOk && !_inReceiveMode) startRx();
    }

    int recvRaw(uint8_t* bytes, int sz) override {
        if (!_initOk) return 0;
        if (!_inReceiveMode) return 0;

        const uint64_t now = esp_timer_get_time();
        if (now - _lastFloorSampleUs >= 250000) {   // 250 ms, as the SX1262 backend
            _lastFloorSampleUs = now;
            sampleNoiseFloor();
        }

        const uint32_t irq = _radio.getIrqFlags();
        if (irq == 0) return 0;

        // A failed reception: drop every flag it left behind and re-arm RX.
        if (irq & (RADIOLIB_LR2021_IRQ_CRC_ERROR | RADIOLIB_LR2021_IRQ_TIMEOUT |
                   RADIOLIB_LR2021_IRQ_LORA_HDR_CRC_ERROR)) {
            clearAndResetRx();
            return 0;
        }
        if (!(irq & RADIOLIB_LR2021_IRQ_RX_DONE)) {
            // PREAMBLE_DETECTED / HEADER_VALID only: a packet is in flight.
            // Leave the receiver alone; isReceiving() reports the state and
            // expires the flags if they go stale.
            return 0;
        }

        size_t len = _radio.getPacketLength();
        if (len == 0 || (int)len > sz) {
            clearAndResetRx();
            return 0;
        }
        int16_t st = _radio.readData(bytes, len);
        if (st != RADIOLIB_ERR_NONE) {
            clearAndResetRx();
            return 0;
        }
        _lastRSSI = _radio.getRSSI();
        _lastSNR  = _radio.getSNR();
        clearAndResetRx();
        _pktRecv++;
        return (int)len;
    }

    uint32_t getEstAirtimeFor(int len_bytes) override {
        // RadioLib's time-on-air at the current parameters (microseconds).
        RadioLibTime_t us = _radio.getTimeOnAir((size_t)len_bytes);
        uint32_t ms = (uint32_t)((us + 999) / 1000);
        return ms == 0 ? 1 : ms;
    }

    float packetScore(float snr, int packet_len) override {
        // As the SX1262 backend: -20 dB -> 0.0, +20 dB -> 1.0
        return (snr + 20.0f) / 40.0f;
    }

    bool startSendRaw(const uint8_t* bytes, int len) override {
        if (!_initOk) return false;
        _radio.standby();
        _radio.clearIrqFlags(RADIOLIB_LR2021_IRQ_ALL);
        clearRxActivity();
        int16_t st = _radio.startTransmit(bytes, (size_t)len);
        if (st != RADIOLIB_ERR_NONE) {
            printf("P4LR2021Radio: startTransmit failed: %d\n", (int)st);
            startRx();
            return false;
        }
        _inReceiveMode = false;
        _sending = true;
        _txStartUs = esp_timer_get_time();
        // Generous bound: twice the estimated airtime plus a second.
        _txTimeoutUs = (uint64_t)getEstAirtimeFor(len) * 2000ULL + 1000000ULL;
        return true;
    }

    bool isSendComplete() override {
        if (!_sending) return true;
        const uint32_t irq = _radio.getIrqFlags();
        if (irq & RADIOLIB_LR2021_IRQ_TX_DONE) return true;
        if (esp_timer_get_time() - _txStartUs > _txTimeoutUs) {
            printf("P4LR2021Radio: TX timeout\n");
            return true;
        }
        return false;
    }

    void onSendFinished() override {
        _radio.finishTransmit();
        _sending = false;
        _pktSent++;
        startRx();
    }

    bool isInRecvMode() const override { return _inReceiveMode; }

    // Ported from upstream CustomLR2021::isReceiving() (v1.17.1): preamble
    // without a header within _preambleMillis, or a header without RX_DONE
    // within _maxPayloadMillis, is treated as a stuck flag and cleared.
    bool isReceiving() override {
        if (!_initOk || !_inReceiveMode) return false;
        const uint32_t irq = _radio.getIrqFlags();
        const bool preamble = (irq & RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED) != 0;
        const bool header   = (irq & RADIOLIB_LR2021_IRQ_LORA_HEADER_VALID) != 0;
        const bool hdrErr   = (irq & RADIOLIB_LR2021_IRQ_LORA_HDR_CRC_ERROR) != 0;
        const uint64_t now  = esp_timer_get_time();
        const uint32_t progressBits = RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED |
                                      RADIOLIB_LR2021_IRQ_LORA_HEADER_VALID |
                                      RADIOLIB_LR2021_IRQ_LORA_HDR_CRC_ERROR;
        if (hdrErr) {
            _radio.clearIrqFlags(progressBits);
            clearRxActivity();
            return false;
        }
        if (!header && _headerSeen) {
            clearRxActivity();
            return false;
        }
        if (header) {
            if (!_headerSeen) { _headerSeen = true; _activityAtUs = now; }
            if (now - _activityAtUs > (uint64_t)_maxPayloadMillis * 1000ULL) {
                printf("P4LR2021Radio: clearing header IRQ after %ums\n", (unsigned)_maxPayloadMillis);
                _radio.clearIrqFlags(progressBits);
                clearRxActivity();
                return false;
            }
            return true;
        }
        if (preamble) {
            if (_activityAtUs == 0) _activityAtUs = now;
            if (now - _activityAtUs > (uint64_t)_preambleMillis * 1000ULL) {
                printf("P4LR2021Radio: clearing preamble IRQ after %ums\n", (unsigned)_preambleMillis);
                _radio.clearIrqFlags(RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED);
                _activityAtUs = 0;
                return false;
            }
            return true;
        }
        clearRxActivity();
        return false;
    }

    float getLastRSSI() const override { return _lastRSSI; }
    float getLastSNR() const override { return _lastSNR; }
    int getNoiseFloor() const override { return _noiseFloor; }
    void resetAGC() override { }   // not implemented on this backend

    // ---- Meck-side accessors (same names as P4SX1262Radio) ----
    uint32_t getPacketsRecv() const { return _pktRecv; }
    uint32_t getPacketsSent() const { return _pktSent; }

    // Apply new LoRa parameters (from radio_set_params). Preamble length and
    // sync word follow the SX1262 backend: 32 symbols at SF<=8 else 16, and
    // MeshCore's private sync word.
    void setParams(float freq, float bw, uint8_t sf, uint8_t cr) {
        _currentFreq = freq; _currentBW = bw; _currentSF = sf; _currentCR = cr;
        if (!_initOk) return;
        _radio.standby();
        int16_t st;
        if ((st = _radio.setFrequency(freq)) != RADIOLIB_ERR_NONE)        printf("P4LR2021Radio: setFrequency %d\n", (int)st);
        if ((st = _radio.setBandwidth(bw)) != RADIOLIB_ERR_NONE)          printf("P4LR2021Radio: setBandwidth %d\n", (int)st);
        if ((st = _radio.setSpreadingFactor(sf)) != RADIOLIB_ERR_NONE)    printf("P4LR2021Radio: setSpreadingFactor %d\n", (int)st);
        if ((st = _radio.setCodingRate(cr)) != RADIOLIB_ERR_NONE)         printf("P4LR2021Radio: setCodingRate %d\n", (int)st);
        if ((st = _radio.setPreambleLength(preambleFor(sf))) != RADIOLIB_ERR_NONE) printf("P4LR2021Radio: setPreambleLength %d\n", (int)st);
        if ((st = _radio.setSyncWord((uint8_t)RADIOLIB_LR2021_LORA_SYNC_WORD_PRIVATE)) != RADIOLIB_ERR_NONE) printf("P4LR2021Radio: setSyncWord %d\n", (int)st);
        if ((st = _radio.setOutputPower((int8_t)_txPower)) != RADIOLIB_ERR_NONE) printf("P4LR2021Radio: setOutputPower %d\n", (int)st);
        computeIrqDeadlines();
        startRx();
    }

    void setTxPower(uint8_t dbm) {
        _txPower = dbm;
        if (!_initOk) return;
        // Upstream PR #3218: on the LR2021, setting output power while the
        // radio is receiving can leave the receiver down until reboot, so
        // drop to standby first and re-arm RX afterwards (setParams already
        // follows this shape).
        _radio.standby();
        int16_t st = _radio.setOutputPower((int8_t)dbm);
        if (st != RADIOLIB_ERR_NONE) printf("P4LR2021Radio: setOutputPower(%u) %d\n", (unsigned)dbm, (int)st);
        clearAndResetRx();
    }

    bool isInitialised() const { return _initOk; }

private:
    P4RadioHal _hal;
    Module     _mod;
    LR2021     _radio;

    bool     _inReceiveMode;
    bool     _sending;
    bool     _initOk;
    uint64_t _txStartUs;
    uint64_t _txTimeoutUs;
    float    _lastRSSI;
    float    _lastSNR;
    int      _noiseFloor;
    uint32_t _pktRecv;
    uint32_t _pktSent;
    uint64_t _lastFloorSampleUs;
    float    _currentFreq;
    float    _currentBW;
    uint8_t  _currentSF;
    uint8_t  _currentCR;
    uint8_t  _txPower;

    // Receive-in-progress tracking for isReceiving() (see there).
    uint32_t _preambleMillis;
    uint32_t _maxPayloadMillis;
    uint64_t _activityAtUs;
    bool     _headerSeen;

    static uint16_t preambleFor(uint8_t sf) { return (sf <= 8) ? 32 : 16; }

    void clearRxActivity() { _activityAtUs = 0; _headerSeen = false; }

    // Continuous RX with the receive-progress bits reported (upstream
    // CustomLR2021::startReceive adds PREAMBLE_DETECTED to the defaults).
    void startRx() {
        int16_t st = _radio.startReceive(RADIOLIB_LR2021_RX_TIMEOUT_INF,
                                         RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED),
                                         RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
        if (st != RADIOLIB_ERR_NONE) {
            printf("P4LR2021Radio: startReceive failed: %d\n", (int)st);
            _inReceiveMode = false;
            return;
        }
        _inReceiveMode = true;
    }

    void clearAndResetRx() {
        _radio.clearIrqFlags(RADIOLIB_LR2021_IRQ_ALL);
        clearRxActivity();
        startRx();
    }

    // Stuck-IRQ deadlines from the current params (upstream
    // RadioLibWrapper::calcMaxPacketMillis, PR #2977): preamble +8 symbols
    // plus 6.25 (SF5/6) or 4.25 symbols; payload = max packet at the current
    // settings minus that, rescaled to CR 4/8.
    void computeIrqDeadlines() {
        if (_currentBW <= 0.0f || _currentSF == 0) return;
        float bw_hz   = _currentBW * 1000.0f;
        float tsym_us = (float)(1UL << _currentSF) * 1e6f / bw_hz;
        float sf_coeff = (_currentSF == 5 || _currentSF == 6) ? 6.25f : 4.25f;
        uint32_t preamble_us = (uint32_t)(((float)preambleFor(_currentSF) + 8.0f + sf_coeff) * tsym_us);
        uint32_t total_us    = getEstAirtimeFor(MAX_TRANS_UNIT) * 1000u;
        uint32_t payload_us  = (total_us > preamble_us) ? (total_us - preamble_us) : (4000000u - preamble_us);
        if (_currentCR >= 5 && _currentCR < 8) payload_us = (payload_us * 8u) / _currentCR;
        _preambleMillis   = (preamble_us + 999u) / 1000u;
        _maxPayloadMillis = (payload_us + 999u) / 1000u;
        printf("P4LR2021Radio: irq deadlines preamble=%ums payload=%ums\n",
               (unsigned)_preambleMillis, (unsigned)_maxPayloadMillis);
    }

    // Instantaneous RSSI folded into an 8-sample EMA, skipped while a packet
    // is in flight (same gating and clamps as the SX1262 backend).
    void sampleNoiseFloor() {
        if (!_inReceiveMode) return;
        const uint32_t irq = _radio.getIrqFlags();
        if (irq & (RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED | RADIOLIB_LR2021_IRQ_LORA_HEADER_VALID)) return;
        float inst = _radio.getRSSI(false, true);   // instantaneous, do not re-arm RX
        if (inst == 0.0f) return;
        int sample = (int)inst;
        if (sample > -50) return;
        _noiseFloor = (_noiseFloor * 7 + sample) / 8;
        if (_noiseFloor < -120) _noiseFloor = -120;
    }
};
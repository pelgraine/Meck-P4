/*
 * P4SX1262Radio.h — MeshCore mesh::Radio implementation for T-Display P4
 *
 * Wraps cpp_bus_driver::Sx126x to implement the mesh::Radio interface that
 * MeshCore's Dispatcher expects. This replaces RadioLib + CustomSX1262Wrapper
 * used on Arduino-based targets.
 *
 * Hardware notes:
 *   - SX1262 SPI: direct GPIO (CS=24, BUSY=6, SCLK=2, MOSI=3, MISO=4)
 *   - SX1262 RST: XL9535 IO16 (I/O expander, not direct GPIO)
 *   - SX1262 DIO1: XL9535 IO17 (IRQ via I/O expander — NOT RELIABLE for polling)
 *   - SKY13453 RF switch: XL9535 IO1 (VCTL high = TX/RX path)
 *
 * NOTE: DIO1 polling through the XL9535 I2C expander does not work reliably.
 * All IRQ detection uses direct SPI reads of the SX1262 IRQ status register.
 *
 * Reference: Homertrix main.cpp lora_tx_raw() / lora_rx_task()
 */

#pragma once

#include <Dispatcher.h>   // for mesh::Radio interface
#include "cpp_bus_driver_library.h"
#include "t_display_p4_config.h"
#include "esp_timer.h"

// LilyGo's main.cpp defines `auto SX1262 = std::make_unique<...>(...)` at
// file scope. That gives a global with external linkage. We reference it
// here so the radio adapter methods can drive the chip.
//
// NOTE: this header MUST NOT be included by main.cpp directly — doing so
// produces a conflicting-declaration error against the same-named `auto`
// global. Internal meshcore-component code is fine to include it.
extern std::unique_ptr<Cpp_Bus_Driver::Sx126x> SX1262;

class P4SX1262Radio : public mesh::Radio {
public:
    P4SX1262Radio() 
        : _inReceiveMode(false)
        , _lastRSSI(0)
        , _lastSNR(0)
        , _pktRecv(0)
        , _pktSent(0)
        , _currentFreq(0)
        , _currentBW(0)
        , _currentSF(0)
        , _currentCR(0)
        , _noiseFloor(-120)            // matches MeshCore's clamp / cold-start
        , _lastFloorSampleUs(0)
    {}

    // ---- mesh::Radio interface implementation ----

    void begin() override {
        // Radio hardware init is done in radio_init() (target.cpp)
        // This is called after that, so radio should be in RX mode already
        _inReceiveMode = true;
    }

    int recvRaw(uint8_t* bytes, int sz) override {
        // Periodic noise-floor sample. This rides the existing recvRaw()
        // call cadence (Dispatcher loop, SPI lock already held). Fires at
        // most every 2 s — the same rate MeshCore uses for its calibrate
        // tick. sampleNoiseFloor() handles its own gating (must be in RX
        // mode, must not be mid-packet).
        uint64_t now_us = esp_timer_get_time();
        if (now_us - _lastFloorSampleUs >= 2000000ULL) {
            _lastFloorSampleUs = now_us;
            sampleNoiseFloor();
        }

        if (!_inReceiveMode) return 0;

        // Poll IRQ status register directly via SPI (DIO1 via XL9535 unreliable)
        uint16_t irq = SX1262->get_irq_flag();
        if (irq == 0) return 0;

        // Parse IRQ status
        Cpp_Bus_Driver::Sx126x::Irq_Status irq_status;
        if (!SX1262->parse_irq_status(irq, irq_status)) {
            clearAndResetRx();
            return 0;
        }

        if (irq_status.all_flag.crc_error) {
            SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::CRC_ERROR);
            resetToRx();
            return 0;
        }

        if (irq_status.all_flag.tx_rx_timeout) {
            SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TIMEOUT);
            resetToRx();
            return 0;
        }

        if (!irq_status.all_flag.rx_done) {
            clearAndResetRx();
            return 0;
        }

        // Read received data
        uint8_t recv_len = SX1262->receive_data(bytes);
        if (recv_len == 0 || recv_len > sz) {
            SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
            resetToRx();
            return 0;
        }

        // Read signal metrics
        Cpp_Bus_Driver::Sx126x::Packet_Metrics pm;
        if (SX1262->get_lora_packet_metrics(pm)) {
            _lastRSSI = (float)pm.lora.rssi_average;
            _lastSNR = (float)pm.lora.snr;
        }

        SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
        resetToRx();

        _pktRecv++;
        return (int)recv_len;
    }

    uint32_t getEstAirtimeFor(int len_bytes) override {
        // Standard LoRa airtime calculation
        // Reference: SX1262 datasheet section 6.1.4, RadioLib implementation
        if (_currentBW <= 0 || _currentSF == 0) return 100;  // fallback

        float bw_hz = _currentBW * 1000.0f;
        float ts = powf(2.0f, (float)_currentSF) / bw_hz;  // symbol time in seconds
        
        // Preamble time
        float preamble_symbols = (_currentSF <= 8) ? 32.0f : 16.0f;
        float t_preamble = (preamble_symbols + 4.25f) * ts;

        // Payload symbols
        int de = (_currentSF >= 11 && _currentBW <= 125.0f) ? 1 : 0;  // low data rate optimize
        int cr_val = _currentCR;  // coding rate 5-8 for 4/5 to 4/8
        
        float numerator = 8.0f * len_bytes - 4.0f * _currentSF + 28.0f + 16.0f;  // CRC=ON
        float denominator = 4.0f * ((float)_currentSF - 2.0f * de);
        if (denominator <= 0) denominator = 1;
        
        int n_payload = 8 + (int)(ceilf(numerator / denominator) * (cr_val));
        if (n_payload < 8) n_payload = 8;

        float t_payload = (float)n_payload * ts;
        float airtime_s = t_preamble + t_payload;

        return (uint32_t)(airtime_s * 1000.0f);  // return milliseconds
    }

    float packetScore(float snr, int packet_len) override {
        // Higher SNR = better. Normalize to ~0-1 range.
        // MeshCore uses this for path quality scoring
        return (snr + 20.0f) / 40.0f;  // -20dB → 0.0, +20dB → 1.0
    }

    bool startSendRaw(const uint8_t* bytes, int len) override {
        _inReceiveMode = false;
        printf("P4SX1262Radio::startSendRaw: %d bytes\n", len);

        // Configure for TX
        SX1262->start_lora_transmit(
            Cpp_Bus_Driver::Sx126x::Chip_Mode::TX, 0,
            Cpp_Bus_Driver::Sx126x::Fallback_Mode::FS
        );
        SX1262->set_irq_pin_mode(
            Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TX_DONE,
            Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE,
            Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE
        );
        SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TX_DONE);

        // Send data
        // cpp_bus_driver::send_data takes non-const pointer; cast is safe here
        SX1262->send_data(const_cast<uint8_t*>(bytes), len);

        _pktSent++;
        return true;
    }

    bool isSendComplete() override {
        // Poll IRQ register directly via SPI (DIO1 via XL9535 unreliable)
        Cpp_Bus_Driver::Sx126x::Irq_Status irq_status;
        if (SX1262->parse_irq_status(SX1262->get_irq_flag(), irq_status)) {
            if (irq_status.all_flag.tx_done) {
                return true;
            }
        }
        return false;
    }

    void onSendFinished() override {
        SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TX_DONE);
        resetToRx();
        _inReceiveMode = true;
    }

    bool isInRecvMode() const override {
        return _inReceiveMode;
    }

    bool isReceiving() override {
        // Check if radio is currently mid-packet (BUSY high during RX)
        // On SX1262, BUSY pin goes high during packet reception
        // This is a direct GPIO read, not via XL9535
        return (gpio_get_level((gpio_num_t)SX1262_BUSY) == 1) && _inReceiveMode;
    }

    float getLastRSSI() const override { return _lastRSSI; }
    float getLastSNR() const override { return _lastSNR; }

    int getNoiseFloor() const override {
        return _noiseFloor;
    }

    // Sample the chip's instantaneous RSSI and fold it into the running
    // noise-floor estimate. Called periodically from recvRaw() under the
    // SPI lock. Skips when not in RX mode or while a packet is mid-decode
    // (BUSY high), to avoid contaminating the floor with signal energy.
    // Smoothing: simple 8-sample EMA. Clamp at -120 dBm matches MeshCore.
    void sampleNoiseFloor() {
        if (!_inReceiveMode) return;
        // Skip if a packet is currently being demodulated. Same guard
        // isReceiving() uses, repeated here so this method can be safely
        // called from anywhere without depending on caller's gating.
        if (gpio_get_level((gpio_num_t)SX1262_BUSY) == 1) return;

        int8_t inst = SX1262->get_rssi_inst();
        if (inst == 0) return;             // read/parse failure
        int sample = (int)inst;

        // Reject implausibly-high readings (likely a signal leaking in
        // before BUSY went high, or a transient). Anything stronger than
        // -50 dBm is not noise, regardless of band.
        if (sample > -50) return;

        // EMA: alpha = 1/8 — responsive enough to follow band changes,
        // smoothed enough that a single weak packet edge doesn't budge
        // the floor visibly.
        _noiseFloor = (_noiseFloor * 7 + sample) / 8;
        if (_noiseFloor < -120) _noiseFloor = -120;
    }

    void resetAGC() override {
        // SX1262 RX boosted gain mode for better sensitivity
        // cpp_bus_driver may not expose this directly; 
        // can be implemented via raw register write if needed
    }

    // ---- Additional accessors for stats ----
    uint32_t getPacketsRecv() const { return _pktRecv; }
    uint32_t getPacketsSent() const { return _pktSent; }

    // ---- Radio parameter storage (set by radio_set_params) ----
    void setParams(float freq, float bw, uint8_t sf, uint8_t cr) {
        _currentFreq = freq;
        _currentBW = bw;
        _currentSF = sf;
        _currentCR = cr;
    }

private:
    bool _inReceiveMode;
    float _lastRSSI;
    float _lastSNR;
    uint32_t _pktRecv;
    uint32_t _pktSent;

    // Current radio parameters (for airtime calculation)
    float _currentFreq;
    float _currentBW;
    uint8_t _currentSF;
    uint8_t _currentCR;

    // Noise-floor estimator state. Updated periodically from recvRaw().
    // _noiseFloor is the current running value in dBm; _lastFloorSampleUs
    // throttles sampling to ~2 s intervals.
    int      _noiseFloor;
    uint64_t _lastFloorSampleUs;

    void resetToRx() {
        SX1262->start_lora_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::RX);
        SX1262->set_irq_pin_mode(
            Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE,
            Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE,
            Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE
        );
        SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
    }

    void clearAndResetRx() {
        SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
        resetToRx();
    }
};
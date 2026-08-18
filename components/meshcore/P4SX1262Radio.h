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

// Debug Logs: rewrites printf -> meck_debug_log_printf so the
// startSendRaw line and any other printfs below land in the SD log
// file when Settings > Debug Logs > Start is active. See meck_log.h.
#include "meck_log.h"

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
#if defined(MECK_RX_DUTY_CYCLE)
        computeDutyPeriods();
        _dutyState = DutyState::LISTEN;
        _dutyWindowStartUs = esp_timer_get_time();
#endif
    }

    int recvRaw(uint8_t* bytes, int sz) override {
#if !defined(MECK_RX_DUTY_CYCLE)
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
#endif

        if (!_inReceiveMode) return 0;

#if defined(MECK_RX_DUTY_CYCLE)
        // Advance the listen/sleep state machine. While the chip is in a
        // sleep window there is nothing to read, and we must not touch it
        // over SPI (that would wake it), so bail before any register access.
        dutyCycleTick();
        if (_dutyState == DutyState::ASLEEP) return 0;
#endif

        // Poll IRQ status register directly via SPI (DIO1 via XL9535 unreliable)
        uint16_t irq = SX1262->get_irq_flag();
        if (irq == 0) return 0;

        // Parse IRQ status
        Cpp_Bus_Driver::Sx126x::Irq_Status irq_status;
        if (!SX1262->parse_irq_status(irq, irq_status)) {
            clearAndResetRx();
            return 0;
        }

        // A failed reception (bad CRC, RX timeout, bad header): drop every
        // flag it left behind -- including the PREAMBLE_DETECTED /
        // HEADER_VALID bits from the same packet -- and re-arm RX. Clearing
        // only the one bit, as before, left the others set until the next
        // packet (and a stuck HEADER_ERROR re-armed RX on every loop pass).
        if (irq_status.all_flag.crc_error || irq_status.all_flag.tx_rx_timeout ||
            irq_status.lora_reg_flag.header_error) {
            clearAndResetRx();
            return 0;
        }

        if (!irq_status.all_flag.rx_done) {
            // Only PREAMBLE_DETECTED / HEADER_VALID: a packet is in flight.
            // Leave the receiver alone; isReceiving() reports the state and
            // expires the flags if they go stale (upstream #3036 / #2977).
            if (irq_status.all_flag.preamble_detected || irq_status.lora_reg_flag.header_valid) {
                return 0;
            }
            clearAndResetRx();
            return 0;
        }

        // Read received data
        uint8_t recv_len = SX1262->receive_data(bytes);
        if (recv_len == 0 || recv_len > sz) {
            clearAndResetRx();
            return 0;
        }

        // Read signal metrics
        Cpp_Bus_Driver::Sx126x::Packet_Metrics pm;
        if (SX1262->get_lora_packet_metrics(pm)) {
            _lastRSSI = (float)pm.lora.rssi_average;
            _lastSNR = (float)pm.lora.snr;
        }

        // Packet consumed: clear all of its flags (RX_DONE plus the
        // preamble / header bits) so isReceiving() does not keep reporting
        // it, then re-arm.
        clearAndResetRx();

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
#if defined(MECK_RX_DUTY_CYCLE)
        // If a duty-cycle sleep window is in progress the chip is asleep;
        // wake it to standby before configuring TX.
        if (_dutyState == DutyState::ASLEEP) {
            SX1262->set_standby(Cpp_Bus_Driver::Sx126x::Stdby_Config::STDBY_RC);
            _dutyState = DutyState::LISTEN;
        }
#endif
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
#if defined(MECK_RX_DUTY_CYCLE)
        // Resume the duty cycle with a fresh listen window.
        _dutyState = DutyState::LISTEN;
        _dutyWindowStartUs = esp_timer_get_time();
#endif
    }

    bool isInRecvMode() const override {
        return _inReceiveMode;
    }

    // Is a packet arriving? Ported from upstream MeshCore v1.17.1
    // CustomSX1262::isReceiving() (PRs #3036, #2977 and follow-ups): the
    // PREAMBLE_DETECTED and HEADER_VALID IRQ bits (enabled by armRxIrqMask)
    // report a packet in flight, and each is expired on a deadline if the
    // reception never progresses -- preamble without a header within
    // _preambleMillis, or header without RX_DONE within _maxPayloadMillis --
    // so a stuck flag cannot hold the radio "busy" until the next packet.
    // The previous BUSY-pin reading is kept as an additional signal only.
    bool isReceiving() override {
        if (!_inReceiveMode) return false;
#if defined(MECK_RX_DUTY_CYCLE)
        // Asleep between listen windows: nothing can be arriving, and an SPI
        // read here would wake the chip.
        if (_dutyState == DutyState::ASLEEP) return false;
#endif
        const bool busy = (gpio_get_level((gpio_num_t)SX1262_BUSY) == 1);

        const uint16_t irq = SX1262->get_irq_flag();
        const bool preamble = (irq & IRQ_PREAMBLE_DETECTED) != 0;
        const bool header   = (irq & IRQ_HEADER_VALID) != 0;
        const bool hdrErr   = (irq & IRQ_HEADER_ERROR) != 0;
        const uint64_t now  = esp_timer_get_time();

        if (hdrErr) {
            clearIrqBits(IRQ_PREAMBLE_DETECTED | IRQ_SYNC_WORD_VALID | IRQ_HEADER_VALID | IRQ_HEADER_ERROR);
            clearRxActivity();
            return busy;
        }
        if (!header && _headerSeen) {
            // Something cleared the header flag (recvRaw took the packet); reset.
            clearRxActivity();
            return busy;
        }
        if (header) {
            if (!_headerSeen) { _headerSeen = true; _activityAtUs = now; }
            if (now - _activityAtUs > (uint64_t)_maxPayloadMillis * 1000ULL) {
                printf("P4SX1262Radio: clearing header IRQ after %ums\n", (unsigned)_maxPayloadMillis);
                clearIrqBits(IRQ_PREAMBLE_DETECTED | IRQ_SYNC_WORD_VALID | IRQ_HEADER_VALID | IRQ_HEADER_ERROR);
                clearRxActivity();
                return busy;
            }
            return true;
        }
        if (preamble) {
            if (_activityAtUs == 0) _activityAtUs = now;
            if (now - _activityAtUs > (uint64_t)_preambleMillis * 1000ULL) {
                printf("P4SX1262Radio: clearing preamble IRQ after %ums\n", (unsigned)_preambleMillis);
                clearIrqBits(IRQ_PREAMBLE_DETECTED);
                _activityAtUs = 0;
                return busy;
            }
            return true;
        }
        clearRxActivity();
        return busy;
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
        computeIrqDeadlines();
#if defined(MECK_RX_DUTY_CYCLE)
        computeDutyPeriods();
#endif
    }

    // IRQ bits (SX126x table 13-29). The driver's Irq_Mask_Flag carries the
    // same values; raw bits are used where several are combined.
    static constexpr uint16_t IRQ_TX_DONE           = 0x0001;
    static constexpr uint16_t IRQ_RX_DONE           = 0x0002;
    static constexpr uint16_t IRQ_PREAMBLE_DETECTED = 0x0004;
    static constexpr uint16_t IRQ_SYNC_WORD_VALID   = 0x0008;
    static constexpr uint16_t IRQ_HEADER_VALID      = 0x0010;
    static constexpr uint16_t IRQ_HEADER_ERROR      = 0x0020;
    static constexpr uint16_t IRQ_CRC_ERROR         = 0x0040;
    static constexpr uint16_t IRQ_TIMEOUT           = 0x0200;

    // IRQ mask while listening. The driver's set_irq_pin_mode() fixes the
    // chip's IRQ mask at TX_DONE | RX_DONE | HEADER_ERROR | CRC_ERROR |
    // TIMEOUT (0x0263), which never reports a preamble or a valid header,
    // so nothing but BUSY could say a packet was in flight. PREAMBLE_DETECTED
    // and HEADER_VALID are added here (upstream MeshCore PR #3036); DIO1
    // keeps RX_DONE, though nothing reads it -- the IRQ register is polled
    // over SPI. Called wherever RX is (re-)armed: resetToRx() and
    // meck_radio_attach() in target.cpp.
    static constexpr uint16_t RX_IRQ_MASK = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_HEADER_ERROR |
                                            IRQ_CRC_ERROR | IRQ_TIMEOUT |
                                            IRQ_PREAMBLE_DETECTED | IRQ_HEADER_VALID;
    void armRxIrqMask() {
        SX1262->set_dio_irq_params(RX_IRQ_MASK, IRQ_RX_DONE, 0, 0);
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

    // Receive-in-progress tracking for isReceiving() (see there). Deadlines
    // are refreshed by computeIrqDeadlines() from the current LoRa params;
    // the initial values are upstream's defaults.
    uint32_t _preambleMillis   = 66;
    uint32_t _maxPayloadMillis = 3934;
    uint64_t _activityAtUs     = 0;
    bool     _headerSeen       = false;

    void clearRxActivity() { _activityAtUs = 0; _headerSeen = false; }

    // Clear a combination of IRQ bits. The driver's clear_irq_flag() takes
    // one Irq_Mask_Flag; the enum's values are the raw bits, so a combined
    // value cast to it clears them all in one write.
    void clearIrqBits(uint16_t bits) {
        SX1262->clear_irq_flag((Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag)bits);
    }

    // Stuck-IRQ deadlines from the current params: upstream's
    // RadioLibWrapper::calcMaxPacketMillis() (PR #2977) re-derived on our own
    // airtime maths and preamble length (see target.cpp radio_set_params).
    // preamble: (preamble + 8) symbols plus 6.25 (SF5/6) or 4.25 symbols for
    // sync word / SFD / header. payload: a max-size packet at the current
    // settings minus that, rescaled to CR 4/8 so any sender's packet fits.
    void computeIrqDeadlines() {
        if (_currentBW <= 0.0f || _currentSF == 0) return;
        float bw_hz   = _currentBW * 1000.0f;
        float tsym_us = (float)(1UL << _currentSF) * 1e6f / bw_hz;
        uint32_t preamble_symbols = (_currentSF <= 8) ? 32u : 16u;
        float sf_coeff = (_currentSF == 5 || _currentSF == 6) ? 6.25f : 4.25f;
        uint32_t preamble_us = (uint32_t)(((float)preamble_symbols + 8.0f + sf_coeff) * tsym_us);
        uint32_t total_us    = getEstAirtimeFor(MAX_TRANS_UNIT) * 1000u;
        // fallback of 4 s if the airtime estimate is unusable (it never is:
        // getEstAirtimeFor includes the preamble)
        uint32_t payload_us  = (total_us > preamble_us) ? (total_us - preamble_us) : (4000000u - preamble_us);
        if (_currentCR >= 5 && _currentCR < 8) payload_us = (payload_us * 8u) / _currentCR;
        _preambleMillis   = (preamble_us + 999u) / 1000u;
        _maxPayloadMillis = (payload_us + 999u) / 1000u;
        printf("P4SX1262Radio: irq deadlines preamble=%ums payload=%ums\n",
               (unsigned)_preambleMillis, (unsigned)_maxPayloadMillis);
    }

#if defined(MECK_RX_DUTY_CYCLE)
    // ---- MCU-driven RX duty cycle (approach B) ----
    // The MCU cycles the SX1262 between a short RX listen window and a
    // warm-start sleep window, both timed here over SPI. Windows are sized
    // from the preamble airtime so any incoming preamble overlaps a listen
    // window long enough to be detected, and the cycle never sleeps while a
    // packet is mid-reception (BUSY high). If the preamble is too short to
    // carve out a safe sleep window at the current SF/BW, duty cycling is
    // disabled for that config and the radio stays in continuous RX (same
    // fallback behaviour as RadioLib's startReceiveDutyCycleAuto).
    //
    // This relies on the mesh layer issuing no SPI transaction to the radio
    // during a sleep window (an SPI access wakes a warm-start sleep):
    // recvRaw() bails before any register access when ASLEEP, isReceiving()
    // reads only the BUSY GPIO, noise-floor sampling is compiled out above,
    // and TX wakes the chip first (see startSendRaw).
    enum class DutyState { LISTEN, ASLEEP };
    DutyState _dutyState = DutyState::LISTEN;
    uint64_t  _dutyWindowStartUs = 0;
    uint32_t  _rxWindowUs = 0;
    uint32_t  _sleepWindowUs = 0;   // 0 => duty cycling disabled for current params

    // Listen window length, in preamble symbols. Must cover the chip's
    // preamble-detection time. Larger is safer against MCU timing jitter,
    // smaller yields more sleep. The sleep window is the remaining preamble
    // airtime after this and the jitter margin.
    static constexpr uint32_t DUTY_RX_SYMBOLS = 16;
    // Subtracted from the sleep window to absorb MCU scheduling and SPI /
    // BUSY-assert latency, so a preamble cannot slip entirely through a
    // sleep gap undetected.
    static constexpr uint32_t DUTY_JITTER_MARGIN_US = 4000;

    // Recompute listen/sleep windows from the current LoRa params. Preamble
    // symbol count matches radio_set_params() in target.cpp.
    void computeDutyPeriods() {
        if (_currentBW <= 0.0f || _currentSF == 0) { _rxWindowUs = 0; _sleepWindowUs = 0; return; }
        float bw_hz = _currentBW * 1000.0f;
        float symbol_us = (float)(1UL << _currentSF) * 1e6f / bw_hz;
        uint32_t preamble_symbols = (_currentSF <= 8) ? 32u : 16u;
        uint32_t preamble_air_us  = (uint32_t)(preamble_symbols * symbol_us);
        uint32_t rx_us            = (uint32_t)(DUTY_RX_SYMBOLS * symbol_us);
        if (rx_us + DUTY_JITTER_MARGIN_US >= preamble_air_us) {
            _rxWindowUs = 0;
            _sleepWindowUs = 0;   // preamble too short to sleep safely: continuous RX
            return;
        }
        _rxWindowUs = rx_us;
        _sleepWindowUs = preamble_air_us - rx_us - DUTY_JITTER_MARGIN_US;
    }

    // Advance the listen/sleep state machine. Called at the top of recvRaw()
    // while in receive mode.
    void dutyCycleTick() {
        if (_sleepWindowUs == 0) return;   // disabled for current params
        uint64_t now = esp_timer_get_time();
        if (_dutyState == DutyState::LISTEN) {
            if (isReceiving()) {           // packet in flight (BUSY high): hold RX
                _dutyWindowStartUs = now;
                return;
            }
            if (now - _dutyWindowStartUs >= _rxWindowUs) {
                SX1262->set_sleep(Cpp_Bus_Driver::Sx126x::Sleep_Mode::WARM_START);
                _dutyState = DutyState::ASLEEP;
                _dutyWindowStartUs = now;
            }
        } else {  // ASLEEP
            if (now - _dutyWindowStartUs >= _sleepWindowUs) {
                SX1262->set_standby(Cpp_Bus_Driver::Sx126x::Stdby_Config::STDBY_RC);
                resetToRx();               // re-arm RX + RX_DONE IRQ (warm start retains LoRa config)
                _dutyState = DutyState::LISTEN;
                _dutyWindowStartUs = now;
            }
        }
    }
#endif

    void resetToRx() {
        SX1262->start_lora_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::RX);
        armRxIrqMask();
        SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
    }

    // Drop every IRQ flag and the receive-progress state, then re-arm RX.
    void clearAndResetRx() {
        SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::ALL);
        clearRxActivity();
        resetToRx();
    }
};
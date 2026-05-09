/*
 * target.cpp — MeshCore radio attach + LoRa params for T-Display P4 (Meck)
 *
 * Architecture: LilyGo's main.cpp constructs all hardware globals
 * (XL9535, SX1262, SPI buses, etc.) at file scope as `auto` declarations
 * with external linkage. We reference SX1262 here via a matching `extern`.
 *
 *   - meck_radio_attach() does NOT redo any hardware init. LilyGo already
 *     called SX1262->begin(10MHz), set up power rails, sequenced reset, etc.
 *     We only overwrite the LoRa modulation parameters with MeshCore's
 *     Australia Narrow preset and put the radio back into RX with our IRQ
 *     mask.
 *   - The mesh::Radio interface is implemented entirely by P4SX1262Radio
 *     (header-only); this file just provides the singleton and the
 *     attach/configure helpers.
 */

#include "target.h"
#include "t_display_p4_config.h"
#include "esp_random.h"
#include <stdio.h>

// LilyGo's main.cpp defines `auto SX1262 = std::make_unique<...>(...)` at
// file scope. We declare it extern here so target.cpp can reach it. The
// declaration is also in P4SX1262Radio.h (transitively included via
// target.h above) — duplicate extern declarations are fine, only duplicate
// definitions break.

// ---- MeshCore radio adapter instance ----
P4SX1262Radio radio_driver;


// ============================================================
// meck_radio_attach()
// Called ONCE from app_main() after LilyGo's init is complete.
// ============================================================

bool meck_radio_attach() {
    printf("meck_radio_attach() — wrapping LilyGo's already-running SX1262\n");
    meck_set_antenna_default();

    // Apply MeshCore's preset (overwrites LilyGo's demo SF9/125kHz/920MHz)
    radio_set_params(LORA_FREQ_DEFAULT, LORA_BW_DEFAULT,
                     LORA_SF_DEFAULT, LORA_CR_DEFAULT);
    radio_set_tx_power(LORA_TX_POWER_DEFAULT);

    // Re-enter RX mode with MeshCore-friendly IRQ mask
    SX1262->clear_buffer();
    SX1262->start_lora_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::RX);
    SX1262->set_irq_pin_mode(
        Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE,
        Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE,
        Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::DISABLE
    );
    SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);

    radio_driver.begin();

    printf("meck_radio_attach() — radio ready on %.3f MHz, SF%u, BW=%.1f kHz\n",
           (double)LORA_FREQ_DEFAULT, (unsigned)LORA_SF_DEFAULT,
           (double)LORA_BW_DEFAULT);
    return true;
}


// ============================================================
// radio_set_params() — Configure LoRa modulation
// ============================================================

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr) {
    Cpp_Bus_Driver::Sx126x::Lora_Bw bw_enum;
    if (bw >= 500.0f)       bw_enum = Cpp_Bus_Driver::Sx126x::Lora_Bw::BW_500000HZ;
    else if (bw >= 250.0f)  bw_enum = Cpp_Bus_Driver::Sx126x::Lora_Bw::BW_250000HZ;
    else if (bw >= 125.0f)  bw_enum = Cpp_Bus_Driver::Sx126x::Lora_Bw::BW_125000HZ;
    else if (bw >= 62.5f)   bw_enum = Cpp_Bus_Driver::Sx126x::Lora_Bw::BW_62500HZ;
    else if (bw >= 41.67f)  bw_enum = Cpp_Bus_Driver::Sx126x::Lora_Bw::BW_41670HZ;
    else if (bw >= 31.25f)  bw_enum = Cpp_Bus_Driver::Sx126x::Lora_Bw::BW_31250HZ;
    else                    bw_enum = Cpp_Bus_Driver::Sx126x::Lora_Bw::BW_15630HZ;

    Cpp_Bus_Driver::Sx126x::Sf sf_enum = (Cpp_Bus_Driver::Sx126x::Sf)sf;

    Cpp_Bus_Driver::Sx126x::Cr cr_enum;
    switch (cr) {
        case 5:  cr_enum = Cpp_Bus_Driver::Sx126x::Cr::CR_4_5; break;
        case 6:  cr_enum = Cpp_Bus_Driver::Sx126x::Cr::CR_4_6; break;
        case 7:  cr_enum = Cpp_Bus_Driver::Sx126x::Cr::CR_4_7; break;
        case 8:  cr_enum = Cpp_Bus_Driver::Sx126x::Cr::CR_4_8; break;
        default: cr_enum = Cpp_Bus_Driver::Sx126x::Cr::CR_4_5; break;
    }

    uint16_t preamble = (sf <= 8) ? 32 : 16;
    uint16_t meshcore_sync_word = 0x1424;  // MeshCore = 0x12, Meshtastic = 0x2B

    if (SX1262) {
        SX1262->config_lora_params(
            freq, bw_enum, 140 /*current limit mA*/, LORA_TX_POWER_DEFAULT,
            sf_enum, cr_enum,
            Cpp_Bus_Driver::Sx126x::Lora_Crc_Type::ON,
            preamble, meshcore_sync_word
        );
    }

    radio_driver.setParams(freq, bw, sf, cr);

    printf("radio_set_params() — freq=%.3f bw=%.1f sf=%u cr=4/%u sync=0x%04X\n",
           (double)freq, (double)bw, (unsigned)sf, (unsigned)cr,
           (unsigned)meshcore_sync_word);
}


// ============================================================
// radio_set_tx_power()
// ============================================================

void radio_set_tx_power(uint8_t dbm) {
    if (dbm > 22) dbm = 22;
    printf("radio_set_tx_power() — %u dBm\n", (unsigned)dbm);
    // Applied via config_lora_params on next radio_set_params() call.
}


// ============================================================
// radio_get_rng_seed() — ESP32-P4 hardware TRNG
// ============================================================

uint32_t radio_get_rng_seed() {
    return esp_random();
}

// ============================================================================
// Deferred radio reconfig (avoids SPI race between LVGL task and meck_task)
// ============================================================================
static volatile bool    g_radio_reconfig_pending = false;
static volatile float   g_pending_freq           = 0;
static volatile float   g_pending_bw             = 0;
static volatile uint8_t g_pending_sf             = 0;
static volatile uint8_t g_pending_cr             = 0;
static volatile uint8_t g_pending_tx_power       = 0;

extern "C" void radio_request_reconfig(float freq, float bw, uint8_t sf, uint8_t cr, uint8_t tx_power) {
    g_pending_freq     = freq;
    g_pending_bw       = bw;
    g_pending_sf       = sf;
    g_pending_cr       = cr;
    g_pending_tx_power = tx_power;
    g_radio_reconfig_pending = true;
    printf("radio_request_reconfig: queued freq=%.3f bw=%.0f sf=%d cr=4/%d tx=%d\n",
           freq, bw, (int)sf, (int)cr, (int)tx_power);
}

extern "C" void radio_apply_pending_reconfig() {
    if (!g_radio_reconfig_pending) return;
    g_radio_reconfig_pending = false;

    float   freq = g_pending_freq;
    float   bw   = g_pending_bw;
    uint8_t sf   = g_pending_sf;
    uint8_t cr   = g_pending_cr;
    uint8_t tx   = g_pending_tx_power;

    printf("radio_apply_pending_reconfig: applying queued config\n");
    radio_set_params(freq, bw, sf, cr);
    radio_set_tx_power(tx);
    printf("radio_apply_pending_reconfig: done\n");
}

// ============================================================================
// SKY13453 antenna selection
// VCTL HIGH = antenna A (internal/PCB)
// VCTL LOW  = antenna B (external/SMA)
// LilyGo's main.cpp configures the pin as OUTPUT but never writes a value,
// so without this the state at boot is undefined and TX may go to the wrong
// port. Standalone sx1262_lora_send_receive example sets it HIGH, so we do
// the same.
// ============================================================================
extern "C" void meck_set_antenna_default() {
    extern std::unique_ptr<Cpp_Bus_Driver::Xl95x5> XL9535;
    if (XL9535) {
        XL9535->pin_write(XL9535_SKY13453_VCTL, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
        printf("meck_set_antenna_default: VCTL=HIGH (antenna A)\n");
    }
}

// ============================================================================
// Battery readout — wrappers around LilyGo's BQ27220 fuel gauge
// ----------------------------------------------------------------------------
// LilyGo's main.cpp constructs `auto BQ27220 = std::make_unique<...>(...)` at
// file scope. We extern it here so the meshcore component can read battery
// state. Voltage is always trustworthy; the chip's SoC% may drift if the
// cell isn't characterised in the BQ27220's data flash (a known issue
// across MeshCore P4 forks — see MeshOS issue #2192).
//
// LilyGo's example firmware and wiki tell users to set Design Capacity to
// 1000 mAh, which is wrong — the physical cell shipped on the T-Display P4
// is 2000 mAh. Our main.cpp now writes 2000 at boot via
// BQ27220->set_design_capacity(2000), so the chip's own current/SoC/time
// readings should now scale correctly. We still treat full-charge mAh as
// a fixed constant here to keep the UI's "Remaining mAh" consistent with
// our voltage-curve-derived percentage, regardless of any drift in the
// chip's learned full-charge value.
// ============================================================================

extern std::unique_ptr<Cpp_Bus_Driver::Bq27220xxxx> BQ27220;

// Actual physical cell capacity. Confirmed via reports from a T-Display P4
// owner (Tony Podlaski, December 2025) and consistent with LilyGo's own
// 2000 mAh advertised spec. Used both in main.cpp's set_design_capacity
// call (so the BQ27220's reported current/SoC scale correctly) and here
// for "Remaining mAh" derivation in the UI.
static const uint16_t MECK_BATTERY_CAP_MAH = 2000;

extern "C" bool meck_battery_available() {
    return (bool)BQ27220;
}

extern "C" uint16_t meck_battery_voltage_mv() {
    if (!BQ27220) return 0;
    return BQ27220->get_voltage();
}

extern "C" int16_t meck_battery_current_ma() {
    if (!BQ27220) return 0;
    return BQ27220->get_current();
}

extern "C" uint8_t meck_battery_pct_from_chip() {
    if (!BQ27220) return 0;
    uint16_t soc = BQ27220->get_status_of_charge();
    if (soc > 100) soc = 100;  // clamp; chip can momentarily report >100
    return (uint8_t)soc;
}

extern "C" int8_t meck_battery_temp_c() {
    if (!BQ27220) return 0;
    // BQ27220 die (chip) temperature, NOT battery temperature. The
    // T-Display P4 schematic wires the cell's NTC thermistor to the
    // LGS4056H charge IC for over-temperature protection during charging
    // (see LilyGo's GitHub README), not to the BQ27220's TS pin. Even
    // with EXTERNAL_NTC mode set in main.cpp, get_temperature_celsius()
    // would read whatever floats on the disconnected TS pin — typically
    // near the floor of the gauge's range and meaningless.
    //
    // The die temperature is at least real and tracks roughly with
    // ambient + load. Expect ~35-45°C with screen on / LoRa active /
    // GPS running, dropping toward ambient as the device idles. This is
    // surfaced in the UI as "Chip temp" so the user isn't misled into
    // thinking the cell itself is at this temperature.
    return (int8_t)BQ27220->get_chip_temperature_celsius();
}

// Forward decl for use below
extern "C" uint8_t meck_battery_pct_from_voltage(uint16_t mv);

extern "C" uint16_t meck_battery_remaining_mah() {
    if (!BQ27220) return 0;
    // Derive from voltage so this stays consistent with the displayed
    // voltage curve % even when the chip's own SoC drifts.
    uint16_t mv = BQ27220->get_voltage();
    uint8_t pct = meck_battery_pct_from_voltage(mv);
    return (uint16_t)(((uint32_t)MECK_BATTERY_CAP_MAH * pct) / 100);
}

extern "C" uint16_t meck_battery_full_charge_mah() {
    // Always report the actual cell capacity, not what the chip thinks.
    return MECK_BATTERY_CAP_MAH;
}

extern "C" uint16_t meck_battery_time_to_empty_min() {
    if (!BQ27220) return 0;
    return BQ27220->get_time_to_empty();
}

// Voltage-to-SoC for a single Li-ion cell at rest. Always trustworthy
// regardless of BQ27220 calibration state. Linear interpolation between
// well-known points on the discharge curve. Under load this will read
// slightly low; while charging slightly high — acceptable for a sanity
// check against the chip's reported SoC.
extern "C" uint8_t meck_battery_pct_from_voltage(uint16_t mv) {
    struct Point { uint16_t mv; uint8_t pct; };
    static const Point curve[] = {
        {3300,   0},
        {3500,   5},
        {3700,  20},
        {3850,  50},
        {4000,  85},
        {4200, 100},
    };
    const int N = sizeof(curve) / sizeof(curve[0]);
    if (mv <= curve[0].mv)     return 0;
    if (mv >= curve[N-1].mv)   return 100;
    for (int i = 1; i < N; i++) {
        if (mv < curve[i].mv) {
            uint16_t mv0 = curve[i-1].mv;
            uint16_t mv1 = curve[i].mv;
            uint8_t  p0  = curve[i-1].pct;
            uint8_t  p1  = curve[i].pct;
            return p0 + (uint8_t)(((uint32_t)(mv - mv0) * (p1 - p0)) / (mv1 - mv0));
        }
    }
    return 100;
}

// ============================================================================
// SKY13453 antenna selection
// VCTL HIGH = RF1 (internal/PCB antenna, LilyGo factory default)
// VCTL LOW  = RF2 (external/SMA)
// LilyGo configures the pin as OUTPUT but never writes a value, so without
// this the boot state is undefined and TX may go to a disconnected port.
// Will become a Settings toggle in a follow-up turn.
// ============================================================================
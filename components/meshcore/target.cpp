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
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
// Boot button (ESP32-P4 strapping pin, direct GPIO)
// ----------------------------------------------------------------------------
// PIN_BOOT_BTN (GPIO 35) is the BOOT-0 strapping pin used to enter download
// mode at reset. The LilyGo board ties it to an external pull-up; the physical
// button shorts it to GND. So at runtime: level 1 = released, level 0 = pressed.
//
// We configure it as plain input with no internal pulls (the external pull-up
// is already there, and adding an internal pull-down would fight it). No
// interrupt — the screen-idle timer polls at 250 ms while dimmed, which is
// plenty responsive for a wake button and avoids ISR plumbing.
// ============================================================================
extern "C" void meck_boot_button_init() {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << PIN_BOOT_BTN;
    cfg.mode         = GPIO_MODE_INPUT;
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;
    esp_err_t err = gpio_config(&cfg);
    if (err == ESP_OK) {
        printf("meck_boot_button_init: GPIO %d configured as input (active LOW)\n",
               (int)PIN_BOOT_BTN);
    } else {
        printf("meck_boot_button_init: gpio_config failed: %s\n", esp_err_to_name(err));
    }
}

extern "C" bool meck_boot_button_pressed() {
    return gpio_get_level((gpio_num_t)PIN_BOOT_BTN) == 0;
}

// ============================================================================
// Battery readout — wrappers around LilyGo's BQ27220 fuel gauge
// ----------------------------------------------------------------------------
// LilyGo's main.cpp constructs `auto BQ27220 = std::make_unique<...>(...)` at
// file scope. We extern it here so the meshcore component can read battery
// state.
//
// Cell capacity is 1000 mAh per LilyGo wiki FAQ 9.9 ("About inaccurate
// battery level display and inability to charge when powered off"):
//   https://wiki.lilygo.cc/get_started/en/Display/T-Display-P4/T-Display-P4.html
// The wiki instructs setting design_capacity to 1000 in firmware, then
// running one full charge → natural discharge to power-off → recharge
// cycle so the BQ27220 learns its real Full Charge Capacity (FCC). After
// that single calibration cycle, the gauge's internal coulomb counter is
// referenced against the correct cell capacity.
//
// (A previous comment block here claimed the cell was 2000 mAh based on
// an unverified third-party attribution. The wiki FAQ is the only source
// we've actually verified against LilyGo's own documentation, and it
// says 1000. Code now matches the wiki.)
//
// The BQ27220 has no hardware lock preventing FCC from learning a value
// above design capacity unless the FCC_LIMIT bit is set in the CEDV
// Gauging Configuration data-memory register. The Cpp_Bus_Driver wrapper
// doesn't expose that register, so we defend at the software accessor
// layer: meck_battery_full_charge_mah() caps at MECK_BATTERY_DESIGN_MAH,
// and meck_battery_pct_from_chip() recomputes SoC against min(FCC, design)
// rather than trusting the chip's get_status_of_charge() (which would be
// computed against the chip's possibly-stale internal FCC). Voltage and
// current readings are direct measurements and don't go through this
// path; they stay trustworthy regardless of FCC state.
// ============================================================================

extern std::unique_ptr<Cpp_Bus_Driver::Bq27220xxxx> BQ27220;

// Physical cell capacity per LilyGo wiki FAQ 9.9. Used as the upper bound
// for both the displayed FCC and the denominator in SoC calculations. The
// chip's learned FCC may be lower than this (aged cells degrade downward
// over their lifetime; that's normal and we honour it); it should not be
// higher (cell can't physically hold more than its design capacity, so a
// higher reading indicates stale or corrupted calibration state).
static const uint16_t MECK_BATTERY_DESIGN_MAH = 1000;

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
    // Recompute SoC ourselves rather than calling get_status_of_charge():
    // the chip computes SoC against its internal FCC, which can be stale
    // (still reflecting an earlier design_capacity value before the
    // calibration cycle has completed). RM (remaining capacity) is a
    // direct coulomb-counter measurement and is correct; clamping the
    // denominator at min(FCC, design) yields an accurate percentage even
    // when the chip's stored FCC is out of range.
    uint16_t rm  = BQ27220->get_remaining_capacity();
    uint16_t fcc = BQ27220->get_full_charge_capacity();
    if (fcc == 0) {
        // Gauge hasn't learned an FCC yet — fall back to design capacity.
        fcc = MECK_BATTERY_DESIGN_MAH;
    }
    uint16_t denom = (fcc < MECK_BATTERY_DESIGN_MAH) ? fcc
                                                     : MECK_BATTERY_DESIGN_MAH;
    if (denom == 0) return 0;  // belt and braces
    uint32_t pct = ((uint32_t)rm * 100) / denom;
    if (pct > 100) pct = 100;  // possible if RM > clamped denom
    return (uint8_t)pct;
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
    // The chip's RM (remaining capacity, from the coulomb counter) is a
    // direct measurement and is the authoritative source. We previously
    // derived this from the voltage curve to sidestep chip-FCC drift; now
    // that pct_from_chip clamps properly, we can return the real RM and
    // get accurate readings under load (voltage sags by ~100-200mV at
    // typical TX peaks, which would skew a voltage-curve-derived RM).
    uint16_t rm = BQ27220->get_remaining_capacity();
    // Defensive clamp — RM shouldn't exceed the cell's actual capacity,
    // but if the gauge is mid-recalibration after a stale FCC it can
    // briefly report values that don't make physical sense.
    if (rm > MECK_BATTERY_DESIGN_MAH) rm = MECK_BATTERY_DESIGN_MAH;
    return rm;
}

extern "C" uint16_t meck_battery_full_charge_mah() {
    if (!BQ27220) return MECK_BATTERY_DESIGN_MAH;
    // Cap at design capacity. The chip's learned FCC can legitimately be
    // below this (aged cells); it should not be above (would indicate
    // stale or corrupted calibration state). Reporting min() means:
    //   - fresh device or aged cell: report chip's learned value
    //   - stale FCC > design: report design, prompting the recalibration
    //     cycle described in LilyGo wiki FAQ 9.9
    uint16_t fcc = BQ27220->get_full_charge_capacity();
    if (fcc == 0)  return MECK_BATTERY_DESIGN_MAH;  // not yet learned
    if (fcc > MECK_BATTERY_DESIGN_MAH) {
        // Diagnostic — log once per boot so a stale FCC after firmware
        // upgrade is visible without spamming the serial console. The
        // static guard avoids repeating once the calibration cycle
        // completes and FCC drops back into range.
        static bool warned = false;
        if (!warned) {
            warned = true;
            printf("meck_battery: chip FCC=%u > design=%u, capping. "
                   "Run the LilyGo wiki FAQ 9.9 recalibration cycle "
                   "(full charge → discharge until power-off → full charge).\n",
                   (unsigned)fcc, (unsigned)MECK_BATTERY_DESIGN_MAH);
        }
        return MECK_BATTERY_DESIGN_MAH;
    }
    return fcc;
}

extern "C" uint16_t meck_battery_time_to_empty_min() {
    if (!BQ27220) return 0;
    // TTE is computed by the chip from RM and the rolling average
    // discharge current. Both are direct measurements (no dependence on
    // FCC), so TTE doesn't need the clamp.
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
// BQ27220 full TI calibration procedure (battery design capacity / FCC fix)
// ----------------------------------------------------------------------------
// Why this exists:
//
//   LilyGo's main.cpp calls BQ27220->set_design_capacity(1000) on every boot
//   via the Cpp_Bus_Driver wrapper. That wrapper writes the Design Capacity
//   data-memory cell but skips several steps the TI TRM (SLUUBD4) requires
//   for the change to actually take effect on FCC:
//
//     - No Design Energy write at 0x92A1 (FCC math uses both DC and DE)
//     - Exits CFG_UPDATE with 0x0092 instead of 0x0091 (no REINIT, so the
//       Impedance Track algorithm never re-runs against the new values)
//     - No Seal, no RESET
//     - No fix for Qmax (0x9106) or stored FCC reference (0x929D), which
//       the T-Deck Pro fleet found are the actual culprits when DC is set
//       correctly but the chip's reported FCC stays pinned at the factory
//       3000 mAh.
//
//   The visible symptom is meck_battery_full_charge_mah() logging
//   "chip FCC=3000 > design=1000, capping" on every boot, plus chip-side
//   SoC% and time-to-empty being computed against the wrong baseline.
//
// What this does:
//
//   Runs the full TI procedure proven on the T-Deck Pro fleet:
//     1. Read DC and FCC. If DC matches target and FCC is inside the band
//        [target-100, target+100], return immediately. Self-gated /
//        idempotent.
//     2. Unseal (0x0414, 0x3672) + Full Access (0xFFFF, 0xFFFF). Belt and
//        braces - harmless if the chip is already unsealed, which it is
//        after LilyGo's set_design_capacity returns.
//     3. Enter CFG_UPDATE (0x0090) and poll OperationStatus for the
//        CFGUPMODE bit (bit 10, mask 0x0400).
//     4. Write Design Capacity (if wrong), Design Energy, Qmax Cell 0,
//        and stored FCC reference via MAC differential checksum. Each
//        write is idempotent: bq_dm_write16 short-circuits when the
//        target value is already in place.
//     5. Exit CFG_UPDATE with REINIT (0x0091) - the wrapper's missing
//        piece. This triggers the Impedance Track algorithm to recompute
//        FCC against the new data-memory values.
//     6. SEAL (0x0030).
//     7. RESET (0x0041) with a 2-second settling delay. Forces the gauge
//        to fully reinitialise; without this RESET, IT can retain its
//        previously learned FCC until a full charge/discharge cycle
//        naturally triggers a relearn.
//
// I2C access:
//
//   BQ27220_IIC_Bus is the Hardware_Iic_1 wrapper LilyGo's main.cpp creates
//   for the BQ27220 device (bus shared with XL9535 via set_bus_handle, see
//   main.cpp:5742). Its public write/read/write_read methods give raw byte
//   transactions to address 0x55 - exactly what the MAC procedure needs,
//   on the existing bus handle. No parallel i2c driver instance, no fight
//   for the bus.
// ============================================================================

extern std::shared_ptr<Cpp_Bus_Driver::Hardware_Iic_1> BQ27220_IIC_Bus;

// Data-memory addresses (TI SLUUBD4 Table 1-10)
static constexpr uint16_t kBqAddrDesignCapacity = 0x929F;  // Gas Gauging
static constexpr uint16_t kBqAddrDesignEnergy   = 0x92A1;  // Gas Gauging
static constexpr uint16_t kBqAddrStoredFCC      = 0x929D;  // Gas Gauging
static constexpr uint16_t kBqAddrQmaxCell0      = 0x9106;  // IT Cfg

// Standard registers
static constexpr uint8_t kBqRegControl      = 0x00;
static constexpr uint8_t kBqRegOpStatus     = 0x3A;
static constexpr uint8_t kBqRegMACAddress   = 0x3E;
static constexpr uint8_t kBqRegMACDataStart = 0x40;  // through 0x5F
static constexpr uint8_t kBqRegMACDataSum   = 0x60;
static constexpr uint8_t kBqRegMACDataLen   = 0x61;

// Control subcommands
static constexpr uint16_t kBqSubUnseal1    = 0x0414;
static constexpr uint16_t kBqSubUnseal2    = 0x3672;
static constexpr uint16_t kBqSubFullAccess = 0xFFFF;
static constexpr uint16_t kBqSubEnterCfg   = 0x0090;
static constexpr uint16_t kBqSubExitReinit = 0x0091;
static constexpr uint16_t kBqSubExitOnly   = 0x0092;
static constexpr uint16_t kBqSubSeal       = 0x0030;
static constexpr uint16_t kBqSubReset      = 0x0041;

// OperationStatus CFGUPMODE bit
static constexpr uint16_t kBqOpStatusCfgUpdate = 0x0400;  // bit 10

// FCC acceptance band around design capacity.
static constexpr uint16_t kBqFccBand = 100;

// Nominal LiPo cell voltage used to compute target Design Energy from
// Design Capacity (DE_mWh = DC_mAh * 3.7).
static constexpr uint16_t kBqDesignMv = 3700;

// Read a single byte from a BQ27220 register.
static uint8_t bq_read8(uint8_t reg) {
    uint8_t data = 0;
    if (!BQ27220_IIC_Bus->write_read(&reg, 1, &data, 1)) return 0;
    return data;
}

// Read a 16-bit little-endian value from a BQ27220 register.
static uint16_t bq_read16(uint8_t reg) {
    uint8_t data[2] = {0};
    if (!BQ27220_IIC_Bus->write_read(&reg, 1, data, 2)) return 0;
    return ((uint16_t)data[1] << 8) | data[0];
}

// Write a 16-bit subcommand to the Control register (0x00). Little-endian.
static bool bq_write_control(uint16_t subcmd) {
    uint8_t buf[3] = {
        kBqRegControl,
        (uint8_t)(subcmd & 0xFF),
        (uint8_t)((subcmd >> 8) & 0xFF)
    };
    return BQ27220_IIC_Bus->write(buf, 3);
}

// Differential-checksum write of a 16-bit value to a data-memory address.
// Mirrors the writeDM16 lambda inside TDeckBoard::configureFuelGauge.
// Idempotent: reads the current value first and returns success without
// touching anything if it already matches new_val.
//
// Must be called while the chip is unsealed and in CFG_UPDATE mode.
// Data memory stores values big-endian (MSB at 0x40, LSB at 0x41).
static bool bq_dm_write16(uint16_t addr, uint16_t new_val) {
    // 1. Select the data-memory address by writing little-endian to 0x3E/0x3F.
    {
        uint8_t sel[3] = {
            kBqRegMACAddress,
            (uint8_t)(addr & 0xFF),
            (uint8_t)((addr >> 8) & 0xFF)
        };
        if (!BQ27220_IIC_Bus->write(sel, 3)) {
            printf("meck_battery_calibrate: [0x%04X] select fail\n",
                   (unsigned)addr);
            return false;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // 2. Read current MSB, LSB, checksum, length.
    uint8_t old_msb  = bq_read8(kBqRegMACDataStart);
    uint8_t old_lsb  = bq_read8(kBqRegMACDataStart + 1);
    uint8_t old_chk  = bq_read8(kBqRegMACDataSum);
    uint8_t data_len = bq_read8(kBqRegMACDataLen);
    uint16_t old_val = ((uint16_t)old_msb << 8) | old_lsb;

    if (old_val == new_val) {
        printf("meck_battery_calibrate: [0x%04X] already %u, skip\n",
               (unsigned)addr, (unsigned)new_val);
        return true;
    }

    // 3. Differential checksum (matches T-Deck Pro writeDM16 lambda).
    uint8_t new_msb = (new_val >> 8) & 0xFF;
    uint8_t new_lsb = new_val & 0xFF;
    uint8_t temp    = (uint8_t)(255 - old_chk - old_msb - old_lsb);
    uint8_t new_chk = (uint8_t)(255 - ((temp + new_msb + new_lsb) & 0xFF));

    printf("meck_battery_calibrate: [0x%04X] %u -> %u\n",
           (unsigned)addr, (unsigned)old_val, (unsigned)new_val);

    // 4. Write address + two data bytes in one transaction.
    {
        uint8_t wr[5] = {
            kBqRegMACAddress,
            (uint8_t)(addr & 0xFF),
            (uint8_t)((addr >> 8) & 0xFF),
            new_msb,
            new_lsb
        };
        if (!BQ27220_IIC_Bus->write(wr, 5)) {
            printf("meck_battery_calibrate: [0x%04X] data write fail\n",
                   (unsigned)addr);
            return false;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // 5. Write checksum and length. Both must land before the chip commits
    //    the new data-memory value.
    {
        uint8_t chk[3] = {kBqRegMACDataSum, new_chk, data_len};
        if (!BQ27220_IIC_Bus->write(chk, 3)) {
            printf("meck_battery_calibrate: [0x%04X] checksum write fail\n",
                   (unsigned)addr);
            return false;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    return true;
}

extern "C" void meck_battery_calibrate() {
    if (!BQ27220 || !BQ27220_IIC_Bus) {
        printf("meck_battery_calibrate: fuel gauge not available, skipping\n");
        return;
    }

    const uint16_t target_dc = MECK_BATTERY_DESIGN_MAH;
    const uint16_t target_de =
        (uint16_t)(((uint32_t)target_dc * kBqDesignMv) / 1000);

    uint16_t dc  = BQ27220->get_design_capacity();
    uint16_t fcc = BQ27220->get_full_charge_capacity();
    printf("meck_battery_calibrate: start DC=%u FCC=%u "
           "(target DC=%u DE=%u mWh)\n",
           (unsigned)dc, (unsigned)fcc,
           (unsigned)target_dc, (unsigned)target_de);

    // Self-gate: if DC matches and FCC is inside the band, nothing to do.
    const uint16_t fcc_lo =
        (target_dc > kBqFccBand) ? (uint16_t)(target_dc - kBqFccBand) : 0;
    const uint16_t fcc_hi = (uint16_t)(target_dc + kBqFccBand);
    if (dc == target_dc && fcc >= fcc_lo && fcc <= fcc_hi) {
        printf("meck_battery_calibrate: DC correct and FCC inside [%u..%u], "
               "no action\n",
               (unsigned)fcc_lo, (unsigned)fcc_hi);
        return;
    }

    printf("meck_battery_calibrate: remediation required\n");

    // Diagnostic: log starting OperationStatus and decode the SEC field
    // (bits [2:1] of the low byte): 11=Sealed, 10=Unsealed, 01=Full Access.
    // If we time out below this tells us whether the chip even reached
    // unsealed state.
    uint16_t op_start = bq_read16(kBqRegOpStatus);
    uint8_t  sec_start = (uint8_t)((op_start & 0x0006) >> 1);
    printf("meck_battery_calibrate: pre-unseal OperationStatus=0x%04X SEC=%u\n",
           (unsigned)op_start, (unsigned)sec_start);

    // Unseal + Full Access. Idempotent - safe even if the chip is already
    // unsealed after LilyGo's set_design_capacity call. Capture each write's
    // return value so we can tell if any I2C transaction NACKed.
    bool w_unseal1 = bq_write_control(kBqSubUnseal1);    vTaskDelay(pdMS_TO_TICKS(2));
    bool w_unseal2 = bq_write_control(kBqSubUnseal2);    vTaskDelay(pdMS_TO_TICKS(2));
    bool w_full1   = bq_write_control(kBqSubFullAccess); vTaskDelay(pdMS_TO_TICKS(2));
    bool w_full2   = bq_write_control(kBqSubFullAccess); vTaskDelay(pdMS_TO_TICKS(2));
    printf("meck_battery_calibrate: unseal writes ok=%d,%d full-access ok=%d,%d\n",
           w_unseal1, w_unseal2, w_full1, w_full2);

    uint16_t op_after_unseal = bq_read16(kBqRegOpStatus);
    uint8_t  sec_after_unseal = (uint8_t)((op_after_unseal & 0x0006) >> 1);
    printf("meck_battery_calibrate: post-unseal OperationStatus=0x%04X SEC=%u\n",
           (unsigned)op_after_unseal, (unsigned)sec_after_unseal);

    // Enter CFG_UPDATE and poll OperationStatus for CFGUPMODE bit (0x0400).
    // TRM says this can take up to 1 second; we allow 50 x 20 ms = 1000 ms.
    bool w_enter = bq_write_control(kBqSubEnterCfg);
    printf("meck_battery_calibrate: ENTER_CFG_UPDATE write ok=%d\n", w_enter);
    bool cfg_ready = false;
    uint16_t op_last = 0;
    for (int i = 0; i < 50; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        op_last = bq_read16(kBqRegOpStatus);
        if (op_last & kBqOpStatusCfgUpdate) {
            cfg_ready = true;
            printf("meck_battery_calibrate: CFG_UPDATE entered "
                   "(OperationStatus=0x%04X after %d ms)\n",
                   (unsigned)op_last, (i + 1) * 20);
            break;
        }
    }
    if (!cfg_ready) {
        uint8_t sec_last = (uint8_t)((op_last & 0x0006) >> 1);
        printf("meck_battery_calibrate: ERROR timeout entering CFG_UPDATE, "
               "last OperationStatus=0x%04X SEC=%u, aborting\n",
               (unsigned)op_last, (unsigned)sec_last);
        bq_write_control(kBqSubExitOnly);
        vTaskDelay(pdMS_TO_TICKS(10));
        bq_write_control(kBqSubSeal);
        return;
    }

    // Write all four data-memory cells. Each call is idempotent so if any
    // happens to already match, it just logs "skip" and moves on.
    if (dc != target_dc) {
        // LilyGo's wrapper either failed, or hasn't run yet. Write DC ourselves.
        bq_dm_write16(kBqAddrDesignCapacity, target_dc);
    }
    bq_dm_write16(kBqAddrDesignEnergy, target_de);
    bq_dm_write16(kBqAddrQmaxCell0,    target_dc);
    bq_dm_write16(kBqAddrStoredFCC,    target_dc);

    // Exit CFG_UPDATE with REINIT. This is the wrapper's missing piece -
    // 0x0091 forces IT to reinit with the new data-memory values, where the
    // wrapper's 0x0092 just exits CFG_UPDATE without telling IT to recompute.
    bq_write_control(kBqSubExitReinit);
    printf("meck_battery_calibrate: EXIT_CFG_UPDATE_REINIT sent, "
           "waiting 200 ms\n");
    vTaskDelay(pdMS_TO_TICKS(200));

    // SEAL before RESET so the gauge comes back up locked the way it shipped.
    bq_write_control(kBqSubSeal);
    vTaskDelay(pdMS_TO_TICKS(5));

    // RESET forces a full gauge reinit. Without this, IT can retain its
    // previously learned FCC (typically 3000 mAh from factory) until a real
    // charge/discharge cycle triggers a relearn. With it, FCC should
    // recompute immediately against the new DC/DE/Qmax. 2-second delay
    // matches T-Deck Pro; the gauge needs that long to stabilise before
    // reads are trustworthy.
    printf("meck_battery_calibrate: RESET (forces FCC recalculation)\n");
    bq_write_control(kBqSubReset);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Verify final state.
    uint16_t new_dc  = BQ27220->get_design_capacity();
    uint16_t new_fcc = BQ27220->get_full_charge_capacity();
    printf("meck_battery_calibrate: end DC=%u FCC=%u\n",
           (unsigned)new_dc, (unsigned)new_fcc);

    if (new_fcc > (uint16_t)(target_dc + kBqFccBand)) {
        // RESET didn't fix FCC. Typically resolves after one real
        // charge/discharge cycle. The software clamp in
        // meck_battery_full_charge_mah() still ensures correct display.
        printf("meck_battery_calibrate: WARNING FCC still stale at %u - "
               "software clamp active\n", (unsigned)new_fcc);
    }
}
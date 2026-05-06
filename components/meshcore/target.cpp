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
// SKY13453 antenna selection
// VCTL HIGH = RF1 (internal/PCB antenna, LilyGo factory default)
// VCTL LOW  = RF2 (external/SMA)
// LilyGo configures the pin as OUTPUT but never writes a value, so without
// this the boot state is undefined and TX may go to a disconnected port.
// Will become a Settings toggle in a follow-up turn.
// ============================================================================

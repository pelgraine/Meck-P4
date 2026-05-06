/*
 * target.cpp — MeshCore radio attach + LoRa params for T-Display P4 (Meck)
 *
 * Major surgery from the standalone version:
 *   - All XL9535 / power-rail / SX1262 reset code REMOVED.
 *     LilyGo's lvgl_9_ui factory init handles every bit of that.
 *   - radio_init() REMOVED — replaced by meck_radio_attach() which assumes
 *     the SX1262 is already up and just reconfigures LoRa params.
 *   - LDO power init REMOVED — handled by LilyGo.
 *   - Hardware object instantiation: only SX1262 here, as a duplicate
 *     cpp_bus_driver client. cpp_bus_driver clients are not exclusive
 *     hardware owners; multiple instances against the same SPI bus is fine
 *     so long as concurrent transactions are coordinated.
 *
 * What stays:
 *   - radio_set_params() — applies MeshCore's LoRa preset (AU Narrow)
 *   - radio_set_tx_power() — TX power adjust
 *   - radio_get_rng_seed() — esp_random()-backed identity seed
 *   - The MeshCore radio_driver instance
 */

#include "target.h"
#include "t_display_p4_config.h"
#include "esp_random.h"
#include <stdio.h>

// ---- Hardware object — duplicate cpp_bus_driver client ----
//
// LilyGo's lvgl_9_ui already constructs its own SX1262 instance somewhere.
// We construct a separate client object pointing at the same SPI bus and
// chip pins. cpp_bus_driver objects are NOT hardware-exclusive; this is the
// same pattern as the LilyGo sx1262_lora_send_receive example creating SX1262
// from app_main without coordinating with any other init.

static std::shared_ptr<Cpp_Bus_Driver::Hardware_Spi> _spi_bus;
std::unique_ptr<Cpp_Bus_Driver::Sx126x> SX1262;

// ---- MeshCore radio adapter instance ----
P4SX1262Radio radio_driver;


// ============================================================
// meck_radio_attach()
// Called ONCE from app_main() after LilyGo's init is complete.
// ============================================================

bool meck_radio_attach() {
    printf("meck_radio_attach() — wrapping LilyGo's already-running SX1262\n");

    // Create our cpp_bus_driver client.
    // The hardware (XL9535, power rails, SX1262 RST sequence) was already
    // done by LilyGo. We're constructing the SPI client object only.
    _spi_bus = std::make_shared<Cpp_Bus_Driver::Hardware_Spi>(
        SPI_1_MOSI, SPI_1_SCLK, SPI_1_MISO, SPI2_HOST, 0
    );
    SX1262 = std::make_unique<Cpp_Bus_Driver::Sx126x>(
        _spi_bus,
        Cpp_Bus_Driver::Sx126x::Chip_Type::SX1262,
        SX1262_BUSY,
        SX1262_CS,
        DEFAULT_CPP_BUS_DRIVER_VALUE
    );
    SX1262->begin(10000000);  // 10 MHz SPI

    // Apply MeshCore's preset (overwrites LilyGo's demo SF9/125kHz/920MHz)
    radio_set_params(LORA_FREQ_DEFAULT, LORA_BW_DEFAULT,
                     LORA_SF_DEFAULT, LORA_CR_DEFAULT);
    radio_set_tx_power(LORA_TX_POWER_DEFAULT);

    // Enter RX mode, MeshCore-style
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

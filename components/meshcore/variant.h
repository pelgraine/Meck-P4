/*
 * variant.h — T-Display P4 hardware variant definitions for MeshCore
 *
 * Maps T-Display P4 pin definitions to MeshCore naming conventions.
 * 
 * IMPORTANT: On the P4, many "pins" are actually routed through the XL9535
 * I/O expander (SX1262 RST, DIO1, SKY13453 RF switch, etc). These are 
 * accessed via cpp_bus_driver, not direct GPIO. The defines below document
 * the logical mapping; actual access uses the XL9535 pin enums from
 * t_display_p4_config.h.
 *
 * Pin reference: components/private_library/t_display_p4_config.h
 * Hardware reference: Homertrix meshtastic-tdisplay-p4/src/main.cpp
 */

#pragma once

// ---- Board identification ----
#define MECK_P4                    1
#define LilyGo_TDisplay_P4        1
#define BOARD_NAME                 "T-Display-P4"

// ---- LoRa radio (SX1262 via SPI) ----
// Direct GPIO pins for SPI bus
#define P_LORA_SCLK                2    // SPI_1_SCLK
#define P_LORA_MOSI                3    // SPI_1_MOSI
#define P_LORA_MISO                4    // SPI_1_MISO
#define P_LORA_NSS                 24   // SX1262_CS (direct GPIO)
#define P_LORA_BUSY                6    // SX1262_BUSY (direct GPIO)

// These are on XL9535 I/O expander — NOT direct GPIO!
// Handled in P4SX1262Radio via cpp_bus_driver, not RadioLib Module()
// Defined here for documentation only:
// SX1262_RST  → XL9535 IO16
// SX1262_DIO1 → XL9535 IO17
// SKY13453_VCTL (RF switch) → XL9535 IO1

// ---- GPS (L76K via UART) ----
#define HAS_GPS                    1
#define PIN_GPS_TX                 22   // GPS_TX
#define PIN_GPS_RX                 23   // GPS_RX
// GPS wakeup is on XL9535 IO11

// ---- I2C buses ----
#define I2C_SDA_1                  7    // IIC_1: XL9535, PCF8563, BQ27220, touch
#define I2C_SCL_1                  8
#define I2C_SDA_2                  20   // IIC_2: ES8311, AW86224, ICM20948, camera
#define I2C_SCL_2                  21

// ---- I2C addresses ----
#define ADDR_XL9535                0x20
#define ADDR_ES8311                0x18
#define ADDR_AW86224               0x58
#define ADDR_PCF8563               0x51
#define ADDR_BQ27220               0x55
#define ADDR_ICM20948              0x68

// ---- Display ----
// MIPI DSI — init handled by SDK's screen driver, not direct pin control
// SCREEN_WIDTH and SCREEN_HEIGHT are defined by t_display_p4_driver.h
// (included via the SDK). Do NOT redefine them here.
#if defined(CONFIG_SCREEN_TYPE_HI8561)
  #define HAS_BACKLIGHT_PWM        1
  #define PIN_BACKLIGHT            51   // HI8561_SCREEN_BL — direct GPIO PWM
#elif defined(CONFIG_SCREEN_TYPE_RM69A10)
  #define HAS_BACKLIGHT_PWM        0   // AMOLED brightness via MIPI command
#else
  #define HAS_BACKLIGHT_PWM        1
  #define PIN_BACKLIGHT            51
#endif

// ---- Audio (ES8311 + NS4150B) ----
#define HAS_AUDIO                  1
#define PIN_I2S_BCLK               12   // ES8311_BCLK
#define PIN_I2S_MCLK               13   // ES8311_MCLK
#define PIN_I2S_LRCK               9    // ES8311_WS_LRCK
#define PIN_I2S_DOUT               10   // ES8311_DAC_DATA (speaker)
#define PIN_I2S_DIN                11   // ES8311_ADC_DATA (microphone)

// ---- SD card (SDMMC via SDIO_1) ----
#define HAS_SD                     1
#define PIN_SD_CLK                 43
#define PIN_SD_CMD                 44
#define PIN_SD_D0                  39
#define PIN_SD_D1                  40
#define PIN_SD_D2                  41
#define PIN_SD_D3                  42
// SD power enable is on XL9535 IO15

// ---- ESP32-C6 companion (SDIO_2) ----
// WiFi and BLE routed through C6 via SDIO
#define PIN_C6_SDIO_CLK            18
#define PIN_C6_SDIO_CMD            19
#define PIN_C6_SDIO_D0             14
#define PIN_C6_SDIO_D1             15
#define PIN_C6_SDIO_D2             16
#define PIN_C6_SDIO_D3             17
// C6 enable is on XL9535 IO14
// C6 wakeup is on XL9535 IO13

// ---- Boot button ----
#define PIN_BOOT_BTN               35   // ESP32P4_BOOT

// ---- MeshCore configuration ----
// No BLE on P4 main chip — companion-only via WiFi or USB
#define MAX_CONTACTS               1500  // 32MB PSRAM allows generous limits
#define MAX_GROUP_CHANNELS         8

// LoRa defaults (AU_915 for Sydney)
#define LORA_FREQ_DEFAULT          916.575
#define LORA_BW_DEFAULT            62.5
#define LORA_SF_DEFAULT            7
#define LORA_CR_DEFAULT            8
#define LORA_TX_POWER_DEFAULT      22
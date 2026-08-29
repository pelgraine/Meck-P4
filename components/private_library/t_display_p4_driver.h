/*
 * @Description: t_display_p4_driver
 * @Author: LILYGO_L
 * @Date: 2025-07-07 14:31:51
 * @LastEditTime: 2025-09-01 16:40:08
 * @License: GPL 3.0
 */
#pragma once

#include "t_display_p4_config.h"
#include "hi8561_driver.h"
#include "rm69a10_driver.h"

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
#define SCREEN_ROTATION_DIRECTION_0
#elif defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
#define SCREEN_ROTATION_DIRECTION_90
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

#if defined CONFIG_SCREEN_PIXEL_FORMAT_RGB565
#define SCREEN_BITS_PER_PIXEL 16
#define CAMERA_BITS_PER_PIXEL 16
#define SCREEN_COLOR_RGB_PIXEL_FORMAT LCD_COLOR_PIXEL_FORMAT_RGB565
#define CAMERA_COLOR_RGB_PIXEL_FORMAT LCD_COLOR_PIXEL_FORMAT_RGB565
#define LVGL_COLOR_FORMAT LV_COLOR_FORMAT_RGB565
#elif defined CONFIG_SCREEN_PIXEL_FORMAT_RGB888
#define SCREEN_BITS_PER_PIXEL 24
#define SCREEN_COLOR_RGB_PIXEL_FORMAT LCD_COLOR_PIXEL_FORMAT_RGB888
#define CAMERA_BITS_PER_PIXEL 24
#define CAMERA_COLOR_RGB_PIXEL_FORMAT LCD_COLOR_PIXEL_FORMAT_RGB888
#define LVGL_COLOR_FORMAT LV_COLOR_FORMAT_RGB888
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

// SCREEN
#if defined CONFIG_SCREEN_TYPE_HI8561
#define SCREEN_WIDTH HI8561_SCREEN_WIDTH
#define SCREEN_HEIGHT HI8561_SCREEN_HEIGHT
#define SCREEN_MIPI_DSI_DPI_CLK_MHZ HI8561_SCREEN_MIPI_DSI_DPI_CLK_MHZ
#define SCREEN_MIPI_DSI_HSYNC HI8561_SCREEN_MIPI_DSI_HSYNC
#define SCREEN_MIPI_DSI_HBP HI8561_SCREEN_MIPI_DSI_HBP
#define SCREEN_MIPI_DSI_HFP HI8561_SCREEN_MIPI_DSI_HFP
#define SCREEN_MIPI_DSI_VSYNC HI8561_SCREEN_MIPI_DSI_VSYNC
#define SCREEN_MIPI_DSI_VBP HI8561_SCREEN_MIPI_DSI_VBP
#define SCREEN_MIPI_DSI_VFP HI8561_SCREEN_MIPI_DSI_VFP
#define SCREEN_DATA_LANE_NUM HI8561_SCREEN_DATA_LANE_NUM
#define SCREEN_LANE_BIT_RATE_MBPS HI8561_SCREEN_LANE_BIT_RATE_MBPS

#elif defined CONFIG_SCREEN_TYPE_RM69A10
#define SCREEN_WIDTH RM69A10_SCREEN_WIDTH
#define SCREEN_HEIGHT RM69A10_SCREEN_HEIGHT
#define SCREEN_MIPI_DSI_DPI_CLK_MHZ RM69A10_SCREEN_MIPI_DSI_DPI_CLK_MHZ
#define SCREEN_MIPI_DSI_HSYNC RM69A10_SCREEN_MIPI_DSI_HSYNC
#define SCREEN_MIPI_DSI_HBP RM69A10_SCREEN_MIPI_DSI_HBP
#define SCREEN_MIPI_DSI_HFP RM69A10_SCREEN_MIPI_DSI_HFP
#define SCREEN_MIPI_DSI_VSYNC RM69A10_SCREEN_MIPI_DSI_VSYNC
#define SCREEN_MIPI_DSI_VBP RM69A10_SCREEN_MIPI_DSI_VBP
#define SCREEN_MIPI_DSI_VFP RM69A10_SCREEN_MIPI_DSI_VFP
#define SCREEN_DATA_LANE_NUM RM69A10_SCREEN_DATA_LANE_NUM
#define SCREEN_LANE_BIT_RATE_MBPS RM69A10_SCREEN_LANE_BIT_RATE_MBPS

#else
#error "unknown macro definition, please select the correct macro definition."
#endif

#if defined SCREEN_ROTATION_DIRECTION_0

#define LV_DISPLAY_ROTATION LV_DISPLAY_ROTATION_0

#elif defined SCREEN_ROTATION_DIRECTION_90

#define LV_DISPLAY_ROTATION LV_DISPLAY_ROTATION_90

#elif defined SCREEN_ROTATION_DIRECTION_180

#define LV_DISPLAY_ROTATION LV_DISPLAY_ROTATION_180

#elif defined SCREEN_ROTATION_DIRECTION_270

#define LV_DISPLAY_ROTATION LV_DISPLAY_ROTATION_270

#endif

bool Mipi_Dsi_Init(uint8_t num_data_lanes, uint32_t lane_bit_rate_mbps, uint32_t dpi_clock_freq_mhz, lcd_color_rgb_pixel_format_t color_rgb_pixel_format, uint8_t num_fbs, uint32_t width, uint32_t height,
                   uint32_t mipi_dsi_hsync, uint32_t mipi_dsi_hbp, uint32_t mipi_dsi_hfp, uint32_t mipi_dsi_vsync, uint32_t mipi_dsi_vbp, uint32_t mipi_dsi_vfp,
                   uint32_t bits_per_pixel, esp_lcd_panel_handle_t *mipi_dpi_panel,
                   esp_lcd_dsi_bus_handle_t *out_dsi_bus = nullptr);

bool Screen_Init(esp_lcd_panel_handle_t *mipi_dpi_panel);

bool Camera_Init(esp_lcd_panel_handle_t *mipi_dpi_panel);

bool Init_Ldo_Channel_Power(uint8_t chan_id, uint32_t voltage_mv);

// Meck v0.3.6: accessor for the screen's MIPI-DSI bus handle. Needed for
// the light-sleep screen-off path in main.cpp, which has to call
// esp_lcd_del_dsi_bus(bus) after esp_lcd_panel_del(panel) to release the
// dsi_phy NO_LIGHT_SLEEP PM lock. The bus is created inside Mipi_Dsi_Init
// as a local; the screen-only call site (Screen_Init) now captures it
// into a file-scope static for later retrieval. Camera_Init does NOT
// capture, so opening the camera UI won't clobber this handle.
//
// Returns NULL if Screen_Init has not yet been called, or if the bus has
// been deleted (call sites in meck_screen_off are responsible for
// ensuring the screen is in a state where this handle is valid).
esp_lcd_dsi_bus_handle_t Screen_Get_Mipi_Dsi_Bus_Handle();

// Meck: recreate the DPI panel on the ALREADY-LIVE DSI bus, reusing the bus
// and DBI io captured by Screen_Init. Used by the screen-off power path:
// meck_screen_off() deletes the DPI panel (esp_lcd_panel_del) to release the
// dsi_dpi CPU_FREQ_MAX PM lock so the CPU can drop to its DFS minimum while
// the panel self-refreshes the last frame from GRAM; meck_screen_on() calls
// this to rebuild the panel on wake. This never creates a second DSI bus and
// never touches the bus teardown that hung the P4. Returns false if
// Screen_Init has not run (no live bus/io) or panel creation fails.
bool Screen_Rebuild_Panel(esp_lcd_panel_handle_t *mipi_dpi_panel);
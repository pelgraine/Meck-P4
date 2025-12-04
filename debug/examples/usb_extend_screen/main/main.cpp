/*
 * @Description: usb_extend_screen
 * @Author: LILYGO_L
 * @Date: 2025-10-17 13:37:32
 * @LastEditTime: 2025-10-23 11:38:41
 * @License: GPL 3.0
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "t_display_p4_config.h"
#include "t_display_p4_driver.h"
#include "cpp_bus_driver_library.h"
#include "app_usb.h"
#include "app_lcd.h"
#if CONFIG_HID_TOUCH_ENABLE
#include "app_touch.h"
#endif

#define MCLK_MULTIPLE i2s_mclk_multiple_t::I2S_MCLK_MULTIPLE_256

esp_lcd_panel_handle_t Screen_Mipi_Dpi_Panel = NULL;

auto XL9535_IIC_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(XL9535_SDA, XL9535_SCL, I2C_NUM_0);
auto ES8311_IIC_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(ES8311_SDA, ES8311_SCL, I2C_NUM_1);

auto ES8311_IIS_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iis>(ES8311_ADC_DATA, ES8311_DAC_DATA, ES8311_WS_LRCK, ES8311_BCLK, ES8311_MCLK);

auto XL9535 = std::make_unique<Cpp_Bus_Driver::Xl95x5>(XL9535_IIC_Bus, XL9535_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);
auto ES8311 = std::make_unique<Cpp_Bus_Driver::Es8311>(ES8311_IIC_Bus, ES8311_IIS_Bus, ES8311_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);

#if defined CONFIG_SCREEN_TYPE_HI8561
auto ESP32P4 = std::make_unique<Cpp_Bus_Driver::Tool>();

auto HI8561_T_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(HI8561_TOUCH_SDA, HI8561_TOUCH_SCL, I2C_NUM_0);

auto HI8561_T = std::make_unique<Cpp_Bus_Driver::Hi8561_Touch>(HI8561_T_Bus, HI8561_TOUCH_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);

#elif defined CONFIG_SCREEN_TYPE_RM69A10

auto GT9895_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(GT9895_TOUCH_SDA, GT9895_TOUCH_SCL, I2C_NUM_0);

auto GT9895 = std::make_unique<Cpp_Bus_Driver::Gt9895>(GT9895_Bus, GT9895_IIC_ADDRESS, GT9895_X_SCALE_FACTOR, GT9895_Y_SCALE_FACTOR,
                                                       DEFAULT_CPP_BUS_DRIVER_VALUE);

#else
#error "unknown macro definition, please select the correct macro definition."
#endif

void ES8311_Init(void)
{
    ES8311->begin(MCLK_MULTIPLE, CONFIG_UAC_SAMPLE_RATE, i2s_data_bit_width_t::I2S_DATA_BIT_WIDTH_16BIT);

    if (ES8311->begin(50000) == true)
    {
        printf("es8311 initialization success\n");
    }
    else
    {
        printf("es8311 initialization fail\n");
    }

    ES8311->set_master_clock_source(Cpp_Bus_Driver::Es8311::Clock_Source::ADC_DAC_MCLK);
    ES8311->set_clock(Cpp_Bus_Driver::Es8311::Clock_Source::ADC_DAC_MCLK, true);
    ES8311->set_clock(Cpp_Bus_Driver::Es8311::Clock_Source::ADC_DAC_BCLK, true);

    ES8311->set_clock_coeff(MCLK_MULTIPLE, CONFIG_UAC_SAMPLE_RATE);

    ES8311->set_serial_port_mode(Cpp_Bus_Driver::Es8311::Serial_Port_Mode::SLAVE);

    ES8311->set_sdp_data_bit_length(Cpp_Bus_Driver::Es8311::Sdp::ADC, Cpp_Bus_Driver::Es8311::Bits_Per_Sample::DATA_16BIT);
    ES8311->set_sdp_data_bit_length(Cpp_Bus_Driver::Es8311::Sdp::DAC, Cpp_Bus_Driver::Es8311::Bits_Per_Sample::DATA_16BIT);
    Cpp_Bus_Driver::Es8311::Power_Status ps =
        {
            .contorl =
                {
                    .analog_circuits = true,               // 开启模拟电路
                    .analog_bias_circuits = true,          // 开启模拟偏置电路
                    .analog_adc_bias_circuits = true,      // 开启模拟ADC偏置电路
                    .analog_adc_reference_circuits = true, // 开启模拟ADC参考电路
                    .analog_dac_reference_circuit = true,  // 开启模拟DAC参考电路
                    .internal_reference_circuits = false,  // 关闭内部参考电路
                },
            .vmid = Cpp_Bus_Driver::Es8311::Vmid::START_UP_VMID_NORMAL_SPEED_CHARGE,
        };
    ES8311->set_power_status(ps);
    ES8311->set_pga_power(true);
    ES8311->set_adc_power(true);
    ES8311->set_dac_power(true);
    ES8311->set_output_to_hp_drive(true);
    ES8311->set_adc_offset_freeze(Cpp_Bus_Driver::Es8311::Adc_Offset_Freeze::DYNAMIC_HPF);
    ES8311->set_adc_hpf_stage2_coeff(10);
    ES8311->set_dac_equalizer(false);

    ES8311->set_mic(Cpp_Bus_Driver::Es8311::Mic_Type::ANALOG_MIC, Cpp_Bus_Driver::Es8311::Mic_Input::MIC1P_1N);
    ES8311->set_adc_auto_volume_control(false);
    ES8311->set_adc_gain(Cpp_Bus_Driver::Es8311::Adc_Gain::GAIN_18DB);
    ES8311->set_adc_pga_gain(Cpp_Bus_Driver::Es8311::Adc_Pga_Gain::GAIN_30DB);

    ES8311->set_adc_volume(191);
    ES8311->set_dac_volume(200);

    // 将ADC的数据自动输出到DAC上
    // ES8311->set_adc_data_to_dac(true);
}

extern "C" void app_main(void)
{
    printf("Ciallo\n");
    XL9535->begin();

    XL9535->pin_mode(XL9535_ESP32P4_VCCA_POWER_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_mode(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_mode(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    // 开关3.3v电压时候必须先将GPS断电
    XL9535->pin_mode(XL9535_GPS_WAKE_UP, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(XL9535_GPS_WAKE_UP, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    // 开关3.3v电压时候必须先将ESP32C6断电
    XL9535->pin_mode(XL9535_ESP32C6_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(XL9535_ESP32C6_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);

    XL9535->pin_write(XL9535_ESP32P4_VCCA_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);

    XL9535->pin_write(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));

    XL9535->pin_write(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(10));

    XL9535->pin_mode(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));

#if defined CONFIG_SCREEN_TYPE_HI8561
    ESP32P4->create_pwm(HI8561_SCREEN_BL, ledc_channel_t::LEDC_CHANNEL_0, 2000);

#elif defined CONFIG_SCREEN_TYPE_RM69A10
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

    Init_Ldo_Channel_Power(3, 1830);

    vTaskDelay(pdMS_TO_TICKS(100));

    XL9535->pin_mode(XL9535_TOUCH_RST, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(XL9535_TOUCH_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_TOUCH_RST, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_TOUCH_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));

#if defined CONFIG_SCREEN_TYPE_HI8561
    ESP32P4->create_pwm(HI8561_SCREEN_BL, ledc_channel_t::LEDC_CHANNEL_0, 2000);

    HI8561_T_Bus->set_bus_handle(XL9535_IIC_Bus->get_bus_handle());

    HI8561_T->begin();

#elif defined CONFIG_SCREEN_TYPE_RM69A10

    GT9895_Bus->set_bus_handle(XL9535_IIC_Bus->get_bus_handle());

    GT9895->begin();

#else
#error "unknown macro definition, please select the correct macro definition."
#endif

    ES8311_Init();

    app_usb_init();
    app_lcd_init(&Screen_Mipi_Dpi_Panel);
#if CONFIG_HID_TOUCH_ENABLE
    app_touch_init();
#endif

#if defined CONFIG_LCD_PIXEL_FORMAT_RGB565
    // 设置整个屏幕为白色
    size_t screen_size = SCREEN_WIDTH * SCREEN_HEIGHT * 2; // RGB565: 2 bytes per pixel
    size_t data_cache_line_size = 16;                      // 通常16或32，具体可查芯片手册
    void *white_buf = heap_caps_aligned_calloc(data_cache_line_size, 1, screen_size, MALLOC_CAP_SPIRAM);
    if (white_buf)
    {
        uint16_t *p = (uint16_t *)white_buf;
        for (size_t i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; ++i)
        {
            p[i] = 0xFFFF; // RGB565白色
        }
        esp_err_t err = esp_lcd_panel_draw_bitmap(Screen_Mipi_Dpi_Panel, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, white_buf);
        if (err != ESP_OK)
        {
            printf("esp_lcd_panel_draw_bitmap (white) fail (error code: %#X)\n", err);
        }
        heap_caps_free(white_buf);
    }
#elif defined CONFIG_LCD_PIXEL_FORMAT_RGB888

    // 设置整个屏幕为白色
    size_t screen_size = SCREEN_WIDTH * SCREEN_HEIGHT * 3; // RGB888: 3 bytes per pixel
    size_t data_cache_line_size = 16;                      // 通常16或32，具体可查芯片手册
    void *white_buf = heap_caps_aligned_calloc(data_cache_line_size, 1, screen_size, MALLOC_CAP_SPIRAM);
    if (white_buf)
    {
        uint8_t *p = (uint8_t *)white_buf;
        for (size_t i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; ++i)
        {
            p[i * 3 + 0] = 0xFF; // R
            p[i * 3 + 1] = 0xFF; // G
            p[i * 3 + 2] = 0xFF; // B
        }
        esp_err_t err = esp_lcd_panel_draw_bitmap(Screen_Mipi_Dpi_Panel, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, white_buf);
        if (err != ESP_OK)
        {
            printf("esp_lcd_panel_draw_bitmap (white) fail (error code: %#X)\n", err);
        }
        heap_caps_free(white_buf);
    }

#else
#error "unknown macro definition, please select the correct macro definition."
#endif

#if defined CONFIG_SCREEN_TYPE_HI8561
    HI8561_T->start_pwm_gradient_time(100, 500);
#elif defined CONFIG_SCREEN_TYPE_RM69A10
    for (uint8_t i = 0; i < 255; i += 5)
    {
        set_rm69a10_brightness(Screen_Mipi_Dpi_Panel, i);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* Wi-Fi iperf Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <errno.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_console.h"
#include "cmd_system.h"

extern "C"
{
/* component manager */
#include "iperf.h"
#include "wifi_cmd.h"
#include "iperf_cmd.h"
#include "ping_cmd.h"
}

#include "cpp_bus_driver_library.h"

#if CONFIG_ESP_WIFI_ENABLE_WIFI_TX_STATS || CONFIG_ESP_WIFI_ENABLE_WIFI_RX_STATS
#include "esp_wifi_he.h"
#endif
#if CONFIG_ESP_WIFI_ENABLE_WIFI_TX_STATS
extern int wifi_cmd_get_tx_statistics(int argc, char **argv);
extern int wifi_cmd_clr_tx_statistics(int argc, char **argv);
#endif
#if CONFIG_ESP_WIFI_ENABLE_WIFI_RX_STATS
extern int wifi_cmd_get_rx_statistics(int argc, char **argv);
extern int wifi_cmd_clr_rx_statistics(int argc, char **argv);
#endif

#if defined(CONFIG_ESP_EXT_CONN_ENABLE) && defined(CONFIG_ESP_HOST_WIFI_ENABLED)
#include "esp_extconn.h"
#endif

auto IIC_Bus_2 = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(7, 8, I2C_NUM_0);

auto XL9535 = std::make_unique<Cpp_Bus_Driver::Xl95x5>(IIC_Bus_2, 0x20, DEFAULT_CPP_BUS_DRIVER_VALUE);

void iperf_hook_show_wifi_stats(iperf_traffic_type_t type, iperf_status_t status)
{
    if (status == IPERF_STARTED)
    {
#if CONFIG_ESP_WIFI_ENABLE_WIFI_TX_STATS
        if (type != IPERF_UDP_SERVER)
        {
            wifi_cmd_clr_tx_statistics(0, NULL);
        }
#endif
#if CONFIG_ESP_WIFI_ENABLE_WIFI_RX_STATS
        if (type != IPERF_UDP_CLIENT)
        {
            wifi_cmd_clr_rx_statistics(0, NULL);
        }
#endif
    }

    if (status == IPERF_STOPPED)
    {
#if CONFIG_ESP_WIFI_ENABLE_WIFI_TX_STATS
        if (type != IPERF_UDP_SERVER)
        {
            wifi_cmd_get_tx_statistics(0, NULL);
        }
#endif
#if CONFIG_ESP_WIFI_ENABLE_WIFI_RX_STATS
        if (type != IPERF_UDP_CLIENT)
        {
            wifi_cmd_get_rx_statistics(0, NULL);
        }
#endif
    }
}

extern "C" void app_main(void)
{
    printf("Ciallo\n");

    XL9535->begin();
    XL9535->pin_mode(Cpp_Bus_Driver::Xl95x5::Pin::IO6, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_mode(Cpp_Bus_Driver::Xl95x5::Pin::IO0, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);

    XL9535->pin_write(Cpp_Bus_Driver::Xl95x5::Pin::IO6, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    XL9535->pin_write(Cpp_Bus_Driver::Xl95x5::Pin::IO0, Cpp_Bus_Driver::Xl95x5::Value::LOW);

    // ESP32C6复位
    XL9535->pin_mode(Cpp_Bus_Driver::Xl95x5::Pin::IO14, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(Cpp_Bus_Driver::Xl95x5::Pin::IO14, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
    XL9535->pin_write(Cpp_Bus_Driver::Xl95x5::Pin::IO14, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(100));
    XL9535->pin_write(Cpp_Bus_Driver::Xl95x5::Pin::IO14, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(1000));

#if defined(CONFIG_ESP_EXT_CONN_ENABLE) && defined(CONFIG_ESP_HOST_WIFI_ENABLED)
    esp_extconn_config_t ext_config = ESP_EXTCONN_CONFIG_DEFAULT();
    esp_extconn_init(&ext_config);
#endif

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* initialise wifi */
    app_wifi_initialise_config_t config = APP_WIFI_CONFIG_DEFAULT();
    config.storage = WIFI_STORAGE_RAM;
    config.ps_type = WIFI_PS_NONE;
    app_initialise_wifi(&config);
#if CONFIG_ESP_WIFI_ENABLE_WIFI_RX_STATS
#if CONFIG_ESP_WIFI_ENABLE_WIFI_RX_MU_STATS
    esp_wifi_enable_rx_statistics(true, true);
#else
    esp_wifi_enable_rx_statistics(true, false);
#endif
#endif
#if CONFIG_ESP_WIFI_ENABLE_WIFI_TX_STATS
    esp_wifi_enable_tx_statistics(ESP_WIFI_ACI_BE, true);
#endif

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "iperf>";

    // init console REPL environment
#if CONFIG_ESP_CONSOLE_UART
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
#endif

    /* Register commands */
    register_system();
    app_register_all_wifi_commands();
    app_register_iperf_commands();
    ping_cmd_register_ping();
    app_register_iperf_hook_func(iperf_hook_show_wifi_stats);

    printf("\n ==================================================\n");
    printf(" |       Steps to test WiFi throughput            |\n");
    printf(" |                                                |\n");
    printf(" |  1. Print 'help' to gain overview of commands  |\n");
    printf(" |  2. Configure device to station or soft-AP     |\n");
    printf(" |  3. Setup WiFi connection                      |\n");
    printf(" |  4. Run iperf to test UDP/TCP RX/TX throughput |\n");
    printf(" |                                                |\n");
    printf(" =================================================\n\n");

    // start console REPL
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

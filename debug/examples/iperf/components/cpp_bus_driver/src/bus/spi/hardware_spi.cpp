/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2025-02-13 15:04:49
 * @LastEditTime: 2025-10-14 15:26:51
 * @License: GPL 3.0
 */
#include "hardware_spi.h"

namespace Cpp_Bus_Driver
{
#if defined DEVELOPMENT_FRAMEWORK_ESPIDF
    bool Hardware_Spi::begin(int32_t freq_hz, int32_t cs)
    {
        if ((_bus_init_flag == true) && (_device_init_flag == true))
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "hardware_spi has been initialized\n");
            return true;
        }

        if (freq_hz == DEFAULT_CPP_BUS_DRIVER_VALUE)
        {
            freq_hz = DEFAULT_CPP_BUS_DRIVER_SPI_FREQ_HZ;
        }

        if (_flags == DEFAULT_CPP_BUS_DRIVER_VALUE)
        {
            _flags = static_cast<uint32_t>(NULL);
        }

        assert_log(Log_Level::INFO, __FILE__, __LINE__, "hardware_spi config _mosi: %d\n", _mosi);
        assert_log(Log_Level::INFO, __FILE__, __LINE__, "hardware_spi config _sclk: %d\n", _sclk);
        assert_log(Log_Level::INFO, __FILE__, __LINE__, "hardware_spi config _miso: %d\n", _miso);
        assert_log(Log_Level::INFO, __FILE__, __LINE__, "hardware_spi config cs: %d\n", cs);
        assert_log(Log_Level::INFO, __FILE__, __LINE__, "hardware_spi config _port: %d\n", _port);
        assert_log(Log_Level::INFO, __FILE__, __LINE__, "hardware_spi config _mode: %d\n", _mode);
        assert_log(Log_Level::INFO, __FILE__, __LINE__, "hardware_spi config _clock_source: %d\n", _clock_source);
        assert_log(Log_Level::INFO, __FILE__, __LINE__, "hardware_spi config _flags: %d\n", _flags);
        assert_log(Log_Level::INFO, __FILE__, __LINE__, "hardware_spi config freq_hz: %d hz\n", freq_hz);

        if (_bus_init_flag == false)
        {
            const spi_bus_config_t bus_config =
                {
                    .mosi_io_num = _mosi,
                    .miso_io_num = _miso,
                    .sclk_io_num = _sclk,
                    .quadwp_io_num = -1, // WP引脚不设置，这个引脚配置Quad SPI的时候才有用
                    .quadhd_io_num = -1, // HD引脚不设置，这个引脚配置Quad SPI的时候才有用
                    .data4_io_num = -1,
                    .data5_io_num = -1,
                    .data6_io_num = -1,
                    .data7_io_num = -1,
                    .data_io_default_level = 0,
                    .max_transfer_sz = 0,
                    .flags = SPICOMMON_BUSFLAG_MASTER,
                    .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
                    .intr_flags = static_cast<uint32_t>(NULL),
                };

            esp_err_t assert = spi_bus_initialize(_port, &bus_config, SPI_DMA_CH_AUTO);
            if (assert != ESP_OK)
            {
                assert_log(Log_Level::BUS, __FILE__, __LINE__, "spi_bus_initialize fail (error code: %#X)\n", assert);
                return false;
            }

            _bus_init_flag = true;
        }

        if (_device_init_flag == false)
        {
            const spi_device_interface_config_t device_config =
                {
                    .command_bits = 0,
                    .address_bits = 0,
                    .dummy_bits = 0, // 无虚拟位
                    .mode = _mode,
                    .clock_source = _clock_source, // 时钟源
                    .duty_cycle_pos = 128,         // 50% 占空比
                    .cs_ena_pretrans = 1,          // 在数据传输开始之前，片选信号（CS）应该提前多少个SPI位周期被激活
                    .cs_ena_posttrans = 1,         // 在数据传输结束后，片选信号（CS）应该保持激活状态多少个SPI位周期
                    .clock_speed_hz = freq_hz,
                    .input_delay_ns = 0, // 无输入延迟
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
                    .sample_point = spi_sampling_point_t::SPI_SAMPLING_POINT_PHASE_0,
#endif
                    .spics_io_num = cs,
                    .flags = _flags, // 标志，可以填入SPI_DEVICE_BIT_LSBFIRST等信息
                    .queue_size = 1,
                    .pre_cb = NULL,  // 无传输前回调
                    .post_cb = NULL, // 无传输后回调
                };
            esp_err_t assert = spi_bus_add_device(_port, &device_config, &_spi_device);
            if (assert != ESP_OK)
            {
                assert_log(Log_Level::BUS, __FILE__, __LINE__, "spi_bus_add_device fail (error code: %#X)\n", assert);
                return false;
            }

            _device_init_flag = true;
        }

        _freq_hz = freq_hz;
        _cs = cs;

        return true;
    }

    bool Hardware_Spi::write(const void *data, size_t byte)
    {
        spi_transaction_t buffer =
            {
                .flags = static_cast<uint32_t>(NULL),
                .cmd = 0,
                .addr = 0,
                .length = byte * 8,
                .rxlength = 0,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
                .override_freq_hz = 0,
#endif
                .user = (void *)0,
                .tx_buffer = data,
                .rx_buffer = NULL,
            };

        esp_err_t assert = spi_device_polling_transmit(_spi_device, &buffer);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "spi_device_polling_transmit fail (error code: %#X)\n", assert);
            return false;
        }

        return true;
    }

    bool Hardware_Spi::read(void *data, size_t byte)
    {
        spi_transaction_t buffer =
            {
                .flags = static_cast<uint32_t>(NULL),
                .cmd = 0,
                .addr = 0,
                .length = byte * 8,
                .rxlength = 0,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
                .override_freq_hz = 0,
#endif
                .user = (void *)0,
                .tx_buffer = NULL,
                .rx_buffer = data,
            };

        esp_err_t assert = spi_device_polling_transmit(_spi_device, &buffer);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "spi_device_polling_transmit fail (error code: %#X)\n", assert);
            return false;
        }

        return true;
    }

    bool Hardware_Spi::write_read(const void *write_data, void *read_data, size_t data_byte)
    {
        spi_transaction_t buffer =
            {
                .flags = static_cast<uint32_t>(NULL),
                .cmd = 0,
                .addr = 0,
                .length = data_byte * 8,
                .rxlength = 0,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
                .override_freq_hz = 0,
#endif
                .user = (void *)0,
                .tx_buffer = write_data,
                .rx_buffer = read_data,
            };

        esp_err_t assert = spi_device_polling_transmit(_spi_device, &buffer);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "spi_device_polling_transmit fail (error code: %#X)\n", assert);
            return false;
        }

        return true;
    }
#endif
}
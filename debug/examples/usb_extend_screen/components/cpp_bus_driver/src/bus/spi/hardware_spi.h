/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2024-12-16 17:47:28
 * @LastEditTime: 2025-10-11 15:36:22
 * @License: GPL 3.0
 */
#pragma once

#include "../bus_guide.h"

namespace Cpp_Bus_Driver
{
#if defined DEVELOPMENT_FRAMEWORK_ESPIDF
    class Hardware_Spi : public Bus_Spi_Guide
    {
    public:
        int32_t _mosi, _sclk, _miso, _cs, _freq_hz;
        spi_host_device_t _port;
        uint8_t _mode;
        spi_clock_source_t _clock_source;
        uint32_t _flags;
        bool _bus_init_flag = false;
        bool _device_init_flag = false;

        spi_device_handle_t _spi_device;

        Hardware_Spi(int32_t mosi, int32_t sclk, int32_t miso = DEFAULT_CPP_BUS_DRIVER_VALUE,
                     spi_host_device_t port = SPI2_HOST, int8_t mode = 0, spi_clock_source_t clock_source = SPI_CLK_SRC_DEFAULT,
                     uint32_t flags = DEFAULT_CPP_BUS_DRIVER_VALUE)
            : _mosi(mosi), _sclk(sclk), _miso(miso), _port(port), _mode(mode), _clock_source(clock_source), _flags(flags)
        {
        }

        bool begin(int32_t freq_hz = DEFAULT_CPP_BUS_DRIVER_VALUE, int32_t cs = DEFAULT_CPP_BUS_DRIVER_VALUE) override;
        bool write(const void *data, size_t byte) override;
        bool read(void *data, size_t byte) override;
        bool write_read(const void *write_data, void *read_data, size_t data_byte) override;
    };
#endif
}

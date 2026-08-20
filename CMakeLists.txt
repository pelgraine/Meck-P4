# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(PROJECT_VERSION  "1.0.0")

project(t-display-p4)

# RadioLib's ESP-IDF HAL (EspHal.cpp) refuses to compile when the FreeRTOS
# tick is below 1000 Hz unless this is defined. Relaxing is safe here:
# EspHal::delay() is a hybrid sleep + busy-wait that is microsecond-accurate
# at any tick rate (the strict check exists for LoRaWAN RX-window timing,
# which Meck does not use), and the LR2021 driver's millisecond delays
# (1/5/10/50 ms) go through exactly that path. This keeps the project at
# the IDF-default 100 Hz tick instead of retuning LilyGo's entire firmware
# for one HAL. Applies to every build config, since EspHal.cpp is compiled
# into the RadioLib component unconditionally.
idf_component_get_property(radiolib_lib RadioLib COMPONENT_LIB)
target_compile_definitions(${radiolib_lib} PRIVATE RADIOLIB_RELAX_RTOS_TICK=1)
cmake_minimum_required(VERSION 3.8)
project(telemetrysystem C CXX ASM)

set(NRF_TARGET "nrf52")

set(ARM_NONE_EABI_TOOLCHAIN_PATH "/usr/bin")
set(NRF5_SDK_PATH "../nRF5_SDK_12.3.0_d7731ad")
set(OPENOCD "/usr/bin/openocd")

include("CMake_nRF5x.cmake")

nRF5x_setup()
nRF5x_addAppUART()
nRF5x_addTwi()
nRF5x_addTwiDrvMaster()
addSaadc()

list(APPEND SOURCE_FILES src/main.cpp src/serial.cpp src/i2c.cpp src/adc.c)


nRF5x_addExecutable(${PROJECT_NAME} "${SOURCE_FILES}")


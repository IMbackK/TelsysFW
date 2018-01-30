cmake_minimum_required(VERSION 3.8)
project(telemetrysystem C CXX ASM)

set(NRF_TARGET "nrf52")

set(ARM_NONE_EABI_TOOLCHAIN_PATH "/usr/bin")
set(NRF5_SDK_PATH "../nRF5_SDK_12.3.0_d7731ad")
set(OPENOCD "/usr/bin/openocd")

include("nRF52.cmake")

nRF52_setup()
nRF52_addAppUART()
nRF52_addTwi()
nRF52_addTwiDrvMaster()
nRF52_addBLEGATT()
nRF52_addBLEAdvertising()
nRF52_addBLElbs()
nRF52_addBLEnus()
nRF52_addSaadc()
nRF52_addAppFDS()
nRF52_addAppTimer()
nRF52_addDrvTimer()
nRF52_addAppFDS()
nRF52_addPPI()

add_definitions(-DBOARD_PROTOONE)

list(APPEND SOURCE_FILES src/main.cpp src/serial.cpp src/i2c.cpp src/adc.c src/twiCshm.c src/MPU9150.cpp src/btleserial.cpp src/MCP4725.cpp )


nRF52_addExecutable(${PROJECT_NAME} "${SOURCE_FILES}")


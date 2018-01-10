cmake_minimum_required(VERSION 3.6)

# check if all the necessary toolchain SDK and tools paths have been provided.
if (NOT ARM_NONE_EABI_TOOLCHAIN_PATH)
    message(FATAL_ERROR "The path to the arm-none-eabi-gcc toolchain (ARM_NONE_EABI_TOOLCHAIN_PATH) must be set.")
endif ()

if (NOT NRF5_SDK_PATH)
    message(FATAL_ERROR "The path to the nRF5 SDK (NRF5_SDK_PATH) must be set.")
endif ()

if (NOT OPENOCD)
    message(FATAL_ERROR "The path to the openocd utility (OPENOCD) must be set.")
endif ()

macro(nRF52_setup)

    set(CMAKE_C_STANDARD 99)
    set(CMAKE_CXX_STANDARD 98)

    #use arm-none-eabi-gcc
    set(CMAKE_C_COMPILER "${ARM_NONE_EABI_TOOLCHAIN_PATH}/arm-none-eabi-gcc")
    set(CMAKE_CXX_COMPILER "${ARM_NONE_EABI_TOOLCHAIN_PATH}/arm-none-eabi-c++")
    set(CMAKE_ASM_COMPILER "${ARM_NONE_EABI_TOOLCHAIN_PATH}/arm-none-eabi-gcc")

    include_directories("${NRF5_SDK_PATH}/components/softdevice/common/softdevice_handler")

    list(APPEND SDK_SOURCE_FILES "${NRF5_SDK_PATH}/components/softdevice/common/softdevice_handler/softdevice_handler.c")

    set(NRF5_LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/ld/gcc_nrf52.ld")
    set(CPU_FLAGS "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16")
    add_definitions(-DNRF52 -DNRF52832 -DNRF52_PAN_64 -DNRF52_PAN_12 -DNRF52_PAN_58 -DNRF52_PAN_54 -DNRF52_PAN_31 -DNRF52_PAN_51 -DNRF52_PAN_36 -DNRF52_PAN_15 -DNRF52_PAN_20 -DNRF52_PAN_55 -DBOARD_PCA10040)
    add_definitions(-DSOFTDEVICE_PRESENT -DS132 -DBLE_STACK_SUPPORT_REQD -DNRF_SD_BLE_API_VERSION=3)
    include_directories(
            "${NRF5_SDK_PATH}/components/softdevice/s132/headers"
            "${NRF5_SDK_PATH}/components/softdevice/s132/headers/nrf52"
    )
    list(APPEND SDK_SOURCE_FILES
            "${NRF5_SDK_PATH}/components/toolchain/system_nrf52.c"
            "${NRF5_SDK_PATH}/components/toolchain/gcc/gcc_startup_nrf52.S"
            )
    set(SOFTDEVICE_PATH "${NRF5_SDK_PATH}/components/softdevice/s132/hex/s132_nrf52_3.0.0_softdevice.hex")

    set(COMMON_FLAGS "-MP -MD -mthumb -mabi=aapcs -Wall -Os -g3 -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin --short-enums ${CPU_FLAGS}")

    # compiler/assambler/linker flags
    set(CMAKE_C_FLAGS "${COMMON_FLAGS}")
    set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++11") #todo: why ist -std=c++11 overriden?
    message( "g++ flags: ${CMAKE_CXX_FLAGS}" )
    set(CMAKE_ASM_FLAGS "-MP -MD -std=c11 -x assembler-with-cpp")  
    set(CMAKE_EXE_LINKER_FLAGS "-mthumb -mabi=aapcs -std=c++11 -std=c99 -L ${NRF5_SDK_PATH}/components/toolchain/gcc -T${NRF5_LINKER_SCRIPT} ${CPU_FLAGS} -Wl,--gc-sections --specs=nano.specs -lc -lnosys -lm")
    #Override the default flags so that CMAKE_C_FLAGS are not added automaticlly.
    set(CMAKE_C_LINK_EXECUTABLE "${CMAKE_C_COMPILER} <LINK_FLAGS> <OBJECTS> -o <TARGET>")
    set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_C_COMPILER} <LINK_FLAGS> <OBJECTS> -lstdc++ -o <TARGET>")

    include_directories(".")

    # basic board definitions and drivers
    include_directories(
            "${NRF5_SDK_PATH}/components/boards"
            "${NRF5_SDK_PATH}/components/device"
            "${NRF5_SDK_PATH}/components/libraries/util"
            "${NRF5_SDK_PATH}/components/drivers_nrf/make"
            "${NRF5_SDK_PATH}/components/drivers_nrf/common"
            "${NRF5_SDK_PATH}/components/drivers_nrf/delay"
            "${NRF5_SDK_PATH}/components/drivers_nrf/uart"
            "${NRF5_SDK_PATH}/components/drivers_nrf/hal"
            "${NRF5_SDK_PATH}/components/drivers_nrf/clock"
            "${NRF5_SDK_PATH}/components/drivers_nrf/rtc"
            "${NRF5_SDK_PATH}/components/drivers_nrf/gpiote"
    )

    # gcc toolchain specyfic
    include_directories(
            "${NRF5_SDK_PATH}/components/toolchain/"
            "${NRF5_SDK_PATH}/components/toolchain/gcc"
            "${NRF5_SDK_PATH}/components/toolchain/cmsis/include"
    )

    # log
    include_directories(
            "${NRF5_SDK_PATH}/components/libraries/log"
            "${NRF5_SDK_PATH}/components/libraries/log/src"
            "${NRF5_SDK_PATH}/components/libraries/timer"
    )

    # Segger RTT
    include_directories(
            "${NRF5_SDK_PATH}/external/segger_rtt/"
    )

    # basic board support and drivers
    list(APPEND SDK_SOURCE_FILES
            "${NRF5_SDK_PATH}/components/boards/boards.c"
            "${NRF5_SDK_PATH}/components/drivers_nrf/common/nrf_drv_common.c"
            "${NRF5_SDK_PATH}/components/drivers_nrf/clock/nrf_drv_clock.c"
            #"${NRF5_SDK_PATH}/components/drivers_nrf/uart/nrf_drv_uart.c"
            "${NRF5_SDK_PATH}/components/drivers_nrf/rtc/nrf_drv_rtc.c"
            "${NRF5_SDK_PATH}/components/drivers_nrf/gpiote/nrf_drv_gpiote.c"
            )

    # drivers and utils
    list(APPEND SDK_SOURCE_FILES
            "${NRF5_SDK_PATH}/components/libraries/hardfault/hardfault_implementation.c"
            "${NRF5_SDK_PATH}/components/libraries/util/nrf_assert.c"
            "${NRF5_SDK_PATH}/components/libraries/util/sdk_errors.c"
            "${NRF5_SDK_PATH}/components/libraries/util/app_error.c"
            "${NRF5_SDK_PATH}/components/libraries/util/app_error_weak.c"
            "${NRF5_SDK_PATH}/components/libraries/util/app_util_platform.c"
            "${NRF5_SDK_PATH}/components/libraries/log/src/nrf_log_backend_serial.c"
            "${NRF5_SDK_PATH}/components/libraries/log/src/nrf_log_frontend.c"
            "${NRF5_SDK_PATH}/components/libraries/util/app_util_platform.c"
            "${NRF5_SDK_PATH}/components/libraries/util/sdk_mapped_flags.c"
            )

    #Segger RTT
    list(APPEND SDK_SOURCE_FILES
            "${NRF5_SDK_PATH}/external/segger_rtt/RTT_Syscalls_GCC.c"
            "${NRF5_SDK_PATH}/external/segger_rtt/SEGGER_RTT.c"
            "${NRF5_SDK_PATH}/external/segger_rtt/SEGGER_RTT_printf.c"
            )

    # Common Bluetooth Low Energy files
    include_directories(
            "${NRF5_SDK_PATH}/components/ble"
            "${NRF5_SDK_PATH}/components/ble/common"
    )

    list(APPEND SDK_SOURCE_FILES
            "${NRF5_SDK_PATH}/components/ble/common/ble_advdata.c"
            "${NRF5_SDK_PATH}/components/ble/common/ble_conn_params.c"
            "${NRF5_SDK_PATH}/components/ble/common/ble_conn_state.c"
            "${NRF5_SDK_PATH}/components/ble/common/ble_srv_common.c"
            )

    # adds target for erasing and flashing the board with a softdevice
    add_custom_target(FLASH_SOFTDEVICE
            COMMAND ${OPENOCD} -f interface/stlink-v2.cfg -f target/nrf52.cfg -c init -c "reset init" -c halt -c "program ${SOFTDEVICE_PATH} verify" -c reset -c exit
            COMMENT "flashing SoftDevice"
            )

    add_custom_target(FLASH_ERASE
            COMMAND ${OPENOCD} -f interface/stlink-v2.cfg -f target/nrf52.cfg -c init -c "reset init" -c halt -c "nrf5 mass_erase" -c reset -c exit
            COMMENT "erasing flash"
            )
endmacro(nRF52_setup)

# adds a target for comiling and flashing an executable
macro(nRF52_addExecutable EXECUTABLE_NAME SOURCE_FILES)
    # executable
    add_executable(${EXECUTABLE_NAME} ${SDK_SOURCE_FILES} ${SOURCE_FILES})
    set_target_properties(${EXECUTABLE_NAME} PROPERTIES SUFFIX ".out")
    set_target_properties(${EXECUTABLE_NAME} PROPERTIES LINK_FLAGS "-Wl,-Map=${EXECUTABLE_NAME}.map")

    # additional POST BUILD setps to create the .bin and .hex files
    add_custom_command(TARGET ${EXECUTABLE_NAME}
            POST_BUILD
            COMMAND ${ARM_NONE_EABI_TOOLCHAIN_PATH}/arm-none-eabi-size ${EXECUTABLE_NAME}.out
            COMMAND ${ARM_NONE_EABI_TOOLCHAIN_PATH}/arm-none-eabi-objcopy -O binary ${EXECUTABLE_NAME}.out "${EXECUTABLE_NAME}.bin"
            COMMAND ${ARM_NONE_EABI_TOOLCHAIN_PATH}/arm-none-eabi-objcopy -O ihex ${EXECUTABLE_NAME}.out "${EXECUTABLE_NAME}.hex"
            COMMENT "post build steps for ${EXECUTABLE_NAME}")

    # custom target for flashing the board
    add_custom_target("FLASH_${EXECUTABLE_NAME}" ALL
            COMMAND ${OPENOCD} -f interface/stlink-v2.cfg -f target/nrf52.cfg -c init -c "reset init" -c halt -c "program ${EXECUTABLE_NAME}.hex verify" -c reset -c exit
            DEPENDS ${EXECUTABLE_NAME}
            COMMENT "flashing ${EXECUTABLE_NAME}.hex"
            )
endmacro()

# adds app-level scheduler library
macro(nRF52_addAppScheduler)
    include_directories("${NRF5_SDK_PATH}/components/libraries/scheduler")
    list(APPEND SDK_SOURCE_FILES "${NRF5_SDK_PATH}/components/libraries/scheduler/app_scheduler.c" "${NRF5_SDK_PATH}/components/softdevice/common/softdevice_handler/softdevice_handler_appsh.c")
endmacro(nRF52_addAppScheduler)

# adds app-level FIFO libraries
macro(nRF52_addAppFIFO)
    include_directories("${NRF5_SDK_PATH}/components/libraries/fifo")
    list(APPEND SDK_SOURCE_FILES "${NRF5_SDK_PATH}/components/libraries/fifo/app_fifo.c")
endmacro(nRF52_addAppFIFO)

# adds app-level Timer libraries
macro(nRF52_addAppTimer)
    list(APPEND SDK_SOURCE_FILES "${NRF5_SDK_PATH}/components/libraries/timer/app_timer.c")
endmacro(nRF52_addAppTimer)

# adds app-level UART libraries
macro(nRF52_addAppUART)
    include_directories("${NRF5_SDK_PATH}/components/libraries/uart")
    list(APPEND SDK_SOURCE_FILES"${NRF5_SDK_PATH}/components/libraries/uart/app_uart_fifo.c")
endmacro(nRF52_addAppUART)

# adds app-level Button library
macro(nRF52_addAppButton)
    include_directories("${NRF5_SDK_PATH}/components/libraries/button")
    list(APPEND SDK_SOURCE_FILES"${NRF5_SDK_PATH}/components/libraries/button/app_button.c")
endmacro(nRF52_addAppButton)

# adds app-level TWI library
macro(nRF52_addTwi)
    include_directories("${NRF5_SDK_PATH}/components/libraries/twi")
    list(APPEND SDK_SOURCE_FILES "${NRF5_SDK_PATH}/components/libraries/twi/app_twi.c")
endmacro(nRF52_addTwi)

# adds twi master library
macro(nRF52_addTwiDrvMaster)
    include_directories("${NRF5_SDK_PATH}/components/drivers_nrf/twi_master")
    list(APPEND SDK_SOURCE_FILES "${NRF5_SDK_PATH}/components/drivers_nrf/twi_master/nrf_drv_twi.c")
endmacro(nRF52_addTwiDrvMaster)

# adds Bluetooth Low Energy GATT support library
macro(nRF52_addBLEGATT)
    include_directories("${NRF5_SDK_PATH}/components/ble/nrf_ble_gatt")
    list(APPEND SDK_SOURCE_FILES "${NRF5_SDK_PATH}/components/ble/nrf_ble_gatt/nrf_ble_gatt.c")
endmacro(nRF52_addBLEGATT)

# adds Bluetooth Low Energy advertising support library
macro(nRF52_addBLEAdvertising)
    include_directories(
            "${NRF5_SDK_PATH}/components/ble/ble_advertising"
    )

    list(APPEND SDK_SOURCE_FILES
            "${NRF5_SDK_PATH}/components/ble/ble_advertising/ble_advertising.c"
            )

endmacro(nRF52_addBLEAdvertising)

# adds Bluetooth Low Energy advertising support library
macro(nRF52_addBLEPeerManager)
    include_directories("${NRF5_SDK_PATH}/components/ble/peer_manager")

    list(APPEND SDK_SOURCE_FILES
            "${NRF5_SDK_PATH}/components/ble/peer_manager/gatt_cache_manager.c"
            "${NRF5_SDK_PATH}/components/ble/peer_manager/gatts_cache_manager.c"
            "${NRF5_SDK_PATH}/components/ble/peer_manager/id_manager.c"
            "${NRF5_SDK_PATH}/components/ble/peer_manager/peer_data.c"
            "${NRF5_SDK_PATH}/components/ble/peer_manager/peer_data_storage.c"
            "${NRF5_SDK_PATH}/components/ble/peer_manager/peer_database.c"
            "${NRF5_SDK_PATH}/components/ble/peer_manager/peer_id.c"
            "${NRF5_SDK_PATH}/components/ble/peer_manager/peer_manager.c"
            "${NRF5_SDK_PATH}/components/ble/peer_manager/pm_buffer.c"
            "${NRF5_SDK_PATH}/components/ble/peer_manager/pm_mutex.c"
            "${NRF5_SDK_PATH}/components/ble/peer_manager/security_dispatcher.c"
            "${NRF5_SDK_PATH}/components/ble/peer_manager/security_manager.c"
            )

endmacro(nRF52_addBLEPeerManager)

macro(nRF52_addBLElbs)
    include_directories("${NRF5_SDK_PATH}/components/ble/ble_services/ble_lbs")

    list(APPEND SDK_SOURCE_FILES
            "${NRF5_SDK_PATH}/components/ble/ble_services/ble_lbs/ble_lbs.c"

            )

endmacro(nRF52_addBLElbs)

#bluetooth serial service
macro(nRF52_addBLEnus)
    include_directories("${NRF5_SDK_PATH}/components/ble/ble_services/ble_nus")

    list(APPEND SDK_SOURCE_FILES
            "${NRF5_SDK_PATH}/components/ble/ble_services/ble_nus/ble_nus.c"

            )

endmacro(nRF52_addBLEnus)

macro(nRF52_addBsp)
    include_directories("${NRF5_SDK_PATH}/components/libraries/bsp/")

    list(APPEND SDK_SOURCE_FILES
            "${NRF5_SDK_PATH}/components/libraries/bsp/bsp.c"

            )

endmacro(nRF52_addBsp)

# adds app-level FDS (flash data storage) library
macro(nRF52_addAppFDS)
    include_directories(
            "${NRF5_SDK_PATH}/components/libraries/fds"
            "${NRF5_SDK_PATH}/components/libraries/fstorage"
            "${NRF5_SDK_PATH}/components/libraries/experimental_section_vars"
    )

    list(APPEND SDK_SOURCE_FILES
            "${NRF5_SDK_PATH}/components/libraries/fds/fds.c"
            "${NRF5_SDK_PATH}/components/libraries/fstorage/fstorage.c"
            )

endmacro(nRF52_addAppFDS)

macro(nRF52_addSaadc)
    include_directories("${NRF5_SDK_PATH}/components/drivers_nrf/saadc")
    list(APPEND SDK_SOURCE_FILES "${NRF5_SDK_PATH}/components/drivers_nrf/saadc/nrf_drv_saadc.c"
                                 "${NRF5_SDK_PATH}/components/drivers_nrf/hal/nrf_saadc.c")
endmacro(nRF52_addSaadc)

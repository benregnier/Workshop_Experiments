# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/Users/tom/pico/pico-sdk")

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(Computer_HelloWorld C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Computer_HelloWorld Computer_HelloWorld.c )

pico_set_program_name(Computer_HelloWorld "Computer_HelloWorld")
pico_set_program_version(Computer_HelloWorld "0.1")

pico_enable_stdio_uart(Computer_HelloWorld 0)
pico_enable_stdio_usb(Computer_HelloWorld 1)

# Add the standard library to the build
target_link_libraries(Computer_HelloWorld
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Computer_HelloWorld PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(Computer_HelloWorld 
        hardware_spi
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_timer
        hardware_clocks
        hardware_adc
        pico_stdlib
        hardware_pwm
        )

pico_add_extra_outputs(Computer_HelloWorld)


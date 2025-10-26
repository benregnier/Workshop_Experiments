# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/tom/pico/pico-sdk/tools/pioasm"
  "/Users/tom/Documents/GitHub/DYSKI/Firmware/RP2040/Computer_HelloWorld/build/pioasm"
  "/Users/tom/Documents/GitHub/DYSKI/Firmware/RP2040/Computer_HelloWorld/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm"
  "/Users/tom/Documents/GitHub/DYSKI/Firmware/RP2040/Computer_HelloWorld/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp"
  "/Users/tom/Documents/GitHub/DYSKI/Firmware/RP2040/Computer_HelloWorld/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
  "/Users/tom/Documents/GitHub/DYSKI/Firmware/RP2040/Computer_HelloWorld/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src"
  "/Users/tom/Documents/GitHub/DYSKI/Firmware/RP2040/Computer_HelloWorld/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/tom/Documents/GitHub/DYSKI/Firmware/RP2040/Computer_HelloWorld/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/tom/Documents/GitHub/DYSKI/Firmware/RP2040/Computer_HelloWorld/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()

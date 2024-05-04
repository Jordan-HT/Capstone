# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/VSARM/sdk/pico/pico-sdk/tools/pioasm"
  "E:/Git/Capstone/Software/RP2040/build/pioasm"
  "E:/Git/Capstone/Software/RP2040/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm"
  "E:/Git/Capstone/Software/RP2040/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp"
  "E:/Git/Capstone/Software/RP2040/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
  "E:/Git/Capstone/Software/RP2040/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src"
  "E:/Git/Capstone/Software/RP2040/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/Git/Capstone/Software/RP2040/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/Git/Capstone/Software/RP2040/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()

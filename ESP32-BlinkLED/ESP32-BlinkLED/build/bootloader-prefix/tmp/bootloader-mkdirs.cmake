# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/juanm/esp/v5.3.1/esp-idf/components/bootloader/subproject"
  "D:/Documents/GitHub/ESP32-Examples/ESP32-BlinkLED/ESP32-BlinkLED/build/bootloader"
  "D:/Documents/GitHub/ESP32-Examples/ESP32-BlinkLED/ESP32-BlinkLED/build/bootloader-prefix"
  "D:/Documents/GitHub/ESP32-Examples/ESP32-BlinkLED/ESP32-BlinkLED/build/bootloader-prefix/tmp"
  "D:/Documents/GitHub/ESP32-Examples/ESP32-BlinkLED/ESP32-BlinkLED/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Documents/GitHub/ESP32-Examples/ESP32-BlinkLED/ESP32-BlinkLED/build/bootloader-prefix/src"
  "D:/Documents/GitHub/ESP32-Examples/ESP32-BlinkLED/ESP32-BlinkLED/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Documents/GitHub/ESP32-Examples/ESP32-BlinkLED/ESP32-BlinkLED/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Documents/GitHub/ESP32-Examples/ESP32-BlinkLED/ESP32-BlinkLED/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()

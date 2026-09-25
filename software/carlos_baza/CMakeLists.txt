# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# micro-ROS buduje swoje narzędzia systemowym gcc. Asemblery z toolchainów ESP
# (katalogi .../<toolchain>/<toolchain>/bin) zasłaniają systemowy /usr/bin/as
# i psują tę kompilację, więc usuwamy je z PATH na czas konfiguracji.
string(REPLACE ":" ";" _path_list "$ENV{PATH}")
set(_clean_path "")
foreach(_dir IN LISTS _path_list)
   if(NOT _dir MATCHES "/riscv32-esp-elf/riscv32-esp-elf/bin/?$|/xtensa-esp-elf/xtensa-esp-elf/bin/?$|/esp32ulp-elf/esp32ulp-elf/bin/?$")
    list(APPEND _clean_path "${_dir}")
  endif()
endforeach()
string(REPLACE ";" ":" _clean_path "${_clean_path}")
set(ENV{PATH} "${_clean_path}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(carlos_baza)

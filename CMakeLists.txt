# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

add_compile_options(-Wno-ignored-qualifiers -Wno-missing-field-initializers)
add_compile_definitions(ARDUINO_USB_CDC_ON_BOOT=1 ARDUINO_USB_MODE=1)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

idf_build_set_property(MINIMAL_BUILD ON)
project(platform)

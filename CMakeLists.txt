cmake_minimum_required(VERSION 2.8.4)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)
set(PROJECT_NAME arduino-motorfocus)
project(${PROJECT_NAME})

set(${CMAKE_PROJECT_NAME}_SKETCH arduino-motorfocus.ino)
#set_source_files_properties(arduino-motorfocus.ino PROPERTIES LANGUAGE CXX)

#### Uncomment below additional settings as needed.
# set(ARDUINO_DEFAULT_BOARD nano328)
set(${CMAKE_PROJECT_NAME}_BOARD nano)
set(${CMAKE_PROJECT_NAME}_PORT /dev/ttyUSB0)

set(nano.upload.maximum_size 30720)
set(nano.upload.maximum_data_size 2048)
set(nano.upload.speed 57600)
set(nano.bootloader.low_fuses 0xFF)
set(nano.bootloader.high_fuses 0xDA)
set(nano.bootloader.extended_fuses 0x05)
set(nano.bootloader.file atmega/ATmegaBOOT_168_atmega328.hex)
set(nano.build.mcu atmega328p)

# libs
link_directories(${CMAKE_SOURCE_DIR}/../libraries)
set(AccelStepper_RECURSE True)

generate_arduino_firmware(${CMAKE_PROJECT_NAME})

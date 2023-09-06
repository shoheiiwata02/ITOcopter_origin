cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_copter C CXX ASM)
set(CMAKE_C_STNDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()
include_directories(/home/shohei/pico/eigen)

add_executable(pico_copter
  pico_copter.cpp
  sensor.cpp
  ekf.cpp
  lsm9ds1_reg.c
  pwm.cpp
  radio.cpp
  control.cpp
  WS2812.cpp
  rgbled.cpp
  i2c.cpp
  pid.cpp
)

pico_generate_pio_header(pico_copter ${CMAKE_CURRENT_LIST_DIR}/WS2812.pio)


target_link_libraries(pico_copter 
  hardware_spi 
  hardware_pwm 
  hardware_uart 
  hardware_pio
  hardware_i2c
  pico_stdlib
  pico_multicore)

pico_enable_stdio_usb(pico_copter 1)
pico_enable_stdio_uart(pico_copter 0)

pico_add_extra_outputs(pico_copter)
cmake_minimum_required(VERSION 3.16)

project(bno085_uart_shtp_test C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(bno085_uart_shtp_test
  src/bno085_uart_shtp_test.cpp
  src/uart_sh2_hal.cpp

  third_party/sparkfun_bno08x/src/sh2.c
  third_party/sparkfun_bno08x/src/sh2_util.c
  third_party/sparkfun_bno08x/src/shtp.c
  third_party/sparkfun_bno08x/src/sh2_SensorValue.c
)

target_include_directories(bno085_uart_shtp_test PRIVATE
  third_party/sparkfun_bno08x/src
  src
)

target_compile_options(bno085_uart_shtp_test PRIVATE -Wall -Wextra -Wpedantic)

add_executable(bno085_i2c_shtp_test
  src/bno085_i2c_shtp_test.cpp
  src/i2c_sh2_hal.cpp

  third_party/sparkfun_bno08x/src/sh2.c
  third_party/sparkfun_bno08x/src/sh2_util.c
  third_party/sparkfun_bno08x/src/shtp.c
  third_party/sparkfun_bno08x/src/sh2_SensorValue.c
)

target_include_directories(bno085_i2c_shtp_test PRIVATE
  third_party/sparkfun_bno08x/src
  src
)

target_compile_options(bno085_i2c_shtp_test PRIVATE -Wall -Wextra -Wpedantic)


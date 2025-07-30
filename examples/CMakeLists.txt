cmake_minimum_required(VERSION 3.10)

# Project name
project(serial_comms_examples C)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)


# DART Protocol example
add_executable(example_uart_struct example_uart_struct.c)

target_link_libraries(example_uart_struct
    serial_comms
    checksum
)

target_include_directories(example_uart_struct PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

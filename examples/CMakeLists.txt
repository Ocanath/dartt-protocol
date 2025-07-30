cmake_minimum_required(VERSION 3.10)

# Project name
project(dartt_protocol_examples C)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)


# DARTT Protocol example
add_executable(example_uart_struct example_uart_struct.c)

target_link_libraries(example_uart_struct
    dartt_protocol
    dartt_checksum
)

target_include_directories(example_uart_struct PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

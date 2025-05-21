cmake_minimum_required(VERSION 3.10)

# Project name
project(serial_comms_examples C)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Create executable
add_executable(debug_general_handler debug_general_handler.c)

# Link with source files
target_link_libraries(debug_general_handler
    serial_comms
    checksum
)

# Include directories
target_include_directories(debug_general_handler PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

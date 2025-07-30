cmake_minimum_required(VERSION 3.10)

# Create dart_protocol library
add_library(dart_protocol
    serial-comms.c
    serial-motor-comms.c
)

# Create dart_checksum library
add_library(dart_checksum
    checksum.c
)

# Include directories for both libraries
target_include_directories(dart_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(dart_checksum PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
) 

cmake_minimum_required(VERSION 3.10)

# Create serial_comms library
add_library(serial_comms
    serial-comms.c
    serial-motor-comms.c
)

# Create checksum library
add_library(checksum
    checksum.c
)

# Include directories for both libraries
target_include_directories(serial_comms PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(checksum PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
) 
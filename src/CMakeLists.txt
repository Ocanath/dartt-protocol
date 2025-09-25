cmake_minimum_required(VERSION 3.10)

# Create dartt_protocol library
add_library(dartt_protocol
    dartt.c
	dartt_sync.c
)

# Create dartt_checksum library
add_library(dartt_checksum
    checksum.c
)

# Include directories for both libraries
target_include_directories(dartt_protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(dartt_checksum PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
) 

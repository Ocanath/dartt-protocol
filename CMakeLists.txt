cmake_minimum_required(VERSION 3.10)

# Project name
project(dartt_protocol C)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Enable warnings and treat them as errors
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror")

# Add subdirectories
add_subdirectory(src)

# Only configure build type and examples when built standalone.
# When included via add_subdirectory() these would clobber the parent's settings.
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    # Debug configuration
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type (Debug or Release)" FORCE)
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0")
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -g")

    add_subdirectory(examples)
endif()

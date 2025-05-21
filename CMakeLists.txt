cmake_minimum_required(VERSION 3.10)

# Force C compiler
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER gcc)

# Project name
project(serial_comms C)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Add subdirectories
add_subdirectory(src)
add_subdirectory(examples) 
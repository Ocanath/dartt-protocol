# DARTT - Dual Address Real-Time Transport Protocol

## Introduction

DARTT is a serial communication protocol that provides a simple, lightweight and low-overhead method of communicating with embedded systems. Its primary intended use case for robotics (sensors, motor controllers), but is useful as a general-purpose protocol for any simple communication network with a controller-peripheral structure. 

DARTT has the following key advantages:

### Rapid Development

Above all else, DARTT is designed for rapid protoyping and deployment of highly customized communication protocols. It is perfect for small teams or solo developers who want to set up fully featured communication on many different devices as quickly as possible. In most situations, a DARTT memory layout can be defined with a typedef struct in a single, shared header, containing all attributes that might need to be exposed over a communication interface. 

### Near-Zero Overhead

DARTT is designed to be extremely minimal. The generic block memory read/write frame formats have at minimum two bytes of overhead, and at maximum 5 bytes. Messages sizes are also highly predictable, meaning that even the generic access format is suitable for real-time communication. Additionally, the dual addressing feature allows for zero overhead communication with specialized frame formats.

### Flexibility

DARTT is designed to be supported over nearly all microcontroller peripheral communication protocols with minimum overhead, including:

- SPI
- I2C
- CAN
- UART
- Multi-drop serial protocols, such as RS485
- UDP/TCP over WiFi
- BLE

DARTT messages will be formed depending on the supported features of the communciation protocol - for example, in CAN, where error handling via CRC and device filtering (by message ID) are handled on a protocol level, these elements are stripped from the frame format to minimize overhead. For communication over protocols such as RS485, DARTT handles both address filtering and error checking via CRC. For protocols with built-in device selection but no error handling (SPI, I2C, or point-to-point protocols such as UART), DARTT handles CRC but removes redundant address filtering.

The DARTT library is designed to be C language standards compliant - however, in embedded systems where the controller and peripheral endian-ness and bit-width match, type punning is the recommended way to define the block memory structure. This is especially true for the peripheral, which is most commonly an embedded system with fixed CPU architecture, and therefore will have a predictable memory layout. This is also the reason that DARTT is little-endian - for the most commonly used CPU architectures (ARM and x86), little-endian is the default. Support across multiple languages (such as python) is possible, but requires additional development effort to support (manual enumeration, serialization tools, etc. ).

## Integration into Your Project

The recommended way to include DARTT is as a git submodule. The required minimum files are:
```
checksum.c
checksum.h
dartt.c
dartt.h
```

For controller devices, it is recommended to also include:
```
dartt_sync.c
dartt_sync.h
```


## Building and Testing

### Prerequisites
- CMake (version 3.10 or higher)
- C compiler with C11 support
- ceedling 1.0.1 or higher

### Building the Project
```bash
mkdir build
cd build
cmake ..
cmake --build .
```

### Running Unit Tests
The test suite can be run via ceedling, which must first be installed.
```bash
ceedling test:all
```


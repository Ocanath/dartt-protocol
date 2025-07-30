# DARTT - Dual Address Real-Time Transport Protcol

## Summary

DARTT is a lightweight serial communication protocol that provides a simple, lightweight and low-overhead method of communicating with embedded systems. It's primary intended use case is in motor controllers for robots, but is useful as a general-purpose protocol for any simple communication network with a controller-peripheral structure. 

The protocol defines two primary frame types - an application defined 'motor' frame type, and a general purpose 'misc' frame type. These are generally accessed with a 'dual addressing' schema, where each peripheral responds to two different addresses - a 'motor' address and a 'misc' address. 

The 'motor' frame is an application defined tight/packed frame for highest possible bandwidth containing specialized command and response patterns (such as voltage, current, position, velocity, etc). The 'misc' frames are protocol defined, and function as block memory read and write messages. 

The motor frame and block memory layout are application specific, and occupy the 'application' layer. Below this layer is the 'protocol' layer - this defines the underlying structure of DARTT generic/misc read and write messages (with block index, payload, read/write bit, etc), detailed below. Below the 'protocol' layer is the 'frame' layer - this is transport protocol specific, and may contain an address and CRC (type 0), just a CRC (type 1), or neither an address nor CRC (type 2). 

The DARTT library is designed to be C language standards compliant - however, in embedded systems where the controller and peripheral endian-ness and bit-width match, type punning is the recommended way to define the block memory structure. This is especially true for the peripheral, which is most commonly an embedded system with fixed CPU architecture, and therefore will have a predictable memory layout. This is also the reason that DARTT is little-endian - for the most commonly used CPU architectures (ARM and x86), little-endian is the default.

## Overview

The protocol supports three different message types to accommodate various transport layers:
- **TYPE_SERIAL_MESSAGE**: Raw serial with address and CRC
- **TYPE_ADDR_MESSAGE**: Built-in addressing with CRC
- **TYPE_ADDR_CRC_MESSAGE**: Built-in addressing and CRC

All multi-byte integers are transmitted in **little-endian** format.

## Frame Formats

### TYPE_SERIAL_MESSAGE (Type 0)
Used for raw serial protocols like UART, RS485, RS232.

#### Write Frame Format
```
+--------+--------+--------+--------+--------+--------+--------+
| Byte 0 | Byte 1 | Byte 2 |  ...   |  ...   | CRC Lo | CRC Hi |
+--------+--------+--------+--------+--------+--------+--------+
|Address | Index Lo| Index Hi|    Payload Data         |   CRC  |
+--------+--------+--------+--------+--------+--------+--------+
```

#### Read Frame Format
```
+--------+--------+--------+--------+--------+--------+--------+
| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | CRC Lo | CRC Hi |
+--------+--------+--------+--------+--------+--------+--------+
|Address |   Index (R=1)   |   Num Bytes     |      CRC       |
+--------+--------+--------+--------+--------+--------+--------+
```

### TYPE_ADDR_MESSAGE (Type 1)
Used for protocols with built-in addressing like SPI, I2C.

#### Write Frame Format
```
+--------+--------+--------+--------+--------+--------+
| Byte 0 | Byte 1 |  ...   |  ...   | CRC Lo | CRC Hi |
+--------+--------+--------+--------+--------+--------+
| Index Lo| Index Hi|    Payload Data     |   CRC  |
+--------+--------+--------+--------+--------+--------+
```

#### Read Frame Format
```
+--------+--------+--------+--------+--------+--------+
| Byte 0 | Byte 1 | Byte 2 | Byte 3 | CRC Lo | CRC Hi |
+--------+--------+--------+--------+--------+--------+
|   Index (R=1)   |   Num Bytes     |      CRC       |
+--------+--------+--------+--------+--------+--------+
```

### TYPE_ADDR_CRC_MESSAGE (Type 2)
Used for protocols with built-in addressing and CRC like CAN, UDP.

#### Write Frame Format
```
+--------+--------+--------+--------+--------+
| Byte 0 | Byte 1 |  ...   |  ...   |  ...  |
+--------+--------+--------+--------+--------+
| Index Lo| Index Hi|    Payload Data      |
+--------+--------+--------+--------+--------+
```

#### Read Frame Format
```
+--------+--------+--------+--------+
| Byte 0 | Byte 1 | Byte 2 | Byte 3 |
+--------+--------+--------+--------+
|   Index (R=1)   |   Num Bytes     |
+--------+--------+--------+--------+
```

## Read Reply Frame Formats

When a peripheral responds to a read request, the frame format varies by message type:

### TYPE_SERIAL_MESSAGE (Type 0) - Read Reply
The reply contains the requested data block with a prepended address and appended CRC:
```
+--------+--------+--------+--------+--------+--------+
| Byte 0 |  ...   |  ...   |  ...   | CRC Lo | CRC Hi |
+--------+--------+--------+--------+--------+--------+
|Address |    Requested Data Block    |      CRC       |
+--------+--------+--------+--------+--------+--------+
```

### TYPE_ADDR_MESSAGE (Type 1) - Read Reply  
The reply contains the requested data block with an appended CRC:
```
+--------+--------+--------+--------+--------+
|  ...   |  ...   |  ...   | CRC Lo | CRC Hi |
+--------+--------+--------+--------+--------+
|    Requested Data Block    |      CRC       |
+--------+--------+--------+--------+--------+
```

### TYPE_ADDR_CRC_MESSAGE (Type 2) - Read Reply
The reply contains only the requested data block with no additional data:
```
+--------+--------+--------+--------+
|  ...   |  ...   |  ...   |  ...   |
+--------+--------+--------+--------+
|    Requested Data Block             |
+--------+--------+--------+--------+
```

## Field Descriptions

### Address (1 byte)
- **Range**: 0x00 - 0xFF
- **Purpose**: Identifies the target device
- **Special Values**:
  - `0x7F`: Master motor address
  - `0x80`: Master misc address (complement of 0x7F)

### Index (2 bytes, little-endian)
- **Bit 15**: Read/Write flag (1 = Read, 0 = Write)
- **Bits 14-0**: 32-bit word-aligned index (actual byte offset = index × 4)
- **Range**: 0x0000 - 0x7FFF (word indices)

### Payload Data (Variable length)
- **Write frames**: Contains data to be written to the target device
- **Read frames**: Not present (read size specified separately)

### Num Bytes (2 bytes, little-endian)
- **Purpose**: Specifies number of bytes to read
- **Range**: 0x0000 - 0xFFFF
- **Only present in read frames**

### CRC (2 bytes, little-endian)
- **Algorithm**: CRC-16
- **Coverage**: All bytes except the CRC field itself
- **Present in**: TYPE_SERIAL_MESSAGE and TYPE_ADDR_MESSAGE only

## Examples

### Write Example (TYPE_SERIAL_MESSAGE)
Write 4 bytes `[0x12, 0x34, 0x56, 0x78]` to word index 5 on device 0x42:

```
Byte:  0    1    2    3    4    5    6    7    8
Data: 0x42 0x05 0x00 0x12 0x34 0x56 0x78 0xXX 0xXX
      |    |    |    |              | |    CRC   |
      |    |    |    +--- Payload ---+
      |    +- Index -+
      Address
```

### Read Example (TYPE_SERIAL_MESSAGE)
Read 8 bytes starting from word index 10 on device 0x42:

```
Byte:  0    1    2    3    4    5    6
Data: 0x42 0x8A 0x00 0x08 0x00 0xXX 0xXX
      |    |    |    |    |    |  CRC  |
      |    |    |    +- Num Bytes -+
      |    +- Index (0x800A) -+
      Address
```

## Addressing Scheme

The 8-bit address space is divided into four distinct regions to support different device types and communication patterns:

### Address Regions

| Address Range | Purpose      | Description                    |
|---------------|--------------|--------------------------------|
| `0x00-0x7E`   | Motor        | Individual motor device addresses (127 addresses) |
| `0x7F`        | Motor Master | Master address for motor communications |
| `0x80`        | Misc Master  | Master address for misc/general communications |
| `0x81-0xFF`   | Misc         | Individual misc device addresses (127 addresses) |

### Device Address Pairing

Each slave device has **two addresses**:
- **Motor Address**: Used for high-performance, application-specific motor control
- **Misc Address**: Used for general configuration, diagnostics, and data exchange via block memory reads and writes

The addresses are mathematically related by the formula:
```
misc_address = 0xFF - motor_address
```

### Address Examples

| Motor Address | Misc Address | Device Type |
|---------------|--------------|-------------|
| `0x01`        | `0xFE`       | Device 1    |
| `0x10`        | `0xEF`       | Device 16   |
| `0x42`        | `0xBD`       | Device 66   |
| `0x7E`        | `0x81`       | Device 126  |
| `0x7F`        | `0x80`       | Master      |

### Master Communication

- **Motor Master (`0x7F`)**: Used when master initiates motor-specific commands
- **Misc Master (`0x80`)**: Used as the source address for slave replies and general communications


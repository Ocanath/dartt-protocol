# Serial Communications Protocol

This document describes the byte-level frame format for the serial communications protocol used in this library.

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

## Constants

```c
#define NUM_BYTES_ADDRESS           1
#define NUM_BYTES_INDEX             2
#define NUM_BYTES_NUMWORDS_READREQUEST  2
#define NUM_BYTES_CHECKSUM          2
#define READ_WRITE_BITMASK          0x8000
```

## Error Codes

- `SERIAL_PROTOCOL_SUCCESS (0)`: Operation completed successfully
- `ADDRESS_FILTERED (-1)`: Message filtered due to address mismatch
- `ERROR_MALFORMED_MESSAGE (-2)`: Invalid frame format
- `ERROR_CHECKSUM_MISMATCH (-3)`: CRC validation failed
- `ERROR_INVALID_ARGUMENT (-4)`: Invalid function parameters
- `ERROR_MEMORY_OVERRUN (-5)`: Buffer overflow detected
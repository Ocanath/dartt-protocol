# DARTT - Dual Address Real-Time Transport Protcol

## Introduction

DARTT has the following key features:

### Dual-addressing Schema

The protocol defines two primary frame types - an application defined 'motor' frame type, and a general purpose 'misc' frame type. These are generally accessed with a 'dual addressing' schema, where each peripheral responds to two different addresses - a 'motor' address and a 'misc' address. There is a 1:1 mapping between motor and misc addresses - each device gets only one 'identifier', which splits the address range.

For applications that need absolutely zero overhead, the 'motor' address is used for custom frame formats exchanged with the highest possible bandwidth. Intended use is to pack specialized command and response patterns (such as voltage, current, position, velocity, etc). Typical use will be to use a fixed or context-defined zero-overhead frame structure. 

The 'misc' frames are generic and protocol defined, and function as block memory read and write messages. The application simply needs to define a memory layout with all information that needs to be accessible over the network, and map it to DARTT. The rest of the application code reads from and writes to that memory block. Misc frames have very low overhead and predictable sizes, making them suitable for real-time control applications. 

### Flexible Frame Format/Protocol Agnostic

DARTT frames have three different 'modes' which effect the frame overhead. 

The protocol supports three different message types to accommodate various transport layers:
- **TYPE_SERIAL_MESSAGE**: Raw serial with address and CRC
- **TYPE_ADDR_MESSAGE**: Built-in addressing with CRC
- **TYPE_ADDR_CRC_MESSAGE**: Built-in addressing and CRC

For protocols with no built-in arbitration or error handling, `TYPE_SERIAL_MESSAGE` will prepend an address for arbitration and append a CRC16 for validation/error handling. For protocols such as SPI and I2C which have built-in arbitration but no built in error handling, `TYPE_ADDR_MESSAGE` ensures only the CRC16 will be appended to the core DARTT layer message. For running DARTT over communication channels with fully managed arbitration and error handling (such as CAN-FD, UDP, BLE), `TYPE_ADDR_CRC_MESSAGE` removes address and CRC16 overhead from the core DARTT payload, minimizing overhead. 

**Note:** For CAN specifically, it is necessary to use `TYPE_ADDR_CRC_MESSAGE` and at least a 6 byte payload size for DARTT to be supported. For CAN-FD with payload sizes above 8 bytes, application-driven padding logic and/or or careful message sizing must be performed to prevent errors, as CAN-FD payload sizes between 8 and 64 bytes are enumerated in non-uniform increments.

## Frame Formats

DARTT generic block memory access relies on three different basic frame structures.
1. Write frame, consisting of a 16-bit index word where the MSB is a read/write bit (R = 0), followed by an array of N bytes
1. Read request frame, consisting of a 16-bit index word where the MSB is a read/write bit (R = 0), followed by a 16-bit 'number of bytes' argument
1. Read reply frame, consisting of an N-byte array containing the requested data from the Read request frame.

Indexes are 32-bit aligned. I.e. a DARTT write frame with index argument `2` would begin writing data at byte offset `8`.

The upper bound on DARTT memory block sizes is strictly bounded by the read request frame format - the maximum index argument is 15-bits, and 65535 bytes can be requested from that maximum offset, meaning the maximum possible block size is precisely 163,835 bytes. Practical considerations on maximum single read frame size further restrict this to ~131kb. If a control memory layout larger than 131kb is needed, custom extensions to the DARTT protocol should be implemented (i.e. filtering messages with additional arguments by some fixed preamble before passing to DARTT, subscibing to multiple addresses on one device, etc.) or else a different protocol entirely should be considered.

In order to support multiple communication protocols, core DARTT messages can be extended with addressing and error checking. Addresses are always single-byte and prepended to the frame, and checksums are always CRC16 and are appended to the frame. 

Index arguments and CRC16 words are always little-endian. Payload formatting is fully application defined and does not have any expected endian-ness or required formatting.

### TYPE_SERIAL_MESSAGE (Type 0)
Used for raw serial protocols like UART, RS485, RS232.

#### Write Frame Format

| Byte 0  | Bytes 1-2     | Bytes 3-N    | Bytes N+1 to N+2 |
|---------|---------------|--------------|------------------|
| Address | Index (R=0)   | Payload Data | CRC              |

#### Read Frame Format

| Byte 0  | Bytes 1-2   | Bytes 3-4 | Bytes 5-6 |
|---------|-------------|-----------|-----------|
| Address | Index (R=1) | Num Bytes | CRC       |

### TYPE_ADDR_MESSAGE (Type 1)
Used for protocols with built-in addressing like SPI, I2C.

#### Write Frame Format

| Bytes 0-1   | Bytes 2-N    | Bytes N+1 to N+2 |
|-------------|--------------|------------------|
| Index (R=0) | Payload Data | CRC              |

#### Read Frame Format

| Bytes 0-1   | Bytes 2-3 | Bytes 4-5 |
|-------------|-----------|-----------|
| Index (R=1) | Num Bytes | CRC       |

### TYPE_ADDR_CRC_MESSAGE (Type 2)
Used for protocols with built-in addressing and CRC like CAN, UDP.

#### Write Frame Format

| Bytes 0-1   | Bytes 2-N    |
|-------------|--------------|
| Index (R=0) | Payload Data |

#### Read Frame Format

| Bytes 0-1   | Bytes 2-3 |
|-------------|-----------|
| Index (R=1) | Num Bytes |

## Read Reply Frame Formats

When a peripheral responds to a read request, the frame format varies by message type:

### TYPE_SERIAL_MESSAGE (Type 0) - Read Reply
The reply contains the requested data block with a prepended address and appended CRC:

| Byte 0  | Bytes 1-N            | Bytes N+1 to N+2 |
|---------|----------------------|------------------|
| Address | Requested Data Block | CRC              |

### TYPE_ADDR_MESSAGE (Type 1) - Read Reply
The reply contains the requested data block with an appended CRC:

| Bytes 0-N            | Bytes N+1 to N+2 |
|----------------------|------------------|
| Requested Data Block | CRC              |

### TYPE_ADDR_CRC_MESSAGE (Type 2) - Read Reply
The reply contains only the requested data block with no additional data:

| Bytes 0-N            |
|----------------------|
| Requested Data Block |

## Field Descriptions

### Address (1 byte)
- **Range**
	- `0x00`-`0x7E`: Motor Address range
	- `0x7F`: Controller Address (default, when applicable)
	- `0x80`: Controller Misc Address (default, when applicable)
	- `0x81`-`0xFF`: Misc Address range
- **Purpose**: Identifies the target device. 

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



## Addressing Scheme

The 8-bit address space is divided into four distinct regions to support different device types and communication patterns:

### Address Regions

| Address Range | Purpose      | Description                    |
|---------------|--------------|--------------------------------|
| `0x00-0x7E`   | Motor        | Individual motor device addresses (127 addresses) |
| `0x7F`        | Motor Controller | Controller address for motor communications |
| `0x80`        | Misc Controller  | Controller address for misc/general communications |
| `0x81-0xFF`   | Misc         | Individual misc device addresses (127 addresses) |

### Device Address Pairing

Each peripheral device has **two addresses**:
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
| `0x7F`        | `0x80`       | Controller      |

### Controller Communication

- **Motor Controller (`0x7F`)**: Used when controller initiates motor-specific commands
- **Misc Controller (`0x80`)**: Used as the source address for peripheral replies and general communications


### Final Note on Addressing

Single byte dual addressing is only enforced at a protcol/API level for Type 0 messages. Outside of Type 0, arbitration must be handled outside of the DARTT API. This also means that the specific implementation of 'dual addressing' can be left up to the user in such situations. I.e. for CAN, which uses 11-bit identifiers, the same general technique of splitting the address range can be applied, or a completely different addressing schema can be used (i.e. the device can have only one fixed address and handle generic DARTT messages only). For point-to-point communications (such as UART), it is possible to omit the 'dual addressing' feature altogether and pass Type 1 generic messages back and forth, since it is only possible for the peripheral to recieve data from a single controller, and vice versa.
# DARTT Library Usage Guide

## Overview

The DARTT (Dual Address Real-Time Transport) library provides a C implementation for communicating using the DARTT protocol. This guide covers proper library usage, initialization, and best practices for both controller and peripheral implementations.

## Library Components

The DARTT library consists of two main components:

- **`dartt_protocol`**: Core protocol functions for frame creation, parsing, and communication
- **`dartt_checksum`**: CRC-16 checksum calculation utilities

## Including the Library

```c
#include "serial-comms.h"  // Core protocol functions
#include "checksum.h"      // CRC calculation utilities
```

Note: The library has two main components that need to be linked:
- `dartt_protocol` - Contains serial-comms.c and serial-motor-comms.c
- `dartt_checksum` - Contains checksum.c

## Usage Patterns

The DARTT library supports two main use cases with distinct APIs:

## Controller (Master) Implementation

Controllers initiate communication, send commands, and parse replies from peripherals.

#### 1. Initialize Communication
#### 2. Create Write Requests
#### 3. Create Read Requests
#### 4. Send and Parse Read Replies

### Peripheral (Slave) Usage

#### 1. Frame Reception and Parsing
#### 2. Handle General Messages (Read/Write)

## Message Types and Transport Layers

Choose the appropriate message type based on your transport layer:

### TYPE_SERIAL_MESSAGE (Type 0)
- **Use for**: UART, RS485, RS232
- **Features**: Includes address and CRC in frame
- **Best for**: Point-to-point or multi-drop serial communications
- **Highly recommended**: Use COBS (consistent-overhead byte stuffing) on top of DARTT before sending a DARTT message across the physical layer.

### TYPE_ADDR_MESSAGE (Type 1)  
- **Use for**: SPI, I2C
- **Features**: Includes CRC, relies on transport for addressing
- **Best for**: Protocols with built-in device addressing

### TYPE_ADDR_CRC_MESSAGE (Type 2)
- **Use for**: CAN, UDP, TCP
- **Features**: Minimal overhead, relies on transport for addressing and error detection
- **Best for**: Protocols with built-in addressing and error checking

## Memory Layout and Type Punning

### Recommended Approach (Embedded Systems)
For embedded systems with matching endianness and architecture:

```c
// Define your data structures
typedef struct {
    float position;
    float velocity; 
    uint16_t status;
    uint16_t padding;
} motor_data_t;

// Use type punning for direct memory access
motor_data_t* motor_data = (motor_data_t*)&device_memory[word_index * 4];
```

### Standards-Compliant Approach
For portable code or mixed architectures:

```c
// Use explicit serialization/deserialization
void serialize_motor_data(motor_data_t* data, unsigned char* buffer) {
    // Explicit byte-by-byte packing
    memcpy(buffer, &data->position, sizeof(float));
    memcpy(buffer + 4, &data->velocity, sizeof(float));
    // ... etc
}
```

## Error Handling

The library defines several error codes in `serial-comms.h`:

```c
// Error codes
ERROR_MEMORY_OVERRUN = -5
ERROR_INVALID_ARGUMENT = -4  
ERROR_CHECKSUM_MISMATCH = -3
ERROR_MALFORMED_MESSAGE = -2
ADDRESS_FILTERED = -1
SERIAL_PROTOCOL_SUCCESS = 0
```

### CRC Validation
```c
// Always validate CRC on received frames (Types 0 and 1)
int result = validate_crc(&rx_frame);
if (result != SERIAL_PROTOCOL_SUCCESS) {
    // Handle CRC error - discard frame or request retransmission
    return result;  // Will be ERROR_CHECKSUM_MISMATCH
}
```

### Address Validation
```c
// Address validation is handled automatically by frame_to_payload()
// It will return ADDRESS_FILTERED if the frame is not for this device
int result = frame_to_payload(&rx_frame, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &payload_msg);
if (result == ADDRESS_FILTERED) {
    // Frame not for this device - ignore
    return result;
}
```

### Bounds Checking
```c
// The library automatically validates memory bounds
int result = parse_base_serial_message(&payload_msg, &memory_buffer, &response);
if (result == ERROR_MEMORY_OVERRUN) {
    // Handle out-of-bounds access
    return result;
}
```

## Best Practices

### Performance Optimization
- Use type punning on embedded systems with known architecture
- Pre-allocate buffers to avoid dynamic memory allocation
- Implement circular buffers for continuous communication
- Consider DMA for high-throughput applications

### Reliability
- Always validate CRC when present
- Implement timeout mechanisms for response handling
- Use appropriate retry logic for failed communications
- Validate all memory accesses against device capabilities

### Code Organization
- Separate transport layer from protocol layer
- Use consistent naming conventions for addresses
- Document your memory map and data structures
- Implement proper error codes and handling

## Example Implementation

See `examples/example_uart_struct.c` for a complete working example demonstrating:
- Device configuration using typedef structs with type punning
- Using `index_of_field()` helper for automatic field indexing
- Master-slave communication flow with `frame_to_payload()` and `parse_general_message()`
- Buffer management with `buffer_t` structures
- Complete round-trip communication example

Key functions demonstrated:
- `create_read_struct_frame()` - Helper using `index_of_field()` and stores original read message
- `frame_to_payload()` - Frame layer to payload layer conversion
- `parse_general_message()` - High-level message parsing
- `parse_read_struct_reply()` - Reply parsing using original read message for proper offset calculation

## Common Pitfalls

1. **Buffer Structure**: Always initialize `buffer_t` with `.buf`, `.size`, and `.len` fields
2. **Message Types**: Use correct `serial_message_type_t` for your transport layer
3. **Address Confusion**: Use `get_complementary_address()` to get misc address from motor address
4. **Word Alignment**: Index values represent 32-bit word offsets, multiply by 4 for byte offsets
5. **Memory Management**: Use `PAYLOAD_ALIAS` for efficiency, `PAYLOAD_COPY` for safety
6. **Error Checking**: Always check return values against `SERIAL_PROTOCOL_SUCCESS`
7. **Read Reply Parsing**: Always pass the original `misc_read_message_t` to `parse_read_reply()` - it handles offset calculation automatically

## Integration Notes

- Link with both `dartt_protocol` and `dartt_checksum` libraries
- Ensure your build system includes the library headers in the include path
- The library is designed to be C11 compliant for maximum portability
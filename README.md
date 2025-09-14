# DARTT - Dual Address Real-Time Transport Protocol

## Overview

DARTT is a lightweight serial communication protocol that provides a simple, lightweight and low-overhead method of communicating with embedded systems. It's primary intended use case is in motor controllers for robots, but is useful as a general-purpose protocol for any simple communication network with a controller-peripheral structure. 

The protocol defines two primary frame types - an application defined 'motor' frame type, and a general purpose 'misc' frame type. These are generally accessed with a 'dual addressing' schema, where each peripheral responds to two different addresses - a 'motor' address and a 'misc' address. 

The 'motor' frame is an application defined tight/packed frame for highest possible bandwidth containing specialized command and response patterns (such as voltage, current, position, velocity, etc). The 'misc' frames are protocol defined, and function as block memory read and write messages. 

The motor frame and block memory layout are application specific, and occupy the 'application' layer. Below this layer is the 'protocol' layer - this defines the underlying structure of DARTT generic/misc read and write messages (with block index, payload, read/write bit, etc), detailed below. Below the 'protocol' layer is the 'frame' layer - this is transport protocol specific, and may contain an address and CRC (type 0), just a CRC (type 1), or neither an address nor CRC (type 2). 

The DARTT library is designed to be C language standards compliant - however, in embedded systems where the controller and peripheral endian-ness and bit-width match, type punning is the recommended way to define the block memory structure. This is especially true for the peripheral, which is most commonly an embedded system with fixed CPU architecture, and therefore will have a predictable memory layout. This is also the reason that DARTT is little-endian - for the most commonly used CPU architectures (ARM and x86), little-endian is the default.

## Integration into Your Project

### Required Files
Copy these files into your project:
- `src/dartt.c`
- `src/dartt.h`
- `src/checksum.c`
- `src/checksum.h`

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
make
```

### Running Unit Tests
The test suite can be run via ceedling
```bash
ceedling test:all
```

### Building Examples
The example code is located in the `examples` directory. To build:
```bash
cd build
make
```

The examples demonstrate:
- Basic message creation and parsing
- Read/write operations
- Error handling

## Usage Examples

### Basic Initialization
```c
#include "dartt.h"

comms_t comms = {0};  // Initialize your comms structure
```

### Creating a Read Message
```c
unsigned char msg_buf[256];
int msg_len = create_misc_read_message(
    device_address,    // Target device address
    start_index,       // Starting word index (32bit word offset)
    num_words,         // Number of words to read
    serial_write_buf,           // Buffer used to write out response
    sizeof(msg_buf)    // Size of buffer to prevent overrun
);
```

### Creating a Write Message
```c
unsigned char payload[] = {0x01, 0x02, 0x03, 0x04};
int msg_len = create_misc_write_message(
    device_address,    // Target device address
    start_index,       // Starting word index (32bit word offset)
    payload,           // Data to write
    sizeof(payload),   // Payload size
    msg_buf,           // Message buffer
    sizeof(msg_buf)    // Buffer size
);
```

### Parsing Incoming Messages
```c
unsigned char reply_buf[256];
int reply_len;
int result = parse_general_message(
    my_address,        // This device's address
    received_msg,      // Received message buffer
    msg_len,          // Received message length
    reply_buf,        // Reply buffer
    sizeof(reply_buf), // Reply buffer size
    &reply_len,       // Pointer to store reply length
    &comms            // Your comms structure
);
``` 


### DEBUGGING Unit Tests in vscode
Open the specific test file you want to run (i.e. test_serial_comms.c) in vscode. It will attempt to debug the build artifact named for the currently opened file. Hit F5 or debug - the test will run and debugging will be enabled. You may need to pre-run ceedling test:all before debugging in order to ensure the .out artifact is up to date.

### Message Structure:
```
Write Message:
    Full Serial: [Byte 0: address][Bytes 1-2: read/write bit (MSB), index (low 15)][Bytes 3 - (N-3): payload][Bytes (N-2)-(N-1): checksum]

```


### Protocol Definition:
``` 
This protocol has a layered structure. 

It is designed to have a flexible and easy to implement custom Application layer. The Application layer consists of an address On embedded systems, the application layer can be implemented with typedef structs (with packed attributes for different types), or
can be made standard-compliant with pointer arithmetic. The Application layer is simply implemented on a block of protected memory, which is written to and read from via the Payload layer. 
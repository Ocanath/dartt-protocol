# Serial Communication Library

A lightweight serial communication library for embedded systems that provides a structured way to handle read/write operations over serial interfaces.

## Integration into Your Project

### Required Files
Copy these files into your project:
- `src/serial-comms.c`
- `src/serial-comms.h`
- `src/serial-comms-struct.h`
- `src/checksum.c`
- `src/checksum.h`
- `src/serial-motor-comms.c` (if using motor-specific functionality)

### Customization
The library is designed to be customized through the `comms_t` struct defined in `serial-comms-struct.h`. You can extend this struct to include your specific data structures and communication needs.

Example of extending the struct:
```c
typedef struct {
    // Your custom data structures
    uint32_t custom_data;
    float sensor_readings[4];
    // ... other fields
} my_custom_data_t;

typedef struct {
    my_custom_data_t data;
    // ... other fields from the base comms_t struct
} comms_t;
```

## Building and Testing

### Prerequisites
- CMake (version 3.10 or higher)
- C compiler with C11 support

### Building the Project
```bash
mkdir build
cd build
cmake ..
make
```

### Running Unit Tests
The test suite can be run from the build directory:
```bash
cd build
make test
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
#include "serial-comms.h"

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
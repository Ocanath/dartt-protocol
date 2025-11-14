# DARTT Sync User Guide

## Table of Contents
1. [Introduction](#1-introduction)
2. [Core Concepts](#2-core-concepts)
3. [Setup and Initialization](#3-setup-and-initialization)
4. [The Three Core Functions](#4-the-three-core-functions)
5. [Advanced Usage Patterns](#5-advanced-usage-patterns)
6. [Common Pitfalls and Best Practices](#6-common-pitfalls-and-best-practices)
7. [Example: Complete Workflow](#7-example-complete-workflow)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Introduction

DARTT Sync provides high-level convenience functions for synchronizing memory between a controller device and peripheral devices over the DARTT protocol. Instead of manually constructing read/write messages, you maintain two local copies of the peripheral's memory layout and let DARTT Sync handle the protocol details.

**Philosophy**: The controller maintains:
- **`ctl_base`**: The "master copy" - what you WANT the peripheral to be
- **`periph_base`**: The "shadow copy" - what you BELIEVE the peripheral currently is

DARTT Sync automatically detects differences and synchronizes them efficiently.

---

## 2. Core Concepts

### 2.1 Memory Model

```
Controller Device Memory:
+------------------+          +------------------+
|   ctl_base       |          |  periph_base     |
|  (master copy)   |          | (shadow copy)    |
+------------------+          +------------------+
        |                            ^
        |                            |
        v                            |
    [DARTT Protocol] -------> Peripheral Device
         write                    read back
```

### 2.2 The dartt_sync_t Structure

```c
typedef struct dartt_sync_t
{
        unsigned char address;	 // Target peripheral address
		buffer_t ctl_base;			// Base of master control structure
		buffer_t periph_base;		 // Base of shadow copy structure
		serial_message_type_t msg_type;	// Message framing type
		buffer_t tx_buf;		// Transmission buffer
		buffer_t rx_buf;		 // Reception buffer
		int (*blocking_tx_callback)(unsigned char, buffer_t*, uint32_t timeout);	//Callback for (blocking) transmissions with a millisecond timeout
		int (*blocking_rx_callback)(buffer_t*, uint32_t timeout);		//Callback for (blocking) receptions with a millisecond timeout
		uint32_t timeout_ms;		// Communication timeout
}dartt_sync_t;
```

**Critical Requirements**:
- `ctl_base` and `periph_base` MUST have identical `size` fields
- `ctl_base.buf` and `periph_base.buf` MUST point to DIFFERENT memory locations
- Both structures should represent the same layout (typically same typedef struct, but any method of decoding a shared memory structure will work)
- All pointers must be 32-bit aligned (4-byte boundaries)

### 2.3 Buffer Sizing Constraints

**For CAN/CAN-FD**:
- **Standard CAN**: Set tx_buf and rx_buf to 8 bytes for proper operation
- **CAN-FD (IMPORTANT )**: For DLC > 8, message lengths follow a lookup table (not continuous integers). DARTT Sync does not handle LUT sizes for FD can, so so keep buffers ≤ 8 when doing block memory operations with CAN-FD.

---

## 3. Setup and Initialization

### 3.1 Define Your Memory Structure

Example:
```c
typedef struct motor_config_t {
    uint32_t device_id;         // Word 0
    uint32_t max_speed;         // Word 1
    uint32_t position_target;   // Word 2
    uint32_t current_position;  // Word 3
    uint32_t status_flags;      // Word 4
} motor_config_t;
```

### 3.2 Allocate Master and Shadow Copies

```c
// Master copy (what we want the motor to be)
motor_config_t motor_master = {
    .device_id = 0x1234,
    .max_speed = 1000,
    .position_target = 0,
    .current_position = 0,
    .status_flags = 0
};

// Shadow copy (what we believe the motor currently is)
motor_config_t motor_shadow = {0};  // Start zeroed
```

### 3.3 Implement Communication Callbacks

```c
int my_blocking_tx(unsigned char address, buffer_t* frame, uint32_t timeout_ms) {
    // Send frame->buf (length: frame->len) to specified address
    // Wait up to timeout_ms for transmission completion
    // Return DARTT_PROTOCOL_SUCCESS or error code

    // Example for CAN:
    // can_send(address, frame->buf, frame->len);
    // wait_for_tx_complete(timeout_ms);
    return DARTT_PROTOCOL_SUCCESS;
}

int my_blocking_rx(buffer_t* frame, uint32_t timeout_ms) {
    // Wait up to timeout_ms to receive a frame
    // Store received data in frame->buf
    // Set frame->len to number of bytes received
    // Return DARTT_PROTOCOL_SUCCESS or error code

    // IMPORTANT: This should return the RAW FRAME (not pre-processed)

    // Example for CAN:
    // if (can_receive(frame->buf, &frame->len, timeout_ms) == SUCCESS) {
    //     return DARTT_PROTOCOL_SUCCESS;
    // }
    return ERROR_TIMEOUT;
}
```

### 3.4 Initialize dartt_sync_t Structure

```c
// Communication buffers
unsigned char tx_buffer[64];
unsigned char rx_buffer[64];

// Initialize sync structure
dartt_sync_t motor_sync = {
    .address = 0x03,  // Motor's address (function uses complementary internally)

    .ctl_base = {
        .buf = (unsigned char*)&motor_master,
        .size = sizeof(motor_config_t),
        .len = 0  // Not used for base
    },

    .periph_base = {
        .buf = (unsigned char*)&motor_shadow,
        .size = sizeof(motor_config_t),
        .len = 0  // Not used for base
    },

    .msg_type = TYPE_SERIAL_MESSAGE,  // or TYPE_ADDR_MESSAGE, TYPE_ADDR_CRC_MESSAGE

    .tx_buf = {
        .buf = tx_buffer,
        .size = sizeof(tx_buffer),
        .len = 0
    },

    .rx_buf = {
        .buf = rx_buffer,
        .size = sizeof(rx_buffer),
        .len = 0
    },

    .blocking_tx_callback = my_blocking_tx,
    .blocking_rx_callback = my_blocking_rx,
    .timeout_ms = 100
};
```

---

## 4. The Three Core Functions

### 4.1 dartt_sync() - Differential Synchronization

**Purpose**: Scan for differences between master and shadow, then write+verify changes.

```c
int dartt_sync(buffer_t * ctl, dartt_sync_t * psync);
```

**How it works**:
1. Compares `ctl` region (from master) with corresponding `periph_base` region
2. Finds contiguous blocks of differences
3. For each difference: writes master→peripheral, reads back, verifies match
4. Updates shadow copy after successful verification

**Example - Sync a single field**:
```c
// Update a single field
motor_master.position_target = 5000;

// Create buffer alias to the field you want to sync
buffer_t target_field = {
    .buf = (unsigned char*)&motor_master.position_target,
    .size = sizeof(motor_master.position_target),
    .len = sizeof(motor_master.position_target)
};

// Sync just this field
int rc = dartt_sync(&target_field, &motor_sync);
if (rc == DARTT_PROTOCOL_SUCCESS) {
    // motor_shadow.position_target now matches motor_master.position_target
    // Peripheral device has been updated and verified
}
```

**Example - Sync entire structure**:
```c
buffer_t full_struct = {
    .buf = (unsigned char*)&motor_master,
    .size = sizeof(motor_config_t),
    .len = sizeof(motor_config_t)
};
rc = dartt_sync(&full_struct, &motor_sync);
```

**When to use**:
- You want verification that the write succeeded
- You need to detect peripheral write failures
- Updating critical configuration that must be confirmed

### 4.2 dartt_write_multi() - Write Without Verification

**Purpose**: Write data from master to peripheral (no read-back verification).

```c
int dartt_write_multi(buffer_t * ctl, dartt_sync_t * psync);
```

**Key differences from dartt_sync()**:
- Does NOT read back or verify
- Does NOT update shadow copy
- Faster but less safe
- Automatically breaks large writes into chunks

**Example**:
```c
// Write entire configuration without verification
buffer_t config = {
    .buf = (unsigned char*)&motor_master,
    .size = sizeof(motor_config_t),
    .len = sizeof(motor_config_t)
};

int rc = dartt_write_multi(&config, &motor_sync);
// Data sent, but shadow NOT updated!
```

**When to use**:
- High-frequency updates where verification would slow things down
- Non-critical data where occasional write failures are acceptable
- You plan to read back later to verify

### 4.3 dartt_read_multi() - Read Into Shadow Copy

**Purpose**: Read data from peripheral into shadow copy.

```c
int dartt_read_multi(buffer_t * ctl, dartt_sync_t * psync);
```

**⚠️ CONFUSING PARAMETER PATTERN** (Most common source of errors):
- The `ctl` parameter specifies **WHAT** to read (the memory region)
- Results go into `psync->periph_base` at the corresponding offset
- The `ctl` parameter is **NOT** the destination!

**Example - Reading position from peripheral**:
```c
// Read current position from peripheral
buffer_t position_region = {
    .buf = (unsigned char*)&motor_master.current_position,  // Specifies WHAT to read
    .size = sizeof(motor_master.current_position),
    .len = sizeof(motor_master.current_position)
};

int rc = dartt_read_multi(&position_region, &motor_sync);
if (rc == DARTT_PROTOCOL_SUCCESS) {
    // Result is now in motor_shadow.current_position
    // (NOT in motor_master.current_position!)
    printf("Position: %d\n", motor_shadow.current_position);

    // If you want to update master, do it explicitly:
    motor_master.current_position = motor_shadow.current_position;
}
```

**When to use**:
- Polling sensor data from peripheral
- Reading status/feedback information
- Verifying peripheral state

---

## 5. Advanced Usage Patterns

### 5.1 Periodic Polling

```c
void update_motor_status(void) {
    // Read status fields from peripheral (position + status_flags)
    buffer_t status_region = {
        .buf = (unsigned char*)&motor_master.current_position,
        .size = 2 * sizeof(uint32_t),  // Read two consecutive fields
        .len = 2 * sizeof(uint32_t)
    };

    dartt_read_multi(&status_region, &motor_sync);
    // Results in motor_shadow.current_position and motor_shadow.status_flags

    // Copy to master if needed
    motor_master.current_position = motor_shadow.current_position;
    motor_master.status_flags = motor_shadow.status_flags;
}
```

### 5.2 Batch Updates

```c
void configure_motor(uint32_t speed, uint32_t target) {
    // Update multiple fields in master
    motor_master.max_speed = speed;
    motor_master.position_target = target;

    // Sync both fields at once
    buffer_t config_region = {
        .buf = (unsigned char*)&motor_master.max_speed,
        .size = 2 * sizeof(uint32_t),
        .len = 2 * sizeof(uint32_t)
    };

    dartt_sync(&config_region, &motor_sync);
    // Both fields written and verified as a batch
}
```

### 5.3 Initialize Shadow Copy from Peripheral

```c
void initialize_from_peripheral(void) {
    // Read entire peripheral state into shadow
    buffer_t full_read = {
        .buf = (unsigned char*)&motor_master,  // Specifies full structure
        .size = sizeof(motor_config_t),
        .len = sizeof(motor_config_t)
    };

    dartt_read_multi(&full_read, &motor_sync);
    // motor_shadow now contains peripheral's current state

    // Copy to master to start synchronized
    memcpy(&motor_master, &motor_shadow, sizeof(motor_config_t));
}
```

---

## 6. Common Pitfalls and Best Practices

### ❌ Pitfall 1: Confusing ctl Parameter in read_multi

```c
// WRONG: Expecting result in motor_master
buffer_t position = {
    .buf = (unsigned char*)&motor_master.current_position,
    .size = 4, .len = 4
};
dartt_read_multi(&position, &motor_sync);
uint32_t value = motor_master.current_position;  // ❌ WRONG! Still old value
```

```c
// CORRECT: Result goes to motor_shadow
dartt_read_multi(&position, &motor_sync);
uint32_t value = motor_shadow.current_position;  // ✅ Correct!
```

### ❌ Pitfall 2: Buffer Pointer Outside Base

```c
// WRONG: ctl not within ctl_base
motor_config_t temp_config;
buffer_t bad_buf = {
    .buf = (unsigned char*)&temp_config,  // ❌ Not part of motor_master!
    .size = sizeof(motor_config_t),
    .len = sizeof(motor_config_t)
};
dartt_sync(&bad_buf, &motor_sync);  // Returns ERROR_INVALID_ARGUMENT
```

```c
// CORRECT: ctl points within motor_master
buffer_t good_buf = {
    .buf = (unsigned char*)&motor_master.position_target,  // ✅ Inside motor_master
    .size = sizeof(uint32_t),
    .len = sizeof(uint32_t)
};
dartt_sync(&good_buf, &motor_sync);  // Works correctly
```

### ❌ Pitfall 3: Misaligned Pointers

```c
// WRONG: Pointer not 32-bit aligned
buffer_t misaligned = {
    .buf = (unsigned char*)&motor_master + 1,  // ❌ +1 byte, not aligned!
    .size = 4, .len = 4
};
dartt_sync(&misaligned, &motor_sync);  // Returns ERROR_INVALID_ARGUMENT
```

### ❌ Pitfall 4: Using write_multi Then Expecting Shadow to Update

```c
// WRONG: Assuming shadow updates after write_multi
motor_master.position_target = 1000;
buffer_t target = {
    .buf = (unsigned char*)&motor_master.position_target,
    .size = 4, .len = 4
};
dartt_write_multi(&target, &motor_sync);
// motor_shadow.position_target is still old value! ❌
```

```c
// CORRECT: Use dartt_sync for automatic shadow update
dartt_sync(&target, &motor_sync);
// motor_shadow.position_target now matches motor_master ✅

// Or manually update shadow after write_multi
dartt_write_multi(&target, &motor_sync);
motor_shadow.position_target = motor_master.position_target;  // Manual update
```

### ✅ Best Practice 1: Validate Field Pointers

```c
// Use index_of_field to verify field is within structure
int idx = index_of_field(&motor_master.position_target,
                         &motor_master,
                         sizeof(motor_config_t));
if (idx < 0) {
    // Handle error - field not within structure
}
```

### ✅ Best Practice 2: Initialize Properly

```c
// On startup, read peripheral state first
dartt_read_multi(&full_struct, &motor_sync);

// Copy shadow to master for initial sync
memcpy(&motor_master, &motor_shadow, sizeof(motor_config_t));

// Now modify master and sync changes
motor_master.control_mode = MODE_ACTIVE;
dartt_sync(&mode_field, &motor_sync);
```

### ✅ Best Practice 3: Use Appropriate Function for Task

```c
// For critical config: use dartt_sync (with verification)
dartt_sync(&critical_field, &motor_sync);

// For high-frequency commands: use dartt_write_multi (faster)
dartt_write_multi(&position_cmd, &motor_sync);

// For reading sensor data: use dartt_read_multi
dartt_read_multi(&sensor_region, &motor_sync);
```

---

## 7. Example: Complete Workflow

```c
#include "dartt_sync.h"
#include <string.h>

// 1. Define structure
typedef struct {
    uint32_t id;
    uint32_t speed;
    uint32_t position;
    uint32_t status;
} device_t;

// 2. Allocate copies
device_t master = {.id = 1, .speed = 100, .position = 0, .status = 0};
device_t shadow = {0};

// 3. Communication buffers
unsigned char tx[64], rx[64];

// 4. Callbacks (implementation depends on your hardware)
int hw_tx(unsigned char addr, buffer_t* buf, uint32_t timeout) {
    // Your TX implementation (CAN, UART, etc.)
    return DARTT_PROTOCOL_SUCCESS;
}

int hw_rx(buffer_t* buf, uint32_t timeout) {
    // Your RX implementation
    return DARTT_PROTOCOL_SUCCESS;
}

// 5. Initialize sync structure
dartt_sync_t sync = {
    .address = 0x05,
    .ctl_base = {(unsigned char*)&master, sizeof(device_t), 0},
    .periph_base = {(unsigned char*)&shadow, sizeof(device_t), 0},
    .msg_type = TYPE_SERIAL_MESSAGE,
    .tx_buf = {tx, sizeof(tx), 0},
    .rx_buf = {rx, sizeof(rx), 0},
    .blocking_tx_callback = hw_tx,
    .blocking_rx_callback = hw_rx,
    .timeout_ms = 100
};

// 6. Initialization sequence
void init_sequence(void) {
    // Read current peripheral state
    buffer_t full_struct = {
        .buf = (unsigned char*)&master,
        .size = sizeof(device_t),
        .len = sizeof(device_t)
    };
    dartt_read_multi(&full_struct, &sync);

    // Copy shadow to master
    memcpy(&master, &shadow, sizeof(device_t));

    // Now we're synchronized!
}

// 7. Usage in main loop
void main_loop(void) {
    // Write new speed
    master.speed = 500;
    buffer_t speed_buf = {
        .buf = (unsigned char*)&master.speed,
        .size = 4, .len = 4
    };
    dartt_sync(&speed_buf, &sync);

    // Read current position and status
    buffer_t status_buf = {
        .buf = (unsigned char*)&master.position,
        .size = 8, .len = 8  // Read 2 fields (position + status)
    };
    dartt_read_multi(&status_buf, &sync);

    // Result is in shadow!
    printf("Position: %u, Status: %u\n", shadow.position, shadow.status);
}
```

---

## 8. Troubleshooting

### ERROR_INVALID_ARGUMENT

**Possible causes**:
- `ctl->buf` is not within `psync->ctl_base` range
- Pointer is not 32-bit aligned (not on 4-byte boundary)
- `ctl->size` is not a multiple of 4

**Solutions**:
```c
// Check alignment
if (((uintptr_t)ctl->buf) % 4 != 0) {
    // Pointer not aligned!
}

// Verify within base
if (ctl->buf < psync->ctl_base.buf ||
    ctl->buf >= psync->ctl_base.buf + psync->ctl_base.size) {
    // Pointer outside base range!
}
```

### ERROR_MEMORY_OVERRUN

**Possible causes**:
- `ctl_base.size != periph_base.size`
- `ctl->buf + ctl->len` exceeds `ctl_base` bounds
- `tx_buf` or `rx_buf` too small for message overhead

**Solutions**:
```c
// Ensure bases match
assert(psync->ctl_base.size == psync->periph_base.size);

// Check buffer bounds
if (ctl->buf + ctl->len > psync->ctl_base.buf + psync->ctl_base.size) {
    // Buffer extends beyond base!
}

// Minimum buffer sizes (for TYPE_ADDR_CRC_MESSAGE):
// tx_buf: 2 (overhead) + 4 (min payload) = 6 bytes minimum
// rx_buf: Same as tx_buf
```

### ERROR_SYNC_MISMATCH

**Cause**: Peripheral didn't write correctly - read-back verification failed.

**Solutions**:
- Increase timeout (peripheral might be slow)
- Check peripheral implementation
- Verify peripheral address is correct
- Check for electrical/communication issues

### ERROR_MALFORMED_MESSAGE / ERROR_TIMEOUT

**Cause**: No response from peripheral, or malformed response.

**Solutions**:
```c
// Verify rx callback returns frame->len > 0
int my_rx(buffer_t* frame, uint32_t timeout) {
    int rc = can_receive(frame->buf, &frame->len, timeout);
    if (rc == SUCCESS && frame->len > 0) {  // ✅ Check len > 0
        return DARTT_PROTOCOL_SUCCESS;
    }
    return ERROR_TIMEOUT;
}

// Check address is correct (remember complementary addressing!)
unsigned char actual_addr = dartt_get_complementary_address(psync->address);
```

### Silent Failures / Unexpected Behavior

**Debug checklist**:
1. Verify `ctl_base` and `periph_base` point to different memory locations
2. Check that structures are 32-bit aligned (size is multiple of 4)
3. Ensure callbacks actually send/receive data
4. Use a logic analyzer to verify physical layer
5. Add logging to callbacks to trace message flow

---

## Summary

**Key Takeaways**:
1. **Two copies**: Maintain master (ctl_base) and shadow (periph_base)
2. **dartt_sync()**: For verified writes with read-back
3. **dartt_write_multi()**: For fast writes without verification
4. **dartt_read_multi()**: Reads into shadow (NOT into ctl buffer!)
5. **Always** ensure 32-bit alignment
6. **Initialize** by reading peripheral state first

For more details on the protocol itself, see [DARTT.md](DARTT.md).

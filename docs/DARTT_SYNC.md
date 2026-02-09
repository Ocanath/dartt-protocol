# DARTT Sync User Guide

## Table of Contents

1. [Introduction](#1-introduction)
2. [Design Philosophy](#2-design-philosophy)
3. [The dartt_sync_t Structure](#3-the-dartt_sync_t-structure)
4. [The Three Core Functions](#4-the-three-core-functions)
5. [Understanding the ctl Parameter Pattern](#5-understanding-the-ctl-parameter-pattern)
6. [Common Pitfalls](#6-common-pitfalls)
7. [Troubleshooting](#7-troubleshooting)

---

## 1. Introduction

DARTT Sync provides high-level convenience functions for synchronizing memory between a controller device and peripheral devices over the DARTT protocol. Instead of manually constructing read/write messages, you maintain two local copies of the peripheral's memory layout and let DARTT Sync handle the protocol details.

---

## 2. Design Philosophy

### 2.1 The Two-Copy Model

DARTT Sync is built around maintaining **two local copies** of the peripheral's memory structure:

```
Controller Device Memory:
+------------------+          +------------------+
|   ctl_base       |          |  periph_base     |
|  (controller copy)   |          | (shadow copy)    |
+------------------+          +------------------+
        |                            ^
        |                            |
        v                            |
    [DARTT Protocol] -------> Peripheral Device
         write                    read back
```

**Control Base (Controller Copy)**:

- Represents what you **WANT** the target device to be
- You modify this when you want to change target memory state
- Source of truth for desired configuration

**Peripheral Base (Shadow Copy)**:

- Represents what you **BELIEVE** the target currently is
- Updated after successful writes or reads
- Used for comparison to detect changes

This separation allows DARTT Sync to:

- Automatically detect only what has changed
- Verify writes by comparing read-back data
- Maintain consistency between controller and peripheral

### 2.2 Operating on Regions

All DARTT Sync functions operate on **regions** within these base structures. You don't always have to sync the entire structure - you can target specific fields or subsections. This enables:

- Efficient updates (only sync what changed)
- Flexible access patterns (for instance, call `dartt_sync` on a write-only section of the target memory layout, and `dartt_read_multi` on a different read-only section of the target memory layout)

The `ctl` parameter in all functions specifies which region to operate on. It must always point within `ctl_base`, or the function will return an overflow error.

---

## 3. The `dartt_sync_t` Structure

### 3.1 Structure Overview

```c
typedef struct dartt_sync_t
{
    unsigned char address;           // Target peripheral address
    dartt_buffer_t ctl_base;              // Base of controller structure
    dartt_buffer_t periph_base;           // Base of shadow copy structure
    serial_message_type_t msg_type; // Message framing type
    dartt_buffer_t tx_buf;                // Transmission buffer
    dartt_buffer_t rx_buf;                // Reception buffer
    int (*blocking_tx_callback)(unsigned char, dartt_buffer_t*, uint32_t timeout);
    int (*blocking_rx_callback)(dartt_buffer_t*, uint32_t timeout);
    uint32_t timeout_ms;            // Communication timeout
}dartt_sync_t;
```

### 3.2 Critical Requirements

**Base Buffer Constraints**:

- `ctl_base.size` and `periph_base.size` **MUST be identical**
- `ctl_base.buf` and `periph_base.buf` **MUST point to different memory**
- Both should represent the same layout (typically same struct type)
- All pointers must be **32-bit aligned** (4-byte boundaries)

**Why these requirements matter**:

- Same size: Functions calculate offsets assuming parallel structures
- Different memory: Prevents comparing a structure to itself, preserve the shadow-copy paradigm
- 32-bit alignment: DARTT protocol uses 32-bit word-based indexing

### Note - Buffer Sizing for CAN/CAN-FD

**Standard CAN**: Set `tx_buf` and `rx_buf` to 8 bytes.

**CAN-FD**: For DLC > 8, message lengths follow a non-continuous lookup table. DARTT Sync assumes continuous integer sizing, so keep buffers ≤ 8 bytes for block memory operations on CAN-FD.

### 3.4 Callback Expectations

`blocking_tx_callback`

- Receives a fully burdened DARTT frame to transmit
- If any additional encoding is used (byte stuffing, etc.) it should be done here before transmission
- Should block until transmission completes or timeout expires
- Returns `DARTT_PROTOCOL_SUCCESS` or error code

`blocking_rx_callback`:

- Should block until a fully burdened DARTT reply frame is received or timeout expires
- Must set `frame->len` to the number of bytes received
- Returns `DARTT_PROTOCOL_SUCCESS` or error code

---

## 4. The Three Core Functions

### 4.1 `dartt_sync()` - Differential Synchronization with Verification

```c
int dartt_sync(dartt_buffer_t * ctl, dartt_sync_t * psync);
```

**Purpose**: Compare controller and shadow, write differences to peripheral, verify with read-back.

**Operation**:

1. Scans the `ctl` region comparing controller (ctl_base) to shadow (periph_base)
2. Identifies contiguous blocks of differences
3. For each difference:
   - Writes controller copy content to the target device
   - Reads back from target device into shadow copy
   - Checks shadow copy for discrepancies with controller copy. Returns `ERROR_SYNC_MISMATCH` if there is any mismatch.

**Automatic chunking**: If a mismatch region exceeds `tx_buf` capacity, it's automatically split into multiple write/read operations.

**When to use**:

- Critical configuration that must be verified
- Situations where write failures must be detected
- When you need confirmation the peripheral accepted the write

**Key behavior**: Shadow copy is updated **only after verification**, ensuring it accurately reflects peripheral state.

### 4.2 dartt_write_multi() - Write Without Verification

```c
int dartt_write_multi(dartt_buffer_t * ctl, dartt_sync_t * psync);
```

**Purpose**: Write data from controller to peripheral without read-back verification.

**Operation**:

1. Writes the entire `ctl` region to peripheral
2. Automatically splits into multiple messages if needed
3. **Does NOT** read back
4. **Does NOT** update shadow copy

**When to use**:

- High-frequency command updates where verification adds latency
- Non-critical data where occasional failures are acceptable
- Situations where you'll verify later with a read

**Important**: Since shadow isn't updated, subsequent `dartt_sync()` calls will see mismatches unless you manually update the shadow or perform a read.

### 4.3 dartt_read_multi() - Read Into Shadow Copy

```c
int dartt_read_multi(dartt_buffer_t * ctl, dartt_sync_t * psync);
```

**Purpose**: Read data from target/peripheral into shadow copy.

**Operation**:

1. Calculates offset of `ctl` region within `ctl_base`
2. Reads corresponding region from peripheral
3. Stores result in `periph_base` at the matching offset
4. Automatically splits into multiple reads if needed

**When to use**:

- Polling sensor data or status from peripheral
- Initializing shadow copy from peripheral state
- Verifying peripheral state after writes

---

## 5. Understanding the ctl Parameter Pattern

### 5.1 The Source of Confusion

The `ctl` parameter behaves differently depending on context, which is the **most common source of errors** when using DARTT Sync.

**For dartt_sync() and dartt_write_multi()**:

- `ctl` specifies **WHAT to send**
- Data comes **FROM** the `ctl` buffer
- Straightforward: you're writing what `ctl` points to

**For dartt_read_multi()**:

- `ctl` specifies **WHAT to read** (the region/offset)
- Data goes **TO** `periph_base`, **NOT** to `ctl`!
- `ctl` is only used to calculate the offset

### 5.2 Why This Design?

This design maintains API consistency - the controller copy is the record of your desired peripheral state, while the shadow copy is the most up-to-date record of what the actual state of the peripheral is. The `ctl` parameter is used to specify "which region of the structure" to operate on - since `ctl_base` (controller copy) and `periph_base` (shadow copy) have identical layouts, any subset of `ctl_base` has a corresponding subset of `periph_base`.

This behavior has to be maintained specifically for `dartt_read_multi`, because this function has multiple potential uses. In a situation where `dartt_read_multi` is polling 'read-only' sensor data, it would be logical to copy the result to the controller copy automatically. However, in a situation where `dartt_read_multi` is being used to verify a previous `dartt_write_multi` operation, this would be a destructive operation. To cover both use cases, a convenience function `dartt_update_controller()` can be called after dartt_read_multi with the same `ctl` parameter so that the controller copy is synchronized.

### 5.3 Practical Implications

**After calling dartt_read_multi()**:

- Check results in `periph_base`, not `ctl_base`

- If you want to synchronize the controller copy with the current peripheral state, explicitly copy the corresponding region of shadow → controller using the helper function `dartt_update_controller()`. An example:
  
  ```c
  dartt_read_multi(&region, &sync);    //read into periph_base based on 'region'
  dartt_update_controller(&region, &sync);    //copy the result we just obtained into ctl_base
  ```

---

## 7. Troubleshooting

### ERROR_INVALID_ARGUMENT

**Possible causes**:

- `ctl->buf` not within `psync->ctl_base` range
- Pointer not 32-bit aligned
- `ctl->size` not a multiple of 4

### ERROR_MEMORY_OVERRUN

**Possible causes**:

- `ctl_base.size` ≠ `periph_base.size`
- `ctl->buf + ctl->len` exceeds `ctl_base` bounds
- `tx_buf` or `rx_buf` too small for message overhead
- Calculated write would exceed `periph_base` bounds

### ERROR_SYNC_MISMATCH

**Cause**: Read-back verification failed - peripheral didn't store the written value correctly.

**Possible reasons**:

- Peripheral has read-only fields (can't be written)
- Peripheral modified value (e.g., clamping to valid range)
- Transmission error corrupted the write
- Peripheral is malfunctioning

The application is responsible for managing this error - can be ignored, trigger a retransmission pattern if due to a physical error, etc.

### ERROR_MALFORMED_MESSAGE / ERROR_TIMEOUT

**Cause**: No response from peripheral, or response couldn't be parsed.

**Possible reasons**:

- Peripheral not connected or powered
- Address mismatch (wrong peripheral address)
- Callback returning incorrect data
- Timeout too short for communication medium

---

## Summary

**Core Concepts**:

1. Maintain two copies: **controller** (ctl_base) and **shadow** (periph_base)
2. Functions operate on **regions** within these bases
3. The `ctl` parameter specifies **which region**, with behavior depending on function

**Function Selection**:

- **dartt_sync()**: Verified writes with automatic shadow update - safest option
- **dartt_write_multi()**: Fast unverified writes - use for high-frequency commands
- **dartt_read_multi()**: Read peripheral state into shadow - **results go to periph_base**

**Critical Requirements**:

- 32-bit alignment for all pointers
- Identical sizes for ctl_base and periph_base
- Callbacks return raw frames, not processed payloads
- For CAN: keep buffers ≤ 8 bytes

For protocol details and message formats, see [DARTT.md](DARTT.md).

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
|  (master copy)   |          | (shadow copy)    |
+------------------+          +------------------+
        |                            ^
        |                            |
        v                            |
    [DARTT Protocol] -------> Peripheral Device
         write                    read back
```

**Control Base (Master Copy)**:

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
    buffer_t ctl_base;              // Base of master control structure
    buffer_t periph_base;           // Base of shadow copy structure
    serial_message_type_t msg_type; // Message framing type
    buffer_t tx_buf;                // Transmission buffer
    buffer_t rx_buf;                // Reception buffer
    int (*blocking_tx_callback)(unsigned char, buffer_t*, uint32_t timeout);
    int (*blocking_rx_callback)(buffer_t*, uint32_t timeout);
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
int dartt_sync(buffer_t * ctl, dartt_sync_t * psync);
```

**Purpose**: Compare master and shadow, write differences to peripheral, verify with read-back.

**Operation**:

1. Scans the `ctl` region comparing master (ctl_base) to shadow (periph_base)
2. Identifies contiguous blocks of differences
3. For each difference:
   - Writes controller copy content to the target device
   - Reads back from target device into shadow copy
   - Checks shadow copy for discrepancies with controller copy. Returns `ERROR_SYNC_MISMATCH` if there is any mismatch.
   - 
4. 

**Automatic chunking**: If a mismatch region exceeds `tx_buf` capacity, it's automatically split into multiple write/read operations.

**When to use**:

- Critical configuration that must be verified
- Situations where write failures must be detected
- When you need confirmation the peripheral accepted the write

**Key behavior**: Shadow copy is updated **only after verification**, ensuring it accurately reflects peripheral state.

### 4.2 dartt_write_multi() - Write Without Verification

```c
int dartt_write_multi(buffer_t * ctl, dartt_sync_t * psync);
```

**Purpose**: Write data from master to peripheral without read-back verification.

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
int dartt_read_multi(buffer_t * ctl, dartt_sync_t * psync);
```

**Purpose**: Read data from peripheral into shadow copy.

**Operation**:

1. Calculates offset of `ctl` region within `ctl_base`
2. Reads corresponding region from peripheral
3. Stores result in `periph_base` at the matching offset
4. Automatically splits into multiple reads if needed

**When to use**:

- Polling sensor data or status from peripheral
- Initializing shadow copy from peripheral state
- Verifying peripheral state after writes

**Critical note**: See section 5 for detailed explanation of the confusing parameter pattern.

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

This design maintains API consistency - all functions use `ctl` to specify "which region of the structure" to operate on. However, for reads:

- You can't read directly into `ctl_base` (it's the master copy you control)
- Results must go into `periph_base` (the shadow reflecting peripheral state)

The `ctl` pointer essentially says "I want to operate on the field at THIS offset", and the function translates that to the corresponding offset in `periph_base`.

### 5.3 Practical Implications

**After calling dartt_read_multi()**:

- Check results in `periph_base`, not `ctl_base`
- If you want master to reflect peripheral state, explicitly copy shadow → master
- Don't assume anything about `ctl_base` changing

**Common mistake**: Reading a value and checking the wrong buffer for results.

---

## 6. Common Pitfalls

### 6.1 Reading and Expecting Results in Master

**Problem**: After `dartt_read_multi()`, checking the `ctl_base` field instead of `periph_base`.

**Why it fails**: Read results go to shadow (periph_base), not master (ctl_base).

**Solution**: Always access `periph_base` after reads. If you need the value in master, explicitly copy it.

### 6.2 Buffer Pointer Outside Base Range

**Problem**: Creating a `ctl` buffer that points to memory not within `ctl_base`.

**Why it fails**: Functions calculate field offsets assuming `ctl` is within `ctl_base`. Invalid offsets cause `ERROR_INVALID_ARGUMENT`.

**Solution**: Always ensure `ctl->buf` is within the range `[ctl_base.buf, ctl_base.buf + ctl_base.size)`.

### 6.3 Misaligned Pointers

**Problem**: Pointing to a non-4-byte-aligned address.

**Why it fails**: DARTT uses 32-bit word indexing. Misaligned pointers produce incorrect field indices.

**Solution**: Only point to fields that start on 4-byte boundaries. Structures with proper packing naturally satisfy this.

### 6.4 Using write_multi() Then Expecting Sync to Work

**Problem**: Using `dartt_write_multi()`, then being surprised when `dartt_sync()` re-transmits the same data.

**Why it fails**: `write_multi()` doesn't update shadow, so `dartt_sync()` sees master ≠ shadow and thinks it needs to write.

**Solution**: Either:

- Use `dartt_sync()` if you want automatic shadow updates
- Manually update shadow after `write_multi()`
- Use `dartt_read_multi()` to refresh shadow from peripheral

### 6.5 Mismatched Base Sizes

**Problem**: `ctl_base.size` ≠ `periph_base.size`.

**Why it fails**: Functions assume parallel structures. Offset calculations break with mismatched sizes, causing `ERROR_MEMORY_OVERRUN`.

**Solution**: Always initialize both bases to point to structures of identical size.

---

## 7. Troubleshooting

### ERROR_INVALID_ARGUMENT

**Possible causes**:

- `ctl->buf` not within `psync->ctl_base` range
- Pointer not 32-bit aligned
- `ctl->size` not a multiple of 4
- NULL pointers in critical fields

**Debug approach**:

- Verify `ctl->buf >= ctl_base.buf` and `ctl->buf < ctl_base.buf + ctl_base.size`
- Check `((uintptr_t)ctl->buf) % 4 == 0` for alignment
- Ensure structure size is multiple of `sizeof(uint32_t)`

### ERROR_MEMORY_OVERRUN

**Possible causes**:

- `ctl_base.size` ≠ `periph_base.size`
- `ctl->buf + ctl->len` exceeds `ctl_base` bounds
- `tx_buf` or `rx_buf` too small for message overhead
- Calculated write would exceed `periph_base` bounds

**Debug approach**:

- Add assertion: `assert(ctl_base.size == periph_base.size)`
- Verify buffer doesn't extend beyond base: `ctl->buf + ctl->len <= ctl_base.buf + ctl_base.size`
- Check minimum buffer sizes accommodate overhead + at least one 4-byte word

### ERROR_SYNC_MISMATCH

**Cause**: Read-back verification failed - peripheral didn't store the written value correctly.

**Possible reasons**:

- Peripheral has read-only fields (can't be written)
- Peripheral modified value (e.g., clamping to valid range)
- Transmission error corrupted the write
- Peripheral is malfunctioning

**Debug approach**:

- Verify the field is actually writable on the peripheral
- Check for value constraints in peripheral firmware
- Use logic analyzer to verify transmission integrity
- Increase timeout in case peripheral is slow to process writes

### ERROR_MALFORMED_MESSAGE / ERROR_TIMEOUT

**Cause**: No response from peripheral, or response couldn't be parsed.

**Possible reasons**:

- Peripheral not connected or powered
- Address mismatch (wrong peripheral address)
- Callback returning incorrect data
- Timeout too short for communication medium

**Debug approach**:

- Verify `rx_callback` sets `frame->len > 0` on successful reception
- Check address is correct (remember DARTT uses complementary addressing internally)
- Ensure callbacks return raw frames, not pre-processed data
- Increase `timeout_ms` for slower communication links

### Silent Failures / Unexpected Behavior

**Debug checklist**:

1. Verify `ctl_base.buf` ≠ `periph_base.buf` (different memory locations)
2. Ensure structures are 32-bit aligned in size
3. Confirm callbacks actually send/receive data (add logging)
4. Check physical layer with logic analyzer or oscilloscope
5. Verify peripheral firmware is running and responding
6. Use `index_of_field()` to validate field pointers before sync operations

---

## Summary

**Core Concepts**:

1. Maintain two copies: **master** (ctl_base) and **shadow** (periph_base)
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

For working code examples, see the `examples/` directory.

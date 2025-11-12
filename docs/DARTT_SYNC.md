# DARTT Sync

## Introduction

In addition to the base protocol handler implemented in (`dartt.c`), `dartt_sync.c` provides convenience methods for synchronizing between two devices. 

The general philosophy of DARTT Sync is for a DARTT controller device to maintain a 'controller' copy and a 'peripheral' copy of the peripheral device's memory layout. To update the peripheral, you simply need to modify the controller copy and call `dartt_sync`, and it will scan for differences between the local controller and local peripheral copies, then prepare and send a series of write/read DARTT messages of minimal size to update the true peripheral device. If multiple locations 

## Library Usage

The key to using DARTT Sync is the `dartt_sync_t` type. 



## TODO DOCUMENTATION
ADDRESS THE FOLLOWING AMBIGUITIES:

1. What is 'base' in the dartt_sync_t struct? why pass the controller as an argument as well as a separate pointer to it in dartt_sync_t? (it's so you can point to specific regions of the controller rather than the whole controller struct)

1. In CAN/CAN FD, you have to set the txbuf and rxbuf sizes to 8 for sync to work properly - it automatically sets the message size in integers, and for FD can above 8 the domain of possible message lengths is not positive integers (it's a LUT). For standard can this should be a non-issue

1. Clarify blocking_rx_read_callback expected behavior - should it return a *payload* message (all overhead removed), rather than the raw payload? or should it just return the raw frame? (it should return the raw frame)


#ifndef DARTT_SYNC_H  
#define DARTT_SYNC_H
#include <stdint.h>
#include <stddef.h>
#include "dartt.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct dartt_sync_t
{
        unsigned char address;	//address of the peripheral target
		buffer_t ctl_base;			//buffer alias/reference to the base of the master copy/controller copy structure. This must always point to the BASE of the memory.
		buffer_t periph_base;
		serial_message_type_t msg_type;	//peripheral target message type
		buffer_t tx_buf;		//buffer for tx payloads
		buffer_t rx_buf;		//buffer for rx payloads
		int (*blocking_tx_callback)(unsigned char, buffer_t*, uint32_t timeout);
		int (*blocking_rx_callback)(buffer_t*, uint32_t timeout);
		uint32_t timeout_ms;
}dartt_sync_t;



int dartt_sync(buffer_t * ctl, dartt_sync_t * psync);
int dartt_ctl_write(buffer_t * ctl, dartt_sync_t * psync);
int dartt_ctl_read(buffer_t * ctl, dartt_sync_t * psync);
int dartt_read_multi(buffer_t * ctl, dartt_sync_t * psync);
int dartt_write_multi(buffer_t * ctl, dartt_sync_t * psync);

#ifdef __cplusplus
}
#endif


#endif


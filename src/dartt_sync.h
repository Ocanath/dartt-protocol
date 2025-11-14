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
        unsigned char address;	 // Target peripheral address
		buffer_t ctl_base;			// Base of controller control structure
		buffer_t periph_base;		 // Base of shadow copy structure
		serial_message_type_t msg_type;	// Message framing type
		buffer_t tx_buf;		// Transmission buffer
		buffer_t rx_buf;		 // Reception buffer
		int (*blocking_tx_callback)(unsigned char, buffer_t*, uint32_t timeout);	//Callback for (blocking) transmissions with a millisecond timeout
		int (*blocking_rx_callback)(buffer_t*, uint32_t timeout);		//Callback for (blocking) receptions with a millisecond timeout
		uint32_t timeout_ms;		// Communication timeout
}dartt_sync_t;



int dartt_sync(buffer_t * ctl, dartt_sync_t * psync);
int dartt_ctl_write(buffer_t * ctl, dartt_sync_t * psync);
int dartt_ctl_read(buffer_t * ctl, dartt_sync_t * psync);
int dartt_read_multi(buffer_t * ctl, dartt_sync_t * psync);
int dartt_write_multi(buffer_t * ctl, dartt_sync_t * psync);
int dartt_update_controller(buffer_t * ctl, dartt_sync_t * psync);

#ifdef __cplusplus
}
#endif


#endif


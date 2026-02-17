#ifndef DARTT_SYNC_H  
#define DARTT_SYNC_H
#include <stdint.h>
#include <stddef.h>
#include "dartt.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct dartt_sync_t
{
        unsigned char address;	 // Target peripheral address
		dartt_mem_t ctl_base;			// Bounding region of controller control structure
		dartt_mem_t periph_base;		 // Bounding region of shadow copy structure
		serial_message_type_t msg_type;	// Message framing type
		dartt_buffer_t tx_buf;		// Transmission buffer
		void * user_context_tx;		//OPTIONAL resource used for tx callback - i.e. serial class, socket, etc. Set to NULL if not needed
		dartt_buffer_t rx_buf;		 // Reception buffer
		void * user_context_rx;		//OPTIONAL resource used for rx callback - i.e. serial class,etc. Set to NULL if not needed
		int (*blocking_tx_callback)(unsigned char, dartt_buffer_t*, void * user_context, uint32_t timeout);	//Callback for (blocking) transmissions with a millisecond timeout
		int (*blocking_rx_callback)(dartt_buffer_t*, void * user_context, uint32_t timeout);		//Callback for (blocking) receptions with a millisecond timeout
		uint32_t timeout_ms;		// Communication timeout
}dartt_sync_t;



int dartt_sync(dartt_buffer_t * ctl, dartt_sync_t * psync);
int dartt_ctl_write(dartt_buffer_t * ctl, dartt_sync_t * psync);
int dartt_ctl_read(dartt_buffer_t * ctl, dartt_sync_t * psync);
int dartt_read_multi(dartt_buffer_t * ctl, dartt_sync_t * psync);
int dartt_write_multi(dartt_buffer_t * ctl, dartt_sync_t * psync);
int dartt_update_controller(dartt_buffer_t * ctl, dartt_sync_t * psync);

#ifdef __cplusplus
}
#endif


#endif


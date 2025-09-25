#ifndef DARTT_SYNC_H  
#define DARTT_SYNC_H
#include <stdint.h>
#include <stddef.h>
#include "dartt.h"

#ifdef __cplusplus
extern "C" {
#endif




int dartt_sync(unsigned char misc_address,
		buffer_t * ctl,
		buffer_t * periph,
		buffer_t * base,
		serial_message_type_t msg_type,
		buffer_t * tx_buf,
		buffer_t * rx_buf,
		int (*blocking_tx_callback)(unsigned char, buffer_t*, uint32_t timeout),
		int (*blocking_rx_callback)(unsigned char, buffer_t*, uint32_t timeout),
		uint32_t timeout_ms
		);




#ifdef __cplusplus
}
#endif


#endif


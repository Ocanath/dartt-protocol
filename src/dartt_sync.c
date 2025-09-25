#include "dartt.h"


/**
 * @brief This function scans two buffers (one control and one peripheral) for the presense of any mismatch between control and peripheral.
 * If a difference is found, the master then writes the control copy TO the peripheral, and reads it back to verify a match.
 * Access to hardware is managed with callback function pointers. Callback function pointer is called with the function.
 * 
 * @param misc_address The address of the target peripheral device for message creation. Can be set to zero if not used (type 1)
 * @param ctl Pointer to the buffer containing the 'control' copy. This is the master copy we are synchronizing peripheral devices to
 * @param periph Pointer to the buffer containing the 'peripheral' copy. This is our internal 'latest' reference to the peripheral
 * @param msg_type Dartt message type for message creation
 * @param tx_buf Pointer to buffer used for blocking transmission
 * @param rx_buf Pointer to buffer used for blocking reception
 * @param blocking_tx_callback Function pointer, callback function for blocking transmission
 * @param blocking_rx_callback Function pointer, callback for blocking reception
 * @param timeout For blocking write and read, a millisecond timeout value.
 * */
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
		)	//callbacks?
{
    /*TODO Implement a dartt_sync_t structure to wrap these things, and a callback registration function to load the function pointers. */
	if(ctl->size != periph->size)
	{
		return ERROR_MEMORY_OVERRUN;
	}
	if(ctl->size % sizeof(int32_t) == 0)
	{
		return ERROR_INVALID_ARGUMENT;	//make sure you're 32 bit aligned in all refs
	}

	for(int field_bidx = 0; field_bidx < ctl->size; field_bidx += sizeof(int32_t))
	{
		uint8_t match = 1;
		for(int i = 0; i < sizeof(int32_t); i++)
		{
			int bidx = field_bidx + i;
			if(ctl->buf[bidx] != periph->buf[bidx])
			{
				match = 0;
				break;
			}
		}
		if(match == 0)
		{
			// uint16_t field_index = field_bidx/sizeof(int32_t);
			uint16_t field_index = index_of_field( (void*)(&ctl->buf[field_bidx]), (void*)(&base->buf[0]), base->size );
			//write then read the word in question
			misc_write_message_t write_msg =
			{
					.address = misc_address,
					.index = field_index,
					.payload = {
							.buf = &ctl->buf[field_bidx],
							.size = sizeof(int32_t),
							.len = sizeof(int32_t)
					}
			};
			int rc = dartt_create_write_frame(&write_msg, msg_type, tx_buf);
			if(rc != SERIAL_PROTOCOL_SUCCESS)
			{
				return rc;
			}
			//blocking write callback
			rc = (*blocking_tx_callback)(misc_address, tx_buf, timeout_ms);
			if(rc != SERIAL_PROTOCOL_SUCCESS)
			{
				return rc;
			}

			misc_read_message_t read_msg =
			{
					.address = misc_address,
					.index = field_index,
					.num_bytes = write_msg.payload.len
			};
			rc = dartt_create_read_frame(&read_msg, msg_type, tx_buf);
			if(rc != SERIAL_PROTOCOL_SUCCESS)
			{
				return rc;
			}
			rc = (*blocking_tx_callback)(misc_address, tx_buf, timeout_ms);
			if(rc != SERIAL_PROTOCOL_SUCCESS)
			{
				return rc;
			}

			rc = (*blocking_rx_callback)(misc_address, rx_buf, timeout_ms);
			if(rc != SERIAL_PROTOCOL_SUCCESS)
			{
				return rc;
			}

			payload_layer_msg_t pld_msg = {};
			rc = dartt_frame_to_payload(rx_buf, msg_type, PAYLOAD_ALIAS, &pld_msg);
			if(rc != SERIAL_PROTOCOL_SUCCESS)
			{
				return rc;
			}


		}
	}

	return SERIAL_PROTOCOL_SUCCESS;

}

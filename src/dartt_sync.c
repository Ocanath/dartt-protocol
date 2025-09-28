#include "dartt_sync.h"
#include <assert.h>


/**
 * @brief This function scans two buffers (one control and one peripheral) for the presense of any mismatch between control and peripheral.
 * If a difference is found, the master then writes the control copy TO the peripheral, and reads it back into the shadow copy (local peripheral or *periph) to verify a match.
 * Access to hardware is managed with callback function pointers. Callback function pointers must be loaded into *psync
 * 
 * @param ctl Pointer to the buffer containing the 'control' copy. This is the master copy we are synchronizing peripheral devices to
 * @param periph Pointer to the buffer containing the 'peripheral' copy. This is our internal 'latest' reference to the peripheral
 * @param psync Pointer to a dartt_sync_t structure containing the address, serial callbacks, message type
 * */
int dartt_sync(buffer_t * ctl, buffer_t * periph, dartt_sync_t * psync)	//callbacks?
{
    /*TODO Implement a dartt_sync_t structure to wrap these things, and a callback registration function to load the function pointers. */
    assert(psync != NULL && ctl != NULL && periph != NULL);
    assert(psync->blocking_rx_callback != NULL && psync->blocking_tx_callback != NULL && psync->base.buf != NULL && psync->base.size != 0);
    assert(ctl != periph);

    if(ctl->size != periph->size)
	{
		return ERROR_MEMORY_OVERRUN;
	}
	if(ctl->size % sizeof(int32_t) != 0)
	{
		return ERROR_INVALID_ARGUMENT;	//make sure you're 32 bit aligned in all refs
	}
        // Runtime checks for buffer bounds - these could be caused by developer error in ctl configuration
    if (ctl->buf < psync->base.buf || ctl->buf >= (psync->base.buf + psync->base.size)) 
    {
        return ERROR_INVALID_ARGUMENT;
    }
    if (ctl->buf + ctl->len > psync->base.buf + psync->base.size) 
    {
        return ERROR_MEMORY_OVERRUN;
    }
    if (ctl->buf + ctl->size > psync->base.buf + psync->base.size) 
    {
        return ERROR_MEMORY_OVERRUN;
    }

    int start_bidx = -1;
    int stop_bidx = -1;
    
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

        // Start a new batch if we found a mismatch and don't have one started
        if(match == 0 && start_bidx < 0)
        {
            start_bidx = field_bidx;
        }

        // Check if we need to end the current batch
        if(start_bidx >= 0)
        {
            // End batch if: found a match, reached end of buffer, or would exceed tx buffer size
            int next_field = field_bidx + sizeof(int32_t);
            int batch_size = next_field - start_bidx;

            if(match == 1 || next_field >= ctl->size || batch_size + NUM_BYTES_NON_PAYLOAD > psync->tx_buf.size)
            {
                stop_bidx = (match == 1) ? field_bidx : next_field;
            }
        }

		if(stop_bidx >= 0)
		{
			// uint16_t field_index = field_bidx/sizeof(int32_t);
            int field_index = index_of_field( (void*)(&ctl->buf[start_bidx]), (void*)(&psync->base.buf[0]), psync->base.size );
            if(field_index < 0)
            {
                return field_index; //negative values are error codes, return if you get negative value
            }
            unsigned char misc_address = dartt_get_complementary_address(psync->address);
			//write then read the word in question
			misc_write_message_t write_msg =
			{
					.address = misc_address,
					.index = field_index,
					.payload = {
							.buf = &ctl->buf[start_bidx],
							.size = (stop_bidx - start_bidx),
							.len = (stop_bidx - start_bidx)
					}
			};
			int rc = dartt_create_write_frame(&write_msg, psync->msg_type, &psync->tx_buf);
			if(rc != DARTT_PROTOCOL_SUCCESS)
			{
				return rc;
			}

            //blocking write callback
			rc = (*(psync->blocking_tx_callback))(misc_address, &psync->tx_buf, psync->timeout_ms);
			if(rc != DARTT_PROTOCOL_SUCCESS)
			{
				return rc;
			}

			misc_read_message_t read_msg =
			{
					.address = misc_address,
					.index = field_index,
					.num_bytes = write_msg.payload.len
			};
			rc = dartt_create_read_frame(&read_msg, psync->msg_type, &psync->tx_buf);
			if(rc != DARTT_PROTOCOL_SUCCESS)
			{
				return rc;
			}
			rc = (*(psync->blocking_tx_callback))(misc_address, &psync->tx_buf, psync->timeout_ms);
			if(rc != DARTT_PROTOCOL_SUCCESS)
			{
				return rc;
			}

			rc = (*(psync->blocking_rx_callback))(misc_address, &psync->rx_buf, psync->timeout_ms);
			if(rc != DARTT_PROTOCOL_SUCCESS)
			{
				return rc;
			}

			payload_layer_msg_t pld_msg = {};
			rc = dartt_frame_to_payload(&psync->rx_buf, psync->msg_type, PAYLOAD_ALIAS, &pld_msg);
			if(rc != DARTT_PROTOCOL_SUCCESS)
			{
				return rc;
			}

            for(int i = 0; i < write_msg.payload.len; i++)
            {
                if(write_msg.payload.buf[i] != pld_msg.msg.buf[i])
                {
                    return ERROR_SYNC_MISMATCH;
                }
            }
            for(int i = 0; i < write_msg.payload.len; i++)
            {
                periph->buf[start_bidx + i] = ctl->buf[start_bidx+i];   //copy the mismatched word after confirming the peripheral matches
            }
            start_bidx = -1;
            stop_bidx = -1;
		}
	}

	return DARTT_PROTOCOL_SUCCESS;

}

/**
 * @brief This function implements a full wrapper for dartt write frames.
 * You pass by reference a buffer to a region you want to write, located within the base control structure.
 * ctl must be within psync->base or the function will return an error.
 * 
 * @param ctl Pointer to the memory within the master control structure that you want to write. Essentially just an alias into 
 * the master control structure
 * @param psync Sync structure defining the control memory base, blocking read/write callbacks and memory structures 
 */
int dartt_ctl_write(buffer_t * ctl, dartt_sync_t * psync)
{
    assert(ctl != NULL && psync != NULL);
    assert(ctl->buf != NULL && psync->base.buf != NULL && psync->blocking_tx_callback != NULL && psync->tx_buf.buf != NULL);

    // Runtime checks for buffer bounds - these could be caused by developer error in ctl configuration
    if (ctl->buf < psync->base.buf || ctl->buf >= (psync->base.buf + psync->base.size)) {
        return ERROR_INVALID_ARGUMENT;
    }
    if (ctl->buf + ctl->len > psync->base.buf + psync->base.size) {
        return ERROR_MEMORY_OVERRUN;
    }
    if (ctl->buf + ctl->size > psync->base.buf + psync->base.size) {
        return ERROR_MEMORY_OVERRUN;
    }

    int field_index = index_of_field( (void*)(&ctl->buf[0]), (void*)(&psync->base.buf[0]), psync->base.size );
    if(field_index < 0)
    {
        return field_index; //negative values are error codes, return if you get negative value
    }
    unsigned char misc_address = dartt_get_complementary_address(psync->address);
    //write then read the word in question
    misc_write_message_t write_msg =
    {
            .address = misc_address,
            .index = field_index,
            .payload = {
                    .buf = ctl->buf,
                    .size = ctl->size,
                    .len = ctl->len
            }
    };
    int rc = dartt_create_write_frame(&write_msg, psync->msg_type, &psync->tx_buf);
    if(rc != DARTT_PROTOCOL_SUCCESS)
    {
        return rc;
    }
    //blocking write callback
    rc = (*(psync->blocking_tx_callback))(misc_address, &psync->tx_buf, psync->timeout_ms);
    if(rc != DARTT_PROTOCOL_SUCCESS)
    {
        return rc;
    }
}


/**
 * @brief This function creates a master dartt write/read sequence to load a dartt payload into the dest buffer.
 * Dest should be properly aliased to destination memory (via buf and size) and the length of the desired data should be loaded into dest->len
 * 
 * 
 * @param ctl The region of memory which should be read from the true peripheral device, and dumped into the shadow copy/destination buffer periph
 * @param periph The destination of the read (control shadow copy of the peripheral)
 * @param psync Sync structure defining the control memory base, blocking read/write callbacks and memory structures 
 * @return int 
*/
int dartt_ctl_read(buffer_t * ctl, buffer_t * periph, dartt_sync_t * psync)
{
    assert(periph != NULL && psync != NULL && ctl != NULL);
    assert(ctl->buf != NULL && psync->base.buf != NULL && psync->blocking_tx_callback != NULL && psync->tx_buf.buf != NULL);
    assert(periph->buf != NULL);

    // Runtime checks for buffer bounds - these could be caused by developer error in ctl configuration
    if (ctl->buf < psync->base.buf || ctl->buf >= (psync->base.buf + psync->base.size)) {
        return ERROR_INVALID_ARGUMENT;
    }
    if (ctl->buf + ctl->len > psync->base.buf + psync->base.size) {
        return ERROR_MEMORY_OVERRUN;
    }
    if (ctl->buf + ctl->size > psync->base.buf + psync->base.size) {
        return ERROR_MEMORY_OVERRUN;
    }

    unsigned char misc_address = dartt_get_complementary_address(psync->address);
    
    int field_index = index_of_field( (void*)(&ctl->buf[0]), (void*)(&psync->base.buf[0]), psync->base.size );
    if(field_index < 0)
    {
        return field_index; //negative values are error codes, return if you get negative value
    }
    misc_read_message_t read_msg =
    {
            .address = misc_address,
            .index = field_index,
            .num_bytes = ctl->len
    };
    int rc = dartt_create_read_frame(&read_msg, psync->msg_type, &psync->tx_buf);
    if(rc != DARTT_PROTOCOL_SUCCESS)
    {
        return rc;
    }
    rc = (*(psync->blocking_tx_callback))(misc_address, &psync->tx_buf, psync->timeout_ms);
    if(rc != DARTT_PROTOCOL_SUCCESS)
    {
        return rc;
    }

    rc = (*(psync->blocking_rx_callback))(misc_address, &psync->rx_buf, psync->timeout_ms);
    if(rc != DARTT_PROTOCOL_SUCCESS)
    {
        return rc;
    }

    payload_layer_msg_t pld_msg = {};
    rc = dartt_frame_to_payload(&psync->rx_buf, psync->msg_type, PAYLOAD_ALIAS, &pld_msg);
    if(rc != DARTT_PROTOCOL_SUCCESS)
    {
        return rc;
    }
    if(pld_msg.msg.len > periph->size)
    {
        return ERROR_MEMORY_OVERRUN;
    }
    for(int i = 0; i < pld_msg.msg.len; i++)
    {
        periph->buf[i+field_index*sizeof(uint32_t)] = pld_msg.msg.buf[i];
    }
    return DARTT_PROTOCOL_SUCCESS;
}
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
    assert(psync->tx_buf.buf != NULL && psync->rx_buf.buf != NULL);
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
                /*
                TODO: Rather than breaking, log the current field_bidx as 'start'
                Then, continue incrementing field_bidx by 4 until the following conditions are met:
                    1. you exceed the size of the write buffer (at which point you must write your current contents)
                    2. you encounter a matching full 32bit word
                    3. you exceed the size of the control buffer
                Then, and ONLY then, do you perform the physical, blocking write/read exchange.
                You have to flag the presence of a mismatch with the 'start_bidx' - if this is not, say, -1 (initialized) you have to write something.
                */
			}
		}
        if(match == 0 && start_bidx < 0)    //if you get a match and you haven't started, initialize start to a good value
        {
            start_bidx = field_bidx;
        }
        if(start_bidx >= 0)
        {
            if(match == 1)
            {
                stop_bidx = field_bidx;
            }
            else
            {
                int next_field = field_bidx + sizeof(int32_t);
                if(next_field >= ctl->size)                       //check to see if we're at the last loop iteration. If so, you must transmit because the current word has a mismatch we have to close out
                {
                    stop_bidx = next_field;
                    field_bidx = stop_bidx;
                }
                else if( ((next_field - start_bidx) + NUM_BYTES_NON_PAYLOAD) >= psync->tx_buf.size )    //check to see if we're overrunning the tx buffer. Logic here is a bit tricky
                {
                    //TODO: handle this situation - tx buffer overrun. This should only happen if you change a fuckload of adjacent items and tx buf is undersized
                    if(psync->tx_buf.size <= NUM_BYTES_NON_PAYLOAD)
                    {
                        return ERROR_MEMORY_OVERRUN;    //technically an overflow error guard, but it's not terribly inappropriate to return this
                    }
                    stop_bidx = (((psync->tx_buf.size-NUM_BYTES_NON_PAYLOAD)/sizeof(int32_t))) * sizeof(int32_t) + start_bidx;   //we know we've overrun tx buf, so set stop bidx to the maximum possible size the tx buffer will allow via floor division and reinflation with mutiplication
                    if(stop_bidx <= start_bidx)
                    {
                        return ERROR_MEMORY_OVERRUN;
                    }
                    if(stop_bidx < sizeof(int32_t)) //if stop_bidx is equal to 4, we will set field_bidx to 4 on the next iteration and start there on the next rotation thru
                    {
                        return ERROR_MEMORY_OVERRUN;
                    }
                    field_bidx = stop_bidx - sizeof(int32_t); //we have to step backwards
                }
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
            if(psync->rx_buf.len == 0)  //check for failure to reply. rx blocking should return 0 length if 0 length was obtained
            {
                return ERROR_MALFORMED_MESSAGE;
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
    return (*(psync->blocking_tx_callback))(misc_address, &psync->tx_buf, psync->timeout_ms);
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
    assert(psync->rx_buf.size != 0);
    assert(psync->tx_buf.size != 0);

    // Runtime checks for buffer bounds - these could be caused by developer error in ctl configuration
    if(ctl->len == 0)
    {
        return ERROR_INVALID_ARGUMENT;
    }
    if (ctl->buf < psync->base.buf || ctl->buf >= (psync->base.buf + psync->base.size)) 
    {
        return ERROR_MEMORY_OVERRUN;
    }
    if (ctl->buf + ctl->len > psync->base.buf + psync->base.size) 
    {
        return ERROR_MEMORY_OVERRUN;
    }
    if (ctl->buf + ctl->size > psync->base.buf + psync->base.size) 
    {
        return ERROR_MEMORY_OVERRUN;
    }
    if(ctl->len + NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM > psync->rx_buf.size)
    {
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
    if(psync->rx_buf.len == 0)  //check for failure to reply. rx blocking should return 0 length if 0 length was obtained
    {
        return ERROR_MALFORMED_MESSAGE;
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

/**
 * @brief Wrapper for dartt read that automatically breaks out multiple read operations in order 
 * to read from a larger chunk of memory than there is available read buffer space.
 * 
 * @param ctl 
 * @param perip 
 * @param psync 
 * @return int 
 */
int dartt_read_multi(buffer_t * ctl, buffer_t * perip, dartt_sync_t * psync)
{
    assert(ctl != NULL && perip != NULL && psync != NULL);
    if(!(ctl->buf >= psync->base.buf && ctl->buf < psync->base.buf + psync->base.size))
    {
        return ERROR_MEMORY_OVERRUN;
    }
    size_t rsize = psync->rx_buf.size;
    
    if(psync->msg_type == TYPE_SERIAL_MESSAGE)
    {
        rsize -= NUM_BYTES_ADDRESS;
        rsize -= NUM_BYTES_CHECKSUM;
    }
    else if(psync->msg_type == TYPE_ADDR_MESSAGE)
    {
        rsize -= NUM_BYTES_CHECKSUM;
    }
    else if(psync->msg_type != TYPE_ADDR_CRC_MESSAGE)
    {
        return ERROR_INVALID_ARGUMENT;
    }
    rsize -= rsize % sizeof(uint32_t); //after making sure the dartt framing bytes are removed, you must ensure that the read size is 32 bit aligned for index_of_field

    size_t start_bidx = ctl->buf - psync->base.buf;
    size_t end_bidx = start_bidx + ctl->len;
    int num_full_reads_required = ctl->len/rsize; 
    int i = 0;
    for(i = 0; i < num_full_reads_required; i++)
    {
        buffer_t ctl_chunk = 
        {
            .buf = ctl->buf + rsize * i,
            .size = rsize,
            .len = rsize
        };
        int rc = dartt_ctl_read(&ctl_chunk, perip, psync);
        if(rc != DARTT_PROTOCOL_SUCCESS)
        {
            return rc;
        }
    }
    buffer_t ctl_last_chunk = 
    {
        .buf = ctl->buf + rsize * i,
        .size = ctl->len % rsize,
        .len = ctl->len % rsize
    };
    int rc = dartt_ctl_read(&ctl_last_chunk, perip, psync);
    if(rc != DARTT_PROTOCOL_SUCCESS)
    {
        return rc;
    }
}
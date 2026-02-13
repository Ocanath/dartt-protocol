#include "dartt_sync.h"
#include "dartt_check_buffer.h"
#include <assert.h>


/**
 * @brief This function scans two buffers (one control and one peripheral) for the presence of any mismatch between control and peripheral.
 * If a difference is found, the master then writes the control copy content TO the target device, and reads it back into the shadow copy to verify a match.
 * Access to hardware is managed with callback function pointers. Callback function pointers must be loaded into *psync
 *
 * @param ctl Pointer to the region within ctl_base that should be synchronized. This is the subset of the master copy
 *            we are synchronizing to the peripheral device. Must be equal to or within psync->ctl_base or function returns error.
 *            The corresponding region in psync->periph_base is compared, and if different, the peripheral is updated and
 *            read back to verify the write succeeded.
 * @param psync Pointer to a dartt_sync_t structure containing the address, serial callbacks, message type, ctl_base,
 *              periph_base (shadow copy), and communication buffers.
 * @return DARTT_PROTOCOL_SUCCESS on success, error code on failure
 * */
int dartt_sync(dartt_buffer_t * ctl, dartt_sync_t * psync)
{
    assert(psync != NULL && ctl != NULL);
    assert(psync->blocking_rx_callback != NULL && psync->blocking_tx_callback != NULL && psync->ctl_base.buf != NULL && psync->ctl_base.size != 0);
	assert(psync->periph_base.buf != NULL);
    assert(psync->tx_buf.buf != NULL && psync->rx_buf.buf != NULL);
    assert(psync->ctl_base.buf != psync->periph_base.buf);
    
    if(psync->ctl_base.size != psync->periph_base.size)
	{
		return ERROR_MEMORY_OVERRUN;
	}
	if(ctl->size % sizeof(int32_t) != 0)
	{
		return ERROR_INVALID_ARGUMENT;	//make sure you're 32 bit aligned in all refs
	}
        // Runtime checks for buffer bounds - these could be caused by developer error in ctl configuration
    if (ctl->buf < psync->ctl_base.buf || ctl->buf >= (psync->ctl_base.buf + psync->ctl_base.size)) 
    {
        return ERROR_INVALID_ARGUMENT;
    }
    if (ctl->buf + ctl->len > psync->ctl_base.buf + psync->ctl_base.size) 
    {
        return ERROR_MEMORY_OVERRUN;
    }
    if (ctl->buf + ctl->size > psync->ctl_base.buf + psync->ctl_base.size) 
    {
        return ERROR_MEMORY_OVERRUN;
    }

    size_t nbytes_writemsg_overhead = 0;
    if(psync->msg_type == TYPE_SERIAL_MESSAGE)
    {
    	nbytes_writemsg_overhead = (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM);	//serial message writes have the maximum overhead, 5 bytes
    }
    else if(psync->msg_type == TYPE_ADDR_MESSAGE)
    {
    	nbytes_writemsg_overhead = (NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM);	//if inherently addressed, 4 bytes
    }
    else if(psync->msg_type == TYPE_ADDR_CRC_MESSAGE)
    {
    	nbytes_writemsg_overhead = NUM_BYTES_INDEX;	//if inherently addressed and error checked, only two bytes
    }
    else
    {
    	return ERROR_INVALID_ARGUMENT;
    }


	
	size_t base_bidx = ctl->buf - psync->ctl_base.buf;	//safe due to guards at beginning of function
	if(base_bidx + ctl->size > psync->periph_base.size)
	{
		return ERROR_MEMORY_OVERRUN;
	}
	if(base_bidx % sizeof(int32_t) != 0)
	{
		return ERROR_INVALID_ARGUMENT;
	}

    int start_bidx = -1;
    int stop_bidx = -1;
	for(int field_bidx = 0; field_bidx < ctl->size; field_bidx += sizeof(int32_t))
	{
		uint8_t match = 1;
		for(int i = 0; i < sizeof(int32_t); i++)
		{
			int bidx = field_bidx + i;
			if(ctl->buf[bidx] != psync->periph_base.buf[bidx+base_bidx])
			{
				match = 0;
				break;  
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
					int max_stop = (int)((((psync->tx_buf.size-nbytes_writemsg_overhead)/sizeof(int32_t))) * sizeof(int32_t) + start_bidx);   //we know we've overrun tx buf, so set stop bidx to the maximum possible size the tx buffer will allow via floor division and reinflation with mutiplication	
					if(next_field < max_stop)
					{
						stop_bidx = next_field;
						field_bidx = stop_bidx;
					}
					else
					{
						stop_bidx = max_stop;
						field_bidx = stop_bidx - sizeof(int32_t);
					}
					if(stop_bidx <= start_bidx)
                    {
                        return ERROR_MEMORY_OVERRUN;
                    }
                }
                else if( ((next_field - start_bidx) + nbytes_writemsg_overhead) >= psync->tx_buf.size )    //check to see if we're overrunning the tx buffer. This is how we manage splitting large syncs into many writes
                {
                    if(psync->tx_buf.size <= nbytes_writemsg_overhead)
                    {
                        return ERROR_MEMORY_OVERRUN;    //technically an overflow error guard, but it's not terribly inappropriate to return this
                    }
                    stop_bidx = (int)((((psync->tx_buf.size-nbytes_writemsg_overhead)/sizeof(int32_t))) * sizeof(int32_t) + start_bidx);   //we know we've overrun tx buf, so set stop bidx to the maximum possible size the tx buffer will allow via floor division and reinflation with mutiplication
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
            int field_index = index_of_field( (void*)(&ctl->buf[start_bidx]), (void*)(&psync->ctl_base.buf[0]), psync->ctl_base.size );
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
					.num_bytes = (uint16_t)(write_msg.payload.len)
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

			rc = (*(psync->blocking_rx_callback))(&psync->rx_buf, psync->timeout_ms);
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

            if(write_msg.payload.len + NUM_BYTES_READ_REPLY_OVERHEAD_PLD > pld_msg.msg.size)    //overrun guard for the comparison below. May be protected but I think that is non-obvious
            {
                return ERROR_MEMORY_OVERRUN;
            }

            for(int i = 0; i < write_msg.payload.len; i++)
            {
                if(write_msg.payload.buf[i] != pld_msg.msg.buf[i + NUM_BYTES_READ_REPLY_OVERHEAD_PLD])
                {
                    return ERROR_SYNC_MISMATCH;
                }
            }
			if(write_msg.payload.len + base_bidx + start_bidx > psync->periph_base.size)
			{
				return ERROR_MEMORY_OVERRUN;
			}
            for(int i = 0; i < write_msg.payload.len; i++)
            {
                psync->periph_base.buf[base_bidx + start_bidx + i] = ctl->buf[start_bidx+i];   //copy the mismatched word after confirming the peripheral matches
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
 * ctl must be within psync->ctl_base or the function will return an error. Additionally, if ctl->len
 * exceeds psync->tx_buf.size, this will return an error. Use dartt_write_multi to automatically manage
 * multi-frame transmission for undersized transmit buffers.
 * 
 * @param ctl Pointer to the memory within the master control structure that you want to write. Essentially just an alias into 
 * the master control structure
 * @param psync Sync structure defining the control memory base, blocking read/write callbacks and memory structures 
 */
int dartt_ctl_write(dartt_buffer_t * ctl, dartt_sync_t * psync)
{
    assert(ctl != NULL && psync != NULL);
    assert(ctl->buf != NULL && psync->ctl_base.buf != NULL && psync->blocking_tx_callback != NULL && psync->tx_buf.buf != NULL);

    // Runtime checks for buffer bounds - these could be caused by developer error in ctl configuration
    if (ctl->buf < psync->ctl_base.buf || ctl->buf >= (psync->ctl_base.buf + psync->ctl_base.size)) {
        return ERROR_INVALID_ARGUMENT;
    }
    if (ctl->buf + ctl->len > psync->ctl_base.buf + psync->ctl_base.size) 
	{
        return ERROR_MEMORY_OVERRUN;
    }
    if (ctl->buf + ctl->size > psync->ctl_base.buf + psync->ctl_base.size) 
	{
        return ERROR_MEMORY_OVERRUN;
    }

    int field_index = index_of_field( (void*)(&ctl->buf[0]), (void*)(&psync->ctl_base.buf[0]), psync->ctl_base.size );
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
 * @brief This function creates a master dartt write/read sequence to read data from the peripheral device
 * and store it in the shadow copy (psync->periph_base). It is primarily used as a helper function -
 * the wrapper dartt_read_multi is preferred in almost all situations, unless the full reply will fit in psync->rx_buf.
 *
 * IMPORTANT: The ctl parameter specifies WHAT to read (the memory region), but results are stored in psync->periph_base
 * at the corresponding offset, NOT in the ctl buffer itself.
 *
 * @param ctl The region of memory (within ctl_base) specifying WHAT to read from the peripheral device.
 *            The ctl->len field specifies how many bytes to read. Results are stored in psync->periph_base
 *            at the offset corresponding to ctl's position within ctl_base.
 * @param psync Sync structure defining the control memory base (ctl_base), peripheral shadow copy base (periph_base),
 *              blocking read/write callbacks and memory structures.
 * @return DARTT_PROTOCOL_SUCCESS on success, error code on failure
 */
int dartt_ctl_read(dartt_buffer_t * ctl, dartt_sync_t * psync)
{
    assert(psync != NULL && ctl != NULL);
	assert(psync->ctl_base.size == psync->periph_base.size);
    assert(ctl->buf != NULL && psync->ctl_base.buf != NULL && psync->blocking_tx_callback != NULL && psync->tx_buf.buf != NULL);
	assert(psync->periph_base.buf != NULL);
    assert(psync->rx_buf.size != 0);
    assert(psync->tx_buf.size != 0);

    // Runtime checks for buffer bounds - these could be caused by developer error in ctl configuration
    if(ctl->len == 0)
    {
        return ERROR_INVALID_ARGUMENT;
    }
    if (ctl->buf < psync->ctl_base.buf || ctl->buf >= (psync->ctl_base.buf + psync->ctl_base.size)) 
    {
        return ERROR_MEMORY_OVERRUN;
    }
    if (ctl->buf + ctl->len > psync->ctl_base.buf + psync->ctl_base.size) 
    {
        return ERROR_MEMORY_OVERRUN;
    }
    if (ctl->buf + ctl->size > psync->ctl_base.buf + psync->ctl_base.size) 
    {
        return ERROR_MEMORY_OVERRUN;
    }
    //ensure the read reply we're requesting won't overrun the read buffer
    size_t nb_overhead_read_reply = NUM_BYTES_READ_REPLY_OVERHEAD_PLD;
    if(psync->msg_type == TYPE_SERIAL_MESSAGE)
    {
    	nb_overhead_read_reply += NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM;
    }
    else if(psync->msg_type == TYPE_ADDR_MESSAGE)
    {
    	nb_overhead_read_reply += NUM_BYTES_CHECKSUM;
    }
	if(ctl->len + nb_overhead_read_reply > psync->rx_buf.size)
	{
		return ERROR_MEMORY_OVERRUN;
	}



    unsigned char misc_address = dartt_get_complementary_address(psync->address);
    
    int field_index = index_of_field( (void*)(&ctl->buf[0]), (void*)(&psync->ctl_base.buf[0]), psync->ctl_base.size );
    if(field_index < 0)
    {
        return field_index; //negative values are error codes, return if you get negative value
    }
    misc_read_message_t read_msg =
    {
            .address = misc_address,
            .index = field_index,
            .num_bytes = (uint16_t)ctl->len
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

    rc = (*(psync->blocking_rx_callback))(&psync->rx_buf, psync->timeout_ms);
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
    return dartt_parse_read_reply(&pld_msg, &read_msg, &psync->periph_base);
}

/**
 * @brief Wrapper for dartt_ctl_read that automatically breaks large read operations into multiple
 * smaller read messages to fit within available buffer space.
 *
 * IMPORTANT: Like dartt_ctl_read, the ctl parameter specifies WHAT to read, but results go into psync->periph_base.
 * This function will automatically split the read into multiple operations if needed.
 *
 * @param ctl Pointer to region within ctl_base specifying WHAT to read. The function will automatically
 *            split this into multiple read operations if the requested length exceeds available rx buffer space.
 *            Results are stored in psync->periph_base at the corresponding offset.
 * @param psync Sync structure with ctl_base, periph_base, callbacks, and buffers.
 * @return DARTT_PROTOCOL_SUCCESS on success, error code on failure
 */
int dartt_read_multi(dartt_buffer_t * ctl, dartt_sync_t * psync)
{
    assert(ctl != NULL && psync != NULL);
	assert(psync->ctl_base.buf != NULL && psync->periph_base.buf != NULL);
	assert(psync->ctl_base.buf != psync->periph_base.buf);	//basic sanity check - the master and shadow copy can't point to the same memory

	if(psync->ctl_base.size != psync->periph_base.size)
	{
		return ERROR_MEMORY_OVERRUN;
	}
    if(!(ctl->buf >= psync->ctl_base.buf && ctl->buf < psync->ctl_base.buf + psync->ctl_base.size))
    {
        return ERROR_MEMORY_OVERRUN;
    }
    size_t nbytes_read_overhead = NUM_BYTES_READ_REPLY_OVERHEAD_PLD;  //
    if(psync->msg_type == TYPE_SERIAL_MESSAGE)
    {
		nbytes_read_overhead += (NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM);
    }
    else if(psync->msg_type == TYPE_ADDR_MESSAGE)
    {
        nbytes_read_overhead += NUM_BYTES_CHECKSUM;
    }
    else if(psync->msg_type != TYPE_ADDR_CRC_MESSAGE)
    {
        return ERROR_INVALID_ARGUMENT;
    }
	if(psync->rx_buf.size < nbytes_read_overhead + sizeof(int32_t))
	{
		return ERROR_MEMORY_OVERRUN;
	}
	size_t rsize = psync->rx_buf.size - nbytes_read_overhead;
    rsize -= rsize % sizeof(uint32_t); //after making sure the dartt framing bytes are removed, you must ensure that the read size is 32 bit aligned for index_of_field

    int num_full_reads_required = (int)(ctl->len/rsize); 
    int i = 0;
    for(i = 0; i < num_full_reads_required; i++)
    {
        dartt_buffer_t ctl_chunk = 
        {
            .buf = ctl->buf + rsize * i,
            .size = rsize,
            .len = rsize
        };
        int rc = dartt_ctl_read(&ctl_chunk, psync);
        if(rc != DARTT_PROTOCOL_SUCCESS)
        {
            return rc;
        }
    }
	size_t last_read_size = ctl->len % rsize;
	if(last_read_size != 0)
	{
		dartt_buffer_t ctl_last_chunk = 
		{
			.buf = ctl->buf + rsize * i,
			.size = last_read_size,
			.len = last_read_size
		};
		return dartt_ctl_read(&ctl_last_chunk, psync); //pass final read rc
	}
	else
	{
		return DARTT_PROTOCOL_SUCCESS;
	}
}

/**
 * @brief Wrapper for dartt_ctl_write that automatically breaks large write operations into multiple
 * smaller write messages for undersized write buffers.
 *
 * @param ctl Pointer to region within ctl_base specifying WHAT to write. The function will automatically
 *            split this into multiple write operations if the requested length exceeds available tx buffer space.
 * 
 * @param psync Sync structure with ctl_base, periph_base, callbacks, and buffers.
 * @return DARTT_PROTOCOL_SUCCESS on success, error code on failure
 */
int dartt_write_multi(dartt_buffer_t * ctl, dartt_sync_t * psync)
{
	assert(psync != NULL && ctl != NULL);
	
    size_t nbytes_writemsg_overhead = 0;
    if(psync->msg_type == TYPE_SERIAL_MESSAGE)
    {
    	nbytes_writemsg_overhead = (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM);	//serial message writes have the maximum overhead, 5 bytes
    }
    else if(psync->msg_type == TYPE_ADDR_MESSAGE)
    {
    	nbytes_writemsg_overhead = (NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM);	//if inherently addressed, 4 bytes
    }
    else if(psync->msg_type == TYPE_ADDR_CRC_MESSAGE)
    {
    	nbytes_writemsg_overhead = NUM_BYTES_INDEX;	//if inherently addressed and error checked, only two bytes
    }
    else
    {
    	return ERROR_INVALID_ARGUMENT;
    }
	if(psync->tx_buf.size < nbytes_writemsg_overhead + sizeof(int32_t)) 	//for completeness, due to DARTT indexing every 4 bytes, you must at minimum be able to write out one full 4 byte word for complete write access
	{
		return ERROR_MEMORY_OVERRUN;
	}
	
	size_t wsize = psync->tx_buf.size - nbytes_writemsg_overhead;	
	wsize -= wsize % sizeof(int32_t);	//must make sure every chunkified write is 32bit aligned due to dartt indexing

	int num_undersized_writes = (int)(ctl->len / wsize);
	int i = 0;
	for(i = 0; i < num_undersized_writes; i++)
	{
		dartt_buffer_t ctl_chunk = 
        {
            .buf = ctl->buf + wsize * i,
            .size = wsize,
            .len = wsize
        };
		int rc = dartt_ctl_write(&ctl_chunk, psync);
		if(rc != DARTT_PROTOCOL_SUCCESS)
		{
			return rc;
		}
	}
	
	int last_write_pld_size = ctl->len % wsize;
	if(last_write_pld_size != 0)			//last write
	{
		dartt_buffer_t ctl_last_chunk = 
		{
			.buf = ctl->buf + wsize * i,
			.size = last_write_pld_size,
			.len = last_write_pld_size
		};
		int rc = dartt_ctl_write(&ctl_last_chunk, psync);
		if(rc != DARTT_PROTOCOL_SUCCESS)
		{
			return rc;
		}
	}
	return DARTT_PROTOCOL_SUCCESS;
}


/**
 * @brief Helper function for copying data FROM a specific region of the shadow copy TO the corresponding region in the controller copy.
 *
 * @param ctl Pointer to region within ctl_base specifying the destination region to update, from periph_base (the shadow copy). Typical 
 * call pattern is to make a call to dartt_read_multi, immediately followed by a call to dartt_update_controller with the exact same input
 *  parameters.
 * 
 * @param psync Sync structure with ctl_base, periph_base, callbacks, and buffers.
 * @return DARTT_PROTOCOL_SUCCESS on success, error code on failure
 */
int dartt_update_controller(dartt_buffer_t * ctl, dartt_sync_t * psync)
{
	assert(ctl != NULL && psync != NULL);
	assert(ctl->buf != NULL);
	assert(psync->ctl_base.buf != NULL && psync->periph_base.buf != NULL);
    int cb = check_buffer(ctl);
    if(cb != DARTT_PROTOCOL_SUCCESS)
    {
        return cb;
    }
    if(ctl->buf < psync->ctl_base.buf || ( (ctl->buf + ctl->size) > (psync->ctl_base.buf + psync->ctl_base.size) ))
    {
        return ERROR_MEMORY_OVERRUN;
    }

	int field_offset = 	index_of_field(ctl->buf, psync->ctl_base.buf, psync->ctl_base.size);	//inherit ctl parameter validation from index_of_field
	if(field_offset < 0)
	{
		return field_offset;	//must allow inde_of_field error messages to fall through
	}
	size_t base_bidx = (size_t)(field_offset*sizeof(uint32_t));	//index_of_field ensures 32bit alignment
	if(base_bidx + ctl->size > psync->periph_base.size)	
	{
		return ERROR_MEMORY_OVERRUN;	//memory overrun guard for the copy op we're about to do
	}
	for(int i = 0; i < ctl->size; i++)
	{
		ctl->buf[i] = psync->periph_base.buf[base_bidx + i];
	}
	return DARTT_PROTOCOL_SUCCESS;
}
#include <stdint.h>
#include <assert.h>
#include "serial-comms.h"
#include "checksum.h"

/*
    Helper function to get the index of a field in a comms struct.
    Arguments:
        p_field: the field to get the index of
        comms: the comms struct
    Returns:
        The index of the field in the comms struct, for creating misc messages.
*/
int index_of_field(void * p_field, void * mem, size_t mem_size)
{
    //null pointer checks
    if(p_field == NULL || mem == NULL)
    {
        return ERROR_INVALID_ARGUMENT;
    }
	// Ensure p_field is within the bounds of the comms struct
    unsigned char * pbase = (unsigned char *)(mem);
    unsigned char * p_field_nonvoid = (unsigned char *)p_field;

    if(p_field_nonvoid < pbase || p_field_nonvoid > (void*)(pbase + mem_size))
    {
        return ERROR_INVALID_ARGUMENT;
    }
    
    //Check for underrun
    if(p_field_nonvoid < pbase)
    {
        return ERROR_INVALID_ARGUMENT;
    }

    size_t offset = p_field_nonvoid - pbase;
    
    // Check if the field is actually within the struct (overrun check)
    if(offset >= mem_size)
    {
        return ERROR_INVALID_ARGUMENT;
    }
    
    // Ensure the offset is aligned to 32-bit boundaries
    if(offset % sizeof(int32_t) != 0)
    {
        return ERROR_INVALID_ARGUMENT;
    }
    
    return offset / sizeof(int32_t);
}

/*
    Helper function to calculate the complementary address.
    Motor addresses (0x00-0x7E) map to misc addresses (0x81-0xFF).
    Master addresses also map: 0x7F (motor master) ↔ 0x80 (misc master).
    Arguments:
        address: Input address (motor, misc, or master)
    Returns:
        The complementary address using the formula: 0xFF - address
*/
unsigned char get_complementary_address(unsigned char address)
{
    return 0xFF - address;
}

/*
    Helper function to check the message and output buffer setups.
    Intended use in a release build: static memory allocation for msg and output, and
    a single call to check_write_args for all msg instances to all output instances before use.
 */
int check_write_args(misc_write_message_t * msg, serial_message_type_t type, buffer_t * output)
{
    if(msg == NULL || output == NULL)
    {
        return ERROR_INVALID_ARGUMENT;
    }
	if(!(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE || type == TYPE_ADDR_CRC_MESSAGE))
	{
		return ERROR_INVALID_ARGUMENT;
	}
    if(msg->payload.len == 0 || msg->payload.buf == NULL || output->buf == NULL)
    {
        return ERROR_INVALID_ARGUMENT;  
    }

    //pre-check lengths for overrun
    if(type == TYPE_SERIAL_MESSAGE)
    {
        if( (msg->payload.len + (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM) ) > output->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
    }
    else if(type == TYPE_ADDR_MESSAGE)
    {
        if(msg->payload.len + (NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM) > output->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
    }
    else if (type == TYPE_ADDR_CRC_MESSAGE)
    {
        if( (msg->payload.len + NUM_BYTES_INDEX) > output->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
    }
    else
    {
        return ERROR_INVALID_ARGUMENT;
    }
    return SERIAL_PROTOCOL_SUCCESS;
}

int create_write_frame(misc_write_message_t * msg, serial_message_type_t type, buffer_t * output)
{
    assert(check_write_args(msg,type,output) == SERIAL_PROTOCOL_SUCCESS);  //assert to save on runtime execution
    
    //prepare the serial buffer
    output->len = 0;
    if(type == TYPE_SERIAL_MESSAGE)
    {
        output->buf[output->len++] = msg->address;        
    }
    uint16_t rw_index = (msg->index & (~READ_WRITE_BITMASK));   //MSB = 0 for write, low 15 for index
    output->buf[output->len++] = (unsigned char)(rw_index & 0x00FF);
    output->buf[output->len++] = (unsigned char)((rw_index & 0xFF00) >> 8);
    for(int i = 0; i < msg->payload.len; i++)
    {
        output->buf[output->len++] = msg->payload.buf[i];
    }
    if(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE)
    {
        uint16_t crc = get_crc16(output->buf, output->len);
        output->buf[output->len++] = (unsigned char)(crc & 0x00FF);
        output->buf[output->len++] = (unsigned char)((crc & 0xFF00) >> 8);
    }
    return SERIAL_PROTOCOL_SUCCESS;
}

/*
Check for validity of msg,type,output combination.
Intended use in release is to check pre-allocated memory once on init, then call with impunity with
pre-checked arguments.
*/
int check_read_args(misc_read_message_t * msg, serial_message_type_t type, buffer_t * output)
{
    if(msg == NULL || output == NULL)
    {
        return ERROR_INVALID_ARGUMENT;
    }
	if(!(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE || type == TYPE_ADDR_CRC_MESSAGE))
	{
		return ERROR_INVALID_ARGUMENT;
	}
    if(output->buf == NULL)
    {
        return ERROR_INVALID_ARGUMENT;  
    }

    //pre-check lengths for overrun
    if(type == TYPE_SERIAL_MESSAGE)
    {
        if( ( (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + NUM_BYTES_NUMWORDS_READREQUEST + NUM_BYTES_CHECKSUM) ) > output->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
    }
    else if(type == TYPE_ADDR_MESSAGE)
    {
        if( (NUM_BYTES_INDEX + NUM_BYTES_NUMWORDS_READREQUEST + NUM_BYTES_CHECKSUM) > output->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
    }
    else if (type == TYPE_ADDR_CRC_MESSAGE)
    {
        if( (NUM_BYTES_INDEX + NUM_BYTES_NUMWORDS_READREQUEST) > output->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
    }
    else
    {
        return ERROR_INVALID_ARGUMENT;
    }
    return SERIAL_PROTOCOL_SUCCESS;
}

/*
 */
int create_read_frame(misc_read_message_t * msg, serial_message_type_t type, buffer_t * output)
{
    assert(check_read_args(msg,type,output) == SERIAL_PROTOCOL_SUCCESS);
    assert(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE || type == TYPE_ADDR_CRC_MESSAGE);

    output->len = 0;
    if(type == TYPE_SERIAL_MESSAGE)
    {
        output->buf[output->len++] = msg->address;
    }
    uint16_t rw_index = msg->index | READ_WRITE_BITMASK;
    output->buf[output->len++] = (unsigned char)(rw_index & 0x00FF);
    output->buf[output->len++] = (unsigned char)((rw_index & 0xFF00) >> 8);
    output->buf[output->len++] = (unsigned char)(msg->num_bytes & 0x00FF);
    output->buf[output->len++] = (unsigned char)((msg->num_bytes & 0xFF00) >> 8);
    if(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE)
    {
        uint16_t crc = get_crc16(output->buf, output->len);
        output->buf[output->len++] = (unsigned char)(crc & 0x00FF);
        output->buf[output->len++] = (unsigned char)((crc & 0xFF00) >> 8);
    }    
    return SERIAL_PROTOCOL_SUCCESS;
}

/* 
    Slave only - parse an incoming master message and act on it
    The input buffer contents must not include the address or crc, if relevant - it is therefore agnostic to 
    message type. It will bifurcate based on the read-write bit. Address and CRC filtering are assumed to have 
    been done before calling this function.
    
    input: input_buffer_base - the serial message, stripped of any CRC and address information
    mem_base: buffer/pointer to the memory region we are reading and writing to
    reply_raw: in the case of a read message, this will contain an un-framed (no address or CRC) message, ready for address+crc framing, link layer framing, and transmission
        
        NOTE TO FUTURE JESSE ^ when implementing the caller to this function, make sure the reply_raw buffer_t makes space for the address with pointer arithmetic.
        i.e.:
            buffer_t shifted_reply = {
                .buf = reply->buf + 1
                .size = reply->size - 1
                .len = 0
            };
            parse_base_serial_message(&shifted_input, mem, &shifted_reply);
            -can use similar logic to handle the input adjustments, with local buffer_t's. that allows you 
            to leave the original buffer_t's untouched, for a small RAM cost, while eliminating O(n) shifting
*/
int parse_base_serial_message(payload_layer_msg_t* pld_msg, buffer_t * mem_base, buffer_t * reply_base)
{
    assert(pld_msg != NULL && mem_base != NULL && reply_base != NULL);
    assert(pld_msg->msg.buf != NULL && mem_base->buf != NULL && reply_base->buf != NULL);
    assert(pld_msg->msg.size > NUM_BYTES_INDEX && mem_base->size > 0 && reply_base->size > 0);
    assert(pld_msg->msg.len <= pld_msg->msg.size && mem_base->len <= mem_base->size && reply_base->len <= reply_base->size);
    
    //critical check - keep as runtime since this is data-dependent
    if(pld_msg->msg.len <= NUM_BYTES_INDEX)   //if write, it must contain at least one byte of payload. If read, it must contain exactly two additional bytes of read size
    {
        return ERROR_MALFORMED_MESSAGE;
    }

    size_t bidx = 0;
    uint16_t rw_index = 0;
    rw_index |= (uint16_t)(pld_msg->msg.buf[bidx++]);
    rw_index |= (((uint16_t)(pld_msg->msg.buf[bidx++])) << 8);
    uint16_t rw_bit = rw_index & READ_WRITE_BITMASK;  //omit the shift and perform zero comparison for speed
    size_t word_offset = ((size_t)(rw_index & (~READ_WRITE_BITMASK)))*sizeof(uint32_t); 
    if(rw_bit != 0) //read
    {
        if(pld_msg->msg.len != NUM_BYTES_INDEX + NUM_BYTES_NUMWORDS_READREQUEST)  //read messages must have precisely this content (once addr and crc are removed, if relevant)
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        uint16_t num_bytes = 0;
        num_bytes |= (uint16_t)(pld_msg->msg.buf[bidx++]);
        num_bytes |= (((uint16_t)(pld_msg->msg.buf[bidx++])) << 8);
        if(num_bytes > reply_base->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }

        
        unsigned char * cpy_ptr = mem_base->buf + word_offset;
        if(word_offset + num_bytes > mem_base->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }

        uint16_t i;
        for(i = 0; i < num_bytes; i++)
        {
            reply_base->buf[i] = cpy_ptr[i];
        }
        reply_base->len = i;
        return SERIAL_PROTOCOL_SUCCESS; //caller needs to finish the reply formatting
    }
    else    //write
    {
        //from our min length and min size checks, we know the input buffer must have both minimum size and
        //minimum length of 3, and we know that if we are here, bidx is equal to 2.
        //Therefore, it is safe to subtract bidx from len
        unsigned char * write_ptr = pld_msg->msg.buf + bidx;
        size_t nbytes_to_write = pld_msg->msg.len - bidx; //this can be 1 at minimum, and cannot underflow due to our checks above. overrun protection is guaranteed here too due to size and len checks
        
        if(word_offset + nbytes_to_write > mem_base->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
        unsigned char * mem_ptr = mem_base->buf + word_offset;
        for(int i = 0; i < nbytes_to_write; i++)
        {
            mem_ptr[i] = write_ptr[i];  //perform the copy
        }
        reply_base->len = 0;    //erase the reply. Success and nonzero reply len should trigger transmission of a reply frame, and we don't reply to write messages!
        return SERIAL_PROTOCOL_SUCCESS; //no reply, so caller doesn't need to do anything else
    }
}

/*
    Helper function to validate the crc in a buffer_t, which is always the last two bytes of the message
    if present.
 */
int validate_crc(buffer_t * input)
{
    assert(input != NULL);
    assert(input->buf != NULL);
    assert(input->size != 0);
    assert(input->len <= input->size);
    if(input->len <= NUM_BYTES_CHECKSUM)
    {
        return ERROR_INVALID_ARGUMENT;
    }
    uint16_t crc = get_crc16(input->buf, input->len - NUM_BYTES_CHECKSUM);
    uint16_t m_crc = 0;
    unsigned char * pchecksum = input->buf + (input->len - NUM_BYTES_CHECKSUM);
    m_crc |= (uint16_t)pchecksum[0];
    m_crc |= ((uint16_t)pchecksum[1]) << 8;
    if(m_crc == crc)
    {
        return SERIAL_PROTOCOL_SUCCESS;
    }
    else
    {
        return ERROR_CHECKSUM_MISMATCH;
    }
}

/*
    Helper function to append a crc to an existing buffer_t
 */
int append_crc(buffer_t * input)
{
    assert(input != NULL);
    assert(input->buf != NULL);
    assert(input->size != 0);
    assert(input->len <= input->size);

    if(input->len + NUM_BYTES_CHECKSUM > input->size)
    {
        return ERROR_MEMORY_OVERRUN;
    }
    uint16_t crc = get_crc16(input->buf, input->len);
    input->buf[input->len++] = (unsigned char)(crc & 0x00FF);
    input->buf[input->len++] = (unsigned char)((crc & 0xFF00) >> 8);

    return SERIAL_PROTOCOL_SUCCESS;
}

/*
    Master function to parse slave reply.
    input: the input buffer/message. This can be of any serial_message_type_t
    type: the message type
    dest: location to copy the reply contents to (similar to mem in slave parse)
*/
int parse_read_reply(buffer_t * input, serial_message_type_t type, buffer_t * dest)
{     
    assert(dest != NULL && input != NULL);
    assert(dest->buf != NULL && input->buf != NULL);
    assert(dest->size > 0 && input->size > 0);
    assert(dest->len <= dest->size && input->len <= input->size);
    assert(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE || type == TYPE_ADDR_CRC_MESSAGE);

    buffer_t input_cpy = {  
        .buf = input->buf,
        .size = input->size,
        .len = input->len
    };
    
    if(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE)    //crc filtering if relevant
    {
        int rc = validate_crc(&input_cpy);
        if(rc != SERIAL_PROTOCOL_SUCCESS)
        {
            return rc;
        }
        input_cpy.len -= NUM_BYTES_CHECKSUM;
    }
    if(type == TYPE_SERIAL_MESSAGE)     //address filtering if relevant
    {
        if(input_cpy.len <= NUM_BYTES_ADDRESS)  //input.len now must be greater or equal to 2 for a valid message. CRC may have been removed
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        unsigned char addr = input_cpy.buf[0];
        if(addr != MASTER_MISC_ADDRESS)
        {
            return ADDRESS_FILTERED;
        }
        input_cpy.buf++;
        input_cpy.len--;
    }
    if(input_cpy.len > dest->size)
    {
        return ERROR_MEMORY_OVERRUN;
    }
    dest->len = 0;
    for(int i = 0; i < input_cpy.len; i++)
    {
        dest->buf[dest->len++] = input_cpy.buf[i];
    }
    return SERIAL_PROTOCOL_SUCCESS;

}

/*
	This function takes as input a serial message of any type, and strips away the address
		1. validates the checksum (if applicable)
		2. loads the address into the base protocol message (if applicable). Otherwise, it will pass through

	This is a Frame Layer to Payload Layer translation function. The input is a Frame/Transport layer message
	of any serial_message_type.

	IMPORTANT: This function does not have any expectations with regards to frame structure. It simply removes address if present, and crc if present.
	That is it! Frame structure is decoded downstream of this function, which only performs CRC filtering and address removal.
*/
int frame_to_payload(buffer_t * ser_msg, serial_message_type_t type, payload_mode_t pld_mode, payload_layer_msg_t * pld)
{
    assert(ser_msg != NULL && pld != NULL);
    assert(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE || type == TYPE_ADDR_CRC_MESSAGE);
	assert(ser_msg->buf != NULL);
	assert(ser_msg->size != 0);
	assert(ser_msg->len != 0);
	assert((pld->msg.buf == NULL && pld->msg.size == 0) || (pld->msg.buf != NULL && pld->msg.size != 0));

	if(type == TYPE_SERIAL_MESSAGE)
    {
		//first step - crc validation on input message
        if(ser_msg->len <= (NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM))    //message has to have room for a checksum, an index, and at least one additional byte
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        int rc = validate_crc(ser_msg);
        if(rc != SERIAL_PROTOCOL_SUCCESS)
        {
            return rc;	//checksum must match
        }

		if(pld_mode == PAYLOAD_ALIAS)	//Use pointer arithmetic
		{
			pld->msg.buf = ser_msg->buf;
			pld->msg.size = ser_msg->size;
			pld->msg.len = ser_msg->len - NUM_BYTES_CHECKSUM;	
			
			pld->address = ser_msg->buf[0];
			pld->msg.buf += NUM_BYTES_ADDRESS;
			pld->msg.len -= NUM_BYTES_ADDRESS;
			//truncate checksum off and use pointer arithmetic to load payload to addr
		}
		else if(pld_mode == PAYLOAD_COPY)	//
		{
			if(pld->msg.buf == NULL)
			{
				return ERROR_INVALID_ARGUMENT;
			}
			size_t newlen  = ser_msg->len - (NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM);
			if(newlen > pld->msg.size)
			{
				return ERROR_MEMORY_OVERRUN;
			}
			pld->address = ser_msg->buf[0];
			unsigned char * sm_start = ser_msg->buf + NUM_BYTES_ADDRESS; //skip address
			
			for(int i = 0; i < newlen; i++)
			{
				pld->msg.buf[i] = sm_start[i];
			}
			pld->msg.len = newlen;
		}
		else
		{
			return ERROR_INVALID_ARGUMENT;
		}
        return SERIAL_PROTOCOL_SUCCESS;
    }
	else if (type == TYPE_ADDR_MESSAGE)
    {
        if(ser_msg->len <= NUM_BYTES_CHECKSUM)    //message has to have room for a checksum, an index, and at least one additional byte
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        int rc = validate_crc(ser_msg);
        if(rc != SERIAL_PROTOCOL_SUCCESS)
        {
            return rc;
        }
		if(pld_mode == PAYLOAD_ALIAS)
		{
			pld->msg.buf = ser_msg->buf;
			pld->msg.size = ser_msg->size;
			pld->msg.len = (ser_msg->len - NUM_BYTES_CHECKSUM);
		}
		else if (pld_mode == PAYLOAD_COPY)
		{
			if(pld->msg.buf == NULL)
			{
				return ERROR_INVALID_ARGUMENT;
			}
			size_t newlen  = ser_msg->len - NUM_BYTES_CHECKSUM;
			if(newlen > pld->msg.size)
			{
				return ERROR_MEMORY_OVERRUN;
			}
			for(int i = 0; i < newlen; i++)
			{
				pld->msg.buf[i] = ser_msg->buf[i];
			}
			pld->msg.len = newlen;
		}
		else
		{
			return ERROR_INVALID_ARGUMENT;
		}
        return SERIAL_PROTOCOL_SUCCESS;
    }
	else if(type == TYPE_ADDR_CRC_MESSAGE)
	{
		if(pld_mode == PAYLOAD_ALIAS)	//use pointer arithmetic to have the pld->msg refer to the payload section of the frame layer message
		{
			pld->msg.buf = ser_msg->buf;
			pld->msg.size = ser_msg->size;
			pld->msg.len = ser_msg->len;
		}
		else if(pld_mode == PAYLOAD_COPY)	//make a copy
		{
			if(pld->msg.buf == NULL)
			{
				return ERROR_INVALID_ARGUMENT;
			}
			if(ser_msg->len > pld->msg.size)
			{
				return ERROR_MEMORY_OVERRUN;
			}
			for(int i = 0; i < ser_msg->len; i++)
			{
				pld->msg.buf[i] = ser_msg->buf[i];
			}
			pld->msg.len = ser_msg->len;
		}
		else
		{
			return ERROR_INVALID_ARGUMENT;
		}
		return SERIAL_PROTOCOL_SUCCESS;
	}
}

/*
    Implement the general message memory write-reply behavior on a payload-layer message.
	Loads a reply frame with the associated frame layer format (type) based on the input frame layer type.
	Usage: call within a parent function that calls frame_to_payload, bifurcates based on address range, and calls this if it's in the misc range.
*/
int parse_general_message(payload_layer_msg_t * pld_msg, serial_message_type_t type, buffer_t * mem_base, buffer_t * reply)
{
    assert(pld_msg != NULL && mem_base != NULL && reply != NULL);
    assert(pld_msg->msg.buf != NULL && mem_base->buf != NULL && reply->buf != NULL);
    assert(pld_msg->msg.size != 0 && mem_base->size != 0 && reply->size != 0);
    assert(pld_msg->msg.len <= pld_msg->msg.size && mem_base->len < mem_base->size && reply->len < reply->size);   
    assert(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE || type == TYPE_ADDR_CRC_MESSAGE);
	
    if(type == TYPE_SERIAL_MESSAGE)
    {
        buffer_t reply_cpy = {
            .buf = reply->buf + NUM_BYTES_ADDRESS,     //make room for the address, which we will load after if necessary
            .size = reply->size - 1,
            .len = 0
        };
        int rc = parse_base_serial_message(pld_msg, mem_base, &reply_cpy);    //will copy from 1 to len. the original reply buffer is now ready for address and crc loading
        if(rc == SERIAL_PROTOCOL_SUCCESS && reply_cpy.len != 0)
        {
            //append address
            reply->buf[0] = MASTER_MISC_ADDRESS;
            reply->len = reply_cpy.len + NUM_BYTES_ADDRESS; //update len now that we have the address in the base message
            return append_crc(reply);   //the checks in this function also check for address length increase/overrun
        }
        else if(rc != SERIAL_PROTOCOL_SUCCESS)
        {
            return rc;
        }
        return SERIAL_PROTOCOL_SUCCESS;
    }
    else if (type == TYPE_ADDR_MESSAGE)
    {
        reply->len = 0;
        int rc = parse_base_serial_message(pld_msg, mem_base, reply);
        if(rc == SERIAL_PROTOCOL_SUCCESS && reply->len != 0)
        {
            return append_crc(reply);
        }
        else if (rc != SERIAL_PROTOCOL_SUCCESS)
        {
            return rc;
        }
        return SERIAL_PROTOCOL_SUCCESS;

    }
    else if (type == TYPE_ADDR_CRC_MESSAGE)
    {
        return parse_base_serial_message(pld_msg, mem_base, reply);   //type 3 carries the base protocol with no additional payload dressings
    }
    else
    {
        return ERROR_INVALID_ARGUMENT;  //should never end up here - assert should catch this. Can only happen in release builds untested in debug
    }
}

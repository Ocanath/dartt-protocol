#include <stdint.h>
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
    if(p_field < mem || p_field > mem + mem_size)
    {
        return ERROR_INVALID_ARGUMENT;
    }
    // Ensure p_field is within the bounds of the comms struct
    unsigned char * pbase = (unsigned char *)(mem);
    unsigned char * p_field_nonvoid = (unsigned char *)p_field;
    
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
    Create a message packet for a read operation.
    Arguments:
        address: the address of the device to read from
        index: the word index (32 bit words) of the structure as your starting point for the read
        num_words: the number of words to read

*/
int create_misc_read_message(unsigned char address, uint16_t index, uint16_t num_words, buffer_t * output)
{
    //basic error checking (bounds overrun, null pointer checks)
    if(output == NULL)
    {
        return 0;
    }
    if(output->size < NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + sizeof(uint16_t) + NUM_BYTES_CHECKSUM)
    {
        return 0;   //ensure there is enough space to load message
    }
    // add more bounds checking

    int cur_byte_index = 0;
    output->buf[cur_byte_index++] = address;    //byte 0 loaded
    index = index | READ_WRITE_BITMASK; //set the read bit always
    unsigned char * p_index_word = (unsigned char *)(&index);
    output->buf[cur_byte_index++] = p_index_word[0];    //byte 1 loaded
    output->buf[cur_byte_index++] = p_index_word[1];    //byte 2 loaded

    unsigned char * p_num_words = (unsigned char *)(&num_words);
    output->buf[cur_byte_index++] = p_num_words[0];    //byte 3 loaded
    output->buf[cur_byte_index++] = p_num_words[1];    //byte 4 loaded

    uint16_t checksum = get_crc16(output->buf, cur_byte_index);
    unsigned char * p_checksum = (unsigned char *)(&checksum);
    output->buf[cur_byte_index++] = p_checksum[0];    //byte 5 loaded
    output->buf[cur_byte_index++] = p_checksum[1];    //byte 6 loaded

    output->len = cur_byte_index;
    return cur_byte_index;
}

/*
	Helper function to set the read/write bit argument of the index parameter in the message_t. 
	If paired with set_index, can be used in any order.
	It will overwrite the MSB of the index argument.
*/
int set_rw(misc_message_t * msg, read_write_type_t read_write)
{
	if(read_write == READ_MESSAGE)
	{
		msg->rw_index |= READ_WRITE_BITMASK;
	}
	else
	{
		msg->rw_index &= ~READ_WRITE_BITMASK;
	}
	return SERIAL_PROTOCOL_SUCCESS;
}

/*
	Helper function to load the index portion of the rw_index argument of a misc_message.
	
*/
int set_index(misc_message_t * msg, uint16_t index)
{
	if(msg == NULL)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	msg->rw_index &= READ_WRITE_BITMASK;	//zero out all bits below the R/W bit
	msg->rw_index |= (index & (~READ_WRITE_BITMASK));	//or-in all nonzero bits of the lowest 15 bits of the argument. This effectively copies in the index without overwriting the rw bit, allowing order independent calling of these functions
	return SERIAL_PROTOCOL_SUCCESS;
}

/*
	Format an output buffer_t based on a serial message_t structure.
	You must load the msg address, rw_index, and payload fields properly
*/
int misc_message_to_serial_buf(misc_message_t * msg, serial_message_type_t type, buffer_t * output)
{
	if(msg == NULL || output == NULL)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	if(type == TYPE_UART_MESSAGE)
	{
		if( (msg->payload.len + NUM_BYTES_NON_PAYLOAD) > output->size)	//we'll write an address, two index, and a crc. Precheck for overrun
		{
			return ERROR_MEMORY_OVERRUN;
		}

		int bidx = 0;
		output->buf[bidx++] = msg->address;
		//implement little-endian loading op for maximum portability. we always use little endian due to mainly using this on little endian systems.
		output->buf[bidx++] = (unsigned char)(msg->rw_index & 0x00FF);	
		output->buf[bidx++] = (unsigned char)((msg->rw_index & 0xFF00) >> 8);	
		if(msg->rw_index & READ_WRITE_BITMASK == 0)	//iff this is a write message, we need to copy the msg->payload. Otherwise we can fall through and simply load the crc, then exit
		{
			for(int i = 0; i < msg->payload.len; i++)
			{
				output->buf[bidx++] = msg->payload.buf[i];
			}
		}
		output->buf[bidx++] = get_crc16(output->buf, bidx);
		output->len = bidx;
		return SERIAL_PROTOCOL_SUCCESS;
	}
	else	//fall through to simple payload copying if you aren't a UART message. This can be handled outside this function trivially in a true CAN implementation for speed
	{
		if(msg->payload.len > output->size)
		{
			return ERROR_MEMORY_OVERRUN;
		}
		for(int i = 0; i < msg->payload.len; i++)
		{
			output->buf[i] = msg->payload.buf[i];
		}
		output->len = msg->payload.len;
		return SERIAL_PROTOCOL_SUCCESS;
	}
}

/*load a misc_message_t based on a serial input buffer.
	ARGUMENTS:
		input: the serial message itself
		type: flag to indicate if we need to parse out the crc and address. If it is a CAN type message the address is ignored (whatever is in msg falls through)
		mem_base: 
			1. 	If valid, this is treated as the 'base pointer' for the memory we're writing, asuming we are getting a write request as a reciever.
				We will then set the msg->payload.buf pointer as an offset to mem_base and copy the payload section from the input to the target. 
				IMPORTANT: If this is valid, msg->payload MUST be NULL
			2. 	If invalid (NULL or ->size = 0), this is ignored. The function will then copy the payload directly to the buffer msg->payload (zero aligned). This 
				allows us to double-buffer read messages if desired. 
				IMPORTANT: if the intent is to use a pre-setup msg->payload, you MUST set mem_base to NULL
		msg: the misc_message from the buffer_t
			msg->address will contain the message address, IFF it is not a CAN type message (otherwise, the address is not touched and falls through - it is assumed to be properly loaded beforehand)
			msg->index will contain the message index argument, including the MSB read/write bit
			msg->payload should be invalid (NULL or ->size = 0) when this function is called if mem_base is valid - it will then be assigned to an offset from mem_base, according to the input message content
			msg->payload should be valid (points to valid memory region) when this function is called, and the intended use is to simply make a copy of the input buffer payload.
	Note-pointer logic does not apply to 'read' type messages. If the message type is 'read', the payload pointers are simply ignored and they fall through
 */
int serial_buf_to_misc_message(buffer_t * input, serial_message_type_t type, buffer_t * mem_base, misc_message_t * msg)
{
	
}

/*
    Write message packet creation function.
    Arguments:
        address: the address of the device to write to
        index: the word index (32 bit words) of the structure as your starting point for the write
        payload: the bytes to write to the device
        payload_size: the number of bytes of payload we are writing
        msg_buf: the buffer containing the message
        msg_len: the length of the message (variable, pass by pointer)
        msg_buf_size: the size of the message buffer
    Returns:
        The number of bytes written to the message buffer
*/
int create_misc_write_message(unsigned char address, uint16_t index, buffer_t * payload, buffer_t * output)
{
    //basic error checking (bounds overrun, null pointer checks)
    if(payload == NULL || output == NULL)
    {
        return 0;
    }
    if(output->size < (payload->len + NUM_BYTES_NON_PAYLOAD)) //fixed size
    {
        return 0;
    }
    
    //load the address
    int cur_byte_index = 0;
    output->buf[cur_byte_index++] = address;
    
    //load the index, clear the read bit
    index = index & 0x7FFF;
    unsigned char * p_index_word = (unsigned char *)(&index);
    output->buf[cur_byte_index++] = p_index_word[0];
    output->buf[cur_byte_index++] = p_index_word[1];
    
    //load the payload
    for(int i = 0; i < payload->len; i++)
    {
        output->buf[cur_byte_index++] = payload->buf[i];
    }

    //load the checksum
    uint16_t checksum = get_crc16(output->buf, cur_byte_index);
    unsigned char * p_checksum = (unsigned char *)(&checksum);
    output->buf[cur_byte_index++] = p_checksum[0];
    output->buf[cur_byte_index++] = p_checksum[1];

    output->len = cur_byte_index;
    return cur_byte_index;
}


/*
    Arguments:
        input: the message buffer, excluding address and checksum. If byte stuffing is used, this must be the unstuffed message.
        len: the length of the message
        comms: the global comms struct
    Returns:
        -2 if the message is malformed
        -1 if the message is not intended for this device
        0 if the message is successfully parsed
 */
int parse_misc_command(buffer_t * input, serial_message_type_t type, buffer_t * reply, void * mem, size_t mem_size)
{
    if(input == NULL) //  || input->len < (NUM_BYTES_ADDRESS+NUM_BYTES_INDEX+NUM_BYTES_CHECKSUM) //message length check should have been done before we enter here
    {
        return ERROR_MALFORMED_MESSAGE;
    }

    //TODO: finish the implementation of this, where you drop address and checksum if they're handled in hardware or by the parent protocol
    uint32_t num_bytes_address = NUM_BYTES_ADDRESS;
    uint32_t num_bytes_checksum = NUM_BYTES_CHECKSUM;
    if(type != TYPE_UART_MESSAGE)
    {
        num_bytes_address = 0;
        num_bytes_checksum = 0;
    }

    uint16_t index_argument;
    index_argument = (input->buf[NUM_BYTES_ADDRESS + 1] << 8) | input->buf[NUM_BYTES_ADDRESS];
    uint16_t read_mask = index_argument & READ_WRITE_BITMASK;
    uint16_t index = index_argument & 0x7FFF;
    uint32_t byte_index = (uint32_t)(index*sizeof(uint32_t));
    
    if(read_mask == 0)    //
    {
        //write    
        int write_len = input->len - (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM);   //We start after the index section, and go until we hit the checksum
        if(byte_index + write_len > mem_size)
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        else
        {
            unsigned char * pcomms = (unsigned char *)(mem);
            pcomms = &pcomms[byte_index];
            unsigned char * pinput = &input->buf[NUM_BYTES_ADDRESS+NUM_BYTES_INDEX];  //skip past the index argument portion for the write payload
            for(int i = 0; i < write_len; i++)
            {
                pcomms[i] = pinput[i];
            }
            reply->len = 0;
            return SERIAL_PROTOCOL_SUCCESS;
        }
    }
    else
    {
        //read
        if( (input->len != (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + NUM_BYTES_NUMWORDS_READREQUEST + NUM_BYTES_CHECKSUM) ) || reply == NULL)
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        else
        {
            //read
            uint16_t * p_numread_words = (uint16_t*)(&input->buf[NUM_BYTES_ADDRESS+NUM_BYTES_INDEX]);
            uint32_t numread_bytes = (uint32_t)(*p_numread_words * sizeof(uint32_t));
            if(numread_bytes + byte_index > mem_size || (numread_bytes + NUM_BYTES_CHECKSUM + NUM_BYTES_ADDRESS) > reply->size)	//pre-check size once
            {
                return ERROR_MALFORMED_MESSAGE;
            }
            else
            {
                unsigned char * p_comms = (unsigned char *)mem;
                int bidx = 0;

                //first byte is the master address (all replies go to master)
                reply->buf[bidx++] = MASTER_MISC_ADDRESS;

                //next n bytes get loaded into the payload
                for(int i = 0; i < numread_bytes; i++)
                {
                    reply->buf[bidx++] = p_comms[byte_index + i];
                }   

                //final 2 bytes get checksum
                uint16_t checksum = get_crc16(reply->buf, bidx);
                unsigned char * p_checksum = (unsigned char *)(&checksum);
                reply->buf[bidx++] = p_checksum[0];
                reply->buf[bidx++] = p_checksum[1];

                //load reply len for serial transmission
                reply->len = bidx;

                return SERIAL_PROTOCOL_SUCCESS;
            }
        }
    }
}

/*
    Use the message protocol without splitting behavior based on address.
*/
int parse_general_message(unsigned char address, buffer_t * input, serial_message_type_t type, buffer_t * reply, void * mem, size_t mem_size)
{
    if(input->len < MINIMUM_MESSAGE_LENGTH) //minimum message length is 5 bytes (device address + register address + data + checksum)
    {
        return ERROR_MALFORMED_MESSAGE;
    }
    uint16_t * pchecksum = (uint16_t *)(input->buf + input->len - sizeof(uint16_t));
    uint16_t checksum = get_crc16(input->buf, input->len - sizeof(uint16_t));
    if(checksum != *pchecksum)
    {
        return ERROR_CHECKSUM_MISMATCH;
    }
    else
    {
        if(input->buf[0] == address)
        {
            return parse_misc_command(input, type, reply, mem, mem_size);
        }
        else
        {
            return ADDRESS_FILTERED;
        }
    }
}

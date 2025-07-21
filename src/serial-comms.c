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

int misc_write_message_to_serial_buf(misc_write_message_t * msg, serial_message_type_t type, buffer_t * output)
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
int misc_read_message_to_serial_buf(misc_read_message_t * msg, serial_message_type_t type, buffer_t * output)
{
    assert(check_read_args(msg,type,output) == SERIAL_PROTOCOL_SUCCESS);
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
int parse_base_serial_message(buffer_t * input_buffer_base, buffer_t * mem_base, buffer_t * reply_raw)
{
    if(input_buffer_base == NULL || mem_base == NULL || reply_raw == NULL)
    {
        return ERROR_INVALID_ARGUMENT;
    }
    //critical check 
    if(input_buffer_base->len <= NUM_BYTES_INDEX || input_buffer_base->size <= NUM_BYTES_INDEX)   //if write, it must contain at least one byte of payload. If read, it must contain exactly two additional bytes of read size
    {
        return ERROR_MALFORMED_MESSAGE;
    }

    // //optional checks - basic buffer_t construction
    // if(input_buffer_base->len > input_buffer_base->size || mem_base->len > mem_base->size || reply_raw->len > reply_raw->size)  //is this really necessary
    // {
    //     return ERROR_INVALID_ARGUMENT;
    // }

    size_t bidx = 0;
    uint16_t rw_index = 0;
    rw_index |= (uint16_t)(input_buffer_base->buf[bidx++]);
    rw_index |= (((uint16_t)(input_buffer_base->buf[bidx++])) << 8);
    uint16_t rw_bit = rw_index & READ_WRITE_BITMASK;  //omit the shift and perform zero comparison for speed
    size_t word_offset = ((size_t)(rw_index & (~READ_WRITE_BITMASK)))*sizeof(uint32_t); 
    if(rw_bit != 0) //read
    {
        if(input_buffer_base->len != NUM_BYTES_INDEX + NUM_BYTES_NUMWORDS_READREQUEST)  //read messages must have precisely this content (once addr and crc are removed, if relevant)
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        uint16_t num_bytes = 0;
        num_bytes |= (uint16_t)(input_buffer_base->buf[bidx++]);
        num_bytes |= (((uint16_t)(input_buffer_base->buf[bidx++])) << 8);
        if(num_bytes > reply_raw->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }

        
        unsigned char * cpy_ptr = mem_base->buf + word_offset;
        if(word_offset + num_bytes > mem_base->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }

        reply_raw->len = 0;
        for(uint16_t i = 0; i < num_bytes; i++)
        {
            reply_raw->buf[reply_raw->len++] = cpy_ptr[i];
        }
        return SERIAL_PROTOCOL_SUCCESS; //caller needs to finish the reply formatting
    }
    else    //write
    {
        //from our min length and min size checks, we know the input buffer must have both minimum size and
        //minimum length of 3, and we know that if we are here, bidx is equal to 2.
        //Therefore, it is safe to subtract bidx from len
        unsigned char * write_ptr = input_buffer_base->buf + bidx;
        size_t nbytes_to_write = input_buffer_base->len - bidx; //this can be 1 at minimum, and cannot underflow due to our checks above. overrun protection is guaranteed here too due to size and len checks
        
        if(word_offset + nbytes_to_write > mem_base->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
        unsigned char * mem_ptr = mem_base->buf + word_offset;
        for(int i = 0; i < nbytes_to_write; i++)
        {
            mem_ptr[i] = write_ptr[i];  //perform the copy
        }
        return SERIAL_PROTOCOL_SUCCESS; //no reply, so caller doesn't need to do anything else
    }
}

/*
    Master function to parse slave reply.
    input: copy of a buffer_t reference to the input buffer. This will be modified within the function scope to deal with message types (crc and address)
    type: the message type
    dest: location to copy the reply contents to (similar to mem in slave parse)
*/
int parse_read_reply(buffer_t input, serial_message_type_t type, buffer_t * dest)
{     
    if(dest == NULL)
    {
        return ERROR_INVALID_ARGUMENT;
    }
    if(dest->size == 0)
    {
        return ERROR_INVALID_ARGUMENT;
    }

    
    if(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE)    //crc filtering if relevant
    {
        if(input.len <= NUM_BYTES_CHECKSUM)
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        uint16_t crc = get_crc16(input.buf, input.len - NUM_BYTES_CHECKSUM);
        uint16_t msg_crc = 0;
        msg_crc |= (uint16_t)(input.buf[input.len - 2]);
        msg_crc |= (((uint16_t)(input.buf[input.len - 1])) << 8);
        if(crc != msg_crc)
        {
            return ERROR_CHECKSUM_MISMATCH;
        }
        input.len -= NUM_BYTES_CHECKSUM;
    }
    if(type == TYPE_SERIAL_MESSAGE)     //address filtering if relevant
    {
        if(input.len <= NUM_BYTES_ADDRESS)  //input.len now must be greater or equal to 2 for a valid message. CRC may have been removed
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        unsigned char addr = input.buf[0];
        if(addr != MASTER_MISC_ADDRESS)
        {
            return ADDRESS_FILTERED;
        }
        input.buf++;
        input.len--;
    }
    if(input.len > dest->size)
    {
        return ERROR_MEMORY_OVERRUN;
    }
    dest->len = 0;
    for(int i = 0; i < input.len; i++)
    {
        dest->buf[dest->len++] = input.buf[i];
    }
    return SERIAL_PROTOCOL_SUCCESS;

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
    if(type != TYPE_SERIAL_MESSAGE)
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

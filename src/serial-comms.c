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
    index = index | 0x8000; //set the read bit always
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

/*format an output buffer_t based on a serial message_t structure*/
int misc_message_to_serial_buf(misc_message_t * msg, serial_message_type_t type, buffer_t * output)
{

}

/*load a misc_message_t based on a serial input buffer */
int serial_buf_to_misc_message(buffer_t * input, serial_message_type_t type, misc_message_t * msg)
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
    uint16_t read_mask = index_argument & 0x8000;
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

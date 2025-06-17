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
int index_of_field(void * p_field, comms_t * comms)
{
    //null pointer checks
    if(p_field == NULL || comms == NULL)
    {
        return ERROR_INVALID_ARGUMENT;
    }
    
    // Ensure p_field is within the bounds of the comms struct
    unsigned char * pbase = (unsigned char *)(comms);
    unsigned char * p_field_nonvoid = (unsigned char *)p_field;
    
    //Check for underrun
    if(p_field_nonvoid < pbase)
    {
        return ERROR_INVALID_ARGUMENT;
    }

    size_t offset = p_field_nonvoid - pbase;
    
    // Check if the field is actually within the struct (overrun check)
    if(offset >= sizeof(comms_t))
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
        msg_buf: the buffer to store the message
        msg_buf_size: the size of the message buffer
*/
int create_misc_read_message(unsigned char address, uint16_t index, uint16_t num_words, buffer_t * msg)
{
    //basic error checking (bounds overrun, null pointer checks)
    if(msg == NULL)
    {
        return 0;
    }
    if(msg->size < NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + sizeof(uint16_t) + NUM_BYTES_CHECKSUM)
    {
        return 0;   //ensure there is enough space to load message
    }
    // add more bounds checking

    int cur_byte_index = 0;
    msg->buf[cur_byte_index++] = address;    //byte 0 loaded
    index = index | 0x8000; //set the read bit always
    unsigned char * p_index_word = (unsigned char *)(&index);
    msg->buf[cur_byte_index++] = p_index_word[0];    //byte 1 loaded
    msg->buf[cur_byte_index++] = p_index_word[1];    //byte 2 loaded

    unsigned char * p_num_words = (unsigned char *)(&num_words);
    msg->buf[cur_byte_index++] = p_num_words[0];    //byte 3 loaded
    msg->buf[cur_byte_index++] = p_num_words[1];    //byte 4 loaded

    uint16_t checksum = get_crc16(msg->buf, cur_byte_index);
    unsigned char * p_checksum = (unsigned char *)(&checksum);
    msg->buf[cur_byte_index++] = p_checksum[0];    //byte 5 loaded
    msg->buf[cur_byte_index++] = p_checksum[1];    //byte 6 loaded

    msg->len = cur_byte_index;
    return cur_byte_index;
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
int create_misc_write_message(unsigned char address, uint16_t index, buffer_t * payload, buffer_t * msg)
{
    //basic error checking (bounds overrun, null pointer checks)
    if(payload == NULL || msg == NULL)
    {
        return 0;
    }
    if(msg->size < (payload->len + NUM_BYTES_NON_PAYLOAD)) //fixed size
    {
        return 0;
    }
    
    //load the address
    int cur_byte_index = 0;
    msg->buf[cur_byte_index++] = address;
    
    //load the index, clear the read bit
    index = index & 0x7FFF;
    unsigned char * p_index_word = (unsigned char *)(&index);
    msg->buf[cur_byte_index++] = p_index_word[0];
    msg->buf[cur_byte_index++] = p_index_word[1];
    
    //load the payload
    for(int i = 0; i < payload->len; i++)
    {
        msg->buf[cur_byte_index++] = payload->buf[i];
    }

    //load the checksum
    uint16_t checksum = get_crc16(msg->buf, cur_byte_index);
    unsigned char * p_checksum = (unsigned char *)(&checksum);
    msg->buf[cur_byte_index++] = p_checksum[0];
    msg->buf[cur_byte_index++] = p_checksum[1];

    msg->len = cur_byte_index;
    return cur_byte_index;
}


/*
    Arguments:
        msg: the message buffer, excluding address and checksum. If byte stuffing is used, this must be the unstuffed message.
        len: the length of the message
        comms: the global comms struct
    Returns:
        -2 if the message is malformed
        -1 if the message is not intended for this device
        0 if the message is successfully parsed
 */
int parse_misc_command(buffer_t * msg, buffer_t * reply, comms_t * comms)
{
    if(msg == NULL) //  || msg->len < (NUM_BYTES_ADDRESS+NUM_BYTES_INDEX+NUM_BYTES_CHECKSUM) //message length check should have been done before we enter here
    {
        return ERROR_MALFORMED_MESSAGE;
    }

    uint16_t * p_index_argument = (uint16_t *)(&msg->buf[NUM_BYTES_ADDRESS]);		//previously this function offsetted with pointer logic before entry. That is dumb. We parse the whole message now with correct offsets
    uint16_t read_mask = *p_index_argument & 0x8000;
    uint16_t index = *p_index_argument & 0x7FFF;
    uint32_t byte_index = (uint32_t)(index*sizeof(uint32_t));
    
    if(read_mask == 0)    //
    {
        //write    
        int write_len = msg->len - (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM);   //We start after the index section, and go until we hit the checksum
        if(byte_index + write_len > sizeof(comms_t))
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        else
        {
            unsigned char * pcomms = (unsigned char *)(comms);
            pcomms = &pcomms[byte_index];
            unsigned char * pmsg = &msg->buf[NUM_BYTES_ADDRESS+NUM_BYTES_INDEX];  //skip past the index argument portion for the write payload
            for(int i = 0; i < write_len; i++)
            {
                pcomms[i] = pmsg[i];
            }
            reply->len = 0;
            return SERIAL_PROTOCOL_SUCCESS;
        }
    }
    else
    {
        //read
        if( (msg->len != (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + NUM_BYTES_NUMWORDS_READREQUEST + NUM_BYTES_CHECKSUM) ) || reply == NULL)
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        else
        {
            //read
            uint16_t * p_numread_words = (uint16_t*)(&msg->buf[NUM_BYTES_ADDRESS+NUM_BYTES_INDEX]);
            uint32_t numread_bytes = (uint32_t)(*p_numread_words * sizeof(uint32_t));
            if(numread_bytes + byte_index > sizeof(comms_t) || (numread_bytes + NUM_BYTES_CHECKSUM + NUM_BYTES_ADDRESS) > reply->size)	//pre-check size once
            {
                return ERROR_MALFORMED_MESSAGE;
            }
            else
            {
                unsigned char * p_comms = (unsigned char *)comms;
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
int parse_general_message(unsigned char address, buffer_t * msg, buffer_t * reply, comms_t * comms)
{
    if(msg->len < MINIMUM_MESSAGE_LENGTH) //minimum message length is 5 bytes (device address + register address + data + checksum)
    {
        return ERROR_MALFORMED_MESSAGE;
    }
    uint16_t * pchecksum = (uint16_t *)(msg->buf + msg->len - sizeof(uint16_t));
    uint16_t checksum = get_crc16(msg->buf, msg->len - sizeof(uint16_t));
    if(checksum != *pchecksum)
    {
        return ERROR_CHECKSUM_MISMATCH;
    }
    else
    {
        if(msg->buf[0] == address)
        {
            return parse_misc_command(msg, reply, comms);
        }
        else
        {
            return ADDRESS_FILTERED;
        }
    }
}

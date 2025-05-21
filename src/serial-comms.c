#include <stdint.h>
#include "serial-comms.h"
#include "checksum.h"

#define NUM_BYTES_INDEX 2
#define NUM_BYTES_CHECKSUM 2
#define NUM_BYTES_ADDRESS 1
#define NUM_BYTES_NON_PAYLOAD NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM + NUM_BYTES_ADDRESS

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
int create_write_message(unsigned char address, uint16_t index, unsigned char * payload, int payload_size, unsigned char * msg_buf, int msg_buf_size)
{
    //basic error checking (bounds overrun, null pointer checks)
    if(payload == NULL || msg_buf == NULL || msg_len == NULL)
    {
        return 0;
    }
    if(msg_buf_size < payload_size + NUM_BYTES_NON_PAYLOAD) //fixed size
    {
        return 0;
    }
    
    //load the address
    int cur_byte_index = 0;
    msg_buf[cur_byte_index++] = address;
    
    //load the index
    index = index & 0x7FFF;
    unsigned char * p_index_word = (unsigned char *)(&index);
    msg_buf[cur_byte_index++] = p_index_word[0];
    msg_buf[cur_byte_index++] = p_index_word[1];
    
    //load the payload
    for(int i = 0; i < payload_size; i++)
    {
        msg_buf[cur_byte_index++] = payload[i];
    }

    //load the checksum
    uint16_t checksum = get_checksum16(msg_buf, payload_size + NUM_BYTES_ADDRESS + NUM_BYTES_INDEX);
    unsigned char * p_checksum = (unsigned char *)(&checksum);
    msg_buf[cur_byte_index++] = p_checksum[0];
    msg_buf[cur_byte_index++] = p_checksum[1];

    return cur_byte_index;
}

/*
    Use the message protocol without splitting behavior based on address.
*/
int parse_general_message(unsigned char address, unsigned char * msg, int len, comms_t * comms)
{
    if(len < MINIMUM_MESSAGE_LENGTH) //minimum message length is 5 bytes (device address + register address + data + checksum)
    {
        return ERROR_MALFORMED_MESSAGE;
    }
    uint16_t * pchecksum = (uint16_t *)(msg + len - sizeof(uint16_t));
    uint16_t checksum = get_checksum16(msg, len - sizeof(uint16_t));
    if(checksum != *pchecksum)
    {
        return ERROR_MALFORMED_MESSAGE;
    }
    else
    {
        if(msg[0] == address)
        {
            //remove address and checksum from the message and then parse
            msg = &(msg[1]);
            return parse_misc_command(msg, len-2, comms);
        }
        else
        {
            return ADDRESS_FILTERED;
        }
    }
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
int parse_misc_command(unsigned char * msg, int len, comms_t * comms, unsigned char * p_replybuf, int * reply_len, int replybuf_size)
{
    if(msg == NULL || len < 4)
    {
        return ERROR_MALFORMED_MESSAGE;
    }

    uint16_t * p_index_argument = (uint16_t *)(&msg[0]);
    uint16_t read_mask = *p_index_argument & 0x8000;
    uint16_t index = *p_index_argument & 0x7FFF;
    uint32_t byte_index = (uint32_t)(index*sizeof(uint32_t));
    
    if(read_mask == 0)    //
    {
        //write    
        int write_len = len - sizeof(uint16_t);   //index argument parsed, skip it
        if(byte_index + write_len >= sizeof(comms))
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        else
        {
            unsigned char * pcomms = (unsigned char *)(comms);
            pcomms = &pcomms[byte_index];
            unsigned char * pmsg = &msg[sizeof(uint16_t)];  //skip past the index argument portion for the write payload
            for(int i = 0; i < write_len; i++)
            {
                pcomms[i] = pmsg[i];
            }
            *reply_len = 0;
            return SUCCESS;
        }
    }
    else
    {
        //read
        if(len != 4 && p_replybuf == NULL)
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        else
        {
            //read
            uint16_t * p_numread_words = (uint16_t*)(&msg[2]);
            uint32_t numread_bytes = (uint32_t)(*p_numread_words * sizeof(uint32_t));
            if(numread_bytes + byte_index >= sizeof(comms) || numread_bytes >= replybuf_size)
            {
                return ERROR_MALFORMED_MESSAGE;
            }
            else
            {
                for(int i = 0; i < numread_bytes; i++)
                {
                    p_replybuf[i] = ((unsigned char *)(&comms))[byte_index + i];
                }   
                *reply_len = numread_bytes;
                return SUCCESS;
            }
        }
    }
}

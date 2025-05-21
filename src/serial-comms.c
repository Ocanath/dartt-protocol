#include <stdint.h>
#include "serial-comms.h"
#include "checksum.h"


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

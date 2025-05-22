#include "serial-comms.h"
#include "checksum.h"

enum {REPLY_MODE_1 = 0, REPLY_MODE_2 = 1};

/**
TODO: fill this out based on your project needs. 
 */
int cherrypick_reply_data(unsigned char * reply, int reply_len, comms_t * comms)
{
    if(reply == NULL || comms == NULL)
    {
        return 0;
    }
    else if(comms->motor_command_mode == REPLY_MODE_1)
    {
        if(reply_len < sizeof(int32_t) * 2) //check that the target buffer is large enough
        {
            return 0;
        }

        int bidx = 0;   //index for the reply buffer

        //load the first value
        unsigned char * p_val = (unsigned char *)(&comms->gl_iq);
        for(int i = 0; i < sizeof(int32_t); i++)
        {
            reply[bidx++] = p_val[i];
        }

        //load the second value
        p_val = (unsigned char *)(&comms->gl_joint_theta);
        for(int i = 0; i < sizeof(int32_t); i++)
        {
            reply[bidx++] = p_val[i];
        }
        return bidx;
    }
    else if(comms->motor_command_mode == REPLY_MODE_2)
    {
        //ETC.
    }
}



/*
    Parse an unstuffed (raw) message, including checksum and address splitting.
    Returns:
        -2 if the message is malformed
        -1 if the message is not intended for this device
        0 if the message is successfully parsed

    This function does address filtering and checksum validation
 */
int parse_motor_message(unsigned char motor_address, unsigned char misc_address, unsigned char * msg, int len, unsigned char * p_replybuf, int replybuf_size, int * reply_len,  comms_t * comms)
{   
    if(len < MINIMUM_MESSAGE_LENGTH)
    {
        return ERROR_MALFORMED_MESSAGE;
    }
    uint16_t * pchecksum = (uint16_t *)(msg + len - sizeof(uint16_t));
    uint16_t checksum = get_crc16(msg, len - sizeof(uint16_t));
    if(checksum != *pchecksum)
    {
        return ERROR_CHECKSUM_MISMATCH;
    }
    else
    {
        if(msg[0] == motor_address)
        {
            int32_t * p_cmd = (int32_t *)(&msg[1]);
            //do something with the command we parsed
            cherrypick_reply_data(p_replybuf, replybuf_size, comms);
        }
        else if(msg[0] == misc_address)
        {
            //remove address and checksum from the message and then parse
            msg = &(msg[1]);
            return parse_misc_command(msg, (len-(NUM_BYTES_CHECKSUM + NUM_BYTES_ADDRESS)), p_replybuf, replybuf_size, reply_len, comms);
        }
        else
        {
            return ADDRESS_FILTERED;
        }
    }
}

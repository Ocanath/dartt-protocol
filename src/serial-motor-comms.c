#include "serial-comms.h"

/*
    Parse an unstuffed (raw) message, including checksum and address splitting.
    Returns:
        -2 if the message is malformed
        -1 if the message is not intended for this device
        0 if the message is successfully parsed

    This function does address filtering and checksum validation
 */
int parse_motor_message(unsigned char motor_address, unsigned char misc_address, unsigned char * msg, int len, comms_t * comms)
{
    if(len < MINIMUM_MESSAGE_LENGTH) //minimum message length is 5 bytes (device address + register address + data + checksum)
    {
        return -2;
    }
    uint16_t * pchecksum = (uint16_t *)(msg + len - sizeof(uint16_t));
    uint16_t checksum = get_checksum16(msg, len - sizeof(uint16_t));
    if(checksum != *pchecksum)
    {
        return -2;
    }
    else
    {
        if(msg[0] == motor_address)
        {
            return parse_motor_command(msg, len, comms);
        }
        else if(msg[0] == misc_address)
        {
            return parse_misc_command(msg, len, comms);
        }
        else
        {
            return -1;
        }
    }    
}

/*
    Special case for motor commands.
    Returns:
        -2 if the message is malformed
        -1 if the message is not intended for this device
        0 if the message is successfully parsed
    p_reply: pointer to the beginning of the memory you should pipe out over serial in reply to the message
    
    This command is special, because instead of sending out a region of the comms struct, it cherry picks data from the comms struct and sends it out in a reply buffer.
    It will do this based on the reply_mode field in the comms struct.
 */
int parse_motor_command(unsigned char * msg, int len, comms_t * comms, unsigned char * p_replybuf, int * reply_len, int replybuf_size)
{

}
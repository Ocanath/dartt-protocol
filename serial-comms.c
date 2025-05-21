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
        if(msg[0] == address)
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
    Arguments:
        msg: the message buffer, excluding checksum. If byte stuffing is used, this must be the unstuffed message.
        len: the length of the message
        comms: the global comms struct
    Returns:
        -2 if the message is malformed
        -1 if the message is not intended for this device
        0 if the message is successfully parsed
 */
int parse_misc_command(unsigned char * msg, int len, comms_t * comms)
{

}
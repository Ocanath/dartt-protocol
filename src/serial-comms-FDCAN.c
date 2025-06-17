/*
 * serial-comms-FDCAN.c
 *
 *  Created on: Jun 17, 2025
 *      Author: ocanath
 */
#include "serial-comms-FDCAN.h"
#include "checksum.h"

/*
 * Take a 'serial write' data frame and parse it into a can struct (i.e. separate out the fields for address, ID, and checksum)
 * This adds a bit of compute overhead and a redundant crc, so it's better to load the can_frame directly rather than
 * */
int create_can_frame_from_message(buffer_t * msg, can_frame_t * canframe)
{
	if(msg == NULL || canframe == NULL)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	canframe->id = (uint32_t)msg->buf[0]; //address is always the first argument
    canframe->data = (uint8_t*)(&msg->buf[NUM_BYTES_ADDRESS]);	//start the data section after the address. Instead of copying the buffer, use a pointer
    canframe->length = (uint32_t)(msg->len - (NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM));	//CRC and address are built-in for CAN, so chop them off the length
    return SERIAL_PROTOCOL_SUCCESS;
}


/*
 * Take a CAN frame and load it into a buffer_t
 * This does require copying
 * */
int create_message_from_can_frame(can_frame_t * canframe, buffer_t * msg)
{
	if(msg == NULL || canframe == NULL)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	//load the address
	int cur_byte_index = 0;
	msg->buf[cur_byte_index++] = (uint8_t)(canframe->id & 0xFF);
	//bounds check for array overrun protection
	if(canframe->length + cur_byte_index >= msg->size)
	{
		return ERROR_INVALID_ARGUMENT;	//don't overrun the message buffer. pre-check before loop entry
	}
	//copy the payload section
	for(int i = 0; i < canframe->length; i++)
	{
		msg->buf[cur_byte_index++] = canframe->data[i];
	}
	//compute and load teh checksum
    uint16_t checksum = get_crc16(msg->buf, cur_byte_index);
    unsigned char * p_checksum = (unsigned char *)(&checksum);
    msg->buf[cur_byte_index++] = p_checksum[0];    //byte 5 loaded
    msg->buf[cur_byte_index++] = p_checksum[1];    //byte 6 loaded
    //done, load size and return
    msg->len = cur_byte_index;
    return SERIAL_PROTOCOL_SUCCESS;
}

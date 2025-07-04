/*
 * serial-comms-FDCAN.h
 *
 *  Created on: Jun 17, 2025
 *      Author: ocanath
 */

#ifndef SRC_SERIAL_COMMS_FDCAN_H_
#define SRC_SERIAL_COMMS_FDCAN_H_
#include "serial-comms.h"
#include "serial-comms-struct.h"

typedef struct can_frame_t
{
	uint32_t id;	//loaded with the device address
	uint32_t length;
	uint8_t * data;	//the can 'payload' content
}can_frame_t;

int create_can_frame_from_message(buffer_t * msg, can_frame_t * canframe);
int create_message_from_can_frame(can_frame_t * canframe, buffer_t * msg);

#endif /* SRC_SERIAL_COMMS_FDCAN_H_ */

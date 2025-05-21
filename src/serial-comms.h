#ifndef SERIAL_COMMS_H  
#define SERIAL_COMMS_H
#include "serial-comms-struct.h"

#define MINIMUM_MESSAGE_LENGTH 5

enum {ERROR_MALFORMED_MESSAGE = -2, ADDRESS_FILTERED = -1, SUCCESS = 0};

int create_write_message(unsigned char address, uint16_t index, unsigned char * payload, int payload_size, unsigned char * msg_buf, int msg_buf_size);
int parse_general_message(unsigned char address, unsigned char * msg, int len, comms_t * comms);

#endif


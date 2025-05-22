#ifndef SERIAL_COMMS_H  
#define SERIAL_COMMS_H
#include "serial-comms-struct.h"

#define MINIMUM_MESSAGE_LENGTH 5

enum {ERROR_CHECKSUM_MISMATCH = -3, ERROR_MALFORMED_MESSAGE = -2, ADDRESS_FILTERED = -1, SUCCESS = 0};

int create_misc_write_message(unsigned char address, uint16_t index, unsigned char * payload, int payload_size, unsigned char * msg_buf, int msg_buf_size);
int create_misc_read_message(unsigned char address, uint16_t index, uint16_t num_words, unsigned char * msg_buf, int msg_buf_size);
int parse_general_message(unsigned char address, unsigned char * msg, int len, unsigned char * reply_buf, int replybuf_size, int * reply_len, comms_t * comms);
int parse_misc_command(unsigned char * msg, int len, unsigned char * p_replybuf, int replybuf_size, int * reply_len, comms_t * comms);

#endif


#ifndef SERIAL_COMMS_H  
#define SERIAL_COMMS_H
#include <stdint.h>
#include <stddef.h>
#include "serial-comms-struct.h"


#define NUM_BYTES_INDEX 2
#define NUM_BYTES_CHECKSUM 2
#define NUM_BYTES_ADDRESS 1
#define NUM_BYTES_NON_PAYLOAD (NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM + NUM_BYTES_ADDRESS)
#define MINIMUM_MESSAGE_LENGTH NUM_BYTES_NON_PAYLOAD

enum {ERROR_INVALID_ARGUMENT = -4, ERROR_CHECKSUM_MISMATCH = -3, ERROR_MALFORMED_MESSAGE = -2, ADDRESS_FILTERED = -1, SUCCESS = 0};


int create_misc_write_message(unsigned char address, uint16_t index, unsigned char * payload, int payload_size, unsigned char * msg_buf, int msg_buf_size);
int create_misc_read_message(unsigned char address, uint16_t index, uint16_t num_words, unsigned char * msg_buf, int msg_buf_size);
int parse_general_message(unsigned char address, unsigned char * msg, int len, unsigned char * reply_buf, int replybuf_size, int * reply_len, comms_t * comms);
int parse_misc_command(unsigned char * msg, int len, unsigned char * p_replybuf, int replybuf_size, int * reply_len, comms_t * comms);
int index_of_field(void * p_field, comms_t * comms);

#endif


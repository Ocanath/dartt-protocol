#ifndef SERIAL_COMMS_H  
#define SERIAL_COMMS_H
#include <stdint.h>
#include <stddef.h>

#define NUM_BYTES_ADDRESS sizeof(unsigned char)
#define NUM_BYTES_INDEX sizeof(uint16_t)
#define NUM_BYTES_NUMWORDS_READREQUEST	sizeof(uint16_t)	//for a read struct request, we send a fixed 16bit integer argument in the payload section for the readsize request
#define NUM_BYTES_CHECKSUM sizeof(uint16_t)
#define NUM_BYTES_NON_PAYLOAD (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM)
#define MINIMUM_MESSAGE_LENGTH NUM_BYTES_NON_PAYLOAD

//This is a fixed address that always corresponds
#define MASTER_MOTOR_ADDRESS	0x7F
#define MASTER_MISC_ADDRESS		(0xFF - 0x7F)

enum {ERROR_INVALID_ARGUMENT = -4, ERROR_CHECKSUM_MISMATCH = -3, ERROR_MALFORMED_MESSAGE = -2, ADDRESS_FILTERED = -1, SERIAL_PROTOCOL_SUCCESS = 0};

typedef enum {TYPE_UART_MESSAGE = 0, TYPE_CAN_MESSAGE = 1} serial_message_type_t;	

typedef struct buffer_t
{
	unsigned char * buf;
	int size;
	int len;
} buffer_t;

typedef struct misc_message_t
{
	uint8_t address;	//the address
	uint16_t index;	//MSB is read (1) / write (0)
	buffer_t * payload;	//the content of the messasge
}misc_message_t;



int create_misc_write_message(unsigned char address, uint16_t index, buffer_t * payload, buffer_t * msg);
int create_misc_read_message(unsigned char address, uint16_t index, uint16_t num_words, buffer_t * msg);
int parse_general_message(unsigned char address, buffer_t * msg, serial_message_type_t type, buffer_t * reply, void * mem, size_t mem_size);
int parse_misc_command(buffer_t * msg, serial_message_type_t type, buffer_t * reply, void * mem, size_t mem_size);
int index_of_field(void * p_field, void * mem, size_t mem_size);

#endif


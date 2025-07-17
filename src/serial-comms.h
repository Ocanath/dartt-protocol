#ifndef SERIAL_COMMS_H  
#define SERIAL_COMMS_H
#include <stdint.h>
#include <stddef.h>

/*
	TODO: put the information in this comment into the README
	
	WE ARE ALWAYS LITTLE ENDIAN.
	This library is meant to be C-standard compliant, but to be used in a way which is technically NOT standard compliant (type punning).
	Specifically, if you are targeting little-endian 32bit architectures, such as an ARM STM32, you *can* set this up so that the generic memory
	regions you are reading and writing to are typedef structs (with the packed attribute if desired). This means you don't need any additional
	parsing layer - once you load the memory from an incoming write request frame, it is immediately accessible via the struct.

	The rationale is that:
		1. if you control the target architecture and it never changes, this optimization will always work
		2. DX improves drastically if you don't have to implement explicit parsing. Laying out your memory as a struct means you never have to write parsers for general settings messages - 
			only for very tight and specific message types!
*/

/*
	TODO:
		unit test set_rw and set_index. Make sure they are order independent as desired
		unit test the forward and reverse functions (msg to buf and buf to msg)
		refactor the parse function to use the above
		refactor the unit tests to use only the new functions; remove create_msic_write, create_misc_read, and parse_misc_command. Keep parse_general and index_of_field. Fix warnings in index_of_field - ensure casting is explicit
*/

#define NUM_BYTES_ADDRESS sizeof(unsigned char)
#define NUM_BYTES_INDEX sizeof(uint16_t)
#define NUM_BYTES_NUMWORDS_READREQUEST	sizeof(uint16_t)	//for a read struct request, we send a fixed 16bit integer argument in the payload section for the readsize request
#define NUM_BYTES_CHECKSUM sizeof(uint16_t)
#define NUM_BYTES_NON_PAYLOAD (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM)
#define MINIMUM_MESSAGE_LENGTH NUM_BYTES_NON_PAYLOAD

//This is a fixed address that always corresponds
#define MASTER_MOTOR_ADDRESS	0x7F
#define MASTER_MISC_ADDRESS		(0xFF - 0x7F)

#define READ_WRITE_BITMASK	0x8000	//msg is the read write bit. 1 for read, 0 for write.

enum {ERROR_MEMORY_OVERRUN = -5, ERROR_INVALID_ARGUMENT = -4, ERROR_CHECKSUM_MISMATCH = -3, ERROR_MALFORMED_MESSAGE = -2, ADDRESS_FILTERED = -1, SERIAL_PROTOCOL_SUCCESS = 0};

typedef enum {TYPE_UART_MESSAGE = 0, TYPE_CAN_MESSAGE = 1} serial_message_type_t;	

typedef enum {WRITE_MESSAGE, READ_MESSAGE} read_write_type_t;

typedef struct buffer_t
{
	unsigned char * buf;
	int size;
	int len;
} buffer_t;

typedef struct misc_message_t
{
	uint8_t address;	//the address
	uint16_t rw_index;	//MSB is read (1) / write (0). The low 15 bits are an index argument, corresponding to a 4-byte index offset from a base pointer. memory is assumed to be 32-bit aligned
	buffer_t payload;	//the content of the messasge
	//the checksum is computed in the message loader function, iff the hardware doesn't support it inherently. Therefore it is considered 'user alterable' data and not part of the message structure.
}misc_message_t;



int create_misc_write_message(unsigned char address, uint16_t index, buffer_t * payload, buffer_t * msg);
int create_misc_read_message(unsigned char address, uint16_t index, uint16_t num_words, buffer_t * msg);
int parse_general_message(unsigned char address, buffer_t * msg, serial_message_type_t type, buffer_t * reply, void * mem, size_t mem_size);
int parse_misc_command(buffer_t * msg, serial_message_type_t type, buffer_t * reply, void * mem, size_t mem_size);
int index_of_field(void * p_field, void * mem, size_t mem_size);

#endif


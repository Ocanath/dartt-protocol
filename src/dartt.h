#ifndef DARTT_H  
#define DARTT_H
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_BYTES_ADDRESS sizeof(unsigned char)
#define NUM_BYTES_INDEX sizeof(uint16_t)
#define NUM_BYTES_NUMWORDS_READREQUEST	sizeof(uint16_t)	//for a read struct request, we send a fixed 16bit integer argument in the payload section for the readsize request
#define NUM_BYTES_CHECKSUM sizeof(uint16_t)
#define NUM_BYTES_NON_PAYLOAD (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM)
#define MINIMUM_MESSAGE_LENGTH NUM_BYTES_NON_PAYLOAD
//
#define NUM_BYTES_READ_REPLY_OVERHEAD_PLD NUM_BYTES_INDEX	//non frame layer overhead in read replies. 

//This is a fixed address that always corresponds
#define MASTER_MOTOR_ADDRESS	0x7F
#define MASTER_MISC_ADDRESS		(0xFF - 0x7F)

#define READ_WRITE_BITMASK	0x8000	//msg is the read write bit. 1 for read, 0 for write.

enum {ERROR_CTL_READ_LEN_MISMATCH = -7, ERROR_SYNC_MISMATCH = -6, ERROR_MEMORY_OVERRUN = -5, ERROR_INVALID_ARGUMENT = -4, ERROR_CHECKSUM_MISMATCH = -3, ERROR_MALFORMED_MESSAGE = -2, ADDRESS_FILTERED = -1, DARTT_PROTOCOL_SUCCESS = 0};

/*
 * Flags to capture byte field definitions for different physical and link layer protocols,
 	such as UART, CAN, FDCAN, UDP, SPI, and I2C.
 */
typedef enum 
{
	TYPE_SERIAL_MESSAGE = 0, 	//raw serial bytes. Must include our own address filtering and CRC appending. Examples: UART, RS485, RS232
	TYPE_ADDR_MESSAGE = 1,	//built-in addressing, but no build-in CRC. CRC must be added to the payload Examples: SPI, I2C
	TYPE_ADDR_CRC_MESSAGE = 2,	//built-in address filtering and CRC filtering-no address or crc fields required in payload. Examples: CAN, UDP
	
} serial_message_type_t;	

typedef enum {WRITE_MESSAGE, READ_MESSAGE} read_write_type_t;

typedef enum {
	PAYLOAD_ALIAS,	//indicates that the payload section should be aliased, i.e. .buf points to part of the original frame and the len and size are loaded accordingly
	PAYLOAD_COPY	//copy the relevant parts of the frame to the pld buffer
} payload_mode_t;	

typedef struct dartt_buffer_t
{
	unsigned char * buf;
	size_t size;	//size of the buffer
	size_t len;	//length of the current message in the buffer, referenced to the zero index
} dartt_buffer_t;

/*
	Containter struct for base protocol messages.
	address: the address of the associated message
	msg: the base message, stripped of any address and crc information.
		-if a motor message, this is a custom format designed to be very tight/fast
		-if a general message, this follows the [r][index][nbytes]/[w][index][payload] pattern.
	reply: if there is a reply, it will be contained in this dartt_buffer_t
*/
typedef struct payload_layer_msg_t
{
	unsigned char address;
	dartt_buffer_t msg;
} payload_layer_msg_t;

/*
	Contatiner struct for frame layer messages.
	Depending on type, these will have:
		TYPE_SERIAL_MESSAGE: address (byte 0) - crc (last two bytes)
		TYPE_ADDR_MESSAGE: crc (last two bytes)
		TYPE_ADDR_CRC_MESSAGE: neither address nor crc - the frame_msg contents are equal to the payload layer contents
*/
// typedef struct frame_layer_msg_t
// {
// 	serial_message_type_t type;
// 	dartt_buffer_t * ser_msg;
// }frame_layer_msg_t;

/*
Master write message/write request
 */
typedef struct misc_write_message_t
{
	unsigned char address;		//slave destination address
	uint16_t index;		//32bit-aligned index offset, where we want the payload to start writing to
	dartt_buffer_t payload;	//the content of the message, bytewise, which we will be writing
	//the checksum is computed in the message loader function, iff the hardware doesn't support it inherently. Therefore it is considered 'user alterable' data and not part of the message structure.
}misc_write_message_t;

/*
Master read message/read request
 */
typedef struct misc_read_message_t
{
	unsigned char address;		//slave destination address
	uint16_t index;		//32bit-aligned index offset, where we want the payload to start reading from
	uint16_t num_bytes;	//2^16 byte read requests at a time maximum. Not recommended to use buffers this large. 
}misc_read_message_t;

/*
Slave reply message
 */
typedef struct misc_reply_t
{
	unsigned char address;	
	dartt_buffer_t reply;
}misc_reply_t;

// // Consider: Context struct
// typedef struct 
// {
// 	unsigned char address;
// 	serial_message_type_t type;
// 	dartt_buffer_t * mem_base;
// } protocol_context_t;

int index_of_field(void * p_field, void * mem, size_t mem_size);
int copy_buf_full(dartt_buffer_t * in, dartt_buffer_t * out);
unsigned char dartt_get_complementary_address(unsigned char address);
int dartt_create_write_frame(misc_write_message_t * msg, serial_message_type_t type, dartt_buffer_t * output);
int dartt_create_read_frame(misc_read_message_t * msg, serial_message_type_t type, dartt_buffer_t * output);
int dartt_frame_to_payload(dartt_buffer_t * ser_msg, serial_message_type_t type, payload_mode_t pld_mode, payload_layer_msg_t * pld);
int dartt_parse_base_serial_message(payload_layer_msg_t* pld_msg, dartt_buffer_t * mem_base, dartt_buffer_t * reply_base);
int dartt_parse_general_message(payload_layer_msg_t * pld_msg, serial_message_type_t type, dartt_buffer_t * mem_base, dartt_buffer_t * reply);
int append_crc(dartt_buffer_t * input);
int validate_crc(const dartt_buffer_t * input);
int dartt_parse_read_reply(payload_layer_msg_t * payload_msg, misc_read_message_t * original_msg, dartt_buffer_t * dest);

#ifdef __cplusplus
}
#endif


#endif


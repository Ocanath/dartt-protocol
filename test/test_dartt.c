#include "checksum.h"
#include "dartt.h"
#include "unity.h"
#include "dartt_check_buffer.h"
/*
	TODO:
		Add test of dartt_frame_to_payload of a type 0 serial message consisting of only address and crc
		Add test of dartt_frame_to_payload for a motor command and a dual address command
 
		index_of_field - minimum overrun (index/offset is base+size, not base+size-1)
		all index of field error conditions - at least one assert per case
*/

//example structure for block mem testing. Further testing in test_struct
typedef struct comms_t
{
    uint32_t motor_command_mode;

    /*Block off the words corresponding to default motor send values, so they can be sent in blocks */
    int32_t gl_iq;
    int32_t gl_joint_theta;
    int32_t gl_rotor_theta;
    int32_t gl_rotor_velocity;

    //other random motor specific commands
    int32_t gl_id;

	int32_t command_word;

    uint32_t write_filesystem_flag;
    //more fields


} comms_t;




void test_check_write_args(void)
{
	int rc;
	
	// Test memory overrun for TYPE_SERIAL_MESSAGE
	{
		unsigned char wpld[3] = {};
		misc_write_message_t msg = {
			.address = 0x12,
			.index = 0,
			.payload = {
				.buf = wpld,
				.size = sizeof(wpld),
				.len = sizeof(wpld)
			}
		};
		unsigned char output_buf[3] = {};
		dartt_buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};

		rc = check_write_lengths(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);
	}
	
	// Test NULL argument validation
	{
		misc_write_message_t msg = {};
		dartt_buffer_t output = {};
		rc = check_write_args(&msg, 5, &output);
		TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);
		rc = check_write_args(NULL, 0, &output);
		TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);
		rc = check_write_args(&msg, 0, NULL);
		TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);
	}
	
	// Test invalid message type
	{
		unsigned char wpld[4] = {};
		misc_write_message_t msg = {
			.address = 0x12,
			.index = 0,
			.payload = {
				.buf = wpld,
				.size = sizeof(wpld),
				.len = sizeof(wpld)
			}
		};
		unsigned char output_buf[10] = {};
		dartt_buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_args(&msg, 99, &output);
		TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);
	}
	
	// Test payload.len == 0
	{
		unsigned char wpld[4] = {};
		misc_write_message_t msg = {
			.address = 0x12,
			.index = 0,
			.payload = {
				.buf = wpld,
				.size = sizeof(wpld),
				.len = 0  // Zero length
			}
		};
		unsigned char output_buf[10] = {};
		dartt_buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_lengths(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);
	}
	
	// Test payload.buf == NULL
	{
		misc_write_message_t msg = {
			.address = 0x12,
			.index = 0,
			.payload = {
				.buf = NULL,  // NULL buffer
				.size = 4,
				.len = 4
			}
		};
		unsigned char output_buf[10] = {};
		dartt_buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);
	}
	
	// Test output.buf == NULL
	{
		unsigned char wpld[4] = {};
		misc_write_message_t msg = {
			.address = 0x12,
			.index = 0,
			.payload = {
				.buf = wpld,
				.size = sizeof(wpld),
				.len = sizeof(wpld)
			}
		};
		dartt_buffer_t output = {
			.buf = NULL,  // NULL output buffer
			.size = 10,
			.len = 0
		};
		
		rc = check_write_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);
	}
	
	// Test successful TYPE_SERIAL_MESSAGE (address + index + checksum overhead)
	{
		unsigned char wpld[4] = {};
		misc_write_message_t msg = {
			.address = 0x12,
			.index = 0,
			.payload = {
				.buf = wpld,
				.size = sizeof(wpld),
				.len = sizeof(wpld)
			}
		};
		unsigned char output_buf[10] = {};  // 4 + 1 + 2 + 2 = 9 bytes needed, 10 available
		dartt_buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	}
	
	// Test memory overrun for TYPE_ADDR_MESSAGE (index + checksum overhead)
	{
		unsigned char wpld[6] = {};
		misc_write_message_t msg = {
			.address = 0x12,
			.index = 0,
			.payload = {
				.buf = wpld,
				.size = sizeof(wpld),
				.len = sizeof(wpld)
			}
		};
		unsigned char output_buf[8] = {};  // 6 + 2 + 2 = 10 bytes needed, only 8 available
		dartt_buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_lengths(&msg, TYPE_ADDR_MESSAGE, &output);
		TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);
	}
	
	// Test successful TYPE_ADDR_MESSAGE
	{
		unsigned char wpld[4] = {};
		misc_write_message_t msg = {
			.address = 0x12,
			.index = 0,
			.payload = {
				.buf = wpld,
				.size = sizeof(wpld),
				.len = sizeof(wpld)
			}
		};
		unsigned char output_buf[8] = {};  // 4 + 2 + 2 = 8 bytes needed
		dartt_buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_args(&msg, TYPE_ADDR_MESSAGE, &output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	}
	
	// Test memory overrun for TYPE_ADDR_CRC_MESSAGE (index overhead only)
	{
		unsigned char wpld[6] = {};
		misc_write_message_t msg = {
			.address = 0x12,
			.index = 0,
			.payload = {
				.buf = wpld,
				.size = sizeof(wpld),
				.len = sizeof(wpld)
			}
		};
		unsigned char output_buf[7] = {};  // 6 + 2 = 8 bytes needed, only 7 available
		dartt_buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_lengths(&msg, TYPE_ADDR_CRC_MESSAGE, &output);
		TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);
	}
	
	// Test successful TYPE_ADDR_CRC_MESSAGE
	{
		unsigned char wpld[4] = {};
		misc_write_message_t msg = {
			.address = 0x12,
			.index = 0,
			.payload = {
				.buf = wpld,
				.size = sizeof(wpld),
				.len = sizeof(wpld)
			}
		};
		unsigned char output_buf[8] = {};  // 4 + 2 = 6 bytes needed, 8 available
		dartt_buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_args(&msg, TYPE_ADDR_CRC_MESSAGE, &output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
		rc = check_write_lengths(&msg, TYPE_ADDR_CRC_MESSAGE, &output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	}
	
	// Test edge case: minimum payload size (1 byte)
	{
		unsigned char wpld[1] = {0x42};
		misc_write_message_t msg = {
			.address = 0x12,
			.index = 0,
			.payload = {
				.buf = wpld,
				.size = sizeof(wpld),
				.len = sizeof(wpld)
			}
		};
		unsigned char output_buf[8] = {};  // 1 + 1 + 2 + 2 = 6 bytes needed
		dartt_buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	}
}

void test_check_read_args(void)
{
	int rc;
	
	// Test NULL argument validation - msg == NULL
	{
		dartt_buffer_t output = {
			.buf = (unsigned char[10]){},
			.size = 10,
			.len = 0
		};
		
		rc = check_read_args(NULL, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);
	}
	
	// Test NULL argument validation - output == NULL
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 4
		};
		
		rc = check_read_args(&msg, TYPE_SERIAL_MESSAGE, NULL);
		TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);
	}
	
	// Test output.buf == NULL
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 4
		};
		dartt_buffer_t output = {
			.buf = NULL,
			.size = 10,
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);
	}
	
	// Test memory overrun for TYPE_SERIAL_MESSAGE (address + index + num_bytes + checksum)
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 4
		};
		dartt_buffer_t output = {
			.buf = (unsigned char[4]){},
			.size = 4,  // Need 1 + 2 + 2 + 2 = 7 bytes, only have 4
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);
	}
	
	// Test successful TYPE_SERIAL_MESSAGE
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 4
		};
		dartt_buffer_t output = {
			.buf = (unsigned char[10]){},
			.size = 10,  // 1 + 2 + 2 + 2 = 7 bytes needed, 10 available
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	}
	
	// Test memory overrun for TYPE_ADDR_MESSAGE (index + num_bytes + checksum)
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 4
		};
		dartt_buffer_t output = {
			.buf = (unsigned char[5]){},
			.size = 5,  // Need 2 + 2 + 2 = 6 bytes, only have 5
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_ADDR_MESSAGE, &output);
		TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);
	}
	
	// Test successful TYPE_ADDR_MESSAGE
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 4
		};
		dartt_buffer_t output = {
			.buf = (unsigned char[8]){},
			.size = 8,  // 2 + 2 + 2 = 6 bytes needed, 8 available
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_ADDR_MESSAGE, &output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	}
	
	// Test memory overrun for TYPE_ADDR_CRC_MESSAGE (index + num_bytes only)
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 4
		};
		dartt_buffer_t output = {
			.buf = (unsigned char[3]){},
			.size = 3,  // Need 2 + 2 = 4 bytes, only have 3
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_ADDR_CRC_MESSAGE, &output);
		TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);
	}
	
	// Test successful TYPE_ADDR_CRC_MESSAGE
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 4
		};
		dartt_buffer_t output = {
			.buf = (unsigned char[6]){},
			.size = 6,  // 2 + 2 = 4 bytes needed, 6 available
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_ADDR_CRC_MESSAGE, &output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	}
	
	// Test invalid message type
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 4
		};
		dartt_buffer_t output = {
			.buf = (unsigned char[10]){},
			.size = 10,
			.len = 0
		};
		
		rc = check_read_args(&msg, 99, &output);
		TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);
	}
	
	// Test edge case: minimum output buffer for TYPE_SERIAL_MESSAGE
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 1
		};
		dartt_buffer_t output = {
			.buf = (unsigned char[7]){},
			.size = 7,  // Exactly 1 + 2 + 2 + 2 = 7 bytes needed
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	}
	
	// Test edge case: minimum output buffer for TYPE_ADDR_MESSAGE
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 1
		};
		dartt_buffer_t output = {
			.buf = (unsigned char[6]){},
			.size = 6,  // Exactly 2 + 2 + 2 = 6 bytes needed
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_ADDR_MESSAGE, &output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	}
	
	// Test edge case: minimum output buffer for TYPE_ADDR_CRC_MESSAGE
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 1
		};
		dartt_buffer_t output = {
			.buf = (unsigned char[4]){},
			.size = 4,  // Exactly 2 + 2 = 4 bytes needed
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_ADDR_CRC_MESSAGE, &output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	}
	
	// Test large read request
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 1000
		};
		dartt_buffer_t output = {
			.buf = (unsigned char[1010]){},
			.size = 1010,  // 1000 + 1 + 2 + 2 + 2 = 1007 bytes needed, 1010 available
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	}
	
	// Test zero num_bytes (edge case - should still work)
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 0
		};
		dartt_buffer_t output = {
			.buf = (unsigned char[10]){},
			.size = 10,
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	}
}

/*Test-scope only - helper functions to parse frames back into structs 
Reciprocal test function to dartt_create_write_frame - primary use case is testing dartt_create_write_frame.
*/
int write_frame_to_struct(dartt_buffer_t * input, serial_message_type_t type, misc_write_message_t * msg)
{
	if(input == NULL || msg == NULL || input->buf == NULL)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	if(!(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE || type == TYPE_ADDR_CRC_MESSAGE))
	{
		return ERROR_INVALID_ARGUMENT;
	}
	
	size_t bidx = 0;
	
	// Parse address if present
	if(type == TYPE_SERIAL_MESSAGE)
	{
		if(input->len < NUM_BYTES_ADDRESS)
		{
			return ERROR_MALFORMED_MESSAGE;
		}
		msg->address = input->buf[bidx++];
	}
	else
	{
		msg->address = 0; // Default for non-addressed types
	}
	
	// Parse index (always present)
	if(input->len < bidx + NUM_BYTES_INDEX)
	{
		return ERROR_MALFORMED_MESSAGE;
	}
	uint16_t rw_index = 0;
	rw_index |= (uint16_t)(input->buf[bidx++]);
	rw_index |= (((uint16_t)(input->buf[bidx++])) << 8);
	
	// Check if this is actually a write frame (MSB should be 0)
	if(rw_index & READ_WRITE_BITMASK)
	{
		return ERROR_MALFORMED_MESSAGE; // This is a read frame, not write
	}
	msg->index = rw_index;
	
	// Calculate expected total frame size
	size_t expected_overhead = bidx; // address + index
	if(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE)
	{
		expected_overhead += NUM_BYTES_CHECKSUM;
	}
	
	if(input->len < expected_overhead)
	{
		return ERROR_MALFORMED_MESSAGE;
	}
	
	// Calculate payload length
	size_t payload_len = input->len - expected_overhead;
	
	// Validate payload fits in msg structure
	if(payload_len > msg->payload.size)
	{
		return ERROR_MEMORY_OVERRUN;
	}
	
	// Copy payload
	for(size_t i = 0; i < payload_len; i++)
	{
		msg->payload.buf[i] = input->buf[bidx + i];
	}
	msg->payload.len = payload_len;
	
	return DARTT_PROTOCOL_SUCCESS;
}

void test_write_frame_to_struct(void)
{
	unsigned char sermsg_buf[] = {0x12,2,3,4,5,6,0,0};
	dartt_buffer_t buf = {
		.buf = sermsg_buf,
		.size = sizeof(sermsg_buf),
		.len = 6
	};
	append_crc(&buf);
	unsigned char dump[32] = {};
	misc_write_message_t writemessage = {
		.address = 0,
		.index = 0,
		.payload = {
			.buf = dump,
			.size = sizeof(dump),
			.len = 0
		}
	};
	int rc = write_frame_to_struct(&buf, TYPE_SERIAL_MESSAGE, &writemessage);
	TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	TEST_ASSERT_EQUAL(0x12, writemessage.address);
	TEST_ASSERT_EQUAL(3, writemessage.payload.len);
	TEST_ASSERT_EQUAL(4, writemessage.payload.buf[0]);
	TEST_ASSERT_EQUAL(5, writemessage.payload.buf[1]);
	TEST_ASSERT_EQUAL(6, writemessage.payload.buf[2]);
	TEST_ASSERT_EQUAL(0x0302, writemessage.index);
}

/*
Reciprocal test function to dartt_create_read_frame - primary use case is testing dartt_create_read_frame.
 */
int read_frame_to_struct(dartt_buffer_t * input, serial_message_type_t type, misc_read_message_t * msg)
{
	if(input == NULL || msg == NULL || input->buf == NULL)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	if(!(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE || type == TYPE_ADDR_CRC_MESSAGE))
	{
		return ERROR_INVALID_ARGUMENT;
	}
	
	size_t bidx = 0;
	
	// Parse address if present
	if(type == TYPE_SERIAL_MESSAGE)
	{
		if(input->len < NUM_BYTES_ADDRESS)
		{
			return ERROR_MALFORMED_MESSAGE;
		}
		msg->address = input->buf[bidx++];
	}
	else
	{
		msg->address = 0; // Default for non-addressed types
	}
	
	// Calculate expected frame size
	size_t expected_size = bidx + NUM_BYTES_INDEX + NUM_BYTES_NUMWORDS_READREQUEST;
	if(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE)
	{
		expected_size += NUM_BYTES_CHECKSUM;
	}
	
	if(input->len != expected_size)
	{
		return ERROR_MALFORMED_MESSAGE;
	}
	
	// Parse index
	uint16_t rw_index = 0;
	rw_index |= (uint16_t)(input->buf[bidx++]);
	rw_index |= (((uint16_t)(input->buf[bidx++])) << 8);
	
	// Check if this is actually a read frame (MSB should be 1)
	if(!(rw_index & READ_WRITE_BITMASK))
	{
		return ERROR_MALFORMED_MESSAGE; // This is a write frame, not read
	}
	msg->index = rw_index & (~READ_WRITE_BITMASK); // Remove read bit
	
	// Parse num_bytes
	uint16_t num_bytes = 0;
	num_bytes |= (uint16_t)(input->buf[bidx++]);
	num_bytes |= (((uint16_t)(input->buf[bidx++])) << 8);
	msg->num_bytes = num_bytes;
	
	return DARTT_PROTOCOL_SUCCESS;
}


void test_append_crc(void)
{
	{	//happy path 1
		unsigned char mem[] = {1,2,3,4,0,0};
		dartt_buffer_t buf = 
		{
			.buf = mem,
			.size = sizeof(mem),
			.len = 4
		};
		int rc = append_crc(&buf);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
		TEST_ASSERT_EQUAL(6, buf.len);	
		rc = validate_crc(&buf);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	}
	{	//sad path 1
		unsigned char mem[] = {1,2,3,4, 0, 0};
		dartt_buffer_t buf = 
		{
			.buf = mem,
			.size = sizeof(mem),
			.len = sizeof(mem)
		};
		int rc = append_crc(&buf);
		TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);
	}
}

void test_validate_crc(void)
{
	{	//happy path 1
		unsigned char mem[] = {1,2,3,4,0,0};
		dartt_buffer_t buf = 
		{
			.buf = mem,
			.size = sizeof(mem),
			.len = 4
		};
		int rc = append_crc(&buf);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
		TEST_ASSERT_EQUAL(6, buf.len);
		rc = validate_crc(&buf);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	}
	{	//sad path 1
		unsigned char mem[] = {1,2,3,4,0,0};
		dartt_buffer_t buf = 
		{
			.buf = mem,
			.size = sizeof(mem),
			.len = 4
		};
		int rc = validate_crc(&buf);
		TEST_ASSERT_EQUAL(ERROR_CHECKSUM_MISMATCH, rc);
	}
}

void test_dartt_create_write_frame(void)
{
	{	//happy path
		comms_t block_mem = {};
		misc_write_message_t msg = {
			.address = 0x34,
			.index = 3,
			.payload = {
				.buf = (unsigned char *)(&block_mem),
				.size = sizeof(comms_t),
				.len = 0
			}
		};	//create a message

		//fill the payload with nonzero garbage
		for(int i = 0; i < msg.payload.size; i++)
		{
			msg.payload.buf[i] = ((unsigned char)(i % 255))+ 1;
		}

		//point the payload to a random small section of memory
		size_t offset = 3;
		TEST_ASSERT_GREATER_THAN(offset, msg.payload.size);
		msg.payload.buf += offset;
		msg.payload.size -= offset;
		TEST_ASSERT_GREATER_THAN(offset, msg.payload.size);
		msg.payload.len = msg.payload.size - offset;
		TEST_ASSERT_GREATER_THAN(1, msg.payload.len);

		unsigned char msg_buf[256] = {};	//more than enough memory
		dartt_buffer_t output = {
			.buf = msg_buf,
			.size = sizeof(msg),
			.len = 0
		};
		int rc = dartt_create_write_frame(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
		TEST_ASSERT_EQUAL(output.len, (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + msg.payload.len + NUM_BYTES_CHECKSUM) );
		TEST_ASSERT_EQUAL(output.buf[0], msg.address);
		rc = validate_crc(&output);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
		TEST_ASSERT_EQUAL((unsigned char)(msg.index & 0x00FF), output.buf[1]);
		TEST_ASSERT_EQUAL( (unsigned char)((msg.index & 0x0FF00)>>8), output.buf[2]);
		
		unsigned char chkbuf[32] = {};
		misc_write_message_t chk_w_msg = {
			.address=  0,
			.index = 0,
			.payload = {
				.buf = chkbuf,
				.size = sizeof(chkbuf),
				.len = 0
			}
		};
		rc = write_frame_to_struct(&output, TYPE_SERIAL_MESSAGE, &chk_w_msg);
		TEST_ASSERT_EQUAL(msg.address, chk_w_msg.address);		
		TEST_ASSERT_EQUAL(msg.index, chk_w_msg.index);
		TEST_ASSERT_EQUAL(msg.payload.len, chk_w_msg.payload.len);
		for(int i = 0; i < msg.payload.len; i++)
		{
			TEST_ASSERT_EQUAL(msg.payload.buf[i], chk_w_msg.payload.buf[i]);
		}




	
	}
	{	//test - payload length = 1
		unsigned char pld_buf[1] = {};//one byte write frame
		pld_buf[0] = 0xFE;
		misc_write_message_t msg = {
			.address = 0x34,
			.index = 3,
			.payload = {
				.buf = pld_buf,
				.size = sizeof(pld_buf),
				.len = 1
			}
		};	//create a message
		
	}
}


void test_dartt_create_read_frame(void)
{

}

// Helper function to create a test message and generate frame
void create_test_message_and_frame(serial_message_type_t type, misc_write_message_t * msg, dartt_buffer_t * frame, unsigned char * payload_buf)
{
	// Initialize test payload
	unsigned char test_payload[] = {0x12, 0x34, 0x56, 0x78, 0xAB, 0xCD};
	for(int i = 0; i < sizeof(test_payload); i++) {
		payload_buf[i] = test_payload[i];
	}
	
	// Setup message
	msg->address = (type == TYPE_SERIAL_MESSAGE) ? 0x42 : 0;
	msg->index = 0x1234; // Test index
	msg->payload.buf = payload_buf;
	msg->payload.size = sizeof(test_payload);
	msg->payload.len = sizeof(test_payload);
	
	// Generate frame using existing function
	frame->len = 0;
	int rc = dartt_create_write_frame(msg, type, frame);
	TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
}

// Helper function to setup payload layer message for copy vs alias
void setup_payload_msg(payload_mode_t mode, payload_layer_msg_t * pld, unsigned char * copy_buffer, size_t copy_size)
{
	pld->address = 0;
	if(mode == PAYLOAD_ALIAS) {
		pld->msg.buf = NULL;
		pld->msg.size = 0;
		pld->msg.len = 0;
	} else { // PAYLOAD_COPY
		pld->msg.buf = copy_buffer;
		pld->msg.size = copy_size;
		pld->msg.len = 0;
	}
}


void test_check_buffer(void)
{
	dartt_buffer_t b = {};
	TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, check_buffer(NULL));
	TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, check_buffer(&b));
	unsigned char arr[9] = {};
	b.buf = arr;
	TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, check_buffer(&b));
	b.len = sizeof(arr);
	TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, check_buffer(&b));
	b.size = sizeof(arr);
	TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, check_buffer(&b));
}

void test_f2p_memory_overrun_bug(void)
{
	{	//warm up - this test case failed in the last master commit
		dartt_buffer_t serial_msg;
		char buf[7] = {};
		serial_msg.buf = buf;
		serial_msg.size = 5;
		serial_msg.len = sizeof(buf);
		payload_layer_msg_t pld = {};
		int rc = dartt_frame_to_payload(&serial_msg, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &pld);
		TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);
	}
	{	//
		dartt_buffer_t serial_msg;
		char buf[7] = {};
		serial_msg.buf = buf;
		serial_msg.size = sizeof(buf);
		serial_msg.len = sizeof(buf)-2;
		append_crc(&serial_msg);
		payload_layer_msg_t pld = {};
		int rc = dartt_frame_to_payload(&serial_msg, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &pld);
		TEST_ASSERT_EQUAL(sizeof(buf)-(NUM_BYTES_ADDRESS+NUM_BYTES_CHECKSUM), pld.msg.len);
		TEST_ASSERT_EQUAL(serial_msg.len - (NUM_BYTES_ADDRESS+NUM_BYTES_CHECKSUM), pld.msg.len);
		TEST_ASSERT_EQUAL(serial_msg.size - (NUM_BYTES_ADDRESS+NUM_BYTES_CHECKSUM), pld.msg.size);
	}

		{	//
		dartt_buffer_t serial_msg;
		char buf[15] = {};
		serial_msg.buf = buf;
		serial_msg.size = sizeof(buf);
		serial_msg.len = 7;
		append_crc(&serial_msg);
		payload_layer_msg_t pld = {};
		int rc = dartt_frame_to_payload(&serial_msg, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &pld);
		TEST_ASSERT_EQUAL(serial_msg.len - (NUM_BYTES_ADDRESS+NUM_BYTES_CHECKSUM), pld.msg.len);
		TEST_ASSERT_EQUAL(serial_msg.size - (NUM_BYTES_ADDRESS+NUM_BYTES_CHECKSUM), pld.msg.size);
	}

}

// Test happy path scenarios
void f2p_happy_path_helper(serial_message_type_t type, payload_mode_t mode)
{
	unsigned char payload_buf[16] = {};
	unsigned char frame_buf[32] = {};
	misc_write_message_t msg = {};
	dartt_buffer_t frame = {.buf = frame_buf, .size = sizeof(frame_buf), .len = 0};
	
	create_test_message_and_frame(type, &msg, &frame, payload_buf);
	
	unsigned char copy_buf[32] = {};
	payload_layer_msg_t pld = {};
	setup_payload_msg(mode, &pld, copy_buf, sizeof(copy_buf));
	
	int rc = dartt_frame_to_payload(&frame, type, mode, &pld);
	TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	
	// Verify address is extracted correctly for TYPE_SERIAL_MESSAGE
	if(type == TYPE_SERIAL_MESSAGE) {
		TEST_ASSERT_EQUAL(msg.address, pld.address);
	}
	
	// Verify payload length is correct
	size_t expected_payload_len = 0;
	if(type == TYPE_SERIAL_MESSAGE) {
		expected_payload_len = frame.len - NUM_BYTES_ADDRESS - NUM_BYTES_CHECKSUM;
	} else if(type == TYPE_ADDR_MESSAGE) {
		expected_payload_len = frame.len - NUM_BYTES_CHECKSUM;
	} else { // TYPE_ADDR_CRC_MESSAGE
		expected_payload_len = frame.len;
	}
	
	TEST_ASSERT_EQUAL(expected_payload_len, pld.msg.len);
	
	// Verify payload content
	if(mode == PAYLOAD_ALIAS) {
		// For alias mode, verify pointer points into original frame
		if(type == TYPE_SERIAL_MESSAGE) {
			TEST_ASSERT_EQUAL_PTR(frame.buf + NUM_BYTES_ADDRESS, pld.msg.buf);
		} else if(type == TYPE_ADDR_MESSAGE) {
			TEST_ASSERT_EQUAL_PTR(frame.buf, pld.msg.buf);
		} else { // TYPE_ADDR_CRC_MESSAGE
			TEST_ASSERT_EQUAL_PTR(frame.buf, pld.msg.buf);
		}
	} else { // PAYLOAD_COPY
		// For copy mode, verify content was copied correctly
		unsigned char * expected_start = frame.buf;
		if(type == TYPE_SERIAL_MESSAGE) {
			expected_start += NUM_BYTES_ADDRESS;
		}
		
		for(int i = 0; i < pld.msg.len; i++) {
			TEST_ASSERT_EQUAL(expected_start[i], pld.msg.buf[i]);
		}
	}
}

// Test checksum mismatch scenarios
void f2p_checksum_mismatch_helper(serial_message_type_t type, payload_mode_t mode)
{
	unsigned char payload_buf[16] = {};
	unsigned char frame_buf[32] = {};
	misc_write_message_t msg = {};
	dartt_buffer_t frame = {.buf = frame_buf, .size = sizeof(frame_buf), .len = 0};
	
	create_test_message_and_frame(type, &msg, &frame, payload_buf);
	
	// Corrupt the checksum
	frame.buf[frame.len - 1] ^= 0xFF;
	
	unsigned char copy_buf[32] = {};
	payload_layer_msg_t pld = {};
	setup_payload_msg(mode, &pld, copy_buf, sizeof(copy_buf));
	
	int rc = dartt_frame_to_payload(&frame, type, mode, &pld);
	TEST_ASSERT_EQUAL(ERROR_CHECKSUM_MISMATCH, rc);
}

// Test memory overrun scenarios (PAYLOAD_COPY only)
void f2p_memory_overrun_helper(serial_message_type_t type, payload_mode_t mode)
{
	unsigned char payload_buf[16] = {};
	unsigned char frame_buf[32] = {};
	misc_write_message_t msg = {};
	dartt_buffer_t frame = {.buf = frame_buf, .size = sizeof(frame_buf), .len = 0};
	
	create_test_message_and_frame(type, &msg, &frame, payload_buf);
	
	// Create a payload buffer that's too small
	unsigned char copy_buf[2] = {}; // Very small buffer
	payload_layer_msg_t pld = {};
	setup_payload_msg(mode, &pld, copy_buf, sizeof(copy_buf));
	
	int rc = dartt_frame_to_payload(&frame, type, mode, &pld);
	TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);
}

// Test malformed input scenarios
void f2p_malformed_input_helper(serial_message_type_t type, payload_mode_t mode)
{
	unsigned char frame_buf[32] = {};
	dartt_buffer_t frame = {.buf = frame_buf, .size = sizeof(frame_buf), .len = 0};
	
	// Create a frame that's too short
	size_t min_len = 0;
	if(type == TYPE_SERIAL_MESSAGE) {
		min_len = NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM;
	} else if(type == TYPE_ADDR_MESSAGE) {
		min_len = NUM_BYTES_CHECKSUM;
	} else { // TYPE_ADDR_CRC_MESSAGE
		min_len = 1; // Needs at least some content
	}
	
	frame.len = min_len; // Exactly at the minimum (should be malformed)
	
	unsigned char copy_buf[32] = {};
	payload_layer_msg_t pld = {};
	setup_payload_msg(mode, &pld, copy_buf, sizeof(copy_buf));
	
	int rc = dartt_frame_to_payload(&frame, type, mode, &pld);
	if(type == TYPE_ADDR_CRC_MESSAGE) {
		// TYPE_ADDR_CRC_MESSAGE with len=1 should actually work
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	} else {
		TEST_ASSERT_EQUAL(ERROR_MALFORMED_MESSAGE, rc);
	}
}

// Test invalid argument scenarios
void f2p_invalid_args_helper(serial_message_type_t type, payload_mode_t mode)
{
	unsigned char payload_buf[16] = {};
	unsigned char frame_buf[32] = {};
	misc_write_message_t msg = {};
	dartt_buffer_t frame = {.buf = frame_buf, .size = sizeof(frame_buf), .len = 0};
	
	create_test_message_and_frame(type, &msg, &frame, payload_buf);
	
	unsigned char copy_buf[32] = {};
	payload_layer_msg_t pld = {};
	
	// Test PAYLOAD_COPY with NULL buffer
	if(mode == PAYLOAD_COPY) {
		setup_payload_msg(mode, &pld, NULL, 0); // NULL buffer
		int rc = dartt_frame_to_payload(&frame, type, mode, &pld);
		TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);
	}
}

void test_dartt_frame_to_payload_comprehensive(void)
{
	serial_message_type_t types[] = {TYPE_SERIAL_MESSAGE, TYPE_ADDR_MESSAGE, TYPE_ADDR_CRC_MESSAGE};
	payload_mode_t modes[] = {PAYLOAD_ALIAS, PAYLOAD_COPY};
	
	for(int type_idx = 0; type_idx < sizeof(types)/sizeof(serial_message_type_t); type_idx++) 
	{
		for(int mode_idx = 0; mode_idx < sizeof(modes)/sizeof(payload_mode_t); mode_idx++) 
		{
			serial_message_type_t type = types[type_idx];
			payload_mode_t mode = modes[mode_idx];
			
			// Test happy path
			f2p_happy_path_helper(type, mode);
			
			// Test checksum mismatch (only for types with CRC)
			if(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE) 
			{
				f2p_checksum_mismatch_helper(type, mode);
			}
			
			// Test memory overrun (only for PAYLOAD_COPY)
			if(mode == PAYLOAD_COPY) 
			{
				f2p_memory_overrun_helper(type, mode);
			}
			
			// Test malformed messages
			f2p_malformed_input_helper(type, mode);
			
			// Test invalid arguments
			f2p_invalid_args_helper(type, mode);
		}
	}
}

/*
Test that dartt_parse_base_serial_message enforces NUM_BYTES_READ_REPLY_OVERHEAD_PLD in the reply buffer sizing.
The reply for a read contains [index_lo][index_hi][data...], so reply_base->size must be >= num_bytes + NUM_BYTES_READ_REPLY_OVERHEAD_PLD.
*/
void test_parse_base_read_reply_overhead(void)
{
	//8 words of backing memory, filled with a pattern
	uint32_t mem_words[8] = {0xAABBCCDD, 0x11223344, 0x55667788, 0x99AABBCC,
							 0xDDEEFF00, 0x12345678, 0x9ABCDEF0, 0xFEDCBA98};
	dartt_mem_t mem_base = {
		.buf = (unsigned char *)mem_words,
		.size = sizeof(mem_words)
	};

	//construct a read request payload: [index_lo | 0x80][index_hi][num_bytes_lo][num_bytes_hi]
	//read 4 bytes starting at word index 0
	uint16_t num_bytes_requested = 4;
	unsigned char read_pld[] = {
		0x00 | 0x00, 0x80,	//index=0 with read bit set (big end has the MSB)
		(unsigned char)(num_bytes_requested & 0xFF), (unsigned char)((num_bytes_requested >> 8) & 0xFF)
	};
	payload_layer_msg_t pld_msg = {
		.address = 0x42,
		.msg = {
			.buf = read_pld,
			.size = sizeof(read_pld),
			.len = sizeof(read_pld)
		}
	};

	// SAD PATH: reply buffer sized to hold only the data (no overhead) - must fail
	{
		unsigned char reply_mem[4] = {};	//exactly num_bytes_requested, missing NUM_BYTES_READ_REPLY_OVERHEAD_PLD
		dartt_buffer_t reply = {
			.buf = reply_mem,
			.size = sizeof(reply_mem),
			.len = 0
		};
		int rc = dartt_parse_base_serial_message(&pld_msg, &mem_base, &reply);
		TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);
	}

	// SAD PATH: reply buffer one byte short of full overhead
	{
		unsigned char reply_mem[4 + NUM_BYTES_READ_REPLY_OVERHEAD_PLD - 1] = {};
		dartt_buffer_t reply = {
			.buf = reply_mem,
			.size = sizeof(reply_mem),
			.len = 0
		};
		int rc = dartt_parse_base_serial_message(&pld_msg, &mem_base, &reply);
		TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);
	}

	// HAPPY PATH: reply buffer exactly num_bytes + NUM_BYTES_READ_REPLY_OVERHEAD_PLD
	{
		unsigned char reply_mem[4 + NUM_BYTES_READ_REPLY_OVERHEAD_PLD] = {};
		dartt_buffer_t reply = {
			.buf = reply_mem,
			.size = sizeof(reply_mem),
			.len = 0
		};
		int rc = dartt_parse_base_serial_message(&pld_msg, &mem_base, &reply);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
		TEST_ASSERT_EQUAL(num_bytes_requested + NUM_BYTES_READ_REPLY_OVERHEAD_PLD, reply.len);
		//verify the index is prepended in the reply
		uint16_t reply_index = (uint16_t)reply.buf[0] | ((uint16_t)reply.buf[1] << 8);
		TEST_ASSERT_EQUAL(0, reply_index);
		//verify the data bytes match the memory
		for(int i = 0; i < num_bytes_requested; i++)
		{
			TEST_ASSERT_EQUAL(mem_base.buf[i], reply.buf[NUM_BYTES_READ_REPLY_OVERHEAD_PLD + i]);
		}
	}
}

/*
Test that dartt_parse_general_message produces read reply frames whose total length
accounts for NUM_BYTES_READ_REPLY_OVERHEAD_PLD plus framing, across all message types.
*/
void test_parse_general_message_reply_overhead(void)
{
	uint32_t mem_words[4] = {0x12345678, 0x9ABCDEF0, 0xAABBCCDD, 0x11223344};
	dartt_mem_t mem_base = {
		.buf = (unsigned char *)mem_words,
		.size = sizeof(mem_words)
	};

	uint16_t num_bytes_requested = 8;

	//for each message type, verify the reply frame length includes the read reply overhead
	serial_message_type_t types[] = {TYPE_SERIAL_MESSAGE, TYPE_ADDR_MESSAGE, TYPE_ADDR_CRC_MESSAGE};
	for(int t = 0; t < 3; t++)
	{
		serial_message_type_t type = types[t];

		//construct the read request payload (stripped of framing - this is what parse_general_message sees)
		unsigned char read_pld[] = {
			0x00, 0x80,	//index=0 with read bit set
			(unsigned char)(num_bytes_requested & 0xFF), (unsigned char)((num_bytes_requested >> 8) & 0xFF)
		};
		payload_layer_msg_t pld_msg = {
			.address = 0x42,
			.msg = {
				.buf = read_pld,
				.size = sizeof(read_pld),
				.len = sizeof(read_pld)
			}
		};

		//allocate generous reply buffer
		unsigned char reply_mem[64] = {};
		dartt_buffer_t reply = {
			.buf = reply_mem,
			.size = sizeof(reply_mem),
			.len = 0
		};

		int rc = dartt_parse_general_message(&pld_msg, type, &mem_base, &reply);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);

		//calculate expected reply frame length
		size_t expected_len = NUM_BYTES_READ_REPLY_OVERHEAD_PLD + num_bytes_requested;
		if(type == TYPE_SERIAL_MESSAGE)
		{
			expected_len += NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM;
		}
		else if(type == TYPE_ADDR_MESSAGE)
		{
			expected_len += NUM_BYTES_CHECKSUM;
		}
		TEST_ASSERT_EQUAL(expected_len, reply.len);
	}
}

void test_dartt_frame_to_payload(void)
{
	// Run the comprehensive test suite
	test_dartt_frame_to_payload_comprehensive();
	
	// Keep your original focused test as well
	comms_t block_mem = {};
	misc_write_message_t msg = {
		.address = 0x34,
		.index = 3,
		.payload = {
			.buf = (unsigned char *)(&block_mem),
			.size = sizeof(comms_t),
			.len = 0
		}
	};
	
	// Fill with test data
	for(int i = 0; i < 8; i++) {
		msg.payload.buf[i] = ((unsigned char)(i % 255)) + 1;
	}
	msg.payload.len = 8;
	
	unsigned char msg_buf[256] = {};
	dartt_buffer_t output = {
		.buf = msg_buf,
		.size = sizeof(msg_buf),
		.len = 0
	};
	int rc = dartt_create_write_frame(&msg, TYPE_SERIAL_MESSAGE, &output);
	TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	
	// Test alias mode
	payload_layer_msg_t pld = {
		.address = 0,
		.msg = {
			.buf = NULL,
			.size = 0,
			.len = 0
		}
	};
	
	rc = dartt_frame_to_payload(&output, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &pld);	
	TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
	TEST_ASSERT_EQUAL(output.buf[0], pld.address);
	TEST_ASSERT_EQUAL(output.len, pld.msg.len + NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM);
}
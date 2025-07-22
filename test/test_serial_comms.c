#include "checksum.h"
#include "serial-comms.h"
#include "unity.h"
#include "serial-comms-struct.h"

/*
	TODO:
		Create reciprocal create functions (for testing only) - validate create functions
		Create edge case conditions for parse_base. Test a read message which is one byte too small, etc.
		Add test of frame_to_payload of a type 0 serial message consisting of only address and crc
		Add test of frame_to_payload for a motor command and a serial command

*/

/*
	Test helper function index_of_field.
*/
void test_index_of_field(void)
{
	comms_t comms = {};
	TEST_ASSERT_EQUAL(2, index_of_field(&comms.gl_joint_theta, (void*)&comms, sizeof(comms_t)));
	TEST_ASSERT_EQUAL(1, index_of_field(&comms.gl_iq, (void*)&comms, sizeof(comms_t)));
	TEST_ASSERT_EQUAL(0, index_of_field(&comms.motor_command_mode, (void*)&comms, sizeof(comms_t)));
}

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
		buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};

		rc = check_write_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);
	}
	
	// Test NULL argument validation
	{
		misc_write_message_t msg = {};
		buffer_t output = {};
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
		buffer_t output = {
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
		buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_args(&msg, TYPE_SERIAL_MESSAGE, &output);
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
		buffer_t output = {
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
		buffer_t output = {
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
		buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
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
		buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_args(&msg, TYPE_ADDR_MESSAGE, &output);
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
		buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_args(&msg, TYPE_ADDR_MESSAGE, &output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
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
		buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_args(&msg, TYPE_ADDR_CRC_MESSAGE, &output);
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
		buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_args(&msg, TYPE_ADDR_CRC_MESSAGE, &output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
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
		buffer_t output = {
			.buf = output_buf,
			.size = sizeof(output_buf),
			.len = 0
		};
		
		rc = check_write_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	}
}

void test_check_read_args(void)
{
	int rc;
	
	// Test NULL argument validation - msg == NULL
	{
		buffer_t output = {
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
		buffer_t output = {
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
		buffer_t output = {
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
		buffer_t output = {
			.buf = (unsigned char[10]){},
			.size = 10,  // 1 + 2 + 2 + 2 = 7 bytes needed, 10 available
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	}
	
	// Test memory overrun for TYPE_ADDR_MESSAGE (index + num_bytes + checksum)
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 4
		};
		buffer_t output = {
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
		buffer_t output = {
			.buf = (unsigned char[8]){},
			.size = 8,  // 2 + 2 + 2 = 6 bytes needed, 8 available
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_ADDR_MESSAGE, &output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	}
	
	// Test memory overrun for TYPE_ADDR_CRC_MESSAGE (index + num_bytes only)
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 4
		};
		buffer_t output = {
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
		buffer_t output = {
			.buf = (unsigned char[6]){},
			.size = 6,  // 2 + 2 = 4 bytes needed, 6 available
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_ADDR_CRC_MESSAGE, &output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	}
	
	// Test invalid message type
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 4
		};
		buffer_t output = {
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
		buffer_t output = {
			.buf = (unsigned char[7]){},
			.size = 7,  // Exactly 1 + 2 + 2 + 2 = 7 bytes needed
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	}
	
	// Test edge case: minimum output buffer for TYPE_ADDR_MESSAGE
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 1
		};
		buffer_t output = {
			.buf = (unsigned char[6]){},
			.size = 6,  // Exactly 2 + 2 + 2 = 6 bytes needed
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_ADDR_MESSAGE, &output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	}
	
	// Test edge case: minimum output buffer for TYPE_ADDR_CRC_MESSAGE
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 1
		};
		buffer_t output = {
			.buf = (unsigned char[4]){},
			.size = 4,  // Exactly 2 + 2 = 4 bytes needed
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_ADDR_CRC_MESSAGE, &output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	}
	
	// Test large read request
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 1000
		};
		buffer_t output = {
			.buf = (unsigned char[1010]){},
			.size = 1010,  // 1000 + 1 + 2 + 2 + 2 = 1007 bytes needed, 1010 available
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	}
	
	// Test zero num_bytes (edge case - should still work)
	{
		misc_read_message_t msg = {
			.address = 0x12,
			.index = 0,
			.num_bytes = 0
		};
		buffer_t output = {
			.buf = (unsigned char[10]){},
			.size = 10,
			.len = 0
		};
		
		rc = check_read_args(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	}
}

/*Test-scope only - helper functions to parse frames back into structs 
Reciprocal test function to create_write_frame - primary use case is testing create_write_frame.
*/
int write_frame_to_struct(buffer_t * input, serial_message_type_t type, misc_write_message_t * msg)
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
	
	return SERIAL_PROTOCOL_SUCCESS;
}

void test_write_frame_to_struct(void)
{
	unsigned char sermsg_buf[] = {0x12,2,3,4,5,6,0,0};
	buffer_t buf = {
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
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	TEST_ASSERT_EQUAL(0x12, writemessage.address);
	TEST_ASSERT_EQUAL(3, writemessage.payload.len);
	TEST_ASSERT_EQUAL(4, writemessage.payload.buf[0]);
	TEST_ASSERT_EQUAL(5, writemessage.payload.buf[1]);
	TEST_ASSERT_EQUAL(6, writemessage.payload.buf[2]);
	TEST_ASSERT_EQUAL(0x0302, writemessage.index);
}

/*
Reciprocal test function to create_read_frame - primary use case is testing create_read_frame.
 */
int read_frame_to_struct(buffer_t * input, serial_message_type_t type, misc_read_message_t * msg)
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
	
	return SERIAL_PROTOCOL_SUCCESS;
}


void test_append_crc(void)
{
	{	//happy path 1
		unsigned char mem[] = {1,2,3,4,0,0};
		buffer_t buf = 
		{
			.buf = mem,
			.size = sizeof(mem),
			.len = 4
		};
		int rc = append_crc(&buf);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
		TEST_ASSERT_EQUAL(6, buf.len);	
		rc = validate_crc(&buf);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	}
	{	//sad path 1
		unsigned char mem[] = {1,2,3,4, 0, 0};
		buffer_t buf = 
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
		buffer_t buf = 
		{
			.buf = mem,
			.size = sizeof(mem),
			.len = 4
		};
		int rc = append_crc(&buf);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
		TEST_ASSERT_EQUAL(6, buf.len);
		rc = validate_crc(&buf);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	}
	{	//sad path 1
		unsigned char mem[] = {1,2,3,4,0,0};
		buffer_t buf = 
		{
			.buf = mem,
			.size = sizeof(mem),
			.len = 4
		};
		int rc = validate_crc(&buf);
		TEST_ASSERT_EQUAL(ERROR_CHECKSUM_MISMATCH, rc);
	}
}

void test_create_write_frame(void)
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
		buffer_t output = {
			.buf = msg_buf,
			.size = sizeof(msg),
			.len = 0
		};
		int rc = create_write_frame(&msg, TYPE_SERIAL_MESSAGE, &output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
		TEST_ASSERT_EQUAL(output.len, (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + msg.payload.len + NUM_BYTES_CHECKSUM) );
		TEST_ASSERT_EQUAL(output.buf[0], msg.address);
		rc = validate_crc(&output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
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


void test_create_read_frame(void)
{

}

void test_frame_to_payload(void)
{
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
	buffer_t output = {
		.buf = msg_buf,
		.size = sizeof(msg),
		.len = 0
	};
	int rc = create_write_frame(&msg, TYPE_SERIAL_MESSAGE, &output);

	
	
	/*Output transmitted to slave for decoding...*/
	
	
	
	
	payload_layer_msg_t pld = {
		.address = 0,
		.msg = {	//initialize to empty. Consider helper function to load so it doesn't take up so many lines
			.buf = NULL,	//f2p initializes empty buffers to point to the payload section of a frame layer packet - same memory. If allocated, it does a O(n) copy
			.size = 0,
			.len = 0
		}
	};
	rc = frame_to_payload(&output, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &pld);	
	TEST_ASSERT_EQUAL(rc, SERIAL_PROTOCOL_SUCCESS);
	TEST_ASSERT_EQUAL(output.buf[0], pld.address);
	TEST_ASSERT_EQUAL(output.len, pld.msg.len + NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM);
	for(int i = 1; i < output.len - 2; i++)
	{
		TEST_ASSERT_EQUAL(output.buf[i], pld.msg.buf[i-1]);
	}
}
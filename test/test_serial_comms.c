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

// /*Test-scope only - helper */
// int write_frame_to_struct(buffer_t * input, misc_write_message_t * msg)
// {
// 	return SERIAL_PROTOCOL_SUCCESS;
// }
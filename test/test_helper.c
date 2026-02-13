#include "checksum.h"
#include "dartt.h"
#include "unity.h"

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

/*
	Test helper function index_of_field - basic happy path tests.
*/
void test_index_of_field(void)
{
	comms_t comms = {};
	TEST_ASSERT_EQUAL(2, index_of_field(&comms.gl_joint_theta, (void*)&comms, sizeof(comms_t)));
	TEST_ASSERT_EQUAL(1, index_of_field(&comms.gl_iq, (void*)&comms, sizeof(comms_t)));
	TEST_ASSERT_EQUAL(0, index_of_field(&comms.motor_command_mode, (void*)&comms, sizeof(comms_t)));
}

/*
	Comprehensive test for index_of_field - covers all error conditions and edge cases.
	This test prevents regression of the off-by-one bug where ptr == base + size was incorrectly accepted.
*/
void test_index_of_field_comprehensive(void)
{
	comms_t comms = {};

	// Test 1: NULL p_field pointer
	TEST_ASSERT_EQUAL(DARTT_ERROR_INVALID_ARGUMENT,
	                  index_of_field(NULL, (void*)&comms, sizeof(comms_t)));

	// Test 2: NULL mem pointer
	TEST_ASSERT_EQUAL(DARTT_ERROR_INVALID_ARGUMENT,
	                  index_of_field(&comms.gl_iq, NULL, sizeof(comms_t)));

	// Test 3: Both NULL
	TEST_ASSERT_EQUAL(DARTT_ERROR_INVALID_ARGUMENT, index_of_field(NULL, NULL, 0));

	// Test 4: Pointer below base (create pointer arithmetic below base)
	unsigned char* below_base = (unsigned char*)&comms - 4;
	TEST_ASSERT_EQUAL(DARTT_ERROR_MEMORY_OVERRUN,
	                  index_of_field(below_base, (void*)&comms, sizeof(comms_t)));

	// Test 5: OFF-BY-ONE BUG - Pointer exactly at base+size (CRITICAL REGRESSION TEST)
	// This test captures the bug fixed where p_field_nonvoid >= (pbase + mem_size) was changed to use >=
	unsigned char* at_end = (unsigned char*)&comms + sizeof(comms_t);
	TEST_ASSERT_EQUAL(DARTT_ERROR_MEMORY_OVERRUN,
	                  index_of_field(at_end, (void*)&comms, sizeof(comms_t)));

	// Test 6: Pointer beyond base+size
	unsigned char* beyond_end = (unsigned char*)&comms + sizeof(comms_t) + 4;
	TEST_ASSERT_EQUAL(DARTT_ERROR_MEMORY_OVERRUN,
	                  index_of_field(beyond_end, (void*)&comms, sizeof(comms_t)));

	// Test 7: Misaligned pointer (not 4-byte aligned) - +1 byte
	unsigned char* misaligned1 = (unsigned char*)&comms + 1;
	TEST_ASSERT_EQUAL(DARTT_ERROR_INVALID_ARGUMENT,
	                  index_of_field(misaligned1, (void*)&comms, sizeof(comms_t)));

	// Test 8: Misaligned pointer - +2 bytes
	unsigned char* misaligned2 = (unsigned char*)&comms + 2;
	TEST_ASSERT_EQUAL(DARTT_ERROR_INVALID_ARGUMENT,
	                  index_of_field(misaligned2, (void*)&comms, sizeof(comms_t)));

	// Test 9: Misaligned pointer - +3 bytes
	unsigned char* misaligned3 = (unsigned char*)&comms + 3;
	TEST_ASSERT_EQUAL(DARTT_ERROR_INVALID_ARGUMENT,
	                  index_of_field(misaligned3, (void*)&comms, sizeof(comms_t)));

	// Test 10: Valid boundary case - last valid field
	// Calculate pointer to last 4-byte aligned field within structure
	uint32_t* last_valid_field = (uint32_t*)((unsigned char*)&comms + sizeof(comms_t) - sizeof(uint32_t));
	int result = index_of_field(last_valid_field, (void*)&comms, sizeof(comms_t));
	TEST_ASSERT_GREATER_OR_EQUAL(0, result); // Should succeed (return field index >= 0)

	// Test 11: Valid first field (redundant with basic test, but good for completeness)
	TEST_ASSERT_EQUAL(0, index_of_field(&comms.motor_command_mode, (void*)&comms, sizeof(comms_t)));

	// Test 12: Valid middle field
	TEST_ASSERT_EQUAL(1, index_of_field(&comms.gl_iq, (void*)&comms, sizeof(comms_t)));
}

void test_dartt_get_complementary_address(void)
{
	// Test motor address to misc address mapping
	TEST_ASSERT_EQUAL(0xFE, dartt_get_complementary_address(0x01));
	TEST_ASSERT_EQUAL(0xEF, dartt_get_complementary_address(0x10));
	TEST_ASSERT_EQUAL(0xBD, dartt_get_complementary_address(0x42));
	TEST_ASSERT_EQUAL(0x81, dartt_get_complementary_address(0x7E));
	
	// Test misc address to motor address mapping (reverse)
	TEST_ASSERT_EQUAL(0x01, dartt_get_complementary_address(0xFE));
	TEST_ASSERT_EQUAL(0x10, dartt_get_complementary_address(0xEF));
	TEST_ASSERT_EQUAL(0x42, dartt_get_complementary_address(0xBD));
	TEST_ASSERT_EQUAL(0x7E, dartt_get_complementary_address(0x81));
	
	// Test master address pairing
	TEST_ASSERT_EQUAL(0x80, dartt_get_complementary_address(0x7F)); // Motor master -> Misc master
	TEST_ASSERT_EQUAL(0x7F, dartt_get_complementary_address(0x80)); // Misc master -> Motor master
	
	// Test edge cases (the wraparound behavior we discussed)
	TEST_ASSERT_EQUAL(0xFF, dartt_get_complementary_address(0x00)); // Edge case: 0xFF - 0x00 = 0xFF
	TEST_ASSERT_EQUAL(0x00, dartt_get_complementary_address(0xFF)); // Edge case: 0xFF - 0xFF = 0x00
	
	// Test symmetry property: f(f(x)) = x
	unsigned char test_addresses[] = {0x00, 0x01, 0x10, 0x42, 0x7E, 0x7F, 0x80, 0x81, 0xBD, 0xEF, 0xFE, 0xFF};
	for(int i = 0; i < sizeof(test_addresses); i++) {
		unsigned char addr = test_addresses[i];
		unsigned char complement = dartt_get_complementary_address(addr);
		unsigned char double_complement = dartt_get_complementary_address(complement);
		TEST_ASSERT_EQUAL(addr, double_complement);
	}
}

void test_copy_buf_full(void)
{
	// Test successful copy with same sizes
	{
		unsigned char in_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
		unsigned char out_data[8] = {};
		dartt_buffer_t in_buf = {
			.buf = in_data,
			.size = sizeof(in_data),
			.len = sizeof(in_data)
		};
		dartt_buffer_t out_buf = {
			.buf = out_data,
			.size = sizeof(out_data),
			.len = 0
		};
		
		int rc = copy_buf_full(&in_buf, &out_buf);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
		
		// Verify all data was copied correctly
		for(int i = 0; i < 8; i++) {
			TEST_ASSERT_EQUAL(in_data[i], out_data[i]);
		}
	}
	
	// Test NULL input buffer
	{
		unsigned char out_data[8] = {};
		dartt_buffer_t out_buf = {
			.buf = out_data,
			.size = sizeof(out_data),
			.len = 0
		};
		
		int rc = copy_buf_full(NULL, &out_buf);
		TEST_ASSERT_EQUAL(DARTT_ERROR_INVALID_ARGUMENT, rc);
	}
	
	// Test NULL output buffer
	{
		unsigned char in_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
		dartt_buffer_t in_buf = {
			.buf = in_data,
			.size = sizeof(in_data),
			.len = sizeof(in_data)
		};
		
		int rc = copy_buf_full(&in_buf, NULL);
		TEST_ASSERT_EQUAL(DARTT_ERROR_INVALID_ARGUMENT, rc);
	}
	
	// Test NULL both buffers
	{
		int rc = copy_buf_full(NULL, NULL);
		TEST_ASSERT_EQUAL(DARTT_ERROR_INVALID_ARGUMENT, rc);
	}
	
	// Test size mismatch - output buffer smaller
	{
		unsigned char in_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
		unsigned char out_data[4] = {};
		dartt_buffer_t in_buf = {
			.buf = in_data,
			.size = sizeof(in_data),
			.len = sizeof(in_data)
		};
		dartt_buffer_t out_buf = {
			.buf = out_data,
			.size = sizeof(out_data),
			.len = 0
		};
		
		int rc = copy_buf_full(&in_buf, &out_buf);
		TEST_ASSERT_EQUAL(DARTT_ERROR_MEMORY_OVERRUN, rc);
	}
	
	// Test size mismatch - output buffer larger
	{
		unsigned char in_data[4] = {0x01, 0x02, 0x03, 0x04};
		unsigned char out_data[8] = {};
		dartt_buffer_t in_buf = {
			.buf = in_data,
			.size = sizeof(in_data),
			.len = sizeof(in_data)
		};
		dartt_buffer_t out_buf = {
			.buf = out_data,
			.size = sizeof(out_data),
			.len = 0
		};
		
		int rc = copy_buf_full(&in_buf, &out_buf);
		TEST_ASSERT_EQUAL(DARTT_ERROR_MEMORY_OVERRUN, rc);
	}
	
	// Test zero-sized buffers
	{
		dartt_buffer_t in_buf = {
			.buf = NULL,
			.size = 0,
			.len = 0
		};
		dartt_buffer_t out_buf = {
			.buf = NULL,
			.size = 0,
			.len = 0
		};
		
		int rc = copy_buf_full(&in_buf, &out_buf);
		TEST_ASSERT_EQUAL(DARTT_ERROR_INVALID_ARGUMENT, rc);
	}
	
	// Test single byte copy
	{
		unsigned char in_data[1] = {0xAB};
		unsigned char out_data[1] = {};
		dartt_buffer_t in_buf = {
			.buf = in_data,
			.size = sizeof(in_data),
			.len = sizeof(in_data)
		};
		dartt_buffer_t out_buf = {
			.buf = out_data,
			.size = sizeof(out_data),
			.len = 0
		};
		
		int rc = copy_buf_full(&in_buf, &out_buf);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
		TEST_ASSERT_EQUAL(0xAB, out_data[0]);
	}
	
	// Test large buffer copy
	{
		unsigned char in_data[256] = {};
		unsigned char out_data[256] = {};
		
		// Fill input with pattern
		for(int i = 0; i < 256; i++) {
			in_data[i] = (unsigned char)(i % 256);
		}
		
		dartt_buffer_t in_buf = {
			.buf = in_data,
			.size = sizeof(in_data),
			.len = sizeof(in_data)
		};
		dartt_buffer_t out_buf = {
			.buf = out_data,
			.size = sizeof(out_data),
			.len = 0
		};
		
		int rc = copy_buf_full(&in_buf, &out_buf);
		TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
		
		// Verify pattern was copied correctly
		for(int i = 0; i < 256; i++) {
			TEST_ASSERT_EQUAL((unsigned char)(i % 256), out_data[i]);
		}
	}
}
#include "checksum.h"
#include "serial-comms.h"
#include "unity.h"

void setUp(void)
{

}

void test_correct_create_write_message(void)
{	
	int32_t value = -15127;
	unsigned char message_buf[32] = {};


	/* Hack/kludge to the  */
	uint16_t index = 2;
	
	
	int size = create_write_message(17, index, (unsigned char *)(&value), sizeof(value), message_buf, sizeof(message_buf));
	for(int i = 0; i < size; i++)
	{
		printf("%02X ", message_buf[i]);
	}
	printf("\r\n");

	TEST_ASSERT_EQUAL(size, 4+1+2+2);
	TEST_ASSERT_EQUAL(17, message_buf[0]);
	TEST_ASSERT_EQUAL(2, message_buf[1]);
	TEST_ASSERT_EQUAL(0, message_buf[2]);

	unsigned char * p_value = (unsigned char *)(&value);
	for(int i = 0; i < sizeof(value); i++)
	{
		TEST_ASSERT_EQUAL(p_value[i], message_buf[i+3]);
	}	
	
	uint16_t checksum = get_crc16(message_buf, size-2);
	uint16_t * p_checksum = (uint16_t *)(&message_buf[size-2]);
	TEST_ASSERT_EQUAL(checksum, *p_checksum);

}

void test_memory_bounds_create_write_message(void)
{
	unsigned char message_buf[32] = {};
	int size = create_write_message(17, 0, NULL, 0, message_buf, sizeof(message_buf));
	TEST_ASSERT_EQUAL(0, size);

	int32_t value = -15127;
	size = create_write_message(17, 0, (unsigned char *)(&value), sizeof(value), NULL, 0);
	TEST_ASSERT_EQUAL(0, size);

	size = create_write_message(17, 0, NULL, 0, NULL, 0);
	TEST_ASSERT_EQUAL(0, size);

	unsigned char payload[] = {0,1,2,3,4,5};
	unsigned char small_message_buf[] = {0,0};
	size = create_write_message(17, 0, payload, sizeof(payload), small_message_buf, sizeof(small_message_buf));
	TEST_ASSERT_EQUAL(0, size);
		
	unsigned char payload_2[] = {0,1,2,3,4,5};
	unsigned char small_message_buf_2[] = {0,0,0,0,0,0};	//address, index, one byte payload, checksum
	size = create_write_message(17, 0, payload_2, sizeof(payload_2), small_message_buf_2, sizeof(small_message_buf_2));
	TEST_ASSERT_EQUAL(0, size);
}

void test_parse_general_message_write(void)
{
	comms_t comms = {};
	unsigned char message_buf[32] = {};
	uint8_t address = 17;
	uint16_t index = 2;// second value in the comms struct
	int32_t value = -15127;
	int size = create_write_message(address, index, (unsigned char *)(&value), sizeof(value), message_buf, sizeof(message_buf));
	

	unsigned char reply_buf[32] = {};
	int reply_len = 0;
	int parse_result = parse_general_message(address, message_buf, size, reply_buf, sizeof(reply_buf), &reply_len, &comms);
	TEST_ASSERT_EQUAL(0, parse_result);
	TEST_ASSERT_EQUAL(value, comms.gl_joint_theta);
}

#include "serial-comms.h"
#include "unity.h"

void setUp(void)
{

}

void test_create_write_message(void)
{
	comms_t comms = {};
	
	int32_t value = -15127;
	unsigned char message_buf[32] = {};
	unsigned char * pgliq = (unsigned char * )(&comms.gl_iq);
	unsigned char * pcommbase = (unsigned char *)(&comms);
	uint16_t index = pgliq - pcommbase;
	printf("index = %d\r\n", index);
	
	
	int size = create_write_message(17, index, (unsigned char *)(&value), sizeof(value), message_buf, sizeof(message_buf));
	TEST_ASSERT_EQUAL(size, 4+1+2+2);
	TEST_ASSERT_EQUAL(message_buf[0], 17);
	TEST_ASSERT_EQUAL(message_buf[1], 4);
	TEST_ASSERT_EQUAL(message_buf[2], 0);

}

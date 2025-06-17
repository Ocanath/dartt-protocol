#include "checksum.h"
#include "serial-comms.h"
#include "serial-comms-FDCAN.h"
#include "unity.h"

void test_create_can_frame(void)
{
	comms_t comms = {};

	unsigned char tx_buf[32] = {};
	buffer_t tx_message = {
			.buf = tx_buf,
			.size = sizeof(tx_buf),
			.len = 0
	};
	int32_t word = 0x1851;
	buffer_t payload = {
			.buf = (unsigned char *)(&word),
			.size = sizeof(word),
			.len = sizeof(word)
	};
	unsigned char address = 0x17;
	uint16_t index = 1;
	int rc = create_misc_write_message(address, index, &payload, &tx_message);
	TEST_ASSERT_GREATER_THAN(0, rc);
	can_frame_t can_frame;
	rc = create_can_frame_from_message(&tx_message, &can_frame);
	TEST_ASSERT_EQUAL(0, rc);
	TEST_ASSERT_EQUAL(6, can_frame.length);
	TEST_ASSERT_EQUAL(address, can_frame.id);


	rc = create_misc_read_message(address, index, 1, &tx_message);
	TEST_ASSERT_GREATER_THAN(0, rc);
	rc = create_can_frame_from_message(&tx_message, &can_frame);
	TEST_ASSERT_EQUAL(0, rc);
	TEST_ASSERT_EQUAL(4, can_frame.length);
	TEST_ASSERT_EQUAL(address, can_frame.id);

	unsigned char check_msg_buf[32] = {};
	buffer_t check_msg = {
			.buf = check_msg_buf,
			.size = sizeof(check_msg_buf),
			.len = 0
	};
	rc = create_message_from_can_frame(&can_frame, &check_msg);
	TEST_ASSERT_EQUAL(tx_message.len, check_msg.len);
	for(int i = 0; i < check_msg.len; i++)
	{
		TEST_ASSERT_EQUAL(tx_message.buf[i], check_msg.buf[i]);
	}
}

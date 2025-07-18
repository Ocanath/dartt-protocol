#include "checksum.h"
#include "serial-comms.h"
#include "unity.h"
#include "serial-comms-struct.h"

void setUp(void)
{

}

void test_correct_create_write_message(void)
{	
	unsigned char message_buf[32] = {};
	buffer_t msg = {
			.buf = message_buf,
			.size = sizeof(message_buf),
			.len = 0
	};

	/* Hack/kludge to the  */
	uint16_t index = 2;

	int32_t value = -15127;
	buffer_t payload = {
			.buf = (unsigned char *)&value,
			.size = sizeof(value),
			.len = sizeof(value)
	};
	
	int size = create_misc_write_message(17, index, &payload, &msg);

	TEST_ASSERT_EQUAL(4+1+2+2, size);
	TEST_ASSERT_EQUAL(17, message_buf[0]);
	TEST_ASSERT_EQUAL(2, message_buf[1]);
	TEST_ASSERT_EQUAL(0, message_buf[2]);

	unsigned char * p_value = (unsigned char *)(&value);
	for(int i = 0; i < sizeof(value); i++)
	{
		TEST_ASSERT_EQUAL(p_value[i], message_buf[i+3]);
	}	
	
	uint16_t checksum = get_crc16(msg.buf, msg.len-2);
	uint16_t * p_checksum = (uint16_t *)(&msg.buf[msg.len - 2]);
	TEST_ASSERT_EQUAL(checksum, *p_checksum);

}

void test_memory_bounds_create_write_message(void)
{
	unsigned char message_buf[32] = {};
	buffer_t msg = {
			.buf = message_buf,
			.size = sizeof(message_buf),
			.len = 0
	};
	int size = create_misc_write_message(17, 0, NULL, &msg);
	TEST_ASSERT_EQUAL(0, size);

	int32_t value = -15127;
	buffer_t payload = {
			.buf = (unsigned char *)(&value),
			.size = sizeof(value),
			.len = sizeof(value)
	};
	size = create_misc_write_message(17, 0, &payload, NULL);
	TEST_ASSERT_EQUAL(0, size);

	size = create_misc_write_message(17, 0, NULL, NULL);
	TEST_ASSERT_EQUAL(0, size);

	unsigned char payload_buf[] = {0,1,2,3,4,5};
	payload.buf = payload_buf;
	payload.size = sizeof(payload_buf);
	payload.len = sizeof(payload_buf);
	unsigned char small_message_buf[] = {0,0};
	msg.buf = small_message_buf;
	msg.size = sizeof(small_message_buf);
	msg.len = 0;
	size = create_misc_write_message(17, 0, &payload, &msg);
	TEST_ASSERT_EQUAL(0, size);

	unsigned char payload_2[] = {0,1,2,3,4,5};
	payload.buf = payload_2;
	payload.size = sizeof(payload_2);
	payload.len = sizeof(payload_2);
	unsigned char small_message_buf_2[] = {0,0,0,0,0,0};	//address, index, one byte payload, checksum
	msg.buf = small_message_buf_2;
	msg.size = sizeof(small_message_buf_2);
	msg.len = 0;
	size = create_misc_write_message(17, 0, &payload, &msg);
	TEST_ASSERT_EQUAL(0, size);
}

void test_parse_general_message_write(void)
{
	comms_t comms = {};
	unsigned char message_buf[32] = {};
	buffer_t message = {
			.buf = message_buf,
			.size = sizeof(message_buf),
			.len = 0
	};
	uint8_t address = 17;
	uint16_t index = 2;// second value in the comms struct
	int32_t value = -15127;
	buffer_t payload = {
			.buf = (unsigned char *)&value,
			.size = sizeof(value),
			.len = sizeof(value)
	};
	int size = create_misc_write_message(address, index, &payload, &message);
	

	unsigned char reply_buf[32] = {};
	buffer_t reply = {
			.buf = reply_buf,
			.size = sizeof(reply_buf),
			.len = 0
	};
	int parse_result = parse_general_message(address, &message, TYPE_UART_MESSAGE, &reply, (void*)&comms, sizeof(comms_t));
	TEST_ASSERT_EQUAL(0, parse_result);
	TEST_ASSERT_EQUAL(value, comms.gl_joint_theta);
}

void test_create_misc_read_message(void)
{
	unsigned char message_buf[32] = {};
	buffer_t message = {
			.buf = message_buf,
			.size = sizeof(message_buf),
			.len = 0
	};
	int size = create_misc_read_message(17, 1, 2, &message);
	TEST_ASSERT_EQUAL(size, 7);
	TEST_ASSERT_EQUAL(17, message_buf[0]);

	unsigned char too_small_message_buf[4] = {};
	buffer_t too_small_message = {
			.buf = too_small_message_buf,
			.size = sizeof(too_small_message_buf),
			.len = 0
	};
	size = create_misc_read_message(17, 1, 2, &too_small_message);
	TEST_ASSERT_EQUAL(0, size);

	size = create_misc_read_message(17, 1, 2, NULL);
	TEST_ASSERT_EQUAL(0, size);

	// size = create_misc_read_message(-1, -10000, -200000, -12, -124);
	// TEST_ASSERT_EQUAL(0, size);
}

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
/*
	Test that the parse_general_message function works for a read message.
*/
void test_parse_general_message_read(void)
{
	comms_t comms = {};
	comms.gl_joint_theta = -75411;
	comms.gl_iq = 151151;
	comms.motor_command_mode = 7;
	comms.gl_rotor_velocity = 123456;
	uint8_t address = 17;

	unsigned char message_buf[32] = {};
	buffer_t message = {
			.buf = message_buf,
			.size = sizeof(message_buf),
			.len = 0
	};
	unsigned char reply_buf[32] = {};
	buffer_t reply = {
			.buf = reply_buf,
			.size = sizeof(reply_buf),
			.len = 0
	};
	int size = create_misc_read_message(address, 2, 1, &message);
	int parse_result = parse_general_message(address, &message, TYPE_UART_MESSAGE, &reply, (void*)&comms, sizeof(comms_t));
	TEST_ASSERT_EQUAL(0, parse_result);
	TEST_ASSERT_EQUAL(7, reply.len);
	TEST_ASSERT_EQUAL(MASTER_MISC_ADDRESS, reply_buf[0]);
	int32_t * p_value = (int32_t *)(&reply_buf[1]);
	TEST_ASSERT_EQUAL(comms.gl_joint_theta, *p_value);
	uint16_t checksum = get_crc16(reply_buf, reply.len-2);
	uint16_t * p_checksum = (uint16_t *)(&reply_buf[reply.len-2]);
	TEST_ASSERT_EQUAL(checksum, *p_checksum);


	// Test reading multiple fields
	size = create_misc_read_message(address, index_of_field(&comms.gl_iq, (void*)&comms, sizeof(comms_t)), 3, &message);
	parse_result = parse_general_message(address, &message, TYPE_UART_MESSAGE, &reply, (void*)&comms, sizeof(comms_t));
	TEST_ASSERT_EQUAL(0, parse_result);
	TEST_ASSERT_EQUAL(15, reply.len);
	TEST_ASSERT_EQUAL(MASTER_MISC_ADDRESS, reply_buf[0]);
	p_value = (int32_t *)(&reply_buf[1]);
	TEST_ASSERT_EQUAL(comms.gl_iq, *p_value);
	p_value = (int32_t *)(&reply_buf[5]);
	TEST_ASSERT_EQUAL(comms.gl_joint_theta, *p_value);
	p_value = (int32_t *)(&reply_buf[9]);
	TEST_ASSERT_EQUAL(comms.gl_rotor_theta, *p_value);
	checksum = get_crc16(reply_buf, reply.len-2);
	p_checksum = (uint16_t *)(&reply_buf[reply.len-2]);
	TEST_ASSERT_EQUAL(checksum, *p_checksum);
	
}


void test_general_message_read_replybuf_overflow(void)
{
	comms_t comms = {};
	comms.gl_joint_theta = -75411;
	comms.gl_iq = 151151;
	comms.motor_command_mode = 7;
	comms.gl_rotor_velocity = 123456;
	uint8_t address = 17;

	unsigned char message_buf[32] = {};
	buffer_t message = {
			.buf = message_buf,
			.size = sizeof(message_buf),
			.len = 0
	};

	unsigned char reply_buf[4] = {};
	buffer_t reply = {
			.buf = reply_buf,
			.size = sizeof(reply_buf),
			.len = 0
	};

		// Test reading multiple fields
	int size = create_misc_read_message(address, index_of_field(&comms.gl_iq, (void*)&comms, sizeof(comms_t)), 3, &message);
	int parse_result = parse_general_message(address, &message, TYPE_UART_MESSAGE, &reply, (void*)&comms, sizeof(comms_t));
	TEST_ASSERT_EQUAL(ERROR_MALFORMED_MESSAGE, parse_result);
	TEST_ASSERT_EQUAL(0, reply.len);
}

/*
	Test that the address filtering works.
*/
void test_address_filtering(void)
{
	comms_t comms = {};
	unsigned char message_buf[32] = {};
	buffer_t message = {
			.buf = message_buf,
			.size = sizeof(message_buf),
			.len = 0
	};

	unsigned char reply_buf[32] = {};
	buffer_t reply = {
			.buf = reply_buf,
			.size = sizeof(reply_buf),
			.len = 0
	};

	uint8_t address = 32;
	int size = create_misc_read_message(address, 2, 1, &message);
	int parse_result = parse_general_message(address+1, &message, TYPE_UART_MESSAGE, &reply, (void*)&comms, sizeof(comms_t));
	TEST_ASSERT_EQUAL(ADDRESS_FILTERED, parse_result);
}


void test_misc_message_to_serial_buf(void)
{
	{	//happy path test
		unsigned char msgbuf[] = {1,2,3,4,5,6};
		misc_message_t msg = {
			.address = 0x45,
			.rw_index = 0x0123,
			.payload = {
				.buf = msgbuf,
				.len = sizeof(msgbuf),
				.size = sizeof(msgbuf)
			}
		};
		unsigned char outpubuf[] = {1,2,3,4,5,6,7,8,9,10,11,12};
		buffer_t output = {
			.buf = outpubuf,
			.len = 0,
			.size = sizeof(outpubuf)
		};
		int rc = misc_message_to_serial_buf(&msg, TYPE_UART_MESSAGE, &output);
		TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
		TEST_ASSERT_EQUAL(0x45, output.buf[0]);
		TEST_ASSERT_EQUAL(sizeof(msgbuf)+NUM_BYTES_NON_PAYLOAD, output.len);
		uint16_t val = 0;
		unsigned char * pval = &val;
		for(int i = 0; i < sizeof(uint16_t); i++)
		{
			pval[i] = output.buf[i+1];
		}
		TEST_ASSERT_EQUAL(0x0123, val);
		for(int i = 0; i < sizeof(msgbuf); i++)
		{
			TEST_ASSERT_EQUAL(msgbuf[i], output.buf[i+3]);
		}
		for(int i = 0; i < sizeof(uint16_t); i++)
		{
			pval[i] = output.buf[i+output.len-2];
		}
		uint16_t checksum = get_crc16(output.buf, output.len-2);
		TEST_ASSERT_EQUAL(checksum, val);
	}
}

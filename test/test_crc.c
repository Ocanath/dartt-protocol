
#include "checksum.h"
#include "dartt.h"
#include "unity.h"



void test_checksum(void)
{
    {
        unsigned char array[] = {1,2,3,4,5,6,7,8,0xff,0xfe,0xfd};
        uint16_t crc = get_crc16(array, sizeof(array));
        TEST_ASSERT_EQUAL(0xC6E5, crc);

        // uint16_t crc_lut = get_crc16_lut(array, sizeof(array));
        // TEST_ASSERT_EQUAL(crc, crc_lut);
    }

    {
        unsigned char array[] = {0x10,0x51,0x05,0x17,0x58,0x92,0x35,0xff};
        uint16_t crc = get_crc16(array, sizeof(array));
        TEST_ASSERT_EQUAL(0x0990, crc);

        // uint16_t crc_lut = get_crc16_lut(array, sizeof(array));
        // TEST_ASSERT_EQUAL(crc, crc_lut);
    }
}


/*
	Basic Confirm CRC-32/ISO-HDLC
	Confirmation testing performed with: https://crccalc.com/
*/
void test_crc32_algo(void)
{
	{
		unsigned char arr[] = {0x45, 0xFF, 0x00, 0x12, 0xAB, 0xFE};
		unsigned int crc32val = get_crc32((const unsigned char *)arr, (unsigned int)(sizeof(arr)));
		TEST_ASSERT_EQUAL(0x2C6730D7, crc32val);
	}
	{
		uint32_t testval = 0;
		uint32_t crc32val = get_crc32( (unsigned char *)(&testval), sizeof(testval));
		TEST_ASSERT_EQUAL(0x2144DF1C, crc32val);
	}
	{
		unsigned char arr[] = {0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x12, 0x3f, 0xff, 0xff};
		uint32_t crc32val = get_crc32((const unsigned char *)arr, (unsigned int)(sizeof(arr)));
		TEST_ASSERT_EQUAL(0xAEA095F9, crc32val);
	}
}


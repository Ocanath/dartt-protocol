
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


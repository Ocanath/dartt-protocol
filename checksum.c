#include <stdint.h>



/*
Generic 2's complement hex checksum calculation.
 */
uint16_t get_checksum16(uint16_t* arr, int size)
{
	int16_t checksum = 0;
	for (int i = 0; i < size; i++)
		checksum += (int16_t)arr[i];
	return -checksum;
}



/*
    Calculate the CRC16 checksum of a message.

    TODO: TEST THIS SHIT! This was a vibe coded function and I have no idea what it's doing.
 */
uint16_t get_crc16(uint8_t* arr, int size)
{
    uint16_t crc = 0xFFFF;
    for(int i = 0; i < size; i++)
    {
        crc ^= arr[i];
        for(int j = 0; j < 8; j++)
        {
            if(crc & 0x0001)
            {
                crc >>= 1;
            }
            else
            {
                crc = (crc >> 1) ^ 0xA001;
            }
        }
    }
    return crc;
}   


#include "serial-comms.h"
#include <stdio.h>

int main(void)
{
	comms_t comms = {};
	unsigned char message_buf[32] = {};
	uint8_t address = 17;
	uint16_t index = 2;// second value in the comms struct
	int32_t value = -15127;
	int size = create_write_message(address, index, (unsigned char *)(&value), sizeof(value), message_buf, sizeof(message_buf));
    
    for(int i = 0; i < size; i++)
    {
        printf("%02X ", message_buf[i]);
    }
    printf("\r\n");

	int parse_result = parse_general_message(address, message_buf, size, &comms);	
    printf("Parse result: %d\r\n", parse_result);

}


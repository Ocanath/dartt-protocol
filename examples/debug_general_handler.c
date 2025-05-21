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

    unsigned char reply_buf[32] = {};
    int reply_len = 0;
	int parsed_result = parse_general_message(address, message_buf, size, reply_buf, sizeof(reply_buf), &reply_len, &comms);	
    printf("Parse result: %d\r\n", parsed_result);


}


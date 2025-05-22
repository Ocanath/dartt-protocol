#include "serial-comms.h"
#include <stdio.h>

int main(void)
{

    {
    	comms_t comms = {};
        unsigned char message_buf[32] = {};
        uint8_t address = 17;
        uint16_t index = 2;// second value in the comms struct
        int32_t value = -15127;
        int size = create_misc_write_message(address, index, (unsigned char *)(&value), sizeof(value), message_buf, sizeof(message_buf));
        
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

    {
        comms_t comms = {};
        comms.gl_joint_theta = -75411;
        comms.gl_iq = 151151;
        comms.motor_command_mode = 7;
        uint8_t address = 17;

        unsigned char message_buf[32] = {};
        unsigned char reply_buf[32] = {};
        int reply_len = 0;
        int size = create_misc_read_message(address, 2, 1, message_buf, sizeof(message_buf));
        int parse_result = parse_general_message(address, message_buf, size, reply_buf, sizeof(reply_buf), &reply_len, &comms);
        if(reply_len == 4 && parse_result == 0)
        {
            int32_t * p_value = (int32_t *)(&reply_buf[0]);
            printf("gl_joint_theta: %d, replybuf_value = %d\r\n", comms.gl_joint_theta, *p_value);            
        }
    }
	return 0;
}


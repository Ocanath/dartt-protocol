#include "serial-comms.h"
#include "checksum.h"
#include "serial-comms-struct.h"

/*
Helper function to create a motor command.
INPUTS: address of motor
Returns:
	number of bytes written to msg
*/
int create_motor_command(unsigned char motor_address, int32_t command_word, buffer_t * msg)
{
	if(msg->size < NUM_BYTES_ADDRESS+sizeof(int32_t)+NUM_BYTES_CHECKSUM)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	int bidx = 0;
	msg->buf[bidx++] = motor_address;
	int32_t * pcmd = (int32_t *)(&msg[bidx]);
	*pcmd = command_word;
	bidx += sizeof(int32_t);
	uint16_t * pchecksum = (uint16_t *)(&msg->buf[bidx]);
	*pchecksum = get_crc16(msg->buf, bidx);
	bidx += sizeof(uint16_t);
	msg->len = bidx;
	return bidx;
}

/*Helper function to get the misc address from the motor address*/
unsigned char get_misc_address(unsigned char motor_address)
{
	return 0xFF - motor_address;
}

/*
 * Write motor struct
 * */
int create_write_struct_mem_message(void * pstart, size_t num_bytes, comms_t * pcomm, buffer_t * msg)
{
	int idx = index_of_field(pstart, pcomm);
	if(idx < 0)
	{
		return idx;
	}
	if( (idx * sizeof(int32_t) + num_bytes) > sizeof(comms_t))
	{
		return ERROR_MALFORMED_MESSAGE;
	}
	unsigned char misc_address = get_misc_address(pcomm->fds.module_number);
	buffer_t payload = {.buf = (unsigned char *)pstart, .size = num_bytes, .len = num_bytes};
	int len = create_misc_write_message(misc_address, idx, &payload, msg);
	return len;
}

/*
 * Helper function to create the read request message for a specific 32bit-word in the structure
 * */
int create_read_struct_mem_message(void * pstart, size_t num_words, comms_t * pcomm, buffer_t * msg)
{
	int idx = index_of_field(pstart, pcomm);
	if(idx < 0)
	{
		return idx;
	}
	if( ((idx + num_words)*sizeof(int32_t)) > sizeof(comms_t))
	{
		return ERROR_MALFORMED_MESSAGE;
	}
	unsigned char misc_address = get_misc_address(pcomm->fds.module_number);
	int len = create_misc_read_message(misc_address, idx, num_words, msg);
	return len;
}

/*
 * Helper function to take a misc read response and load it back up into the master struct
 * */
int update_comms_with_read_reply(void * pword, comms_t * pcomms, buffer_t * msg)
{
	if(msg == NULL || pword == NULL || pcomms == NULL)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	int idx = index_of_field(pword,pcomms);
	if(idx < 0)
	{
		return idx;
	}
	int payload_size = (msg->len - (NUM_BYTES_CHECKSUM + NUM_BYTES_ADDRESS));
	if(idx*sizeof(int32_t)+payload_size > sizeof(comms_t))
	{
		return ERROR_MALFORMED_MESSAGE;
	}

	unsigned char * pstruct = (unsigned char *)pword;
	for(int i = 0; i < payload_size; i++)
	{
		pstruct[i] = msg->buf[i+NUM_BYTES_ADDRESS];
	}
	return SERIAL_PROTOCOL_SUCCESS;
}

/*
 * Parse a reply from a motor command
 * Motor commands are special: they consist of only an address, a context-dependent command word, and a checksum
 * The replies are always addressed to the master, and consist an address, cherry-picked motor values (iq, dtheta, theta_rotor) and a checksum 
 * This function parses the reply into the Master comms struct.
 * 
 * The master should maintain an instance of a comms struct for each motor
 * 
 */
int parse_motor_message_reply(buffer_t * msg, comms_t * comms)
{
    int bidx = 1;
    const int expected_len = NUM_BYTES_ADDRESS + sizeof(int16_t) + sizeof(int16_t) + sizeof(int32_t) + NUM_BYTES_CHECKSUM;
    if(msg->len != expected_len)
    {
        return ERROR_MALFORMED_MESSAGE;
    }
    int checksum_index = (NUM_BYTES_ADDRESS + sizeof(int16_t) + sizeof(int16_t) + sizeof(int32_t));
    if(get_crc16(msg->buf, checksum_index) != *(uint16_t *)(&msg->buf[checksum_index]))
    {
        return ERROR_CHECKSUM_MISMATCH;
    }

    int16_t * pi16;
    int32_t * pi32;
    pi16 = (int16_t *)(&msg->buf[bidx]);
    bidx += sizeof(int16_t);
    comms->foc.gl_iq = (int32_t)(*pi16);

    pi16 = (int16_t *)(&msg->buf[bidx]);
    bidx += sizeof(int16_t);
    comms->foc.gl_dtheta_fixedpoint_rad_p_sec = (int32_t)(*pi16);

    pi32 = (int32_t *)(&msg->buf[bidx]);
    bidx += sizeof(int32_t);
    comms->foc.gl_theta_rem_m = *pi32;
    
    return SERIAL_PROTOCOL_SUCCESS;
}

/*
    Parse an unstuffed (raw) message, including checksum and address splitting.
    Returns:
        -2 if the message is malformed
        -1 if the message is not intended for this device
        0 if the message is successfully parsed

    This function does address filtering and checksum validation
 */
int parse_motor_message(unsigned char motor_address, unsigned char misc_address, buffer_t * msg, buffer_t * reply, comms_t * comms)
{   
    if(msg->len < MINIMUM_MESSAGE_LENGTH)
    {
        return ERROR_MALFORMED_MESSAGE;
    }
    uint16_t * pchecksum = (uint16_t *)(&(msg->buf[msg->len - sizeof(uint16_t)]));
    uint16_t checksum = get_crc16(msg->buf, msg->len - sizeof(uint16_t));
    if(checksum != *pchecksum)
    {
        return ERROR_CHECKSUM_MISMATCH;
    }
    else
    {
        if(msg->buf[0] == motor_address)
        {
			/*
			Byte 0: motor address
			Byte 1-4: command word
			Byte 5-8: checksum
			*/
            if(msg->len < NUM_BYTES_ADDRESS+sizeof(int32_t)+NUM_BYTES_CHECKSUM)
            {
                return ERROR_MALFORMED_MESSAGE;
            }
            int32_t * p_cmd = (int32_t*)(&msg->buf[1]);
            comms->command_word = *p_cmd;

            if(reply->size < NUM_BYTES_ADDRESS + sizeof(int16_t) +sizeof(int16_t) + sizeof(int32_t) + NUM_BYTES_CHECKSUM)
            {
                return ERROR_MALFORMED_MESSAGE;
            }

            int bidx = 0;

			//Consider: instead of always sending to master, we could send to the address of the motor that sent the message. 
			//We would want to make sure that the serial handler on slaves masks reads during self transmission, because
			//RS485 will put them on the bus. But this information could be more useful than addressing the master, as it 
			//helps us confirm that the message was sent by the intended motor.
            reply->buf[bidx++] = MASTER_MOTOR_ADDRESS;

            int16_t val_compressed_i16 = (int16_t)(comms->foc.gl_iq);
            unsigned char * p_val = (unsigned char *)(&val_compressed_i16);
            for(int i = 0; i < sizeof(val_compressed_i16); i++)
            {
                reply->buf[bidx++] = p_val[i];
            }

            val_compressed_i16 = (int16_t)(comms->foc.gl_dtheta_fixedpoint_rad_p_sec);
            p_val = (unsigned char *)(&val_compressed_i16);
            for(int i = 0; i < sizeof(val_compressed_i16); i++)
            {
                reply->buf[bidx++] = p_val[i];
            }

            p_val = (unsigned char *)(&comms->foc.gl_theta_rem_m);
            for(int i = 0; i < sizeof(int32_t); i++)
            {
                reply->buf[bidx++] = p_val[i];
            }

            uint16_t checksum = get_crc16(reply->buf, bidx);
            p_val = (unsigned char *)(&checksum);
            for(int i = 0; i < sizeof(uint16_t); i++)
            {
                reply->buf[bidx++] = p_val[i];
            }
            reply->len = bidx;

            return SERIAL_PROTOCOL_SUCCESS;
        }
        else if(msg->buf[0] == misc_address)
        {
            return parse_misc_command(msg, reply, comms);
        }
        else
        {
            return ADDRESS_FILTERED;
        }
    }
}

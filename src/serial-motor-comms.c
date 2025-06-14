#include "serial-comms.h"
#include "checksum.h"

enum {REPLY_MODE_1 = 0, REPLY_MODE_2 = 1};

/*
Helper function to create a motor command.
INPUTS: address of motor
Returns:
	number of bytes written to msg
*/
int create_motor_command(unsigned char motor_address, int32_t command_word, unsigned char * msg, int msg_size)
{
	if(msg_size < NUM_BYTES_ADDRESS+sizeof(int32_t)+NUM_BYTES_CHECKSUM)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	int bidx = 0;
	msg[bidx++] = motor_address;
	int32_t * pcmd = (int32_t *)(&msg[bidx]);
	*pcmd = command_word;
	bidx += sizeof(int32_t);
	uint16_t * pchecksum = (uint16_t *)(&msg[bidx]);
	*pchecksum = get_crc16(msg, bidx);
	bidx += sizeof(uint16_t);
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
int create_write_struct_mem_message(void * pstart, size_t num_bytes, comms_t * pcomm, unsigned char * msg_buf, size_t msg_size)
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
	int len = create_misc_write_message(misc_address, idx, (unsigned char *)pstart, num_bytes, msg_buf, msg_size);
	return len;
}

/*
 * Helper function to create the read request message for a specific 32bit-word in the structure
 * */
int create_read_struct_mem_message(void * pstart, size_t num_words, comms_t * pcomm, unsigned char * tx_buf, size_t tx_size)
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
	int len = create_misc_read_message(misc_address, idx, num_words, tx_buf, tx_size);
	return len;
}

/*
 * Helper function to take a misc read response and load it back up into the master struct
 * */
int update_comms_with_read_reply(void * pword, comms_t * pcomms, unsigned char * msg_buf, int msg_len)
{
	int idx = index_of_field(pword,pcomms);
	if(idx < 0)
	{
		return idx;
	}
	int payload_size = (msg_len - (NUM_BYTES_CHECKSUM + NUM_BYTES_ADDRESS));
	if(idx*sizeof(int32_t)+payload_size > sizeof(comms_t))
	{
		return ERROR_MALFORMED_MESSAGE;
	}

	unsigned char * pstruct = (unsigned char *)pword;
	for(int i = 0; i < payload_size; i++)
	{
		pstruct[i] = msg_buf[i+NUM_BYTES_ADDRESS];
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
int parse_motor_message_reply(unsigned char * msg, int msg_len, comms_t * comms)
{
	//zero'th byte is address, so we can ignore it
	int bidx = 1;
	const int expected_len = NUM_BYTES_ADDRESS + sizeof(int16_t) + sizeof(int16_t) + sizeof(int32_t) + NUM_BYTES_CHECKSUM;
	if(msg_len != expected_len)
	{
		return ERROR_MALFORMED_MESSAGE;
	}
	int checksum_index = (NUM_BYTES_ADDRESS + sizeof(int16_t) + sizeof(int16_t) + sizeof(int32_t));
	if(get_crc16(msg, checksum_index) != *(uint16_t *)(&msg[checksum_index]))
	{
		return ERROR_CHECKSUM_MISMATCH;
	}

	//parse the first value (two bytes of gl_iq)
	int16_t * pi16;
	int32_t * pi32;
	pi16 = (int16_t *)(&msg[bidx]);
	bidx += sizeof(int16_t);
	comms->gl_iq = (int32_t)(*pi16);

	pi16 = (int16_t *)(&msg[bidx]);
	bidx += sizeof(int16_t);
	comms->gl_rotor_velocity = (int32_t)(*pi16);

	pi32 = (int32_t *)(&msg[bidx]);
	bidx += sizeof(int32_t);
	comms->gl_joint_theta = *pi32;
	
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
int parse_motor_message(unsigned char motor_address, unsigned char misc_address, unsigned char * msg, int len, unsigned char * p_replybuf, int replybuf_size, int * reply_len,  comms_t * comms)
{   
    if(len < MINIMUM_MESSAGE_LENGTH)
    {
        return ERROR_MALFORMED_MESSAGE;
    }
	//POTENTIAL VULN: assumes len is correct, and it may not be.
    uint16_t * pchecksum = (uint16_t *)(&(msg[len - sizeof(uint16_t)]));
    uint16_t checksum = get_crc16(msg, len - sizeof(uint16_t));
    if(checksum != *pchecksum)
    {
        return ERROR_CHECKSUM_MISMATCH;
    }
    else
    {
        if(msg[0] == motor_address)
        {
			/*
			Byte 0: motor address
			Byte 1-4: command word
			Byte 5-8: checksum
			*/
        	if(len < NUM_BYTES_ADDRESS+sizeof(int32_t)+NUM_BYTES_CHECKSUM)	//memory check for oaddnig the command word
        	{
        		return ERROR_MALFORMED_MESSAGE;
        	}
        	int32_t * p_cmd = (int32_t*)(&msg[1]);
        	comms->command_word = *p_cmd;	//load command word

        	if(replybuf_size < NUM_BYTES_ADDRESS + sizeof(int16_t) +sizeof(int16_t) + sizeof(int32_t) + NUM_BYTES_CHECKSUM)	//check that our intended reply will fit in the buffer
        	{
        		return ERROR_MALFORMED_MESSAGE;
        	}

        	int bidx = 0;   //index for the reply buffer

			//Consider: instead of always sending to master, we could send to the address of the motor that sent the message. 
			//We would want to make sure that the serial handler on slaves masks reads during self transmission, because
			//RS485 will put them on the bus. But this information could be more useful than addressing the master, as it 
			//helps us confirm that the message was sent by the intended motor.
        	p_replybuf[bidx++] = MASTER_MOTOR_ADDRESS;	//always send to master.

        	//load the first value (two bytes of gl_iq)
        	int16_t val_compressed_i16 = (int16_t)(comms->gl_iq);
        	unsigned char * p_val = (unsigned char *)(&val_compressed_i16);
        	for(int i = 0; i < sizeof(val_compressed_i16); i++)
        	{
        		p_replybuf[bidx++] = p_val[i];
        	}

        	//load the second value (two bytes of velocity)
        	val_compressed_i16 = (int16_t)(comms->gl_rotor_velocity);
        	p_val = (unsigned char *)(&val_compressed_i16);
        	for(int i = 0; i < sizeof(val_compressed_i16); i++)
        	{
        		p_replybuf[bidx++] = p_val[i];
        	}

        	//load the third value (4 bytes of gl_theta_rem_m)
        	p_val = (unsigned char *)(&comms->gl_rotor_theta);
        	for(int i = 0; i < sizeof(int32_t); i++)
        	{
        		p_replybuf[bidx++] = p_val[i];
        	}
			
			uint16_t checksum = get_crc16(p_replybuf, bidx);
			p_val = (unsigned char *)(&checksum);
			for(int i = 0; i < sizeof(uint16_t); i++)
			{
				p_replybuf[bidx++] = p_val[i];
			}
			*reply_len = bidx;

        	return SERIAL_PROTOCOL_SUCCESS;

        }
        else if(msg[0] == misc_address)
        {
            //remove address and checksum from the message and then parse
            msg = &(msg[1]);
            return parse_misc_command(msg, (len-(NUM_BYTES_CHECKSUM + NUM_BYTES_ADDRESS)), p_replybuf, replybuf_size, reply_len, comms);
        }
        else
        {
            return ADDRESS_FILTERED;
        }
    }
}

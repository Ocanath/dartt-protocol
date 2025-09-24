#include <stdint.h>
#include <assert.h>
#include "dartt.h"
#include "checksum.h"

/**
 * @brief Calculate the 32-bit word index of a field within a memory structure.
 * 
 * This function determines the offset of a specific field within a larger memory structure,
 * expressed as a 32-bit word index. Used primarily for creating memory access messages
 * that reference specific fields by their word offset.
 * 
 * @param p_field Pointer to the field whose index is to be calculated
 * @param mem Pointer to the base of the memory structure
 * @param mem_size Size of the memory structure in bytes
 * 
 * @return Word index of the field (offset / sizeof(int32_t)), or error code:
 *         - ERROR_INVALID_ARGUMENT if pointers are NULL, field is out of bounds,
 *           or field is not aligned to 32-bit boundaries
 * 
 * @note The field must be aligned to 32-bit (4-byte) boundaries.
 * @note This function performs bounds checking to ensure the field is within the structure.
 */
size_t index_of_field(void * p_field, void * mem, size_t mem_size)
{
    //null pointer checks
    if(p_field == NULL || mem == NULL)
    {
        return ERROR_INVALID_ARGUMENT;
    }
	// Ensure p_field is within the bounds of the comms struct
    unsigned char * pbase = (unsigned char *)(mem);
    unsigned char * p_field_nonvoid = (unsigned char *)p_field;

    if(p_field_nonvoid < pbase || p_field_nonvoid > (pbase + mem_size))
    {
        return ERROR_INVALID_ARGUMENT;
    }
    
    //Check for underrun
    if(p_field_nonvoid < pbase)
    {
        return ERROR_INVALID_ARGUMENT;
    }

    size_t offset = p_field_nonvoid - pbase;
    
    // Check if the field is actually within the struct (overrun check)
    if(offset >= mem_size)
    {
        return ERROR_INVALID_ARGUMENT;
    }
    
    // Ensure the offset is aligned to 32-bit boundaries
    if(offset % sizeof(int32_t) != 0)
    {
        return ERROR_INVALID_ARGUMENT;
    }
    
    return offset / sizeof(int32_t);
}

/**
 * @brief Copy the entire contents of one buffer to another buffer of identical size.
 * 
 * This function performs a complete copy of data from an input buffer to an output buffer.
 * Both buffers must have identical sizes, and all buffer pointers must be valid.
 * 
 * @param in Source buffer containing data to copy
 * @param out Destination buffer to receive copied data
 * 
 * @return SERIAL_PROTOCOL_SUCCESS on success, or error code:
 *         - ERROR_INVALID_ARGUMENT if any buffer pointer is NULL
 *         - ERROR_MEMORY_OVERRUN if buffer sizes don't match
 * 
 * @note Both buffers must have identical size fields for the copy to proceed.
 * @note The function copies exactly buffer->size bytes, regardless of the len field.
 */
int copy_buf_full(buffer_t * in, buffer_t * out)
{
	if(in == NULL || out == NULL)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	else if(in->buf == NULL || out->buf == NULL)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	if(in->size != out->size)
	{
		return ERROR_MEMORY_OVERRUN;
	}
	for(int i = 0; i < in->size; i++)
	{
		out->buf[i] = in->buf[i];
	}
	return SERIAL_PROTOCOL_SUCCESS;
}

/**
 * @brief Calculate the complementary address for address space mapping.
 * 
 * This function implements a bidirectional address mapping between motor and misc
 * address spaces using the formula: complementary = 0xFF - address.
 * 
 * Address mappings:
 * - Motor addresses (0x00-0x7E) ↔ Misc addresses (0x81-0xFF)
 * - Motor master (0x7F) ↔ Misc master (0x80)
 * 
 * @param address Input address from either address space
 * 
 * @return Complementary address in the opposite address space
 * 
 * @note This mapping is symmetric: dartt_get_complementary_address(dartt_get_complementary_address(x)) == x
 * @note Used for routing messages between motor and misc subsystems.
 */
unsigned char dartt_get_complementary_address(unsigned char address)
{
    return 0xFF - address;
}

/**
 * @brief Validate write message parameters and buffer capacity before frame creation.
 * 
 * This function performs comprehensive validation of write message parameters
 * and ensures the output buffer has sufficient capacity for the resulting frame.
 * Intended for use during initialization to validate statically allocated buffers.
 * 
 * @param msg Write message structure containing payload and addressing information
 * @param type Serial message type (TYPE_SERIAL_MESSAGE, TYPE_ADDR_MESSAGE, or TYPE_ADDR_CRC_MESSAGE)
 * @param output Output buffer that will receive the generated frame
 * 
 * @return SERIAL_PROTOCOL_SUCCESS if validation passes, or error code:
 *         - ERROR_INVALID_ARGUMENT if parameters are NULL, invalid type, or empty payload
 *         - ERROR_MEMORY_OVERRUN if output buffer is too small for the resulting frame
 * 
 * @note Call this once during initialization on statically defined memory.
 * @note Frame overhead varies by type: SERIAL (addr+idx+crc), ADDR (idx+crc), ADDR_CRC (idx only).
 */
int check_write_args(misc_write_message_t * msg, serial_message_type_t type, buffer_t * output)
{
    if(msg == NULL || output == NULL)
    {
        return ERROR_INVALID_ARGUMENT;
    }
	if(!(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE || type == TYPE_ADDR_CRC_MESSAGE))
	{
		return ERROR_INVALID_ARGUMENT;
	}
    if(msg->payload.len == 0 || msg->payload.buf == NULL || output->buf == NULL)
    {
        return ERROR_INVALID_ARGUMENT;  
    }

    //pre-check lengths for overrun
    if(type == TYPE_SERIAL_MESSAGE)
    {
        if( (msg->payload.len + (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM) ) > output->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
    }
    else if(type == TYPE_ADDR_MESSAGE)
    {
        if(msg->payload.len + (NUM_BYTES_INDEX + NUM_BYTES_CHECKSUM) > output->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
    }
    else if (type == TYPE_ADDR_CRC_MESSAGE)
    {
        if( (msg->payload.len + NUM_BYTES_INDEX) > output->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
    }
    else
    {
        return ERROR_INVALID_ARGUMENT;
    }
    return SERIAL_PROTOCOL_SUCCESS;
}

/**
 * @brief Generate a write frame from a message structure.
 * 
 * This function constructs a complete write frame ready for transmission,
 * including addressing (if applicable), payload data, and CRC (if applicable).
 * The read/write bit in the index is cleared to indicate a write operation.
 * 
 * This function provides a traversal from the Payload layer to the Frame layer.
 * 
 * @param msg Write message containing address, index, and payload data
 * @param type Frame type determining structure (address and CRC inclusion)
 * @param output Buffer to receive the generated frame (len will be updated)
 * 
 * @return SERIAL_PROTOCOL_SUCCESS on successful frame generation
 * 
 * @note Arguments must be pre-validated using check_write_args().
 * @note Frame structure varies by type:
 *       - TYPE_SERIAL_MESSAGE: [addr][idx_lo][idx_hi][payload...][crc_lo][crc_hi]
 *       - TYPE_ADDR_MESSAGE: [idx_lo][idx_hi][payload...][crc_lo][crc_hi]
 *       - TYPE_ADDR_CRC_MESSAGE: [idx_lo][idx_hi][payload...]
 * @note The MSB of the index is cleared to indicate write operation.
 */
int dartt_create_write_frame(misc_write_message_t * msg, serial_message_type_t type, buffer_t * output)
{
    assert(check_write_args(msg,type,output) == SERIAL_PROTOCOL_SUCCESS);  //assert to save on runtime execution
    
    //prepare the serial buffer
    output->len = 0;
    if(type == TYPE_SERIAL_MESSAGE)
    {
        output->buf[output->len++] = msg->address;        
    }
    uint16_t rw_index = (msg->index & (~READ_WRITE_BITMASK));   //MSB = 0 for write, low 15 for index
    output->buf[output->len++] = (unsigned char)(rw_index & 0x00FF);
    output->buf[output->len++] = (unsigned char)((rw_index & 0xFF00) >> 8);
    for(int i = 0; i < msg->payload.len; i++)
    {
        output->buf[output->len++] = msg->payload.buf[i];
    }
    if(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE)
    {
        uint16_t crc = get_crc16(output->buf, output->len);
        output->buf[output->len++] = (unsigned char)(crc & 0x00FF);
        output->buf[output->len++] = (unsigned char)((crc & 0xFF00) >> 8);
    }
    return SERIAL_PROTOCOL_SUCCESS;
}

/**
 * @brief Validate read message parameters and buffer capacity before frame creation.
 * 
 * This function performs comprehensive validation of read message parameters
 * and ensures the output buffer has sufficient capacity for the resulting frame.
 * Intended for use during initialization to validate statically allocated buffers.
 * 
 * @param msg Read message structure containing address, index, and byte count
 * @param type Serial message type (TYPE_SERIAL_MESSAGE, TYPE_ADDR_MESSAGE, or TYPE_ADDR_CRC_MESSAGE)
 * @param output Output buffer that will receive the generated frame
 * 
 * @return SERIAL_PROTOCOL_SUCCESS if validation passes, or error code:
 *         - ERROR_INVALID_ARGUMENT if parameters are NULL or invalid type
 *         - ERROR_MEMORY_OVERRUN if output buffer is too small for the resulting frame
 * 
 * @note Call this once during initialization, then use dartt_create_read_frame() with confidence.
 * @note Read frames have fixed size based on type: address + index + num_bytes + CRC (if applicable).
 */
int check_read_args(misc_read_message_t * msg, serial_message_type_t type, buffer_t * output)
{
    if(msg == NULL || output == NULL)
    {
        return ERROR_INVALID_ARGUMENT;
    }
	if(!(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE || type == TYPE_ADDR_CRC_MESSAGE))
	{
		return ERROR_INVALID_ARGUMENT;
	}
    if(output->buf == NULL)
    {
        return ERROR_INVALID_ARGUMENT;  
    }

    //pre-check lengths for overrun
    if(type == TYPE_SERIAL_MESSAGE)
    {
        if( ( (NUM_BYTES_ADDRESS + NUM_BYTES_INDEX + NUM_BYTES_NUMWORDS_READREQUEST + NUM_BYTES_CHECKSUM) ) > output->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
    }
    else if(type == TYPE_ADDR_MESSAGE)
    {
        if( (NUM_BYTES_INDEX + NUM_BYTES_NUMWORDS_READREQUEST + NUM_BYTES_CHECKSUM) > output->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
    }
    else if (type == TYPE_ADDR_CRC_MESSAGE)
    {
        if( (NUM_BYTES_INDEX + NUM_BYTES_NUMWORDS_READREQUEST) > output->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
    }
    else
    {
        return ERROR_INVALID_ARGUMENT;
    }
    return SERIAL_PROTOCOL_SUCCESS;
}

/**
 * @brief Generate a read frame from a message structure.
 * 
 * This function constructs a complete read frame ready for transmission,
 * including addressing (if applicable), index with read bit set, byte count,
 * and CRC (if applicable).
 * 
 * This function provides a traversal from the Payload layer to the Frame layer.
 * 
 * @param msg Read message containing address, index, and number of bytes to read
 * @param type Frame type determining structure (address and CRC inclusion)
 * @param output Buffer to receive the generated frame (len will be updated)
 * 
 * @return SERIAL_PROTOCOL_SUCCESS on successful frame generation
 * 
 * @note Arguments must be pre-validated using check_read_args().
 * @note Frame structure varies by type:
 *       - TYPE_SERIAL_MESSAGE: [addr][idx_lo|0x80][idx_hi][bytes_lo][bytes_hi][crc_lo][crc_hi]
 *       - TYPE_ADDR_MESSAGE: [idx_lo|0x80][idx_hi][bytes_lo][bytes_hi][crc_lo][crc_hi]
 *       - TYPE_ADDR_CRC_MESSAGE: [idx_lo|0x80][idx_hi][bytes_lo][bytes_hi]
 * @note The MSB of the index is set to indicate read operation.
 */
int dartt_create_read_frame(misc_read_message_t * msg, serial_message_type_t type, buffer_t * output)
{
    assert(check_read_args(msg,type,output) == SERIAL_PROTOCOL_SUCCESS);
    assert(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE || type == TYPE_ADDR_CRC_MESSAGE);

    output->len = 0;
    if(type == TYPE_SERIAL_MESSAGE)
    {
        output->buf[output->len++] = msg->address;
    }
    uint16_t rw_index = msg->index | READ_WRITE_BITMASK;
    output->buf[output->len++] = (unsigned char)(rw_index & 0x00FF);
    output->buf[output->len++] = (unsigned char)((rw_index & 0xFF00) >> 8);
    output->buf[output->len++] = (unsigned char)(msg->num_bytes & 0x00FF);
    output->buf[output->len++] = (unsigned char)((msg->num_bytes & 0xFF00) >> 8);
    if(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE)
    {
        uint16_t crc = get_crc16(output->buf, output->len);
        output->buf[output->len++] = (unsigned char)(crc & 0x00FF);
        output->buf[output->len++] = (unsigned char)((crc & 0xFF00) >> 8);
    }    
    return SERIAL_PROTOCOL_SUCCESS;
}

/**
 * @brief Parse and execute a payload-layer message (slave-side message handler).
 * 
 * This function processes incoming messages that have been stripped of address and CRC
 * information by upstream processing. It determines whether the message is a read or write
 * operation based on the read/write bit and executes the appropriate action on the target
 * memory space.
 * 
 * This function provides traversal from Payload to Application (via block memory) for perhiperals
 * 
 * @param pld_msg Payload layer message (address and CRC already removed)
 * @param mem_base Target memory space for read/write operations
 * @param reply_base Buffer for read reply data (raw payload, no framing)
 * 
 * @return SERIAL_PROTOCOL_SUCCESS on successful operation, or error code:
 *         - ERROR_MALFORMED_MESSAGE if message structure is invalid
 *         - ERROR_MEMORY_OVERRUN if operation would exceed buffer bounds
 * 
 * @note Input message format: [idx_lo][idx_hi][payload...] for writes
 *                             [idx_lo|0x80][idx_hi][num_bytes_lo][num_bytes_hi] for reads
 * @note For read operations, reply_base will contain the requested data
 * @note For write operations, reply_base->len is set to 0 (no reply)
 * @note Caller should reserve space for address framing using pointer arithmetic
 * @note This function is message-type agnostic - framing is handled upstream
 */
int dartt_parse_base_serial_message(payload_layer_msg_t* pld_msg, buffer_t * mem_base, buffer_t * reply_base)
{
    assert(pld_msg != NULL && mem_base != NULL && reply_base != NULL);
    assert(pld_msg->msg.buf != NULL && mem_base->buf != NULL && reply_base->buf != NULL);
    assert(pld_msg->msg.size > NUM_BYTES_INDEX && mem_base->size > 0 && reply_base->size > 0);
    assert(pld_msg->msg.len <= pld_msg->msg.size && mem_base->len <= mem_base->size && reply_base->len <= reply_base->size);
    
    //critical check - keep as runtime since this is data-dependent
    if(pld_msg->msg.len <= NUM_BYTES_INDEX)   //if write, it must contain at least one byte of payload. If read, it must contain exactly two additional bytes of read size
    {
        return ERROR_MALFORMED_MESSAGE;
    }

    size_t bidx = 0;
    uint16_t rw_index = 0;
    rw_index |= (uint16_t)(pld_msg->msg.buf[bidx++]);
    rw_index |= (((uint16_t)(pld_msg->msg.buf[bidx++])) << 8);
    uint16_t rw_bit = rw_index & READ_WRITE_BITMASK;  //omit the shift and perform zero comparison for speed
    size_t word_offset = ((size_t)(rw_index & (~READ_WRITE_BITMASK)))*sizeof(uint32_t); 
    if(rw_bit != 0) //read
    {
        if(pld_msg->msg.len != NUM_BYTES_INDEX + NUM_BYTES_NUMWORDS_READREQUEST)  //read messages must have precisely this content (once addr and crc are removed, if relevant)
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        uint16_t num_bytes = 0;
        num_bytes |= (uint16_t)(pld_msg->msg.buf[bidx++]);
        num_bytes |= (((uint16_t)(pld_msg->msg.buf[bidx++])) << 8);
        if(num_bytes > reply_base->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }

        
        unsigned char * cpy_ptr = mem_base->buf + word_offset;
        if(word_offset + num_bytes > mem_base->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }

        uint16_t i;
        for(i = 0; i < num_bytes; i++)
        {
            reply_base->buf[i] = cpy_ptr[i];
        }
        reply_base->len = i;
        return SERIAL_PROTOCOL_SUCCESS; //caller needs to finish the reply formatting
    }
    else    //write
    {
        //from our min length and min size checks, we know the input buffer must have both minimum size and
        //minimum length of 3, and we know that if we are here, bidx is equal to 2.
        //Therefore, it is safe to subtract bidx from len
        unsigned char * write_ptr = pld_msg->msg.buf + bidx;
        size_t nbytes_to_write = pld_msg->msg.len - bidx; //this can be 1 at minimum, and cannot underflow due to our checks above. overrun protection is guaranteed here too due to size and len checks
        
        if(word_offset + nbytes_to_write > mem_base->size)
        {
            return ERROR_MEMORY_OVERRUN;
        }
        unsigned char * mem_ptr = mem_base->buf + word_offset;
        for(int i = 0; i < nbytes_to_write; i++)
        {
            mem_ptr[i] = write_ptr[i];  //perform the copy
        }
        reply_base->len = 0;    //erase the reply. Success and nonzero reply len should trigger transmission of a reply frame, and we don't reply to write messages!
        return SERIAL_PROTOCOL_SUCCESS; //no reply, so caller doesn't need to do anything else
    }
}

/**
 * @brief Validate the CRC checksum of a message buffer.
 * 
 * This function verifies that the CRC-16 checksum appended to the end of a message
 * matches the calculated checksum of the message content. The CRC is always stored
 * as the last two bytes of the buffer in little-endian format.
 * 
 * This function operates within the frame layer only
 * 
 * @param input Buffer containing message data with appended CRC
 * 
 * @return SERIAL_PROTOCOL_SUCCESS if CRC is valid, or error code:
 *         - ERROR_INVALID_ARGUMENT if buffer is too short to contain CRC
 *         - ERROR_CHECKSUM_MISMATCH if calculated CRC doesn't match stored CRC
 * 
 * @note CRC is calculated over all bytes except the last two (the CRC itself)
 * @note CRC bytes are stored in little-endian format: [crc_low][crc_high]
 * @note Buffer must be at least NUM_BYTES_CHECKSUM + 1 bytes long
 */
int validate_crc(buffer_t * input)
{
    assert(input != NULL);
    assert(input->buf != NULL);
    assert(input->size != 0);
    assert(input->len <= input->size);
    if(input->len <= NUM_BYTES_CHECKSUM)
    {
        return ERROR_INVALID_ARGUMENT;
    }
    uint16_t crc = get_crc16(input->buf, input->len - NUM_BYTES_CHECKSUM);
    uint16_t m_crc = 0;
    unsigned char * pchecksum = input->buf + (input->len - NUM_BYTES_CHECKSUM);
    m_crc |= (uint16_t)pchecksum[0];
    m_crc |= ((uint16_t)pchecksum[1]) << 8;
    if(m_crc == crc)
    {
        return SERIAL_PROTOCOL_SUCCESS;
    }
    else
    {
        return ERROR_CHECKSUM_MISMATCH;
    }
}

/**
 * @brief Append a CRC-16 checksum to the end of a message buffer.
 * 
 * This function calculates the CRC-16 checksum of the current buffer contents
 * and appends it to the end of the buffer in little-endian format. The buffer's
 * length field is updated to include the appended CRC bytes.
 * 
 * This function operates within the frame layer only
 * 
 * @param input Buffer containing message data (len will be increased by 2)
 * 
 * @return SERIAL_PROTOCOL_SUCCESS on success, or error code:
 *         - ERROR_MEMORY_OVERRUN if buffer doesn't have space for CRC bytes
 * 
 * @note CRC is calculated over the current buffer contents (0 to len-1)
 * @note CRC bytes are appended in little-endian format: [crc_low][crc_high]
 * @note Buffer must have at least NUM_BYTES_CHECKSUM free bytes remaining
 * @note The len field is automatically updated to include the CRC bytes
 */
int append_crc(buffer_t * input)
{
    assert(input != NULL);
    assert(input->buf != NULL);
    assert(input->size != 0);
    assert(input->len <= input->size);

    if(input->len + NUM_BYTES_CHECKSUM > input->size)
    {
        return ERROR_MEMORY_OVERRUN;
    }
    uint16_t crc = get_crc16(input->buf, input->len);
    input->buf[input->len++] = (unsigned char)(crc & 0x00FF);
    input->buf[input->len++] = (unsigned char)((crc & 0xFF00) >> 8);

    return SERIAL_PROTOCOL_SUCCESS;
}

/**
 * @brief Parse a slave's read reply and copy data to the appropriate memory location (master-side).
 * 
 * This function processes a read reply from a slave device, validating the reply length
 * against the original request and copying the data to the correct offset within the
 * destination memory buffer. The offset is calculated from the original read message's index.
 * 
 * This function provides a traversal from payload layer to application layer, for controller devices
 * 
 * @param payload Payload layer message containing the slave's reply data
 * @param original_msg Original read message that generated this reply
 * @param dest Destination memory buffer to receive the reply data
 * 
 * @return SERIAL_PROTOCOL_SUCCESS on successful parsing, or error code:
 *         - ERROR_MEMORY_OVERRUN if calculated offset exceeds destination bounds
 *         - ERROR_MALFORMED_MESSAGE if reply length doesn't match requested length
 * 
 * @note The destination offset is calculated as: original_msg->index * sizeof(uint32_t)
 * @note Reply length must exactly match original_msg->num_bytes
 * @note This function is called after dartt_frame_to_payload() has extracted the raw payload
 * @note Used by master devices to reconstruct remote memory after read operations
 */
int dartt_parse_read_reply(payload_layer_msg_t * payload, misc_read_message_t * original_msg, buffer_t * dest)
{     
    assert(dest != NULL && payload != NULL && original_msg != NULL);
    assert(dest->buf != NULL && payload->msg.buf != NULL);
    assert(dest->size > 0 && payload->msg.size > 0);
    assert(dest->len <= dest->size && payload->msg.len <= payload->msg.size);
    
    // Calculate the offset into the destination buffer based on the original read index
    size_t byte_offset = ((size_t)original_msg->index) * sizeof(uint32_t);
    
    // Validate that the offset and data length don't exceed destination buffer bounds
    if(byte_offset >= dest->size)
    {
        return ERROR_MEMORY_OVERRUN;
    }
    if(byte_offset + payload->msg.len > dest->size)
    {
        return ERROR_MEMORY_OVERRUN;
    }
    
    // Validate that the reply length matches what we requested
    if(payload->msg.len != original_msg->num_bytes)
    {
        return ERROR_MALFORMED_MESSAGE;
    }
    
    // Copy the reply data to the correct offset in the destination buffer
    unsigned char * dest_ptr = dest->buf + byte_offset;
    for(int i = 0; i < payload->msg.len; i++)
    {
        dest_ptr[i] = payload->msg.buf[i];
    }
    
    return SERIAL_PROTOCOL_SUCCESS;
}

/**
 * @brief Convert a frame-layer message to payload-layer format by removing framing overhead.
 * 
 * This function performs frame-to-payload translation by stripping addressing and CRC
 * information from incoming messages while validating checksums where applicable.
 * It supports both alias mode (pointer arithmetic) and copy mode for payload extraction.
 * 
 * This function is for frame layer to payload layer traversal - it applies to any message type, 
 * including motor writes/replies, misc write, misc read, and misc read reply messages. Essentially
 * it strips away any address and/or checksum information present and delivers the payload,
 * either through pointer arithmetic/aliasing (fast) or copying (flexible).
 * 
 * @param ser_msg Input frame-layer message buffer
 * @param type Message type determining frame structure
 * @param pld_mode PAYLOAD_ALIAS (use pointers) or PAYLOAD_COPY (copy data)
 * @param pld Output payload-layer message structure
 * 
 * @return SERIAL_PROTOCOL_SUCCESS on successful conversion, or error code:
 *         - ERROR_MALFORMED_MESSAGE if frame is too short or malformed
 *         - ERROR_CHECKSUM_MISMATCH if CRC validation fails
 *         - ERROR_INVALID_ARGUMENT if pld_mode is invalid or copy buffer is NULL
 *         - ERROR_MEMORY_OVERRUN if payload doesn't fit in copy buffer
 * 
 * @note Frame processing by type:
 *       - TYPE_SERIAL_MESSAGE: Validates CRC, extracts address, removes both from payload
 *       - TYPE_ADDR_MESSAGE: Validates CRC, removes CRC from payload (no address)
 *       - TYPE_ADDR_CRC_MESSAGE: No validation, payload = entire frame
 * @note PAYLOAD_ALIAS mode uses pointer arithmetic (zero-copy, but payload tied to frame)
 * @note PAYLOAD_COPY mode copies payload data (safe for frame buffer reuse)
 * @note This function only handles framing - payload structure is decoded downstream
 */
int dartt_frame_to_payload(buffer_t * ser_msg, serial_message_type_t type, payload_mode_t pld_mode, payload_layer_msg_t * pld)
{
    assert(ser_msg != NULL && pld != NULL);
    assert(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE || type == TYPE_ADDR_CRC_MESSAGE);
	assert(ser_msg->buf != NULL);
	assert(ser_msg->size != 0);
	assert(ser_msg->len != 0);
	assert((pld->msg.buf == NULL && pld->msg.size == 0) || (pld->msg.buf != NULL && pld->msg.size != 0));

	if(type == TYPE_SERIAL_MESSAGE)
    {
		//first step - crc validation on input message
        if(ser_msg->len <= (NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM))    //message has to have room for a checksum, an index, and at least one additional byte
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        int rc = validate_crc(ser_msg);
        if(rc != SERIAL_PROTOCOL_SUCCESS)
        {
            return rc;	//checksum must match
        }

		if(pld_mode == PAYLOAD_ALIAS)	//Use pointer arithmetic
		{
			pld->msg.buf = ser_msg->buf;
			pld->msg.size = ser_msg->size;
			pld->msg.len = ser_msg->len - NUM_BYTES_CHECKSUM;	
			
			pld->address = ser_msg->buf[0];
			pld->msg.buf += NUM_BYTES_ADDRESS;
			pld->msg.len -= NUM_BYTES_ADDRESS;
			//truncate checksum off and use pointer arithmetic to load payload to addr
		}
		else if(pld_mode == PAYLOAD_COPY)	//
		{
			if(pld->msg.buf == NULL)
			{
				return ERROR_INVALID_ARGUMENT;
			}
			size_t newlen  = ser_msg->len - (NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM);
			if(newlen > pld->msg.size)
			{
				return ERROR_MEMORY_OVERRUN;
			}
			pld->address = ser_msg->buf[0];
			unsigned char * sm_start = ser_msg->buf + NUM_BYTES_ADDRESS; //skip address
			
			for(int i = 0; i < newlen; i++)
			{
				pld->msg.buf[i] = sm_start[i];
			}
			pld->msg.len = newlen;
		}
		else
		{
			return ERROR_INVALID_ARGUMENT;
		}
        return SERIAL_PROTOCOL_SUCCESS;
    }
	else if (type == TYPE_ADDR_MESSAGE)
    {
        if(ser_msg->len <= NUM_BYTES_CHECKSUM)    //message has to have room for a checksum, an index, and at least one additional byte
        {
            return ERROR_MALFORMED_MESSAGE;
        }
        int rc = validate_crc(ser_msg);
        if(rc != SERIAL_PROTOCOL_SUCCESS)
        {
            return rc;
        }
		if(pld_mode == PAYLOAD_ALIAS)
		{
			pld->msg.buf = ser_msg->buf;
			pld->msg.size = ser_msg->size;
			pld->msg.len = (ser_msg->len - NUM_BYTES_CHECKSUM);
		}
		else if (pld_mode == PAYLOAD_COPY)
		{
			if(pld->msg.buf == NULL)
			{
				return ERROR_INVALID_ARGUMENT;
			}
			size_t newlen  = ser_msg->len - NUM_BYTES_CHECKSUM;
			if(newlen > pld->msg.size)
			{
				return ERROR_MEMORY_OVERRUN;
			}
			for(int i = 0; i < newlen; i++)
			{
				pld->msg.buf[i] = ser_msg->buf[i];
			}
			pld->msg.len = newlen;
		}
		else
		{
			return ERROR_INVALID_ARGUMENT;
		}
        return SERIAL_PROTOCOL_SUCCESS;
    }
	else if(type == TYPE_ADDR_CRC_MESSAGE)
	{
		if(pld_mode == PAYLOAD_ALIAS)	//use pointer arithmetic to have the pld->msg refer to the payload section of the frame layer message
		{
			pld->msg.buf = ser_msg->buf;
			pld->msg.size = ser_msg->size;
			pld->msg.len = ser_msg->len;
		}
		else if(pld_mode == PAYLOAD_COPY)	//make a copy
		{
			if(pld->msg.buf == NULL)
			{
				return ERROR_INVALID_ARGUMENT;
			}
			if(ser_msg->len > pld->msg.size)
			{
				return ERROR_MEMORY_OVERRUN;
			}
			for(int i = 0; i < ser_msg->len; i++)
			{
				pld->msg.buf[i] = ser_msg->buf[i];
			}
			pld->msg.len = ser_msg->len;
		}
		else
		{
			return ERROR_INVALID_ARGUMENT;
		}
		return SERIAL_PROTOCOL_SUCCESS;
	}
	return ERROR_INVALID_ARGUMENT;
}

/**
 * @brief Process a payload-layer message and generate an appropriately formatted reply frame.
 * 
 * This function implements the complete message processing pipeline for misc address space
 * messages, including payload parsing, memory operations, and reply frame generation.
 * It handles frame formatting based on the input message type.
 * 
 * This function provides traversal from payload to application for peripherals (block memory access) as well as payload to frame (read replies), simultaneously
 * The intended use is for peripheral devices, after converting and incoming frame to the payload layer using dartt_frame_to_payload.
 * 
 * @param pld_msg Payload-layer message to process
 * @param type Original frame type (determines reply frame format)
 * @param mem_base Target memory space for operations
 * @param reply Buffer to receive formatted reply frame
 * 
 * @return SERIAL_PROTOCOL_SUCCESS on successful processing, or error code from:
 *         dartt_parse_base_serial_message() or append_crc()
 * 
 * @note Reply formatting by type:
 *       - TYPE_SERIAL_MESSAGE: [MASTER_MISC_ADDRESS][payload][crc] (if read reply exists)
 *       - TYPE_ADDR_MESSAGE: [payload][crc] (if read reply exists)
 *       - TYPE_ADDR_CRC_MESSAGE: [payload] (if read reply exists)
 * @note Write operations produce no reply (reply->len = 0)
 * @note Read operations generate reply data formatted according to frame type
 * @note This function coordinates payload processing with frame formatting
 * @note Typically called after dartt_frame_to_payload() and address range validation
 */
int dartt_parse_general_message(payload_layer_msg_t * pld_msg, serial_message_type_t type, buffer_t * mem_base, buffer_t * reply)
{
    assert(pld_msg != NULL && mem_base != NULL && reply != NULL);
    assert(pld_msg->msg.buf != NULL && mem_base->buf != NULL && reply->buf != NULL);
    assert(pld_msg->msg.size != 0 && mem_base->size != 0 && reply->size != 0);
    assert(pld_msg->msg.len <= pld_msg->msg.size && mem_base->len <= mem_base->size && reply->len <= reply->size);   
    assert(type == TYPE_SERIAL_MESSAGE || type == TYPE_ADDR_MESSAGE || type == TYPE_ADDR_CRC_MESSAGE);
	
    if(type == TYPE_SERIAL_MESSAGE)
    {
        buffer_t reply_cpy = {
            .buf = reply->buf + NUM_BYTES_ADDRESS,     //make room for the address, which we will load after if necessary
            .size = reply->size - 1,
            .len = 0
        };
        int rc = dartt_parse_base_serial_message(pld_msg, mem_base, &reply_cpy);    //will copy from 1 to len. the original reply buffer is now ready for address and crc loading
        if(rc == SERIAL_PROTOCOL_SUCCESS)
        {
			if(reply_cpy.len != 0)
			{
				//append address
				reply->buf[0] = MASTER_MISC_ADDRESS;
				reply->len = reply_cpy.len + NUM_BYTES_ADDRESS; //update len now that we have the address in the base message
				return append_crc(reply);   //the checks in this function also check for address length increase/overrun
			}
			else
			{
				reply->len = reply_cpy.len;
			}
        }
        else if(rc != SERIAL_PROTOCOL_SUCCESS)
        {
            return rc;
        }
        return SERIAL_PROTOCOL_SUCCESS;
    }
    else if (type == TYPE_ADDR_MESSAGE)
    {
        reply->len = 0;
        int rc = dartt_parse_base_serial_message(pld_msg, mem_base, reply);
        if(rc == SERIAL_PROTOCOL_SUCCESS && reply->len != 0)
        {
            return append_crc(reply);
        }
        else if (rc != SERIAL_PROTOCOL_SUCCESS)
        {
            return rc;
        }
        return SERIAL_PROTOCOL_SUCCESS;

    }
    else if (type == TYPE_ADDR_CRC_MESSAGE)
    {
        return dartt_parse_base_serial_message(pld_msg, mem_base, reply);   //type 3 carries the base protocol with no additional payload dressings
    }
    else
    {
        return ERROR_INVALID_ARGUMENT;  //should never end up here - assert should catch this. Can only happen in release builds untested in debug
    }
}




/**
 * @brief This function scans two buffers (one control and one peripheral) for the presense of any mismatch between control and peripheral.
 * If a difference is found, the master then writes the control copy TO the peripheral, and reads it back to verify a match.
 * Access to hardware is managed with callback function pointers. Callback function pointer is called with the function.
 * 
 * @param misc_address The address of the target peripheral device for message creation. Can be set to zero if not used (type 1)
 * @param ctl Pointer to the buffer containing the 'control' copy. This is the master copy we are synchronizing peripheral devices to
 * @param periph Pointer to the buffer containing the 'peripheral' copy. This is our internal 'latest' reference to the peripheral
 * @param msg_type Dartt message type for message creation
 * @param tx_buf Pointer to buffer used for blocking transmission
 * @param rx_buf Pointer to buffer used for blocking reception
 * @param blocking_tx_callback Function pointer, callback function for blocking transmission
 * @param blocking_rx_callback Function pointer, callback for blocking reception
 * @param timeout For blocking write and read, a millisecond timeout value.
 * */
int dartt_sync(unsigned char misc_address,
		buffer_t * ctl,
		buffer_t * periph,
		buffer_t * base,
		serial_message_type_t msg_type,
		buffer_t * tx_buf,
		buffer_t * rx_buf,
		int (*blocking_tx_callback)(unsigned char, buffer_t*, uint32_t timeout),
		int (*blocking_rx_callback)(unsigned char, buffer_t*, uint32_t timeout),
		uint32_t timeout_ms
		)	//callbacks?
{
    /*TODO Implement a dartt_sync_t structure to wrap these things, and a callback registration function to load the function pointers. */
	if(ctl->size != periph->size)
	{
		return ERROR_MEMORY_OVERRUN;
	}
	if(ctl->size % sizeof(int32_t) == 0)
	{
		return ERROR_INVALID_ARGUMENT;	//make sure you're 32 bit aligned in all refs
	}
	int field_bidx;
	for(field_bidx = 0; field_bidx < ctl->size; field_bidx += sizeof(int32_t))
	{
		uint8_t match = 1;
		for(int i = 0; i < sizeof(int32_t); i++)
		{
			int bidx = field_bidx + i;
			if(ctl->buf[bidx] != periph->buf[bidx])
			{
				match = 0;
				break;
			}
		}
		if(match == 0)
		{
			// uint16_t field_index = field_bidx/sizeof(int32_t);
			uint16_t field_index = index_of_field( (void*)(&ctl->buf[field_bidx]), (void*)(&base->buf[0]), base->size );
			//write then read the word in question
			misc_write_message_t write_msg =
			{
					.address = misc_address,
					.index = field_index,
					.payload = {
							.buf = &ctl->buf[field_bidx],
							.size = sizeof(int32_t),
							.len = sizeof(int32_t)
					}
			};
			int rc = dartt_create_write_frame(&write_msg, msg_type, tx_buf);
			if(rc != SERIAL_PROTOCOL_SUCCESS)
			{
				return rc;
			}
			//blocking write callback
			rc = (*blocking_tx_callback)(misc_address, tx_buf, timeout_ms);
			if(rc != SERIAL_PROTOCOL_SUCCESS)
			{
				return rc;
			}

			misc_read_message_t read_msg =
			{
					.address = misc_address,
					.index = field_index,
					.num_bytes = write_msg.payload.len
			};
			rc = dartt_create_read_frame(&read_msg, msg_type, tx_buf);
			if(rc != SERIAL_PROTOCOL_SUCCESS)
			{
				return rc;
			}
			rc = (*blocking_tx_callback)(misc_address, tx_buf, timeout_ms);
			if(rc != SERIAL_PROTOCOL_SUCCESS)
			{
				return rc;
			}

			rc = (*blocking_rx_callback)(misc_address, rx_buf, timeout_ms);
			if(rc != SERIAL_PROTOCOL_SUCCESS)
			{
				return rc;
			}

			payload_layer_msg_t pld_msg = {};
			rc = dartt_frame_to_payload(rx_buf, msg_type, PAYLOAD_ALIAS, &pld_msg);
			if(rc != SERIAL_PROTOCOL_SUCCESS)
			{
				return rc;
			}


		}
	}

	return SERIAL_PROTOCOL_SUCCESS;

}

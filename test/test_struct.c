#include "checksum.h"
#include "serial-comms.h"
#include "unity.h"

// Example device configuration struct (packed for type-punning)
typedef struct device_config_t
{
    uint32_t device_id;         // Word index 0 - Device identification  
    uint32_t max_speed;         // Word index 1 - Maximum speed limit
    uint32_t acceleration;      // Word index 2 - Acceleration setting
    uint32_t position_target;   // Word index 3 - Target position
    uint32_t current_position;  // Word index 4 - Current position (read-only)
    uint32_t status_flags;      // Word index 5 - Status and error flags
    uint32_t temperature;       // Word index 6 - Temperature reading
    uint32_t firmware_version;  // Word index 7 - Firmware version
} device_config_t;

// Simulated device memory (motor)
static device_config_t motor_config = {
    .device_id = 0x12345678,
    .max_speed = 1000,
    .acceleration = 50,
    .position_target = 0,
    .current_position = 100,
    .status_flags = 0x0001,  // Ready flag
    .temperature = 25,
    .firmware_version = 0x010203
};
//buffer reference to the block of memory - in this application, a typedef struct
buffer_t motor_config_ref = {
	.buf = (unsigned char *)(&motor_config),
	.size = sizeof(device_config_t),
	.len = 0
};

// Simulated device memory (controller) - empty
static device_config_t controller_config = {};
buffer_t controller_config_ref = {
	.buf = (unsigned char * )(&controller_config),
	.size = sizeof(device_config_t),
	.len = 0
};

//simulated tx buffer from the controller
unsigned char controller_tx_mem[64] = {};
buffer_t controller_tx = {
	.buf = controller_tx_mem,
	.size = sizeof(controller_tx_mem),
	.len = 0
};

//create a buffer for the motor tx/motor reply
unsigned char motor_tx_mem[64] = {};
buffer_t motor_tx = {
	.buf = motor_tx_mem,
	.size = sizeof(motor_tx_mem),
	.len = 0
};


/*
    Example-specific wrapper function that creates a TYPE_SERIAL_MESSAGE write frame
    using index_of_field to automatically calculate the struct field index.
    
    Arguments:
        p_field: Pointer to the struct field to write to
        mem_base: Pointer to the base of the struct
        mem_size: Size of the struct in bytes
        address: Target device address
        field_data: Pointer to the data to write
        field_size: Size of the field in bytes
        output_frame: Buffer to store the complete frame
        
    Returns:
        SERIAL_PROTOCOL_SUCCESS on success, error code on failure
*/
int create_struct_write_frame(unsigned char address, 
	unsigned char * field, 
	size_t field_size, 
	device_config_t * pstruct, 
	buffer_t * output_frame)
{
    // Calculate the field index using index_of_field
    int field_index = index_of_field((void*)field, (void*)pstruct, sizeof(device_config_t));
    if(field_index < 0) {
        return field_index; // Return the error code
    }
    if(field_size > sizeof(device_config_t))
	{
		return ERROR_MEMORY_OVERRUN;
	}

    // Create the write message
    misc_write_message_t write_msg = {
        .address = address,
        .index = (uint16_t)field_index,
        .payload = {
            .buf = (unsigned char*)field,
            .size = sizeof(device_config_t) - field_size,
            .len = field_size
        }
    };
    
    // Create the frame (TYPE_SERIAL_MESSAGE only)
    return create_write_frame(&write_msg, TYPE_SERIAL_MESSAGE, output_frame);
}

int create_read_struct_frame(unsigned char address,
	 unsigned char * field,
	  size_t field_size,
	   device_config_t * pstruct, 
	   misc_read_message_t * read_msg_out,
	   buffer_t * output_frame)
{
    // Calculate the field index using index_of_field
    int field_index = index_of_field((void*)field, (void*)pstruct, sizeof(device_config_t));
    if(field_index < 0) 
	{
        return field_index; // Return the error code
    }
	read_msg_out->address = address;
	read_msg_out->index = field_index;
	read_msg_out->num_bytes = field_size;
	
	return create_read_frame(read_msg_out, TYPE_SERIAL_MESSAGE, output_frame);
}

int parse_read_struct_reply(buffer_t* reply_frame, misc_read_message_t* original_msg, device_config_t* pstruct)
{
    // First use frame_to_payload to strip address/CRC and get payload data
    payload_layer_msg_t payload_msg = {
        .address = 0,
        .msg = {.buf = NULL, .size = 0, .len = 0}  // For PAYLOAD_ALIAS mode
    };
    
    int rc = frame_to_payload(reply_frame, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &payload_msg);
    if(rc != SERIAL_PROTOCOL_SUCCESS) {
        return rc;
    }
    
    // Now use parse_read_reply on the clean payload data
    buffer_t dest_buffer = {
        .buf = (unsigned char*)pstruct,
        .size = sizeof(device_config_t),
        .len = 0
    };
    
    return parse_read_reply(&payload_msg, original_msg, &dest_buffer);
}

int create_struct_ref(device_config_t * cfg, buffer_t * ref)
{
	if(cfg == NULL || ref == NULL){return -1;};
	ref->buf = (unsigned char *)cfg;
	ref->size = sizeof(device_config_t);
	ref->len = 0;
	return 0;
}



/*

*/
void test_struct_block_read(void)
{
	const unsigned char motor_address = 3;
	
	//erase ref
	for(int i = 0; i < controller_config_ref.size; i++)
	{
		controller_config_ref.buf[i] = 0;
	}
	device_config_t motor_cfg_backup;
	buffer_t motor_cfg_backup_ref;
	create_struct_ref(&motor_cfg_backup, &motor_cfg_backup_ref);
	copy_buf_full(&motor_config_ref, &motor_cfg_backup_ref);
	

	//create the write message
	misc_read_message_t read_msg = {};
	int rc = create_read_struct_frame(get_complementary_address(motor_address),
		(unsigned char *)(&controller_config.current_position), 
		sizeof(controller_config.current_position),
		&controller_config,
		&read_msg,
		&controller_tx
	);

	//send to peripheral...

	//peripheral recieved message and begins parsing it
	payload_layer_msg_t pld_msg ={};	//can be statically allocated, or local and initialized to zero
	rc = frame_to_payload(&controller_tx, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &pld_msg);	//decode frame to payload - confirmed in previous unit test so exhaustive coverage not required here
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);	
	TEST_ASSERT_EQUAL(get_complementary_address(motor_address), pld_msg.address);	//real application would perform address filtering
	rc = parse_general_message(&pld_msg, TYPE_SERIAL_MESSAGE, &motor_config_ref, &motor_tx);	//parse the decoded payload message
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);	
	TEST_ASSERT_EQUAL(MASTER_MISC_ADDRESS, motor_tx.buf[0]);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, validate_crc(&motor_tx));
	TEST_ASSERT_EQUAL(NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM + read_msg.num_bytes, motor_tx.len);


	rc = frame_to_payload(&motor_tx, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &pld_msg);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);	
	rc = parse_read_reply(&pld_msg, &read_msg, &controller_config_ref);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);	
	TEST_ASSERT_EQUAL(motor_config.current_position, controller_config.current_position);
	for(int i = 0; i < motor_config_ref.size; i++)
	{
		TEST_ASSERT_EQUAL(motor_cfg_backup_ref.buf[i], motor_config_ref.buf[i]);
	}
	
	//modify read_msg struct changing only the size and index requests
	read_msg.num_bytes = sizeof(device_config_t);
	read_msg.index = 0;	
	rc = create_read_frame(&read_msg, TYPE_SERIAL_MESSAGE, &controller_tx);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);

	rc = frame_to_payload(&controller_tx, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &pld_msg);	//decode frame to payload - confirmed in previous unit test so exhaustive coverage not required here
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);	
	rc = parse_general_message(&pld_msg, TYPE_SERIAL_MESSAGE, &motor_config_ref, &motor_tx);	//parse the decoded payload message
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);	
	TEST_ASSERT_EQUAL(MASTER_MISC_ADDRESS, motor_tx.buf[0]);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, validate_crc(&motor_tx));
	TEST_ASSERT_EQUAL(NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM + read_msg.num_bytes, motor_tx.len);
	rc = frame_to_payload(&motor_tx, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &pld_msg);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);	
	rc = parse_read_reply(&pld_msg, &read_msg, &controller_config_ref);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);	
	for(int i = 0; i < controller_config_ref.size; i++)
	{
		TEST_ASSERT_EQUAL(motor_config_ref.buf[i], controller_config_ref.buf[i]);
	}

}	

/*

*/
void test_struct_block_read_write(void)
{
	memset(&controller_config, 0, sizeof(controller_config));
	const unsigned char motor_address = 3;
	//create the write message
	misc_read_message_t read_msg = {
		.address = get_complementary_address(motor_address),
		.index = 0,
		.num_bytes = sizeof(device_config_t)
	};
	int rc = create_read_frame(&read_msg, TYPE_SERIAL_MESSAGE, &controller_tx);	//this goes straight from application to frame
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);

	payload_layer_msg_t pld = {};
	rc = frame_to_payload(&controller_tx, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &pld);
	TEST_ASSERT_EQUAL(get_complementary_address(motor_address), pld.address);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	rc = parse_general_message(&pld, TYPE_SERIAL_MESSAGE, &motor_config_ref, &motor_tx); //frame to app
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);

	rc = frame_to_payload(&motor_tx, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &pld);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	rc = parse_read_reply(&pld, &read_msg, &controller_config_ref);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	
	for(int i = 0; i < controller_config_ref.size; i++)
	{
		TEST_ASSERT_EQUAL(motor_config_ref.buf[i], controller_config_ref.buf[i]);
	}


	controller_config.acceleration++;
	controller_config.current_position--;
	controller_config.device_id++;
	controller_config.firmware_version--;
	controller_config.max_speed++;
	controller_config.position_target--;
	controller_config.status_flags++;
	controller_config.temperature--;
	rc = create_struct_write_frame(
		get_complementary_address(motor_address),
		&controller_config.acceleration,
		sizeof(uint32_t),
		&controller_config,
		&controller_tx
	);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);

	frame_to_payload(&controller_tx, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &pld);
	TEST_ASSERT_EQUAL(get_complementary_address(motor_address), pld.address);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	rc = parse_general_message(&pld, TYPE_SERIAL_MESSAGE, &motor_config_ref, &motor_tx);
	TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
	TEST_ASSERT_EQUAL(0, motor_tx.len);	//may change this? for now write confirmations are performed by reads
	TEST_ASSERT_EQUAL(motor_config.acceleration, controller_config.acceleration);


}
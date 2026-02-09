/*
    DARTT Protocol Demonstration
    
    This example shows how to use the DARTT protocol library functions
    with type-punning for direct struct access. This demonstrates the
    intended use case on little-endian 32-bit architectures like ARM STM32.
    
    WARNING: This code uses type-punning which is technically not C standard
    compliant, but works reliably on controlled embedded targets where:
    1. The target architecture is known and fixed
    2. The target is little-endian
    3. The compiler behavior is well-understood
    
    This approach eliminates the need for explicit parsing/serialization.
*/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include "dartt.h"

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
dartt_buffer_t motor_config_ref = {
	.buf = (unsigned char *)(&motor_config),
	.size = sizeof(device_config_t),
	.len = 0
};

// Simulated device memory (controller) - empty
static device_config_t controller_config = {};
dartt_buffer_t controller_config_ref = {
	.buf = (unsigned char *)&controller_config,
	.size = sizeof(device_config_t),
	.len = 0
};


//simulated tx buffer from the controller
unsigned char controller_tx_mem[64] = {};
dartt_buffer_t controller_tx = {
	.buf = controller_tx_mem,
	.size = sizeof(controller_tx_mem),
	.len = 0
};

//create a buffer for the motor tx/motor reply
unsigned char motor_tx_mem[64] = {};
dartt_buffer_t motor_tx = {
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
        DARTT_PROTOCOL_SUCCESS on success, error code on failure
*/
int create_struct_write_frame(unsigned char address, 
	unsigned char * field, 
	size_t field_size, 
	device_config_t * pstruct, 
	dartt_buffer_t * output_frame)
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
    return dartt_create_write_frame(&write_msg, TYPE_SERIAL_MESSAGE, output_frame);
}

int create_read_struct_frame(unsigned char address,
	 unsigned char * field,
	  size_t field_size,
	   device_config_t * pstruct, 
	   misc_read_message_t * read_msg_out,
	   dartt_buffer_t * output_frame)
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
	
	return dartt_create_read_frame(read_msg_out, TYPE_SERIAL_MESSAGE, output_frame);
}

/*
    Helper function to print device_config_t with nice formatting
*/
void print_device_config(device_config_t * config)
{
    printf("  device_id:        0x%08X\n", config->device_id);
    printf("  max_speed:        %u\n", config->max_speed);
    printf("  acceleration:     %u\n", config->acceleration);
    printf("  position_target:  %u\n", config->position_target);
    printf("  current_position: %u\n", config->current_position);
    printf("  status_flags:     0x%08X\n", config->status_flags);
    printf("  temperature:      %u\n", config->temperature);
    printf("  firmware_version: 0x%08X\n", config->firmware_version);
}

/*
    Helper function to print dartt_buffer_t with nice formatting
*/
void print_buffer(dartt_buffer_t * buffer)
{
    printf("Buffer (size=%zu, len=%zu): ", buffer->size, buffer->len);
    if (buffer->len == 0) {
        printf("(empty)\n");
    } else {
        for (size_t i = 0; i < buffer->len; i++) {
            printf("%02X ", buffer->buf[i]);
            if ((i + 1) % 16 == 0 && i < buffer->len - 1) {
                printf("\n                              ");
            }
        }
        printf("\n");
    }
}


int main(void)
{
    printf("DARTT Protocol (Dual-Address Real-Time Transport) Demonstration\n");
    printf("=============================================================\n");
    printf("Focus: TYPE_SERIAL_MESSAGE with struct field wrapper\n\n");
    
	printf("Example 1: controller block read");
	
	const unsigned char motor_address = 3;
	
	printf("Before: controller current position = %d\r\n", controller_config.current_position);
	printf("Controller config:\r\n");
	print_device_config(&controller_config);
    printf("Motor config: \r\n");
    print_device_config(&motor_config);
	printf("Create master tx frame\r\n");
	//create a DARTT frame to read the current position - using type-punning and application defined structs
	misc_read_message_t read_msg = {};
	int rc = create_read_struct_frame(dartt_get_complementary_address(motor_address),
		(unsigned char *)(&controller_config.current_position), 
		sizeof(controller_config.current_position),
		&controller_config,
		&read_msg,
		&controller_tx
	);
	printf("Message: ");
	print_buffer(&controller_tx);
	printf("Controller sends message to motor\r\n");
	
	printf("Motor recieved the message\r\n");
	payload_layer_msg_t pld_msg ={};	//can be statically allocated, or local and initialized to zero
	rc = dartt_frame_to_payload(&controller_tx, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &pld_msg);
	rc = dartt_parse_general_message(&pld_msg, TYPE_SERIAL_MESSAGE, &motor_config_ref, &motor_tx);
	printf("Motor parsed master message and sends reply\r\n");
	print_buffer(&motor_tx);

	printf("Controller recieved reply\r\n");
    rc = dartt_frame_to_payload(&motor_tx, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &pld_msg);
	rc = dartt_parse_read_reply(&pld_msg, &read_msg, &controller_config_ref);
	printf("After: controller current position = %d\r\n", controller_config.current_position);
	printf("Controller config:\r\n");
	print_device_config(&controller_config);
    printf("Motor config: \r\n");
    print_device_config(&motor_config);

	
    printf("\n=== Demo Complete ===\n");
    
    return 0;
}
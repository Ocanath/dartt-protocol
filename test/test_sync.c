
#include "checksum.h"
#include "dartt.h"
#include "dartt_sync.h"
#include "unity.h"
#include <string.h>
/**
 * TODO: Add test coverage for:
 * 1. periphbase and ctlbase size mismatch
 * 2. _sync, _write_multi and _read_multi with the following cases:
 * 		ctl points to somewhere in the middle of ctl_base (valid)
 * 		ctl points to somewhere in the middle of ctl_base and overruns ctl_base (return with errors)
 * 	Note - use wrapper test functions for this with inputs so we can reduce test code volume. I'd say 
 * 	use offset and size as input parameters for each wrapper, as well as message type - then we can enumerate
 * 	edge cases, msg types, etc. We can use the global flag to modify callback function behavior
 * 
 * */

typedef struct i32_t
{
	int32_t i32;
	int32_t radix;	//'decimal' point. true value is i32/2^radix
}i32_t;

//position control structure
typedef struct fixed_PI_2_params_t
{
	i32_t kp;
	i32_t ki;
	int32_t x_integral_div;
	int32_t x;
	int32_t x_sat;
	uint8_t out_rshift;
}fixed_PI_2_params_t;


typedef struct fds_t
{
    int32_t module_number;
    int32_t align_offset;
}fds_t;


typedef struct motor_params_t
{
    fixed_PI_2_params_t pi_vq;
    fds_t fds;
}motor_params_t;

typedef struct test_struct_t
{
    int32_t m1_set;
    int32_t m2_set;
    motor_params_t mp[32];
}test_struct_t;

uint8_t tx_mem[64] = {};
buffer_t * p_sync_tx_buf;
uint8_t rx_mem[64] = {};

int init_struct_buffer(test_struct_t * s, buffer_t * buf)
{
    buf->buf = (unsigned char *)s;
    buf->size = sizeof(test_struct_t);
    buf->len = 0;
}

int dartt_init_buffer(buffer_t * b, uint8_t * arr, size_t size)
{
    if(b == NULL || arr == NULL)
    {
        return ERROR_INVALID_ARGUMENT;
    }
    b->buf = (unsigned char *)arr;
    b->size = size;
    b->len = 0;
    return DARTT_PROTOCOL_SUCCESS;
}

//periph copy
test_struct_t gl_periph = {};
buffer_t periph_alias = 
{
    .buf = (unsigned char * )(&gl_periph),
    .size = sizeof(test_struct_t),
    .len = 0
};

int synctest_rx_blocking(buffer_t * rx, uint32_t timeout)
{
    //model peripheral with reply behavior and modifications to periph via alias
    payload_layer_msg_t rxpld_msg = {};
    dartt_frame_to_payload(p_sync_tx_buf, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &rxpld_msg);
    dartt_parse_general_message(&rxpld_msg, TYPE_SERIAL_MESSAGE, &periph_alias, rx);
    return DARTT_PROTOCOL_SUCCESS;
}

int synctest_rx_blocking_fdcan(buffer_t * rx, uint32_t timeout)
{
    //model peripheral with reply behavior and modifications to periph via alias
    payload_layer_msg_t rxpld_msg = {};
    dartt_frame_to_payload(p_sync_tx_buf, TYPE_ADDR_CRC_MESSAGE, PAYLOAD_ALIAS, &rxpld_msg);
    dartt_parse_general_message(&rxpld_msg, TYPE_ADDR_CRC_MESSAGE, &periph_alias, rx);
    return DARTT_PROTOCOL_SUCCESS;
}

serial_message_type_t gl_msg_type = TYPE_SERIAL_MESSAGE;
uint32_t gl_send_count = 0;    //flag to indicate to test software if tx is called. Zero before caller
int synctest_tx_blocking(unsigned char addr, buffer_t * tx, uint32_t timeout)
{
    // printf("transmitted: a = 0x%X, rx=0x");
    // for(int i = 0; i < tx->len; i++)
    // {
    //     printf("%0.2X", tx->buf[i]);
    // }
    // printf("\n");
    
    
    unsigned char tx_cpy[sizeof(tx_mem)] = {};
    buffer_t tx_cpy_alias = {.buf = tx_cpy, .size = sizeof(tx_cpy), .len=tx->len};
    for(int i = 0; i < tx->size; i++)
    {
        tx_cpy_alias.buf[i] = tx->buf[i];
    }
    payload_layer_msg_t rxpld_msg = {};
    dartt_frame_to_payload(&tx_cpy_alias, gl_msg_type, PAYLOAD_ALIAS, &rxpld_msg);
    dartt_parse_general_message(&rxpld_msg, gl_msg_type, &periph_alias, &tx_cpy_alias);    //pipe reply to tx, it's fine if we corrupt it with this call. It should pretty much just set the len to 0
    // printf("tx len = %d\n", tx->len);
    gl_send_count++;
    return DARTT_PROTOCOL_SUCCESS;
}


int synctest_tx_blocking_fdcan(unsigned char addr, buffer_t * tx, uint32_t timeout)
{
    // printf("transmitted: a = 0x%X, rx=0x");
    // for(int i = 0; i < tx->len; i++)
    // {
    //     printf("%0.2X", tx->buf[i]);
    // }
    // printf("\n");
    
    
    unsigned char tx_cpy[sizeof(tx_mem)] = {};
    buffer_t tx_cpy_alias = {.buf = tx_cpy, .size = sizeof(tx_cpy), .len=tx->len};
    for(int i = 0; i < tx->size; i++)
    {
        tx_cpy_alias.buf[i] = tx->buf[i];
    }
    payload_layer_msg_t rxpld_msg = {};
    dartt_frame_to_payload(&tx_cpy_alias, TYPE_ADDR_CRC_MESSAGE, PAYLOAD_ALIAS, &rxpld_msg);
    dartt_parse_general_message(&rxpld_msg, TYPE_ADDR_CRC_MESSAGE, &periph_alias, &tx_cpy_alias);    //pipe reply to tx, it's fine if we corrupt it with this call. It should pretty much just set the len to 0
    // printf("tx len = %d\n", tx->len);
    gl_send_count++;
    return DARTT_PROTOCOL_SUCCESS;
}


void test_dartt_sync_full(void)
{
    TEST_ASSERT_EQUAL(0, sizeof(test_struct_t)%sizeof(int32_t));//ensure struct is 32bit word aligned
    //master structs and aliases
    test_struct_t ctl_master = {};
    buffer_t ctl_master_alias;
    init_struct_buffer(&ctl_master, &ctl_master_alias);
    test_struct_t periph_master = {};
    //sync params
    dartt_sync_t ctl_sync = {};
    ctl_sync.address = 3;
    init_struct_buffer(&ctl_master, &ctl_sync.ctl_base);
	init_struct_buffer(&periph_master, &ctl_sync.periph_base);
    ctl_sync.msg_type = TYPE_SERIAL_MESSAGE;
    int rc = dartt_init_buffer(&ctl_sync.tx_buf, tx_mem, sizeof(tx_mem));
    TEST_ASSERT_EQUAL(0,rc);
    rc = dartt_init_buffer(&ctl_sync.rx_buf, rx_mem, sizeof(rx_mem));
    TEST_ASSERT_EQUAL(0,rc);
    ctl_sync.blocking_rx_callback = &synctest_rx_blocking;
	gl_msg_type = ctl_sync.msg_type;
    ctl_sync.blocking_tx_callback = &synctest_tx_blocking;
    ctl_sync.timeout_ms = 10;
    p_sync_tx_buf = &ctl_sync.tx_buf;   //for unit testing only - set up ref for us to make fake peripheral device in the callbacks
    //setup test structs
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        ctl_master_alias.buf[i] = (i % 254) + 1;
        ctl_sync.periph_base.buf[i] = ctl_master_alias.buf[i];
    }
    TEST_ASSERT_EQUAL(ctl_master_alias.size, ctl_sync.periph_base.size);
    TEST_ASSERT_EQUAL(ctl_master_alias.size, periph_alias.size);
    for(int i = 0; i < periph_alias.size; i++)
    {
        periph_alias.buf[i] = 0;
    }
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_NOT_EQUAL(ctl_master_alias.buf[i], periph_alias.buf[i]);
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], ctl_sync.periph_base.buf[i]);
    }

    ctl_master.m1_set = 10;
    ctl_master.mp[0].fds.align_offset=15;
    ctl_master.mp[1].pi_vq.ki.radix=7;
    ctl_master.mp[1].pi_vq.ki.i32 = 145;
    TEST_ASSERT_NOT_EQUAL(ctl_master.m1_set, periph_master.m1_set);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[0].fds.align_offset, periph_master.mp[0].fds.align_offset);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.ki.radix, periph_master.mp[1].pi_vq.ki.radix);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.ki.i32, periph_master.mp[1].pi_vq.ki.i32);
    TEST_ASSERT_EQUAL(sizeof(test_struct_t), ctl_master_alias.size);
    rc = dartt_sync(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(sizeof(test_struct_t), ctl_master_alias.size);
    TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], ctl_sync.periph_base.buf[i]);
    }   //these should match perfectly when sync runs - the behavior is that the when the master and shadow copy are out of sync, the peripheral is loaded and the shadow updated with a read from the peripheral
    /**/
    TEST_ASSERT_EQUAL(ctl_master.m1_set, gl_periph.m1_set);
    TEST_ASSERT_EQUAL(ctl_master.mp[0].fds.align_offset,gl_periph.mp[0].fds.align_offset);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].pi_vq.ki.radix,gl_periph.mp[1].pi_vq.ki.radix);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].pi_vq.ki.i32,gl_periph.mp[1].pi_vq.ki.i32);
    TEST_ASSERT_NOT_EQUAL(ctl_master.m2_set, gl_periph.m2_set);     //technically all but the 4 values changed should not match, but we'll just throw one in for basic demonstration
    for(int i = 0; i < periph_alias.size; i++)  //verify all in the true peripheral copy are initialized to zero except those touched by the sync call above
    {
        if(ctl_master_alias.buf[i] != periph_alias.buf[i])
        {
            TEST_ASSERT_EQUAL(0, periph_alias.buf[i]);
        }
    }

    //check the last word in the structure
    ctl_master.mp[1].fds.align_offset = 1234;
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].fds.align_offset, periph_master.mp[1].fds.align_offset);
    TEST_ASSERT_EQUAL(0, gl_periph.mp[1].fds.align_offset);//double check init to 0
    TEST_ASSERT_NOT_EQUAL(periph_master.mp[1].fds.align_offset, gl_periph.mp[1].fds.align_offset);
    rc = dartt_sync(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(0, rc);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].fds.align_offset, periph_master.mp[1].fds.align_offset);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].fds.align_offset, gl_periph.mp[1].fds.align_offset);


    //change the aliases to only target a small specific region, which should limit sync
    ctl_master_alias.buf = (unsigned char *)(&ctl_master.mp[0].pi_vq);
    ctl_master_alias.size = sizeof(fixed_PI_2_params_t);
    ctl_master_alias.len = 0;

    ctl_master.m2_set = 100;
    periph_master.m2_set = -50; //explicitly load these differently (they were already different but just for clarity)

    //load two of the targets, not at zero and non-adjacent
    ctl_master.mp[0].pi_vq.x = 124538;
    ctl_master.mp[0].pi_vq.out_rshift = 7;
    rc = dartt_sync(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(0, rc);
    TEST_ASSERT_EQUAL(-50, periph_master.m2_set);
    TEST_ASSERT_EQUAL(100, ctl_master.m2_set);
    TEST_ASSERT_EQUAL(ctl_master.mp[0].pi_vq.x, gl_periph.mp[0].pi_vq.x);
    TEST_ASSERT_EQUAL(ctl_master.mp[0].pi_vq.out_rshift, gl_periph.mp[0].pi_vq.out_rshift);
    

    //final sync test - make the control and shadow copies completely out of sync, triggering a complete buffer write.
    //since tx buf is small, this tests the logic of breaking up the tx buffer as well
    init_struct_buffer(&ctl_master, &ctl_master_alias); //reinit both alias buffers to have proper size
    init_struct_buffer(&periph_master, &ctl_sync.periph_base);
    //reset all memory (both master copies and peripheral copy to a test state for the full write attempt)
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        ctl_master_alias.buf[i] = (i % 254) + 1;
        ctl_sync.periph_base.buf[i] = ((ctl_master_alias.buf[i] + 1) % 254) + 1;
        periph_alias.buf[i] = 0;
        TEST_ASSERT_NOT_EQUAL(0, ctl_master_alias.buf[i]);
        TEST_ASSERT_NOT_EQUAL(0, ctl_sync.periph_base.buf[i]);
        TEST_ASSERT_NOT_EQUAL(ctl_master_alias.buf[i], ctl_sync.periph_base.buf[i]);
    }

    gl_send_count = 0;
    rc = dartt_sync(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(0, rc);
    TEST_ASSERT_GREATER_THAN(2, gl_send_count);//must be multiple - depends on size
    TEST_ASSERT_EQUAL(ctl_master_alias.size, sizeof(test_struct_t));
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], ctl_sync.periph_base.buf[i]);
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], periph_alias.buf[i]);
        TEST_ASSERT_NOT_EQUAL(0, periph_alias.buf[i]);
    }
}


void test_dartt_write(void)
{
    TEST_ASSERT_EQUAL(0, sizeof(test_struct_t)%sizeof(int32_t));//ensure struct is 32bit word aligned
    //master structs and aliases
    test_struct_t ctl_master = {};
    buffer_t ctl_master_alias;
    init_struct_buffer(&ctl_master, &ctl_master_alias);
    test_struct_t periph_master = {};
	buffer_t periph_master_alias;
	init_struct_buffer(&periph_master, &periph_master_alias);

    //sync params
    dartt_sync_t ctl_sync = {};
    ctl_sync.address = 3;
    init_struct_buffer(&ctl_master, &ctl_sync.ctl_base);
	init_struct_buffer(&periph_master, &ctl_sync.periph_base);
    ctl_sync.msg_type = TYPE_SERIAL_MESSAGE;
    int rc = dartt_init_buffer(&ctl_sync.tx_buf, tx_mem, sizeof(tx_mem));
    TEST_ASSERT_EQUAL(0,rc);
    rc = dartt_init_buffer(&ctl_sync.rx_buf, rx_mem, sizeof(rx_mem));
    TEST_ASSERT_EQUAL(0,rc);
    ctl_sync.blocking_rx_callback = &synctest_rx_blocking;
	gl_msg_type = ctl_sync.msg_type;
    ctl_sync.blocking_tx_callback = &synctest_tx_blocking;
    ctl_sync.timeout_ms = 10;
    p_sync_tx_buf = &ctl_sync.tx_buf;   //for unit testing only - set up ref for us to make fake peripheral device in the callbacks
    //setup test structs
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        ctl_master_alias.buf[i] = (i % 254) + 1;
        periph_master_alias.buf[i] = ctl_master_alias.buf[i];
    }
    TEST_ASSERT_EQUAL(ctl_master_alias.size, periph_master_alias.size);
    TEST_ASSERT_EQUAL(ctl_master_alias.size, periph_alias.size);
    for(int i = 0; i < periph_alias.size; i++)
    {
        periph_alias.buf[i] = 0;
    }
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_NOT_EQUAL(ctl_master_alias.buf[i], periph_alias.buf[i]);
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], periph_master_alias.buf[i]);
    }

    buffer_t motor_commands = {
        .buf = (unsigned char *)(&ctl_master.m1_set),
        .size = 2*sizeof(int32_t),  //(&ctl_master.m2_set+sizeof(int32_t)) - &ctl_master.m1_set
        .len = 2*sizeof(int32_t)
    };
    ctl_master.m1_set = 1234;
    ctl_master.m2_set = 5689;
    gl_send_count = 0;
    TEST_ASSERT_EQUAL(0, gl_periph.m1_set);
    TEST_ASSERT_EQUAL(0, gl_periph.m2_set);
    rc = dartt_ctl_write(&motor_commands, &ctl_sync);
    TEST_ASSERT_EQUAL(0, rc);
    TEST_ASSERT_EQUAL(1, gl_send_count);
    gl_send_count = 0;
    TEST_ASSERT_EQUAL(ctl_master.m1_set, gl_periph.m1_set);
    TEST_ASSERT_EQUAL(ctl_master.m2_set, gl_periph.m2_set);
    

    TEST_ASSERT_NOT_EQUAL(ctl_master.m1_set, periph_master.m1_set);
    TEST_ASSERT_NOT_EQUAL(ctl_master.m2_set, periph_master.m2_set);
    gl_send_count = 0;
    rc = dartt_ctl_read(&motor_commands, &ctl_sync);
    TEST_ASSERT_EQUAL(1, gl_send_count);
    TEST_ASSERT_EQUAL(0, rc);
    TEST_ASSERT_EQUAL(ctl_master.m1_set, periph_master.m1_set);
    TEST_ASSERT_EQUAL(ctl_master.m2_set, periph_master.m2_set);


    motor_commands.buf = (unsigned char *)(&ctl_master.mp[0].pi_vq.kp.i32);
    motor_commands.size = 4*sizeof(uint32_t);
    motor_commands.len = motor_commands.size;

    TEST_ASSERT_NOT_EQUAL(0, periph_master.mp[0].pi_vq.kp.i32);
    TEST_ASSERT_NOT_EQUAL(0, periph_master.mp[0].pi_vq.kp.radix);
    TEST_ASSERT_NOT_EQUAL(0, periph_master.mp[0].pi_vq.ki.i32);
    TEST_ASSERT_NOT_EQUAL(0, periph_master.mp[0].pi_vq.ki.radix);
    rc = dartt_ctl_read(&motor_commands, &ctl_sync);
    TEST_ASSERT_EQUAL(0, rc);
    TEST_ASSERT_EQUAL(0, periph_master.mp[0].pi_vq.kp.i32);
    TEST_ASSERT_EQUAL(0, periph_master.mp[0].pi_vq.kp.radix);
    TEST_ASSERT_EQUAL(0, periph_master.mp[0].pi_vq.ki.i32);
    TEST_ASSERT_EQUAL(0, periph_master.mp[0].pi_vq.ki.radix);


    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.kp.i32, gl_periph.mp[1].pi_vq.kp.i32);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.kp.radix, gl_periph.mp[1].pi_vq.kp.radix);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.kp.i32, gl_periph.mp[1].pi_vq.kp.i32);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.kp.radix, gl_periph.mp[1].pi_vq.kp.radix);
    motor_commands.buf = (unsigned char *)(&ctl_master.mp[1].pi_vq.kp.i32);
    motor_commands.size = 4*sizeof(uint32_t);
    motor_commands.len = motor_commands.size;
    rc = dartt_ctl_write(&motor_commands, &ctl_sync);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].pi_vq.kp.i32, gl_periph.mp[1].pi_vq.kp.i32);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].pi_vq.kp.radix, gl_periph.mp[1].pi_vq.kp.radix);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].pi_vq.kp.i32, gl_periph.mp[1].pi_vq.kp.i32);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].pi_vq.kp.radix, gl_periph.mp[1].pi_vq.kp.radix);
    
    periph_master.mp[1].pi_vq.kp.i32 = 0;
    periph_master.mp[1].pi_vq.kp.radix = 0;
    periph_master.mp[1].pi_vq.ki.i32 = 0;
    periph_master.mp[1].pi_vq.ki.radix = 0;
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.kp.i32, periph_master.mp[1].pi_vq.kp.i32);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.kp.radix, periph_master.mp[1].pi_vq.kp.radix);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.kp.i32, periph_master.mp[1].pi_vq.kp.i32);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.kp.radix, periph_master.mp[1].pi_vq.kp.radix);    
    rc = dartt_ctl_read(&motor_commands, &ctl_sync);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].pi_vq.kp.i32, periph_master.mp[1].pi_vq.kp.i32);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].pi_vq.kp.radix, periph_master.mp[1].pi_vq.kp.radix);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].pi_vq.kp.i32, periph_master.mp[1].pi_vq.kp.i32);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].pi_vq.kp.radix, periph_master.mp[1].pi_vq.kp.radix);    

}


void test_bad_inputs(void)
{

    buffer_t b1, b2, b3, b4;

    uint8_t b1_mem[4] = {};
    uint8_t b2_mem[4] = {};
    uint8_t b3_mem[4] = {};
    uint8_t b4_mem[4] = {};
    dartt_init_buffer(&b1, b1_mem, sizeof(b1_mem));
    dartt_init_buffer(&b4, b4_mem, sizeof(b4_mem));
    dartt_sync_t ds = {};
    ds.blocking_rx_callback = &synctest_rx_blocking;
	gl_msg_type = TYPE_SERIAL_MESSAGE;
    ds.blocking_tx_callback = &synctest_tx_blocking;
    dartt_init_buffer(&ds.tx_buf, tx_mem, sizeof(tx_mem));
    dartt_init_buffer(&ds.rx_buf, tx_mem, sizeof(tx_mem));
    dartt_init_buffer(&ds.ctl_base, b3_mem, sizeof(b3_mem));
	dartt_init_buffer(&ds.periph_base, b2_mem, sizeof(b2_mem));
    int rc = dartt_sync(&b1, &ds);
    TEST_ASSERT_NOT_EQUAL(0, rc);
    dartt_init_buffer(&ds.ctl_base, b1_mem, sizeof(b1_mem));
    rc = dartt_sync(&b1, &ds);
    TEST_ASSERT_EQUAL(0, rc);
}

void test_undersized_tx_buffers(void)
{
    test_struct_t ctl_master = {};
    test_struct_t periph_master = {};
    buffer_t ctl_alias;
    init_struct_buffer(&ctl_master, &ctl_alias);
    

    dartt_sync_t ctl_sync = {};
    ctl_sync.address = 3;
    ctl_sync.ctl_base = ctl_alias;
	init_struct_buffer(&periph_master, &ctl_sync.periph_base);
    ctl_sync.msg_type = TYPE_SERIAL_MESSAGE;
    ctl_sync.blocking_rx_callback = &synctest_rx_blocking;
	gl_msg_type = ctl_sync.msg_type;
    ctl_sync.blocking_tx_callback = &synctest_tx_blocking;
    ctl_sync.timeout_ms = 10;

    // Test 1: Extremely undersized buffer (1 byte) - should fail immediately
    uint8_t tiny_tx_mem[1] = {};
    uint8_t tiny_rx_mem[64] = {};
    dartt_init_buffer(&ctl_sync.tx_buf, tiny_tx_mem, sizeof(tiny_tx_mem));
    dartt_init_buffer(&ctl_sync.rx_buf, tiny_rx_mem, sizeof(tiny_rx_mem));

    // Make buffers different to trigger sync attempt
    ctl_master.m1_set = 100;
    periph_master.m1_set = 0;

    int rc = dartt_sync(&ctl_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);

    // Test 2: Buffer exactly at minimum non-payload size (5 bytes) - should fail
    uint8_t min_tx_mem[5] = {};
    dartt_init_buffer(&ctl_sync.tx_buf, min_tx_mem, sizeof(min_tx_mem));

    rc = dartt_sync(&ctl_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);

    // Test 3: Buffer with 4 bytes (less than minimum) - should fail
    uint8_t small_tx_mem[4] = {};
    dartt_init_buffer(&ctl_sync.tx_buf, small_tx_mem, sizeof(small_tx_mem));

    rc = dartt_sync(&ctl_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);
}

void test_minimum_sized_tx_buffers(void)
{
    test_struct_t ctl_master = {};
    test_struct_t periph_master = {};
    buffer_t ctl_alias;
    init_struct_buffer(&ctl_master, &ctl_alias);
    

    dartt_sync_t ctl_sync = {};
    ctl_sync.address = 3;
    ctl_sync.ctl_base = ctl_alias;
	init_struct_buffer(&periph_master, &ctl_sync.periph_base);
    ctl_sync.msg_type = TYPE_SERIAL_MESSAGE;
    ctl_sync.blocking_rx_callback = &synctest_rx_blocking;
	gl_msg_type = ctl_sync.msg_type;
    ctl_sync.blocking_tx_callback = &synctest_tx_blocking;
    ctl_sync.timeout_ms = 10;
    p_sync_tx_buf = &ctl_sync.tx_buf;

    uint8_t rx_mem_local[64] = {};
    dartt_init_buffer(&ctl_sync.rx_buf, rx_mem_local, sizeof(rx_mem_local));

    // Test 1: Buffer with exactly 6 bytes (5 + 1 payload byte) - should fail for 4-byte payload
    uint8_t min6_tx_mem[6] = {};
    dartt_init_buffer(&ctl_sync.tx_buf, min6_tx_mem, sizeof(min6_tx_mem));

    // Try to sync a 4-byte field (int32_t) - should fail due to insufficient space
    ctl_master.m1_set = 100;
    periph_master.m1_set = 0;

    int rc = dartt_sync(&ctl_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);

    // Test 2: Buffer with exactly 9 bytes (5 + 4 payload bytes) - should work for single int32_t
    uint8_t min9_tx_mem[9] = {};
    dartt_init_buffer(&ctl_sync.tx_buf, min9_tx_mem, sizeof(min9_tx_mem));

    rc = dartt_sync(&ctl_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
    TEST_ASSERT_EQUAL(ctl_master.m1_set, periph_master.m1_set);

    // Test 3: Test buffer with 10 bytes - should handle single field but fail with adjacent changes
    uint8_t tx10_mem[10] = {};
    dartt_init_buffer(&ctl_sync.tx_buf, tx10_mem, sizeof(tx10_mem));

    // Reset and change two adjacent int32_t fields
    ctl_master.m1_set = 200;
    ctl_master.m2_set = 300;
    periph_master.m1_set = 0;
    periph_master.m2_set = 0;

    gl_send_count = 0;
    rc = dartt_sync(&ctl_alias, &ctl_sync);
    // Should succeed but may require multiple transmissions due to buffer splitting
    TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
    TEST_ASSERT_EQUAL(4, gl_send_count);
    TEST_ASSERT_EQUAL(ctl_master.m1_set, periph_master.m1_set);
    TEST_ASSERT_EQUAL(ctl_master.m2_set, periph_master.m2_set);
}

void test_tx_buffer_edge_cases(void)
{
    test_struct_t ctl_master = {};
    test_struct_t periph_master = {};
	buffer_t ctl_alias;
    init_struct_buffer(&ctl_master, &ctl_alias);

    dartt_sync_t ctl_sync = {};
    ctl_sync.address = 3;
	init_struct_buffer(&ctl_master, &ctl_sync.ctl_base);
	init_struct_buffer(&periph_master, &ctl_sync.periph_base);
    ctl_sync.msg_type = TYPE_SERIAL_MESSAGE;
    ctl_sync.blocking_rx_callback = &synctest_rx_blocking;
	gl_msg_type = ctl_sync.msg_type;
    ctl_sync.blocking_tx_callback = &synctest_tx_blocking;
    ctl_sync.timeout_ms = 10;
    p_sync_tx_buf = &ctl_sync.tx_buf;

    uint8_t rx_mem_local[64] = {};
    dartt_init_buffer(&ctl_sync.rx_buf, rx_mem_local, sizeof(rx_mem_local));

    // Test 1: Buffer exactly fitting one 32-bit word (9 bytes: 5 overhead + 4 payload)
    uint8_t exact_tx_mem[9] = {};
    dartt_init_buffer(&ctl_sync.tx_buf, exact_tx_mem, sizeof(exact_tx_mem));

    // Change only the first field
    ctl_master.m1_set = 123;
    periph_master.m1_set = 0;

    gl_send_count = 0;
    int rc = dartt_sync(&ctl_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
    TEST_ASSERT_EQUAL(ctl_master.m1_set, periph_master.m1_set);

    // Test 2: Non-adjacent changes forcing multiple transmissions
    uint8_t small_tx_mem[13] = {}; // 5 + 8 bytes (can fit 2 int32_t but not more)
    dartt_init_buffer(&ctl_sync.tx_buf, small_tx_mem, sizeof(small_tx_mem));

    // Reset state
    for(int i = 0; i < ctl_alias.size; i++) 
    {
        ctl_alias.buf[i] = 0;
        periph_alias.buf[i] = 0;
		ctl_sync.periph_base.buf[i] = 0;
    }

    // Change non-adjacent fields that would require more buffer space if transmitted together
    ctl_master.m2_set = 111;
    ctl_master.mp[0].pi_vq.kp.i32 = 777;
    ctl_master.mp[0].fds.module_number = 222;
    ctl_master.mp[1].fds.align_offset = 333;

    TEST_ASSERT_NOT_EQUAL(ctl_master.m2_set, periph_master.m2_set);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[0].pi_vq.kp.i32, periph_master.mp[0].pi_vq.kp.i32);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[0].fds.module_number, periph_master.mp[0].fds.module_number);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].fds.align_offset, periph_master.mp[1].fds.align_offset);
    gl_send_count = 0;
    rc = dartt_sync(&ctl_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
    // Should require multiple transmissions due to buffer splitting logic
    TEST_ASSERT_EQUAL(6, gl_send_count);	//dartt sync calls the tx callback twice per messge sent - once for write, once again for read confirmation
    // Verify all changes were applied
    TEST_ASSERT_EQUAL(ctl_master.m2_set, periph_master.m2_set);
    TEST_ASSERT_EQUAL(ctl_master.mp[0].pi_vq.kp.i32, periph_master.mp[0].pi_vq.kp.i32);
    TEST_ASSERT_EQUAL(ctl_master.mp[0].fds.module_number, periph_master.mp[0].fds.module_number);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].fds.align_offset, periph_master.mp[1].fds.align_offset);
    
}

void test_buffer_alignment_edge_cases(void)
{
    // Test with buffers that are not 32-bit aligned in size
    uint8_t ctl_mem[7] = {}; // 7 bytes - not aligned to 4-byte boundary
    uint8_t periph_mem[7] = {};

    buffer_t ctl_buf = {.buf = ctl_mem, .size = sizeof(ctl_mem), .len = 0};

    dartt_sync_t ctl_sync = {};
    ctl_sync.address = 3;
    ctl_sync.ctl_base = ctl_buf;
	//init shadow copy buffer
	ctl_sync.periph_base.buf = periph_mem;
	ctl_sync.periph_base.size = sizeof(periph_mem);
	ctl_sync.periph_base.len = 0;

    ctl_sync.msg_type = TYPE_SERIAL_MESSAGE;
    ctl_sync.blocking_rx_callback = &synctest_rx_blocking;
	gl_msg_type = ctl_sync.msg_type;
    ctl_sync.blocking_tx_callback = &synctest_tx_blocking;
    ctl_sync.timeout_ms = 10;

    uint8_t tx_mem_local[64] = {};
    uint8_t rx_mem_local[64] = {};
    dartt_init_buffer(&ctl_sync.tx_buf, tx_mem_local, sizeof(tx_mem_local));
    dartt_init_buffer(&ctl_sync.rx_buf, rx_mem_local, sizeof(rx_mem_local));
    p_sync_tx_buf = &ctl_sync.tx_buf;

    // This should fail due to non-32-bit alignment
    int rc = dartt_sync(&ctl_buf, &ctl_sync);
    TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);

    // Test with 8-byte aligned buffer (should work)
    uint8_t ctl_mem8[8] = {};
    uint8_t periph_mem8[8] = {};

    buffer_t ctl_buf8 = {.buf = ctl_mem8, .size = sizeof(ctl_mem8), .len = sizeof(ctl_mem8)};
	buffer_t periph_buf8 = {.buf = periph_mem8, .size = sizeof(periph_mem8), .len = sizeof(periph_mem8)};
	
    // Make them different to trigger sync
    ctl_mem8[0] = 1;
    periph_mem8[0] = 0;

    rc = dartt_sync(&ctl_buf8, &ctl_sync);	
    TEST_ASSERT_EQUAL(ERROR_INVALID_ARGUMENT, rc);	//we reassigned the ctl pointer without reassigning the base - this is invalid argument (or potentially memory overrun) error, so return with a code

	ctl_sync.ctl_base = ctl_buf8;
	rc = dartt_sync(&ctl_buf8, &ctl_sync);	
    TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);	//we reassigned the ctl pointer without reassigning the base - this is invalid argument (or potentially memory overrun) error, so return with a code

	ctl_sync.periph_base = periph_buf8;

	rc = dartt_sync(&ctl_buf8, &ctl_sync);	
    TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);	//we reassigned the ctl pointer without reassigning the base - this is invalid argument (or potentially memory overrun) error, so return with a code

}


void test_dartt_read_multi(void)
{
    TEST_ASSERT_EQUAL(0, sizeof(test_struct_t)%sizeof(int32_t));//ensure struct is 32bit word aligned
    //master structs and aliases
    test_struct_t ctl_master = {};
    buffer_t ctl_master_alias;
    init_struct_buffer(&ctl_master, &ctl_master_alias);
    test_struct_t periph_master = {};
    buffer_t periph_master_alias;
    init_struct_buffer(&periph_master, &periph_master_alias);
    //sync params
    dartt_sync_t ctl_sync = {};
    ctl_sync.address = 3;
    init_struct_buffer(&ctl_master, &ctl_sync.ctl_base);
	init_struct_buffer(&periph_master, &ctl_sync.periph_base);
    ctl_sync.msg_type = TYPE_SERIAL_MESSAGE;
    int rc = dartt_init_buffer(&ctl_sync.tx_buf, tx_mem, sizeof(tx_mem));
    TEST_ASSERT_EQUAL(0,rc);
    rc = dartt_init_buffer(&ctl_sync.rx_buf, rx_mem, sizeof(rx_mem));
    TEST_ASSERT_EQUAL(0,rc);
    ctl_sync.blocking_rx_callback = &synctest_rx_blocking;
	gl_msg_type = ctl_sync.msg_type;
    ctl_sync.blocking_tx_callback = &synctest_tx_blocking;
    ctl_sync.timeout_ms = 10;
    p_sync_tx_buf = &ctl_sync.tx_buf;   //for unit testing only - set up ref for us to make fake peripheral device in the callbacks
    //setup test structs
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        ctl_master_alias.buf[i] = (i % 254) + 1;
        periph_master_alias.buf[i] = ctl_master_alias.buf[i];
    }
    TEST_ASSERT_EQUAL(ctl_master_alias.size, periph_master_alias.size);
    TEST_ASSERT_EQUAL(ctl_master_alias.size, periph_alias.size);
    for(int i = 0; i < periph_alias.size; i++)
    {
        periph_alias.buf[i] = 0;
    }
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_NOT_EQUAL(ctl_master_alias.buf[i], periph_alias.buf[i]);
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], periph_master_alias.buf[i]);
        TEST_ASSERT_NOT_EQUAL(0, periph_master_alias.buf[i]);
    }
    
    TEST_ASSERT_LESS_THAN(ctl_master_alias.size, ctl_sync.rx_buf.size); //must be true for the test to function properly
    ctl_master_alias.len = ctl_master_alias.size;   //indicate we want to read the full memory
    rc = dartt_ctl_read(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);   //because the transmit buffer is much smaller than the data we're trying to read, it should fail with a code (memory overrun)

    rc = dartt_read_multi(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(0, rc);
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_EQUAL(periph_alias.buf[i], periph_master_alias.buf[i]);
    }

    gl_periph.m1_set = 1234;
    gl_periph.m2_set = 4321;
    gl_periph.mp[31].fds.align_offset = -24;
    rc = dartt_read_multi(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(0, rc);
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_EQUAL(periph_alias.buf[i], periph_master_alias.buf[i]);
    }


}







void test_dartt_read_multi_fdcan(void)
{
    TEST_ASSERT_EQUAL(0, sizeof(test_struct_t)%sizeof(int32_t));//ensure struct is 32bit word aligned
    //master structs and aliases
    test_struct_t ctl_master = {};
    buffer_t ctl_master_alias;
    init_struct_buffer(&ctl_master, &ctl_master_alias);
    test_struct_t periph_master = {};
    buffer_t periph_master_alias;
    init_struct_buffer(&periph_master, &periph_master_alias);
    //sync params
    dartt_sync_t ctl_sync = {};
    ctl_sync.address = 3;
    init_struct_buffer(&ctl_master, &ctl_sync.ctl_base);
	init_struct_buffer(&periph_master, &ctl_sync.periph_base);
    ctl_sync.msg_type = TYPE_ADDR_CRC_MESSAGE;
    int rc = dartt_init_buffer(&ctl_sync.tx_buf, tx_mem, 8);
    TEST_ASSERT_EQUAL(0,rc);
    rc = dartt_init_buffer(&ctl_sync.rx_buf, rx_mem, 8);
    TEST_ASSERT_EQUAL(0,rc);
    ctl_sync.blocking_rx_callback = &synctest_rx_blocking_fdcan;
	gl_msg_type = ctl_sync.msg_type;
    ctl_sync.blocking_tx_callback = &synctest_tx_blocking_fdcan;
    ctl_sync.timeout_ms = 10;
    p_sync_tx_buf = &ctl_sync.tx_buf;   //for unit testing only - set up ref for us to make fake peripheral device in the callbacks
    //setup test structs
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        ctl_master_alias.buf[i] = (i % 254) + 1;
        periph_master_alias.buf[i] = ctl_master_alias.buf[i];
    }
    TEST_ASSERT_EQUAL(ctl_master_alias.size, periph_master_alias.size);
    TEST_ASSERT_EQUAL(ctl_master_alias.size, periph_alias.size);
    for(int i = 0; i < periph_alias.size; i++)
    {
        periph_alias.buf[i] = 0;
    }
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_NOT_EQUAL(ctl_master_alias.buf[i], periph_alias.buf[i]);
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], periph_master_alias.buf[i]);
        TEST_ASSERT_NOT_EQUAL(0, periph_master_alias.buf[i]);
    }
    
    TEST_ASSERT_LESS_THAN(ctl_master_alias.size, ctl_sync.rx_buf.size); //must be true for the test to function properly
    ctl_master_alias.len = ctl_master_alias.size;   //indicate we want to read the full memory
    rc = dartt_ctl_read(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);   //because the transmit buffer is much smaller than the data we're trying to read, it should fail with a code (memory overrun)

    rc = dartt_read_multi(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(0, rc);
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_EQUAL(periph_alias.buf[i], periph_master_alias.buf[i]);
    }

    gl_periph.m1_set = 1234;
    gl_periph.m2_set = 4321;
    gl_periph.mp[31].fds.align_offset = -24;
    rc = dartt_read_multi(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(0, rc);
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_EQUAL(periph_alias.buf[i], periph_master_alias.buf[i]);
    }


}



















void test_dartt_sync_full_fdcan(void)
{
    TEST_ASSERT_EQUAL(0, sizeof(test_struct_t)%sizeof(int32_t));//ensure struct is 32bit word aligned
    //master structs and aliases
    test_struct_t ctl_master = {};
    buffer_t ctl_master_alias;
    init_struct_buffer(&ctl_master, &ctl_master_alias);
    test_struct_t periph_master = {};
    buffer_t periph_master_alias;
    init_struct_buffer(&periph_master, &periph_master_alias);
    //sync params
    dartt_sync_t ctl_sync = {};
    ctl_sync.address = 3;
    init_struct_buffer(&ctl_master, &ctl_sync.ctl_base);
	init_struct_buffer(&periph_master, &ctl_sync.periph_base);
    ctl_sync.msg_type = TYPE_ADDR_CRC_MESSAGE;
    int rc = dartt_init_buffer(&ctl_sync.tx_buf, tx_mem, 8);
    TEST_ASSERT_EQUAL(0,rc);
    rc = dartt_init_buffer(&ctl_sync.rx_buf, rx_mem, 8);
    TEST_ASSERT_EQUAL(0,rc);
    ctl_sync.blocking_rx_callback = &synctest_rx_blocking_fdcan;
	gl_msg_type = ctl_sync.msg_type;
    ctl_sync.blocking_tx_callback = &synctest_tx_blocking_fdcan;
    ctl_sync.timeout_ms = 10;
    p_sync_tx_buf = &ctl_sync.tx_buf;   //for unit testing only - set up ref for us to make fake peripheral device in the callbacks
    //setup test structs
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        ctl_master_alias.buf[i] = (i % 254) + 1;
        periph_master_alias.buf[i] = ctl_master_alias.buf[i];
    }
    TEST_ASSERT_EQUAL(ctl_master_alias.size, periph_master_alias.size);
    TEST_ASSERT_EQUAL(ctl_master_alias.size, periph_alias.size);
    for(int i = 0; i < periph_alias.size; i++)
    {
        periph_alias.buf[i] = 0;
    }
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_NOT_EQUAL(ctl_master_alias.buf[i], periph_alias.buf[i]);
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], periph_master_alias.buf[i]);
    }

    ctl_master.m1_set = 10;
    ctl_master.mp[0].fds.align_offset=15;
    ctl_master.mp[1].pi_vq.ki.radix=7;
    ctl_master.mp[1].pi_vq.ki.i32 = 145;
    TEST_ASSERT_NOT_EQUAL(ctl_master.m1_set, periph_master.m1_set);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[0].fds.align_offset, periph_master.mp[0].fds.align_offset);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.ki.radix, periph_master.mp[1].pi_vq.ki.radix);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.ki.i32, periph_master.mp[1].pi_vq.ki.i32);
    TEST_ASSERT_EQUAL(sizeof(test_struct_t), ctl_master_alias.size);
    rc = dartt_sync(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(sizeof(test_struct_t), ctl_master_alias.size);
    TEST_ASSERT_EQUAL(DARTT_PROTOCOL_SUCCESS, rc);
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], periph_master_alias.buf[i]);
    }   //these should match perfectly when sync runs - the behavior is that the when the master and shadow copy are out of sync, the peripheral is loaded and the shadow updated with a read from the peripheral
    /**/
    TEST_ASSERT_EQUAL(ctl_master.m1_set, gl_periph.m1_set);
    TEST_ASSERT_EQUAL(ctl_master.mp[0].fds.align_offset,gl_periph.mp[0].fds.align_offset);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].pi_vq.ki.radix,gl_periph.mp[1].pi_vq.ki.radix);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].pi_vq.ki.i32,gl_periph.mp[1].pi_vq.ki.i32);
    TEST_ASSERT_NOT_EQUAL(ctl_master.m2_set, gl_periph.m2_set);     //technically all but the 4 values changed should not match, but we'll just throw one in for basic demonstration
    for(int i = 0; i < periph_alias.size; i++)  //verify all in the true peripheral copy are initialized to zero except those touched by the sync call above
    {
        if(ctl_master_alias.buf[i] != periph_alias.buf[i])
        {
            TEST_ASSERT_EQUAL(0, periph_alias.buf[i]);
        }
    }

    //check the last word in the structure
    ctl_master.mp[1].fds.align_offset = 1234;
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].fds.align_offset, periph_master.mp[1].fds.align_offset);
    TEST_ASSERT_EQUAL(0, gl_periph.mp[1].fds.align_offset);//double check init to 0
    TEST_ASSERT_NOT_EQUAL(periph_master.mp[1].fds.align_offset, gl_periph.mp[1].fds.align_offset);
    rc = dartt_sync(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(0, rc);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].fds.align_offset, periph_master.mp[1].fds.align_offset);
    TEST_ASSERT_EQUAL(ctl_master.mp[1].fds.align_offset, gl_periph.mp[1].fds.align_offset);


    //change the aliases to only target a small specific region, which should limit sync
    ctl_master_alias.buf = (unsigned char *)(&ctl_master.mp[0].pi_vq);
    ctl_master_alias.size = sizeof(fixed_PI_2_params_t);
    ctl_master_alias.len = 0;
    periph_master_alias.buf = (unsigned char *)(&periph_master.mp[0].pi_vq);
    periph_master_alias.size = sizeof(fixed_PI_2_params_t);
    periph_master_alias.len = 0;

    ctl_master.m2_set = 100;
    periph_master.m2_set = -50; //explicitly load these differently (they were already different but just for clarity)

    //load two of the targets, not at zero and non-adjacent
    ctl_master.mp[0].pi_vq.x = 124538;
    ctl_master.mp[0].pi_vq.out_rshift = 7;
    rc = dartt_sync(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(0, rc);
    TEST_ASSERT_EQUAL(-50, periph_master.m2_set);
    TEST_ASSERT_EQUAL(100, ctl_master.m2_set);
    TEST_ASSERT_EQUAL(ctl_master.mp[0].pi_vq.x, gl_periph.mp[0].pi_vq.x);
    TEST_ASSERT_EQUAL(ctl_master.mp[0].pi_vq.out_rshift, gl_periph.mp[0].pi_vq.out_rshift);
    

    //final sync test - make the control and shadow copies completely out of sync, triggering a complete buffer write.
    //since tx buf is small, this tests the logic of breaking up the tx buffer as well
    init_struct_buffer(&ctl_master, &ctl_master_alias); //reinit both alias buffers to have proper size
    init_struct_buffer(&periph_master, &periph_master_alias);
    //reset all memory (both master copies and peripheral copy to a test state for the full write attempt)
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        ctl_master_alias.buf[i] = (i % 254) + 1;
        periph_master_alias.buf[i] = ((ctl_master_alias.buf[i] + 1) % 254) + 1;
        periph_alias.buf[i] = 0;
        TEST_ASSERT_NOT_EQUAL(0, ctl_master_alias.buf[i]);
        TEST_ASSERT_NOT_EQUAL(0, periph_master_alias.buf[i]);
        TEST_ASSERT_NOT_EQUAL(ctl_master_alias.buf[i], periph_master_alias.buf[i]);
    }

    gl_send_count = 0;
    rc = dartt_sync(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(0, rc);
    TEST_ASSERT_GREATER_THAN(2, gl_send_count);//must be multiple - depends on size
    TEST_ASSERT_EQUAL(ctl_master_alias.size, sizeof(test_struct_t));
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], periph_master_alias.buf[i]);
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], periph_alias.buf[i]);
        TEST_ASSERT_NOT_EQUAL(0, periph_alias.buf[i]);
    }
}







void dartt_write_multi_wrapper(int rbufsize, int tbufsize, serial_message_type_t type)
{
    TEST_ASSERT_EQUAL(0, sizeof(test_struct_t)%sizeof(int32_t));//ensure struct is 32bit word aligned
    //master structs and aliases
    test_struct_t ctl_master = {};
    buffer_t ctl_master_alias;
    init_struct_buffer(&ctl_master, &ctl_master_alias);
    test_struct_t periph_master = {};
    buffer_t periph_master_alias;
    init_struct_buffer(&periph_master, &periph_master_alias);
    //sync params
    dartt_sync_t ctl_sync = {};
    ctl_sync.address = 3;
    init_struct_buffer(&ctl_master, &ctl_sync.ctl_base);
	init_struct_buffer(&periph_master, &ctl_sync.periph_base);
    ctl_sync.msg_type = type;
    int rc = dartt_init_buffer(&ctl_sync.tx_buf, tx_mem, tbufsize);
    TEST_ASSERT_EQUAL(0,rc);
    rc = dartt_init_buffer(&ctl_sync.rx_buf, rx_mem, rbufsize);
    TEST_ASSERT_EQUAL(0,rc);
    ctl_sync.blocking_rx_callback = &synctest_rx_blocking;
	gl_msg_type = ctl_sync.msg_type;
    ctl_sync.blocking_tx_callback = &synctest_tx_blocking;
    ctl_sync.timeout_ms = 10;
    p_sync_tx_buf = &ctl_sync.tx_buf;   //for unit testing only - set up ref for us to make fake peripheral device in the callbacks
    //setup test structs
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        ctl_master_alias.buf[i] = (i % 254) + 1;
        periph_master_alias.buf[i] = ctl_master_alias.buf[i];
    }
    TEST_ASSERT_EQUAL(ctl_master_alias.size, periph_master_alias.size);
    TEST_ASSERT_EQUAL(ctl_master_alias.size, periph_alias.size);
    for(int i = 0; i < periph_alias.size; i++)
    {
        periph_alias.buf[i] = 0;
    }
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_NOT_EQUAL(ctl_master_alias.buf[i], periph_alias.buf[i]);
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], periph_master_alias.buf[i]);
        TEST_ASSERT_NOT_EQUAL(0, periph_master_alias.buf[i]);
    }
    
    TEST_ASSERT_LESS_THAN(ctl_master_alias.size, ctl_sync.tx_buf.size); //verify that write multi is actually going to write multi
    ctl_master_alias.len = ctl_master_alias.size;   //indicate we want to read the full memory
    rc = dartt_ctl_write(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);   //because the transmit buffer is much smaller than the data we're trying to read, it should fail with a code (memory overrun)

    rc = dartt_write_multi(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(0, rc);
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], periph_alias.buf[i]);
    }
	ctl_master.m1_set = 5646;
    ctl_master.m2_set = -1415;
    ctl_master.mp[31].fds.align_offset = -24;
	ctl_master.mp[24].fds.module_number = 1;
    rc = dartt_write_multi(&ctl_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(0, rc);
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], periph_alias.buf[i]);
    }
}


void test_dartt_write_multi(void)
{
	dartt_write_multi_wrapper(sizeof(rx_mem), sizeof(tx_mem), TYPE_SERIAL_MESSAGE);
	dartt_write_multi_wrapper(sizeof(rx_mem), sizeof(tx_mem), TYPE_ADDR_MESSAGE);
	dartt_write_multi_wrapper(sizeof(rx_mem), sizeof(tx_mem), TYPE_ADDR_CRC_MESSAGE);
	dartt_write_multi_wrapper(8, 8, TYPE_ADDR_CRC_MESSAGE);
}

//initialize memory
test_struct_t gl_master_copy;
test_struct_t gl_shadow_copy;
unsigned char gl_tx_buffer[sizeof(gl_master_copy)+NUM_BYTES_NON_PAYLOAD] = {0};
unsigned char gl_rx_buffer[sizeof(gl_master_copy)+NUM_BYTES_NON_PAYLOAD] = {0};
//initialize sync
dartt_sync_t gl_ds;

void init_gl_ds(void)
{
	gl_ds.address = 0x3;
	init_struct_buffer(&gl_master_copy, &gl_ds.ctl_base);
	init_struct_buffer(&gl_shadow_copy, &gl_ds.periph_base);
	gl_ds.msg_type = TYPE_SERIAL_MESSAGE;	//ignored by this function
	dartt_init_buffer(&gl_ds.tx_buf, gl_tx_buffer, sizeof(gl_tx_buffer));
	dartt_init_buffer(&gl_ds.rx_buf, gl_rx_buffer, sizeof(gl_rx_buffer));
	gl_ds.blocking_rx_callback = &synctest_rx_blocking;
	gl_ds.blocking_tx_callback = &synctest_tx_blocking;
	gl_ds.timeout_ms = 5;
	TEST_ASSERT_EQUAL(gl_ds.periph_base.size, periph_alias.size);
	TEST_ASSERT_EQUAL(gl_ds.ctl_base.size, periph_alias.size);
	for(int i = 0; i < gl_ds.ctl_base.size; i++)
	{
		gl_ds.ctl_base.buf[i] = 0;
		gl_ds.periph_base.buf[i] = 0;
		periph_alias.buf[i] = 0;
	}
}

void scramble_buffers(void)
{
	for(int i = 0; i < gl_ds.ctl_base.size; i++)
	{
		gl_ds.ctl_base.buf[i] = (i % 255) + 1;
		gl_ds.periph_base.buf[i] = ((gl_ds.ctl_base.buf[i] + 1) % 255) + 1;
		
		periph_alias.buf[i] = 1;
		if(periph_alias.buf[i] == gl_ds.ctl_base.buf[i])
		{
			periph_alias.buf[i]++;
		}
		if(periph_alias.buf[i] == gl_ds.periph_base.buf[i])
		{
			periph_alias.buf[i]++;
		}

		TEST_ASSERT_NOT_EQUAL(gl_ds.ctl_base.buf[i], gl_ds.periph_base.buf[i]);
		TEST_ASSERT_NOT_EQUAL(gl_ds.periph_base.buf[i], periph_alias.buf[i]);
		TEST_ASSERT_NOT_EQUAL(gl_ds.ctl_base.buf[i], periph_alias.buf[i]);
	}
}

/*
Happy path testing for update_controller
*/
void test_dartt_update_controller(void)
{

	//init and zero out the gl_ds structure
	init_gl_ds();
	//ensure callbacks are configured to match message type. not necessary here because this is an internal function - just here for test consistency
	gl_msg_type = gl_ds.msg_type;	
	//make everything unequal
	scramble_buffers();

	unsigned char ctl_copy_backup[sizeof(gl_master_copy)] = {};
	memcpy(ctl_copy_backup, gl_ds.ctl_base.buf, gl_ds.ctl_base.size);	

	//happy path
	buffer_t ctl;
	ctl.buf = (unsigned char *)(&gl_master_copy.m2_set);
	ctl.size = 6*sizeof(uint32_t);
	ctl.len = ctl.size;
	int rc = dartt_update_controller(&ctl, &gl_ds);
	TEST_ASSERT_EQUAL(0, rc);
	int fidx = index_of_field(ctl.buf, gl_ds.ctl_base.buf, gl_ds.ctl_base.size);
	TEST_ASSERT_EQUAL(1, fidx);
	TEST_ASSERT_LESS_THAN(gl_ds.ctl_base.size, fidx*sizeof(uint32_t)+ctl.size);
	int match_count = 0;
	for(int i = 0; i < gl_ds.ctl_base.size; i++)
	{
		if(i >= fidx * sizeof(uint32_t) && i < fidx*sizeof(uint32_t) + ctl.size)
		{
			TEST_ASSERT_NOT_EQUAL(ctl_copy_backup[i], gl_ds.ctl_base.buf[i]);
			TEST_ASSERT_EQUAL(gl_ds.periph_base.buf[i], gl_ds.ctl_base.buf[i]);
			match_count++;
		}
		else
		{
			TEST_ASSERT_EQUAL(ctl_copy_backup[i], gl_ds.ctl_base.buf[i]);
		}
	}
	TEST_ASSERT_EQUAL(ctl.size, match_count);
}
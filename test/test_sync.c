
#include "checksum.h"
#include "dartt.h"
#include "dartt_sync.h"
#include "unity.h"

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
    motor_params_t mp[2];
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

int synctest_rx_blocking(unsigned char addr, buffer_t * rx, uint32_t timeout)
{
    //model peripheral with reply behavior and modifications to periph via alias
    payload_layer_msg_t rxpld_msg = {};
    dartt_frame_to_payload(p_sync_tx_buf, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &rxpld_msg);
    dartt_parse_general_message(&rxpld_msg, TYPE_SERIAL_MESSAGE, &periph_alias, rx);
    return DARTT_PROTOCOL_SUCCESS;
}


uint32_t gl_send_count = 0;    //flag to indicate to test software if tx is called. Zero before caller
int synctest_tx_blocking(unsigned char addr, buffer_t * tx, uint32_t timeout)
{
    printf("transmitted: a = 0x%X, rx=0x");
    for(int i = 0; i < tx->len; i++)
    {
        printf("%0.2X", tx->buf[i]);
    }
    printf("\n");
    
    
    unsigned char tx_cpy[sizeof(tx_mem)] = {};
    buffer_t tx_cpy_alias = {.buf = tx_cpy, .size = sizeof(tx_cpy), .len=tx->len};
    for(int i = 0; i < tx->size; i++)
    {
        tx_cpy_alias.buf[i] = tx->buf[i];
    }
    payload_layer_msg_t rxpld_msg = {};
    dartt_frame_to_payload(&tx_cpy_alias, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &rxpld_msg);
    dartt_parse_general_message(&rxpld_msg, TYPE_SERIAL_MESSAGE, &periph_alias, &tx_cpy_alias);    //pipe reply to tx, it's fine if we corrupt it with this call. It should pretty much just set the len to 0
    printf("tx len = %d\n", tx->len);
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
    buffer_t periph_master_alias;
    init_struct_buffer(&periph_master, &periph_master_alias);
    //sync params
    dartt_sync_t ctl_sync = {};
    ctl_sync.address = 3;
    init_struct_buffer(&ctl_master, &ctl_sync.base);
    ctl_sync.msg_type = TYPE_SERIAL_MESSAGE;
    int rc = dartt_init_buffer(&ctl_sync.tx_buf, tx_mem, sizeof(tx_mem));
    TEST_ASSERT_EQUAL(0,rc);
    rc = dartt_init_buffer(&ctl_sync.rx_buf, rx_mem, sizeof(rx_mem));
    TEST_ASSERT_EQUAL(0,rc);
    ctl_sync.blocking_rx_callback = &synctest_rx_blocking;
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

    ctl_master.m1_set = 10;
    ctl_master.mp[0].fds.align_offset=15;
    ctl_master.mp[1].pi_vq.ki.radix=7;
    ctl_master.mp[1].pi_vq.ki.i32 = 145;
    TEST_ASSERT_NOT_EQUAL(ctl_master.m1_set, periph_master.m1_set);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[0].fds.align_offset, periph_master.mp[0].fds.align_offset);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.ki.radix, periph_master.mp[1].pi_vq.ki.radix);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.ki.i32, periph_master.mp[1].pi_vq.ki.i32);
    TEST_ASSERT_EQUAL(sizeof(test_struct_t), ctl_master_alias.size);
    rc = dartt_sync(&ctl_master_alias, &periph_master_alias, &ctl_sync);
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
    rc = dartt_sync(&ctl_master_alias, &periph_master_alias, &ctl_sync);
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
    rc = dartt_sync(&ctl_master_alias, &periph_master_alias, &ctl_sync);
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
    rc = dartt_sync(&ctl_master_alias, &periph_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(ERROR_MEMORY_OVERRUN, rc);
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
    init_struct_buffer(&ctl_master, &ctl_sync.base);
    ctl_sync.msg_type = TYPE_SERIAL_MESSAGE;
    int rc = dartt_init_buffer(&ctl_sync.tx_buf, tx_mem, sizeof(tx_mem));
    TEST_ASSERT_EQUAL(0,rc);
    rc = dartt_init_buffer(&ctl_sync.rx_buf, rx_mem, sizeof(rx_mem));
    TEST_ASSERT_EQUAL(0,rc);
    ctl_sync.blocking_rx_callback = &synctest_rx_blocking;
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
        .buf = &ctl_master.m1_set,
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
    rc = dartt_ctl_read(&motor_commands, &periph_master_alias, &ctl_sync);
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
    rc = dartt_ctl_read(&motor_commands, &periph_master_alias, &ctl_sync);
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
    rc = dartt_ctl_read(&motor_commands, &periph_master_alias, &ctl_sync);
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
    dartt_init_buffer(&b2, b2_mem, sizeof(b2_mem));
    dartt_init_buffer(&b4, b4_mem, sizeof(b4_mem));
    dartt_sync_t ds = {};
    ds.blocking_rx_callback = &synctest_rx_blocking;
    ds.blocking_tx_callback = &synctest_tx_blocking;
    dartt_init_buffer(&ds.base, b3_mem, sizeof(b3_mem));    
    int rc = dartt_sync(&b1, &b2, &ds);
    TEST_ASSERT_NOT_EQUAL(0, rc);
    dartt_init_buffer(&ds.base, b1_mem, sizeof(b1_mem));
    rc = dartt_sync(&b1, &b2, &ds);
    TEST_ASSERT_EQUAL(0, rc);
}
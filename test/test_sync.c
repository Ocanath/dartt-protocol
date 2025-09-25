
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
    return SERIAL_PROTOCOL_SUCCESS;
}

//periph copy
test_struct_t periph = {};
buffer_t periph_alias = 
{
    .buf = (unsigned char * )(&periph),
    .size = sizeof(test_struct_t),
    .len = 0
};

int rx_blocking(unsigned char addr, buffer_t * rx, uint32_t timeout)
{
    //model peripheral with reply behavior and modifications to periph via alias
    payload_layer_msg_t rxpld_msg = {};
    dartt_frame_to_payload(p_sync_tx_buf, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &rxpld_msg);
    dartt_parse_general_message(&rxpld_msg, TYPE_SERIAL_MESSAGE, &periph_alias, rx);
    return SERIAL_PROTOCOL_SUCCESS;
}

int tx_blocking(unsigned char addr, buffer_t * tx, uint32_t timeout)
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
    return SERIAL_PROTOCOL_SUCCESS;
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
    ctl_sync.blocking_rx_callback = &rx_blocking;
    ctl_sync.blocking_tx_callback = &tx_blocking;
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
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_NOT_EQUAL(ctl_master_alias.buf[i], periph_alias.buf[i]);
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], periph_master_alias.buf[i]);
    }

    ctl_master.m1_set += 10;
    ctl_master.mp[0].fds.align_offset++;
    ctl_master.mp[1].pi_vq.ki.radix++;
    ctl_master.mp[1].pi_vq.ki.i32 -= 145;
    TEST_ASSERT_NOT_EQUAL(ctl_master.m1_set, periph_master.m1_set);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[0].fds.align_offset, periph_master.mp[0].fds.align_offset);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.ki.radix, periph_master.mp[1].pi_vq.ki.radix);
    TEST_ASSERT_NOT_EQUAL(ctl_master.mp[1].pi_vq.ki.i32, periph_master.mp[1].pi_vq.ki.i32);




    rc = dartt_sync(&ctl_master_alias, &periph_master_alias, &ctl_sync);
    TEST_ASSERT_EQUAL(SERIAL_PROTOCOL_SUCCESS, rc);
    for(int i = 0; i < ctl_master_alias.size; i++)
    {
        TEST_ASSERT_EQUAL(ctl_master_alias.buf[i], periph_master_alias.buf[i]);
    }

}
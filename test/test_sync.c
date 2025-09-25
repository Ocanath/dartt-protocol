
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



void test_dartt_sync(void)
{
    TEST_ASSERT_EQUAL(0, sizeof(test_struct_t)%sizeof(int32_t));//ensure struct is 32bit word aligned
    test_struct_t ctl_master;
    test_struct_t periph_master;
    test_struct_t periph;
       
}
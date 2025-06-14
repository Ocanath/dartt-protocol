#ifndef SERIAL_COMMS_STRUCT_H
#define SERIAL_COMMS_STRUCT_H
#include <stdint.h>
//any additional includes here for adding structs

typedef struct comms_t
{
    uint32_t motor_command_mode;

    /*Block off the words corresponding to default motor send values, so they can be sent in blocks */
    int32_t gl_iq;
    int32_t gl_joint_theta;
    int32_t gl_rotor_theta;
    int32_t gl_rotor_velocity;

    //other random motor specific commands
    int32_t gl_id;

	int32_t command_word;

    uint32_t write_filesystem_flag;
    //more fields


} comms_t;

#endif

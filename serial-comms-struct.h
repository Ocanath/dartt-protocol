#ifndef SERIAL_COMMS_STRUCT_H
#define SERIAL_COMMS_STRUCT_H
#include <stdint.h>
//any additional includes here for adding structs

typedef struct comms_t
{
    /*Block off the words corresponding to default motor send values, so they can be sent in blocks */
    int32_t gl_iq;
    int32_t gl_joint_theta;
    int32_t gl_rotor_theta;
    int32_t gl_rotor_velocity;

    int32_t gl_id;
    
    uint32_t reply_mode;
    uint32_t write_filesystem_flag;

    //more fields

    

} comms_t;

#endif

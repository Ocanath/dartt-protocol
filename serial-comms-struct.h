#ifndef SERIAL_COMMS_STRUCT_H
#define SERIAL_COMMS_STRUCT_H
#include <stdint.h>
//any additional includes here for adding structs

typedef struct comms_t
{
    int32_t gl_iq;
    int32_t gl_id;
    uint32_t flag_1;
    uint32_t flag_2;

} comms_t;

#endif

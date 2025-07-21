#include "checksum.h"
#include "serial-comms.h"
#include "unity.h"
#include "serial-comms-struct.h"

void setUp(void)
{

}

/*
	TODO:
		Create reciprocal create functions (for testing only) - validate create functions
		Create edge case conditions for parse_base. Test a read message which is one byte too small, etc.
		Add test of frame_to_payload of a type 0 serial message consisting of only address and crc
		Add test of frame_to_payload for a motor command and a serial command

*/
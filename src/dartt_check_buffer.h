#include "dartt.h"


/**
* @brief Helper function to validate buffer.
* 
* This function checks the three basic conditions for a buffer - non-null pointer arguments, 
* and overrun guard based on len/size. 
* 
* @param b The buffer we are checking for validity
*/
static inline int check_buffer(const dartt_buffer_t * b)
{
	if(b == NULL)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	if(b->buf == NULL)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	if(b->size == 0)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	if(b->len > b->size)
	{
		return ERROR_MEMORY_OVERRUN;
	}
	return DARTT_PROTOCOL_SUCCESS;
}

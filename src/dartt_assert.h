#ifndef DARTT_ASSERT_H_
#define DARTT_ASSERT_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DARTT_ASSERT
	#include <assert.h>
	#define DARTT_ASSERT(expression) assert(expression)
#endif

#ifdef __cplusplus
}
#endif

#endif

#ifndef CHECKSUM_H
#define CHECKSUM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif


uint16_t get_crc16(const unsigned char * arr, size_t size);
uint32_t get_crc32(const unsigned char * message, size_t len);

#ifdef __cplusplus
}
#endif

#endif


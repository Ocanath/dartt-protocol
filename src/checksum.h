#ifndef CHECKSUM_H
#define CHECKSUM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


uint16_t get_checksum16(uint16_t* arr, size_t size);
uint16_t get_crc16(uint8_t* arr, size_t size);
uint32_t get_crc32(unsigned char *message, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif


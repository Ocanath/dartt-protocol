#ifndef CHECKSUM_H
#define CHECKSUM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t get_checksum16(uint16_t* arr, size_t size);
uint16_t get_crc16(uint8_t* arr, size_t size);

#ifdef __cplusplus
}
#endif

#endif


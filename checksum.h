#ifndef CHECKSUM_H
#define CHECKSUM_H

#include <stdint.h>

uint16_t get_checksum16(uint16_t* arr, int size);
uint16_t get_crc16(uint8_t* arr, int size);

#endif


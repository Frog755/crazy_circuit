#ifndef BASE_TRANSFER_H__
#define BASE_TRANSFER_H__

#include <stdint.h>
#include <string.h>

float uint8Array2Float(uint8_t* u8Array, uint8_t Flip);
void float2uint8Array(uint8_t* u8Array, float* fdata, uint8_t Flip);

#endif

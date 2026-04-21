#pragma once

#include <stdint.h>

#define NUM_ELEM(x) (sizeof(x) / sizeof(*(x)))

#ifdef __cplusplus
extern "C" {
#endif

typedef struct IrCode {
    uint32_t carrier_freq;
    uint8_t num_pairs;
    uint32_t *pairs;
    uint8_t *sequence;
} IrCode;

extern struct IrCode *NApowerCodes[];
extern struct IrCode *EUpowerCodes[];
extern uint8_t num_NAcodes;
extern uint8_t num_EUcodes;

#ifdef __cplusplus
}
#endif

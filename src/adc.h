#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool adcInit();

int32_t getAdcValue();


#ifdef __cplusplus
}
#endif

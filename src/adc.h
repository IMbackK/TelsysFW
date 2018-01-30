#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool adcInit(void);

int32_t adcCreateAsyncTask(void (*callbackFunction)(int16_t* buffer, uint16_t length ));

int32_t getAdcValue(void);


#ifdef __cplusplus
}
#endif

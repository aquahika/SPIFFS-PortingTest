#ifndef ___SPIFFS_HAL_H___
#define ___SPIFFS_HAL_H___

#include "spiffs/spiffs_config.h"

s32_t spiffs_hal_read(u32_t addr, u32_t size, u8_t *dst);
s32_t spiffs_hal_write(u32_t addr, u32_t size, u8_t *src);
s32_t spiffs_hal_erase(u32_t addr, u32_t size);


#endif
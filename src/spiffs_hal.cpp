#include <Arduino.h>
#include "spiffs/spiffs.h"

#include "W25Q64/W25Q64.h"

static s32_t my_spiffs_read(u32_t addr, u32_t size, u8_t *dst) {
    W25Q64_read(addr,dst,size);
    return SPIFFS_OK;
}

static s32_t my_spiffs_write(u32_t addr, u32_t size, u8_t *src) {
    W25Q64_write(addr,src,size);
    return SPIFFS_OK;
}

const uint32_t SPI_FLASH_SEC_SIZE = 4096;

int32_t spiffs_hal_erase(uint32_t addr, uint32_t size) {
    if ((size & (SPI_FLASH_SEC_SIZE - 1)) != 0 ||
        (addr & (SPI_FLASH_SEC_SIZE - 1)) != 0) {
        Serial.print("Invalid erase sector address.");
        abort();
    }
    const uint32_t sector = addr / SPI_FLASH_SEC_SIZE;
    const uint32_t sectorCount = size / SPI_FLASH_SEC_SIZE;
    for (uint32_t i = 0; i < sectorCount; ++i) {
        W25Q64_eraseSector(sector + i,true);
    }
    return SPIFFS_OK;
}
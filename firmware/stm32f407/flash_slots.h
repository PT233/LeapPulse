#ifndef LEAPPULSE_FLASH_SLOTS_H
#define LEAPPULSE_FLASH_SLOTS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LP_APPLICATION_BASE UINT32_C(0x08010000)
#define LP_STAGING_BASE UINT32_C(0x08040000)
#define LP_SLOT_SIZE UINT32_C(0x00030000)

bool lp_flash_slots_available(void);
int lp_flash_begin(void *context, uint32_t size, uint32_t crc);
int lp_flash_write(void *context, uint32_t offset, const uint8_t *data, size_t length);
int lp_flash_read(void *context, uint32_t offset, uint8_t *data, size_t length);
int lp_flash_checkpoint(void *context, uint32_t size, uint32_t crc, uint32_t offset);
int lp_flash_resume(void *context, uint32_t *size, uint32_t *crc, uint32_t *offset);
int lp_flash_activate(void *context, uint32_t size, uint32_t crc);
bool lp_flash_install_pending(void);
bool lp_flash_application_valid(void);

#endif

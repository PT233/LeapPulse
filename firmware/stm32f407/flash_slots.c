#include "flash_slots.h"

#include <string.h>
#include "stm32f4xx_hal.h"

#define JOURNAL_BASE UINT32_C(0x08008000)
#define JOURNAL_SIZE UINT32_C(0x4000)
#define HEADER_MAGIC UINT32_C(0x4c505531)
#define READY_MAGIC UINT32_C(0x5ab4c367)
#define DONE_MAGIC UINT32_C(0xa9753c68)

struct journal_header {
    uint32_t size;
    uint32_t crc;
    uint32_t magic;
    uint32_t ready;
    uint32_t done;
};

struct checkpoint {
    uint32_t offset;
    uint32_t inverse;
};

static const volatile struct journal_header *const journal = (const void *)JOURNAL_BASE;

static bool span_ok(uint32_t offset, size_t length)
{
    return offset <= LP_SLOT_SIZE && length <= LP_SLOT_SIZE - offset;
}

bool lp_flash_slots_available(void)
{
    return *(const volatile uint16_t *)FLASHSIZE_BASE >= 512;
}

static bool erase_sectors(uint32_t sector, uint32_t count)
{
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = sector,
        .NbSectors = count,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t failed;
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }
    HAL_StatusTypeDef rc = HAL_FLASHEx_Erase(&erase, &failed);
    HAL_FLASH_Lock();
    return rc == HAL_OK;
}

static void flush_data_cache(void)
{
    if (FLASH->ACR & FLASH_ACR_DCEN) {
        __HAL_FLASH_DATA_CACHE_DISABLE();
        __HAL_FLASH_DATA_CACHE_RESET();
        __HAL_FLASH_DATA_CACHE_ENABLE();
    }
}

static bool program_bytes(uint32_t address, const uint8_t *bytes, size_t length)
{
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }

    bool ok = true;
    for (size_t i = 0; i < length; ++i) {
        uint32_t target = address + (uint32_t)i;
        uint8_t current = *(const volatile uint8_t *)target;

        if (current == bytes[i]) {
            continue;
        }
        /* A replay can complete an interrupted program if it only clears bits. */
        if ((current & bytes[i]) != bytes[i] ||
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, target, bytes[i]) != HAL_OK) {
            ok = false;
            break;
        }
    }
    HAL_FLASH_Lock();
    flush_data_cache();
    return ok && memcmp((const void *)address, bytes, length) == 0;
}

static bool mark(uint32_t address, uint32_t value)
{
    return program_bytes(address, (const uint8_t *)&value, sizeof(value));
}

int lp_flash_begin(void *context, uint32_t size, uint32_t crc)
{
    (void)context;
    if (!lp_flash_slots_available() || size < 8 || size > LP_SLOT_SIZE ||
        (journal->ready == READY_MAGIC && journal->done != DONE_MAGIC) ||
        !erase_sectors(FLASH_SECTOR_2, 1) ||
        !erase_sectors(FLASH_SECTOR_6, 1)) {
        return -1;
    }
    IWDG->KR = 0xAAAA;
    if (!erase_sectors(FLASH_SECTOR_7, 1)) {
        return -1;
    }

    const struct journal_header header = { size, crc, HEADER_MAGIC, UINT32_MAX, UINT32_MAX };
    return program_bytes(JOURNAL_BASE, (const uint8_t *)&header, sizeof(header)) ? 0 : -1;
}

int lp_flash_write(void *context, uint32_t offset, const uint8_t *data, size_t length)
{
    (void)context;
    if (!lp_flash_slots_available() || journal->magic != HEADER_MAGIC ||
        journal->ready != UINT32_MAX || !span_ok(offset, length) ||
        offset > journal->size || length > journal->size - offset) {
        return -1;
    }
    return program_bytes(LP_STAGING_BASE + offset, data, length) ? 0 : -1;
}

int lp_flash_read(void *context, uint32_t offset, uint8_t *data, size_t length)
{
    (void)context;
    if (!lp_flash_slots_available() || !span_ok(offset, length)) {
        return -1;
    }
    memcpy(data, (const void *)(LP_STAGING_BASE + offset), length);
    return 0;
}

static bool matching_header(uint32_t size, uint32_t crc)
{
    return lp_flash_slots_available() && journal->magic == HEADER_MAGIC &&
           journal->size == size && journal->crc == crc && size >= 8 && size <= LP_SLOT_SIZE;
}

int lp_flash_checkpoint(void *context, uint32_t size, uint32_t crc, uint32_t offset)
{
    (void)context;
    if (!matching_header(size, crc) || journal->ready != UINT32_MAX || offset > size) {
        return -1;
    }

    for (uint32_t address = JOURNAL_BASE + sizeof(struct journal_header);
         address < JOURNAL_BASE + JOURNAL_SIZE; address += sizeof(struct checkpoint)) {
        const volatile struct checkpoint *record = (const void *)address;
        if (record->offset == UINT32_MAX && record->inverse == UINT32_MAX) {
            const struct checkpoint next = { offset, ~offset };
            return program_bytes(address, (const uint8_t *)&next, sizeof(next)) ? 0 : -1;
        }
    }
    return -1;
}

int lp_flash_resume(void *context, uint32_t *size, uint32_t *crc, uint32_t *offset)
{
    (void)context;
    uint32_t image_size = journal->size;
    uint32_t image_crc = journal->crc;
    if (!matching_header(image_size, image_crc) || journal->ready != UINT32_MAX) {
        return -1;
    }

    *size = image_size;
    *crc = image_crc;
    *offset = 0;
    for (uint32_t address = JOURNAL_BASE + sizeof(struct journal_header);
         address < JOURNAL_BASE + JOURNAL_SIZE; address += sizeof(struct checkpoint)) {
        const volatile struct checkpoint *record = (const void *)address;
        if (record->offset <= image_size && record->inverse == ~record->offset) {
            *offset = record->offset;
        }
    }
    return 0;
}

static bool image_vectors_valid(uint32_t stored_at, uint32_t size)
{
    const uint32_t *vectors = (const void *)stored_at;
    uint32_t sp = vectors[0];
    uint32_t reset = vectors[1];
    bool valid_sp = ((sp > UINT32_C(0x20000000) && sp <= UINT32_C(0x20020000)) ||
                     (sp > UINT32_C(0x10000000) && sp <= UINT32_C(0x10010000))) &&
                    !(sp & 7u);
    return valid_sp && (reset & 1u) && reset >= LP_APPLICATION_BASE &&
           reset - LP_APPLICATION_BASE < size;
}

static uint32_t image_crc32(uint32_t address, uint32_t size)
{
    const uint8_t *data = (const void *)address;
    uint32_t crc = UINT32_MAX;
    for (uint32_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            uint32_t polynomial = (crc & 1u) ? UINT32_C(0xedb88320) : 0;
            crc = (crc >> 1) ^ polynomial;
        }
    }
    return ~crc;
}

int lp_flash_activate(void *context, uint32_t size, uint32_t crc)
{
    (void)context;
    if (!matching_header(size, crc) || !image_vectors_valid(LP_STAGING_BASE, size)) {
        return -1;
    }
    return mark(JOURNAL_BASE + offsetof(struct journal_header, ready), READY_MAGIC) ? 0 : -1;
}

bool lp_flash_install_pending(void)
{
    if (!lp_flash_slots_available()) {
        return false;
    }
    if (journal->ready != READY_MAGIC || journal->done == DONE_MAGIC) {
        return true;
    }

    uint32_t size = journal->size;
    uint32_t crc = journal->crc;
    if (!matching_header(size, crc) || !image_vectors_valid(LP_STAGING_BASE, size) ||
        image_crc32(LP_STAGING_BASE, size) != crc) {
        return false;
    }

    if (memcmp((const void *)LP_APPLICATION_BASE, (const void *)LP_STAGING_BASE, size) != 0) {
        IWDG->KR = 0xAAAA;
        if (!erase_sectors(FLASH_SECTOR_4, 1)) {
            return false;
        }
        IWDG->KR = 0xAAAA;
        if (!erase_sectors(FLASH_SECTOR_5, 1)) {
            return false;
        }
        for (uint32_t offset = 0; offset < size; offset += 128u) {
            uint8_t block[128];
            size_t length = size - offset < sizeof(block) ? size - offset : sizeof(block);
            memcpy(block, (const void *)(LP_STAGING_BASE + offset), length);
            if (!program_bytes(LP_APPLICATION_BASE + offset, block, length)) {
                return false;
            }
            IWDG->KR = 0xAAAA;
        }
    }

    return image_crc32(LP_APPLICATION_BASE, size) == crc &&
           mark(JOURNAL_BASE + offsetof(struct journal_header, done), DONE_MAGIC);
}

bool lp_flash_application_valid(void)
{
    if (!image_vectors_valid(LP_APPLICATION_BASE, LP_SLOT_SIZE)) {
        return false;
    }
    if (journal->ready == READY_MAGIC && journal->done == DONE_MAGIC) {
        return matching_header(journal->size, journal->crc) &&
               image_crc32(LP_APPLICATION_BASE, journal->size) == journal->crc;
    }
    return true;
}

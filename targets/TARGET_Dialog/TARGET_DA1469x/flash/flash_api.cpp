#include "flash_api.h"
#include "mbed.h"
extern "C"
{
#include "default_config.h"
#include "qspi_automode.h"
}
namespace
{
constexpr auto flash_base_addr = 0x16000000;
}

int32_t flash_init(flash_t *obj)
{
    // debug("flash_init\n");
    return 0;
}
int32_t flash_free(flash_t *obj)
{
    // debug("flash_free\n");
    return 0;
}
int32_t flash_erase_sector(flash_t *obj, uint32_t address)
{
    // debug("flash_erase_sector %08X \n", address);
    qspi_automode_erase_flash_sector(address - flash_base_addr);
    return 0;
}
int32_t flash_read(flash_t *obj, uint32_t address, uint8_t *data, uint32_t size)
{
    auto const result = qspi_automode_read(address - flash_base_addr, data, size);
    // debug("flash_read %08X [%lu]: %lu\n", address, size, result);
    return 0;
}
int32_t flash_program_page(flash_t *obj, uint32_t address, const uint8_t *data, uint32_t size)
{
    auto result = qspi_automode_write_flash_page(address - flash_base_addr, data, size);
    // debug("flash_program_page %08X [%lu]: %d\n", address, size, result);
    return 0;
}
uint32_t flash_get_sector_size(const flash_t *obj, uint32_t address)
{
    return FLASH_SECTOR_SIZE;
}
uint32_t flash_get_page_size(const flash_t *obj)
{
    return dg_configFLASH_MAX_WRITE_SIZE;
}
uint32_t flash_get_start_address(const flash_t *obj)
{
    return flash_base_addr;
}
uint32_t flash_get_size(const flash_t *obj)
{
    uint32_t const device_size = qspi_get_device_size(HW_QSPIC);
    // debug("device size: %lu\n", device_size);
    return device_size;
}
uint8_t flash_get_erase_value(const flash_t *obj)
{
    return 0xFF;
}
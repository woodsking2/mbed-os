#include "DA1469x_flash.h"
#include "gsl/gsl"
#include "flash_api.h"
#include "mbed_toolchain.h"
#include "mbed_critical.h"
#include "mbed_debug.h"
using namespace gsl;
using namespace std;
namespace
{
constexpr auto flash_start_address{0x16000000};
constexpr auto flash_size{0x1000000}; // 128 / 8 * 1024 * 1024;
constexpr auto page_size{256};
constexpr auto sector_size{4096};
union da1469x_qspi_data_reg {
    uint32_t d32;
    uint16_t d16;
    uint8_t d8;
};

MBED_SECTION(".text_ram") void write_u8(uint8_t data)
{
    volatile union da1469x_qspi_data_reg *reg = (union da1469x_qspi_data_reg *)&QSPIC->QSPIC_WRITEDATA_REG;
    reg->d8 = data;
}
MBED_SECTION(".text_ram") void write_u32(uint32_t data)
{
    volatile union da1469x_qspi_data_reg *reg = (union da1469x_qspi_data_reg *)&QSPIC->QSPIC_WRITEDATA_REG;
    reg->d32 = data;
}
MBED_SECTION(".text_ram") uint8_t read_u8(void)
{
    volatile union da1469x_qspi_data_reg *reg = (union da1469x_qspi_data_reg *)&QSPIC->QSPIC_READDATA_REG;
    return reg->d8;
}
MBED_SECTION(".text_ram") void set_manual()
{
    QSPIC->QSPIC_CTRLMODE_REG &= ~QSPIC_QSPIC_CTRLMODE_REG_QSPIC_AUTO_MD_Msk;
}
MBED_SECTION(".text_ram") void set_auto()
{
    QSPIC->QSPIC_CTRLMODE_REG |= QSPIC_QSPIC_CTRLMODE_REG_QSPIC_AUTO_MD_Msk;
}
MBED_SECTION(".text_ram") void set_single()
{
    QSPIC->QSPIC_CTRLBUS_REG = QSPIC_QSPIC_CTRLBUS_REG_QSPIC_SET_SINGLE_Msk;
    QSPIC->QSPIC_CTRLMODE_REG |=
        QSPIC_QSPIC_CTRLMODE_REG_QSPIC_IO2_OEN_Msk | QSPIC_QSPIC_CTRLMODE_REG_QSPIC_IO2_DAT_Msk | QSPIC_QSPIC_CTRLMODE_REG_QSPIC_IO3_OEN_Msk | QSPIC_QSPIC_CTRLMODE_REG_QSPIC_IO3_DAT_Msk;

    QSPIC->QSPIC_CTRLBUS_REG = QSPIC_QSPIC_CTRLBUS_REG_QSPIC_EN_CS_Msk;
    write_u8(0xff);
    QSPIC->QSPIC_CTRLBUS_REG = QSPIC_QSPIC_CTRLBUS_REG_QSPIC_DIS_CS_Msk;
}
MBED_SECTION(".text_ram") void set_quad()
{
    QSPIC->QSPIC_CTRLBUS_REG = QSPIC_QSPIC_CTRLBUS_REG_QSPIC_SET_QUAD_Msk;
    QSPIC->QSPIC_CTRLMODE_REG &= ~(QSPIC_QSPIC_CTRLMODE_REG_QSPIC_IO2_OEN_Msk | QSPIC_QSPIC_CTRLMODE_REG_QSPIC_IO3_OEN_Msk);
}
MBED_SECTION(".text_ram") uint8_t read_status()
{
    uint8_t status{};

    QSPIC->QSPIC_CTRLBUS_REG = QSPIC_QSPIC_CTRLBUS_REG_QSPIC_EN_CS_Msk;
    write_u8(0x05);
    status = read_u8();
    QSPIC->QSPIC_CTRLBUS_REG = QSPIC_QSPIC_CTRLBUS_REG_QSPIC_DIS_CS_Msk;

    return status;
}
MBED_SECTION(".text_ram") void wait_busy()
{
    uint8_t status{};
    do
    {
        status = read_status();
    } while (status & 0x01);
}
MBED_SECTION(".text_ram") void enable_write()
{
    uint8_t status;

    do
    {
        QSPIC->QSPIC_CTRLBUS_REG = QSPIC_QSPIC_CTRLBUS_REG_QSPIC_EN_CS_Msk;
        write_u8(0x06);
        QSPIC->QSPIC_CTRLBUS_REG = QSPIC_QSPIC_CTRLBUS_REG_QSPIC_DIS_CS_Msk;

        do
        {
            status = read_status();
        } while (status & 0x01);
    } while (!(status & 0x02));
}
MBED_SECTION(".text_ram") void qspi_flash_erase_sector(uint32_t address)
{

    core_util_critical_section_enter();
    set_manual();
    set_single();
    wait_busy();
    enable_write();

    QSPIC->QSPIC_CTRLBUS_REG = QSPIC_QSPIC_CTRLBUS_REG_QSPIC_EN_CS_Msk;
    address = __REV(address) & 0xffffff00;
    write_u32(address | 0x20);
    QSPIC->QSPIC_CTRLBUS_REG = QSPIC_QSPIC_CTRLBUS_REG_QSPIC_DIS_CS_Msk;

    wait_busy();
    set_quad();
    set_auto();
    CACHE->CACHE_CTRL1_REG |= CACHE_CACHE_CTRL1_REG_CACHE_FLUSH_Msk;
    core_util_critical_section_exit();
}
// MBED_SECTION(".text_ram")
void qspi_flash_read(uint32_t address, gsl::span<uint8_t> data)
{
    memcpy(data.data(), (void *)address, data.size());
}
MBED_SECTION(".text_ram") void qspi_flash_program_page(uint32_t address, gsl::span<uint32_t const> data)
{
    core_util_critical_section_enter();
    set_manual();
    set_single();
    wait_busy();

    enable_write();
    QSPIC->QSPIC_CTRLBUS_REG = QSPIC_QSPIC_CTRLBUS_REG_QSPIC_EN_CS_Msk;
    address = __REV(address) & 0xffffff00;
    write_u32(address | 0x32);
    set_quad();
    for (auto value : data)
    {
        write_u32(value);
    }

    QSPIC->QSPIC_CTRLBUS_REG = QSPIC_QSPIC_CTRLBUS_REG_QSPIC_DIS_CS_Msk;

    set_single();

    wait_busy();

    set_quad();
    set_auto();
    CACHE->CACHE_CTRL1_REG |= CACHE_CACHE_CTRL1_REG_CACHE_FLUSH_Msk;
    core_util_critical_section_exit();
}

} // namespace

/**
 * \defgroup flash_hal Flash HAL API
 * @{
 */

/** Initialize the flash peripheral and the flash_t object
 *
 * @param obj The flash object
 * @return 0 for success, -1 for error
 */
int32_t flash_init(flash_t *obj)
{
    return 0;
}

/** Uninitialize the flash peripheral and the flash_t object
 *
 * @param obj The flash object
 * @return 0 for success, -1 for error
 */
int32_t flash_free(flash_t *obj)
{
    return 0;
}

/** Erase one sector starting at defined address
 *
 * The address should be at sector boundary. This function does not do any check for address alignments
 * @param obj The flash object
 * @param address The sector starting address
 * @return 0 for success, -1 for error
 */
int32_t flash_erase_sector(flash_t *obj, uint32_t address)
{
    debug("flash erase sector： %u\n", address);
    Expects(address >= flash_start_address);
    Expects(address < flash_start_address + flash_size);
    uint32_t flash_address{};
    flash_address = address - flash_start_address;
    Expects(flash_address % sector_size == 0);
    qspi_flash_erase_sector(flash_address);
    return 0;
}

/** Read data starting at defined address
 *
 * This function has a WEAK implementation using memcpy for backwards compatibility.
 * @param obj The flash object
 * @param address Address to begin reading from
 * @param data The buffer to read data into
 * @param size The number of bytes to read
 * @return 0 for success, -1 for error
 */
int32_t flash_read(flash_t *obj, uint32_t address, uint8_t *data, uint32_t size)
{
    debug("flash read %x %u\n", address, size);
    qspi_flash_read(address, span<uint8_t>(data, size));
    return 0;
}

/** Program pages starting at defined address
 *
 * The pages should not cross multiple sectors.
 * This function does not do any check for address alignments or if size is aligned to a page size.
 * @param obj The flash object
 * @param address The sector starting address
 * @param data The data buffer to be programmed
 * @param size The number of bytes to program
 * @return 0 for success, -1 for error
 */
int32_t flash_program_page(flash_t *obj, uint32_t address, const uint8_t *data, uint32_t size)
{
    debug("flash program %x %u\n", address, size);
    Expects(address >= flash_start_address);
    Expects(address < flash_start_address + flash_size);
    uint32_t flash_address{};
    flash_address = address - flash_start_address;
    Expects(flash_address % page_size == 0);
    Expects(size % page_size == 0);
    span<uint32_t const> data_span{reinterpret_cast<uint32_t const *>(data), size / 4};
    qspi_flash_program_page(flash_address, data_span);
    return 0;
}

/** Get sector size
 *
 * @param obj The flash object
 * @param address The sector starting address
 * @return The size of a sector
 */
uint32_t flash_get_sector_size(const flash_t *obj, uint32_t address)
{
    return sector_size;
}

/** Get page size
 *
 * The page size defines the writable page size
 * @param obj The flash object
 * @return The size of a page
 */
uint32_t flash_get_page_size(const flash_t *obj)
{
    return page_size;
}

/** Get start address for the flash region
 *
 * @param obj The flash object
 * @return The start address for the flash region
 */
uint32_t flash_get_start_address(const flash_t *obj)
{
    return flash_start_address;
}

/** Get the flash region size
 *
 * @param obj The flash object
 * @return The flash region size
 */
uint32_t flash_get_size(const flash_t *obj)
{
    return flash_size; // 128 / 8 * 1024 * 1024;
}

/** Get the flash erase value
 *
 * @param obj The flash object
 * @return The flash erase value
 */
uint8_t flash_get_erase_value(const flash_t *obj)
{
    return 0xFF;
}

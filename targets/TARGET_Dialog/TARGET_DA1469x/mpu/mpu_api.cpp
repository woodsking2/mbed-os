#include "mpu_api.h"

/**
 * Initialize the MPU
 *
 * Initialize or re-initialize the memory protection unit.
 * After initialization or re-initialization, ROM and RAM protection
 * are both enabled.
 */
void mbed_mpu_init(void)
{

}

/**
 * Enable or disable ROM MPU protection
 *
 * This function is used to mark all of ROM as read and execute only.
 * When enabled writes to ROM cause a fault.
 *
 * By default writes to ROM are disabled.
 *
 * @param enable true to disable writes to ROM, false otherwise
 */
void mbed_mpu_enable_rom_wn(bool enable)
{

}

/**
 * Enable or disable ram MPU protection
 *
 * This function is used to mark all of RAM as execute never.
 * When enabled code is only allowed to execute from flash.
 *
 * By default execution from RAM is disabled.
 *
 * @param enable true to disable execution from RAM, false otherwise
 */
void mbed_mpu_enable_ram_xn(bool enable)
{

}

/** Deinitialize the MPU
 *
 * Powerdown the MPU in preparation for powerdown, reset or jumping to another application.
 */
void mbed_mpu_free(void)
{
    
}

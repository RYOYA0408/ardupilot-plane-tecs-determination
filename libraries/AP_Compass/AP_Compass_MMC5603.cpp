/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Compass_MMC5603.h"

#if AP_COMPASS_MMC5603_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

#define REG_PRODUCT_ID      0x39
#define REG_XOUT_L          0x00
#define REG_STATUS          0x18
#define REG_ODA             0x1A
#define REG_CONTROL0        0x1B
#define REG_CONTROL1        0x1C
#define REG_CONTROL2        0x1D

// bits in REG_CONTROL0
#define REG_CONTROL0_RESET  0x10 // Set coil for measuring offset
#define REG_CONTROL0_SET    0x08 // Reset coil for measuring offset
#define REG_CONTROL0_TMM    0x01 // Take Measurement for Magnetic field
#define REG_CONTROL0_TMT    0x02 // Take Measurement for Temperature
#define REG_CONTROL0_ASR    0x20 // Automatic Set/Reset
#define REG_CONTROL0_CMM    0x80 // Start the calculation of the measurement period

// bits in REG_CONTROL1
#define REG_CONTROL1_SW_RST 0x80 // Software reset
#define REG_CONTROL1_BW0    0x01
#define REG_CONTROL1_BW1    0x02

// bits in REG_CONTROL2
#define REG_CONTROL2_PRD    0x08 // Enable the function of periodical set
#define REG_CONTROL2_CMM    0x10 // Enter continuous mode

#define MMC5603_ID 0x10

AP_Compass_Backend *AP_Compass_MMC5603::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_MMC5603 *sensor = new AP_Compass_MMC5603(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_MMC5603::AP_Compass_MMC5603(AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                       bool _force_external,
                                       enum Rotation _rotation)
    : dev(std::move(_dev))
    , force_external(_force_external)
    , rotation(_rotation)
    , have_initial_offset(false)
{
}

bool AP_Compass_MMC5603::init()
{
    // take i2c bus semaphore
    WITH_SEMAPHORE(dev->get_semaphore());

    dev->set_retries(10);

    // setup to allow reads on SPI
    if (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        dev->set_read_flag(0x80);
    }

    // Reading REG_PRODUCT_ID fails sometimes on SPI, so we retry up to 10 times
    uint8_t whoami = 0;
    uint8_t tries = 10;
    while (whoami == 0 && tries > 0) {
        tries--;
        dev->read_registers(REG_PRODUCT_ID, &whoami, 1);
        hal.scheduler->delay(5);
    }

    if (whoami != MMC5603_ID) {
        printf("MMC5603 got unexpected product id: %d, expected: %d\n", whoami, MMC5603_ID);
        // not a MMC5603
        return false;
    }

    // reset sensor
    dev->write_register(REG_CONTROL1, REG_CONTROL1_SW_RST);

    // 20ms minimum startup time
    hal.scheduler->delay(30);

    // ODR 50Hz
    if (!dev->write_register(REG_ODA, 50)) {
        return false;
    }

    // Bandwidth 6.6ms
    if (!dev->write_register(REG_CONTROL1, 0x0)) {
        return false;
    }

    // Set Auto_SR_en
    // Set Cmm_freq_en
    if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_ASR | REG_CONTROL0_CMM)){
        return false;
    }

    // Prd_set = 100
    // En_prd_set = 1
    // Set Cmm_en
    if (!dev->write_register(REG_CONTROL2, 0x1b)) {
        return false;
    }

    /* register the compass instance in the frontend */
    dev->set_device_type(DEVTYPE_MMC5603);
    if (!register_compass(dev->get_bus_id(), compass_instance)) {
        return false;
    }

    set_dev_id(compass_instance, dev->get_bus_id());

    printf("Found a MMC5603 on 0x%x as compass %u\n", dev->get_bus_id(), compass_instance);

    set_rotation(compass_instance, rotation);

    if (force_external) {
        set_external(compass_instance, true);
    }

    dev->set_retries(1);

    // call timer() at 100Hz
    dev->register_periodic_callback(10000U,
                                    FUNCTOR_BIND_MEMBER(&AP_Compass_MMC5603::timer, void));

    return true;
}

void AP_Compass_MMC5603::timer()
{
    const uint32_t zero_offset = 524288UL; // 20 bit mode
    const uint32_t sensitivity = 16384UL; // counts per Gauss, 20 bit mode
    constexpr float counts_to_milliGauss = 1.0e3f / sensitivity;

    uint8_t status;
    if (!dev->read_registers(REG_STATUS, &status, 1)) {
        return;
    }

    // check if measurement is ready
    if (!(status & 0x40)) {
        return;
    }

    uint8_t data[9];
    if (!dev->read_registers(REG_XOUT_L, (uint8_t *)&data[0], 9)) {
        return;
    }

    Vector3f field {float((data[0] << 12) + (data[1] << 4) + (data[6] >> 4)) - zero_offset,
                    float((data[2] << 12) + (data[3] << 4) + (data[7] >> 4)) - zero_offset,
                    float((data[4] << 12) + (data[5] << 4) + (data[8] >> 4)) - zero_offset};
    field *= counts_to_milliGauss;
    accumulate_sample(field, compass_instance);
}

void AP_Compass_MMC5603::read()
{
    drain_accumulated_samples(compass_instance);
}

#endif  // AP_COMPASS_MMC5603_ENABLED


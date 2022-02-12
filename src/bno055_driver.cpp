/**
 *  MIT License
 *  Copyright (c) 2021 Christian Castaneda <github.com/christianjc>
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 * 
 * 
 *          https://www.bosch-sensortec.com/bst/products/all_products/bno055
 *          Reference Datasheet: BST_BNO055_DS000_14 (consulted in January 2018)
 * 
*/

#include "bno055_driver/bno055_driver.h"

namespace bno055_imu {
    BNO055Driver::BNO055Driver(std::string device_, int address_){
        _device = device_;
        _address = address_;
        _opmode = OPERATION_MODE_IMUPLUS;

    }

    void BNO055Driver::init() {

        /** open i2c device **/
        file = open(device.c_str(), O_RDWR);
        if (file < 0) {
             throw std::runtime_error("Could not open i2c file");
        }

        /** Set the address for the target device (slave) **/
        if(ioctl(file, I2C_SLAVE, address) < 0) {
            throw std::runtime_error("Could not open i2c device!!");
        }

        /** Verify we have the correct device **/
        if(bno_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
            throw std::runtime_error("incorrect chip ID");
        }

        // Check if we have a calibration profile

        // Reset the device

        // write offsets to the sensor to calibrate it

        // check for status calibration

        // set the external clock 

        // set units if needed

        // Set the device to fusion imu mode
        set_opmode(_opmode);
        
        __s32 data =  bno_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR);
       
        ROS_INFO_STREAM("running sensor imu");
        ROS_INFO_STREAM(data);

    }
    

    bool BNO055Driver::read_imu_data(imu_data_t &data) {
        if(bno_i2c_smbus_read_i2c_block_data(file, BNO055_GYRO_DATA_X_LSB_ADDR, 32, (uint8_t*)&data) != 0x20) {
            throw std::runtime_error("read error");
            return false;
        }
        ROS_INFO_STREAM(data.angular_velocity_x);

        return true;
    }

    void BNO055Driver::set_opmode(opmode_t opmode) {
         if (bno_i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, opmode) < 0) {
             throw std::runtime_error("wirte error in opmode function");
        }
    }

    /**
    *   @brief  Sets to use the external crystal (32.768KHz).
    *
    *   @param  use_external_crystal use external crystal boolean.
    *
    *   @return  ESP_OK - external crystal was set succesfully.
    *          ESP_FAIL - external crystal could not be set.
    */
    void set_external_crystal(void) {
        /* Switch to config mode */
        set_opmode(OPERATION_MODE_CONFIG);

        /* set correct page id */
        write8(BNO055_PAGE_ID_ADDR, 0);

        /* Set the external clock*/
        write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
        
        /* Set previouse operation mode */
        err = set_opmode(OPERATION_MODE_IMUPLUS);
    }

    void BNO055Driver::write8(reg_t reg_, __8 value) {
        if (bno_i2c_smbus_write_byte_data(file, reg_, value) < 0) {
             throw std::runtime_error("wirte error in write8 function");
        }
    }

    void BNO055Driver::read8(reg_t reg_) {
        if (bno_i2c_smbus_read_byte_data(file, reg_) < 0) {
             throw std::runtime_error("read error in read8 function");
        }
    }


}
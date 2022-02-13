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
        file = open(_device.c_str(), O_RDWR);
        if (file < 0) {
             throw std::runtime_error("Could not open i2c file");
        }

        /** Set the address for the target device (slave) **/
        if(ioctl(file, I2C_SLAVE, _address) < 0) {
            throw std::runtime_error("Could not open i2c device!!");
        }

        /** Verify we have the correct device **/
        if(read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
            throw std::runtime_error("incorrect chip ID");
        }

        // Check if we have a calibration profile

        // Reset the device

        // write offsets to the sensor to calibrate it

        // check for status calibration 

        // set the external clock 
        set_external_crystal();

        // set units if needed

        // Set the device to fusion imu mode
        set_opmode(_opmode);
        
        __s32 data =  bno_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR);
       
        ROS_INFO_STREAM("running sensor imu");
        ROS_INFO_STREAM(data);

    }
    
    
    bool BNO055Driver::read_imu_data(sensor_msgs::Imu &imu) {
        imu_data_t data;
        if(bno_i2c_smbus_read_i2c_block_data(file, BNO055_GYRO_DATA_X_LSB_ADDR, 32, (uint8_t*)&data) != 0x20) {
            throw std::runtime_error("read error");
            return false;
        }

        /* Gyroscope */
        /* 1dps = 16 LSB */
        /* 1rps = 900 LSB */
        imu.angular_velocity.x = ((double)data.angular_velocity_x) / 16.0;
        imu.angular_velocity.y = ((double)data.angular_velocity_y) / 16.0;
        imu.angular_velocity.z = ((double)data.angular_velocity_z) / 16.0;

        /*!
         * Assign to Quaternion
         * See
         * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
         * 3.6.5.5 Orientation (Quaternion)
         */
        const double scale = (1.0 / (1 << 14));
        imu.orientation.w = (double)(data.quaternion_w * scale);
        imu.orientation.x = (double)(data.quaternion_x * scale);
        imu.orientation.y = (double)(data.quaternion_y * scale);
        imu.orientation.z = (double)(data.quaternion_z * scale);

        /* Linear acceleration */
        /* 1m/s^2 = 100 LSB */
        /* 1mg = 1 LSB */
        imu.linear_acceleration.x = ((double)data.linear_acceleration_x) / 100.0;
        imu.linear_acceleration.y = ((double)data.linear_acceleration_y) / 100.0;
        imu.linear_acceleration.z = ((double)data.linear_acceleration_z) / 100.0;

        return true;
    }

    /**
      * @brief   Sets bno055 to one of the following operation mode:
      *              OPERATION_MODE_CONFIG
      *              OPERATION_MODE_ACCONLY
      *              OPERATION_MODE_MAGONLY
      *              OPERATION_MODE_GYRONLY
      *              OPERATION_MODE_ACCMAG
      *              OPERATION_MODE_ACCGYRO
      *              OPERATION_MODE_MAGGYRO
      *              OPERATION_MODE_AMG
      *              OPERATION_MODE_IMUPLUS        << This is the mode we use here>>
      *              OPERATION_MODE_COMPASS
      *              OPERATION_MODE_M4G
      *              OPERATION_MODE_NDOF_FMC_OFF
      *              OPERATION_MODE_NDOF
      *
      * @param op_mode   mask config of type opmode_t
      */
    void BNO055Driver::set_opmode(opmode_t opmode) {
         if (bno_i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, opmode) < 0) {
             throw std::runtime_error("wirte error in opmode function");
        }
    }

    /**
      * @brief  Sets to use the external crystal (32.768KHz).
      */
    void BNO055Driver::set_external_crystal(void) {
        /* Switch to config mode */
        set_opmode(OPERATION_MODE_CONFIG);
        /* set correct page id */
        write8(BNO055_PAGE_ID_ADDR, 0);
        /* Set the external clock*/
        write8(BNO055_SYS_TRIGGER_ADDR, 0x80);  
        /* Set previouse operation mode */
        set_opmode(_opmode);
    }

    /**
      * @brief   Writes one byte of data to the BNO055 given register
      * @param   reg: Register address to write the data to.
      * @param   value: Data to be written in the register.
      */
    void BNO055Driver::write8(reg_t reg_, __u8 value) {
        if (bno_i2c_smbus_write_byte_data(file, reg_, value) < 0) {
             throw std::runtime_error("wirte error in write8 function");
        }
    }

    /**
      * @brief Reads one byte from the BNO055 (i2c slave) register
      * @param reg   Register address to read from.
      * @return Returns one byte of data read from the given register.
      */
    __s32 BNO055Driver::read8(reg_t reg_) {
        __s32 ret = bno_i2c_smbus_read_byte_data(file, reg_);
        if (ret < 0) {
             throw std::runtime_error("read error in read8 function");
        }
        return ret;
    }


    /**
      *  @brief  Gets the temperature in degrees celsius.
      *
      *  @return temperature in degrees celsius.
      */
    int8_t get_temp(void) {
        int8_t temp = (int8_t)(read8(BNO055_TEMP_ADDR));
        return temp;
    }


    /**
      *  @brief  Writes the sensor's offset registers from a byte array.
      *
      *  @param  calib_data Calibration offset of type uint8_t (buffer size should be 22).
      */
    esp_err_t set_sensor_offset(uint8_t *calib_data) {
        set_opmode(OPERATION_MODE_CONFIG);
        uint8_t write_buffer[23];
        write_buffer[0] = (uint8_t)ACCEL_OFFSET_X_LSB_ADDR;
        for (int i = 1; i < 23; i++)
        {
            write_buffer[i] = (uint8_t)calib_data[i - 1];
        }

        err = i2c_master_write_to_device(I2C_NUM_0, (uint8_t)BNO055_ADDRESS,
                                        write_buffer, (size_t)23, TIME_TO_WAIT_READ_WRITE);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error in i2c_master_write_to_device: %x", err);
            return err;
        }
        err = set_opmode(opmode);
        return err;
    }

    /**
      *  @brief  Reads the sensor's offset registers into a byte array.
      *
      *  @param  calib_data Calibration offset of type uint8_t (buffer size should be 22).
      */
    esp_err_t get_sensor_offsets(uint8_t *calib_data) {
        if (isFullyCalibrated())
        { // TODO: uncoment this line
            bno055_opmode_t opmode = get_opmode();
            uint8_t read_reg[1];
            read_reg[0] = (uint8_t)ACCEL_OFFSET_X_LSB_ADDR;
            err = set_opmode(OPERATION_MODE_CONFIG);
            if (err != ESP_OK)
                return err;
            err = i2c_master_write_read_device(I2C_NUM_0, (uint8_t)BNO055_ADDRESS,
                                            read_reg, (size_t)1, calib_data, NUM_BNO055_OFFSET_REGISTERS, TIME_TO_WAIT_READ_WRITE);
            // readLen(ACCEL_OFFSET_X_LSB_ADDR, calib_data, NUM_BNO055_OFFSET_REGISTERS);

            err = set_opmode(opmode);
            return err;
        }
        return ESP_FAIL;
    }


}
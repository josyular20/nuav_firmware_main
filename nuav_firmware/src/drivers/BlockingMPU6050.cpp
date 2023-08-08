//
// Created by Ian on 6/5/2022.
//

#include <cstring>
#include <cmath>
#include <cstdio>
#include "BlockingMPU6050.h"


HAL_StatusTypeDef MPU6050::BlockingMPU6050::FetchDataRegisters(uint8_t start_addr, uint16_t num_registers) {
    HAL_StatusTypeDef res;
    res = HAL_I2C_Mem_Read(&i2c_handle_, dev_addr_, start_addr,
                           I2C_MEMADD_SIZE_8BIT, data_buffer_, num_registers, MPU6050_I2C_TIMEOUT);
    return res;
}


HAL_StatusTypeDef MPU6050::BlockingMPU6050::UpdateRegister(uint8_t reg_addr, uint8_t data, uint8_t mask) {
    HAL_StatusTypeDef res;

    // If there is a specific mask, we need to read the register and apply the data with the mask
    if(mask != 0xFF) {
        // Read current register value
        res = HAL_I2C_Mem_Read(&i2c_handle_, dev_addr_, reg_addr,
                               I2C_MEMADD_SIZE_8BIT, data_buffer_, 1, MPU6050_I2C_TIMEOUT);
        if(res != HAL_OK)
            return res;

        // Clear area encompassed by the mask
        data_buffer_[0] &= ~mask;
        // Apply data
        data_buffer_[0] |= data & mask;
    } else {
        data_buffer_[0] = data;
    }

    // Write new value
    res = HAL_I2C_Mem_Write(&i2c_handle_, dev_addr_, reg_addr,
                            I2C_MEMADD_SIZE_8BIT, data_buffer_, 1, MPU6050_I2C_TIMEOUT);

    return res;
}


int16_t MPU6050::BlockingMPU6050::PackBytesSigned(uint8_t high, uint8_t low) {
    return static_cast<int16_t>(((uint16_t)high << 8) | low);
}


void MPU6050::BlockingMPU6050::Initialize() {
    // Set clock source to Gyro as it is apparently more
    // accurate than the default according to the datasheet.
    ConfigurePower(false, MPU6050_CLOCK_PLL_XGYRO);

    // Set default ranges to most sensitive, thus giving the most
    // precise data assuming it isn't moving/accelerating too much
    ConfigureAccelRange(Accel2g);
    ConfigureGyroRange(Gyro250dps);
}

bool MPU6050::BlockingMPU6050::IsConnected() {
    HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(&i2c_handle_, dev_addr_, 2, MPU6050_I2C_TIMEOUT);
    return res == HAL_OK;
}


MPU6050::Vector3 MPU6050::BlockingMPU6050::ReadAccel() {
    MPU6050::Vector3 accel_data{};
    HAL_StatusTypeDef res;

    // Read all 6 registers to get Accel data (first register is MPU6050_RA_ACCEL_XOUT_H)
    res = FetchDataRegisters(MPU6050_RA_ACCEL_XOUT_H, 6);
    // Return the zero vector if acquisition failed
    if (res != HAL_OK)
        return accel_data;

    // Convert high and low byte to a signed 16bit value
    accel_data.x = PackBytesSigned(data_buffer_[0], data_buffer_[1])*accel_range_/INT16_MAX;
    accel_data.y = PackBytesSigned(data_buffer_[2], data_buffer_[3])*accel_range_/INT16_MAX;
    accel_data.z = PackBytesSigned(data_buffer_[4], data_buffer_[5])*accel_range_/INT16_MAX;

    return accel_data;
}


MPU6050::Vector3 MPU6050::BlockingMPU6050::ReadGyro() {
    MPU6050::Vector3 gyro_data{};
    HAL_StatusTypeDef res;

    // Read all 6 registers to get Accel data (first register is MPU6050_RA_ACCEL_XOUT_H)
    res = FetchDataRegisters(MPU6050_RA_GYRO_XOUT_H, 6);
    // Return the zero vector if acquisition failed
    if (res != HAL_OK)
        return gyro_data;

    // Convert high and low byte to a signed 16bit value
    gyro_data.x = PackBytesSigned(data_buffer_[0], data_buffer_[1])*gyro_range_/INT16_MAX;
    gyro_data.y = PackBytesSigned(data_buffer_[2], data_buffer_[3])*gyro_range_/INT16_MAX;
    gyro_data.z = PackBytesSigned(data_buffer_[4], data_buffer_[5])*gyro_range_/INT16_MAX;

    // Return the gyro data removing the bias (calculated via CalibrateGyro)
    return gyro_data-gyro_bias_;
}


double MPU6050::BlockingMPU6050::ReadTempC() {
    HAL_StatusTypeDef res;

    // Read both registers to get Accel data (first register is MPU6050_RA_ACCEL_XOUT_H)
    res = FetchDataRegisters(MPU6050_RA_TEMP_OUT_H, 2);
    // Return the NaN if acquisition failed
    if (res != HAL_OK)
        return NAN;

    // Convert high and low byte to a temperature value
    // Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
    return static_cast<double>(PackBytesSigned(data_buffer_[0], data_buffer_[1])) / 340 + 36.53;
}


MPU6050::MPU6050Data MPU6050::BlockingMPU6050::ReadBatchData() {
    MPU6050::MPU6050Data mpu_data{};
    HAL_StatusTypeDef res;

    // Read all 6 registers to get Accel data (first register is MPU6050_RA_ACCEL_XOUT_H)
    res = FetchDataRegisters(MPU6050_RA_ACCEL_XOUT_H, 14);
    // Return the zero vector if acquisition failed
    if (res != HAL_OK)
        return mpu_data;

    // Convert high and low byte to a signed 16bit value and the convert to g's
    mpu_data.accel_data.x = PackBytesSigned(data_buffer_[0], data_buffer_[1])*accel_range_/INT16_MAX;
    mpu_data.accel_data.y = PackBytesSigned(data_buffer_[2], data_buffer_[3])*accel_range_/INT16_MAX;
    mpu_data.accel_data.z = PackBytesSigned(data_buffer_[4], data_buffer_[5])*accel_range_/INT16_MAX;

    // Convert high and low byte to a signed 16bit value
    mpu_data.temp_data = static_cast<double>(PackBytesSigned(data_buffer_[6], data_buffer_[7])) / 340 + 36.53;

    // Convert high and low byte to a signed 16bit value and then convert to deg/s
    mpu_data.gyro_data.x = PackBytesSigned(data_buffer_[8],  data_buffer_[9]) *gyro_range_/INT16_MAX;
    mpu_data.gyro_data.y = PackBytesSigned(data_buffer_[10], data_buffer_[11])*gyro_range_/INT16_MAX;
    mpu_data.gyro_data.z = PackBytesSigned(data_buffer_[12], data_buffer_[13])*gyro_range_/INT16_MAX;
    mpu_data.gyro_data -= gyro_bias_;

    return mpu_data;
}


void MPU6050::BlockingMPU6050::ConfigureLPF(MPU6050::LPFType band_width) {
    // Update the three DLPF_CFG bits in the config register
    UpdateRegister(MPU6050_RA_CONFIG, band_width, 0b0000'0111);
}


void MPU6050::BlockingMPU6050::ConfigureGyroRange(MPU6050::GyroRange range) {
    // Update the two FS_SEL bits in the gyro config register
    UpdateRegister(MPU6050_RA_GYRO_CONFIG, range, 0b0001'1000);

    switch(range){
        case Gyro250dps:
            gyro_range_ = 250;
            break;
        case Gyro500dps:
            gyro_range_ = 500;
            break;
        case Gyro1000dps:
            gyro_range_ = 1000;
            break;
        case Gyro2000dps:
            gyro_range_ = 2000;
            break;
    }
}


void MPU6050::BlockingMPU6050::ConfigureAccelRange(MPU6050::AccelRange range) {
    // Update the two AFS_SEL bits in the gyro config register
    UpdateRegister(MPU6050_RA_ACCEL_CONFIG, range, 0b0001'1000);

    switch(range){
        case Accel2g:
            accel_range_ = 2;
            break;
        case Accel4g:
            accel_range_ = 4;
            break;
        case Accel8g:
            accel_range_ = 8;
            break;
        case Accel16g:
            accel_range_ = 16;
            break;
    }
}


void MPU6050::BlockingMPU6050::ConfigurePower(bool enable_sleep, uint8_t clksel) {
    // Update the sleep bit (bit6) and the three CLKSEL bits in the first power register
    uint8_t sleep_val = enable_sleep ? MPU6050_PWR1_SLEEP_BIT : 0;
    UpdateRegister(MPU6050_RA_PWR_MGMT_1, sleep_val | clksel, 0b0100'0111);
}

void MPU6050::BlockingMPU6050::CalibrateGyro(uint16_t num_samples) {
    // Average a bunch of samples to get the average bias
    gyro_bias_ = {};
    Vector3 running_sum = {};
    for(uint16_t i = 0; i < num_samples; i++){
        running_sum += ReadGyro();
        HAL_Delay(1);
    }
    gyro_bias_ = running_sum / num_samples;
}





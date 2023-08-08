//
//  Created by Ian on 6/5/2022.
//  Datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
//  Register Reference: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
//  Accel calibration reference: https://www.st.com/resource/en/application_note/dm00119044-parameters-and-calibration-of-a-lowg-3axis-accelerometer-stmicroelectronics.pdf
//  Gyro calibration reference: https://www.analog.com/media/en/technical-documentation/technical-articles/gyrocalibration_edn_eu_7_2010.pdf
//

#ifndef ACTIVE_TETHER_BLOCKINGMPU6050_H
#define ACTIVE_TETHER_BLOCKINGMPU6050_H

#define MPU6050_I2C_TIMEOUT 100

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "MPU6050_register_map.h"

namespace MPU6050 {

enum LPFType {
  LPFOff = MPU6050_DLPF_BW_256,
  LPF188Hz = MPU6050_DLPF_BW_188,
  LPF98Hz = MPU6050_DLPF_BW_98,
  LPF42Hz = MPU6050_DLPF_BW_42,
  LPF20Hz = MPU6050_DLPF_BW_20,
  LPF10Hz = MPU6050_DLPF_BW_10,
  LPF5Hz = MPU6050_DLPF_BW_5
};

enum GyroRange {
  Gyro250dps = MPU6050_GYRO_FS_250,
  Gyro500dps = MPU6050_GYRO_FS_500,
  Gyro1000dps = MPU6050_GYRO_FS_1000,
  Gyro2000dps = MPU6050_GYRO_FS_2000
};

enum AccelRange {
  Accel2g = MPU6050_ACCEL_FS_2,
  Accel4g = MPU6050_ACCEL_FS_4,
  Accel8g = MPU6050_ACCEL_FS_8,
  Accel16g = MPU6050_ACCEL_FS_16,
};

struct Vector3 {
  double x;
  double y;
  double z;

  Vector3 operator+(const Vector3& b) const{return Vector3{x+b.x, y+b.y, z+b.z};}
  Vector3 operator-(const Vector3& b) const{return Vector3{x-b.x, y-b.y, z-b.z};}
  Vector3 operator*(const int& b) const{return Vector3{x*b, y*b, z*b};}
  Vector3 operator/(const int& b) const{return Vector3{x/b, y/b, z/b};}
  void operator*=(const int& b) {x*=b; y*=b; z*=b;}
  void operator/=(const int& b) {x/=b; y/=b; z/=b;}
  void operator-=(const Vector3& b) {x-=b.x; y-=b.y; z-=b.z;}
  void operator+=(const Vector3& b) {x+=b.x; y+=b.y; z+=b.z;}
};

struct MPU6050Data {
  Vector3 accel_data;
  double temp_data; // deg C
  Vector3 gyro_data;
};

class BlockingMPU6050 {
 public:
  /// Constructs a BlockingMPU6050 given the i2c handle for the bus it is on
  explicit BlockingMPU6050(I2C_HandleTypeDef &hi2c, bool alternate_addr = false) : dev_addr_(
      (alternate_addr) ? MPU6050_ADDRESS_AD0_HIGH << 1 : MPU6050_ADDRESS_AD0_LOW << 1), i2c_handle_(hi2c) {};

  /// Initializes the MPU6050
  void Initialize();
  /// Pings to see if the device is connected
  bool IsConnected();

  /// Assuming a stationary gyro, this calculates and applies a bias factor
  void CalibrateGyro(uint16_t num_samples = 1000);

  /// Reads Accelerometer data in g's (0x3B - 0x40)
  Vector3 ReadAccel();
  /// Reads Temperature data in C (0x41 - 0x42)
  double ReadTempC();
  /// Reads Gyroscope data in deg/s (0x43 - 0x48)
  Vector3 ReadGyro();
  /// Reads a batch of data. All data in the batch is guaranteed to be from the same time sample (0x3B - 0x48)
  MPU6050Data ReadBatchData();

  /// Configures the Low-Pass Filter (via MPU6050_RA_CONFIG register).
  /// If enabled (not LPFOff) the gyro's sampling frequency is changed to 1khz rather than 8khz
  void ConfigureLPF(LPFType band_width);
  /// Configures the range of the Gyro (via MPU6050_RA_GYRO_CONFIG)
  void ConfigureGyroRange(GyroRange range);
  /// Configures the range of the Accelerometer (via MPU6050_RA_ACCEL_CONFIG)
  void ConfigureAccelRange(AccelRange range);
  /// Configures the power management register (via MPU6050_RA_PWR_MGMT_1)
  void ConfigurePower(bool enable_sleep, uint8_t clksel);

  Vector3 gyro_bias_ = {};

 private:
  HAL_StatusTypeDef FetchDataRegisters(uint8_t start_addr, uint16_t num_registers);
  HAL_StatusTypeDef UpdateRegister(uint8_t reg_addr, uint8_t data, uint8_t mask = 0xFF);
  static inline int16_t PackBytesSigned(uint8_t high, uint8_t low);

  uint8_t dev_addr_;
  double gyro_range_ = 250;
  double accel_range_ = 2;
  I2C_HandleTypeDef &i2c_handle_;

  // Data buffer for sending/receiving.
  // Max needed size is for a full sensor read of accel, gyro, and temp which requires 6+6+2=14 bytes
  uint8_t data_buffer_[16] = {};
};
}

#endif //ACTIVE_TETHER_BLOCKINGMPU6050_H

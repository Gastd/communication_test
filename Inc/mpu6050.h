typedef struct
{
    uint32_t accelScaleRange;

    uint32_t gyroScaleRange;

}MPU6050Config;


typedef struct
{
    I2C_HandleTypeDef *I2CInterface;

    MPU6050Config config;

    uint32_t deviceAddress;

    uint32_t memAddress;

    uint32_t memSize;

    uint8_t lastData[14];

}MPU6050Imu;

void mpu6050_configDevice(MPU6050Imu *imu6050, I2C_HandleTypeDef* interface, uint32_t accelConfig, uint32_t gyroConfig);
void mpu6050_geData(MPU6050Imu *imu6050);

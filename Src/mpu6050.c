/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "mpu6050.h"

/* Private variables ---------------------------------------------------------*/
#define MPU6050_ADDRESS (0x68 << 1)
#define CONF_ADDRESS (0x6B)
#define GYRO_ADDRESS (0x1B)
#define ACCEL_ADDRESS (0x1C)

uint16_t trials, timeout;

/* Private function prototypes -----------------------------------------------*/
// void mpu6050_requestData(MPU6050Imu *imu6050, uint32_t memAddress, uint8_t *buffer);

/* Body Functions ------------------------------------------------------------*/
void MPU6050_configDevice(MPU6050Imu *imu6050, I2C_HandleTypeDef* interface, uint32_t accelConfig, uint32_t gyroConfig)
{
    trials = 100;
    timeout = 100;
    imu6050->I2CInterface = interface;
    imu6050->deviceAddress = MPU6050_ADDRESS;
    imu6050->memAddress = 0x3B;
    imu6050->memSize = 14;
    imu6050->config.accelScaleRange = accelConfig;
    imu6050->config.gyroScaleRange = gyroConfig;


    uint16_t writeSize = 1;
    uint8_t initDevData = 0x0;
    uint8_t initAccData = 0x0;
    uint8_t initGyrData = 0x0;
    uint16_t gyroConfAddress = GYRO_ADDRESS;
    uint16_t accelConfAddress = ACCEL_ADDRESS;
    uint16_t deviceConfAddress = CONF_ADDRESS;

    if(HAL_I2C_IsDeviceReady(imu6050->I2CInterface, imu6050->deviceAddress, trials, timeout) == HAL_OK)
    {
        /* Initialize Device */
        if(HAL_I2C_Mem_Write(imu6050->I2CInterface, imu6050->deviceAddress, deviceConfAddress, I2C_MEMADD_SIZE_8BIT, &initDevData, writeSize, timeout) != HAL_OK)
        {
          _Error_Handler(__FILE__, __LINE__);
        }
        HAL_Delay(5);

        /* Configure Accelerometers */
        if(HAL_I2C_Mem_Write(imu6050->I2CInterface, imu6050->deviceAddress, accelConfAddress, I2C_MEMADD_SIZE_8BIT, &initAccData, writeSize, timeout) != HAL_OK)
        {
          _Error_Handler(__FILE__, __LINE__);
        }
        HAL_Delay(5);

        /* Configure Gyrometers*/
        if(HAL_I2C_Mem_Write(imu6050->I2CInterface, imu6050->deviceAddress, gyroConfAddress, I2C_MEMADD_SIZE_8BIT, &initGyrData, writeSize, timeout) != HAL_OK)
        {
          _Error_Handler(__FILE__, __LINE__);
        }
    }

}

void MPU6050_geData(MPU6050Imu *imu6050)
{
    uint32_t deviceAddress = imu6050->deviceAddress;
    uint32_t memAddress = imu6050->memAddress;
    uint32_t size = imu6050->memSize;
    uint8_t *data = imu6050->lastData;

    /* Request and Get Data */
    if(HAL_I2C_IsDeviceReady(imu6050->I2CInterface, deviceAddress, trials, timeout) == HAL_OK)
    {
        if(HAL_I2C_Mem_Read(imu6050->I2CInterface, deviceAddress, memAddress, I2C_MEMADD_SIZE_8BIT, data, size, timeout) != HAL_OK)
        {
          _Error_Handler(__FILE__, __LINE__);
        }
    }

}

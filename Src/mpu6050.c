/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "mpu6050.h"

/* Private variables ---------------------------------------------------------*/
#define MPU6050_ADDRESS (0x68 << 1)

uint16_t timeout;

/* Private function prototypes -----------------------------------------------*/
// void mpu6050_requestData(MPU6050Imu *imu6050, uint32_t memAddress, uint8_t *buffer);

/* Body Functions ------------------------------------------------------------*/
void mpu6050_configDevice(MPU6050Imu *imu6050, uint32_t accelConfig, uint32_t gyroConfig)
{
    timeout = 100;
}

void mpu6050_geData(MPU6050Imu *imu6050)
{
    uint32_t count;
    uint32_t memAddress = imu6050->memAddress;
    uint32_t size = imu6050->memSize;
    uint8_t *data = imu6050->lastData;

    for(count = 0; count < size; count++)
    {
        if(HAL_I2C_Mem_Read(imu6050->I2CInterface, memAddress, 0x3B, I2C_MEMADD_SIZE_8BIT, data, 1, 100) != HAL_OK)
        {
          _Error_Handler(__FILE__, __LINE__);
        }
        memAddress++;
        data++;
    }
}

// mpu6050_requestData(MPU6050Imu *imu6050, uint32_t memAddress, uint8_t *buffer, uint32_t size)
// {
//     int count;
//     uint8_t *data = buffer;

//     for(count = 0; count < size; count++)
//     {
//         if(HAL_I2C_Mem_Read(mpu6050->I2CInterface, memAddress, 0x3B, I2C_MEMADD_SIZE_8BIT, data, 1, 100) != HAL_OK)
//         {
//           _Error_Handler(__FILE__, __LINE__);
//         }
//         memAddress++;
//         data++;
//     }
// }


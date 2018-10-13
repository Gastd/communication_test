#include "stm32f1xx_hal.h"
#include "memsense_nanoimu.h"

uint16_t trials, timeout;

void NANOIMU_configDevice(MEMSenseImu* nanoImu, UART_HandleTypeDef* interface)
{
    timeout = 100;
    nanoImu->messageSize = 37;
    nanoImu->UARTInterface = interface;
}

void NANOIMU_geData(MEMSenseImu* nanoImu)
{
    if(HAL_UART_Receive(nanoImu->UARTInterface, nanoImu->data, nanoImu->messageSize, timeout) != HAL_OK)
    {
      Error_Handler();
    }
}

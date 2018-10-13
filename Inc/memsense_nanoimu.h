typedef struct
{
    UART_HandleTypeDef* UARTInterface;

    uint16_t messageSize;
    uint8_t data[37];
}MEMSenseImu;


void NANOIMU_configDevice(MEMSenseImu* nanoImu, UART_HandleTypeDef* interface);
void NANOIMU_geData(MEMSenseImu* nanoImu);

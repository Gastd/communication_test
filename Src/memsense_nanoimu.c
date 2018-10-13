/**
 ******************************************************************************
 * @file      imu-bytes.h
 * @author    Gabriel F P Araujo
 * @date      17/09/2011
 ******************************************************************************
 *
 * @attention Copyright (C) 2018
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Departamento de Engenharia Elétrica (ENE)
 * @attention Universidade de Brasília (UnB)
 *
 *
 *
 ******************************************************************************
 *
 ** ### MEMSense NanoIMU Specifications ###
 *
 *  Serial port:
 *
 *  (#) Speed                   115200
 *  (#) Parity                  None
 *  (#) Bits                    8
 *  (#) Stopbits                1
 *  (#) Flow control            None
 *  (#) Output Rate             150 Hz
 *
 *  Gyrometer:
 *
 *  (#) Dynamic Range           +-300 º/s
 *  (#) Offset                  +-1.5 º/s
 *  (#) Cross-axis sensitivity  +-1 %
 *  (#) Nonlinearity            +-0.1 % of FS (Best fit straight line)
 *  (#) Noise                   0.56 (max 0.95) º/s, sigma
 *  (#) Digital Sensitivity     DS_GYR
 *  (#) Bandwidth               50 Hz
 *
 *  Accelerometer:
 *
 *  (#) Dynamic Range           +-2 g
 *  (#) Offset                  +-30 mg
 *  (#) Nonlinearity            +-0.4 (max +-1.0) % of FS
 *  (#) Noise                   0.6 (max 0.8) mg, sigma
 *  (#) Digital Sensitivity     DS_ACC
 *  (#) Bandwidth               50 Hz
 *
 *  Magnetometer:
 *
 *  (#) Dynamic Range           +-1.9 gauss
 *  (#) Drift                   2700 ppm/ºC
 *  (#) Nonlinearity            +-0.5 % of FS (Best fit straight line)
 *  (#) Noise                   0.00056 (max 0.0015) gauss, sigma
 *  (#) Digital Sensitivity     DS_MAG
 *  (#) Bandwidth               50 Hz
 *
 *  Thermometer:
 *
 *  (#) Digital Sensitivity     DS_TMP
 */


#include "stm32f1xx_hal.h"
#include "memsense_nanoimu.h"

// Default values
#define D_SYNC      0xFF
#define D_MSG_SIZE  0x26
#define D_DEV_ID    0xFF
#define D_MSG_ID    0x14

#define IMU_SYNC_ST         0
#define IMU_HEADER_ST       1
#define IMU_PAYLOAD_ST      2
#define IMU_CHECKSUM_ST     3

#define BYTE_SIZE_2READ     1

uint16_t timeout;

int8_t NANOIMU_checksum(uint8_t *data, uint8_t chksum);

void NANOIMU_configDevice(MEMSenseImu* nanoImu, UART_HandleTypeDef* interface)
{
    timeout = 100;
    nanoImu->messageSize = IMU_PACKET_SIZE;
    nanoImu->UARTInterface = interface;
}

void NANOIMU_geData(MEMSenseImu* nanoImu)
{
    int32_t i;
    int32_t data_ready = 0;

    // State machine variables
    int32_t b = 0, s = IMU_SYNC_ST;

    // Storage for data read from serial port
    uint8_t data_read;

    // Try to sync with IMU and get latest data packet, up to MAX_BYTES read until failure
    for(i = 0; (!data_ready)&&(i < MAX_BYTES); i++)
    {
        // Read data from serial port
        if(HAL_UART_Receive(nanoImu->UARTInterface, &data_read, BYTE_SIZE_2READ, timeout) != HAL_OK)
        {
            Error_Handler();
        }

        // Parse IMU packet (User Guide, p.7)
        switch(s)
        {
            case IMU_SYNC_ST:
            {
                // State logic: Packet starts with 4 sync bytes with value 0xFF
                if(data_read == D_SYNC)
                {
                    nanoImu->data[b] = data_read;
                    b++;
                }
                else
                    // Out of sync, reset
                    b = 0;

                // State transition: I have reached the MSG_SIZE byte without resetting
                if(b == MSG_SIZE)
                    s = IMU_HEADER_ST;
            }
            break;

            case IMU_HEADER_ST:
            {
                // State logic: MSG_SIZE, DEV_ID and MSG_ID have default values
                if((b == MSG_SIZE) && (data_read == D_MSG_SIZE))
                {
                    nanoImu->data[b] = data_read;
                    b++;
                }
                else if((b == DEV_ID) && (data_read == D_DEV_ID))
                {
                    nanoImu->data[b] = data_read;
                    b++;
                }
                else if((b == MSG_ID) && (data_read == D_MSG_ID))
                {
                    nanoImu->data[b] = data_read;
                    b++;
                }
                else
                {
                    // Invalid MSG_SIZE, DEV_ID or MSG_ID, reset
                    b = 0;
                    s = IMU_SYNC_ST;
                }

                // State transition: I have reached the TIME_MSB byte without resetting
                if(b == TIME_MSB)
                    s = IMU_PAYLOAD_ST;

            }
            break;

            case IMU_PAYLOAD_ST:
            {
                // State logic: Grab data until you reach the checksum byte
                nanoImu->data[b] = data_read;
                b++;

                // State transition: I have reached the checksum byte
                if(b == CHECKSUM)
                    s = IMU_CHECKSUM_ST;
            }
            break;

            case IMU_CHECKSUM_ST:
            {
                // State logic: If checksum is OK, grab data
                if(NANOIMU_checksum(nanoImu->data, data_read))
                {
                    nanoImu->data[b] = data_read;
                    // decode();
                    data_ready = 1;
                }

                // State transition: Unconditional reset
                b = 0;
                s = IMU_SYNC_ST;
            }
            break;
        }
    }
}

int8_t NANOIMU_checksum(uint8_t *data, uint8_t chksum)
{
    uint8_t sum = 0;

    for(uint8_t i = 0; i < D_MSG_SIZE-1; i++)
        sum += data[i];

    if(sum != chksum)
    {
        return 0;
    }
    else
        return 1;
}

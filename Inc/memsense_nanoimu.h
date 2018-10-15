/**
 ******************************************************************************
 * @file      imu-bytes.h
 * @author    Gabriel F P Araujo
 * @date      15/10/2018
 ******************************************************************************
 *
 * @attention Copyright (C) 2018
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Departamento de Engenharia Elétrica (ENE)
 * @attention Universidade de Brasília (UnB)
 */


#include "memsense_nanoimu_bytes.h"

#define IMU_PACKET_SIZE 38
#define NANOIMU_BPS     115200
#define MAX_BYTES       100

typedef struct
{
    UART_HandleTypeDef* UARTInterface;

    uint16_t messageSize;
    uint8_t data[IMU_PACKET_SIZE];
}MEMSenseImu;


void NANOIMU_configDevice(MEMSenseImu* nanoImu, UART_HandleTypeDef* interface);
void NANOIMU_geData(MEMSenseImu* nanoImu);

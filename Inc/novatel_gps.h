/**
 ******************************************************************************
 * @file      novatel_gps.h
 * @author    Gabriel F P Araujo
 * @date      15/10/2018
 ******************************************************************************
 *
 * @attention Copyright (C) 2018
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Departamento de Engenharia Elétrica (ENE)
 * @attention Universidade de Brasília (UnB)
 */


#define D_HDR_LEN       28
#define GPS_PACKET_SIZE 500

typedef struct
{
    UART_HandleTypeDef* UARTInterface;

    uint16_t headerSize;
    uint16_t messageSize;

    uint32_t status;

    uint8_t headerData[D_HDR_LEN];
    uint8_t messageData[GPS_PACKET_SIZE];
}NovatelGPS;


void NOVATELGPS_configDevice(NovatelGPS* gps, UART_HandleTypeDef* interface);
void NOVATELGPS_geData(NovatelGPS* gps);

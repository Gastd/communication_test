/**
 ******************************************************************************
 * @file      novatel_gps.c
 * @author    Gabriel F P Araujo
 * @date      15/10/2018
 ******************************************************************************
 *
 * @attention Copyright (C) 2018
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Departamento de Engenharia Elétrica (ENE)
 * @attention Universidade de Brasília (UnB)
 */

#include <string.h>
#include "stm32f1xx_hal.h"
#include "novatel_gps.h"

/* Definitions */

// GPS States
#define GPS_SYNC_ST         0
#define GPS_HEADER_ST       1
#define GPS_PAYLOAD_ST      2
#define GPS_CRC_ST          3

// Serial port
#define TIMEOUT_US      100000
#define MAX_BYTES       1000

// Binary Message Format = 3 sync + 25 header + variable data + 4 CRC

/* Binary Header */

// Header byte order/format
#define SYNC0           0   // char
#define SYNC1           1   // char
#define SYNC2           2   // char
#define HDR_LEN         3   // uchar
#define MSG_ID          4   // ushort
#define MSG_TYPE        6   // char
#define PORT_ADDR       7   // uchar
#define MSG_LEN         8   // ushort
#define SEQ_NUM         10  // ushort
#define IDLE_T          12  // uchar
#define T_STATUS        13  // enum
#define T_WEEK          14  // ushort
#define T_MS            16  // GPSec (ulong)
#define GPS_STATUS      20  // ulong
#define RESERVED        24  // ushort
#define SW_VERS         26  // ushort
#define DATA            28  // variable

// Default values
#define D_SYNC0         0xAA
#define D_SYNC1         0x44
#define D_SYNC2         0x12

// Multi-byte sizes
#define S_MSG_ID        2
#define S_MSG_LEN       2
#define S_SEQ_NUM       2
#define S_T_WEEK        2
#define S_T_MS          4
#define S_GPS_STATUS    4
#define S_RESERVED      2
#define S_SW_VERS       2
#define S_CRC           4

/* Log Message IDs */
#define BESTPOS         42
#define GPGGA           218
#define GPGSA           221
#define GPRMC           225
#define BESTXYZ         241

#define BYTE_SIZE_2READ     1
#define BYTE_SIZE_2SEND     1

uint16_t timeout;

uint32_t CalculateBlockCRC32(uint32_t ulCount, uint8_t *ucBuffer);
uint32_t ByteSwap (uint32_t n);
uint32_t CRC32Value(int i);
void NOVATELGPS_configure(NovatelGPS* gps);
void NOVATELGPS_command(NovatelGPS* gps, const char* command);
int8_t NOVATELGPS_getApproxTime(uint32_t* gps_week_1024, uint32_t* gps_secs);

void NOVATELGPS_configDevice(NovatelGPS* gps, UART_HandleTypeDef* interface)
{
    timeout = 100;
    gps->UARTInterface = interface;
    gps->headerSize = D_HDR_LEN;

    // GPS position should be set approximately (hard coded to LARA/UnB coordinates)
    NOVATELGPS_command(gps, "SETAPPROXPOS -15.765824 -47.872109 1024");

    // char buf[100];
    // double time_f = static_cast<double>(1.0/rate_);
    // sprintf(buf, "LOG BESTXYZB ONTIME 0.05");
    NOVATELGPS_command(gps, "LOG BESTXYZB ONTIME 0.05");
}

void NOVATELGPS_geData(NovatelGPS* gps)
{
    int data_ready = 0;
    // State machine variables
    int b = 0, bb = 0, s = GPS_SYNC_ST;

    // Storage for data read from serial port
    uint8_t data_read;
    uint8_t* gps_data = gps->messageData;

    // Multi-byte data
    uint16_t msg_id, msg_len = 0, t_week;
    uint32_t t_ms, crc_from_packet;

    // Try to sync with GPS and get latest data packet, up to MAX_BYTES read until failure
    for(int i = 0; (!data_ready)&&(i < MAX_BYTES); i++)
    {
        // Read data from UART
        if(HAL_UART_Receive(gps->UARTInterface, &data_read, BYTE_SIZE_2READ, timeout) != HAL_OK)
        {
            Error_Handler();
        }


        // Parse GPS packet (Firmware Reference Manual, p.22)
        switch(s)
        {
            case GPS_SYNC_ST:
            {
                // State logic: Packet starts with 3 sync bytes with values 0xAA, 0x44, 0x12
                switch(b)
                {
                    case SYNC0:
                    {
                        if(data_read == D_SYNC0)
                        {
                            gps_data[b] = data_read;
                            b++;
                        }
                        else
                            // Out of sync, reset
                            b = 0;
                        break;
                    }

                    case SYNC1:
                    {
                        if(data_read == D_SYNC1)
                        {
                            gps_data[b] = data_read;
                            b++;
                        }
                        else
                            // Out of sync, reset
                            b = 0;
                        break;
                    }

                    case SYNC2:
                    {
                        if(data_read == D_SYNC2)
                        {
                            gps_data[b] = data_read;
                            b++;
                        }
                        else
                            // Out of sync, reset
                            b = 0;
                        break;
                    }
                }
                // State transition: I have reached the HDR_LEN byte without resetting
                if(b == HDR_LEN)
                    s = GPS_HEADER_ST;
            }
            break;

            case GPS_HEADER_ST:
            {
                // State logic: HDR_LEN, MSG_ID, MSG_TYPE, PORT_ADDR, MSG_LEN, SEQ_NUM, IDLE_T, T_STATUS, T_WEEK, T_MS, GPS_STATUS, RESERVED, SW_VERS
                switch(b)
                {
                    case HDR_LEN:
                    {
                        if(data_read == D_HDR_LEN)
                        {
                            gps_data[b] = data_read;
                            b++;
                        }
                        else
                        {
                            // Invalid HDR_LEN, reset
                            Error_Handler();
                            b = 0;
                            s = GPS_SYNC_ST;
                        }
                        break;
                    }

                    case MSG_ID:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data[b+bb] = data_read;
                        bb++;

                        if(bb == S_MSG_ID)
                        {
                            // Merge bytes and process
                            memcpy(&msg_id, &gps_data[MSG_ID], sizeof(uint16_t));

                            // Update byte indices
                            bb = 0;
                            b += S_MSG_ID;
                        }
                        break;
                    }

                    case MSG_TYPE:
                    {
                        gps_data[b] = data_read;
                        b++;
                        break;
                    }

                    case PORT_ADDR:
                    {
                        gps_data[b] = data_read;
                        b++;
                        break;
                    }

                    case MSG_LEN:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data[b+bb] = data_read;
                        bb++;

                        if(bb == S_MSG_LEN)
                        {
                            // Merge bytes and process
                            memcpy(&msg_len, &gps_data[MSG_LEN], sizeof(uint16_t));
                            // ROS_INFO("Message Length = %d", msg_len);
                            // I was having some problems with (msg_len == 0)...
                            if(msg_len != 0)
                            {
                                // Update byte indices
                                bb = 0;
                                b += S_MSG_LEN;
                            }
                            else
                            {
                                // Something wrong, reset
                                bb = 0;
                                b = 0;
                                s = GPS_SYNC_ST;
                            }
                        }
                        break;
                    }

                    case SEQ_NUM:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data[b+bb] = data_read;
                        bb++;

                        if(bb == S_SEQ_NUM)
                        {
                            // Merge bytes and process
                            //memcpy(&seq_num, &gps_data[SEQ_NUM], sizeof(uint16_t));

                            // Update byte indices
                            bb = 0;
                            b += S_SEQ_NUM;
                        }
                        break;
                    }

                    case IDLE_T:
                    {
                        gps_data[b] = data_read;
                        b++;
                        break;
                    }

                    case T_STATUS:
                    {
                        gps_data[b] = data_read;
                        b++;
                        break;
                    }

                    case T_WEEK:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data[b+bb] = data_read;
                        bb++;

                        if(bb == S_T_WEEK)
                        {
                            // Merge bytes and process
                            memcpy(&t_week, &gps_data[T_WEEK], sizeof(uint16_t));

                            // Update byte indices
                            bb = 0;
                            b += S_T_WEEK;
                        }
                        break;
                    }

                    case T_MS:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data[b+bb] = data_read;
                        bb++;

                        if(bb == S_T_MS)
                        {
                            // Merge bytes and process
                            memcpy(&t_ms, &gps_data[T_MS], sizeof(uint32_t));

                            // Update byte indices
                            bb = 0;
                            b += S_T_MS;
                        }
                        break;
                    }

                    case GPS_STATUS:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data[b+bb] = data_read;
                        bb++;

                        if(bb == S_GPS_STATUS)
                        {
                            // Merge bytes and process
                            memcpy(&(gps->status), &gps_data[GPS_STATUS], sizeof(uint32_t));

                            // Update byte indices
                            bb = 0;
                            b += S_GPS_STATUS;
                        }
                        break;
                    }

                    case RESERVED:
                    {   
                        // Index bb is for bytes in multi-byte variables
                        // Skip this section, no useful data
                        bb++;

                        if(bb == S_RESERVED)
                        {
                            // Update byte indices
                            bb = 0;
                            b += S_RESERVED;
                        }
                        break;
                    }

                    case SW_VERS:
                    {
                        // Index bb is for bytes in multi-byte variables
                        gps_data[b+bb] = data_read;
                        bb++;

                        if(bb == S_SW_VERS)
                        {
                            // Merge bytes and process
                            //memcpy(&sw_vers, &gps_data[SW_VERS], sizeof(uint16_t));

                            // Update byte indices
                            bb = 0;
                            b += S_SW_VERS;
                        }
                        break;
                    }
                }

                // State transition: I have reached the DATA bytes without resetting
                if(b == DATA)
                    s = GPS_PAYLOAD_ST;  
            }
            break;

            case GPS_PAYLOAD_ST:
            {
                // State logic: Grab data until you reach the CRC bytes
                gps_data[b+bb] = data_read;
                bb++;

                // State transition: I have reached the CRC bytes
                if(bb == msg_len)
                {
                    // Bytes are decoded after CRC check

                    // Update byte indices
                    bb = 0;
                    b += msg_len;
                    s = GPS_CRC_ST;
                }
            }
            break;

            case GPS_CRC_ST:
            {
                // Index bb is for bytes in multi-byte variables
                gps_data[b+bb] = data_read;
                bb++;
                if(bb == S_CRC)
                {
                    uint32_t crc_calculated;
                    // Grab CRC from packet
                    crc_from_packet = (uint32_t)((gps_data[b+3] << 24) | (gps_data[b+2] << 16) | (gps_data[b+1] << 8) | gps_data[b]);

                    // Calculate CRC from packet (b = packet size)
                    // crc_calculated = CalculateBlockCRC32(b, &gps_data[0]); // GAMBIARRA
                    crc_calculated = CalculateBlockCRC32(b, gps_data);
                    // crc_calculated = CalculateBlockCRC32(b, gps_data.data()); // C++11

                    // Compare them to see if valid packet 
                    if(crc_from_packet != ByteSwap(crc_calculated))
                    {
                        // ROS_ERROR("CRC does not match (%0x != %0x)", crc_from_packet, ByteSwap(crc_calculated));
                    }
                    // decode(msg_id);
                    data_ready = 1;

                    // State transition: Unconditional reset
                    bb = 0;
                    b = 0;
                    s = GPS_SYNC_ST;
                }
            }
            break;
        }
    }

    // b = packet size
    gps->messageSize = b;
}

void NOVATELGPS_configure(NovatelGPS* gps)
{
    // GPS should be configured to 9600 and change to 115200 during execution
    char buffer[100];
    // sprintf(buffer, "COM COM1,%d,N,8,1,N,OFF,ON", OLD_BPS);
    // sprintf(buffer, "COM COM1,%d,N,8,1,N,OFF,ON", BPS);
    // command(buffer);
    // command("COM COM2,115200,N,8,1,N,OFF,ON");

    uint32_t gps_week_1024, gps_secs;
    // GPS time should be set approximately
    if(!NOVATELGPS_getApproxTime(&gps_week_1024, &gps_secs))
    {
        Error_Handler();
    }
    else
    {
        // sprintf(buffer, "SETAPPROXTIME %lu %f", gps_week_1024, gps_secs);
        NOVATELGPS_command(gps, buffer);
    }

    // GPS position should be set approximately (hard coded to LARA/UnB coordinates)
    NOVATELGPS_command(gps, "SETAPPROXPOS -15.765824 -47.872109 1024");
    // command("SETAPPROXPOS -15.791372 -48.0227546 1178");
}

void NOVATELGPS_command(NovatelGPS* gps, const char* command)
{
    int32_t i;
    int32_t len = strlen(command);

    for(i = 0; i < len; i++)
    {
        if(HAL_UART_Transmit(gps->UARTInterface, (uint8_t*) &command[i], BYTE_SIZE_2SEND, timeout) != HAL_OK)
        {
            Error_Handler();
        }
        HAL_Delay(5);
    }

    // Sending Carriage Return character
    if(HAL_UART_Transmit(gps->UARTInterface, (uint8_t*) "\r", BYTE_SIZE_2SEND, timeout) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_Delay(5);

    // Sending Line Feed character
    if(HAL_UART_Transmit(gps->UARTInterface, (uint8_t*) "\n", BYTE_SIZE_2SEND, timeout) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_Delay(5);
}

// Calculate GPS week number and seconds, within 10 minutes of actual time, for initialization
int8_t NOVATELGPS_getApproxTime(uint32_t* gps_week_1024, uint32_t* gps_secs)
{
    // Time difference between January 1, 1970 and January 6, 1980
    // Source: http://www.timeanddate.com/date/durationresult.html?d1=1&m1=jan&y1=1970&h1=0&i1=0&s1=0&d2=6&m2=jan&y2=1980&h2=0&i2=0&s2=0
    const uint32_t time_diff = 315964800;

    // # of seconds in a week
    const uint32_t secs_in_week = 604800;

    // Unix time
    time_t cpu_secs = 0;

    // Unprocessed GPS time
    uint32_t gps_time;
    uint32_t gps_week;

    // Get time
    // cpu_secs = time(NULL);

    // Offset to GPS time and calculate weeks and seconds
    gps_time = cpu_secs - time_diff;
    gps_week = gps_time / secs_in_week;
    *gps_week_1024 = gps_week % 1024;
    *gps_secs = gps_time % secs_in_week;

    if((gps_week != 0) && (*gps_secs != 0))
        return 1;
    else
        return 0;
}

/*************************** CRC functions (Firmware Reference Manual, p.32 + APN-030 Rev 1 Application Note) ***************************/

inline uint32_t ByteSwap (uint32_t n)
{ 
   return ( ((n & 0x000000FF)<<24) + ((n & 0x0000FF00)<<8) + ((n & 0x00FF0000)>>8) + (( n & 0xFF000000)>>24) );
}

#define CRC32_POLYNOMIAL    0xEDB88320L

/* --------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions.
-------------------------------------------------------------------------- */
inline uint32_t CRC32Value(int i)
{
    int j;
    uint32_t ulCRC;
    ulCRC = i;

    for ( j = 8 ; j > 0; j-- )
    {
        if ( ulCRC & 1 )
            ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
        else
            ulCRC >>= 1;
    }
    return ulCRC;
}

/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
-------------------------------------------------------------------------- */
inline uint32_t CalculateBlockCRC32
(
    uint32_t ulCount,  /* Number of bytes in the data block */
    uint8_t *ucBuffer /* Data block */
)
{
    uint32_t ulTemp1;
    uint32_t ulTemp2;
    uint32_t ulCRC = 0;

    while ( ulCount-- != 0 )
    {
        ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value( ((int) ((ulCRC) ^ (*ucBuffer)) ) & 0xff );
        ucBuffer++;
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return( ulCRC );
}

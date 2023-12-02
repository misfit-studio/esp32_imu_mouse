#include <Arduino.h>

static void serial_print_hex(uint8_t *data, uint8_t size)
{
    for (int i = 0; i < size; i++)
    {
        Serial.printf("%02x", data[i]);
        Serial.print(" ");
    }
    Serial.println();
}

#define CRC_POLY 0x1021;
static uint16_t calc_crc(uint8_t *data, uint8_t size)
{
    int i, j;
    uint16_t crc = 0;
    for (i = 0; i < size; i++)
    {
        unsigned int xr = data[i] << 8;
        crc = crc ^ xr;

        for (j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1);
                crc = crc ^ CRC_POLY;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }
    crc = crc & 0xFFFF;
    return crc;
}
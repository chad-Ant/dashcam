#pragma once
#include <Adafruit_ILI9341.h>

// IPS panel power, VCOM, inversion and gamma from LCDWiki's ILI9341V_Init.txt.
inline void es3ConfigureIpsPanel(Adafruit_ILI9341& panel)
{
    uint8_t data0[] = {0x0, 0xc1, 0x30};
    panel.sendCommand(0xcf, data0, sizeof(data0));
    uint8_t data1[] = {0x64, 0x3, 0x12, 0x81};
    panel.sendCommand(0xed, data1, sizeof(data1));
    uint8_t data2[] = {0x85, 0x0, 0x78};
    panel.sendCommand(0xe8, data2, sizeof(data2));
    uint8_t data3[] = {0x39, 0x2c, 0x0, 0x34, 0x2};
    panel.sendCommand(0xcb, data3, sizeof(data3));
    uint8_t data4[] = {0x20};
    panel.sendCommand(0xf7, data4, sizeof(data4));
    uint8_t data5[] = {0x0, 0x0};
    panel.sendCommand(0xea, data5, sizeof(data5));
    uint8_t data6[] = {0x13};
    panel.sendCommand(0xc0, data6, sizeof(data6));
    uint8_t data7[] = {0x13};
    panel.sendCommand(0xc1, data7, sizeof(data7));
    uint8_t data8[] = {0x22, 0x35};
    panel.sendCommand(0xc5, data8, sizeof(data8));
    uint8_t data9[] = {0xbd};
    panel.sendCommand(0xc7, data9, sizeof(data9));
    panel.sendCommand(0x21);
    uint8_t data11[] = {0xa, 0xa2};
    panel.sendCommand(0xb6, data11, sizeof(data11));
    uint8_t data12[] = {0x1, 0x30};
    panel.sendCommand(0xf6, data12, sizeof(data12));
    uint8_t data13[] = {0x0, 0x1b};
    panel.sendCommand(0xb1, data13, sizeof(data13));
    uint8_t data14[] = {0x0};
    panel.sendCommand(0xf2, data14, sizeof(data14));
    uint8_t data15[] = {0x1};
    panel.sendCommand(0x26, data15, sizeof(data15));
    uint8_t data16[] = {0xf, 0x35, 0x31, 0xb, 0xe, 0x6, 0x49, 0xa7, 0x33, 0x7, 0xf, 0x3, 0xc, 0xa, 0x0};
    panel.sendCommand(0xe0, data16, sizeof(data16));
    uint8_t data17[] = {0x0, 0xa, 0xf, 0x4, 0x11, 0x8, 0x36, 0x58, 0x4d, 0x7, 0x10, 0xc, 0x32, 0x34, 0xf};
    panel.sendCommand(0xe1, data17, sizeof(data17));
}


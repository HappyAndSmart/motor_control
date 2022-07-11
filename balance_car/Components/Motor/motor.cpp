

#include "motor.h"

uint8_t * Motor::fillCanBuf(uint8_t * buf) const
{
    buf[id*2-2] = givenCurrent >> 8;
    buf[id*2-1] = givenCurrent;
    return buf;
}

void Motor::updateInfo(const uint8_t* rxData)
{
    fbEcd = ((uint16_t) rxData[0] << 8) | rxData[1];
    fbSpeed = ((int16_t) rxData[2] << 8) | rxData[3];
    fbCurrent = ((int16_t) rxData[4] << 8) | rxData[5];
    fbTemp = rxData[6];
}

int16_t Motor::getNormalizedEcd(int16_t rawEcd, int16_t centerOffset) {
    int16_t tmp = 0;
    if (centerOffset >= 4096)
    {
        if (rawEcd > centerOffset - 4096)
        {
            tmp = rawEcd - centerOffset;
        }
        else
        {
            tmp = rawEcd + 8192 - centerOffset;
        }
    }
    else
    {
        if (rawEcd > centerOffset + 4096)
        {
            tmp = rawEcd - 8192 - centerOffset;
        }
        else
        {
            tmp = rawEcd - centerOffset;
        }
    }

    return tmp;
}
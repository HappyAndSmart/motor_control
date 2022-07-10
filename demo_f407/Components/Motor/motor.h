//
// Created by nian on 2022/3/10.
//

#ifndef FIREFLYRTS_FIRMWARE_MOTOR_H
#define FIREFLYRTS_FIRMWARE_MOTOR_H

#include "can.h"
#include "cmsis_os.h"
#include "main.h"


#define ENCODER_RATIO ( 8192.0f / 360.0f )
#define RAD_TO_DEG (180.0f / PI)

class Motor
{
protected:
    CAN_HandleTypeDef *motorCan{};
    uint8_t id;
    int16_t centerOffset{};        //given center ECD value -- 0 degree

public:
    uint16_t fbEcd{};      // encoder value
    int16_t fbSpeed{};     // rpm
    int16_t fbCurrent{};   // N*M  A  V
    uint8_t fbTemp{};
    float fbAngle{};       // degree

    int16_t normalizedEcd=0;  //Relative encoder values in the positive direction

    int16_t givenCurrent{};
    int16_t givenSpeed{};
    int16_t givenEcd{};

    int16_t getNormalizedEcd(int16_t rawEcd, int16_t centerOffset);  // Relative encoder values in the positive direction
    virtual uint8_t * fillCanBuf(uint8_t * buf) const;
    virtual void updateInfo(const uint8_t* rxData);

};

#endif //FIREFLYRTS_FIRMWARE_MOTOR_H

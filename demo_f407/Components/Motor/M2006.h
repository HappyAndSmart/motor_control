//
// Created by nian on 2022/3/16.
//

#ifndef FIREFLYRTS_FIRMWARE_M2006_H
#define FIREFLYRTS_FIRMWARE_M2006_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor.h"
#include "pid.h"

class M2006 : public Motor {
private:
    static const uint8_t motorNum = 3;
    static const uint8_t reductionRatio = 36; //reductionRatio is 36:1
    uint32_t  protocolID1 = 0x200; //when ID is 1~4
    uint32_t  protocolID2 = 0x1FF; //when ID is 5~8
    uint8_t  canDLC = 8;          //set four ID's current in 8 bytes
    //speed control
    PIDParaStruct speedRingPara = {
            7, 0, 0.9,
            6000,
            13000,
            500
    };
    PIDParaStruct posRingPara = {
            0.2, 0.0000, 0.15,
            8200,
            13000,
            500
    };
public:
    PID m2006PIDSpeedRing = PID(&speedRingPara);
    PID m2006PIDPosRing = PID(&posRingPara);
    uint32_t protocolIDrx = 0x200; //CAN 's receive ID

    M2006(uint8_t id_t,CAN_HandleTypeDef *hcan_t);
    void setExpSpeed(int16_t  speed_t);
    void updateInfo(const uint8_t* rxData) override;

    uint8_t * fillCanBuf(uint8_t * buf) const override;
    void sendCanCmd(uint8_t * buf);


};

#ifdef __cplusplus
}
#endif
#endif //FIREFLYRTS_FIRMWARE_M2006_H

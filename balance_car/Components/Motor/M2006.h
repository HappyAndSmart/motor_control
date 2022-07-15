//
// Created by Happy and Smart on 2022/7/11.
//

#ifndef BALANCE_CAR_M2006_H
#define BALANCE_CAR_M2006_H
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
          //  7.0, 0, 1.0,
          6.0,1,3.0,
            20000,
            10000,
            500
    };
    PIDParaStruct posRingPara = {
            //0.2, 0.0000, 0.15,
           // 1000,1.1,800,
          //120,0.21,80,
          80,5,0,
            1000,
            10000,
            2000
    };
public:
    PID m2006PIDSpeedRing = PID(&speedRingPara);
    PID m2006PIDPosRing = PID(&posRingPara);
    uint32_t protocolIDrx = 0x200; //CAN 's receive ID

    M2006(uint8_t id_t,CAN_HandleTypeDef *hcan_t);
    void setExpSpeed(int16_t  speed_t);
    void setExpAngle(float fb_angle);
    void updateInfo(const uint8_t* rxData) override;

    uint8_t * fillCanBuf(uint8_t * buf) const override;
    void sendCanCmd(uint8_t * buf);


};

#ifdef __cplusplus
}
#endif
#endif //BALANCE_CAR_M2006_H

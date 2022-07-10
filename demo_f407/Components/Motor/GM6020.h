//
// Created by nian on 2022/3/10.
//

#ifndef FIREFLYRTS_FIRMWARE_GM6020_H
#define FIREFLYRTS_FIRMWARE_GM6020_H

#include "motor.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

class GM6020 : public Motor {
private:
    static const uint8_t motorNum = 1;
    uint32_t protocolID1 = 0x1FF; //when ID is 1~4
    uint32_t protocolID2 = 0x2FF; //when ID is 5~8
    uint8_t canDLC = 8;

    PIDParaStruct speedRingPara = {
            39.7, 6.5, 0.45,
            650,
            30000,
            6000
    };
    PIDParaStruct angleRingPara = {
            0.35, 0, 0.9,
            8200,
            300,
            100
    };
public:
    PID gm6020PIDSpeedRing = PID(&speedRingPara); //speed PID
    PID gm6020PIDAngleRing = PID(&angleRingPara); //position PID
    uint32_t protocolIDrx = 0x204; //CAN 's receive ID

    GM6020(uint8_t id_t, CAN_HandleTypeDef *hcan_t, int16_t centerOffsetEcd);
    void setExpSpeed(int16_t speed_t);
    void updateInfo(const uint8_t *rxData) override;
    void setExpAngle(float angle_t);    //-180 ~ 180

    void sendCanCmd(uint8_t *buf);
    uint8_t *fillCanBuf(uint8_t *buf) const override;


};

#ifdef __cplusplus
}
#endif
#endif //FIREFLYRTS_FIRMWARE_GM6020_H

//
// Created by nian on 2022/3/12.
//

#ifndef FIREFLYRTS_FIRMWARE_M3508_H
#define FIREFLYRTS_FIRMWARE_M3508_H

#include "motor.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

class M3508 : public Motor {
private:
    static const uint8_t motorNum = 1;
    uint32_t protocolID1 = 0x200; //when ID is 1~4
    uint32_t protocolID2 = 0x1FF; //when ID is 5~8
    uint8_t canDLC = 8;

    PIDParaStruct speedRingPara = {
            15, 0.3, 12.5,
            800,
            16385,
            3000
    };
public:
    uint16_t lastEcd{};
    uint16_t offsetEcd{};  // store the initial encoder values when starting
    int32_t roundCnt{};    // number of turns
    int32_t totalEcd{};    // total ecd to calculate the total angle
    int32_t totalAngle{};  // total angle to calculate the distance
    uint32_t msgCnt{};     // init offset time stamp
    int32_t ecdRawRate{};  // encoder value difference per frame
    uint32_t protocolIDrx = 0x200; //CAN 's receive ID

    PID m3508PIDSpeedRing = PID(&speedRingPara); //speed PID

    M3508(uint8_t id_t, CAN_HandleTypeDef *hcan_t, int16_t centerOffsetEcd);
    void setExpSpeed(int16_t speed_t);
    void sendCanCmd(uint8_t *buf);
    uint8_t *fillCanBuf(uint8_t *buf) const override;
    void updateInfo(const uint8_t *rxData) override;
    void setExpAngle(float angle_t);    //-180 ~ 180
};

#ifdef __cplusplus
}
#endif
#endif //FIREFLYRTS_FIRMWARE_M3508_H

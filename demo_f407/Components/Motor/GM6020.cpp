//
// Created by szy on 2022/7/4.
//

#include "GM6020.h"
#include "cstring"

GM6020::GM6020(uint8_t id_t, CAN_HandleTypeDef *hcan_t, int16_t centerOffsetEcd) {
    id = id_t;
    motorCan = hcan_t;
    centerOffset = centerOffsetEcd;
    if(hcan_t->State == HAL_CAN_STATE_READY)
    {
        CAN_Filter_Init(hcan_t);
        HAL_CAN_Start(hcan_t);
        HAL_CAN_ActivateNotification(hcan_t, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}
//FIXME: speed PID control need improve
void GM6020::setExpSpeed(int16_t speed_t) {
    givenSpeed = speed_t;
    givenCurrent = gm6020PIDSpeedRing.PID_Output(this->fbSpeed, this->givenSpeed);
    uint8_t txbuff[8];
    memset(txbuff, 0, 8);
    sendCanCmd(fillCanBuf(txbuff));
}
void GM6020::setExpAngle(float angle_t) {
    givenEcd = (int16_t)(angle_t * ENCODER_RATIO + centerOffset) % 8192;
    int16_t speed_set;
    speed_set = gm6020PIDAngleRing.PID_Output(fbEcd, givenEcd);
    setExpSpeed(speed_set);
}

void GM6020::updateInfo(const uint8_t* rxData)
{
    fbEcd = ((uint16_t) rxData[0] << 8) | rxData[1];
    fbSpeed = ((int16_t) rxData[2] << 8) | rxData[3];
    fbCurrent = ((int16_t) rxData[4] << 8) | rxData[5];
    fbTemp = rxData[6];

    normalizedEcd = getNormalizedEcd(fbEcd, centerOffset);
    fbAngle = normalizedEcd  / ENCODER_RATIO;

}

uint8_t * GM6020::fillCanBuf(uint8_t *buf) const {
/**  Voltage controlled motors for GM6020   **/
    if(id <=4)
    {
        buf[id*2-2] = givenCurrent >> 8;       //Control current 8~15 bit corresponding to ID
        buf[id*2-1] = givenCurrent;            //Control current 0~7 bit corresponding to ID
    } else if (id <= 8)
    {
        buf[(id-4)*2-2] = givenCurrent >> 8;       //Control current 8~15 bit corresponding to ID
        buf[(id-4)*2-1] = givenCurrent;            //Control current 0~7 bit corresponding to ID
    }
    return buf;
}
void GM6020::sendCanCmd(uint8_t *buf) {
    if(id <=4)
        //when id is 1~4 ,the standard identifier is 0x1ff
        HAL_CAN_Transmit(motorCan,protocolID1,buf,canDLC);
    else if (id <= 7)
        //when id is 5~7 ,the standard identifier is 0x2ff
        HAL_CAN_Transmit(motorCan,protocolID2,buf,canDLC);
}


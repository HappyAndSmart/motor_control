//
// Created by nian on 2022/3/12.
//

#include "M3508.h"
#include "cstring"

M3508::M3508(uint8_t id_t, CAN_HandleTypeDef *hcan_t, int16_t centerOffsetEcd) {
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

void M3508::setExpSpeed(int16_t speed_t) {
    givenSpeed = speed_t;
    givenCurrent = m3508PIDSpeedRing.PID_Output(this->fbSpeed, this->givenSpeed);
    uint8_t txbuff[8];
    memset(txbuff, 0, 8);
    sendCanCmd(fillCanBuf(txbuff));
}

void M3508::updateInfo(const uint8_t* rxData)
{
    msgCnt++;
    if (msgCnt <= 50)    //init encoder value offset
    {
        fbEcd = ((uint16_t) rxData[0] << 8) | rxData[1];
        offsetEcd = fbEcd;
        return;
    }
    lastEcd = fbEcd;

    fbEcd = ((uint16_t) rxData[0] << 8) | rxData[1];
    fbSpeed = ((int16_t) rxData[2] << 8) | rxData[3];
    fbCurrent = ((int16_t) rxData[4] << 8) | rxData[5];
    fbTemp = rxData[6];
    /** Calculation of the encoder value and angle rotated after switch-on **/
    if (fbEcd - lastEcd > 4096)
    {
        roundCnt--;
        ecdRawRate = fbEcd - lastEcd - 8192;
    }
    else if (fbEcd - lastEcd < -4096)
    {
        roundCnt++;
        ecdRawRate = fbEcd - lastEcd + 8192;
    }
    else
    {
        ecdRawRate = fbEcd - lastEcd;
    }

    totalEcd = roundCnt * 8192 + fbEcd - offsetEcd;
    totalAngle = totalEcd / ENCODER_RATIO;
}

uint8_t * M3508::fillCanBuf(uint8_t *buf) const {
    /**  Current controlled motors for M3508   **/
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

void M3508::sendCanCmd(uint8_t *buf) {
    if(id <=4)
        //when id is 1~4 ,the standard identifier is protocolID1
        HAL_CAN_Transmit(motorCan,protocolID1,buf,canDLC);
    else if (id <= 7)
        //when id is 5~7 ,the standard identifier is protocolID2
        HAL_CAN_Transmit(motorCan,protocolID2,buf,canDLC);
}




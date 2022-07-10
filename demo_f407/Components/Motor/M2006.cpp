//
// Created by nian on 2022/3/16.
//

#include "M2006.h"
#include "can.h"
#include "cstring"

static uint8_t txbuff[8];


M2006::M2006(uint8_t id_t, CAN_HandleTypeDef *hcan_t) {
    id = id_t;
    motorCan = hcan_t;
    //if CAN is start,this has no use
    if(hcan_t->State == HAL_CAN_STATE_READY)
    {
        CAN_Filter_Init(hcan_t);
        HAL_CAN_Start(hcan_t);
        HAL_CAN_ActivateNotification(hcan_t, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}
void M2006::updateInfo(const uint8_t *rxData) {
    fbEcd = ((uint16_t) rxData[0] << 8) | rxData[1];
    fbSpeed = ((uint16_t) rxData[2] << 8) | rxData[3];
    fbCurrent = ((uint16_t) rxData[4] << 8) | rxData[5];
    fbAngle = fbEcd  / ENCODER_RATIO;
}
void M2006::setExpSpeed(int16_t  speed_t){
    givenSpeed = speed_t;
    givenCurrent = m2006PIDSpeedRing.PID_Output(fbSpeed, givenSpeed);
    memset(txbuff, 0, 8);
    sendCanCmd(fillCanBuf(txbuff));
}
uint8_t * M2006::fillCanBuf(uint8_t *buf) const {
/*******  when use two CAN and CAN1 ID is 0 ~ motor_CAN2ID-1,CAN2 ID is motor_CAN2ID ~ motorNum ***/
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
void M2006::sendCanCmd(uint8_t *buf) {
    if(id <=4)
        HAL_CAN_Transmit(motorCan,protocolID1,buf,canDLC);
    else if (id <= 8)
        //when id is 5~8 ,the standard identifier is 0x1ff
        HAL_CAN_Transmit(motorCan,protocolID2,buf,canDLC);
}


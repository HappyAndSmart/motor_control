//
// Created by nian on 2021/6/19.
//

#include "cmsis_os.h"
#include "can.h"
#include "main.h"
#include "usart.h"
#include "M2006.h"
#include "GM6020.h"
#include "M3508.h"
#include "cstring"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t rx_buff;
extern M2006 *m2006[3]; //three 2006
extern GM6020 *gm6020[2];
extern M3508 * m3508[4];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;

    uint8_t id;
    static uint8_t rxData[8];
    memset(rxData, 0, 8);
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);

    if (hcan == &hcan1)
    {
        id = rxHeader.StdId - m2006[0]->protocolIDrx;
        // update the relevant m2006 data
        m2006[id-1]->updateInfo(rxData);
        HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);

//        id = rxHeader.StdId - gm6020[0]->protocolIDrx;
//        gm6020[id -1]->updateInfo(rxData);
//        HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);

//        id = rxHeader.StdId - m3508[0]->protocolIDrx;
//        m3508[id -1]->updateInfo(rxData);
//        HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
    }

    if (hcan == &hcan2)
    {

    }
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t id;
    static uint8_t rxData[8];
    for (int l = 0; l < 2; ++l) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);
        if (hcan == &hcan1) {
//            uint8_t id = rxHeader.StdId;
//            memcpy(CAN_Data_FIFO.RX_FIFO[id], rxData, ID_DLC[id]);
//            CAN_Data_FIFO.RX_Readed[id] = 0;
        }
    }
    //HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        HAL_CAN_ResetError(hcan);
    }
}


#ifdef __cplusplus
}
#endif
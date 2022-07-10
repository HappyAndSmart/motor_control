//
// Created by szy on 2022/7/2.
//

#include "motor_task.h"
#include "cmsis_os.h"
#include "M2006.h"
#include "GM6020.h"
#include "M3508.h"
#include "usart.h"

#define m2006_watch
//#define gm6020_watch
//#define m3508_watch

int16_t fbCurrent;
int16_t fbEcd;
int16_t fbSpeed;
int16_t expSpeed;
int16_t tx_v;
float fbAngle;
float ecdSpeed;
uint8_t rx_uart[3] = {0xff,0xff,0xff};
uint16_t tt;

M2006 * m2006[3]; //initialize three m2006 motor
GM6020 * gm6020[2];
M3508 * m3508[4];


int16_t set_output = 120 ;
void motor_task(void const * argument)
{
    // initialize a m2006's ID and can
    m2006[0] = new M2006(1,&hcan1);
    gm6020[0] =new GM6020(1,&hcan1,7563);
    m3508[0] = new M3508(1,&hcan1,0);

    while (1)
    {
#ifdef m2006_watch
        fbCurrent = m2006[0]->fbCurrent;
        fbAngle = m2006[0]->fbAngle;
        fbEcd = m2006[0]->fbEcd;
        fbSpeed = m2006[0]->fbSpeed;
        expSpeed = m2006[0]->givenSpeed;
        //speed control

        m2006[0]->setExpSpeed(set_output);
        //position control !
        //m2006[0]->serExpAngle(120);
#endif
#ifdef gm6020_watch
        fbCurrent = gm6020[0]->fbCurrent;
        fbAngle = gm6020[0]->fbAngle;
        fbEcd = gm6020[0]->fbEcd;
        fbSpeed = gm6020[0]->fbSpeed;
        expSpeed = gm6020[0]->givenSpeed;
        tx_v = gm6020[0]->givenCurrent;
        gm6020[0]->setExpSpeed(set_output);
        //gm6020[0]->setExpAngle(set_output);
#endif
#ifdef m3508_watch
        fbCurrent = m3508[0]->fbCurrent;
        fbAngle = m3508[0]->totalAngle;
        fbEcd = m3508[0]->fbEcd;
        fbSpeed = m3508[0]->fbSpeed;
        expSpeed = m3508[0]->givenSpeed;
        tx_v = m3508[0]->givenCurrent;

        m3508[0]->setExpSpeed(set_output);
#endif
        /** 3s 's period: set_output change rule -120 -> 0 -> 120 -> 10 **/
        tt = tt + 1;
        if (tt == 3000)
        {
            tt = 0;
            switch (set_output) {
                case -120: set_output = 0;break;
                case 0: set_output = 120;break;
                case 120: set_output = 10;break;
                case 10: set_output = -120;break;
            }
        }
        vTaskDelay(1);
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   //FIXME:have question

//    if(huart == &huart1)
//    {
//        set_output = rx_uart[0];
//        if(rx_uart[0] = 0x00)
//        {
//            m2006[0]->m2006PIDSpeedRing.setPIDPara(rx_uart[1],0,rx_uart[2]*0.1);
//        }
//    }
//    HAL_UART_Receive_IT(&huart1,rx_uart,1);
//    HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
}
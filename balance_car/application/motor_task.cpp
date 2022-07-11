#include "motor_task.h"
#include "cmsis_os.h"
#include "M2006.h"
#include "can.h"


#define m2006_watch


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



int16_t set_output = 1400 ;
void motor_task(void const * argument)
{
    // initialize a m2006's ID and can
    m2006[0] = new M2006(1,&hcan1);
    //m2006[0]->m2006PIDSpeedRing.setPIDPara(7,0,0.9);

    while (1)
    {
        fbCurrent = m2006[0]->fbCurrent;
        fbAngle = m2006[0]->fbAngle;
        fbEcd = m2006[0]->fbEcd;
        fbSpeed = m2006[0]->fbSpeed;
        expSpeed = m2006[0]->givenSpeed;
        //speed control


        m2006[0]->setExpSpeed(set_output);
        //position control !
        //m2006[0]->serExpAngle(120);



        vTaskDelay(10);
    }
}
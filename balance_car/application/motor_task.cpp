#include "motor_task.h"
#include "cmsis_os.h"
#include "M2006.h"
#include "can.h"
#include "ins_task.h"


#define m2006_watch


int16_t fbCurrent[2];
int16_t fbEcd;
int16_t fbSpeed[2];
int16_t expSpeed;
int16_t tx_v;
float fbAngle;
float ecdSpeed;
uint8_t rx_uart[3] = {0xff,0xff,0xff};
uint16_t tt;

M2006 * m2006[3]; //initialize three m2006 motor



extern float ins_angle[3];


int16_t givenecd=m2006[0]->givenEcd;
int16_t givencurrent=m2006[0]->givenCurrent;
int16_t set_output = 500 ;
void motor_task(void const * argument)
{
    // initialize a m2006's ID and can
    m2006[0] = new M2006(1,&hcan1);
    m2006[1] = new M2006(2,&hcan1);
    m2006[0]->m2006PIDPosRing.id=1;
    m2006[1]->m2006PIDPosRing.id=1;
    //m2006[0]->m2006PIDSpeedRing.setPIDPara(7,0,0.9);

    while (1)
    {
        fbCurrent[0] = m2006[0]->fbCurrent;
        fbAngle = m2006[0]->fbAngle;
        fbCurrent[1]=m2006[1]->fbCurrent;
        fbSpeed[1]=m2006[1]->fbSpeed;
        fbEcd = m2006[0]->fbEcd;
        fbSpeed[0] = m2006[0]->fbSpeed;
        expSpeed = m2006[0]->givenSpeed;
        givencurrent=m2006[0]->givenCurrent;
        //speed control



        //position control !
        m2006[0]->setExpAngle(ins_angle[2]);
        m2006[1]->setExpAngle(ins_angle[2]);
        m2006[0]->setExpSpeed(set_output);
        m2006[1]->setExpSpeed(set_output);


        vTaskDelay(1);
    }
}
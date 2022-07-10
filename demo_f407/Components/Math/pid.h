#include "main.h"
/*
 * @brief Integral separation PID algorithm
 */
#ifndef __PID_H
#define __PID_H

struct PIDParaStruct
{
    float kp, ki, kd;
    float errorMax;
    float outputMax;
    float integralLimit;
};

class PID
{
    float kp;
    float ki;
    float kd;
    float lastReal{};
    float lastError{};
    float errorMax;
    float outputMax;
    float outputMin;

    float integralSum = 0;
    float integralLimit;

public:

    PID(PIDParaStruct * para);
    virtual float PID_Output(float p_real, float p_exp);
    void setPIDPara(float kp_t,float ki_t,float kd_t);
};



#endif
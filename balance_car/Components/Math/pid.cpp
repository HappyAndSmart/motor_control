#include "pid.h"

#include <cmath>


float PID::PID_Output(float p_real, float p_exp) {
    float error = p_exp - p_real;
    float output,iOut;

//    //速度平滑处理  k*(new-old)+(1-k)*old;
//    //翻车保护
//    if(error>errorMax) return 0;
//    //微分也可以滤波
//    k*(error-lastError)+lastError;
//    //设置死区
//    //高深的pid,三参数可以随误差变化
//    if(id==ANGLE)
//    {
//        if(fabs(error)<0.4)error=0;
//    }


    // integral diverse
    integralSum += error;
    iOut = integralSum * ki;
    //Limit
    if(integralSum > integralLimit) integralSum = integralLimit;
    if(integralSum < -integralLimit) integralSum = -integralLimit;
    if(iOut > integralLimit) iOut = integralLimit;
    if(iOut < -integralLimit) iOut = -integralLimit;

    //if(abs(error)>30)kp_big=100;

    //PID
    if(id==ANGLE)
    {

//        error = 1 / (1 + std::exp(-error))-0.5;
//        error *= 100;
        //lastoutput=output;
        float tmp=error*240+(error-lastError)*0.1+iOut*0.8;
        kp=80+fabsf(tmp)*0.4;
        kd=fabsf(tmp)*0.85;
        output = error * kp + (error - lastError) * kd + iOut;
        if(fabsf(error)<1)output=0;
        // output=output*0.7+lastoutput*0.3;

    }
        output = error * kp + (error - lastError) * kd + iOut;




    //Limit
    if (output > outputMax) output = outputMax;
    if (output < outputMin) output = outputMin;
    //Flash error
    lastError = error;
    return output;
}

void PID::pid_change()
{

}

PID::PID(PIDParaStruct * para) {
    kp = para->kp;
    ki = para->ki;
    kd = para->kd;
    errorMax = para->errorMax;
    outputMax = para->outputMax;
    outputMin = -para->outputMax;
    integralLimit = para->integralLimit;
}
void PID::setPIDPara(float kp_t, float ki_t, float kd_t) {
    kp = kp_t;
    ki = ki_t;
    kd = kd_t;
}
void PID::setPID_limit(float errorMax,float outputMax,float integralLimit)
{
    this->errorMax=errorMax;
    this->outputMax=outputMax;
    this->integralLimit=integralLimit;
}




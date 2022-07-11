#include "pid.h"

float PID::PID_Output(float p_real, float p_exp) {
    float error = p_exp - p_real;
    float output,iOut;

    // integral diverse
    integralSum += error;
    iOut = integralSum * ki;
    //Limit
    if(integralSum > integralLimit) integralSum = integralLimit;
    if(integralSum < -integralLimit) integralSum = -integralLimit;
    if(iOut > integralLimit) iOut = integralLimit;
    if(iOut < -integralLimit) iOut = -integralLimit;

    //PID
    output = error * kp + (error - lastError) * kd + iOut;

    //Limit
    if (output > outputMax) output = outputMax;
    if (output < outputMin) output = outputMin;
    //Flash error
    lastError = error;
    return output;
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
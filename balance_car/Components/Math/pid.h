//
// Created by Happy and Smart on 2022/7/11.
//

#ifndef BALANCE_CAR_PID_H
#define BALANCE_CAR_PID_H


#define ANGLE 1

struct PIDParaStruct
{
    float kp, ki, kd;
    float errorMax;
    float outputMax;
    float integralLimit;
};

class PID
{
public:
    int id;
    float kp;
    float ki;
    float kd;
    float kp_delta;
    float ki_delta;
    float kd_delta;
    float rate;   //学习率
    float lastError{};
    float lastoutput;
    float errorMax;
    float outputMax;
    float outputMin;

    float integralSum = 0;
    float integralLimit;

public:

    PID(PIDParaStruct * para);
    void pid_change();
    virtual float PID_Output(float p_real, float p_exp);
    void setPIDPara(float kp_t,float ki_t,float kd_t);
    void setPID_limit(float errorMax,float outputMax,float integralLimit);
};

#endif //BALANCE_CAR_PID_H

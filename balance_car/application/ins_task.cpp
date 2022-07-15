//
// Created by Happy and Smart on 2022/7/12.
//

/**
 *  @brief    This task is used to gain the output values of accelerometer ,gyroscope and magnetometer.
 *            At the same time, the Euler angles of the three axes are calculated in the loop.
 *  @process  1. Initialize related registers in ins_init() which is called in init_task();
 *            2. Waiting for the ready interrupt of the tri-axis data in the main loop,
 *               then perform a fusion algorithm calculation.
 *  @class
 *            Related Structures :     bmi088_real_data  ist8310_real_data
 *            Main members and arrays:
 *            --------------------------------------------------
 *            bmi088_real_data.accel[3] ---  Gyroscope real data
 *            bmi088_real_data.gyro[3]  ---  Accelerometer real data
 *            ist8310_real_data.mag[3]  ---  Magnetometer real data
 *            ins_angle[3]              ---  Euler angles
 *            ins_quat[4]               ---  Quaternion
 *            --------------------------------------------------
 */
#include "ins_task.h"


#include "cmsis_os.h"

#include "bsp_imu_pwm.h"
#include "BMI088Driver.h"
#include "ist8310driver.h"
#include "semphr.h"
#include "FreeRTOS.h"
#include "task.h"

#include "MahonyAHRS.h"
#include "math.h"

/// TODO:Note the modification of the macro definition here
#define DEBUG_No_TemperatureWaiting

#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)
#include "pid.h"

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
static void imu_temp_control(float temp);
void AHRS_init(float quat[4], float accel[3], float mag[3]);
void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3]);
void get_angle(float quat[4], float *yaw, float *pitch, float *roll);
bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;
//extern UART_HandleTypeDef huart1;
PIDParaStruct imuTempPIDPara = {
        TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD,
        100,
        TEMPERATURE_PID_MAX_OUT,
        TEMPERATURE_PID_MAX_IOUT
};

static uint8_t first_temperate;
static const float imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
PID imu_temp_pid(&imuTempPIDPara);

const int bias_time_ms = 1000;
const float exp_temp = 34.0f;
const float noise_down = 0.2f;
const float noise_up = 0.5f;
float ins_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float ins_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad
float ins_anglebias[3];

uint8_t bmi088_set_offset(void)
{
    float accel[3],gyro[3],gyro_offset[3];
    float mag_null[3]= {0,0,0};
    float ins_quat_bias[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    //wait for the temperature is higher than room's temperature
#ifndef DEBUG_No_TemperatureWaiting
    while (fabs(bmi088_real_data.temp - exp_temp) >= 1)
    {
        BMI088_read(gyro,accel,&bmi088_real_data.temp,mag_null);
        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
        HAL_Delay(2);
    }
#endif
    // calculate the average offset
    for (int i = 0; i < 300; i++)
    {
        BMI088_read(gyro, accel, &bmi088_real_data.temp, mag_null);
        gyro_offset[0] += gyro[0];
        gyro_offset[1] += gyro[1];
        gyro_offset[2] += gyro[2];
        imu_temp_control(bmi088_real_data.temp);
        HAL_Delay(2);
    }
    for (int i = 0; i < 3; i++) {
        bmi088_real_data.gyro_offset[i] = gyro_offset[i] / 300;
    }
    //use a old ins_quat_bias's value when debuging
#ifndef DEBUG_No_TemperatureWaiting
    int now_tick = osKernelSysTick();float old;
    while (osKernelSysTick() - now_tick <= bias_time_ms ) {
        old = gyro[0];
        // Ensure that after the data has been updated in the calculation
        while (old == gyro[0]) {
            BMI088_read(gyro,accel,&bmi088_real_data.temp,bmi088_real_data.gyro_offset);
            osDelay(0.1);
        }
        AHRS_update(ins_quat_bias, 0.002f, gyro, accel, mag_null);
    }
    get_angle(ins_quat_bias, ins_anglebias + INS_YAW_ADDRESS_OFFSET, ins_anglebias + INS_PITCH_ADDRESS_OFFSET, ins_anglebias + INS_ROLL_ADDRESS_OFFSET);
#endif
    return 0;
}

void gyro_noise_verdict()
{
    //To determine whether the gyroscope may have a large error
    //another method to eliminate of gyroscope burr noise
    for (int j = 0; j < 3; ++j) {
        if ( (bmi088_real_data.gyro_last[j] < noise_down && bmi088_real_data.gyro[j] > noise_up)
             || (bmi088_real_data.gyro_last[j] > -1.0*noise_down && bmi088_real_data.gyro[j] < -1.0*noise_up))
        {
            float value = bmi088_real_data.gyro[j];
            bmi088_real_data.gyro[j] = bmi088_real_data.gyro_last[j];
            bmi088_real_data.gyro_last[j] = value;
        } else
        {
            bmi088_real_data.gyro_last[j] = bmi088_real_data.gyro[j];
        }
    }
}
/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void ins_task(void const *pvParameters)
{
    TickType_t PreviousWakeTime = xTaskGetTickCount();
    uint32_t now_tick = osKernelSysTick();
    ins_quat[0]=1.0f;
    while (1)
    {
        //don't use magnetometer due to instability
        float mag_null[3]= {0,0,0};
        //read the relevant data from the register
        BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp, bmi088_real_data.gyro_offset);
        //Preventing data from being used before filtering
        taskENTER_CRITICAL();
        gyro_noise_verdict();
        taskEXIT_CRITICAL();
        // Update the temperature control function after each update of the temperature value
        imu_temp_control(bmi088_real_data.temp);
        //update the quaternion and euler angle
        AHRS_update(ins_quat, 0.001f, bmi088_real_data.gyro, bmi088_real_data.accel, mag_null);
        get_angle(ins_quat, ins_angle + INS_YAW_ADDRESS_OFFSET, ins_angle + INS_PITCH_ADDRESS_OFFSET, ins_angle + INS_ROLL_ADDRESS_OFFSET);
//        MahonyAHRSupdateIMU(ins_quat,bmi088_real_data.gyro[0],bmi088_real_data.gyro[1],bmi088_real_data.gyro[2],bmi088_real_data.accel[0],bmi088_real_data.accel[1],
//                            bmi088_real_data.accel[2]);
        //Reduction of yaw angle zero drift
        if (now_tick - osKernelSysTick() > bias_time_ms){
#ifdef DEBUG_No_TemperatureWaiting
            ins_angle[INS_YAW_ADDRESS_OFFSET] -= 0.1164679f;
#else
            ins_angle[INS_YAW_ADDRESS_OFFSET] -= ins_anglebias[INS_YAW_ADDRESS_OFFSET];
#endif
            now_tick = osKernelSysTick();
        }
        osDelayUntil(&PreviousWakeTime,2);
    }
}
void AHRS_init(float quat[4], float accel[3], float mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}
void get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *yaw *= 57.29578;
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *pitch *= 57.29578;
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
    *roll *= 57.29578;
}


static void imu_temp_control(float temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    float output;
    if (first_temperate)
    {
        output = imu_temp_pid.PID_Output(temp,exp_temp);
        if (output < 0.0f)
        {
            output = 0.0f;
        }
        tempPWM = (uint16_t)output;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //in beginning, max power
        if (temp < exp_temp)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                first_temperate = 1;
                //output = MPU6500_TEMP_PWM_MAX / 2.0f;
                IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
            }
        }
        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

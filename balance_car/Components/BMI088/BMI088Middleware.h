//
// Created by szy on 2022/3/6.
//

#ifndef FIREFLYRTS_FIRMWARE_BMI088MIDDLEWARE_H
#define FIREFLYRTS_FIRMWARE_BMI088MIDDLEWARE_H


#include "main.h"
#define BMI088_USE_SPI
//#define BMI088_USE_IIC

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)

#endif



#endif //FIREFLYRTS_FIRMWARE_BMI088MIDDLEWARE_H

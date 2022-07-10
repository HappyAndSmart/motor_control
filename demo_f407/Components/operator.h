//
// Created by nian on 2022/3/8.
//

#ifndef FIREFLYRTS_FIRMWARE_OPERATOR_H
#define FIREFLYRTS_FIRMWARE_OPERATOR_H

#include "main.h"
#include "cmsis_os.h"

#ifdef __cplusplus
void * operator new (size_t size );

void * operator new[]( size_t size );

void operator delete( void * ptr );

void operator delete[]( void * ptr );
#endif

#endif //FIREFLYRTS_FIRMWARE_OPERATOR_H

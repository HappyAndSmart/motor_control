#include "init_task.h"
#include "motor_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "ins_task.h"

extern osThreadId initTaskHandle;

void init_task(void const * argument) {
    //motor test Task
    osThreadId motorTaskHandle;
    osThreadDef(motorTask, motor_task, osPriorityNormal, 0, 128);
    motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

    osThreadId insTaskHandle;
    osThreadDef(insTask, ins_task, osPriorityNormal, 0, 128);
    insTaskHandle = osThreadCreate(osThread(insTask), NULL);

    //delete task
    vTaskDelete(initTaskHandle);
}
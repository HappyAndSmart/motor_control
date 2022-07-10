//
// Created by nian on 2022/4/11.
//

#ifndef FIREFLYRTS_FIRMWARE_COMPENSATOR_H
#define FIREFLYRTS_FIRMWARE_COMPENSATOR_H

#include "sys.h"

class compensator {
private:
    float u[3]{};
    float y[3]{};
    uint8_t pU, pY;

    float a = 4.595;
    float b = -3.968;
    float c = -4.59;
    float d = 3.974;
    float e = -0.8561;
    float f = -0.4336;
    float g = 0.2897;

    static void pUpdate(uint8_t* p);
public:
    float getOutput(float error);
};


#endif //FIREFLYRTS_FIRMWARE_COMPENSATOR_H

//
// Created by nian on 2022/4/11.
//

#include "compensator.h"

float compensator::getOutput(float uk) {
    float yk = 0;

    yk += a * uk;
    yk += b * u[pU];
    pUpdate(&pU);
    yk += c * u[pU];
    pUpdate(&pU);
    yk += d * u[pU];

    u[pU] = uk;

    yk -= e * y[pY];
    pUpdate(&pY);
    yk -= f * y[pY];
    pUpdate(&pY);
    yk -= g * y[pY];

    u[pY] = yk;

    return yk;
}

void compensator::pUpdate(uint8_t *p) {
    (*p)++;
    if(*p >= 3) *p = 0;
}

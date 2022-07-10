//
// Created by nian on 2022/3/8.
//

#include "operator.h"

void *operator new(size_t size) {
    return pvPortMalloc(size);
}

void *operator new[](size_t size) {
    return pvPortMalloc( size );
}

void operator delete(void *ptr) {
    vPortFree( ptr );
}

void operator delete[](void *ptr) {
    vPortFree( ptr );
}

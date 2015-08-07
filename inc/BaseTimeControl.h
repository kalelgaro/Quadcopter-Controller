#ifndef BASETIMECONTROL_H
#define BASETIMECONTROL_H

#include "stm32f4xx.h"

extern "C" {
static volatile uint32_t delayUs = 0;

void systickInterrupt(void);
void delayMs(float ms);

}

class BaseTimeControl
{
public:
    BaseTimeControl();
    ~BaseTimeControl();
};

#endif // BASETIMECONTROL_H

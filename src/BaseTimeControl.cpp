#include "../inc/BaseTimeControl.h"

void systickInterrupt(void) {
    if(delayUs != 0) {
        --delayUs;
    }
}

void delayMs(float ms) {

    if(ms <= 0.1)
        ms = 0.1;

    delayUs = (ms*10);

    while(delayUs > 0);
}


BaseTimeControl::BaseTimeControl()
{

}

BaseTimeControl::~BaseTimeControl()
{

}


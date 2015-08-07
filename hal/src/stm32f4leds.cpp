#include "../../hal/include/stm32f4leds.h"


STM32F4Leds::STM32F4Leds(u16 toInitializeLeds) :
    STM32F4GPIOHal(GPIOD, toInitializeLeds, GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
{

}

STM32F4Leds::~STM32F4Leds()
{

}


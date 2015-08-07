#ifndef STM32F4LEDS_H
#define STM32F4LEDS_H

#include "../../hal/include/stm32f4gpiohal.h"

class STM32F4Leds : public STM32F4GPIOHal
{
public:

    enum LedsPins : u16 {
        GREEN = GPIO_Pin_12,
        ORANGE = GPIO_Pin_13,
        RED = GPIO_Pin_14,
        BLUE = GPIO_Pin_15
    };


    STM32F4Leds(u16 toInitializeLeds);
    ~STM32F4Leds();

    inline void setLed(u16 led) const;
    inline void clearLed(u16 led) const;

private:

};

void STM32F4Leds::setLed(u16 led) const
{
    STM32F4GPIOHal::setOutputBit(led);
}

void STM32F4Leds::clearLed(u16 led) const
{
    STM32F4GPIOHal::clearOutputBit(led);
}

#endif // STM32F4LEDS_H

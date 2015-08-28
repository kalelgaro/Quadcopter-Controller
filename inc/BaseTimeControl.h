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
    static BaseTimeControl * const getInstance( void ) {
        if(m_instance) {
            return m_instance;
        }
        m_instance = new BaseTimeControl();
        return m_instance;
    }

    ~BaseTimeControl();

    static void systickCallback() {
        m_time += PERIOD_MS;

       BaseTimeControl::getInstance()->decrementDelayControl();
    }

    void delayMs(float ms) {

        if(ms <= 0.1)
            ms = 0.1;

        m_delayUs = (ms*10);

        while(m_delayUs > 0);
    }

private:
    void decrementDelayControl() {
        if(m_delayUs)
            --m_delayUs;
    }

    BaseTimeControl();

    BaseTimeControl(BaseTimeControl & a) = delete;
    BaseTimeControl & operator=(BaseTimeControl & a) = delete;

    static BaseTimeControl *m_instance;

    static uint64_t m_time;

    uint32_t m_delayUs;

    static constexpr float PERIOD_MS = 0.1;
};

#endif // BASETIMECONTROL_H

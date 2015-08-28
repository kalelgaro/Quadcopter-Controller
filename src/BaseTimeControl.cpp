#include "BaseTimeControl.h"

BaseTimeControl * BaseTimeControl::m_instance = 0;

uint64_t BaseTimeControl::m_time = 0;

BaseTimeControl::BaseTimeControl() :
    m_delayUs(0)
{

}

BaseTimeControl::~BaseTimeControl()
{

}

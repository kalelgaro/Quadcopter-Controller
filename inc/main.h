#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "arm_math.h"

#include "stm32f4xx_rcc.h"

#include "BaseTimeControl.h"

#include "callbackstruct.h"

#include "stm32f4gpiohal.h"
#include "stm32f4leds.h"
#include "stm32f4spihal.h"
#include "stm32f4i2chal.h"
#include "STM32F4DMA.h"
#include "STM32F4AsyncSPIHal.h"
#include "NRF24L01P.h"

#include "MPU6050.h"
#include "HMC5883L.h"

#include "QuadcopterController.h"

#include "TelemetryController.h"
#include "AHRSSystem.h"

I2CConfig i2cConfiguration();

#endif // MAIN_H


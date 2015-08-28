#ifndef STM32F4GPIOHAL_H
#define STM32F4GPIOHAL_H

#include "../hal/include/gpiodevice.h"

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

class STM32F4GPIOHal : public GPIODevice<u16>
{
private:
   GPIO_TypeDef *m_currentGPIO;
   GPIO_InitTypeDef m_currentConfig;

   //armazena os pinos DESTE OBJETO que foram inicializados.
   //Garante que as funções de ligar/desligar um pino garantidamente
   //só afetem um dos pinos iniciados por este objeto.
   u16 m_ioPinsInitialized;
   u16 m_analogPinsInitialized;
   u16 m_afPinsInitialized;

   //Variável indica se a inicizalização foi correta
   bool m_isValid;

   //Membros que guardam quais pinos de um determinado periférico ja foram
   //inicializados.
   static u16 sm_GPIOAPinInitializationState;
   static u16 sm_GPIOBPinInitializationState;
   static u16 sm_GPIOCPinInitializationState;
   static u16 sm_GPIODPinInitializationState;
   static u16 sm_GPIOEPinInitializationState;
   static u16 sm_GPIOFPinInitializationState;
   static u16 sm_GPIOGPinInitializationState;
   static u16 sm_GPIOHPinInitializationState;
   static u16 sm_GPIOIPinInitializationState;

   void configureGPIO(GPIO_TypeDef *gpio, GPIO_InitTypeDef *gpioInitStruct);

public:
    //STM32F4GPIOHal() {}
    STM32F4GPIOHal(GPIO_TypeDef* gpio, const GPIO_InitTypeDef &gpioInitStruct);
    STM32F4GPIOHal(GPIO_TypeDef* gpio,  uint32_t pin,
                                        GPIOMode_TypeDef mode,
                                        GPIOSpeed_TypeDef speed,
                                        GPIOOType_TypeDef outputType,
                                        GPIOPuPd_TypeDef pUpDCfg);

    inline void setOutputBit(u16 gpioPin) const;
    inline void clearOutputBit(u16 gpioPin) const;
    inline void toggleOutputBit(u16 gpioPin) const;

    inline bool readInputBit(u16 gpioPin) const;

    /**
     * @brief setAll
     * Liga todos os pinos inicializados por este objeto.
     */
    inline void setAll( void ) const;

    /**
     * @brief clearAll
     * Desliga todos os pino inicializados por este objeto.
     */
    inline void clearAll( void ) const;

    /**
     * @brief toggleAll
     * Alterana o estado de todos o pinos inicializados por este objeto.
     */
    inline void toggleAll( void ) const;

    inline bool isValid() const { return m_isValid; }

    virtual ~STM32F4GPIOHal();
    void configureGPIO(GPIO_InitTypeDef *gpioInitStruct, GPIO_TypeDef *gpio);
};

bool STM32F4GPIOHal::readInputBit(u16 gpioPin) const
{
    if((m_ioPinsInitialized & gpioPin) != 0)
        return GPIO_ReadInputDataBit(m_currentGPIO, gpioPin);
    else
        return false;
}

void STM32F4GPIOHal::setAll() const
{
    m_currentGPIO->BSRRL = m_ioPinsInitialized;
}

void STM32F4GPIOHal::clearAll() const
{
    m_currentGPIO->BSRRH = m_ioPinsInitialized;
}

void STM32F4GPIOHal::toggleAll() const
{
    m_currentGPIO->ODR ^= m_ioPinsInitialized;
}

void STM32F4GPIOHal::toggleOutputBit(u16 gpioPin) const
{
    //FIXME: Problema, da toggle EM TODOS os bits... Realizar correçao para um unico bit.
    //if((m_ioPinsInitialized & gpioPin) != 0)
        m_currentGPIO->ODR ^= gpioPin;
}

void STM32F4GPIOHal::setOutputBit(u16 gpioPin) const
{
    //Checa se o pino foi inicializado por esta classe
    //if((m_ioPinsInitialized & gpioPin) != 0)
        m_currentGPIO->BSRRL = gpioPin;
}

void STM32F4GPIOHal::clearOutputBit(u16 gpioPin) const
{
    //Checa se o pino foi inicializado por esta classe
    //if((m_ioPinsInitialized & gpioPin) != 0)
        m_currentGPIO->BSRRH = gpioPin;
}

//GPIO_ReadInputDataBit
//GPIO_ReadOutputDataBit
#endif // STM32F4GPIOHAL_H

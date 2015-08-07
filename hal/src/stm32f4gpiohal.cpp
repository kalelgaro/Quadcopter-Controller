#include "../../hal/include/stm32f4gpiohal.h"

u16 STM32F4GPIOHal::sm_GPIOAPinInitializationState = 0x0000;
u16 STM32F4GPIOHal::sm_GPIOBPinInitializationState = 0x0000;
u16 STM32F4GPIOHal::sm_GPIOCPinInitializationState = 0x0000;
u16 STM32F4GPIOHal::sm_GPIODPinInitializationState = 0x0000;
u16 STM32F4GPIOHal::sm_GPIOEPinInitializationState = 0x0000;
u16 STM32F4GPIOHal::sm_GPIOFPinInitializationState = 0x0000;
u16 STM32F4GPIOHal::sm_GPIOGPinInitializationState = 0x0000;
u16 STM32F4GPIOHal::sm_GPIOHPinInitializationState = 0x0000;
u16 STM32F4GPIOHal::sm_GPIOIPinInitializationState = 0x0000;

void STM32F4GPIOHal::configureGPIO(GPIO_TypeDef *gpio, GPIO_InitTypeDef *gpioInitStruct)
{
    if(gpio == GPIOA) {
        //Checa se os pinos que se deseja inicialização ja não foram iniciados em outra classe
        if((gpioInitStruct->GPIO_Pin & sm_GPIOAPinInitializationState) == 0) {

            if(!sm_GPIOAPinInitializationState)
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

            GPIO_Init(gpio, gpioInitStruct);

            //Checa se os pinos serão configurados como entrada ou Saída.
            if((gpioInitStruct->GPIO_Mode == GPIO_Mode_IN) ||
                    (gpioInitStruct->GPIO_Mode == GPIO_Mode_OUT)) {
                //Armazena os pinos iniciados neste objeto.
                m_ioPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AF)) {
                m_afPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AN)) {
                m_analogPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }

            //Atualiza a variável compartilhada com os pinos iniciados.
            //Quando um pino ja foi iniciado, o primeiro IF não irá retornar zero
            //(Algum dos bits irá bater), e assim, não inicia o dispositivo.
            sm_GPIOAPinInitializationState |= m_ioPinsInitialized;
            sm_GPIOAPinInitializationState |= m_afPinsInitialized;
            sm_GPIOAPinInitializationState |= m_analogPinsInitialized;

            m_isValid = true;
        }
    }

    if(gpio == GPIOB) {
        //Checa se os pinos que se deseja inicialização ja não foram iniciados em outra classe
        if((gpioInitStruct->GPIO_Pin & sm_GPIOBPinInitializationState) == 0) {
            if(!sm_GPIOBPinInitializationState)
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

            GPIO_Init(gpio, gpioInitStruct);

            //Checa se os pinos serão configurados como entrada ou Saída.
            if( (gpioInitStruct->GPIO_Mode == GPIO_Mode_IN) || (gpioInitStruct->GPIO_Mode == GPIO_Mode_OUT) )
            {
                //Armazena os pinos iniciados neste objeto.
                m_ioPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if( (gpioInitStruct->GPIO_Mode == GPIO_Mode_AF) )
            {
                m_afPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if( (gpioInitStruct->GPIO_Mode == GPIO_Mode_AN) )
            {
                m_analogPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }

            //Atualiza a variável compartilhada com os pinos iniciados.
            //Quando um pino ja foi iniciado, o primeiro IF não irá retornar zero
            //(Algum dos bits irá bater), e assim, não inicia o dispositivo.
            sm_GPIOBPinInitializationState |= m_ioPinsInitialized;
            sm_GPIOBPinInitializationState |= m_afPinsInitialized;
            sm_GPIOBPinInitializationState |= m_analogPinsInitialized;

            m_isValid = true;
        }
    }

    if(gpio == GPIOC) {
        //Checa se os pinos que se deseja inicialização ja não foram iniciados em outra classe
        if((gpioInitStruct->GPIO_Pin & sm_GPIOCPinInitializationState) == 0) {

            if(!sm_GPIOCPinInitializationState)
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

            GPIO_Init(gpio, gpioInitStruct);

            //Checa se os pinos serão configurados como entrada ou Saída.
            if((gpioInitStruct->GPIO_Mode == GPIO_Mode_IN) ||
                    (gpioInitStruct->GPIO_Mode == GPIO_Mode_OUT)) {
                //Armazena os pinos iniciados neste objeto.
                m_ioPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AF)) {
                m_afPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AN)) {
                m_analogPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }

            //Atualiza a variável compartilhada com os pinos iniciados.
            //Quando um pino ja foi iniciado, o primeiro IF não irá retornar zero
            //(Algum dos bits irá bater), e assim, não inicia o dispositivo.
            sm_GPIOCPinInitializationState |= m_ioPinsInitialized;
            sm_GPIOCPinInitializationState |= m_afPinsInitialized;
            sm_GPIOCPinInitializationState |= m_analogPinsInitialized;

            m_isValid = true;
        }
    }

    if(gpio == GPIOD) {
        //Checa se os pinos que se deseja inicialização ja não foram iniciados em outra classe
        if((gpioInitStruct->GPIO_Pin & sm_GPIODPinInitializationState) == 0) {

            if(!sm_GPIODPinInitializationState)
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

            GPIO_Init(gpio, gpioInitStruct);

            //Checa se os pinos serão configurados como entrada ou Saída.
            if((gpioInitStruct->GPIO_Mode == GPIO_Mode_IN) ||
                    (gpioInitStruct->GPIO_Mode == GPIO_Mode_OUT)) {
                //Armazena os pinos iniciados neste objeto.
                m_ioPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AF)) {
                m_afPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AN)) {
                m_analogPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }

            //Atualiza a variável compartilhada com os pinos iniciados.
            //Quando um pino ja foi iniciado, o primeiro IF não irá retornar zero
            //(Algum dos bits irá bater), e assim, não inicia o dispositivo.
            sm_GPIODPinInitializationState |= m_ioPinsInitialized;
            sm_GPIODPinInitializationState |= m_afPinsInitialized;
            sm_GPIODPinInitializationState |= m_analogPinsInitialized;

            m_isValid = true;
        }
    }

    if(gpio == GPIOE) {
        //Checa se os pinos que se deseja inicialização ja não foram iniciados em outra classe
        if((gpioInitStruct->GPIO_Pin & sm_GPIOEPinInitializationState) == 0) {

            if(!sm_GPIOEPinInitializationState)
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

            GPIO_Init(gpio, gpioInitStruct);

            //Checa se os pinos serão configurados como entrada ou Saída.
            if((gpioInitStruct->GPIO_Mode == GPIO_Mode_IN) ||
                    (gpioInitStruct->GPIO_Mode == GPIO_Mode_OUT)) {
                //Armazena os pinos iniciados neste objeto.
                m_ioPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AF)) {
                m_afPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AN)) {
                m_analogPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }

            //Atualiza a variável compartilhada com os pinos iniciados.
            //Quando um pino ja foi iniciado, o primeiro IF não irá retornar zero
            //(Algum dos bits irá bater), e assim, não inicia o dispositivo.
            sm_GPIOEPinInitializationState |= m_ioPinsInitialized;
            sm_GPIOEPinInitializationState |= m_afPinsInitialized;
            sm_GPIOEPinInitializationState |= m_analogPinsInitialized;

            m_isValid = true;
        }
    }

    if(gpio == GPIOF) {
        //Checa se os pinos que se deseja inicialização ja não foram iniciados em outra classe
        if((gpioInitStruct->GPIO_Pin & sm_GPIOFPinInitializationState) == 0) {

            if(!sm_GPIOFPinInitializationState)
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

            GPIO_Init(gpio, gpioInitStruct);

            //Checa se os pinos serão configurados como entrada ou Saída.
            if((gpioInitStruct->GPIO_Mode == GPIO_Mode_IN) ||
                    (gpioInitStruct->GPIO_Mode == GPIO_Mode_OUT)) {
                //Armazena os pinos iniciados neste objeto.
                m_ioPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AF)) {
                m_afPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AN)) {
                m_analogPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }

            //Atualiza a variável compartilhada com os pinos iniciados.
            //Quando um pino ja foi iniciado, o primeiro IF não irá retornar zero
            //(Algum dos bits irá bater), e assim, não inicia o dispositivo.
            sm_GPIOFPinInitializationState |= m_ioPinsInitialized;
            sm_GPIOFPinInitializationState |= m_afPinsInitialized;
            sm_GPIOFPinInitializationState |= m_analogPinsInitialized;

            m_isValid = true;
        }
    }

    if(gpio == GPIOG) {
        //Checa se os pinos que se deseja inicialização ja não foram iniciados em outra classe
        if((gpioInitStruct->GPIO_Pin & sm_GPIOGPinInitializationState) == 0) {

            if(!sm_GPIOGPinInitializationState)
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

            GPIO_Init(gpio, gpioInitStruct);

            //Checa se os pinos serão configurados como entrada ou Saída.
            if((gpioInitStruct->GPIO_Mode == GPIO_Mode_IN) ||
                    (gpioInitStruct->GPIO_Mode == GPIO_Mode_OUT)) {
                //Armazena os pinos iniciados neste objeto.
                m_ioPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AF)) {
                m_afPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AN)) {
                m_analogPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }

            //Atualiza a variável compartilhada com os pinos iniciados.
            //Quando um pino ja foi iniciado, o primeiro IF não irá retornar zero
            //(Algum dos bits irá bater), e assim, não inicia o dispositivo.
            sm_GPIOGPinInitializationState |= m_ioPinsInitialized;
            sm_GPIOGPinInitializationState |= m_afPinsInitialized;
            sm_GPIOGPinInitializationState |= m_analogPinsInitialized;

            m_isValid = true;
        }
    }

    if(gpio == GPIOH) {
        //Checa se os pinos que se deseja inicialização ja não foram iniciados em outra classe
        if((gpioInitStruct->GPIO_Pin & sm_GPIOHPinInitializationState) == 0) {

            if(!sm_GPIOHPinInitializationState)
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

            GPIO_Init(gpio, gpioInitStruct);

            //Checa se os pinos serão configurados como entrada ou Saída.
            if((gpioInitStruct->GPIO_Mode == GPIO_Mode_IN) ||
                    (gpioInitStruct->GPIO_Mode == GPIO_Mode_OUT)) {
                //Armazena os pinos iniciados neste objeto.
                m_ioPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AF)) {
                m_afPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AN)) {
                m_analogPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }

            //Atualiza a variável compartilhada com os pinos iniciados.
            //Quando um pino ja foi iniciado, o primeiro IF não irá retornar zero
            //(Algum dos bits irá bater), e assim, não inicia o dispositivo.
            sm_GPIOHPinInitializationState |= m_ioPinsInitialized;
            sm_GPIOHPinInitializationState |= m_afPinsInitialized;
            sm_GPIOHPinInitializationState |= m_analogPinsInitialized;

            m_isValid = true;
        }
    }

    if(gpio == GPIOI) {
        //Checa se os pinos que se deseja inicialização ja não foram iniciados em outra classe
        if((gpioInitStruct->GPIO_Pin & sm_GPIOIPinInitializationState) == 0) {

            if(!sm_GPIOIPinInitializationState)
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);

            GPIO_Init(gpio, gpioInitStruct);

            //Checa se os pinos serão configurados como entrada ou Saída.
            if((gpioInitStruct->GPIO_Mode == GPIO_Mode_IN) ||
                    (gpioInitStruct->GPIO_Mode == GPIO_Mode_OUT)) {
                //Armazena os pinos iniciados neste objeto.
                m_ioPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AF)) {
                m_afPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }else if((gpioInitStruct->GPIO_Mode == GPIO_Mode_AN)) {
                m_analogPinsInitialized = u16(gpioInitStruct->GPIO_Pin);
            }

            //Atualiza a variável compartilhada com os pinos iniciados.
            //Quando um pino ja foi iniciado, o primeiro IF não irá retornar zero
            //(Algum dos bits irá bater), e assim, não inicia o dispositivo.
            sm_GPIOIPinInitializationState |= m_ioPinsInitialized;
            sm_GPIOIPinInitializationState |= m_afPinsInitialized;
            sm_GPIOIPinInitializationState |= m_analogPinsInitialized;

            m_isValid = true;
        }
    }
}

STM32F4GPIOHal::STM32F4GPIOHal(GPIO_TypeDef *gpio, const GPIO_InitTypeDef &gpioInitStruct) :
    m_ioPinsInitialized(0x0000),
    m_analogPinsInitialized(0x0000),
    m_afPinsInitialized(0x0000)
{
    m_currentGPIO = gpio;
    m_currentConfig = gpioInitStruct;

    configureGPIO(m_currentGPIO, &m_currentConfig);
}

STM32F4GPIOHal::STM32F4GPIOHal(GPIO_TypeDef *gpio, uint32_t pin, GPIOMode_TypeDef mode, GPIOSpeed_TypeDef speed, GPIOOType_TypeDef outputType, GPIOPuPd_TypeDef pUpDCfg) :
    m_ioPinsInitialized(0x0000),
    m_analogPinsInitialized(0x0000),
    m_afPinsInitialized(0x0000),
    m_isValid(false)
{
    m_currentGPIO = gpio;

    GPIO_InitTypeDef tempConfig;

    tempConfig.GPIO_Pin = pin;
    tempConfig.GPIO_Mode = mode;
    tempConfig.GPIO_Speed = speed;
    tempConfig.GPIO_OType = outputType;
    tempConfig.GPIO_PuPd = pUpDCfg;

    m_currentConfig = tempConfig;

    configureGPIO(m_currentGPIO, &m_currentConfig);
}

//TODO: Implementar destrutor para liberar o recurso utilizado
STM32F4GPIOHal::~STM32F4GPIOHal()
{

}


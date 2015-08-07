#include "../../hal/include/stm32f4spihal.h"

/*-########## Implementaçao da classe para transmissoes Sincronas ##########-*/

//Inicialização dos membros static const;
template <typename Targ, size_t bufferSize>
u16 STM32F4SPIHal<Targ, bufferSize>::sm_SPIInitializated = 0x0000;

template <typename Targ, size_t bufferSize>
STM32F4SPIHal<Targ, bufferSize>::STM32F4SPIHal(SPI_TypeDef *spi, const SPIConfig & config) :
    SPIDevice<Targ>(bufferSize),
    m_sck(),
    m_mosi(),
    m_miso(),
    m_spi(spi)
{
    GPIO_InitTypeDef initialGPIOConfig;

    initialGPIOConfig.GPIO_Mode = GPIO_Mode_AF;
    initialGPIOConfig.GPIO_OType  = GPIO_OType_PP;
    initialGPIOConfig.GPIO_Speed = GPIO_Speed_25MHz;
    initialGPIOConfig.GPIO_PuPd = GPIO_PuPd_NOPULL;

    initialGPIOConfig.GPIO_Pin = config.sckPin;
    m_sck = new STM32F4GPIOHal(config.sckGPIO, initialGPIOConfig);

    initialGPIOConfig.GPIO_Pin = config.misoPin;
    m_miso = new STM32F4GPIOHal(config.misoGPIO, initialGPIOConfig);

    initialGPIOConfig.GPIO_Pin = config.mosiPin;
    m_mosi = new STM32F4GPIOHal(config.mosiGPIO, initialGPIOConfig);

    //Se a iniciazaliçaõ de qualquer um dos pinos falhar
    if(!m_sck->isValid() || !m_miso->isValid() || !m_mosi->isValid())
        return;

    uint8_t gpioAf;
    uint32_t device;

    if(spi == SPI1) {
        gpioAf = GPIO_AF_SPI1;

        if((sm_SPIInitializated & SPI1InitCode) == 0) {
            sm_SPIInitializated |= SPI1InitCode;

            device = RCC_APB2Periph_SPI1;
            //RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
        }else {
            return;
        }


    }else if(spi == SPI2) {
        gpioAf = GPIO_AF_SPI2;

        if((sm_SPIInitializated & SPI2InitCode) == 0) {
            sm_SPIInitializated |= SPI2InitCode;

            device = RCC_APB1Periph_SPI2;
            //RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
        }else {
            return;
        }

    }else if(spi == SPI3) {
        gpioAf = GPIO_AF_SPI3;

        if((sm_SPIInitializated & SPI3InitCode) == 0) {
            sm_SPIInitializated |= SPI3InitCode;

            device = RCC_APB1Periph_SPI3;
            //RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
        }else {
            return;
        }

    }else {
        return;
    }

    GPIO_PinSource13;
    uint16_t pinSource = uint16_t(log2(config.sckPin));
    GPIO_PinAFConfig(config.sckGPIO,  pinSource, gpioAf);

    pinSource = uint16_t(log2(config.mosiPin));
    GPIO_PinAFConfig(config.mosiGPIO, pinSource, gpioAf);

    pinSource = uint16_t(log2(config.misoPin));
    GPIO_PinAFConfig(config.misoGPIO, pinSource, gpioAf);


    if( device == RCC_APB1Periph_SPI3 || device == RCC_APB1Periph_SPI2 ) {
        RCC_APB1PeriphClockCmd(device, ENABLE);
    }else if( device == RCC_APB2Periph_SPI1 ) {
        RCC_APB2PeriphClockCmd(device, ENABLE);
    }

    //------- Configuração do SPI

    SPI_InitTypeDef tempConfig = config.basicConfig;
    SPI_Init(spi, &(tempConfig));
    SPI_Cmd(spi, ENABLE);

    m_spi = spi;
}

template <typename Targ, size_t bufferSize>
STM32F4SPIHal<Targ, bufferSize>::~STM32F4SPIHal()
{
    delete m_sck;
    delete m_mosi;
    delete m_miso;

    //Desabilita o bit que indica quais dispositivos estão ativados.
    if(m_spi == SPI1)
        sm_SPIInitializated &= ~SPI1InitCode;

    if(m_spi == SPI2)
        sm_SPIInitializated &= ~SPI2InitCode;

    if(m_spi == SPI3)
        sm_SPIInitializated &= ~SPI3InitCode;

    //SPI_Cmd(m_spi, DISABLE);
}

//---Adiçao das implementações de template com previsão de uso. ---//
template class STM32F4SPIHal<uint8_t,  10>;
template class STM32F4SPIHal<uint16_t, 10>;
template class STM32F4SPIHal<char,     10>;

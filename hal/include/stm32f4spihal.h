#ifndef STM32F4SPIHAL_H
#define STM32F4SPIHAL_H

#include "stm32f4xx_spi.h"
#include "../../hal/include/stm32f4gpiohal.h"
#include "../../hal/include/spidevice.h"
#include "arm_math.h"

#include "../../hal/include/stm32f4leds.h"

//TODO: Implementar m�todos para utiliza��o das interrup��es.
//Criar "delegates" para serem chamados no final das transmiss�es quando forem
//utilizadas as interrup��es.

typedef struct {
    GPIO_TypeDef *mosiGPIO;
    GPIO_TypeDef *misoGPIO;
    GPIO_TypeDef *sckGPIO;

    u16 mosiPin;
    u16 misoPin;
    u16 sckPin;

    SPI_InitTypeDef basicConfig;
} SPIConfig;

enum TransmissionMode {
    Sync=0,
    Async
};

//----- Especializa��o da classe para transmiss�es Syncronas. -----//

template <typename Targ, size_t bufferSize>
class STM32F4SPIHal: public SPIDevice<Targ>
{
public:
    STM32F4SPIHal(SPI_TypeDef *spi, const SPIConfig & config);
    ~STM32F4SPIHal();

  //void enviar_dados_SPI(SPI_TypeDef* SPIx, uint8_t buffer_envio[], uint8_t buffer_recepcao[], uint8_t tamanho)
  inline void sendData(const Targ *toSendDataBuffer, Targ *receivedDataBuffer, size_t maxSize = DEFAULT_SIZE);
  inline void sendData(const Targ &toSendData, Targ &receiveData);
  inline void receiveData(Targ *receivedDataBuffer, size_t maxSize = DEFAULT_SIZE);

  //---- C�digos de inicializa��o do SP1 ----//
  static const u16 SPI1InitCode = 0x01;
  static const u16 SPI2InitCode = 0x02;
  static const u16 SPI3InitCode = 0x04;

private:
    //Vari�veis que sinalizam que o SPI ja foi inicialiado e previnem a cria��o
    //de duas classes para manipular um �nico elemento de hardware.

    static u16 sm_SPIInitializated;

    STM32F4GPIOHal *m_sck;
    STM32F4GPIOHal *m_mosi;
    STM32F4GPIOHal *m_miso;

    SPI_TypeDef *m_spi;

    static const size_t DEFAULT_SIZE;
};

/*---#### Implementa��es das fun��es INLINE. ---####*/

template <typename Targ, size_t bufferSize>
void STM32F4SPIHal<Targ, bufferSize>::sendData(const Targ* toSendDataBuffer, Targ* receivedDataBuffer, size_t maxSize)
{
    for(size_t index = 0; index < maxSize; index++) {
        m_spi->DR = toSendDataBuffer[index];
        while( !(m_spi->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
        while( !(m_spi->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
        while( m_spi->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
        receivedDataBuffer[index] = m_spi->DR;
    }
}

template <typename Targ, size_t bufferSize>
void STM32F4SPIHal<Targ, bufferSize>::sendData(const Targ &toSendData, Targ &receiveData)
{
    m_spi->DR = toSendData;
    while( !(m_spi->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
    while( !(m_spi->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
    while( m_spi->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
    receiveData = m_spi->DR;
}

template <typename Targ, size_t bufferSize>
void STM32F4SPIHal<Targ, bufferSize>::receiveData(Targ *receivedDataBuffer, size_t maxSize)
{
    for(size_t index = 0; index < maxSize; index ++) {
        m_spi->DR = 0;
        while( !(m_spi->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
        while( !(m_spi->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
        while( m_spi->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
        receivedDataBuffer[index] = m_spi->DR;
    }
}
#endif // STM32F4SPIHAL_H

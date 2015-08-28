#ifndef STM32F4ASYNCSPIHAL_H
#define STM32F4ASYNCSPIHAL_H

#include "callbackstruct.h"

#include "stm32f4xx.h"
#include "stm32f4spihal.h"

#include "STM32F4DMA.h"

template <typename Targ, size_t bufferSize>
class STM32F4AsyncSPIHal : public STM32F4SPIHal<Targ>
{
public:
    /**
     * @brief createInstance
     * Constroi as classes para abstra��o do Hardware SPI. Como cada SPI s� pode
     * ser instanciado uma vez, n�o h� sentido em mais um objeto para o mesmo SPI,
     * a implementa��o singletron se faz necess�ria neste contexto. Esta primeira
     * implementa��o de get instance deve ser chamada no in�cio e � respons�vel
     * por instanciar o objeto correto
     * @param spi
     * @param config
     * @return
     */
    static STM32F4AsyncSPIHal* createInstance(const SPI & spi, const SPIConfig & config, DMA::STM32F4DMA &dma) {

        switch(spi) {

        case SPI1Device:
            if( !sm_spiAsyncInstance[SPI1Device] ) {
                sm_spiAsyncInstance[SPI1Device] = new STM32F4AsyncSPIHal(SPI1, config, dma);
            }
            break;

        case SPI2Device:
            if( !sm_spiAsyncInstance[SPI2Device] ) {
                sm_spiAsyncInstance[SPI2Device] = new STM32F4AsyncSPIHal(SPI2, config, dma);
            }
            break;

        case SPI3Device:
            if( !sm_spiAsyncInstance[SPI3Device] ) {
                sm_spiAsyncInstance[SPI3Device] = new STM32F4AsyncSPIHal(SPI3, config, dma);
            }
            break;

        default:
            return NULL;
        }

        return sm_spiAsyncInstance[spi];
    }

    /**
     * @brief getInstance
     * Retorna uma refer�ncia para o ponteiro para o SPI desejado, suponto
     * que este ja esteja inst�nciado na mem�ria. Deve ser utilizado com cautela,
     * pois o objeto n�o tiver sido inst�nciado pode provocar erros imprevi?iveis.
     * FIXME: Como colocar ASSERT?!
     * @param spi
     * @return
     */
    static STM32F4AsyncSPIHal * const getInstance(const SPI & spi) { return sm_spiAsyncInstance[spi]; }

    /**
     * @brief sendAsyncData
     * This method must be called to init a DMA SPI data transfer. It will
     * copy the contants present in the toSendData Array and store it internally
     * inside the m_toSendbuffer. After the copy is complete the DMA will be
     * enabled and the method will return. Once the data is completly received,
     * and thus, like in any SPI transfer received, the call back ISR method will
     * be called.
     * @param toSendData
     * @param size
     */
    void sendAsyncData(const Targ * toSendData, size_t size);

    void DMACallback() {

    }

private:
    STM32F4AsyncSPIHal(SPI_TypeDef *spi, const SPIConfig & config, DMA::STM32F4DMA & dma);
    ~STM32F4AsyncSPIHal();

    /*--- Dele��es de oepradores e construtores n�o utilizados em singletrons. ---*/
    STM32F4AsyncSPIHal(STM32F4AsyncSPIHal &a) = delete;
    STM32F4AsyncSPIHal & operator=(STM32F4AsyncSPIHal &a) = delete;

    Targ *m_toSendbuffer;
    Targ *m_receingBuffer;

    static STM32F4AsyncSPIHal* sm_spiAsyncInstance[3];

    DMA::STM32F4DMA &m_dmaDevice;

    const DMA::StreamChannel RX_STREAM_CHANNEL = DMA::Stream3;
    const DMA::StreamChannel TX_STREAM_CHANNEL = DMA::Stream4;
};

#endif // STM32F4ASYNCSPIHAL_H

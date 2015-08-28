#ifndef STM32F4DMA_H
#define STM32F4DMA_H

#include "stddef.h"

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"

namespace DMA {

enum DMADevice {
    DMA1Device = 0,
    DMA2Device
};

enum StreamChannel {
    Stream0=0x00,
    Stream1,
    Stream2,
    Stream3,
    Stream4,
    Stream5,
    Stream6,
    Stream7
};

struct DMAConfig {

};

/**
 * @brief The STM32F4DMA class
 * This class represents the DMA Device present in the STM32F407.
 * The register location and methods are based in the ST provided Library.
 * All the DMA transactions and requests are focused on this class. The main
 * objective is to provide an easy API to use with the other HAL Objetcs.
 */
class STM32F4DMA
{
public:
    static STM32F4DMA * const createInstance(const DMADevice & dma) {
        switch(dma) {
        case DMA1Device:
            if( !sm_dmaInstance[DMA1Device]) {
                sm_dmaInstance[DMA1Device] = new STM32F4DMA(DMA1Device);
            }
            break;

        case DMA2Device:
            if( !sm_dmaInstance[DMA2Device]) {
                sm_dmaInstance[DMA2Device] = new STM32F4DMA(DMA1Device);
            }
            break;

        default:
            return NULL;
            break;
        }

        return sm_dmaInstance[dma];
    }
    static STM32F4DMA * const getInstance(const DMADevice & dma) {
        return sm_dmaInstance[dma];
    }

    ~STM32F4DMA();

    static void ISRCallback(const DMADevice & dma, const StreamChannel & streamChannelISR) {

    }

    /**
     * @brief configureDMAStreams
     * This method is called for each Async device, like i2c and SPI, and is responsable
     * for configuring the needed streams for each device.
     */
    void configureDMAStreams(const StreamChannel &streamChannel, const DMA_InitTypeDef &streamConfig);

    /**
     * @brief enableDMAStream
     * This method will enable the DMA Stream and start the data transfer.
     * It supose that the stream is correctly configured and thus, the
     * transfer will begin shortly.
     */
    inline void enableDMAStream(const StreamChannel & streamChannel);

    inline DMA_Stream_TypeDef * getStreamRegADddress(const StreamChannel &streamChannel) const;

private:
    STM32F4DMA(const DMADevice &device);

    STM32F4DMA() = delete;
    STM32F4DMA(STM32F4DMA &c) = delete;
    void operator=(STM32F4DMA &c) = delete;

    static STM32F4DMA* sm_dmaInstance[2];

    uint32_t m_dmaBase;

    static bool m_streamsInitialized[2][8];
};

void STM32F4DMA::enableDMAStream(const StreamChannel &streamChannel)
{
    DMA_Stream_TypeDef *stream = getStreamRegADddress(streamChannel);

    DMA_ClearFlag(stream, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4 | DMA_FLAG_TEIF4 | DMA_FLAG_DMEIF4 | DMA_FLAG_FEIF4);
    DMA_Cmd(stream, ENABLE);

    while(DMA_GetCmdStatus(stream) == DISABLE);
}

DMA_Stream_TypeDef *STM32F4DMA::getStreamRegADddress(const StreamChannel &streamChannel) const
{
    DMA_Stream_TypeDef *stream;
    stream = reinterpret_cast<DMA_Stream_TypeDef*>( streamChannel*0x18 + 0x10 + m_dmaBase );

    return stream;
}

}

#endif // STM32F4DMA_H

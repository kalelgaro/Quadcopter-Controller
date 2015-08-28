#include "../../hal/include/STM32F4AsyncSPIHal.h"

template<typename Targ, size_t bufferSize>
STM32F4AsyncSPIHal<Targ, bufferSize>* STM32F4AsyncSPIHal<Targ, bufferSize>::sm_spiAsyncInstance[3] = { NULL, NULL, NULL };

template <typename Targ, size_t bufferSize>
STM32F4AsyncSPIHal<Targ, bufferSize>::STM32F4AsyncSPIHal( SPI_TypeDef *spi, const SPIConfig &config, DMA::STM32F4DMA & dma) :
    STM32F4SPIHal<Targ>( spi, config )
  , m_toSendbuffer( new Targ[bufferSize] )
  , m_receingBuffer( new Targ[bufferSize] )
  , m_dmaDevice( dma )
{
    //Escreve na variável estática do "pai" associado a esta classe. (Singletrons).
    if( spi == SPI1) {
        STM32F4SPIHal<Targ>::sm_spiInstance[SPI1Device] = reinterpret_cast<STM32F4SPIHal<Targ> *>(this);
    }else if( spi == SPI2) {
        STM32F4SPIHal<Targ>::sm_spiInstance[SPI2Device] = reinterpret_cast<STM32F4SPIHal<Targ> *>(this);
    }else if( spi == SPI3) {
        STM32F4SPIHal<Targ>::sm_spiInstance[SPI3Device] = reinterpret_cast<STM32F4SPIHal<Targ> *>(this);
    }


    /*--- Configura o DMA. ---*/

    DMA_InitTypeDef dmaInitConfig;

    //TODO: Checar condições para incialização

    //SPI2 RX =  DMA1 Channel 0 Stream 3
    //SPI2 TX =  DMA1 Channel 0 Stream 4

    DMA_StructInit(&dmaInitConfig);

    /*Configuração do DMA para envio de dados. */
    dmaInitConfig.DMA_Channel = DMA_Channel_0;
    dmaInitConfig.DMA_PeripheralBaseAddr = reinterpret_cast<uint32_t>(&(spi->DR));
    dmaInitConfig.DMA_Memory0BaseAddr = uint32_t(m_toSendbuffer);
    dmaInitConfig.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dmaInitConfig.DMA_BufferSize = bufferSize;
    dmaInitConfig.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInitConfig.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInitConfig.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInitConfig.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dmaInitConfig.DMA_Mode = DMA_Mode_Normal;
    dmaInitConfig.DMA_Priority = DMA_Priority_Medium;
    dmaInitConfig.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dmaInitConfig.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dmaInitConfig.DMA_MemoryBurst = DMA_MemoryBurst_INC16;
    dmaInitConfig.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    m_dmaDevice.configureDMAStreams(TX_STREAM_CHANNEL, dmaInitConfig);

    /*Configurações para recepção de dados através do DMA. */
    dmaInitConfig.DMA_Memory0BaseAddr = uint32_t(m_receingBuffer);
    dmaInitConfig.DMA_DIR = DMA_DIR_PeripheralToMemory;

    m_dmaDevice.configureDMAStreams(RX_STREAM_CHANNEL, dmaInitConfig);

    SPI_I2S_DMACmd(spi, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
}

template <typename Targ, size_t bufferSize>
STM32F4AsyncSPIHal<Targ, bufferSize>::~STM32F4AsyncSPIHal()
{

}

template <typename Targ, size_t bufferSize>
void STM32F4AsyncSPIHal<Targ, bufferSize>::sendAsyncData(const Targ *toSendData, size_t size)
{
    for(int i = 0; i < size; ++i) {
        m_toSendbuffer[i] = toSendData[i];
    }

    if(DMA_GetCmdStatus(DMA1_Stream4)== ENABLE) {
        DMA_Cmd(DMA1_Stream4, DISABLE);
    }
    while(DMA_GetCmdStatus(DMA1_Stream4)== ENABLE);

    DMA_SetCurrDataCounter(DMA1_Stream4, size);

    m_dmaDevice.enableDMAStream(TX_STREAM_CHANNEL);
    m_dmaDevice.enableDMAStream(RX_STREAM_CHANNEL);
}

template class STM32F4AsyncSPIHal<uint8_t, 30>;

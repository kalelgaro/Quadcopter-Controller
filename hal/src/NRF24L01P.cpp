#include "../../hal/include/NRF24L01P.h"

template <typename SPIDataType, typename GPIOPintype>
Telemetry::NRF24L01p<SPIDataType, GPIOPintype>::NRF24L01p(SPIDevice<SPIDataType> &spi,
                                                            GPIODevice<GPIOPintype> &ceGPIO, GPIOPintype cePin,
                                                            GPIODevice<GPIOPintype> &csGPIO, GPIOPintype csPin,
                                                            const NRF24L01pConfig &config) :
        m_ceGPIO(ceGPIO)
      , m_csGPIO(csGPIO)
      , m_cePin(cePin)
      , m_csPin(csPin)
      , m_spi(spi)
      , m_config(config)
      , m_txBuffer()
      , m_rxBuffer()
      , m_lastTxAddress(0)
      , m_status()
      , delay(config.delay)
{
    uint8_t command = PWR_UP;
    clearCEPin();
    writeRegister(CONFIG, command);

    command = PWR_UP | m_config.crcEnabled | m_config.crcConfig | m_config.irqConfig;
    writeRegister(CONFIG, command);

    configureNRF24L01p(config);

    configureRXPipesAddress(config);
}

template <typename SPIDataType, typename GPIOPintype>
Telemetry::NRF24L01p<SPIDataType, GPIOPintype>::~NRF24L01p()
{

}

template <typename SPIDataType, typename GPIOPintype>
void Telemetry::NRF24L01p<SPIDataType, GPIOPintype>::configureNRF24L01p(const Telemetry::NRF24L01pConfig &newConfig)
{
    clearCEPin();

    m_txBuffer[0] = newConfig.enableRXPipes;
    writeRegister(EN_RXADDR, m_txBuffer[0]);

    m_txBuffer[0] = newConfig.addressWidth;
    writeRegister(SETUP_AW, m_txBuffer[0]);

    if(newConfig.enableEnhancedShockBurst) {
        m_txBuffer[0] = newConfig.enableAAPipes;
        writeRegister(EN_AA, m_txBuffer[0]);

        m_txBuffer[0] =  newConfig.resendAttempts;
        m_txBuffer[0] |= newConfig.resendDelay;
        writeRegister(SETUP_RETR, m_txBuffer[0]);
    }

    m_txBuffer[0] =  newConfig.bitrate;
    m_txBuffer[0] |= newConfig.txPower;
    writeRegister(RF_SETUP, m_txBuffer[0]);

    m_txBuffer[0] = 0x00;
    if(newConfig.dynPayloadConfig != NO_DPL)
        m_txBuffer[0] |= EN_DPL;
    if(newConfig.enableACKPayload)
        m_txBuffer[0] |= EN_ACK_PAY;
    if(newConfig.enableDynAck)
        m_txBuffer[0] |= EN_DYN_ACK;
    writeRegister(FEATURE, m_txBuffer[0]);

    if(newConfig.dynPayloadConfig != NO_DPL) {
        /* Tamanho automático, habilita os registradores necessários. */
        m_txBuffer[0] = 0x00;
        m_txBuffer[0] = newConfig.dynPayloadConfig;
        writeRegister(DYNPD, m_txBuffer[0]);

    }else {
        /* Tamanho manual, o tamanho nos dutos deve ser configurado. */
        if(newConfig.enableRXPipes & EN_RX_P0) {
            writeRegister(RX_PW_P0, newConfig.pipesPayloadLenght.P0_PLD_LENGHT);
        }
        if(newConfig.enableRXPipes & EN_RX_P1) {
            writeRegister(RX_PW_P1, newConfig.pipesPayloadLenght.P1_PLD_LENGHT);
        }
        if(newConfig.enableRXPipes & EN_RX_P2) {
            writeRegister(RX_PW_P2, newConfig.pipesPayloadLenght.P2_PLD_LENGHT);
        }
        if(newConfig.enableRXPipes & EN_RX_P3) {
            writeRegister(RX_PW_P3, newConfig.pipesPayloadLenght.P3_PLD_LENGHT);
        }
        if(newConfig.enableRXPipes & EN_RX_P4) {
            writeRegister(RX_PW_P4, newConfig.pipesPayloadLenght.P4_PLD_LENGHT);
        }
        if(newConfig.enableRXPipes & EN_RX_P5) {
            writeRegister(RX_PW_P5, newConfig.pipesPayloadLenght.P5_PLD_LENGHT);
        }
    }

    m_txBuffer[0] = newConfig.irqConfig | newConfig.crcConfig;

    /* ---- Atualiza a configuração antiga salva. ----*/
    m_config = newConfig;

    /* ---- Configura os endereços dos dutos de recepção. ----*/
    configureRXPipesAddress();
}

template <typename SPIDataType, typename GPIOPintype>
void Telemetry::NRF24L01p<SPIDataType, GPIOPintype>::configureTXPipesAddress(const uint64_t &txAddress, const uint8_t & txAddressWidth)
{
    if(m_lastTxAddress != txAddress) {
        m_lastTxAddress = txAddress;

        memcpy(m_txBuffer, &txAddress, txAddressWidth);

        //Se o Enhanced Shock Burst está ligado, o duto P0 deve conter o endereço de envio.
        if(m_config.enableEnhancedShockBurst) {
            writeRegister(RX_ADDR_P0, m_txBuffer, txAddressWidth);
        }
        writeRegister(TX_ADDR, m_txBuffer, txAddressWidth);
    }
}

template <typename SPIDataType, typename GPIOPintype>
void Telemetry::NRF24L01p<SPIDataType, GPIOPintype>::sendData(const uint64_t &address, uint8_t *payload, size_t payloadLenght, uint8_t  txAddressWidht, bool waitAck)
{
    //----- Reconfigura o endereço de envio, se necessário.
    if(address != m_lastTxAddress) {
        configureTXPipesAddress(address, txAddressWidht);
    }
    txMode();

    if(waitAck && m_config.enableEnhancedShockBurst)
        writeTxPayload(payload, payloadLenght);
    else
        writeTxPayloadNoACK(payload, payloadLenght);

    setCEPin();
}

template <typename SPIDataType, typename GPIOPintype>
uint8_t Telemetry::NRF24L01p<SPIDataType, GPIOPintype>::readData(uint8_t *payload, size_t payloadLenght)
{
    m_ceGPIO.clearOutputBit(m_cePin);

    m_txBuffer[0] = R_RX_PAYLOAD;

    m_csGPIO.clearOutputBit(m_csPin);
    m_spi.sendData(m_txBuffer[0], m_rxBuffer[0]);
    m_spi.receiveData(payload, payloadLenght);

    //Atualiza o status. Este sempre é retornado no primeiro byte escrito.
    m_status = m_rxBuffer[0];

    m_csGPIO.setOutputBit(m_csPin);

    return m_status.dataPipeAvailablePld();
}

template <typename SPIDataType, typename GPIOPintype>
uint8_t Telemetry::NRF24L01p<SPIDataType, GPIOPintype>::readData(uint8_t *payload, size_t *payloadLenght)
{
    m_ceGPIO.clearOutputBit(m_cePin);

    m_status = updateStatus();
    m_status = updateStatus();

    //Dado o duto que forneceu o dado que vai ser lido (0 até 5), contido no
    //registrador status, realiza uma leitura do registrador que contém o
    //tamanho do payload recebido.
    readRegister(RX_PW_P0 + m_status.dataPipeAvailablePld(), m_rxBuffer, 1);

    *payloadLenght = m_rxBuffer[0];        //Atualiza o tamanho recebido.

    return readData(payload, *payloadLenght);
}

template <typename SPIDataType, typename GPIOPintype>
void Telemetry::NRF24L01p<SPIDataType, GPIOPintype>::configureRXPipesAddress(const Telemetry::NRF24L01pConfig &addressConfig)
{
    /*Configuração de endereço dependendo dos dutos de recepção que estão ligados. */
    if(addressConfig.enableRXPipes & EN_RX_P0) {
        memcpy(m_txBuffer, &addressConfig.rxAddress.RX_ADDR_P0, addressConfig.addressWidth+2);
        writeRegister(RX_ADDR_P0, m_txBuffer, addressConfig.addressWidth+2);
    }
    if(addressConfig.enableRXPipes & EN_RX_P1) {
        memcpy(m_txBuffer, &addressConfig.rxAddress.RX_ADDR_P1, addressConfig.addressWidth+2);
        writeRegister(RX_ADDR_P1, m_txBuffer, addressConfig.addressWidth+2);
    }
    if(addressConfig.enableRXPipes & EN_RX_P2) {
        writeRegister(RX_ADDR_P2, addressConfig.rxAddress.RX_ADDR_P2);
    }
    if(addressConfig.enableRXPipes & EN_RX_P3) {
        writeRegister(RX_ADDR_P3, addressConfig.rxAddress.RX_ADDR_P3);
    }
    if(addressConfig.enableRXPipes & EN_RX_P4) {
        writeRegister(RX_ADDR_P4, addressConfig.rxAddress.RX_ADDR_P4);
    }
    if(addressConfig.enableRXPipes & EN_RX_P5) {
        writeRegister(RX_ADDR_P5, addressConfig.rxAddress.RX_ADDR_P5);
    }
}

template <typename SPIDataType, typename GPIOPintype>
void Telemetry::NRF24L01p<SPIDataType, GPIOPintype>::configureRXPipesAddress()
{
    /*Configuração de endereço dependendo dos dutos de recepção que estão ligados. */
    if(m_config.enableRXPipes & EN_RX_P0) {
        memcpy(m_txBuffer, &m_config.rxAddress.RX_ADDR_P0, m_config.addressWidth+2);
        writeRegister(RX_ADDR_P0, m_txBuffer, m_config.addressWidth+2);
    }
    if(m_config.enableRXPipes & EN_RX_P1) {
        memcpy(m_txBuffer, &m_config.rxAddress.RX_ADDR_P1, m_config.addressWidth+2);
        writeRegister(RX_ADDR_P1, m_txBuffer, m_config.addressWidth+2);
    }
    if(m_config.enableRXPipes & EN_RX_P2) {
        writeRegister(RX_ADDR_P2, m_config.rxAddress.RX_ADDR_P2);
    }
    if(m_config.enableRXPipes & EN_RX_P3) {
        writeRegister(RX_ADDR_P3, m_config.rxAddress.RX_ADDR_P3);
    }
    if(m_config.enableRXPipes & EN_RX_P4) {
        writeRegister(RX_ADDR_P4, m_config.rxAddress.RX_ADDR_P4);
    }
    if(m_config.enableRXPipes & EN_RX_P5) {
        writeRegister(RX_ADDR_P5, m_config.rxAddress.RX_ADDR_P5);
    }
}


/*--- Pequena "engenharia alternativa" para permitir que templates sejam declarados
 * fora do arquivo de cabeçalhos.                                       ---*/
template class Telemetry::NRF24L01p<uint8_t,        uint16_t>;
//template class Telemetry::NRF24L01p<uint16_t,           uint16_t>;
//template class Telemetry::NRF24L01p<unsigned char, uint32_t>;

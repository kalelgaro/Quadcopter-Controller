#ifndef NRF24L01P_H
#define NRF24L01P_H

//---- Includes com as classes de HAL utilizadas. ----//
#include "../../hal/include/stm32f4gpiohal.h"
#include "../../hal/include/spidevice.h"
#include "../../hal/include/stm32f4leds.h"
#include "BaseTimeControl.h"
#include "string.h"

namespace Telemetry{
//----- Estruturas e enums de configuração do módulo RF NRF -----//
enum EnableAutoAck {    //Liga a auto-confirmação nos dutos.
    EN_AA_P0 = 0x01,
    EN_AA_P1 = 0x02,
    EN_AA_P2 = 0x04,
    EN_AA_P3 = 0x08,
    EN_AA_P4 = 0x10,
    EN_AA_P5 = 0x20
};
inline EnableAutoAck operator|(const EnableAutoAck &a, const EnableAutoAck &b) {
    return static_cast<EnableAutoAck>(static_cast<int>(a) | static_cast<int>(b));
}
/*-----------------------------------------------------------------------*/

enum EnableReceive {    //Liga a recepção nos dutos.
    EN_RX_P0 = 0x01,
    EN_RX_P1 = 0x02,
    EN_RX_P2 = 0x04,
    EN_RX_P3 = 0x08,
    EN_RX_P4 = 0x10,
    EN_RX_P5 = 0x20
};
inline EnableReceive operator|(const EnableReceive &a, const EnableReceive &b) {
    return static_cast<EnableReceive>(static_cast<int>(a) | static_cast<int>(b));
}
/*-----------------------------------------------------------------------*/

enum AddressWidth {
    ADD_WDTH_3_BYTES = 0x01,    //Configura a largura dos endereços para todos os dutos.
    ADD_WDTH_4_BYTES = 0x02,
    ADD_WDTH_5_BYTES = 0x03
};

struct RXPipesAddress {
    RXPipesAddress() :
        RX_ADDR_P0(0)
      , RX_ADDR_P1(0)
      , RX_ADDR_P2(0)
      , RX_ADDR_P3(0)
      , RX_ADDR_P4(0)
      , RX_ADDR_P5(0)
    { }

    uint64_t RX_ADDR_P0;        //Estrutura para configuração dos endereços de recebimento dos dutos.
    uint64_t RX_ADDR_P1;
    uint8_t RX_ADDR_P2;
    uint8_t RX_ADDR_P3;
    uint8_t RX_ADDR_P4;
    uint8_t RX_ADDR_P5;
};

/*-----------------------------------------------------------------------*/

enum AutoACKRetransmitAttempts {
    NO_RETRANSMIT    = 0x00,    //Configura o número de retransmissões em caso de falha.
    DO_01_RETRANSMIT = 0x01,
    DO_02_RETRANSMIT = 0x02,
    DO_03_RETRANSMIT = 0x03,
    DO_04_RETRANSMIT = 0x04,
    DO_05_RETRANSMIT = 0x05,
    DO_06_RETRANSMIT = 0x06,
    DO_07_RETRANSMIT = 0x07,
    DO_08_RETRANSMIT = 0x08,
    DO_09_RETRANSMIT = 0x09,
    DO_10_RETRANSMIT = 0x0A,
    DO_11_RETRANSMIT = 0x0B,
    DO_12_RETRANSMIT = 0x0C,
    DO_13_RETRANSMIT = 0x0D,
    DO_14_RETRANSMIT = 0x0E,
    DO_15_RETRANSMIT = 0x0F
};

enum AutoACKRetransmitDelay {
    DELAY_0250_US    = 0x00,    //Configura o tempo entre as tentativas de retransmissao em caso de falha de envvio.
    DELAY_0500_US    = 0x10,
    DELAY_0750_US    = 0x20,
    DELAY_1000_US    = 0x30,
    DELAY_1250_US    = 0x40,
    DELAY_1500_US    = 0x50,
    DELAY_1750_US    = 0x60,
    DELAY_2000_US    = 0x70,
    DELAY_2250_US    = 0x80,
    DELAY_2500_US    = 0x90,
    DELAY_2750_US    = 0xA0,
    DELAY_3000_US    = 0xB0,
    DELAY_3250_US    = 0xC0,
    DELAY_3500_US    = 0xD0,
    DELAY_3750_US    = 0xE0,
    DELAY_4000_US    = 0xF0
};
/*-----------------------------------------------------------------------*/

enum BitRate{
    BR_250KBPS  = 0b100000,  //Configura as velocidades de transmissão de dados.
    BR_1MBPS    = 0b000000,
    BR_2MBPS    = 0b001000
};
/*-----------------------------------------------------------------------*/

enum RFTxPower {
    MINUS_18_dBm = 0b000,   //Configura a potência do transmissor.
    MINUS_12_dBm = 0b010,
    MINUS_06_dBm = 0b100,
    MINUS_00_dBm = 0b110
};
/*-----------------------------------------------------------------------*/

enum FeatureRegOpt {
    EN_DPL      = 0x04,          //Habilita o payload de tamanho dinâmico.
    EN_ACK_PAY  = 0x02,          //Habilita Payloads no pacote de ACK.
    EN_DYN_ACK  = 0x01           //Habilita a possibilitade de enviar dados sem necessidade de recepção de ACK.
};
inline FeatureRegOpt operator|(const FeatureRegOpt &a, const FeatureRegOpt &b) {
    return static_cast<FeatureRegOpt>(static_cast<int>(a) | static_cast<int>(b));
}
/*-----------------------------------------------------------------------*/

enum DynPayloadEnable {
    NO_DPL      = 0x00,         //Não habilita o payload dinamico
    DPL_P0      = 0x01,         //Habilita o payload dinamico no duto Pn;
    DPL_P1      = 0x02,
    DPL_P2      = 0x04,
    DPL_P3      = 0x08,
    DPL_P4      = 0x10,
    DPL_P5      = 0x20,
};
inline DynPayloadEnable operator|(const DynPayloadEnable &a, const DynPayloadEnable &b) {
    return static_cast<DynPayloadEnable>(static_cast<int>(a) | static_cast<int>(b));
}
/*-----------------------------------------------------------------------*/

enum IRQConfig {
    NO_IRQ      = 0x70,
    MASK_RX_DR  = 0x30,
    MASK_TX_DS  = 0x50,
    MASK_MAX_RT = 0x60
};

inline IRQConfig operator&(const IRQConfig &a, const IRQConfig &b) {
    return static_cast<IRQConfig>(static_cast<int>(a) & static_cast<int>(b));
} 
/*-----------------------------------------------------------------------*/

enum StatusFlags {
    RX_DR   = 0x40,
    TX_DS   = 0x20,
    MAX_RT  = 0x10
};
inline StatusFlags operator|(const StatusFlags &a, const StatusFlags &b) {
    return static_cast<StatusFlags>(static_cast<int>(a) | static_cast<int>(b));
}
/*-----------------------------------------------------------------------*/

struct PipesPayloadLenght {
    PipesPayloadLenght() :
        P0_PLD_LENGHT(0),
        P1_PLD_LENGHT(0),
        P2_PLD_LENGHT(0),
        P3_PLD_LENGHT(0),
        P4_PLD_LENGHT(0),
        P5_PLD_LENGHT(0) { }

    uint8_t P0_PLD_LENGHT;
    uint8_t P1_PLD_LENGHT;
    uint8_t P2_PLD_LENGHT;
    uint8_t P3_PLD_LENGHT;
    uint8_t P4_PLD_LENGHT;
    uint8_t P5_PLD_LENGHT;

};
/*-----------------------------------------------------------------------*/

enum CRCConfig {
    CRC_1_BYTE  = 0x00,         //O CRC é habilitado se o Auto-ACK for habilitado.
    CRC_2_BYTE  = 0x04          //Configura a largura do ACK.
};

enum CRCEnabled {
    CRC_DIS = 0x00,
    CRC_EN  = 0x08
};
/*-----------------------------------------------------------------------*/

/*--- Objeto que representa o registrador de status ---*/
class StatusReg {
public:
        StatusReg() :
          m_rxDataReady( false )
        , m_txDataSent( false )
        , m_txMaxRetries (false )
        , m_rxDataPipe( 7 )
        , m_txFifoFull( false )
        , rawRegister( 0x0E ) { }

    StatusReg(uint8_t initial) :
          m_rxDataReady( initial & RX_DR_POS )
        , m_txDataSent( initial & TX_DS_POS )
        , m_txMaxRetries (initial & MAX_RT_POS )
        , m_rxDataPipe( (initial & RX_DATA_PIPE_POS) >> 1 )
        , m_txFifoFull( initial & TX_FULL_POS )
        , rawRegister(initial) { }

    StatusReg & operator=(const StatusReg & b) {
        this->rawRegister = b.rawRegister;

        this->m_rxDataReady = b.m_rxDataReady;
        this->m_txDataSent = b.m_txDataSent;
        this->m_txMaxRetries = b.m_txMaxRetries;
        this->m_rxDataPipe = b.m_rxDataPipe;
        this->m_txFifoFull = b.m_txFifoFull;

        return *this;
    }

    StatusReg & operator=(const uint8_t initial) {
        this->rawRegister = initial;

        this->m_rxDataReady = initial & RX_DR_POS;
        this->m_txDataSent = initial & TX_DS_POS;
        this->m_txMaxRetries = initial & MAX_RT_POS;
        this->m_rxDataPipe = (initial & RX_DATA_PIPE_POS) >> 1;
        this->m_txFifoFull = initial & TX_FULL_POS;

        return *this;
    }

    /**
     * @brief toUint8
     * Padrão usual utilizado na placa.
     * @return
     */
    const uint8_t & getUint8() const { return rawRegister; }

    inline const bool & isDataReceived() const { return m_rxDataReady; }
    inline const bool & isDataSent() const { return m_txDataSent; }
    inline const bool & transmissionError() const { return m_txMaxRetries; }
    inline const uint8_t & dataPipeAvailablePld() const { return m_rxDataPipe; }
    inline const bool & isTXFifoFull() const { return m_txFifoFull; }

    //Defines dos bits do registrador CONFIG
    static const uint8_t RX_DR_POS           = 0x40;
    static const uint8_t TX_DS_POS           = 0x20;
    static const uint8_t MAX_RT_POS          = 0x10;
    static const uint8_t RX_DATA_PIPE_POS    = 0x0E;
    static const uint8_t TX_FULL_POS         = 0x01;

    static const uint8_t RX_FIFOS_EMPTY      = 0x07;

private:
    bool m_rxDataReady;                 /** Flag que indica a disponibilidade dos dados recebidos. */
    bool m_txDataSent;                 /** Flag Setado quando houver o envio de um dado. Se o ACK estiver habilitado, só ira ocorrer na recepçaõ do ACK. */
    bool m_txMaxRetries;                /** Flag que indica que houve número máximo de tentativas de retransmissão. */
    uint8_t m_rxDataPipe;       /** Número do duto que contém dados para serem lidos 0 - 5 (Número do duto), 7 RX FIFO Vazia. */
    bool m_txFifoFull;          /** Estado do fifo para transmissão dos estdos.*/

    uint8_t rawRegister;        /** Representação do registrador na memória da placa. */
};

struct NRF24L01pConfig{
    NRF24L01pConfig() :
        enableRXPipes(EN_RX_P0)
      , enableAAPipes(EN_AA_P0)
      , addressWidth(ADD_WDTH_3_BYTES)
      , resendAttempts(NO_RETRANSMIT)
      , resendDelay(DELAY_0250_US)
      , bitrate(BR_250KBPS)
      , txPower(MINUS_00_dBm)
      , rxAddress(RXPipesAddress())
      , enableEnhancedShockBurst(false)
      , enablePipes(1)
      , dynPayloadConfig(NO_DPL)
      , pipesPayloadLenght()
      , irqConfig(NO_IRQ)
      , crcConfig(CRC_1_BYTE)
      , crcEnabled(CRC_DIS)
      , enableACKPayload(false)
      , enableDynAck(false)
      , delay(NULL)
    {
    }

    NRF24L01pConfig(const NRF24L01pConfig &config) :
        enableRXPipes(config.enableRXPipes)
      , enableAAPipes(config.enableAAPipes)
      , addressWidth(config.addressWidth)
      , resendAttempts(config.resendAttempts)
      , resendDelay(config.resendDelay)
      , bitrate(config.bitrate)
      , txPower(config.txPower)
      , rxAddress(config.rxAddress)
      , enableEnhancedShockBurst(config.enableEnhancedShockBurst)
      , enablePipes(config.enablePipes)
      , dynPayloadConfig(NO_DPL)
      , pipesPayloadLenght()
      , irqConfig(config.irqConfig)
      , crcConfig(config.crcConfig)
      , crcEnabled(config.crcEnabled)
      , enableACKPayload(false)
      , enableDynAck(false)
      , delay(config.delay)
    {
    }

    EnableReceive enableRXPipes;
    EnableAutoAck enableAAPipes;
    AddressWidth addressWidth;

    AutoACKRetransmitAttempts resendAttempts;
    AutoACKRetransmitDelay  resendDelay;

    BitRate bitrate;
    RFTxPower txPower;

    RXPipesAddress rxAddress;

    bool enableEnhancedShockBurst;

    uint8_t enablePipes;

    DynPayloadEnable dynPayloadConfig;
    PipesPayloadLenght pipesPayloadLenght;

    IRQConfig irqConfig;
    CRCConfig crcConfig;
    CRCEnabled crcEnabled;

    bool enableACKPayload;      //Permite pacotes de dados no pacote de ACK.
    bool enableDynAck;          //Permite Habilitar/Desabilitar envio de pacotes que não necessitem de ACK.

    //Método de delay;

    void (*delay)(float delayMs);
};

class NRF24L01p
{
public:
    NRF24L01p(SPIDevice<uint8_t> &spi,
              STM32F4GPIOHal &ceGPIO, uint16_t cePin,
              STM32F4GPIOHal &csGPIO, uint16_t csPin,
              const NRF24L01pConfig &config);
    ~NRF24L01p();

    void configureNRF24L01p(const NRF24L01pConfig &newConfig);
    void configureRXPipesAddress(const NRF24L01pConfig &addressConfig);
    void configureRXPipesAddress();
    void configureTXPipesAddress(const uint64_t &txAddress, const uint8_t &txAddressWidth);
    inline void configureDelayFunc(void (*delayMethod)(float));
    inline void clearStatusFlags();
    inline void clearStatusFlags(const StatusFlags &toCleanFlags);
    inline void clearTXBuffers();
    inline void clearRXBuffers();

    inline const StatusReg & updateStatus();
    inline const StatusReg & getStatus() const;

    void sendData(const uint64_t &address, uint8_t *payload, size_t payloadLenght, uint8_t txAddressWidht, bool waitAck = false);

    /**
     * @brief readData
     * Implementação de leitura dos dados contidas na FIFO de entrada da
     * NRF24L01P. Esta primeira implementação pressupõe que o taamnho dos payloads
     * é conhecido (pré definido pelo usuário).
     * @param payload
     * Endereço para o vetor de dados no qual os bytes recebidos devem ser escritos.
     * @param payloadLenght
     * @return
     * De qual dos dutos (de 0 até 5) o dado lido foi proveniente.
     *
     */
    uint8_t readData(uint8_t *payload, size_t payloadLenght);

    /**
     * @brief readData
     * Implementação de leitura dos dados contidas na FIFO de entrada da
     * NRF24L01P. Esta implementação pressupõe que o usuário não conhece o tamanho
     * do payload, e desta forma, a primeira etapa da recepção envolve descobrir
     * o tamanho do payload contido no duto que será lido. A variáel payloadLenght
     * será escrita com o tamanho do payload lido.
     * @param payload
     * @param payloadLenght
     * @return
     * De qual dos dutos (de 0 até 5) o dado lido foi proveniente.
     *
     */
    uint8_t readData(uint8_t *payload, size_t *payloadLenght);

    inline void txMode();
    inline void rxMode();

private:
    //---- Estruturas de abstração de Hardware utilizadas ----/
    STM32F4GPIOHal &m_ceGPIO;
    STM32F4GPIOHal &m_csGPIO;

    u16 m_cePin;
    u16 m_csPin;

    SPIDevice<uint8_t> &m_spi;

    NRF24L01pConfig m_config;

    //---- Estruturas de dados (Armazenar o estado) ----/
    uint8_t m_txBuffer[33];
    uint8_t m_rxBuffer[33];

    uint64_t m_lastTxAddress;    /** Armazena o endereço da última transmissão. */

    StatusReg m_status;

    //---- Métodos de interface para utilização do módulo ----//
    inline void writeRegister(uint8_t initialAddress, const uint8_t *data, size_t size);
    inline void writeRegister(uint8_t address, const uint8_t &data);
    inline uint8_t readRegister(uint8_t initialAddress, uint8_t *data, size_t size);

    inline void setCEPin() const;
    inline void clearCEPin() const;

    inline void writeTxPayload(uint8_t *data, size_t payloadLenght);
    inline void writeTxPayloadNoACK(uint8_t *data, size_t payloadLenght);


    //---- Endereço de funções úteis utilizados (Configurados)
    void (*delay)(float);

    //---- Definições dos endereços dos registradores. ----//

    /*----- Comandos -----*/

    static const uint8_t NOP = 0xFF;

        /*--- Comando para escrita/leitura de registradores. ---*/
    static const uint8_t W_REG_MASK = 0b00100000;
    static const uint8_t R_REG_MASK = 0b00000000;

        /*--- Comando para escrita/leitura de Payloads. ---*/
    static const uint8_t R_RX_PAYLOAD = 0b01100001;
    static const uint8_t W_TX_PAYLOAD = 0b10100000;

        /*--- Comando para limpar os payloads (TX/RX). ---*/
    static const uint8_t FLUSH_TX = 0b11100001;
    static const uint8_t FLUSH_RX = 0b11100010;

        /*--- Comando para reutilizar o último payload enviado. ---*/
    static const uint8_t REUSE_TX_PL = 0b11100011;

        /*--- Comando para pegar o tamanho do payload recebido (topo da fifo). ---*/
    static const uint8_t R_RX_PL_WID = 0b01100000;

        /*--- Comando para escrever um paylaod para ser enviado com a msg de ACK
         * Os ultimos três bits deste comando devem conter para qual duto este
         * pacote de ack será enviado (0 até 5). Até três valores podem estar
         * pendentes. ---*/
    static const uint8_t W_ACK_PAYLOAD = 0b10101000;

        /* --- Comando para que o próximo pacote TX não precise de recepção de
         * payload. ---*/
    static const uint8_t W_TX_PAYLOAD_NO_ACK = 0b10110000;

    /*----- Registradores. -----*/
    static const uint8_t CONFIG          = 0x00;
    static const uint8_t EN_AA           = 0x01;
    static const uint8_t EN_RXADDR       = 0x02;
    static const uint8_t SETUP_AW        = 0x03;
    static const uint8_t SETUP_RETR      = 0x04;
    static const uint8_t RF_SETUP        = 0x06;
    static const uint8_t STATUS          = 0x07;

    static const uint8_t RX_ADDR_P0      = 0x0A;
    static const uint8_t RX_ADDR_P1      = 0x0B;
    static const uint8_t RX_ADDR_P2      = 0x0C;
    static const uint8_t RX_ADDR_P3      = 0x0D;
    static const uint8_t RX_ADDR_P4      = 0x0E;
    static const uint8_t RX_ADDR_P5      = 0x0F;
    static const uint8_t TX_ADDR         = 0x10;

    static const uint8_t RX_PW_P0        = 0x11;
    static const uint8_t RX_PW_P1        = 0x12;
    static const uint8_t RX_PW_P2        = 0x13;
    static const uint8_t RX_PW_P3        = 0x14;
    static const uint8_t RX_PW_P4        = 0x15;
    static const uint8_t RX_PW_P5        = 0x16;

    static const uint8_t FIFO_STATUS     = 0x17;

    static const uint8_t DYNPD           = 0x1C;
    static const uint8_t FEATURE         = 0x1D;

    //Defines do registrador de CONFIG
    static const uint8_t PRIM_RX         = 0x01;
    static const uint8_t PWR_UP          = 0x02;
    static const uint8_t CRCO            = 0x04;
    static const uint8_t EN_CRC          = 0x08;
    static const uint8_t MASK_MAX_RT     = 0x10;
    static const uint8_t MASK_TX_DS      = 0x20;
    static const uint8_t MASK_RX_DR      = 0x40;
};


/*---### Funções Inline implenentadas no *.h ---###*/
void NRF24L01p::setCEPin() const
{
    m_ceGPIO.setOutputBit(m_cePin);
}

void NRF24L01p::clearCEPin() const
{
    m_ceGPIO.clearOutputBit(m_cePin);
}

void NRF24L01p::rxMode()
{
    uint8_t cfg = (PWR_UP | PRIM_RX | m_config.crcConfig | m_config.crcEnabled | m_config.irqConfig);
    writeRegister(CONFIG, cfg);
    delayMs(0.2);    //Delay de 0.02mS = 20uS
    m_ceGPIO.setOutputBit(m_cePin);
    delayMs(2);
}

void NRF24L01p::writeTxPayload(uint8_t *data, size_t payloadLenght)
{
    uint8_t command = W_TX_PAYLOAD;
    m_csGPIO.clearOutputBit(m_csPin);
    m_spi.sendData(command, m_rxBuffer[0]);

    //Atualiza o status. Este sempre é retornado
    m_status = m_rxBuffer[0];

    m_spi.sendData(data, m_rxBuffer, payloadLenght);
    m_csGPIO.setOutputBit(m_csPin);
}

void NRF24L01p::writeTxPayloadNoACK(uint8_t *data, size_t payloadLenght)
{
    uint8_t command = W_TX_PAYLOAD_NO_ACK;
    m_csGPIO.clearOutputBit(m_csPin);
    m_spi.sendData(command, m_rxBuffer[0]);

    //Atualiza o status. Este sempre é retornado
    m_status = m_rxBuffer[0];

    m_spi.sendData(data, m_rxBuffer, payloadLenght);
    m_csGPIO.setOutputBit(m_csPin);
}

void NRF24L01p::txMode()
{
    clearCEPin();
    uint8_t cfg = (PWR_UP | m_config.crcConfig | m_config.crcEnabled | m_config.irqConfig);
    writeRegister(CONFIG, cfg);
    //delayMs(0.2);    //Delay de 0.02mS = 20uS
}

void NRF24L01p::configureDelayFunc(void (*delayMethod)(float))
{
    delay = delayMethod;
}

void NRF24L01p::clearStatusFlags()
{
    m_ceGPIO.clearOutputBit(m_cePin);

    uint8_t status = updateStatus().getUint8();
    status = (RX_DR | TX_DS | MAX_RT) & status;

    m_csGPIO.clearOutputBit(m_csPin);
    writeRegister(STATUS, status);
    m_csGPIO.setOutputBit(m_csPin);
}

void NRF24L01p::clearStatusFlags(const StatusFlags &toCleanFlags)
{
    uint8_t clean = toCleanFlags;

    m_csGPIO.clearOutputBit(m_csPin);
    writeRegister(STATUS, clean);
    m_csGPIO.setOutputBit(m_csPin);
}

void NRF24L01p::clearTXBuffers()
{
    clearCEPin();

    m_txBuffer[0] =  FLUSH_TX;

    m_csGPIO.clearOutputBit(m_csPin);
    m_spi.sendData(m_txBuffer[0], m_rxBuffer[0]);

    //Atualiza o status. Este sempre é retornado
    m_status = m_rxBuffer[0];

    m_csGPIO.setOutputBit(m_csPin);
}

void NRF24L01p::clearRXBuffers()
{
    clearCEPin();

    m_txBuffer[0] =  FLUSH_RX;

    m_csGPIO.clearOutputBit(m_csPin);
    m_spi.sendData(m_txBuffer[0], m_rxBuffer[0]);

    //Atualiza o status. Este sempre é retornado
    m_status = m_rxBuffer[0];


    m_csGPIO.setOutputBit(m_csPin);
}

const StatusReg &NRF24L01p::updateStatus()
{
    m_txBuffer[0] = NOP;

    m_csGPIO.clearOutputBit(m_csPin);
    m_spi.sendData(m_txBuffer[0], m_rxBuffer[0]);
    m_csGPIO.setOutputBit(m_csPin);

    m_status = m_rxBuffer[0];
    return m_status;
}

const StatusReg &NRF24L01p::getStatus() const
{
    return m_status;
}

void NRF24L01p::writeRegister(uint8_t initialAddress, const uint8_t *data, size_t size)
{
    clearCEPin();

    uint8_t writeAddress = initialAddress+W_REG_MASK;

    m_csGPIO.clearOutputBit(m_csPin);
    m_spi.sendData(writeAddress, m_rxBuffer[0]);

    //Atualiza o status. Este sempre é retornado
    m_status = m_rxBuffer[0];

    m_spi.sendData(data, m_rxBuffer, size);
    m_csGPIO.setOutputBit(m_csPin);

}

void NRF24L01p::writeRegister(uint8_t address, const uint8_t &data) {
    clearCEPin();
    uint8_t writeAddress = address+W_REG_MASK;

    m_csGPIO.clearOutputBit(m_csPin);
    m_spi.sendData(writeAddress, m_rxBuffer[0]);

    //Atualiza o status. Este sempre é retornado
    m_status = m_rxBuffer[0];

    m_spi.sendData(data, m_rxBuffer[0]);
    m_csGPIO.setOutputBit(m_csPin);
}

uint8_t NRF24L01p::readRegister(uint8_t initialAddress, uint8_t *data, size_t size)
{
    uint8_t readAddress = initialAddress+R_REG_MASK;

    m_csGPIO.clearOutputBit(m_csPin);
    m_spi.sendData(readAddress, m_rxBuffer[0]);

    //Atualiza o status. Este sempre é retornado
    m_status = m_rxBuffer[0];

    m_spi.receiveData(data, size);
    m_csGPIO.setOutputBit(m_csPin);

    return  m_rxBuffer[0];
}

}

#endif // NRF24L01P_H


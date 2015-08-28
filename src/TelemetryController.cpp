#include "TelemetryController.h"

TelemetryController::TelemetryController(GPIODevice<u16> &cs, GPIODevice<u16> &ce, GPIODevice<u16> &irq,  SPIDevice<uint8_t> &spi) :
    m_cs(cs)
  , m_ce(ce)
  , m_nrfIRQ(irq)
  , m_nrfAsyncSPIDriver(spi)
  , m_nrf24l01(NULL)
{
    m_cs.setOutputBit(GPIO_Pin_10);
    m_ce.setOutputBit(GPIO_Pin_12);

    using namespace Telemetry;
    NRF24L01pConfig nrfConfig;

    RXPipesAddress rxAddress;
    rxAddress.RX_ADDR_P0 = 0x0E0D0C0B0A;
    rxAddress.RX_ADDR_P1 = 0x131211100F;

    nrfConfig.enableRXPipes = EN_RX_P0 | EN_RX_P1;
    nrfConfig.enableAAPipes = EN_AA_P0 | EN_AA_P1;
    nrfConfig.enablePipes = 2;
    nrfConfig.addressWidth = ADD_WDTH_5_BYTES;
    nrfConfig.resendAttempts = DO_15_RETRANSMIT;
    nrfConfig.resendDelay = DELAY_1000_US;
    nrfConfig.bitrate = BR_250KBPS;
    nrfConfig.txPower = MINUS_00_dBm;
    nrfConfig.rxAddress = rxAddress;
    nrfConfig.enableEnhancedShockBurst = true;
    nrfConfig.dynPayloadConfig = NO_DPL;
    nrfConfig.pipesPayloadLenght.P0_PLD_LENGHT = 32;
    nrfConfig.pipesPayloadLenght.P1_PLD_LENGHT = 32;
    nrfConfig.irqConfig = MASK_RX_DR & MASK_TX_DS & MASK_MAX_RT;
    nrfConfig.crcEnabled = CRC_EN;
    nrfConfig.crcConfig = CRC_2_BYTE;
    nrfConfig.enableACKPayload = false;
    nrfConfig.enableDynAck = false;

    //nrfConfig.delay = delayMs;
    m_nrf24l01 = new Telemetry::NRF24L01p<uint8_t, uint16_t> (m_nrfAsyncSPIDriver, m_ce, GPIO_Pin_12, m_cs, GPIO_Pin_10, nrfConfig);

    m_nrf24l01->clearStatusFlags();

    m_nrf24l01->sendData(0x0E0D0C0B0A, m_payload, 32, 5, true);
    while(m_nrfIRQ.readInputBit(GPIO_Pin_11));

    m_status = m_nrf24l01->updateStatus();
    m_nrf24l01->clearStatusFlags();

    m_nrf24l01->rxMode();
}

TelemetryController::~TelemetryController()
{

}

void TelemetryController::update()
{
    if(!m_nrfIRQ.readInputBit(GPIO_Pin_11)) {
        m_status = m_nrf24l01->updateStatus();

        if(m_status.isDataReceived()) {
            uint8_t pipe = m_nrf24l01->readData(m_receivedPayload, &m_receivedPayloadSize);

            if(m_receivedPayload[0] == 'T' && m_receivedPayloadSize == 32) {
                 //led.setLed(STM32F4Leds::BLUE);
            }

            m_nrf24l01->clearStatusFlags(Telemetry::RX_DR);

            m_nrf24l01->rxMode();
        }
    }
}


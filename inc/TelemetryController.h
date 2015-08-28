#ifndef TELEMETRYCONTROLLER_H
#define TELEMETRYCONTROLLER_H

#include "gpiodevice.h"
#include "spidevice.h"

#include "STM32F4AsyncSPIHal.h"
#include "stm32f4gpiohal.h"

#include "NRF24L01P.h"


class TelemetryController
{
public:
    TelemetryController(GPIODevice<u16> &cs, GPIODevice<u16> &ce, GPIODevice<u16> &irq,  SPIDevice<uint8_t> &spi);
    ~TelemetryController();

    void update ( void );
private:
    GPIODevice<u16> &m_cs;
    GPIODevice<u16> &m_ce;
    GPIODevice<u16> &m_nrfIRQ;

    SPIDevice<uint8_t> &m_nrfAsyncSPIDriver;

    Telemetry::NRF24L01p<uint8_t, uint16_t> *m_nrf24l01;

    uint8_t m_receivedPayload[32];
    uint8_t m_payload[32];
    size_t m_receivedPayloadSize;

    Telemetry::StatusReg m_status;

};

#endif // TELEMETRYCONTROLLER_H

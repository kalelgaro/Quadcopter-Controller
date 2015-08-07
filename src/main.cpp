/**
  ******************************************************************************
  * @file    IO_Toggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "arm_math.h"

#include "stm32f4xx_rcc.h"

#include "../inc/BaseTimeControl.h"

#include "../../hal/include/stm32f4gpiohal.h"
#include "../../hal/include/stm32f4leds.h"
#include "../../hal/include/stm32f4spihal.h"
#include "../../hal/include/NRF24L01P.h"

STM32F4Leds led(STM32F4Leds::GREEN | STM32F4Leds::BLUE | STM32F4Leds::RED);

int main(void)
{
    /*---- Configuração do SysTick. ----*/
    NVIC_SetPriorityGrouping(5);
    SysTick_Config(168e6/10000);		 										//Frequência 100uS

    led.setLed(STM32F4Leds::GREEN);
    delayMs(1000);
    led.clearLed(STM32F4Leds::GREEN);

    STM32F4GPIOHal *m_cs =     new STM32F4GPIOHal(GPIOB, GPIO_Pin_10, GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_UP);
    STM32F4GPIOHal *m_ce =     new STM32F4GPIOHal(GPIOB, GPIO_Pin_12, GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_UP);
    STM32F4GPIOHal *m_nrfIRQ = new STM32F4GPIOHal(GPIOB, GPIO_Pin_11, GPIO_Mode_IN,  GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);


    SPIConfig nrfSPIConfig;
    nrfSPIConfig.sckGPIO = GPIOB;
    nrfSPIConfig.misoGPIO = GPIOB;
    nrfSPIConfig.mosiGPIO = GPIOB;

    nrfSPIConfig.sckPin = GPIO_Pin_13;
    nrfSPIConfig.misoPin = GPIO_Pin_14;
    nrfSPIConfig.mosiPin = GPIO_Pin_15;

    nrfSPIConfig.basicConfig.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    nrfSPIConfig.basicConfig.SPI_Mode = SPI_Mode_Master;
    nrfSPIConfig.basicConfig.SPI_DataSize = SPI_DataSize_8b;
    nrfSPIConfig.basicConfig.SPI_CPOL =  SPI_CPOL_Low;
    nrfSPIConfig.basicConfig.SPI_CPHA = SPI_CPHA_1Edge;
    nrfSPIConfig.basicConfig.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
    nrfSPIConfig.basicConfig.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    nrfSPIConfig.basicConfig.SPI_FirstBit = SPI_FirstBit_MSB;

    STM32F4SPIHal<uint8_t, 10> m_nrfSPIDriver(SPI2, nrfSPIConfig);

    m_cs->setOutputBit(GPIO_Pin_10);
    m_ce->setOutputBit(GPIO_Pin_12);

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
    Telemetry::NRF24L01p teste(m_nrfSPIDriver, *m_ce, GPIO_Pin_12, *m_cs, GPIO_Pin_10, nrfConfig);

    teste.clearStatusFlags();

    uint8_t receivedPayload[32];
    size_t receivedPayloadSize;
    uint8_t payload[32] = {'B', 'r', 'u', 'n', 'o', ' ', 'F', 'e', 'r', 'n', 'a', 'n', 'd', 'o' };

    teste.sendData(0x0E0D0C0B0A, payload, 32, 5, true);
    while(m_nrfIRQ->readInputBit(GPIO_Pin_11));

    led.setLed(STM32F4Leds::BLUE);

    Telemetry::StatusReg status = teste.updateStatus();
    teste.clearStatusFlags();

    status = status;

    teste.rxMode();

    led.clearLed(STM32F4Leds::BLUE);
    while(true) {
        if(!m_nrfIRQ->readInputBit(GPIO_Pin_11)) {
            status = teste.updateStatus();

            if(status.isDataReceived()) {
                led.setLed(STM32F4Leds::GREEN);

                uint8_t pipe = teste.readData(receivedPayload, &receivedPayloadSize);

                if(receivedPayload[0] == 'T' && receivedPayloadSize == 32) {
                     led.setLed(STM32F4Leds::BLUE);
                }

                teste.clearStatusFlags(RX_DR);

                teste.rxMode();
            }
        }
//        delayMs(10);
//        teste.sendData(0x0E0D0C0B0A, payload, 32, 5, true);     //Envia dados.
//        while(m_nrfIRQ->readInputBit(GPIO_Pin_11));             //Aguarda o envio.
//        status = teste.updateStatus();                          //Pèga o status.
//        teste.clearStatusFlags();                               //Limpa os flags.
//        teste.clearTXBuffers();                                 //Limpa os buffers.
//        status = teste.updateStatus();                          //Pega o status (DEBUG).
//        led.toggleOutputBit(STM32F4Leds::BLUE);                 //Toogle o led.
    }
}

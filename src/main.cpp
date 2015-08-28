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
#include "main.h"

STM32F4Leds led(STM32F4Leds::GREEN | STM32F4Leds::BLUE | STM32F4Leds::RED);

int main(void)
{
    /*---- Configuração do SysTick. ----*/
    NVIC_SetPriorityGrouping(5);
    SysTick_Config(168e6/10000);		 										//Frequência 100uS

    /*--- Inicialização dos dispositivos de HARDWARE utilizados pela telemetria. ---*/
    STM32F4GPIOHal *m_cs = new STM32F4GPIOHal(GPIOB, GPIO_Pin_10, GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_UP);
    STM32F4GPIOHal *m_ce = new STM32F4GPIOHal(GPIOB, GPIO_Pin_12, GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_UP);
    STM32F4GPIOHal *m_irq = new STM32F4GPIOHal(GPIOB, GPIO_Pin_11, GPIO_Mode_IN,  GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);

    SPIConfig nrfSPIConfig;
    nrfSPIConfig.sckGPIO = GPIOB;
    nrfSPIConfig.misoGPIO = GPIOB;
    nrfSPIConfig.mosiGPIO = GPIOB;

    nrfSPIConfig.sckPin = GPIO_Pin_13;
    nrfSPIConfig.misoPin = GPIO_Pin_14;
    nrfSPIConfig.mosiPin = GPIO_Pin_15;

    /*--- Configurações básicas iguais ao do driver da ST. ---*/
    nrfSPIConfig.basicConfig.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    nrfSPIConfig.basicConfig.SPI_Mode = SPI_Mode_Master;
    nrfSPIConfig.basicConfig.SPI_DataSize = SPI_DataSize_8b;
    nrfSPIConfig.basicConfig.SPI_CPOL =  SPI_CPOL_Low;
    nrfSPIConfig.basicConfig.SPI_CPHA = SPI_CPHA_1Edge;
    nrfSPIConfig.basicConfig.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
    nrfSPIConfig.basicConfig.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    nrfSPIConfig.basicConfig.SPI_FirstBit = SPI_FirstBit_MSB;

    /*--- Recupera a instancia do DMA que sera utilizado para os dispositivos Assincronos. ---*/


    DMA::STM32F4DMA *m_dma0 = DMA::STM32F4DMA::createInstance(DMA::DMA1Device);
            /* - O dispositivo DMA sera configurado pelo controlador de DMA. -*/

    STM32F4AsyncSPIHal<uint8_t, 30> *m_nrfAsyncSPIDriver = STM32F4AsyncSPIHal<uint8_t, 30>::createInstance(SPI2Device, nrfSPIConfig, *m_dma0);

    BaseTimeControl::getInstance()->delayMs(10);
    //TelemetryController m_controller(*m_cs, *m_ce, *m_irq, *m_nrfAsyncSPIDriver);
    /********************************************************************************/

    /*--- Inicialização dos dispositivos de HARDWARE utilizados pelo sistema AHRS. -*/
    I2CConfig i2cConfig = i2cConfiguration();
    STM32F4I2CHal m_i2c(I2C3, i2cConfig);

    /* Pinos dos sensores "inerciais". IRQS - Inicialização "temporária" para usar a placa do drone.*/
    STM32F4GPIOHal mpu6050IRQ(GPIOD, GPIO_Pin_0, GPIO_Mode_IN,  GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    STM32F4GPIOHal hmc5883lIRQ(GPIOC, GPIO_Pin_7, GPIO_Mode_IN,  GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);

    Sensors::MPU6050 mpu6050(m_i2c);
    Sensors::HMC5883L hmc5883l;

    //FIXME: A ultima é um magnetometro
    //AHRS::AHRSSystem m_AHRSSystem(mpu6050, mpu6050, hmc5883l);
    /********************************************************************************/

    /*--- Inicialização da classe principal do Quadcoptero - Recebe referência dos sistemas. */
    //QuadcopterController mainSystem(m_AHRSSystem, m_controller);
    /********************************************************************************/

    uint8_t toSendTestBuffer[] = {'B', 'r', 'u', 'n', 'o', ' ', 'F', 'e', 'r', 'n', 'a', 'n', 'd', 'o'};
    uint8_t teste[13];

    led.clearOutputBit(STM32F4Leds::BLUE);
    m_nrfAsyncSPIDriver->sendAsyncData(toSendTestBuffer, 13);
    led.setOutputBit(STM32F4Leds::BLUE);

    //while(DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4)== RESET);

    while(true) {
        //mainSystem.telemetryUpdate();
        BaseTimeControl::getInstance()->delayMs(1);
        led.toggleOutputBit(STM32F4Leds::GREEN);
//        teste.sendData(0x0E0D0C0B0A, payload, 32, 5, true);     //Envia dados.
//        while(m_nrfIRQ->readInputBit(GPIO_Pin_11));             //Aguarda o envio.
//        status = teste.updateStatus();                          //Pèga o status.
//        teste.clearStatusFlags();                               //Limpa os flags.
//        teste.clearTXBuffers();                                 //Limpa os buffers.
//        status = teste.updateStatus();                          //Pega o status (DEBUG).
//        led.toggleOutputBit(STM32F4Leds::BLUE);                 //Toogle o led.
    }
}

I2CConfig i2cConfiguration()
{
    //I2C3: SCL: PA8, SDA: PC9

    I2CConfig i2cConfig;

    i2cConfig.sclGPIO = GPIOA;
    i2cConfig.sclPin = GPIO_Pin_8;

    i2cConfig.sdaGPIO = GPIOC;
    i2cConfig.sdaPin = GPIO_Pin_9;

    i2cConfig.basicConfig.I2C_Ack = I2C_Ack_Disable;
    i2cConfig.basicConfig.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2cConfig.basicConfig.I2C_ClockSpeed = 400000;
    i2cConfig.basicConfig.I2C_DutyCycle = I2C_DutyCycle_2;
    i2cConfig.basicConfig.I2C_Mode = I2C_Mode_I2C;
    i2cConfig.basicConfig.I2C_OwnAddress1 = 0x00;
    i2cConfig.clockSpeed = 400000;

    return i2cConfig;
}

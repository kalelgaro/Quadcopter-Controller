#ifndef STM32F4I2CHAL_H
#define STM32F4I2CHAL_H

#include "i2cdevice.h"

#include "stm32f4gpiohal.h"
#include "stm32f4xx_i2c.h"
#include "arm_math.h"

typedef struct {
    GPIO_TypeDef *sdaGPIO;
    GPIO_TypeDef *sclGPIO;

    u16 sdaPin;
    u16 sclPin;

    uint32_t clockSpeed;

    I2C_InitTypeDef basicConfig;
} I2CConfig;

class STM32F4I2CHal : public I2CDevice<uint8_t, uint8_t, uint8_t>
{
private:
    static u16 sm_I2CInitializated;

    STM32F4GPIOHal *m_sda;
    STM32F4GPIOHal *m_scl;

    uint8_t *m_buffer;

    I2C_TypeDef *m_i2c;

    static const size_t DEFAULT_SIZE;

    inline bool generateStart(uint8_t address, uint8_t direction, bool ack) const;
    inline void generateStop();

    /**
     * @brief sendByte
     * Envia um byte através da interface I2C.
     * @param toSendByte
     */
    inline void sendByte(uint8_t toSendByte) const;
    /**
     * @brief readAndAck
     * Lê um byte e envia um sinal de ACK ao emissor;
     */
    inline uint8_t readAndAck();
    /**
     * @brief readAndNAck
     * Lê um byte e não envia um sinal de ACK ao emissor.
     */
    inline uint8_t readAndNAck();

    void initPins(const I2CConfig &initialConfig, uint8_t gpioAf);

public:
    STM32F4I2CHal(I2C_TypeDef *i2c, const I2CConfig & initialConfig, size_t bufferSize = DEFAULT_SIZE);
    virtual ~STM32F4I2CHal();

    //----- Métodos para transferência de dados -----//
    void sendData(uint8_t slaveAddress, uint8_t regAddress,  uint8_t const * data, size_t size);
    void writeRegister(uint8_t slaveAddress, uint8_t regAddress, uint8_t data);

    uint8_t const * readData(uint8_t deviceAddress, uint8_t registerAddress, size_t size);
    uint8_t readData(uint8_t deviceAddress, uint8_t registerAddress);

    uint8_t readRegister(uint8_t deviceAddress, uint8_t registerAddress);

    static const u16 I2C1InitCode = 0x01;
    static const u16 I2C2InitCode = 0x02;
    static const u16 I2C3InitCode = 0x04;

    static const u16 START_DELAY = 100;

};

bool STM32F4I2CHal::generateStart(uint8_t address, uint8_t direction, bool ack) const
{
    uint16_t i;
    i = START_DELAY;
    while(i--);

    //Gera o start.
    I2C_GenerateSTART(m_i2c, ENABLE);

    while(I2C_GetFlagStatus(m_i2c, I2C_FLAG_SB) == RESET);

    if(ack){
        I2C_AcknowledgeConfig(m_i2c, ENABLE);
    }

    I2C_Send7bitAddress(m_i2c, address, direction);

    if(direction == I2C_Direction_Transmitter) {
        while(I2C_GetFlagStatus(m_i2c, I2C_FLAG_ADDR) == RESET);
    }else if(direction == I2C_Direction_Receiver) {
        while(I2C_GetFlagStatus(m_i2c, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == RESET);
    }
    m_i2c->SR2;

    return true;
}

void STM32F4I2CHal::generateStop()
{
    //FIXME::ERA BOOL
    while (((!I2C_GetFlagStatus(m_i2c, I2C_FLAG_TXE)) || (!I2C_GetFlagStatus(m_i2c, I2C_FLAG_BTF))));

    I2C_GenerateSTOP(m_i2c, ENABLE);
}

void STM32F4I2CHal::sendByte(uint8_t toSendByte) const
{
    while(I2C_GetFlagStatus(m_i2c, I2C_FLAG_TXE) == RESET);
    m_i2c->DR = toSendByte;
}

uint8_t STM32F4I2CHal::readAndAck()
{
    I2C_AcknowledgeConfig(m_i2c, ENABLE);

    while (I2C_CheckEvent(m_i2c, I2C_EVENT_MASTER_BYTE_RECEIVED) == ERROR);

    uint8_t data = (uint8_t)m_i2c->DR;

    return data;
}

uint8_t STM32F4I2CHal::readAndNAck()
{
    I2C_AcknowledgeConfig(m_i2c, DISABLE);

    I2C_GenerateSTOP(m_i2c, ENABLE);

    int timeout = 100000;
    while (!I2C_CheckEvent(m_i2c, I2C_EVENT_MASTER_BYTE_RECEIVED) && (--timeout));

    uint8_t data = I2C_ReceiveData(m_i2c);

    return data;
}

#endif // STM32F4I2CHAL_H

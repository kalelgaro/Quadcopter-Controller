#include "../../hal/include/stm32f4i2chal.h"

const size_t STM32F4I2CHal::DEFAULT_SIZE = 30;

u16 STM32F4I2CHal::sm_I2CInitializated = 0x0000;

STM32F4I2CHal::STM32F4I2CHal(I2C_TypeDef *i2c, const I2CConfig & initialConfig, size_t bufferSize) :
    m_sda(),
    m_scl(),
    m_buffer(new uint8_t(bufferSize)),
    m_i2c()
{
    GPIO_InitTypeDef initialGPIOConfig;

    initialGPIOConfig.GPIO_Mode = GPIO_Mode_AF;
    initialGPIOConfig.GPIO_OType = GPIO_OType_OD;
    initialGPIOConfig.GPIO_PuPd = GPIO_PuPd_UP;
    initialGPIOConfig.GPIO_Speed = GPIO_Speed_100MHz;

    initialGPIOConfig.GPIO_Pin = initialConfig.sclPin;
    m_scl = new STM32F4GPIOHal(initialConfig.sclGPIO, initialGPIOConfig);

    initialGPIOConfig.GPIO_Pin = initialConfig.sdaPin;
    m_sda = new STM32F4GPIOHal(initialConfig.sdaGPIO, initialGPIOConfig);

    if(!m_sda->isValid() || !m_scl->isValid())
        return;

    uint8_t gpioAf;
    if(i2c == I2C1 ) {
        gpioAf = GPIO_AF_I2C1;

        if((sm_I2CInitializated & I2C1InitCode) == 0) {
            sm_I2CInitializated |= I2C1InitCode;

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        }else {
            return;
        }
    }else if(i2c == I2C2) {
        gpioAf = GPIO_AF_I2C2;

        if((sm_I2CInitializated & I2C2InitCode) == 0) {
            sm_I2CInitializated |= I2C2InitCode;

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
        }else {
            return;
        }
    }else if(i2c == I2C3) {
        gpioAf = GPIO_AF_I2C3;

        if((sm_I2CInitializated & I2C3InitCode) == 0) {
            sm_I2CInitializated |= I2C3InitCode;

            RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
        }else {
            return;
        }
    }else {
        return;
    }

    uint16_t pinSource = uint16_t(log2(initialConfig.sclPin));
    GPIO_PinAFConfig(initialConfig.sclGPIO,  pinSource, gpioAf);

    pinSource = uint16_t(log2(initialConfig.sdaPin));
    GPIO_PinAFConfig(initialConfig.sdaGPIO, pinSource, gpioAf);

    //-------- Configuração do I2C

    I2C_InitTypeDef config;
    config = initialConfig.basicConfig;
    config.I2C_ClockSpeed = initialConfig.clockSpeed;
    I2C_Init(i2c, &config);
    I2C_Cmd(i2c, ENABLE);

    m_i2c = i2c;
}

STM32F4I2CHal::~STM32F4I2CHal()
{   
    if(m_i2c == I2C1)
        sm_I2CInitializated &= ~I2C1InitCode;

    if(m_i2c == I2C2)
        sm_I2CInitializated &= ~I2C2InitCode;

    if(m_i2c == I2C3)
        sm_I2CInitializated &= ~I2C3InitCode;

    I2C_Cmd(m_i2c, DISABLE);
}

void STM32F4I2CHal::writeRegister(uint8_t slaveAddress, uint8_t regAddress, uint8_t data)
{
    generateStart(slaveAddress, I2C_Direction_Transmitter, false);
    sendByte(regAddress);
    sendByte(data);
    generateStop();
}

uint8_t const * STM32F4I2CHal::readData(uint8_t deviceAddress, uint8_t registerAddress, size_t size)
{
    generateStart(deviceAddress, I2C_Direction_Transmitter, true);
    sendByte(registerAddress);
    generateStop();
    generateStart(deviceAddress, I2C_Direction_Receiver, true);

    //Decrementa - Leitura até o penúltimo byte.
    --size;
    for(size_t i = 0; i < size; ++i) {
        m_buffer[i] = readAndAck();
    }
    m_buffer[size] = readAndNAck();

    return m_buffer;
}

void STM32F4I2CHal::sendData(uint8_t slaveAddress, uint8_t regAddress, uint8_t const * data, size_t size)
{
    generateStart(slaveAddress, I2C_Direction_Transmitter, 0);
    sendByte(regAddress);
    for(size_t i = 0; i < size; ++i) {
        sendByte(data[i]);
    }
    generateStop();
}

uint8_t STM32F4I2CHal::readRegister(uint8_t deviceAddress, uint8_t registerAddress)
{
    generateStart(deviceAddress, I2C_Direction_Transmitter, true);
    sendByte(registerAddress);
    generateStop();
    generateStart(deviceAddress, I2C_Direction_Receiver, false);
    uint8_t receivedData = readAndNAck();

    return receivedData;
}

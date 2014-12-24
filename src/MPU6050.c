#include "MPU6050.h"

uint16_t accelFullScale;
uint16_t gyroFullScale;

GPIO_TypeDef* dataRdyIntPort;
uint16_t dataRdyIntPin;


//Função de inicialização corrigida baseada no código proposto em: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1591
void MPU6050_Init(I2C_TypeDef *I2Cx, MPU6050_InitStruct *initialConfig) {
	uint8_t i2cDataBuffer = 0x00;

    //Reinicia o dispotivio;
    i2cDataBuffer = MPU6050_DEVICE_RESET;
    TM_I2C_Write(I2Cx, MPU6050_ADDRESS, PWR_MGMT_1, i2cDataBuffer);

//    //Configuração da fonte de clock e do modo de energia (PWR_MGMT_1) - Mantém em modo SLEEP;
//    i2cDataBuffer = TM_I2C_Read(I2Cx, MPU6050_ADDRESS, PWR_MGMT_1);
//    i2cDataBuffer |= initialConfig->clockSource | initialConfig->temperatureSensorDisabled;
//    TM_I2C_Write(I2Cx, MPU6050_ADDRESS, PWR_MGMT_1, i2cDataBuffer);

    //Configuração da frequência de amostragem do acelerômetro.
    i2cDataBuffer = initialConfig->sampleRateDivider;
    TM_I2C_Write(I2Cx, MPU6050_ADDRESS, SMPLRT_DIV, i2cDataBuffer);

    //    //Configuração da fonte de clock e do modo de energia (PWR_MGMT_1)
    i2cDataBuffer |= initialConfig->clockSource | initialConfig->temperatureSensorDisabled;
    TM_I2C_Write(I2Cx, MPU6050_ADDRESS, PWR_MGMT_1, i2cDataBuffer);

    //Configuração do fundo de escala do giroscópio.
    i2cDataBuffer = initialConfig->gyroFullScale;
    TM_I2C_Write(I2Cx, MPU6050_ADDRESS, GYRO_CONFIG, i2cDataBuffer);

    //Configuração do estado das FIFOS
    i2cDataBuffer = initialConfig->fifoEnabled;
    TM_I2C_Write(I2Cx, MPU6050_ADDRESS, USER_CTRL, i2cDataBuffer);

    //Configuração do pino de interurpção.
    i2cDataBuffer = initialConfig->intPinConfig;
    TM_I2C_Write(I2Cx, MPU6050_ADDRESS, INT_PIN_CFG, i2cDataBuffer);

    //Configuração do filtro passa baixa e da frequência de aquisição do gyro (Base do accel).
    i2cDataBuffer = initialConfig->digitalLowPassConfig;
    TM_I2C_Write(I2Cx, MPU6050_ADDRESS, CONFIG_MPU6050, i2cDataBuffer);

    //Configuração do fundo de escala do acelerômetro
    i2cDataBuffer = initialConfig->accelFullScale;
    TM_I2C_Write(I2Cx, MPU6050_ADDRESS, ACCEL_CONFIG, i2cDataBuffer);
    //Configuração das interrupções.
    i2cDataBuffer = initialConfig->interruptsConfig;
    TM_I2C_Write(I2Cx, MPU6050_ADDRESS, INT_ENABLE_MPU6050, i2cDataBuffer);

    //Configuração da fonte de clock e do modo de energia (PWR_MGMT_1) - Retira a do modo SLEEP;
    i2cDataBuffer = TM_I2C_Read(I2Cx, MPU6050_ADDRESS, PWR_MGMT_1);
    //Desliga o BIT de SLEEP
    i2cDataBuffer &= (~MPU6050_SLEEP);
    TM_I2C_Write(I2Cx, MPU6050_ADDRESS, PWR_MGMT_1, i2cDataBuffer);

//FIXME: Modificações teste - Testar se necessita de escrita no PWR_MGMT_1 neste ponto, após todas as configurações.
//    //Configuração da fonte de clock e do modo de energia (PWR_MGMT_1);
//    i2cDataBuffer = initialConfig->clockSource | initialConfig->powerMode | initialConfig->temperatureSensorDisabled;
//    I2C_escrever_registrador(I2Cx, MPU6050_ADDRESS, PWR_MGMT_1, 1, &i2cDataBuffer);

    switch(initialConfig->gyroFullScale) {
    case FS_250DPS:
        gyroFullScale = 250;
        break;

    case FS_500DPS:
        gyroFullScale = 500;
        break;

    case FS_1000DPS:
        gyroFullScale = 1000;
        break;

    case FS_2000DPS:
        gyroFullScale = 2000;
        break;
    }

    switch(initialConfig->accelFullScale) {
    case AFS_2G:
        accelFullScale = 2;
        break;

    case AFS_4G:
        accelFullScale = 4;
        break;

    case AFS_8G:
        accelFullScale = 8;
        break;

    case AFS_16G:
        accelFullScale = 16;
        break;
    }
}

uint8_t MPU6050_checkConectivity(I2C_TypeDef* I2Cx)
{
    uint8_t readData;
    readData = TM_I2C_Read(I2Cx, MPU6050_ADDRESS, WHO_AM_I_MPU6050);

    return readData;
}

float MPU6050_readData(I2C_TypeDef* I2Cx, float accelBuffer[3], float gyroBuffer[3]) {
    uint8_t tempReadData[14];

    TM_I2C_ReadMulti(I2Cx, MPU6050_ADDRESS, ACCEL_XOUT_H, tempReadData, 14);

    accelBuffer[0] = (int16_t)((tempReadData[0]*256)|(tempReadData[1]));
    accelBuffer[1] = (int16_t)((tempReadData[2]*256)|(tempReadData[3]));
    accelBuffer[2] = (int16_t)((tempReadData[4]*256)|(tempReadData[5]));

    accelBuffer[0] = accelBuffer[0]*accelFullScale/((float)32768);
    accelBuffer[1] = accelBuffer[1]*accelFullScale/((float)32768);
    accelBuffer[2] = accelBuffer[2]*accelFullScale/((float)32768);

    gyroBuffer[0] = (int16_t)((tempReadData[8]*256)|(tempReadData[9]));
    gyroBuffer[1] = (int16_t)((tempReadData[10]*256)|(tempReadData[11]));
    gyroBuffer[2] = (int16_t)((tempReadData[12]*256)|(tempReadData[13]));

    gyroBuffer[0] = gyroBuffer[0]*gyroFullScale/((float)32768);
    gyroBuffer[1] = gyroBuffer[1]*gyroFullScale/((float)32768);
    gyroBuffer[2] = gyroBuffer[2]*gyroFullScale/((float)32768);

    //FIXME: Corrigir a escala para obtenção da temperatura.
    return (tempReadData[6]*256)|(tempReadData[7]);
}

uint8_t MPU6050_checkDataReadyIntPin() {
    return GPIO_ReadInputDataBit(dataRdyIntPort, dataRdyIntPin);
}

void MPU6050_configIntPin(uint32_t RCC_AHB1Periph, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    dataRdyIntPort = GPIOx;
    dataRdyIntPin = GPIO_Pin;

    //Configuração do PINO para checar se há dados disponíveis.
    //PINO PD0
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph, ENABLE);

    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = dataRdyIntPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(dataRdyIntPort, &GPIO_InitStructure);
}

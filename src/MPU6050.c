#include "MPU6050.h"

uint16_t accelFullScale;
uint16_t gyroFullScale;

void MPU6050_Init(I2C_TypeDef *I2Cx, MPU6050_InitStruct *initialConfig) {
	uint8_t i2cDataBuffer = 0x00;

    //Configuração da fonte de clock e do modo de energia (PWR_MGMT_1);
    i2cDataBuffer = initialConfig->clockSource | initialConfig->powerMode | initialConfig->temperatureSensorDisabled;
    I2C_escrever_registrador(I2Cx, MPU6050_ADDRESS, PWR_MGMT_1, 1, &i2cDataBuffer);

    //Configuração do filtro passa baixa e da frequência de aquisição do gyro (Base do accel).
    i2cDataBuffer = initialConfig->digitalLowPassConfig;
    I2C_escrever_registrador(I2Cx, MPU6050_ADDRESS, CONFIG_MPU6050, 1, &i2cDataBuffer);

    //Configuração da frequência de amostragem do acelerômetro.
    i2cDataBuffer = initialConfig->sampleRateDivider;
    I2C_escrever_registrador(I2Cx, MPU6050_ADDRESS, SMPLRT_DIV, 1, &i2cDataBuffer);

    //Configuração do fundo de escala do giroscópio.
    i2cDataBuffer = initialConfig->gyroFullScale;
    I2C_escrever_registrador(I2Cx, MPU6050_ADDRESS, GYRO_CONFIG, 1, &i2cDataBuffer);

    //Configuração do fundo de escala do acelerômetro
    i2cDataBuffer = initialConfig->accelFullScale;
    I2C_escrever_registrador(I2Cx, MPU6050_ADDRESS, ACCEL_CONFIG, 1, &i2cDataBuffer);

    //Configuração do estado das FIFOS
    i2cDataBuffer = initialConfig->fifoEnabled;
    I2C_escrever_registrador(I2Cx, MPU6050_ADDRESS, USER_CTRL, 1, &i2cDataBuffer);

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
    I2C_ler_registradores(I2Cx, MPU6050_ADDRESS, WHO_AM_I_MPU6050, 1, &readData);

    return readData;
}

float MPU6050_readData(I2C_TypeDef* I2Cx, float accelBuffer[3], float gyroBuffer[3]) {
    uint8_t tempReadData[14];
    I2C_ler_registradores(I2Cx, MPU6050_ADDRESS, ACCEL_XOUT_H, 14, tempReadData);

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

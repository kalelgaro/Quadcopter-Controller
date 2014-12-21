#include "HMC5883L.h"

float ganho;

GPIO_TypeDef* dataRdyIntPort;
uint16_t dataRdyIntPin;

float magScaleCorrectingFactor[] = {1, 1, 1};

void HMC5883L_Init(I2C_TypeDef *I2Cx, HMC5883L_InitTypeDef *Configuracoes)
{
	uint8_t data_buffer = 0x00;

	/* Configuração do registrador A */
	data_buffer = (Configuracoes->Samples | Configuracoes->Output_DataRate | Configuracoes->Meas_mode);
    TM_I2C_Write(I2Cx, end_HMC5883L, CONFIG_A, data_buffer);

	/* Configuração do registrador B */
	data_buffer = 0x00;
	data_buffer = (Configuracoes->Gain);
    TM_I2C_Write(I2Cx, end_HMC5883L, CONFIG_B, data_buffer);

	/*Configuração do registrador de movo */
	data_buffer = 0x00;
	data_buffer = (Configuracoes->HS_I2C | Configuracoes->Mode);
	
    TM_I2C_Write(I2Cx, end_HMC5883L, MODE, data_buffer);

    switch(Configuracoes->Gain)
    {

    case Gain_0_73:
        ganho = 0.73;
        break;

    case Gain_0_92:
        ganho = 0.92;
        break;

    case Gain_1_22:
        ganho = 1.22;
        break;

    case Gain_1_52:
        ganho = 1.52;
        break;

    case Gain_2_27:
        ganho = 2.27;
        break;

    case Gain_2_56:
        ganho = 2.56;
        break;

    case Gain_3_03:
        ganho = 3.03;
        break;

    case Gain_4_35:
        ganho = 4.35;
        break;
    }

    ganho = ganho/1000;
}

float  HMC5883L_Read_Data(I2C_TypeDef *I2Cx, float dados[])
{
	uint8_t buffer_dados[6];
	
	int16_t buffer_temp;
	
    TM_I2C_ReadMulti(I2Cx, end_HMC5883L, 0x03, buffer_dados, 6);

	buffer_temp = (int16_t)(buffer_dados[0]*256);
	buffer_temp = (int16_t)(buffer_temp | buffer_dados[1]);
    dados[0] = (buffer_temp)*ganho*magScaleCorrectingFactor[0];

	buffer_temp = (int16_t)(buffer_dados[2]*256);
	buffer_temp = (int16_t)(buffer_temp | buffer_dados[3]);
    dados[2] = (buffer_temp)*ganho*magScaleCorrectingFactor[1];
	
	
	buffer_temp = (int16_t)(buffer_dados[4]*256);
	buffer_temp = (int16_t)(buffer_temp | buffer_dados[5]);
    dados[1] = (buffer_temp)*ganho*magScaleCorrectingFactor[2];

	return 0;
}

uint8_t HMC5883L_checkDataReadyIntPin() {
    return GPIO_ReadInputDataBit(dataRdyIntPort, dataRdyIntPin);
}

void HMC5883L_configIntPin(uint32_t RCC_AHB1Periph, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
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


//Procedimentos de calibração baseados em https://github.com/helscream/HMC5883L_Header_Arduino_Auto_calibration;

//Calcula o erro de escala de cada um três eixos do magnetometro. Este não necessariamente necessita
//estar inicializado para este teste.
void HMC5883L_getMagScale(I2C_TypeDef *I2Cx) {
    HMC5883L_InitTypeDef initialStruct;

    initialStruct.Samples = _8_samples;
    initialStruct.Output_DataRate = _75_0_HZ;
    initialStruct.Meas_mode = Positive_bias;
    //initialStruct.Meas_mode = Negative_bias;
    initialStruct.Gain = Gain_1_22;
    initialStruct.Mode = Countinuous;
    initialStruct.HS_I2C = 0;

    HMC5883L_Init(I2Cx, &initialStruct);

    float magReadings[3];
    float magScaleError[3];

    //Realiza uma leitura do mag
    HMC5883L_Read_Data(I2Cx, magReadings);

    //Permanece no laço enquanto a leitura não sesteja positivamente desviada (Positive Bias)
    while((magReadings[0] < 0.300) | (magReadings[1] < 0.300) | (magReadings[2] < 0.300)) {
        HMC5883L_Read_Data(I2Cx, magReadings);
    }

    //Calcula os erros de escala.
    magScaleError[0] = (float)DEFAULT_SELF_TEXT_XY_FIELD/magReadings[0];
    magScaleError[1] = (float)DEFAULT_SELF_TEXT_XY_FIELD/magReadings[1];
    magScaleError[2] = (float)DEFAULT_SELF_TEXT_Z_FIELD/magReadings[2];


    //Prepara o sensor para obtenção dos valores de erro de ganho no modo de desvio negativo (Negative biaS)
    initialStruct.Meas_mode = Negative_bias;
    HMC5883L_Init(I2Cx, &initialStruct);

    //Realiza uma leitura do mag
    HMC5883L_Read_Data(I2Cx, magReadings);

    //Permanece no laço enquanto a leitura não sesteja positivamente desviada (Positive Bias)
    while((magReadings[0] > -0.300) | (magReadings[1] > -0.300) | (magReadings[2] > -0.300)) {
        HMC5883L_Read_Data(I2Cx, magReadings);
    }

    magScaleError[0] = (float)((DEFAULT_SELF_TEXT_XY_FIELD/fabsf(magReadings[0])) + magScaleError[0])/(float)2;
    magScaleError[1] = (float)((DEFAULT_SELF_TEXT_XY_FIELD/fabsf(magReadings[1])) + magScaleError[1])/(float)2;
    magScaleError[2] = (float)((DEFAULT_SELF_TEXT_Z_FIELD/fabsf(magReadings[2])) + magScaleError[2])/(float)2;

    //Sai dos modos de excitação
    initialStruct.Meas_mode = Default_Meas;
    HMC5883L_Init(I2Cx, &initialStruct);
}

//Calcula o offset nso três eixos do magnetometro. este deve estar inicializado antes de executar este método
float* HMC5883L_getMagOffset(I2C_TypeDef *I2Cx)
{
    //Inicia as variáveis
    float xMax = -5000;     float xMin = 5000;

    float yMax = -5000;     float yMin = 5000;

    float zMax = -5000;     float zMin = 5000;

    //Variável que conta o tempo durante um minuto (60segundos/100us),
    uint32_t delayTime = 60000;

    float offsetReadings[3];

    //Realiza algumas leituras no magnetometro
    uint8_t counter = 10;
    while(counter--) {
        HMC5883L_Read_Data(I2Cx, offsetReadings);
    }

    while(delayTime> 0) {
        HMC5883L_Read_Data(I2Cx, offsetReadings);

        //Atualiza os valores de máximo das leituras;
        xMax = max(xMax, offsetReadings[0]);
        yMax = max(yMax, offsetReadings[1]);
        zMax = max(zMax, offsetReadings[2]);

        //Atualiza os valores de mínimo das leituras;
        xMin = min(xMin, offsetReadings[0]);
        yMin = min(yMin, offsetReadings[1]);
        zMin = min(zMin, offsetReadings[2]);

        //Delay de 1 ms
        delay(10);
        delayTime--;
    }

    float magOffset[3];

    magOffset[0] = ((xMax+xMin)/(float)2);
    magOffset[1] = ((yMax+yMin)/(float)2);
    magOffset[2] = ((zMax+zMin)/(float)2);

    uint8_t teste;
    teste = 1;
    return magOffset;
}

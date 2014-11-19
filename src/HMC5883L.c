#include "HMC5883L.h"

float ganho;

GPIO_TypeDef* dataRdyIntPort;
uint16_t dataRdyIntPin;

void HMC5883L_Init(I2C_TypeDef *I2Cx, HMC5883L_InitTypeDef *Configuracoes)
{
	uint8_t data_buffer = 0x00;

	/* Configuração do registrador A */
	data_buffer = (Configuracoes->Samples | Configuracoes->Output_DataRate | Configuracoes->Meas_mode);
	I2C_escrever_registrador(I2Cx, end_HMC5883L, CONFIG_A, 1, &data_buffer);

	/* Configuração do registrador B */
	data_buffer = 0x00;
	data_buffer = (Configuracoes->Gain);
	I2C_escrever_registrador(I2Cx, end_HMC5883L, CONFIG_B, 1, &data_buffer);

	/*Configuração do registrador de movo */
	data_buffer = 0x00;
	data_buffer = (Configuracoes->HS_I2C | Configuracoes->Mode);
	
	I2C_escrever_registrador(I2Cx, end_HMC5883L, MODE, 1, &data_buffer);

	switch(Configuracoes->Gain)	
	{
		case 0x20:
			ganho = 0.92;
			//ganho = 1;
		break;

		case 0xA0:
			ganho = 2.56e-3;
		break;
	}
}

float  HMC5883L_Read_Data(I2C_TypeDef *I2Cx, float dados[])
{
	uint8_t buffer_dados[6];
	
	int16_t buffer_temp;
	
	I2C_ler_registradores(I2Cx, end_HMC5883L, 0x03, 6, buffer_dados);

	buffer_temp = (int16_t)(buffer_dados[0]*256);
	buffer_temp = (int16_t)(buffer_temp | buffer_dados[1]);
	dados[0] = (buffer_temp)*ganho;

	buffer_temp = (int16_t)(buffer_dados[2]*256);
	buffer_temp = (int16_t)(buffer_temp | buffer_dados[3]);
	dados[2] = (buffer_temp)*ganho;
	
	
	buffer_temp = (int16_t)(buffer_dados[4]*256);
	buffer_temp = (int16_t)(buffer_temp | buffer_dados[5]);
	dados[1] = (buffer_temp)*ganho;

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

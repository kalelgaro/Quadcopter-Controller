#include "stm32f4_discovery.h"

#include "HMC5883L.h"
#include "aquisicao_IMU.h"
#include "arm_math.h"

float ganho;

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
			//ganho = 2.56;
			ganho=1;
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
	//dados[0] = (buffer_temp-25)*0.01333333;
	dados[0] = (buffer_temp)*0.01333333;

	buffer_temp = (int16_t)(buffer_dados[2]*256);
	buffer_temp = (int16_t)(buffer_temp | buffer_dados[3]);
	//dados[2] = (buffer_temp+59.5)*0.0148148;
	dados[2] = (buffer_temp)*0.0148148;
	
	buffer_temp = (int16_t)(buffer_dados[4]*256);
	buffer_temp = (int16_t)(buffer_temp | buffer_dados[5]);
	//dados[1] = (buffer_temp-25)*0.01333333;
	dados[1] = (buffer_temp)*0.01333333;

	return 0;
}
/*
 * ADXL345.C
 *
 *  Created on: Jul 21, 2013
 *      Author: Bruno
 */

#include "ADXL345.h"
#include "aquisicao_IMU.h"

float resolucao = 0;
uint8_t escala ;

float ADXL345_Init(I2C_TypeDef* I2Cx, ADXL345_InitTypeDef *Configuracoes)
{
	uint8_t data_buffer = 0x00;

	data_buffer = Configuracoes->bandwidth;

	I2C_escrever_registrador(I2Cx, end_ADXL345, BW_RATE, 1, &data_buffer);

	data_buffer = 0x00;

	data_buffer = Configuracoes->Self_Test | Configuracoes->Resolution | Configuracoes->Full_Scale;	//Padrão justificado à esquerda.

	I2C_escrever_registrador(I2Cx, end_ADXL345, DATA_FORMAT, 1, &data_buffer);

	escala = (Configuracoes->Full_Scale);

	I2C_ler_registradores(I2Cx, end_ADXL345, POWER_CTL, 1, &data_buffer);

	data_buffer &= 0b11000000;
	data_buffer = data_buffer | Configuracoes->Power_Mode;

	I2C_escrever_registrador(I2Cx, end_ADXL345, POWER_CTL, 1, &data_buffer);

	if((((Configuracoes->Resolution)>>3)&0x01) == 0x01)		//Bit de Full res. não está setado -> Resolução de 10 bits (escala 2g)
	{

		resolucao = 0.0039;
	}
	else 													//if(((Configuracoes->Resolution>>3)&0x01) == 0x01)
	{
		switch(escala)
		{
			case 0:
				resolucao = 0.0039;
				escala = 0;
			break;

			case 1:
				resolucao = 0.0078;
				escala = 0;
			break;

			case 2:
				resolucao = 0.0156;
				escala = 0;
			break;

			case 3:
				resolucao = 0.0312;
				escala = 0;
			break;
		}
	}
	return resolucao;
}

void ADXL345_Read_Data(I2C_TypeDef* I2Cx, float buffer_retorno[])
{
	uint8_t buffer_leitura[6];

	int16_t buffer_calc;


	I2C_ler_registradores(I2Cx, end_ADXL345, (DATA_X0), 6, buffer_leitura);


	buffer_calc = (int16_t)((buffer_leitura[1] << 8) | buffer_leitura[0]);

	buffer_retorno[0] = buffer_calc * resolucao;


	buffer_calc = (int16_t)((buffer_leitura[3] << 8) | buffer_leitura[2]);

	buffer_retorno[1] = buffer_calc * resolucao;


	buffer_calc = (int16_t)((buffer_leitura[5] << 8) | buffer_leitura[4]);

	buffer_retorno[2] = buffer_calc * resolucao;

}

void ADXL345_Set_Offset(I2C_TypeDef* I2Cx, uint8_t offsets[])
{
	I2C_escrever_registrador(I2Cx, end_ADXL345, OFSX, 3, offsets);
}


/*
 * L3G4200D.c
 *
 *  Created on: May 28, 2013
 *      Author: Bruno
 */
#include "L3G4200D.h"

uint16_t full_scale;

void L3G4200D_Init(I2C_TypeDef* I2Cx, L3G4200D_InitTypeDef *Configuracoes)
{
	uint8_t data_buffer = 0x00;
	data_buffer = Configuracoes->Output_DataRate | Configuracoes->bandwidth | Configuracoes->Axes_Enable | Configuracoes->Power_Mode;
	I2C_escrever_registrador(I2Cx, end_L3G4200, CTRL_REG1, 1, &data_buffer);

	data_buffer = (Configuracoes->Full_Scale | Configuracoes->Self_Test | Configuracoes->BDU_Enabled);
	I2C_escrever_registrador(I2Cx, end_L3G4200, CTRL_REG4, 1, &data_buffer);

	full_scale = (Configuracoes->Full_Scale >> 4);
	switch(full_scale)
	{
		case 0:
			full_scale = 250;
		break;

		case 1:
			full_scale = 500;
		break;

		case 2:
			full_scale = 2000;
		break;

		case 3:
			full_scale = 2000;
		break;
	}
}

void L3G4200D_HP_Init(I2C_TypeDef* I2Cx, L3G4200D_HPFilterConfigTypeDef *Configuracoes_HPF)
{
	uint8_t data_buffer = 0x00;
	data_buffer = (Configuracoes_HPF->HP_mode | Configuracoes_HPF->HP_cutoff_freq)&0b00111111;
	I2C_escrever_registrador(I2Cx, end_L3G4200, CTRL_REG2, 1, &data_buffer);

	data_buffer = ((Configuracoes_HPF->Filter_Enable | Configuracoes_HPF->Output__Selection));

	I2C_escrever_registrador(I2Cx, end_L3G4200, CTRL_REG5, 1, &data_buffer);

}

void L3G4200D_Read_Data(I2C_TypeDef* I2Cx, float32_t buffer_retorno[])
{
	uint8_t reading_buffer[6];

	I2C_ler_registradores(I2Cx, end_L3G4200, (0x80|OUT_X_L), 6, reading_buffer);

  	buffer_retorno[0] = (int16_t)((reading_buffer[0]|(reading_buffer[1]*256)));
	buffer_retorno[0] = buffer_retorno[0]*full_scale/((float32_t)32768);

	buffer_retorno[1] = (int16_t)((reading_buffer[2]|(reading_buffer[3]*256)));
	buffer_retorno[1] = buffer_retorno[1]*full_scale/((float32_t)32768);

	buffer_retorno[2] = (int16_t)((reading_buffer[4]|(reading_buffer[5]*256)));
	buffer_retorno[2] = buffer_retorno[2]*full_scale/((float32_t)32768);

}

float L33G4200D_Read_Temperatura(I2C_TypeDef* I2Cx)
{
	uint8_t reading_buffer;

	I2C_ler_registradores(I2Cx, end_L3G4200, OUT_TEMP, 1, &reading_buffer);

	return reading_buffer;
}

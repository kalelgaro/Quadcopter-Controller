#ifndef HMC5883L_H_
#define HMC5883L_H_

#include "stm32f4xx.h"

	/*Endereço dos endereços de configuração*/
	#define WHO_AM_I_MG				10


	/*Registradores de configuração*/
	#define CONFIG_A 				00
	#define CONFIG_B 				01
	#define MODE	 				02
	

	/*Registradores de leitura de dados */
	/*X*/
	#define Output_X_MSB 			03
	#define Output_X_LSB 			04

	/*Z*/
	#define Output_Z_MSB 			05
	#define Output_Z_LSB 			06

	/*Y*/
	#define Output_Y_MSB 			07
	#define Output_Y_LSB 			08

	
	/*Registrador de Status*/
	#define STATUS_MG				0x09

	/*Registrador de identificação A*/
	#define Ident_A					0x0A

	/* Defines do registrador CONFIG_A */
	
	#define _8_samples 				0b01100000
	#define _4_samples 				0b01000000
	#define _2_samples 				0b00100000
	#define _1_samples 				0b00000000

	#define _0_75_HZ 				0x00
	#define _1_50_HZ 				0x04
	#define _3_00_HZ 				0x08
	#define _7_50_HZ 				0x0C
	#define _15_0_HZ 				0x10
	#define _30_0_HZ 				0x14
	#define _75_0_HZ 				0x18

	#define Default_Meas 			0x00
	#define Positive_bias 			0x01
	#define Negative_bias 			0x02

	/* Defines do registrador CONFIG_B */

	#define Gain_Default 			0x20
	#define Gain_5					0xA0

	/* Defines do registrador MODE */

	#define HS 						0b10000000
	
	#define Countinuous 			0b00000000
	#define Single_meas 			0b00000001
	#define Idle					0b00000010

	typedef struct
	{
		uint8_t Samples;			//Número de medidas que serão realizadas.
		uint8_t Output_DataRate;	//Frequência de medições.
		uint8_t Meas_mode;			//Modo de operação do CI.
		uint8_t Gain;				//Ganho do dispositivo.
		uint8_t HS_I2C;				//Modo de alta velocidade do I2C -> 3400 Hz.
		uint8_t Mode;				//Modo de operação : Continuo, Single, Idle.
	}HMC5883L_InitTypeDef;

	void HMC5883L_Init(I2C_TypeDef*, HMC5883L_InitTypeDef*);
	float HMC5883L_Read_Data(I2C_TypeDef*, float []);

#endif 
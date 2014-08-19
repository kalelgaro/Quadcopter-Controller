/*
 * ADXL345.h
 *
 *  Created on: Jul 21, 2013
 *      Author: Bruno
 */

#ifndef ADXL345_H_
#define ADXL345_H_

#include "stm32f4xx.h"
#include "aquisicao_IMU.h"
#include "arm_math.h"

	#define DEVICE_ID		0x00		//Default - 0b11100101

	#define THRESH_TAP 		0x1D

	#define OFSX			0x1E
	#define OFSY			0x1F
	#define OFSZ			0x20

	#define DUR				0x21

	#define Latent			0x22

	#define Window			0x23

	#define THRESH_ACT	 	0x24
	#define THRESH_INACT 	0x25

	#define TIME_INACT		0x26
	#define ACT_INACT_CTL	0x27

	#define THRESH_FF		0x28
	#define TIME_FF			0x29

	#define TAP_AXES		0x2A

	#define ACT_TAP_STATUS	0x2B

	#define BW_RATE			0x2C

	#define POWER_CTL		0x2D

	#define INT_ENABLE		0x2E
	#define INT_MAP			0x2F
	#define INT_SOURCE		0x30

	#define DATA_FORMAT		0x31

	#define DATA_X0			0x32
	#define DATA_X1			0x33

	#define DATA_Y0			0x34
	#define DATA_Y1			0x35

	#define DATA_Z0			0x36
	#define DATA_Z1			0x37

	#define FIFO_CTL		0x38
	#define FIFO_STATUS		0x39

	/*----Configuração dos registradores----*/

	/*---Register 0x2A—TAP_AXES (Read/Write)---*/

	#define Suppress 		0b00001000

	#define TAP_X			0b00000100
	#define TAP_Y			0b00000010
	#define TAP_Z			0b00000001

	/*---Register 0x2C—BW_RATE (Read/Write))---*/

	#define	LOW_POWER		0b00010000

	#define	Rate_D3			0b00001000
	#define	Rate_D2			0b00000100
	#define	Rate_D1			0b00000010
	#define	Rate_D0			0b00000001

	/*---Register 0x2D—POWER_CTL (Read/Write)---*/

	#define LINK			0b00100000
	#define AUTO_SLEEP		0b00010000

	#define Measure			0b00001000

	#define Sleep			0b00000100

	#define Wakeup_D1		0b00000010
	#define Wakeup_D2		0b00000001

	/*---Register 0x2E—INT_ENABLE (Read/Write)--- && ---Register 0x2F—INT_MAP (R/!W)---*/

	#define Data_Ready		0b10000000
	#define Single_Tap		0b01000000
	#define Double_Tap		0b00100000
	#define Activity		0b00010000
	#define Inactivity		0b00001000
	#define	Free_Fall		0b00000100
	#define Watermark		0b00000010
	#define Overrun			0b00000001

	/*---Register 0x31—DATA_FORMAT (Read/Write)---*/

	#define	SelfTest		0b10000000

	#define SPI				0b01000000

	#define Int_Invert		0b00100000

	#define Full_Res		0b00001000

	#define Justify			0b00000100

	#define Range_D1		0b00000010
	#define Range_D0		0b00000001

	/*---Register 0x38—FIFO_CTL (Read/Write)---*/

	#define Fifo_Mode_D1	0b10000000
	#define Fifo_Mode_D0	0b01000000

	#define Trigger			0b00100000

	#define Samples_D4		0b00010000
	#define Samples_D3		0b00001000
	#define Samples_D2		0b00000100
	#define Samples_D1		0b00000010
	#define Samples_D0		0b00000001

typedef struct
{
	uint8_t Power_Mode;
	uint8_t bandwidth;
	uint8_t Resolution;
	uint8_t Self_Test;
	uint8_t Full_Scale;

}ADXL345_InitTypeDef;

	float ADXL345_Init(I2C_TypeDef*, ADXL345_InitTypeDef *);
	void ADXL345_Read_Data(I2C_TypeDef*, float []);
	void ADXL345_Set_Offset(I2C_TypeDef*, uint8_t[]);

#endif /* ADXL345_H_ */

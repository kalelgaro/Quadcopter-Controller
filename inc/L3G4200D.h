/*
 * L3G4200D.h
 *
 *  Created on: May 28, 2013
 *      Author: Bruno
 */


#ifndef L3G4200D_H_
#define L3G4200D_H_

#include "stm32f4xx.h"
#include "aquisicao_IMU.h"
#include "arm_math.h"

	/*Endereço dos registradores*/
	#define WHO_AM_I 0x0F

	#define CTRL_REG1 0x20
	#define CTRL_REG2 0x21
	#define CTRL_REG3 0x22
	#define CTRL_REG4 0x23
	#define CTRL_REG5 0x24

	#define REFERENCE_DATACAPTURE 0x25

	#define OUT_TEMP 0x26

	#define STATUS_REG 0x27

	#define OUT_X_L 0x28
	#define OUT_X_H 0x29

	#define OUT_Y_L 0x2A
	#define OUT_Y_H 0x2B

	#define OUT_Z_L 0x2C
	#define OUT_Z_H 0x2D

	#define FIFO_CTRL_REG 0x2E
	#define FIFO_SRC_REG 0x2F

	#define INT1_CFG 0x30
	#define INT1_SRC 0x31

	#define INT1_THS_XH 0x32
	#define INT1_THS_XL 0x33

	#define INT1_THS_YH 0x34
	#define INT1_THS_YL 0x35

	#define INT1_THS_ZH 0x36
	#define INT1_THS_ZL 0x37

	#define INT1_DURATION 0x38

	/*Configuração dos registradores*/

	/*CTRL_REG1*/
	#define DR1 			0b10000000
	#define DR0 			0b01000000

	#define BW1	 			0b00100000
	#define BW0 			0b00010000

	#define P_DOWN_MODE 	0b00000000
	#define NORMAL_MODE 	0b00001000

	#define Z_EN 			0b00000100
	#define Y_EN 			0b00000010
	#define X_EN 			0b00000001

	#define XYZ_EN			0b00000111
	/*---------------------------------------------------*/

	/*CTRL_REG2*/
	#define NORMAL_MODE_RF 	0b00000000
	#define REFERENCE 		0b00010000
	#define NORMALMODE		0b00100000
	#define AUTO_RESET		0b00110000

	#define HPM1			0b00100000
	#define HPM0			0b00010000

	#define HPCF3			0b00001000
	#define HPCF2			0b00000100
	#define HPCF1			0b00000010
	#define HPCF0			0b00000001
	/*---------------------------------------------------*/

	/*CTRL_REG3*/
	#define I1_Int1			0b10000000
	#define I1_Boot			0b01000000

	#define H_LActive		0b00100000

	#define PP_OD			0b00010000

	#define I2_DRDY			0b00001000
	#define I2_WTM			0b00000100
	#define I2_ORun			0b00000010
	#define I2_EMPTY		0b00000001
	/*---------------------------------------------------*/

	/*CTRL_REG4*/
	#define BDU				0b10000000
	#define BLE				0b01000000

	#define FS1				0b00100000
	#define FS0				0b00010000

	#define ST1				0b00000100
	#define ST0				0b00000010

	#define FS250DPS		0b00000000
	#define FS500DPS		0b00010000
	#define FS2000DPS		0b00110000

	#define ST_NORMAL		0b00000000
	#define ST_0			0b00000010
	#define ST_1			0b00000110

	#define SIM				0b00000001
	/*---------------------------------------------------*/

	/*CTRL_REG5*/
	#define BOOT			0b10000000

	#define FIFO_EN			0b01000000
	#define	HPen			0b00010000

	#define INT1_Sel1		0b00001000
	#define INT1_Sel0		0b00000100

	#define Out_Sel1		0b00000010
	#define Out_Sel0		0b00000001
	/*---------------------------------------------------*/

	/*FIFO_CTRL_REG*/
	#define FM2				0b10000000
	#define FM1				0b01000000
	#define FM0				0b00100000

	#define WTM4			0b00010000
	#define	WTM3			0b00001000
	#define	WTM2			0b00000100
	#define	WTM1			0b00000010
	#define WTM0			0b00000001

	#define Bypass_mode		0b00000000
	#define FIFO_mode		0b00100000
	#define Stream_mode		0b01000000
	#define Stream_to_FIFO	0b01100000
	#define Bypass_to_Stream 0b10000000
	/*---------------------------------------------------*/

	/*INT1_CFG*/
	#define AND_OR			0b10000000
	#define LIR				0b01000000

	#define ZHIE			0b00100000
	#define ZLIE			0b00010000

	#define YHIE			0b00001000
	#define YLIE			0b00000100

	#define XHIE			0b00000010
	#define XLIE			0b00000001
	/*---------------------------------------------------*/

	/*INT1_DURATION*/
	#define WAIT			0b10000000

	#define D6				0b01000000
	#define D5				0b00100000
	#define D4				0b00010000
	#define D3				0b00001000
	#define D2				0b00000100
	#define D1				0b00000010
	#define D0				0b00000001
	/*---------------------------------------------------*/

typedef struct
{
	uint8_t Power_Mode;
	uint8_t Axes_Enable;
	uint8_t Output_DataRate;
	uint8_t bandwidth;
	uint8_t Full_Scale;
	uint8_t Self_Test;
	uint8_t BDU_Enabled;
}L3G4200D_InitTypeDef;

typedef struct
{
	uint8_t Filter_Enable;
	uint8_t	Output__Selection;
	uint8_t	HP_cutoff_freq;
	uint8_t	HP_mode;

}L3G4200D_HPFilterConfigTypeDef;

	void L3G4200D_Init(I2C_TypeDef*, L3G4200D_InitTypeDef *);
	void L3G4200D_HP_Init(I2C_TypeDef* ,L3G4200D_HPFilterConfigTypeDef*);
	void L3G4200D_Read_Data(I2C_TypeDef* , float []);
	float L33G4200D_Read_Temperatura(I2C_TypeDef* I2Cx);

#endif /* L3G4200D_H_ */

/*
 * kalman_filter.c
 *
 *  Created on: Sep 8, 2013
 *      Author: Bruno
 */
#ifndef ARM_MATH_CM4
	#define ARM_MATH_CM4
#endif

#define ARM_MATH_MATRIX_CHECK


#ifndef __FPU_USED
  	#define __FPU_USED 1
#endif

#ifndef __FPU_PRESENT
  	#define __FPU_PRESENT 1
#endif

#include <stm32f4xx_conf.h>
#include "kalman_filter.h"
#include "arm_math.h"

//------ Função para cálculo do filtro de kalman sobre o Giroscópio e o Acelerômetro -------///
//Utiliza DCM -> Variação do vetor com base na velocidae agular para atualização da matrz de atualização de estados.
//Bruno f. M. Callegaro - 20/12/2013


void kalman_filter(kalman_filter_state *buffer_filtro, float medida_gyro[], float medida_accel[], float medida_mag[])
{
	//Instancias das matrizes utilizadas para o cálculo
	arm_matrix_instance_f32 X;			//Matriz de estados. [12,1]
	arm_matrix_instance_f32 FJ;			//Matriz de transição de estados. [12,12]
	arm_matrix_instance_f32 FJt;		//Matriz de transição de estados transposta. [12,12]
	arm_matrix_instance_f32 I;			//Matriz identidadee. [12,12]
	arm_matrix_instance_f32 P;			//Matriz de confiabilidade do processo de atualização. [12,12]
	arm_matrix_instance_f32 Pdot;		
	arm_matrix_instance_f32 H;			//Matriz de mapeamento do estado para o erro [6,12]
	arm_matrix_instance_f32 Ht;			//Matriz de mapeamento do estado para o erro transposta. [12,6]
	arm_matrix_instance_f32 Q;			//Matriz de covariância multiplicada por dt; [12,12]
	arm_matrix_instance_f32 R;			//Matriz de variância [6,6]
	arm_matrix_instance_f32 y;			//Matriz de erro entre medidas e estado estimado. [6,1]
	arm_matrix_instance_f32 S;			//Matriz .... [6,6]
	arm_matrix_instance_f32 Sinv;		//Matriz F inversa.
	arm_matrix_instance_f32 K;			//Matriz com os ganhos de Kalman [12,6]

		//Matrices intermediàrias para cálculo

	arm_matrix_instance_f32 temp_calc_1210;
	arm_matrix_instance_f32 temp_calc_1211;
	
	arm_matrix_instance_f32 temp_calc_12120;
	arm_matrix_instance_f32 temp_calc_12121;
	arm_matrix_instance_f32 temp_calc_12122;
	arm_matrix_instance_f32 temp_calc_12123;

	arm_matrix_instance_f32 temp_calc_610;

	arm_matrix_instance_f32 temp_calc_660;

	arm_matrix_instance_f32 temp_calc_1260;
	//Variáveis para cálculos
	
	float acel_x = buffer_filtro->ultimo_estado[0]; 
	float acel_y = buffer_filtro->ultimo_estado[1];
	float acel_z = buffer_filtro->ultimo_estado[2];

	float mag_x = buffer_filtro->ultimo_estado[3];
	float mag_y = buffer_filtro->ultimo_estado[4];
	float mag_z = buffer_filtro->ultimo_estado[5];

	float bias_mag_x = buffer_filtro->ultimo_estado[9];
	float bias_mag_y = buffer_filtro->ultimo_estado[10];
	float bias_mag_z = buffer_filtro->ultimo_estado[11];

	float wx = (medida_gyro[0] - buffer_filtro->ultimo_estado[6]); //Atualiza as variáveis de deslocamento angular. - medida atual menos bias obtido no estado anterior.
	float wy = (medida_gyro[1] - buffer_filtro->ultimo_estado[7]);
	float wz = (medida_gyro[2] - buffer_filtro->ultimo_estado[8]);

	//Matriz identidade utilizada para facilitar atualização das matrizes.
	float I_f32[144] = {1,0,0,0,0,0,0,0,0,0,0,0,
					    0,1,0,0,0,0,0,0,0,0,0,0,
					    0,0,1,0,0,0,0,0,0,0,0,0,
					    0,0,0,1,0,0,0,0,0,0,0,0,
					    0,0,0,0,1,0,0,0,0,0,0,0,
					    0,0,0,0,0,1,0,0,0,0,0,0,
					    0,0,0,0,0,0,1,0,0,0,0,0,
					    0,0,0,0,0,0,0,1,0,0,0,0,
					    0,0,0,0,0,0,0,0,1,0,0,0,
					    0,0,0,0,0,0,0,0,0,1,0,0,
					    0,0,0,0,0,0,0,0,0,0,1,0,
					    0,0,0,0,0,0,0,0,0,0,0,1};

	arm_mat_init_f32(&I, 12, 12, I_f32);

	//Buffers que contém os valores em cada uma das matrizes.

	float X_f32[12];
	arm_copy_f32(buffer_filtro->ultimo_estado, X_f32, 12);
	arm_mat_init_f32(&X, 12, 1, X_f32);

	float P_f32[144];
	arm_copy_f32(buffer_filtro->P, P_f32, 144);
	arm_mat_init_f32(&P, 12, 12, P_f32);

	float Pdot_f32[144];
	arm_mat_init_f32(&Pdot, 12, 12, Pdot_f32);

	//Matriz de atualização dos estados
	float FJ_f32[144] = {0,			(wz), 		-(wy), 		0,		0, 		0, 		0, 		 acel_z,	-acel_y,	0,		0,		0,
					   	-(wz),		0, 			(wx), 		0, 		0, 		0, 		-acel_z, 0, 		acel_x,		0,		0,		0,
					   	(wy), 		-(wx), 		0, 			0, 		0, 		0, 		acel_y,  -acel_x,	0,			0,		0,		0,
					   	0,			0,			0,			0, 		(wz),	-(wy), 	0, 		 mag_z,  	-mag_y,		0,		-(wz),	(wy),
					   	0, 			0, 			0,			-(wz), 	0, 		(wx),	-mag_z,  0,			mag_x,		(wz),	0,		-(wx),
					   	0, 			0, 			0,			(wy),	-(wx), 	0, 		mag_y, 	 -mag_x,  	0,			-(wy),	(wx),	0,
					   	0, 			0, 			0,			0, 		0, 		0, 		0, 		 0, 	  	0,			0,		0,		0,	
					   	0, 			0, 			0, 			0, 		0, 		0, 		0, 		 0, 		0,			0,		0,		0,
					   	0, 			0, 			0, 			0, 		0, 		0, 		0, 		 0, 		0,			0,		0,		0,
					    0, 			0, 			0,			0, 		0, 		0, 		0, 		 0, 	  	0,			0,		0,		0,	
					   	0, 			0, 			0, 			0, 		0, 		0, 		0, 		 0, 		0,			0,		0,		0,
					   	0, 			0, 			0, 			0, 		0, 		0, 		0, 		 0, 		0,			0,		0,		0};
	
	arm_mat_init_f32(&FJ, 12, 12, FJ_f32);

	//Matriz de atualização dos estados transposta
	float FJt_f32[144]= {0,			-(wz), 		(wy), 		0,		0, 		0, 		0, 		0, 			0,			0,		0,		0,
					   	(wz),		0, 			-(wx), 		0, 		0, 		0, 		0, 		0, 			0,			0,		0,		0,
					   	-(wy), 		(wx), 		0, 			0, 		0, 		0, 		0, 		0, 			0,			0,		0,		0,	
					   	0,			0,			0,			0, 		-(wz),	(wy), 	0, 		0, 			0,			0,		0,		0,
					   	0, 			0, 			0,			(wz), 	0, 		-(wx),	0, 		0,			0,			0,		0,		0,
					   	0, 			0, 			0,			-(wy),	(wx), 	0, 		0, 		0, 			0,			0,		0,		0,
					   	0, 			-acel_z,	acel_y, 	0, 		-mag_z,	mag_y, 	0, 		0, 			0,			0,		0,		0,
					   	acel_z,		0, 			-acel_x,	mag_z,  0, 		-mag_x, 0, 		0, 			0,			0,		0,		0,
					   	-acel_y,	acel_x,		0, 			-mag_y,	mag_x,	0, 		0, 		0, 			0,			0,		0,		0,
					   	0,			0,			0,			0,		(wz),	-(wy),	0,		0,			0,			0,		0,		0,
					   	0,			0,			0,			-(wz),	0,		(wx),	0,		0,			0,			0,		0,		0,
					   	0,			0,			0,			(wy),	-(wx),	0,		0,		0,			0,			0,		0,		0};

	arm_mat_init_f32(&FJt, 12, 12, FJt_f32);

	//Matriz de covariâncias
	float Qacel = (buffer_filtro->Q_acel);
	float Qmag = (buffer_filtro->Q_mag);
	float Qbias = (buffer_filtro->Q_bias);
	float Qbias_mag = (buffer_filtro->Qbias_mag);

	float Q_f32[144] = {(Qacel), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 					0, (Qacel), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 					0, 0, (Qacel), 0, 0, 0, 0, 0, 0, 0, 0, 0,
	 					0, 0, 0, (Qmag), 0, 0, 0, 0, 0, 0, 0, 0,
	 					0, 0, 0, 0, (Qmag), 0, 0, 0, 0, 0, 0, 0,
	 					0, 0, 0, 0, 0, (Qmag), 0, 0, 0, 0, 0, 0,
	 					0, 0, 0, 0, 0, 0, (Qbias), 0, 0, 0, 0, 0,
	 					0, 0, 0, 0, 0, 0, 0, (Qbias), 0, 0, 0, 0,
	 					0, 0, 0, 0, 0, 0, 0, 0, (Qbias), 0, 0, 0,
	 					0, 0, 0, 0,	0, 0, 0, 0, 0, (Qbias_mag), 0, 0,
	 					0, 0, 0, 0,	0, 0, 0, 0, 0, 0, (Qbias_mag), 0,
	 					0, 0, 0, 0,	0, 0, 0, 0, 0, 0, 0, (Qbias_mag)};

	arm_mat_init_f32(&Q, 12, 12, Q_f32);


	//Matriz de variâncias
	float Racel = buffer_filtro->R_acel;
	float Rmag = buffer_filtro->R_mag;
	
	float R_f32[36] = {(Racel), 0, 0, 0, 0, 0,
					   0, (Racel), 0, 0, 0, 0,
					   0, 0, (Racel), 0, 0, 0,
					   0, 0, 0, (Rmag), 0, 0,
					   0, 0, 0, 0, (Rmag), 0,
					   0, 0, 0, 0, 0, (Rmag)};

	arm_mat_init_f32(&R, 6, 6, R_f32);

	//Matriz de erro entre medida e estado estimado
	float y_f32[6];
	arm_mat_init_f32(&y, 6, 1, y_f32);

	//Matriz de mapeamento dos estados - H
	float H_f32[72] = {1,0,0,0,0,0,0,0,0,0,0,0,
					   0,1,0,0,0,0,0,0,0,0,0,0,
					   0,0,1,0,0,0,0,0,0,0,0,0,
					   0,0,0,1,0,0,0,0,0,0,0,0,
					   0,0,0,0,1,0,0,0,0,0,0,0,
					   0,0,0,0,0,1,0,0,0,0,0,0};

	arm_mat_init_f32(&H, 6, 12, H_f32);

	//Matriz de mapeamento dos estados transposta - Ht
	float Ht_f32[72] = {1,0,0,0,0,0,
						0,1,0,0,0,0,
						0,0,1,0,0,0,
						0,0,0,1,0,0,
						0,0,0,0,1,0,
						0,0,0,0,0,1,
						0,0,0,0,0,0,
						0,0,0,0,0,0,
						0,0,0,0,0,0,
						0,0,0,0,0,0,
						0,0,0,0,0,0,
						0,0,0,0,0,0}; 
						
	arm_mat_init_f32(&Ht, 12, 6, Ht_f32);

	//Matriz S...
	float S_f32[36];
	arm_mat_init_f32(&S, 6, 6, S_f32);

	float Sinv_f32[36];
	arm_mat_init_f32(&Sinv, 6, 6, Sinv_f32);

	//Matriz do ganho de Kalman 
	float K_f32[72];
	arm_mat_init_f32(&K, 12, 6, K_f32);

	//Matrizes de suporte para o cálculo
		//Matrizes de 12 linhas e 1 coluna
	float temp_calc_1210_f32[12];
	float temp_calc_1211_f32[12];
	
	arm_mat_init_f32(&temp_calc_1210, 12, 1, temp_calc_1210_f32);
	arm_mat_init_f32(&temp_calc_1211, 12, 1, temp_calc_1211_f32);

	//Matrizes de 12 linhas e 12 colunas
	float temp_calc_12120_f32[144];
	float temp_calc_12121_f32[144];
	float temp_calc_12122_f32[144];
	float temp_calc_12123_f32[144];

	arm_mat_init_f32(&temp_calc_12120, 12, 12, temp_calc_12120_f32);
	arm_mat_init_f32(&temp_calc_12121, 12, 12, temp_calc_12121_f32);
	arm_mat_init_f32(&temp_calc_12122, 12, 12, temp_calc_12122_f32);
	arm_mat_init_f32(&temp_calc_12123, 12, 12, temp_calc_12123_f32);


	//Matrizes de 6linhas e 1 coluna
	float temp_calc_610_f32[6];
	arm_mat_init_f32(&temp_calc_610, 6, 1, temp_calc_610_f32);

	//Matrizes de 6 linhas e 6 colunas
	float temp_calc_660_f32[36];
	arm_mat_init_f32(&temp_calc_660, 6, 6, temp_calc_660_f32);

	//Matrizes de calculos de 12 linhas e 6 colunas
	float temp_calc_1260_f32[72];
	arm_mat_init_f32(&temp_calc_1260, 12, 6, temp_calc_1260_f32);


	//Cálculos do filtro de Kalman

	//Fase de update -> Estimativa do novo estado com base no estado anterior

	float acel_x_dot =  0 + acel_y*wz - acel_z*wy;
	float acel_y_dot = -acel_x*wz + 0 + acel_z*wx;
	float acel_z_dot =  acel_x*wy - acel_y*wx + 0;

	float mag_x_dot =  0 + (mag_y-bias_mag_y)*wz - (mag_z-bias_mag_z)*wy;
	float mag_y_dot = -(mag_x-bias_mag_x)*wz + 0 + (mag_z-bias_mag_z)*wx;
	float mag_z_dot =  (mag_x-bias_mag_x)*wy - (mag_y-bias_mag_z)*wx + 0;

	X_f32[0] = acel_x + acel_x_dot*buffer_filtro->dt;
	X_f32[1] = acel_y + acel_y_dot*buffer_filtro->dt;
	X_f32[2] = acel_z + acel_z_dot*buffer_filtro->dt;


	//mag_x = buffer_filtro->ultimo_estado[3]-buffer_filtro->ultimo_estado[9];
	//mag_y = buffer_filtro->ultimo_estado[4]-buffer_filtro->ultimo_estado[10];
	//mag_z = buffer_filtro->ultimo_estado[5]-buffer_filtro->ultimo_estado[11];

	X_f32[3] = (mag_x) + mag_x_dot*buffer_filtro->dt;
	X_f32[4] = (mag_y) + mag_y_dot*buffer_filtro->dt;
	X_f32[5] = (mag_z) + mag_z_dot*buffer_filtro->dt;

	X_f32[6] = X_f32[6];
	X_f32[7] = X_f32[7];
	X_f32[8] = X_f32[8];

	X_f32[9] = X_f32[9];
	X_f32[10] = X_f32[10];
	X_f32[11] = X_f32[11];

	//temp_calc_12120 = FJ*P
	arm_mat_mult_f32(&FJ, &P, &temp_calc_12120);

	//temp_calc_12121 = P*FJ'
	arm_mat_mult_f32(&P, &FJt, &temp_calc_12121);

	//Pdot = temp_calc_12120 + temp_calc_12121
	arm_mat_add_f32(&temp_calc_12120, &temp_calc_12121, &Pdot);

	//tempo_calc_12120 = Pdot + Q;
	arm_mat_add_f32(&Pdot, &Q, &temp_calc_12120);

	//Pdot*dt;
	arm_mat_scale_f32(&temp_calc_12120, buffer_filtro->dt, &Pdot);

	//temp_calc_12121 = P + Pdot*dt
	arm_mat_add_f32(&P, &Pdot, &temp_calc_12121);

	//Faz P apontar para  a matriz 12121.
	arm_copy_f32(temp_calc_12121_f32, P_f32, 144);

	//y = z - H*X = z - H*X
	y_f32[0] = medida_accel[0] - X_f32[0];
	y_f32[1] = medida_accel[1] - X_f32[1];
	y_f32[2] = medida_accel[2] - X_f32[2];

	y_f32[3] = medida_mag[0] - X_f32[3];
	y_f32[4] = medida_mag[1] - X_f32[4];
	y_f32[5] = medida_mag[2] - X_f32[5];

	//S = H*P*H' + R
	temp_calc_660_f32[0] = P_f32[0];
	temp_calc_660_f32[1] = P_f32[1];
	temp_calc_660_f32[2] = P_f32[2];
	temp_calc_660_f32[3] = P_f32[3];
	temp_calc_660_f32[4] = P_f32[4];
	temp_calc_660_f32[5]=  P_f32[5];
	temp_calc_660_f32[6] = P_f32[12];
	temp_calc_660_f32[7] = P_f32[13];
	temp_calc_660_f32[8] = P_f32[14];
	temp_calc_660_f32[9] = P_f32[15];
	temp_calc_660_f32[10] = P_f32[16];
	temp_calc_660_f32[11] = P_f32[17];
	temp_calc_660_f32[12] = P_f32[24];
	temp_calc_660_f32[13] = P_f32[25];
	temp_calc_660_f32[14] = P_f32[26];
	temp_calc_660_f32[15] = P_f32[27];
	temp_calc_660_f32[16] = P_f32[28];
	temp_calc_660_f32[17] = P_f32[29];
	temp_calc_660_f32[18] = P_f32[36];
	temp_calc_660_f32[19] = P_f32[37];
	temp_calc_660_f32[20] = P_f32[38];
	temp_calc_660_f32[21] = P_f32[39];
	temp_calc_660_f32[22] = P_f32[40];
	temp_calc_660_f32[23] = P_f32[41];
	temp_calc_660_f32[24] = P_f32[48];
	temp_calc_660_f32[25] = P_f32[49];
	temp_calc_660_f32[26] = P_f32[50];
	temp_calc_660_f32[27] = P_f32[51];
	temp_calc_660_f32[28] = P_f32[52];
	temp_calc_660_f32[29] = P_f32[53];
	temp_calc_660_f32[30] = P_f32[60];
	temp_calc_660_f32[31] = P_f32[61];
	temp_calc_660_f32[32] = P_f32[62];
	temp_calc_660_f32[33] = P_f32[63];
	temp_calc_660_f32[34] = P_f32[64];
	temp_calc_660_f32[35] = P_f32[65];

	arm_mat_add_f32(&temp_calc_660, &R, &S);

	//Sinv = inv(S);
	arm_mat_inverse_f32(&S, &Sinv);

	//Kk = P*Ht*S^(-1)
		//P*Ht
	temp_calc_1260_f32[0] = P_f32[0];
	temp_calc_1260_f32[1] = P_f32[1];
	temp_calc_1260_f32[2] = P_f32[2];
	temp_calc_1260_f32[3] = P_f32[3];
	temp_calc_1260_f32[4] = P_f32[4];
	temp_calc_1260_f32[5] = P_f32[5];
	temp_calc_1260_f32[6] = P_f32[12];
	temp_calc_1260_f32[7] = P_f32[13];
	temp_calc_1260_f32[8] = P_f32[14];
	temp_calc_1260_f32[9] = P_f32[15];
	temp_calc_1260_f32[10] = P_f32[16];
	temp_calc_1260_f32[11] = P_f32[17];
	temp_calc_1260_f32[12] = P_f32[24];
	temp_calc_1260_f32[13] = P_f32[25];
	temp_calc_1260_f32[14] = P_f32[26];
	temp_calc_1260_f32[15] = P_f32[27];
	temp_calc_1260_f32[16] = P_f32[28];
	temp_calc_1260_f32[17] = P_f32[29];
	temp_calc_1260_f32[18] = P_f32[36];
	temp_calc_1260_f32[19] = P_f32[37];
	temp_calc_1260_f32[20] = P_f32[38];
	temp_calc_1260_f32[21] = P_f32[39];
	temp_calc_1260_f32[22] = P_f32[40];
	temp_calc_1260_f32[23] = P_f32[41];
	temp_calc_1260_f32[24] = P_f32[48];
	temp_calc_1260_f32[25] = P_f32[49];
	temp_calc_1260_f32[26] = P_f32[50];
	temp_calc_1260_f32[27] = P_f32[51];
	temp_calc_1260_f32[28] = P_f32[52];
	temp_calc_1260_f32[29] = P_f32[53];
	temp_calc_1260_f32[30] = P_f32[60];
	temp_calc_1260_f32[31] = P_f32[61];
	temp_calc_1260_f32[32] = P_f32[62];
	temp_calc_1260_f32[33] = P_f32[63];
	temp_calc_1260_f32[34] = P_f32[64];
	temp_calc_1260_f32[35] = P_f32[65];
	temp_calc_1260_f32[36] = P_f32[72];
	temp_calc_1260_f32[37] = P_f32[73];
	temp_calc_1260_f32[38] = P_f32[74];
	temp_calc_1260_f32[39] = P_f32[75];
	temp_calc_1260_f32[40] = P_f32[76];
	temp_calc_1260_f32[41] = P_f32[77];
	temp_calc_1260_f32[42] = P_f32[84];
	temp_calc_1260_f32[43] = P_f32[85];
	temp_calc_1260_f32[44] = P_f32[86];
	temp_calc_1260_f32[45] = P_f32[87];
	temp_calc_1260_f32[46] = P_f32[88];
	temp_calc_1260_f32[47] = P_f32[89];
	temp_calc_1260_f32[48] = P_f32[96];
	temp_calc_1260_f32[49] = P_f32[97];
	temp_calc_1260_f32[50] = P_f32[98];
	temp_calc_1260_f32[51] = P_f32[99];
	temp_calc_1260_f32[52] = P_f32[100];
	temp_calc_1260_f32[53] = P_f32[101];
	temp_calc_1260_f32[54] = P_f32[108];
	temp_calc_1260_f32[55] = P_f32[109];
	temp_calc_1260_f32[56] = P_f32[110];
	temp_calc_1260_f32[57] = P_f32[111];
	temp_calc_1260_f32[58] = P_f32[112];
	temp_calc_1260_f32[59] = P_f32[113];
	temp_calc_1260_f32[60] = P_f32[120];
	temp_calc_1260_f32[61] = P_f32[121];
	temp_calc_1260_f32[62] = P_f32[122];
	temp_calc_1260_f32[63] = P_f32[123];
	temp_calc_1260_f32[64] = P_f32[124];
	temp_calc_1260_f32[65] = P_f32[125];
	temp_calc_1260_f32[66] = P_f32[132];
	temp_calc_1260_f32[67] = P_f32[133];
	temp_calc_1260_f32[68] = P_f32[134];
	temp_calc_1260_f32[69] = P_f32[135];
	temp_calc_1260_f32[70] = P_f32[136];
	temp_calc_1260_f32[71] = P_f32[137];

	arm_mat_mult_f32(&temp_calc_1260, &Sinv, &K);

	//temp_calc_1211 = Kk*y
	arm_mat_mult_f32(&K, &y, &temp_calc_1211);

	//X = X + temp_calc_1211;
	arm_mat_add_f32(&X, &temp_calc_1211, &temp_calc_1210);
	arm_copy_f32(temp_calc_1210_f32, X_f32, 12);

	//P = (I-K*H)*P
	arm_mat_mult_f32(&K, &H, &temp_calc_12122);
	arm_mat_sub_f32(&I, &temp_calc_12122, &temp_calc_12123);

	arm_mat_mult_f32(&temp_calc_12123, &P, &temp_calc_12122);

	arm_copy_f32(X_f32, buffer_filtro->ultimo_estado, 12);
	arm_copy_f32(temp_calc_12122_f32, buffer_filtro->P, 144);

}
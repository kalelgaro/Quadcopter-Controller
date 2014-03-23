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
#include "tratamento_sinal.h"

//------ Função para cálculo do filtro de kalman sobre o Giroscópio e o Acelerômetro -------///
//Utiliza DCM -> Variação do vetor com base na velocidae agular para atualização da matrz de atualização de estados.
//Bruno f. M. Callegaro - 20/12/2013


void kalman_filter(kalman_filter_state *buffer_filtro, float medida_gyro[], float medida_accel[], float medida_mag[])
{
	//Instancias das matrizes utilizadas para o cálculo
	arm_matrix_instance_f32 X;			//Matriz de estados. [12,1]
	arm_matrix_instance_f32 F;			//Matriz de transição de estados. [12,12]
	arm_matrix_instance_f32 Ft;			//Matriz de transição de estados transposta. [12,12]
	arm_matrix_instance_f32 I;			//Matriz identidadee. [12,12]
	arm_matrix_instance_f32 P;			//Matriz de confiabilidade do processo de atualização. [12,12]
	arm_matrix_instance_f32 H;			//Matriz de mapeamento do estado para o erro [6,12]
	arm_matrix_instance_f32 Ht;			//Matriz de mapeamento do estado para o erro transposta. [12,6]
	arm_matrix_instance_f32 Q;			//Matriz de covariância multiplicada por dt; [12,12]
	arm_matrix_instance_f32 R;			//Matriz de variância [6,6]
	arm_matrix_instance_f32 y;			//Matriz de erro entre medidas e estado estimado. [6,1]
	arm_matrix_instance_f32 z;			
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
	arm_matrix_instance_f32 temp_calc_611;

	arm_matrix_instance_f32 temp_calc_660;
	arm_matrix_instance_f32 temp_calc_661;
	arm_matrix_instance_f32 temp_calc_662;

	arm_matrix_instance_f32 temp_calc_6120;

	arm_matrix_instance_f32 temp_calc_1260;
	arm_matrix_instance_f32 temp_calc_1261;
	//Variáveis para cálculos
	
	float dt = buffer_filtro->dt;

	float acel_x = buffer_filtro->ultimo_estado[0]*dt; 
	float acel_y = buffer_filtro->ultimo_estado[1]*dt;
	float acel_z = buffer_filtro->ultimo_estado[2]*dt;

	float mag_x = buffer_filtro->ultimo_estado[3]*dt;
	float mag_y = buffer_filtro->ultimo_estado[4]*dt;
	float mag_z = buffer_filtro->ultimo_estado[5]*dt;

	// float bgx = buffer_filtro->ultimo_estado[6];
	// float bgy = buffer_filtro->ultimo_estado[7];
	// float bgz = buffer_filtro->ultimo_estado[8];

	float bias_mag_x = buffer_filtro->ultimo_estado[9];
	float bias_mag_y = buffer_filtro->ultimo_estado[10];
	float bias_mag_z = buffer_filtro->ultimo_estado[11];

	//mag_x = mag_x - bias_mag_x;
	//mag_y = mag_y - bias_mag_y;
	//mag_z = mag_z - bias_mag_z;

	float wx = (medida_gyro[0])*dt; //Atualiza as variáveis de deslocamento angular. - medida atual menos bias obtido no estado anterior.
	float wy = (medida_gyro[1])*dt;
	float wz = (medida_gyro[2])*dt;
	

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

	//Matriz de atualização dos estados.
	float F_f32[144] = {1,		wz,		-wy,	0,		0,		0,		0,			acel_z,		-acel_y,	0,		0,		0,
						-wz,	1,		wx,		0,		0,		0,		-acel_z,	0,			acel_x,		0,		0,		0,
						wy,		-wx,	1,		0,		0,		0,		acel_y,		-acel_x,	0,			0,		0,		0,
						0,		0,		0,		1,		wz,		-wy,	0,			mag_z,		-mag_y,		0,		0,		0,
						0,		0,		0,		-wz,	1,		wx,		-mag_z,		0,			mag_x,		0,		0,		0,
						0,		0,		0,		wy,		-wx,	1,		mag_y,		-mag_x,		0,			0,		0,		0,
						0,		0,		0,		0,		0,		0,		1,			0,			0,			0,		0,		0,
						0,		0,		0,		0,		0,		0,		0,			1,			0,			0,		0,		0,
						0,		0,		0,		0,		0,		0,		0,			0,			1,			0,		0,		0,
						0,		0,		0,		0,		0,		0,		0,			0,			0,			1,		wz,		-wy,
						0,		0,		0,		0,		0,		0,		0,			0,			0,			-wz,	1,		wx,
						0,		0,		0,		0,		0,		0,		0,			0,			0,			wy,		-wx,	1};


	arm_mat_init_f32(&F, 12, 12, F_f32);

	//Matriz de atualização de estados tranposta.
	float Ft_f32[144] ={1,			-wz,		wy,			0,		0,		0,		0,		0,		0,		0,		0,		0,
						wz,			1,			-wx,		0,		0,		0,		0,		0,		0,		0,		0,		0,
						-wy,		wx,			1,			0,		0,		0,		0,		0,		0,		0,		0,		0,
						0,			0,			0,			1,		-wz,	wy,		0,		0,		0,		0,		0,		0,
						0,			0,			0,			wz,		1,		-wx,	0,		0,		0,		0,		0,		0,
						0,			0,			0,			-wy,	wx,		1,		0,		0,		0,		0,		0,		0,
						0,			-acel_z,	acel_y,		0,		-mag_z,	mag_y,	1,		0,		0,		0,		0,		0,
						acel_z,		0,			-acel_x,	mag_z,	0,		-mag_x,	0,		1,		0,		0,		0,		0,
						-acel_y,	acel_x,		0,			-mag_y,	mag_x,	0,		0,		0,		1,		0,		0,		0,
						0,			0,			0,			0,		0,		0,		0,		0,		0,		1,		-wz,	wy,
						0,			0,			0,			0,		0,		0,		0,		0,		0,		wz,		1,		-wx,
						0,			0,			0,			0,		0,		0,		0,		0,		0,		-wy,	wx,		1};


	arm_mat_init_f32(&Ft, 12, 12, Ft_f32);

	float X_f32[12];
	arm_copy_f32(buffer_filtro->ultimo_estado, X_f32, 12);
	arm_mat_init_f32(&X, 12, 1, X_f32);

	float P_f32[144];
	arm_copy_f32(buffer_filtro->P, P_f32, 144);
	arm_mat_init_f32(&P, 12, 12, P_f32);

	//Matriz de covariâncias
	float Qacel = (buffer_filtro->Q_acel)*(buffer_filtro->dt);
	float Qmag = (buffer_filtro->Q_mag)*(buffer_filtro->dt);
	float Qbias = (buffer_filtro->Q_bias)*(buffer_filtro->dt);
	float Qbias_mag = (buffer_filtro->Qbias_mag)*(buffer_filtro->dt);

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

	float temp_calc_611_f32[6];
	arm_mat_init_f32(&temp_calc_611, 6, 1, temp_calc_611_f32);

	//Matrizes de 6 linhas e 6 colunas
	float temp_calc_660_f32[36];
	arm_mat_init_f32(&temp_calc_660, 6, 6, temp_calc_660_f32);

	float temp_calc_661_f32[36];
	arm_mat_init_f32(&temp_calc_661, 6, 6, temp_calc_661_f32);

	float temp_calc_662_f32[36];
	arm_mat_init_f32(&temp_calc_662, 6, 6, temp_calc_662_f32);

	//Matrizes de calculos de 12 linhas e 6 colunas
	float temp_calc_1260_f32[72];
	arm_mat_init_f32(&temp_calc_1260, 12, 6, temp_calc_1260_f32);

	float temp_calc_1261_f32[72];
	arm_mat_init_f32(&temp_calc_1261, 12, 6, temp_calc_1261_f32);


	float temp_calc_6120_f32[72];
	arm_mat_init_f32(&temp_calc_6120, 6, 12, temp_calc_6120_f32);


	float z_f32[6];
	arm_mat_init_f32(&z, 6, 1, z_f32);

	arm_copy_f32(medida_accel, z_f32, 3);
	arm_copy_f32(medida_mag, z_f32+3, 3);
	//Cálculos do filtro de Kalman

	//Fase de update -> Estimativa do novo estado com base no estado anterior
	arm_mat_mult_f32(&F, &X, &temp_calc_1210);

	arm_copy_f32(temp_calc_1210_f32, X_f32, 12);


	//temp_calc_12120 = F*P
	arm_mat_mult_f32(&F, &P, &temp_calc_12120);

	//temp_calc_12121 = F*P*F'
	arm_mat_mult_f32(&temp_calc_12120, &Ft, &temp_calc_12121);

	//P = temp_calc_12121 + Q = F*P*F' + Q
	arm_mat_add_f32(&temp_calc_12121, &Q, &P);

	//y = z - H*X = z - H*X
	arm_mat_mult_f32(&H,&X,&temp_calc_610);
	arm_mat_sub_f32(&z, &temp_calc_610, &y);

	//S = H*P*H' + R
	arm_mat_mult_f32(&H, &P, &temp_calc_6120);
	arm_mat_mult_f32(&temp_calc_6120, &Ht, &temp_calc_660);
	arm_mat_add_f32(&temp_calc_660, &R, &S);

	//Sinv = inv(S);
	arm_mat_inverse_f32(&S, &Sinv);

	//Kk = P*Ht*S^(-1)
		//P*Ht
	arm_mat_mult_f32(&P, &Ht, &temp_calc_1260);

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

	normalizar_vetor_R3(X_f32);
	normalizar_vetor_R3(X_f32+3);

	arm_copy_f32(X_f32, buffer_filtro->ultimo_estado, 12);
	arm_copy_f32(temp_calc_12122_f32, buffer_filtro->P, 144);

}


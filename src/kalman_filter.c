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


float kalman_filter(kalman_filter_state *buffer_filtro, float medida_gyro[], float medida_accel[])
{
	//Instancias das matrizes utilizadas para o cálculo
	arm_matrix_instance_f32 X;			//Matriz de estados. [3,1]
	arm_matrix_instance_f32 F;			//Matriz de transição de estados. [3,3]
	arm_matrix_instance_f32 Ft;			//Matriz de transição de estados transposta. [3,3]
	arm_matrix_instance_f32 I;			//Matriz identidadee. [3,3]
	arm_matrix_instance_f32 P;			//Matriz de confiabilidade do processo de atualização. [3,3]
	arm_matrix_instance_f32 Qrdt;		//Matriz de covariância multiplicada por dt; [3,3]
	arm_matrix_instance_f32 R;			//Matriz de variância [3,3]
	arm_matrix_instance_f32 z;
	arm_matrix_instance_f32 y;			//Matriz de erro entre medidas e estado estimado. [3,1]
	arm_matrix_instance_f32 S;			//Matriz .... [3,3]
	arm_matrix_instance_f32 K;			//Matriz com os ganhos de Kalman [3,3]

		//Matrices intermediàrias para cálculo

	arm_matrix_instance_f32 temp_calc_310;
	arm_matrix_instance_f32 temp_calc_311;
	arm_matrix_instance_f32 temp_calc_312;

	arm_matrix_instance_f32 temp_calc_330;
	arm_matrix_instance_f32 temp_calc_331;
	arm_matrix_instance_f32 temp_calc_332;
	arm_matrix_instance_f32 temp_calc_333;
	arm_matrix_instance_f32 temp_calc_334;

	//Variáveis para cálculos
	float theta_x = medida_gyro[0]*buffer_filtro->dt; //Atualiza as variáveis de deslocamento angular.
	float theta_y = medida_gyro[1]*buffer_filtro->dt;
	float theta_z = medida_gyro[2]*buffer_filtro->dt;


	//Matriz identidade utilizada para facilitar atualização das matrizes.
	float I_f32[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	arm_mat_init_f32(&I, 3, 3, I_f32);


	//Buffers que contém os valores em cada uma das matrizes.

	float X_f32[3];
	arm_copy_f32(buffer_filtro->ultimo_estado, X_f32, 3);
	arm_mat_init_f32(&X, 3, 1, X_f32);

	float P_f32[9];
	arm_copy_f32(buffer_filtro->P, P_f32, 9);
	arm_mat_init_f32(&P, 3, 3, P_f32);


	//Matriz de atualização dos estados
	float F_f32[9] = {1, (theta_z), (-theta_y), (-theta_z), 1, (theta_x), (theta_y), (-theta_x), 1};
	arm_mat_init_f32(&F, 3, 3, F_f32);


	//Matriz de atualização dos estados transposta
	float Ft_f32[9];
	arm_mat_init_f32(&Ft, 3, 3, Ft_f32);
	arm_mat_trans_f32(&F, &Ft);


	//Matriz de covariâncias
	float Qr_dt_buffer = (buffer_filtro->Q)*(buffer_filtro->dt);
	float Qrdt_f32[9] = {(Qr_dt_buffer), 0, 0, 0, (Qr_dt_buffer), 0, 0, 0, (Qr_dt_buffer)};
	arm_mat_init_f32(&Qrdt, 3, 3, Qrdt_f32);


	//Matriz de variâncias
	float R_buffer = buffer_filtro->R;
	float R_f32[9] = {R_buffer, 0, 0, 0, R_buffer, 0, 0, 0, R_buffer};
	arm_mat_init_f32(&R, 3, 3, R_f32);


	//Matriz de erro entre medida e estado estimado
	float y_f32[3];
	arm_mat_init_f32(&y, 3, 1, y_f32);


	//Matriz que contém as medidas do acelerômetro.
	float z_f32[3] = {medida_accel[0], medida_accel[1], medida_accel[2]};
	arm_mat_init_f32(&z, 3, 1, z_f32);


	//Matriz S...
	float S_f32[9];
	arm_mat_init_f32(&S, 3, 3, S_f32);

	//Matriz do ganho de Kalman 
	float K_f32[9];
	arm_mat_init_f32(&K, 3, 3, K_f32);

	//Matrizes de suporte para o cálculo
		//Matrizes de 3 linhas e 1 coluna
	float temp_calc_310_f32[3];
	float temp_calc_311_f32[3];
	float temp_calc_312_f32[3];

	arm_mat_init_f32(&temp_calc_310, 3, 1, temp_calc_310_f32);
	arm_mat_init_f32(&temp_calc_311, 3, 1, temp_calc_311_f32);
	arm_mat_init_f32(&temp_calc_312, 3, 1, temp_calc_312_f32);

		//Matrizes de 3 linhas e 3 colunas
	float temp_calc_330_f32[9];
	float temp_calc_331_f32[9];
	float temp_calc_332_f32[9];
	float temp_calc_333_f32[9];
	float temp_calc_334_f32[9];

	arm_mat_init_f32(&temp_calc_330, 3, 3, temp_calc_330_f32);
	arm_mat_init_f32(&temp_calc_331, 3, 3, temp_calc_331_f32);
	arm_mat_init_f32(&temp_calc_332, 3, 3, temp_calc_332_f32);
	arm_mat_init_f32(&temp_calc_333, 3, 3, temp_calc_333_f32);
	arm_mat_init_f32(&temp_calc_334, 3, 3, temp_calc_334_f32);

	//Cálculos do filtro de Kalman
	//Etapas de predição - "A Priori"
	//X = F*X
	arm_mat_mult_f32(&F, &X, &temp_calc_310);
	arm_mat_mult_f32(&I, &temp_calc_310, &X);

	//P = F*P*F'
	arm_mat_mult_f32(&F, &P, &temp_calc_330);
	arm_mat_mult_f32(&temp_calc_330, &Ft, &temp_calc_331);

	//P = P+Qrdt
	arm_mat_add_f32(&temp_calc_331, &Qrdt, &P);

	//y = z - H*X = z - X
	arm_mat_sub_f32(&z, &X, &y);

	//S = H*P*H' + R = P + R
	arm_mat_add_f32(&P, &R, &S);

	//temp_calc_332 = inv(S);
	arm_mat_inverse_f32(&S, &temp_calc_332);

	//Kk = P*S^(-1) = P*temp_calc)332
	arm_mat_mult_f32(&P, &temp_calc_332, &K);

	//temp_calc_311 = Kk*y
	arm_mat_mult_f32(&K, &y, &temp_calc_311);

	//temp_calc_312 = X + temp_calc_311 = X + Kk*Y
	arm_mat_add_f32(&X, &temp_calc_311, &temp_calc_312);

	arm_mat_mult_f32(&I, &temp_calc_312, &X);

	arm_mat_sub_f32(&I, &K, &temp_calc_333);

	arm_mat_mult_f32(&temp_calc_333, &P, &temp_calc_334);

	arm_mat_mult_f32(&I, &temp_calc_334, &P);

	arm_copy_f32(X_f32, buffer_filtro->ultimo_estado, 3);
	arm_copy_f32(P_f32, buffer_filtro->P, 9);
	return 0;

}



/*
 * kalman_filter.c
 *
 *  Created on: Sep 8, 2013
 *      Author: Bruno
 */
#include "stm32f4_discovery.h"
 
#include "kalman_filter.h"
#include "arm_math.h"
#include "tratamento_sinal.h"

//------ Função para cálculo do filtro de kalman sobre o Giroscópio e o Acelerômetro -------///
//Utiliza DCM -> Variação do vetor com base na velocidae agular para atualização da matrz de atualização de estados.
//Bruno f. M. Callegaro - 20/12/2013


void kalman_filter(kalman_filter_state *buffer_filtro, float medida_gyro[], float medida_accel[], float medida_mag[], uint16_t estado_motores)
{
	//Instancias das matrizes utilizadas para o cálculo
	arm_matrix_instance_f32 X;			//Matriz de estados. [9,1]
	arm_matrix_instance_f32 F;			//Matriz de transição de estados. [9,9]
	arm_matrix_instance_f32 Ft;			//Matriz de transição de estados transposta. [9,9]
	arm_matrix_instance_f32 I;			//Matriz identidadee. [9,9]
	arm_matrix_instance_f32 P;			//Matriz de confiabilidade do processo de atualização. [9,9]
	arm_matrix_instance_f32 h;			//Matriz de mapeamento do estado para o erro [6,9]
	arm_matrix_instance_f32 H;			//Matriz Jacobiana para atualização da confiabilidade do erro.
	arm_matrix_instance_f32 Ht;			//Matriz Jacobiana transposta para atualização da confiabilidade do erro.
	arm_matrix_instance_f32 Q;			//Matriz de covariância multiplicada por dt; [9,9]
	arm_matrix_instance_f32 R;			//Matriz de variância [6,6]
	arm_matrix_instance_f32 y;			//Matriz de erro entre medidas e estado estimado. [6,1]
	arm_matrix_instance_f32 z;			
	arm_matrix_instance_f32 S;			//Matriz .... [6,6]
	arm_matrix_instance_f32 Sinv;		//Matriz F inversa.
	arm_matrix_instance_f32 K;			//Matriz com os ganhos de Kalman [9,6]

		//Matrices intermediàrias para cálculo

	arm_matrix_instance_f32 temp_calc_910;
	arm_matrix_instance_f32 temp_calc_911;
	
	arm_matrix_instance_f32 temp_calc_990;
	arm_matrix_instance_f32 temp_calc_991;
	arm_matrix_instance_f32 temp_calc_992;
	arm_matrix_instance_f32 temp_calc_993;

	arm_matrix_instance_f32 temp_calc_660;

	arm_matrix_instance_f32 temp_calc_690;

	arm_matrix_instance_f32 temp_calc_960;

	//Matriz S...
	float S_f32[36];
	arm_mat_init_f32(&S, 6, 6, S_f32);

	float Sinv_f32[36];
	arm_mat_init_f32(&Sinv, 6, 6, Sinv_f32);

	//Matriz do ganho de Kalman 
	float K_f32[54];
	arm_mat_init_f32(&K, 9, 6, K_f32);

	//Matrizes de suporte para o cálculo
		//Matrizes de 9 linhas e 1 coluna
	float temp_calc_910_f32[9];
	float temp_calc_911_f32[9];
	
	arm_mat_init_f32(&temp_calc_910, 9, 1, temp_calc_910_f32);
	arm_mat_init_f32(&temp_calc_911, 9, 1, temp_calc_911_f32);

	//Matrizes de 9 linhas e 9 colunas
	float temp_calc_990_f32[81];
	float temp_calc_991_f32[81];
	float temp_calc_992_f32[81];
	float temp_calc_993_f32[81];

	arm_mat_init_f32(&temp_calc_990, 9, 9, temp_calc_990_f32);
	arm_mat_init_f32(&temp_calc_991, 9, 9, temp_calc_991_f32);
	arm_mat_init_f32(&temp_calc_992, 9, 9, temp_calc_992_f32);
	arm_mat_init_f32(&temp_calc_993, 9, 9, temp_calc_993_f32);


	//Matrizes de 6 linhas e 6 colunas
	float temp_calc_660_f32[36];

	arm_mat_init_f32(&temp_calc_660, 6, 6, temp_calc_660_f32);

	//Matrizes de calculos de 12 linhas e 6 colunas
	float temp_calc_960_f32[54];
	arm_mat_init_f32(&temp_calc_960, 9, 6, temp_calc_960_f32);

	float temp_calc_690_f32[54];
	arm_mat_init_f32(&temp_calc_690, 6, 9, temp_calc_690_f32);
	
	
	
	
	/*************************************Atualização dos dados para cálcuo*******************************/
	//Variáveis para cálculos
	
	float dt = buffer_filtro->dt;

	/*Angulos estimados através do filtro. */
	float phi = 	buffer_filtro->ultimo_estado[0]; 
	float theta = 	buffer_filtro->ultimo_estado[1];
	float psi = 	buffer_filtro->ultimo_estado[2];

	/*Bias do giroscópio estimado. */
	float bp = buffer_filtro->ultimo_estado[3];
	float bq = buffer_filtro->ultimo_estado[4];
	float br = buffer_filtro->ultimo_estado[5];

	/*Bias do magnetômetro. */
	float bmx = buffer_filtro->ultimo_estado[6];
	float bmy = buffer_filtro->ultimo_estado[7];
	float bmz = buffer_filtro->ultimo_estado[8];

	/*Velocidades angulares subtraídas dos bias. */
	float p = medida_gyro[0] - bp;
	float q = medida_gyro[1] - bq;
	float r = medida_gyro[2] - br;

	/*Atualização dos estados dos ângulos com base nas velocidades angulares e do bias estimado anteriormente*/
	float phi_ = phi + p*dt + f_sin(phi)*f_tan(theta)*q*dt + f_cos(phi)*f_tan(theta)*r*dt;
	float theta_ = theta + f_cos(phi)*q*dt - f_sin(phi)*r*dt;
	float psi_ = psi + f_sin(phi)*f_sec(theta)*q*dt + f_cos(phi)*f_sec(theta)*r*dt;

	phi = phi_;
	theta = theta_;
	psi = tratar_intervalo_Angulo(psi_);

	//Elementos da matriz Jacobiana para atualização dos estados (F).
	float a11 = 1 + f_cos(phi)*f_tan(theta)*q*dt - f_sin(phi)*f_tan(theta)*r*dt;
	float a12 = f_sin(phi)*(pow(f_sec(theta),2))*q*dt + f_cos(phi)*(pow(f_sec(theta),2))*r*dt;
	float a14 = -dt;
	float a15 = -f_sin(phi)*f_tan(theta)*dt;
	float a16 = -f_cos(phi)*f_tan(theta)*dt;

	float a21 = -f_sin(phi)*q*dt - f_cos(phi)*r*dt;
	float a25 = -f_cos(phi)*dt;
	float a26 = f_sin(phi)*dt;

	float a31 = f_cos(phi)*f_sec(theta)*q*dt - f_sin(phi)*f_sec(theta)*r*dt;
	float a32 = f_sin(phi)*f_tan(theta)*f_sec(theta)*q*dt + f_cos(phi)*f_tan(theta)*f_sec(theta)*r*dt;
	float a35 = -f_sin(phi)*f_sec(theta)*dt;
	float a36 = -f_cos(phi)*f_sec(theta)*dt;

	//Matriz Jacobiana para atualização de P
	float F_f32[81] = {	a11,    a12,    0,    	a14,    a15,    a16,    0,      0,      0,
        				a21,    1,    	0,    	0,    	a25,    a26,    0,      0,      0,
        				a31,    a32,    1,    	0,    	a35,    a36,    0,      0,      0,
        				0,      0,      0,      1,      0,      0,      0,      0,      0,
        				0,      0,      0,      0,      1,      0,      0,      0,      0,
        				0,      0,      0,      0,      0,      1,      0,      0,      0,
        				0,      0,      0,      0,      0,      0,      1,      0,      0,
        				0,      0,      0,      0,      0,      0,      0,      1,      0,
					    0,      0,      0,      0,      0,      0,      0,      0,      1};

	arm_mat_init_f32(&F, 9, 9, F_f32);					   

	//Matriz Jacobiana transposta para atualização de P.
	float Ft_f32[81] = 	{	a11,	a21,	a31,	0,		0,		0,		0,		0,		0,
							a12,	1,		a32,	0,		0,		0,		0,		0,		0,
							0,		0,		1,		0,		0,		0,		0,		0,		0,
							a14,	0,		0,		1,		0,		0,		0,		0,		0,
							a15,	a25,	a35,	0,		1,		0,		0,		0,		0,
							a16,	a26,	a36,	0,		0,		1,		0,		0,		0,
							0,		0,		0,		0,		0,		0,		1,		0,		0,
							0,		0,		0,		0,		0,		0,		0,		1,		0,
							0,		0,		0,		0,		0,		0,		0,		0,		1};						

	arm_mat_init_f32(&Ft, 9, 9, Ft_f32);

	//Processo à priori para atualização da matriz de confiabilidade P.

	//Matriz de covariâncias do processo de atualização (Q).
	float Qacel = (buffer_filtro->Q_acel);
	float Qmag = (buffer_filtro->Q_mag);
	float Qbias = (buffer_filtro->Q_bias);

	float Q_f32[81] = {(Qacel), 0, 0, 0, 0, 0, 0, 0, 0,
	 					0, (Qacel), 0, 0, 0, 0, 0, 0, 0,
	 					0, 0, (Qacel), 0, 0, 0, 0, 0, 0,
	 					0, 0, 0, (Qbias), 0, 0, 0, 0, 0,
	 					0, 0, 0, 0, (Qbias), 0, 0, 0, 0,
	 					0, 0, 0, 0, 0, (Qbias)*1e-6, 0, 0, 0,
	 					0, 0, 0, 0, 0, 0, (Qmag), 0, 0,
	 					0, 0, 0, 0, 0, 0, 0, (Qmag), 0,
	 					0, 0, 0, 0, 0, 0, 0, 0, (Qmag)};

	arm_mat_init_f32(&Q, 9, 9, Q_f32);


	/*Matriz de confiabilidade do processo de atualização. */
	/* Pk|k-1 = Pk-1|k-1 */
	float P_f32[81];
	arm_copy_f32(buffer_filtro->P, P_f32, 81);
	arm_mat_init_f32(&P, 9, 9, P_f32);

	//temp_calc_990 = F*P
	arm_mat_mult_f32(&F, &P, &temp_calc_990);

	//temp_calc_991 = F*P*F'
	arm_mat_mult_f32(&temp_calc_990, &Ft, &temp_calc_991);

	//P = temp_calc_991 + Q = F*P*F' + Q
	arm_mat_add_f32(&temp_calc_991, &Q, &P);

	/*Estados iniciais do magnetômetro */
	float bx = buffer_filtro->MagInicial[0]; 
	float by = buffer_filtro->MagInicial[1]; 
	float bz = buffer_filtro->MagInicial[2]; 

	/* Cálculo das referências com base no magnetômetro e no estado do acelerômetro parado [0; 0; 1]; */
	float bmagx = bmx + f_sin(theta)*(bmz - bz) - f_cos(psi)*f_cos(theta)*(bmx - bx) - f_cos(theta)*f_sin(psi)*(bmy - by);
	float bmagy = bmy + (f_cos(phi)*f_sin(psi) - f_cos(psi)*f_sin(phi)*f_sin(theta))*(bmx - bx) - (f_cos(phi)*f_cos(psi) + f_sin(phi)*f_sin(psi)*f_sin(theta))*(bmy - by) - f_cos(theta)*f_sin(phi)*(bmz - bz);
	float bmagz = bmz - (f_sin(phi)*f_sin(psi) + f_cos(phi)*f_cos(psi)*f_sin(theta))*(bmx - bx) + (f_cos(psi)*f_sin(phi) - f_cos(phi)*f_sin(psi)*f_sin(theta))*(bmy - by) - f_cos(phi)*f_cos(theta)*(bmz - bz);
	
	//float mod_mag = sqrt(pow((bmagx-bmx),2)+pow((bmagy-bmy),2)+pow((bmagz-bmz),2));

	float h_f32[6] = {  -f_sin(theta),
        				f_cos(theta)*f_sin(phi),
        				f_cos(phi)*f_cos(theta),
        				(bmagx+bmx),
        				(bmagy+bmy),
        				(bmagz+bmz)};

	arm_mat_init_f32(&h, 6, 1, h_f32);        				

    /* Cálculo dos erros */

	//Matriz de erro entre medida e estado estimado
	float y_f32[6];
	arm_mat_init_f32(&y, 6, 1, y_f32);

	/*Atualizar a matriz Z com base nasm medidas do giroscópio e acelerômetro */

	float z_f32[6];
	arm_copy_f32(medida_accel, z_f32, 3);
	arm_copy_f32(medida_mag, z_f32+3, 3);
	arm_mat_init_f32(&z, 6, 1, z_f32);

	/*Cálculo da matriz y: y = z - h */
	arm_mat_sub_f32(&z, &h, &y);


	/* Elemetnos da matriz jacobiana para cálculo da confiabilidade do processo de estimação do erro */
	float h12 = -f_cos(theta);

	float h21 = f_cos(phi)*f_cos(theta);
	float h22 = -f_sin(phi)*f_sin(theta);

	float h31 = -f_cos(theta)*f_sin(phi);
	float h32 = -f_cos(phi)*f_sin(theta);

	float h42 = f_cos(theta)*(bmz - bz) + f_cos(psi)*f_sin(theta)*(bmx - bx) + f_sin(psi)*f_sin(theta)*(bmy - by);
	float h43 = f_cos(theta)*f_sin(psi)*(bmx - bx) - f_cos(psi)*f_cos(theta)*(bmy - by);

	float h47 = 1 - f_cos(psi)*f_cos(theta);
	float h48 = -f_cos(theta)*f_sin(psi);
	float h49 = f_sin(theta);

	float h51 = (f_cos(psi)*f_sin(phi) - f_cos(phi)*f_sin(psi)*f_sin(theta))*(bmy - by) - (f_sin(phi)*f_sin(psi) + f_cos(phi)*f_cos(psi)*f_sin(theta))*(bmx - bx) - f_cos(phi)*f_cos(theta)*(bmz - bz);
	float h52 = f_sin(phi)*f_sin(theta)*(bmz - bz) - f_cos(psi)*f_cos(theta)*f_sin(phi)*(bmx - bx) - f_cos(theta)*f_sin(phi)*f_sin(psi)*(bmy - by);
	float h53 = (f_cos(phi)*f_cos(psi) + f_sin(phi)*f_sin(psi)*f_sin(theta))*(bmx - bx) + (f_cos(phi)*f_sin(psi) - f_cos(psi)*f_sin(phi)*f_sin(theta))*(bmy - by);

	float h57 = f_cos(phi)*f_sin(psi) - f_cos(psi)*f_sin(phi)*f_sin(theta);
	float h58 = 1 - f_sin(phi)*f_sin(psi)*f_sin(theta) - f_cos(phi)*f_cos(psi);
	float h59 = -f_cos(theta)*f_sin(phi);

	float h61 = (f_cos(phi)*f_cos(psi) + f_sin(phi)*f_sin(psi)*f_sin(theta))*(bmy - by) - (f_cos(phi)*f_sin(psi) - f_cos(psi)*f_sin(phi)*f_sin(theta))*(bmx - bx) + f_cos(theta)*f_sin(phi)*(bmz - bz);
	float h62 = f_cos(phi)*f_sin(theta)*(bmz - bz) - f_cos(phi)*f_cos(psi)*f_cos(theta)*(bmx - bx) - f_cos(phi)*f_cos(theta)*f_sin(psi)*(bmy - by);
	float h63 = - (f_cos(psi)*f_sin(phi) - f_cos(phi)*f_sin(psi)*f_sin(theta))*(bmx - bx) - (f_sin(phi)*f_sin(psi) + f_cos(phi)*f_cos(psi)*f_sin(theta))*(bmy - by);

	float h67 = -f_sin(phi)*f_sin(psi) - f_cos(phi)*f_cos(psi)*f_sin(theta);
	float h68 = f_cos(psi)*f_sin(phi) - f_cos(phi)*f_sin(psi)*f_sin(theta);
	float h69 = 1 - f_cos(phi)*f_cos(theta);

	/* Matriz Jacobiana para cálculo da confiabilidade do erro */
	float H_f32[54] =	{	0,      h12,    0,      0,      0,      0,      0,      0,      0,
			        		h21,    h22,    0,      0,      0,      0,      0,      0,      0,
			        		h31,    h32,    0,      0,      0,      0,      0,      0,      0,
			        		0,    	h42,    h43,    0,      0,      0,      h47,    h48,    h49,
			        		h51,    h52,    h53,    0,      0,      0,      h57,    h58,	h59,
			        		h61,    h62,    h63,   	0,      0,      0,      h67,   	h68,   	h69};

	arm_mat_init_f32(&H, 6, 9, H_f32);


	/* Matriz Jacobiana transposta para cálculo da confiabilidade do erro . */
	float Ht_f32[54] ={	0,		h21,	h31,	0,		h51,	h61,
						h12,	h22,	h32,	h42,	h52,	h62,
						0,		0,		0,		h43,	h53,	h63,
						0,		0,		0,		0,		0,		0,
						0,		0,		0,		0,		0,		0,
						0,		0,		0,		0,		0,		0,
						0,		0,		0,		h47,	h57,	h67,
						0,		0,		0,		h48,	h58,	h68,
						0,		0,		0,		h49,	h59,	h69};

	arm_mat_init_f32(&Ht, 9, 6, Ht_f32);

	//Matriz de variâncias
	float Racel = buffer_filtro->R_acel;
	float Rmag = buffer_filtro->R_mag; //Variância inicial do magnetômetro.

	float R_f32[36] = {(Racel), 0, 0, 0, 0, 0,
					   0, (Racel), 0, 0, 0, 0,
					   0, 0, (Racel), 0, 0, 0,
					   0, 0, 0, (Rmag), 0, 0,
					   0, 0, 0, 0, (Rmag), 0,
					   0, 0, 0, 0, 0, (Rmag)};

	arm_mat_init_f32(&R, 6, 6, R_f32);


	//Cálculos do filtro de Kalman

	//S = H*P*H' + R
	arm_mat_mult_f32(&H, &P, &temp_calc_690);
	arm_mat_mult_f32(&temp_calc_690, &Ht, &temp_calc_660);
	arm_mat_add_f32(&temp_calc_660, &R, &S);

	//Sinv = inv(S);
	arm_mat_inverse_f32(&S, &Sinv);

	//Kk = P*Ht*S^(-1)
		//P*Ht
	arm_mat_mult_f32(&P, &Ht, &temp_calc_960);

	arm_mat_mult_f32(&temp_calc_960, &Sinv, &K);
	
	//temp_calc_911 = Kk*y
	arm_mat_mult_f32(&K, &y, &temp_calc_911);


	float X_f32[9] = {phi, theta, psi, bp, bq, br, bmx, bmy, bmz};
	arm_mat_init_f32(&X, 9, 1, X_f32);

	//X = X + temp_calc_911;
	arm_mat_add_f32(&X, &temp_calc_911, &temp_calc_910);
	arm_copy_f32(temp_calc_910_f32, X_f32, 9);

	//P = (I-K*H)*P
	
	//Matriz identidade para atualização da matriz P à posteriori.
	float I_f32[81] = {	1,0,0,0,0,0,0,0,0,
					    0,1,0,0,0,0,0,0,0,
					    0,0,1,0,0,0,0,0,0,
					    0,0,0,1,0,0,0,0,0,
					    0,0,0,0,1,0,0,0,0,
					    0,0,0,0,0,1,0,0,0,
					    0,0,0,0,0,0,1,0,0,
					    0,0,0,0,0,0,0,1,0,
					    0,0,0,0,0,0,0,0,1};

	arm_mat_init_f32(&I, 9, 9, I_f32);


	arm_mat_mult_f32(&K, &H, &temp_calc_992);
	arm_mat_sub_f32(&I, &temp_calc_992, &temp_calc_993);

	arm_mat_mult_f32(&temp_calc_993, &P, &temp_calc_992);


	arm_copy_f32(X_f32, buffer_filtro->ultimo_estado, 9);
	arm_copy_f32(temp_calc_992_f32, buffer_filtro->P, 81);

}


/*
 * kalman_filter.c
 *
 *  Created on: Sep 8, 2013
 *      Author: Bruno
 */
#include "kalman_filter.h"

//------ Função para cálculo do filtro de kalman sobre o Giroscópio e o Acelerômetro -------///
//Utiliza DCM -> Variação do vetor com base na velocidae agular para atualização da matrz de atualização de estados.
//Bruno f. M. Callegaro - 20/12/2013


void kalman_filter(kalman_filter_state *buffer_filtro, float medida_gyro[], float medida_accel[], float medida_mag[], uint16_t estado_motores)
{
	//Instancias das matrizes utilizadas para o cálculo
    arm_matrix_instance_f32 X;			//Matriz de estados. [n,1]
    arm_matrix_instance_f32 F;			//Matriz de transição de estados. [n,n]
    arm_matrix_instance_f32 Ft;			//Matriz de transição de estados transposta. [n,n]
    arm_matrix_instance_f32 I;			//Matriz identidadee. [n,n]
    arm_matrix_instance_f32 P;			//Matriz de confiabilidade do processo de atualização. [n,n]
    //arm_matrix_instance_f32 h;		//Matriz de mapeamento de observabilidade [a,n]
    arm_matrix_instance_f32 H;			//Matriz Jacobiana para atualização da confiabilidade do erro [a, n].
    arm_matrix_instance_f32 Ht;			//Matriz Jacobiana transposta para atualização da confiabilidade do erro. [n, a]
    arm_matrix_instance_f32 Q;			//Matriz de covariância multiplicada de processos; [n, n]
    arm_matrix_instance_f32 R;			//Matriz de variância [a ,a]
    arm_matrix_instance_f32 y;			//Matriz de erro entre medidas e estado estimado. [a,1]
	arm_matrix_instance_f32 z;			
    arm_matrix_instance_f32 S;			//Matriz .... [a, a]
    arm_matrix_instance_f32 Sinv;		//Matriz S inversa. [a, a]
    arm_matrix_instance_f32 K;			//Matriz com os ganhos de Kalman [n,a]

		//Matrices intermediàrias para cálculo

    arm_matrix_instance_f32 temp_calc_a1_0;
    arm_matrix_instance_f32 temp_calc_a1_1;

    arm_matrix_instance_f32 temp_calc_n1_0;
    arm_matrix_instance_f32 temp_calc_n1_1;
	
    arm_matrix_instance_f32 temp_calc_aa_0;
    arm_matrix_instance_f32 temp_calc_aa_1;
	
    arm_matrix_instance_f32 temp_calc_na_0;
    arm_matrix_instance_f32 temp_calc_an_0;

    arm_matrix_instance_f32 temp_calc_nn_0;
    arm_matrix_instance_f32 temp_calc_nn_1;

	//Matriz S...
    float S_f32[a*a];
    arm_mat_init_f32(&S, a, a, S_f32);

    float Sinv_f32[a*a];
    arm_mat_init_f32(&Sinv, a, a, Sinv_f32);

	//Matriz do ganho de Kalman 
    float K_f32[a*n];
    arm_mat_init_f32(&K, a, n, K_f32);

	//Matrizes de suporte para o cálculo
		//Matrizes de 9 linhas e 1 coluna
    float temp_calc_n1_0_f32[n];
    float temp_calc_n1_1_f32[n];
	
    arm_mat_init_f32(&temp_calc_n1_0, n, 1, temp_calc_n1_0_f32);
    arm_mat_init_f32(&temp_calc_n1_1, n, 1, temp_calc_n1_1_f32);

	//Matrizes de 9 linhas e 1 coluna
    float temp_calc_a1_0_f32[a];
    float temp_calc_a1_1_f32[a];
	
    arm_mat_init_f32(&temp_calc_a1_0, a, 1, temp_calc_a1_0_f32);
    arm_mat_init_f32(&temp_calc_a1_1, a, 1, temp_calc_a1_1_f32);

	//Matrizes de 6 linhas e 6 colunas
    float temp_calc_aa_0_f32[a*a];
    float temp_calc_aa_1_f32[a*a];

    arm_mat_init_f32(&temp_calc_aa_0, a, a, temp_calc_aa_0_f32);
    arm_mat_init_f32(&temp_calc_aa_1, a, a, temp_calc_aa_1_f32);
	
	//Matrizes de 9 linhas e 6 colunas
    float temp_calc_na_0_f32[n*a];

    arm_mat_init_f32(&temp_calc_na_0, n, a, temp_calc_na_0_f32);

	//Matrizes de 6 linhas e 9 colunas
    float temp_calc_an_0_f32[a*n];

    arm_mat_init_f32(&temp_calc_an_0, a, n, temp_calc_an_0_f32);

	//Matrizes de 9 linhas e 9 colunas
    float temp_calc_nn_0_f32[n*n];
    float temp_calc_nn_1_f32[n*n];

    arm_mat_init_f32(&temp_calc_nn_0, n, n, temp_calc_nn_0_f32);
    arm_mat_init_f32(&temp_calc_nn_1, n, n, temp_calc_nn_1_f32);
	
	/*************************************Atualização dos dados para cálcuo*******************************/
	//Variáveis para cálculos
	
	float dt = buffer_filtro->dt;

	/*Angulos estimados através do filtro. */

	
    /*Bias do magnetômetro. */

	/*Velocidades angulares subtraídas dos bias. */
    float wx = medida_gyro[0];
    float wy = medida_gyro[1];
    float wz = medida_gyro[2];

	/*Atualização dos estados dos ângulos com base nas velocidades angulares e do bias estimado anteriormente*/
    float X_f32[n];
    arm_mat_init_f32(&X, n, 1, X_f32);

    arm_copy_f32(buffer_filtro->ultimo_estado, X_f32, n);


    float a11 = 1;
    float a12 = -(dt*wx)/2;
    float a13 = -(dt*wy)/2;
    float a14 = -(dt*wz)/2;

    float a21 = (dt*wx)/2;
    float a22 = 1;
    float a23 = -(dt*wz)/2;
    float a24 = (dt*wy)/2;

    float a31 = (dt*wy)/2;
    float a32 = (dt*wz)/2;
    float a33 = 1;
    float a34 = (dt*wz)/2;

    float a41 = (dt*wz)/2;
    float a42 = -(dt*wy)/2;
    float a43 = (dt*wx)/2;
    float a44 = 1;

    //Elementos da matriz para atualização dos estados (F).
    float F_f32[n*n] = {	a11,	a12,	a13,	a14,	0,		0,      0,      0,      0,      0,
                            a21,	a22,	a23,	a24,	0,		0,      0,      0,      0,      0,
                            a31,	a32,	a33,	a34,	0,		0,      0,      0,      0,      0,
                            a41,	a42,	a43,	a44,	0,		0,      0,      0,      0,      0,
                            0,		0,		0,		0,		1,		0,      0,      0,      0,      0,
                            0,		0,		0,		0,		0,		1,      0,      0,      0,      0,
                            0,      0,      0,      0,      0,      0,      1,      0,      0,      0,
                            0,      0,      0,      0,      0,      0,      0,      1,      0,      0,
                            0,      0,      0,      0,      0,      0,      0,      0,      1,      0,
                            0,      0,      0,      0,      0,      0,      0,      0,      0,      1};

    arm_mat_init_f32(&F, n, n, F_f32);

    arm_mat_mult_f32(&F, &X, &temp_calc_n1_0);

	//Matriz Jacobiana transposta para atualização de P.
    float Ft_f32[n*n] =	{	a11,	a21,	a31,	a41,	0,		0,      0,      0,      0,      0,
                            a12,	a22,	a32,	a42,	0,		0,      0,      0,      0,      0,
                            a13,	a23,	a33,	a43,	0,		0,      0,      0,      0,      0,
                            a14,	a24,	a34,	a44,	0,		0,      0,      0,      0,      0,
                            0,		0,		0,		0,		1,		0,      0,      0,      0,      0,
                            0,		0,		0,		0,		0,		1,      0,      0,      0,      0,
                            0,      0,      0,      0,      0,      0,      1,      0,      0,      0,
                            0,      0,      0,      0,      0,      0,      0,      1,      0,      0,
                            0,      0,      0,      0,      0,      0,      0,      0,      1,      0,
                            0,      0,      0,      0,      0,      0,      0,      0,      0,      1};

    arm_mat_init_f32(&Ft, n, n, Ft_f32);

	//Processo à priori para atualização da matriz de confiabilidade P.

	//Matriz de covariâncias do processo de atualização (Q).
    float qQuat = (buffer_filtro->Q_quat);
    float qBiasAcel = (buffer_filtro->Q_bias_acel);
    float qBiasMag = (buffer_filtro->Q_bias_mag);

    float Q_f32[n*n] = {	qQuat,	0,  	0,  	0,  	0,          0,          0,          0,          0,          0,
                            0,  	qQuat,	0,  	0,  	0,          0,          0,          0,          0,          0,
                            0,  	0,  	qQuat,	0,  	0,          0,          0,          0,          0,          0,
                            0,  	0,  	0,  	qQuat,	0,          0,          0,          0,          0,          0,
                            0,		0,		0,		0,		qBiasAcel,  0,          0,          0,          0,          0,
                            0,		0,		0,		0,		0,          qBiasAcel,  0,          0,          0,          0,
                            0,      0,      0,      0,      0,          0,          qBiasAcel,  0,          0,          0,
                            0,      0,      0,      0,      0,          0,          0,          qBiasMag,   0,          0,
                            0,      0,      0,      0,      0,          0,          0,          0,          qBiasMag,   0,
                            0,      0,      0,      0,      0,          0,          0,          0,          0,          qBiasMag};

    arm_mat_init_f32(&Q, n, n, Q_f32);


	/*Matriz de confiabilidade do processo de atualização. */
	/* Pk|k-1 = Pk-1|k-1 */
    float P_f32[n*n];
    arm_copy_f32(buffer_filtro->P, P_f32, n*n);
    arm_mat_init_f32(&P, n, n, P_f32);

    //temp_calc_nn_0 = F*P
    arm_mat_mult_f32(&F, &P, &temp_calc_nn_0);

    //temp_calc_nn_1 = F*P*F'
    arm_mat_mult_f32(&temp_calc_nn_0, &Ft, &temp_calc_nn_1);

    //P = temp_calc_nn_1 + Q = F*P*F' + Q
    arm_mat_add_f32(&temp_calc_nn_1, &Q, &P);

	/*Estados iniciais do magnetômetro */
    float magRefVector[3];
    float acelRefVector[3];
    arm_matrix_instance_f32 magRefMatrix;
    arm_matrix_instance_f32 acelRefMatrix;

    arm_mat_init_f32(&magRefMatrix, 3, 1, magRefVector);
    arm_mat_init_f32(&acelRefMatrix, 3, 1, acelRefVector);

    float gx = 0;
    float gy = 0;
    float gz = 1;

    float hx = buffer_filtro->MagInicial[0];
    float hy = buffer_filtro->MagInicial[1];
    float hz = buffer_filtro->MagInicial[2];

    magRefVector[0] = hx;
    magRefVector[1] = hy;
    magRefVector[2] = hz;

    acelRefVector[0] = gx;
    acelRefVector[1] = gy;
    acelRefVector[2] = gz;

    //Matrizes com o resultado das operações de rotação
    float observatedStateVector[a];

    arm_matrix_instance_f32 observatedStateMatrix;
    arm_mat_init_f32(&observatedStateMatrix, a, 1, observatedStateVector);

    arm_matrix_instance_f32 rotatedMagMatrix;
    arm_matrix_instance_f32 rotatedAcelMatrix;
    arm_mat_init_f32(&rotatedAcelMatrix, 3, 1, observatedStateVector);
    arm_mat_init_f32(&rotatedMagMatrix, 3, 1, observatedStateVector+3);

    //Matriz de rotação com base no quarternion estimado.
    float rotationVector[9];
    arm_matrix_instance_f32 rotationMatrix;
    arm_mat_init_f32(&rotationMatrix, 3, 3, rotationVector);
    getRotMatFromQuaternion(temp_calc_n1_0_f32, &rotationMatrix);

    /* Cálculo das referências com base no magnetômetro e no estado do acelerômetro parado [0; 0; 1]; */
    arm_mat_mult_f32(&rotationMatrix, &acelRefMatrix, &rotatedAcelMatrix);
    arm_mat_mult_f32(&rotationMatrix, &magRefMatrix, &rotatedMagMatrix);

    //Vetor com as médidas
    float zkVector[a];

    zkVector[0] = medida_accel[0];
    zkVector[1] = medida_accel[1];
    zkVector[2] = medida_accel[2];

    zkVector[0] = medida_mag[0];
    zkVector[1] = medida_mag[1];
    zkVector[2] = medida_mag[2];

    arm_matrix_instance_f32 zkMatrix;
    arm_mat_init_f32(&zkMatrix, a, 1, zkVector);

    //Vetor de resíduo
    float ykVector[a];
    arm_matrix_instance_f32 ykMatrix;
    arm_mat_init_f32(&ykMatrix, a, 1, ykVector);

    arm_mat_sub_f32(&zkMatrix, &observatedStateMatrix, &ykMatrix);

    float q0 = temp_calc_n1_0_f32[0];
    float q1 = temp_calc_n1_0_f32[1];
    float q2 = temp_calc_n1_0_f32[2];
    float q3 = temp_calc_n1_0_f32[3];

    /* Elemetnos da matriz jacobiana para cálculo da confiabilidade do processo de estimação do erro */
    float h11 = 2*gy*q3 - 2*gz*q2;
    float h12 = 2*gy*q2 + 2*gz*q3;
    float h13 = 2*gy*q1 - 4*gx*q2 - 2*gz*q0;
    float h14 = 2*gy*q0 - 4*gx*q3 + 2*gz*q1;
//    float h15 = 1;


    float h21 = 2*gz*q1 - 2*gx*q3;
    float h22 = 2*gx*q2 - 4*gy*q1 + 2*gz*q0;
    float h23 = 2*gx*q1 + 2*gz*q3;
    float h24 = 2*gz*q2 - 4*gy*q3 - 2*gx*q0;
//    float h25 = 0;
//    float h26 = 1;

    float h31 = 2*gx*q2 - 2*gy*q1;
    float h32 = 2*gx*q3 - 2*gy*q0 - 4*gz*q1;
    float h33 = 2*gx*q0 + 2*gy*q3 - 4*gz*q2;
    float h34 = 2*gx*q1 + 2*gy*q2;
//    float h35 = 0;
//    float h36 = 0;
//    float h37 = 1;

    float h41 = 2*hy*q3 - 2*hz*q2;
    float h42 = 2*hy*q2 + 2*hz*q3;
    float h43 = 2*hy*q1 - 4*hx*q2 - 2*hz*q0;
    float h44 = 2*hy*q0 - 4*hx*q3 + 2*hz*q1;
//    float h45 = 0;
//    float h46 = 0;
//    float h47 = 0;
//    float h48 = 1;

    float h51 = 2*hz*q1 - 2*hx*q3;
    float h52 = 2*hx*q2 - 4*hy*q1 + 2*hz*q0;
    float h53 = 2*hx*q1 + 2*hz*q3;
    float h54 = 2*hz*q2 - 4*hy*q3 - 2*hx*q0;
//    float h55 = 0;
//    float h56 = 0;
//    float h57 = 0;
//    float h58 = 0;
//    float h59 = 1;

    float h61 = 2*hx*q2 - 2*hy*q1;
    float h62 = 2*hx*q3 - 2*hy*q0 - 4*hz*q1;
    float h63 = 2*hx*q0 + 2*hy*q3 - 4*hz*q2;
    float h64 = 2*hx*q1 + 2*hy*q2;
//     float h65 = 0;
//     float h66 = 0;
//     float h67 = 0;
//     float h68 = 0;
//     float h69 = 0;
//     float h610= 1;

	/* Matriz Jacobiana para cálculo da confiabilidade do erro */
    float H_f32[a*n] =	{	h11,    h12,    h13,    h14,    1,      0,      0,      0,      0,      0,
                            h21,    h22,    h23,    h24,    0,      1,      0,      0,      0,      0,
                            h31,    h32,    h33,    h34,    0,      0,      1,      0,      0,      0,
                            h41,    h42,    h43,	h44,   	0,    	0,      0,      1,      0,      0,
                            h51,    h52,    h53,    h54,   	0,		0,      0,      0,      1,      0,
                            h61,    h62,    h63,   	h64,   	0,   	0,      0,      0,      0,      1};

    arm_mat_init_f32(&H, a, n, H_f32);


	/* Matriz Jacobiana transposta para cálculo da confiabilidade do erro . */
    float Ht_f32[n*a] = {	h11,	h21,	h31,	h41,	h51,	h61,
                            h12,	h22,	h32,	h42,	h52,	h62,
                            h13,	h23,	h33,	h43,	h53,	h63,
                            h14,	h24,	h34,	h44,	h54,	h64,
                            1,		0,		0,		0,		0,		0,
                            0,		1,		0,		0,		0,		0,
                            0,      0,      1,      0,      0,      0,
                            0,      0,      0,      1,      0,      0,
                            0,      0,      0,      0,      1,      0,
                            0,      0,      0,      0,      0,      1};

    arm_mat_init_f32(&Ht, n, a, Ht_f32);

	//Matriz de variâncias
	float Racel = buffer_filtro->R_acel;
	float Rmag = buffer_filtro->R_mag;	 //Variância inicial do magnetômetro.

    float R_f32[a*a] = {(Racel), 0, 0, 0, 0, 0,
                        0, (Racel), 0, 0, 0, 0,
                        0, 0, (Racel), 0, 0, 0,
                        0, 0, 0, (Rmag),  0, 0,
                        0, 0, 0, 0, (Rmag),	 0,
                        0, 0, 0, 0, 0, (Rmag)};

    arm_mat_init_f32(&R, a, a, R_f32);


	//Cálculos do filtro de Kalman

	//S = H*P*H' + R
    arm_mat_mult_f32(&H, &P, &temp_calc_an_0);
    arm_mat_mult_f32(&temp_calc_an_0, &Ht, &temp_calc_aa_0);
    arm_mat_add_f32(&temp_calc_aa_0, &R, &S);

	//Sinv = inv(S);
	arm_mat_inverse_f32(&S, &Sinv);

	//Kk = P*Ht*S^(-1)
		//P*Ht
    arm_mat_mult_f32(&P, &Ht, &temp_calc_na_0);

    arm_mat_mult_f32(&temp_calc_na_0, &Sinv, &K);
	
    //temp_calc_n11 = Kk*y
    arm_mat_mult_f32(&K, &y, &temp_calc_n1_0);

    //X = X + temp_calc_n1_1;
    arm_mat_add_f32(&X, &temp_calc_n1_0, &temp_calc_n1_1);
    arm_copy_f32(temp_calc_n1_1_f32, X_f32, n);

	//P = (I-K*H)*P
	
	//Matriz identidade para atualização da matriz P à posteriori.
    float I_f32[n*n] = {	1,0,0,0,0,0,0,0,0,0,
                            0,1,0,0,0,0,0,0,0,0,
                            0,0,1,0,0,0,0,0,0,0,
                            0,0,0,1,0,0,0,0,0,0,
                            0,0,0,0,1,0,0,0,0,0,
                            0,0,0,0,0,1,0,0,0,0,
                            0,0,0,0,0,0,1,0,0,0,
                            0,0,0,0,0,0,0,1,0,0,
                            0,0,0,0,0,0,0,0,1,0,
                            0,0,0,0,0,0,0,0,0,1};

    arm_mat_init_f32(&I, n, n, I_f32);


    arm_mat_mult_f32(&K, &H, &temp_calc_nn_0);
    arm_mat_sub_f32(&I, &temp_calc_nn_0, &temp_calc_nn_1);

    arm_mat_mult_f32(&temp_calc_nn_1, &P, &temp_calc_nn_0);

    float quatMod = powf(X_f32[0],2) + powf(X_f32[1],2) + powf(X_f32[2],2) + powf(X_f32[3],2);
    arm_sqrt_f32(quatMod, &quatMod);

    X_f32[0] /= quatMod;
    X_f32[1] /= quatMod;
    X_f32[2] /= quatMod;
    X_f32[3] /= quatMod;

    arm_copy_f32(X_f32, buffer_filtro->ultimo_estado, n);
    arm_copy_f32(temp_calc_nn_0_f32, buffer_filtro->P, n*n);
}

void getRotMatFromQuaternion(float quaternion[4], arm_matrix_instance_f32 *rotationMatrix) {
    float q0, q1, q2, q3, temp;
    q0 = quaternion[0];
    q1 = quaternion[1];
    q2 = quaternion[2];
    q3 = quaternion[3];

    temp = (- 2*powf(q2,2) - 2*powf(q3,2) + 1);
    arm_mat_set_element(rotationMatrix, 1, 1, temp);

    temp = (2*q0*q3 + 2*q1*q2);
    arm_mat_set_element(rotationMatrix, 1, 2, temp);

    temp = (2*q1*q3 - 2*q0*q2);
    arm_mat_set_element(rotationMatrix, 1, 3, temp);

    temp = (2*q1*q2 - 2*q0*q3);
    arm_mat_set_element(rotationMatrix, 2, 1, temp);

    temp = (- 2*powf(q1,2) - 2*powf(q3,2) + 1);
    arm_mat_set_element(rotationMatrix, 2, 2, temp);

    temp = (2*q0*q1 + 2*q2*q3);
    arm_mat_set_element(rotationMatrix, 2, 3, temp);

    temp = (2*q0*q2 + 2*q1*q3);
    arm_mat_set_element(rotationMatrix, 3, 1, temp);

    temp = (2*q2*q3 - 2*q0*q1);
    arm_mat_set_element(rotationMatrix, 3, 2, temp);

    temp = (- 2*powf(q1,2) - 2*powf(q2,2) + 1);
    arm_mat_set_element(rotationMatrix, 3, 3, temp);
}

void arm_mat_set_element(arm_matrix_instance_f32 *entrada, uint16_t row, uint16_t column, float newValue) {

    if(row <= entrada->numRows && column <= entrada->numCols) {
        entrada->pData[row*entrada->numCols + column] = newValue;
    }
}

float arm_mat_get_element(const arm_matrix_instance_f32 *entrada, uint16_t row, uint16_t column) {
    if(row <= entrada->numRows && column <= entrada->numCols) {
        return entrada->pData[row*entrada->numCols + column];
    }
    return 0;
}



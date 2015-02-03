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


void kalman_filter(kalman_filter_state *buffer_filtro, float medida_gyro[], float medida_accel[], float medida_mag[], float angles[], uint16_t estado_motores)
{
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
    //Insf_tancias das matrizes utilizadas para o cálculo
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
    float K_f32[n*a];
    arm_mat_init_f32(&K, n, a, K_f32);

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

	/*Velocidades angulares subtraídas dos bias. */
    float p = medida_gyro[0];
    float q = medida_gyro[1];
    float r = medida_gyro[2];

	/*Atualização dos estados dos ângulos com base nas velocidades angulares e do bias estimado anteriormente*/
    float X_f32[n];
    arm_mat_init_f32(&X, n, 1, X_f32);

    arm_copy_f32(buffer_filtro->ultimo_estado, X_f32, n);

    float phi =  X_f32[0];
    float theta = X_f32[1];
    float psi =  X_f32[2];

    float bPhi = X_f32[9];
    float bTheta = X_f32[10];
    float bPsi = X_f32[11];

    //Atualizar o estado anterior - Apenas os ângulos precisam serm inicializados, uma vez que os bias são atualizados por uma identidade.
    X_f32[0] = phi + (p)*dt + f_sin(phi)*f_tan(theta)*(q)*dt + f_cos(phi)*f_tan(theta)*(r)*dt;
    X_f32[1] = theta + f_cos(phi)*(q)*dt - f_sin(phi)*(r)*dt;
    X_f32[2] = psi + f_sin(phi)*f_sec(theta)*(q)*dt + f_cos(phi)*f_sec(theta)*(r)*dt;

    phi = X_f32[0];
    theta = X_f32[1];
    psi = X_f32[2];

    //Com os angulos calculados, cálcula os senos e cossenos utilizados para agilizar os processos de cálculo.
    float cos_Phi = f_cos(phi);
    float sin_Phi = f_sin(phi);

    float cos_Theta = f_cos(theta);
    float sin_Theta = f_sin(theta);
    float tan_Theta = f_tan(theta);

    //Elementos da matriz para atualização dos estados (F).
    float F_f32[n*n] = { dt*q*cos_Phi*tan_Theta - dt*r*sin_Phi*tan_Theta + 1,               dt*r*cos_Phi*(pow(tan_Theta,2) + 1) + dt*q*sin_Phi*(pow(tan_Theta,2) + 1), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                 - dt*r*cos_Phi - dt*q*sin_Phi,                                                                                 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         (dt*q*cos_Phi)/cos_Theta - (dt*r*sin_Phi)/cos_Theta, (dt*r*cos_Phi*sin_Theta)/pow(cos_Theta,2) + (dt*q*sin_Phi*sin_Theta)/pow(cos_Theta,2), 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                               0,                                                                                 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                               0,                                                                                 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                                                                               0,                                                                                 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                                                                               0,                                                                                 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                                                                               0,                                                                                 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                                                               0,                                                                                 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                                                                               0,                                                                                 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                                                               0,                                                                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                                                                               0,                                                                                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};

    arm_mat_init_f32(&F, n, n, F_f32);

	//Matriz Jacobiana transposta para atualização de P.
    float Ft_f32[n*n];
    arm_mat_init_f32(&Ft, n, n, Ft_f32);
    arm_mat_trans_f32(&F, &Ft);

	//Processo à priori para atualização da matriz de confiabilidade P.

	//Matriz de covariâncias do processo de atualização (Q).
    float qAngles = (buffer_filtro->Q_angles);
    float qBiasAcel = (buffer_filtro->Q_bias_acel);
    float qBiasMag = (buffer_filtro->Q_bias_mag);
    float qBiasAngles = (buffer_filtro->Q_bias_angle);

    /*Matriz de confiabilidade do processo de atualização. */
    /* Pk|k-1 = Pk-1|k-1 */
    float P_f32[n*n];
    arm_copy_f32(buffer_filtro->P, P_f32, n*n);
    arm_mat_init_f32(&P, n, n, P_f32);

    //temp_calc_nn_0 = F*P
    if(arm_mat_mult_f32(&F, &P, &temp_calc_nn_0) != ARM_MATH_SUCCESS)
        while(1);

    //temp_calc_nn_1 = F*P*F'
    if(arm_mat_mult_f32(&temp_calc_nn_0, &Ft, &temp_calc_nn_1) != ARM_MATH_SUCCESS)
        while(1);


    //Atualiza a matriz de covariâncias dos processos
    float Q_f32[n*n] = {	qAngles,   0,           0,          0,          0,          0,          0,          0,          0,          0,          0,              0,
                            0,          qAngles,    0,          0,          0,          0,          0,          0,          0,          0,          0,              0,
                            0,          0,          qAngles,	0,          0,          0,          0,          0,          0,          0,          0,              0,
                            0,          0,          0,          qBiasAcel,  0,          0,          0,          0,          0,          0,          0,              0,
                            0,          0,          0,          0,          qBiasAcel,  0,          0,          0,          0,          0,          0,              0,
                            0,          0,      	0,          0,          0,          qBiasAcel,  0,          0,          0,          0,          0,              0,
                            0,          0,          0,          0,          0,          0,          qBiasMag,   0,          0,          0,          0,              0,
                            0,          0,          0,          0,          0,          0,          0,          qBiasMag,   0,          0,          0,              0,
                            0,          0,          0,          0,          0,          0,          0,          0,          qBiasMag,   0,          0,              0,
                            0,          0,          0,          0,          0,          0,          0,          0,          0,          qBiasAngles, 0,             0,
                            0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          qBiasAngles,    0,
                            0,          0,          0,          0,          0,          0,          0,          0,          0,          0,          0,              qBiasAngles};

    arm_mat_init_f32(&Q, n, n, Q_f32);

    //P = temp_calc_nn_1 + Q = F*P*F' + Q
    if(arm_mat_add_f32(&temp_calc_nn_1, &Q, &P) != ARM_MATH_SUCCESS)
        while(1);

	/*Estados iniciais do magnetômetro */
    float magRefVector[3];
    float acelRefVector[3];
    arm_matrix_instance_f32 magRefMatrix;
    arm_matrix_instance_f32 acelRefMatrix;

    arm_mat_init_f32(&magRefMatrix, 3, 1, magRefVector);
    arm_mat_init_f32(&acelRefMatrix, 3, 1, acelRefVector);

    float ax = buffer_filtro->AcelInicial[0];
    float ay = buffer_filtro->AcelInicial[1];
    float az = buffer_filtro->AcelInicial[2];

    float hx = buffer_filtro->MagInicial[0];
    float hy = buffer_filtro->MagInicial[1];
    float hz = buffer_filtro->MagInicial[2];

    magRefVector[0] = hx;
    magRefVector[1] = hy;
    magRefVector[2] = hz;

    acelRefVector[0] = ax;
    acelRefVector[1] = ay;
    acelRefVector[2] = az;

    //Matrizes com o resultado das operações de rotação
    float observatedStateVector[a];

    arm_matrix_instance_f32 observatedStateMatrix;
    arm_mat_init_f32(&observatedStateMatrix, a, 1, observatedStateVector);

    //Matriz contendo os valores observados utilizados pra gerar o rezíduo (yk)
    arm_matrix_instance_f32 rotatedMagMatrix;
    arm_matrix_instance_f32 rotatedAcelMatrix;

    arm_mat_init_f32(&rotatedAcelMatrix, 3, 1, observatedStateVector);
    arm_mat_init_f32(&rotatedMagMatrix, 3, 1, observatedStateVector+3);

    //Matriz de rotação com base nos angulos estimados.
    float rotationVector[9];
    arm_matrix_instance_f32 rotationMatrix;
    arm_mat_init_f32(&rotationMatrix, 3, 3, rotationVector);
    getABCRotMatFromEulerAngles(phi-bPhi, theta-bTheta, psi-bPsi, &rotationMatrix);

    /* Cálculo das referências com base no magnetômetro e no estado do acelerômetro parado [0; 0; 1]; */
    if(arm_mat_mult_f32(&rotationMatrix, &acelRefMatrix, &rotatedAcelMatrix) != ARM_MATH_SUCCESS)
        while(1);

    if(arm_mat_mult_f32(&rotationMatrix, &magRefMatrix, &rotatedMagMatrix) != ARM_MATH_SUCCESS)
        while(1);

    //Vetor com as médidas
    float zkVector[a];

    zkVector[0] = medida_accel[0];
    zkVector[1] = medida_accel[1];
    zkVector[2] = medida_accel[2];

    zkVector[3] = medida_mag[0];
    zkVector[4] = medida_mag[1];
    zkVector[5] = medida_mag[2];

    zkVector[6] = angles[0];
    zkVector[7] = angles[1];
    zkVector[8] = angles[2];

    arm_matrix_instance_f32 zkMatrix;
    arm_mat_init_f32(&zkMatrix, a, 1, zkVector);

    //Vetor de resíduo
    float ykVector[a];
    arm_matrix_instance_f32 ykMatrix;
    arm_mat_init_f32(&ykMatrix, a, 1, ykVector);

    //Adiciona os bias estimados pelo filtro
    observatedStateVector[0] += X_f32[3];
    observatedStateVector[1] += X_f32[4];
    observatedStateVector[2] += X_f32[5];
    observatedStateVector[3] += X_f32[6];
    observatedStateVector[4] += X_f32[7];
    observatedStateVector[5] += X_f32[8];

    //Remove o bias estimado pelo filtro
    float correctedPhi =    -(X_f32[0] - X_f32[9]);
    float correctedTheta =  -(X_f32[1] - X_f32[10]);
    float correctedPsi =    -(X_f32[2] - X_f32[11]);

    //Com os angulos corrigidos calculados, cálcula os senos e cossenos utilizados para agilizar os processos de cálculo.
    float cos_correctedPhi = f_cos(correctedPhi);
    float sin_correctedPhi = f_sin(correctedPhi);

    float cos_correctedTheta = f_cos(correctedTheta);
    float sin_correctedTheta = f_sin(correctedTheta);

    float cos_correctedPsi = f_cos(correctedPsi);
    float sin_correctedPsi = f_sin(correctedPsi);
    //
    observatedStateVector[6] = -correctedPhi;
    observatedStateVector[7] = -correctedTheta;
    observatedStateVector[8] = -correctedPsi;

    if(arm_mat_sub_f32(&zkMatrix, &observatedStateMatrix, &ykMatrix) != ARM_MATH_SUCCESS)
        while(1);

    float H_f32[a*n] = {                                                                                                                                                                                                                                0,                                                 ax*sin_correctedTheta*cos_correctedPsi - az*cos_correctedTheta - ay*sin_correctedPsi*sin_correctedTheta,                                                                                                         ay*cos_correctedPsi*cos_correctedTheta + ax*sin_correctedPsi*cos_correctedTheta, 1, 0, 0, 0, 0, 0,                                                                                                                                                                                                                                  0,                                                 az*cos_correctedTheta - ax*sin_correctedTheta*cos_correctedPsi + ay*sin_correctedPsi*sin_correctedTheta,                                                                                                       - ay*cos_correctedPsi*cos_correctedTheta - ax*sin_correctedPsi*cos_correctedTheta,
                         ax*(sin_correctedPhi*sin_correctedPsi - sin_correctedTheta*cos_correctedPhi*cos_correctedPsi) + ay*(sin_correctedPhi*cos_correctedPsi + sin_correctedPsi*sin_correctedTheta*cos_correctedPhi) + az*cos_correctedPhi*cos_correctedTheta, ay*sin_correctedPhi*sin_correctedPsi*cos_correctedTheta - ax*sin_correctedPhi*cos_correctedPsi*cos_correctedTheta - az*sin_correctedPhi*sin_correctedTheta, ay*(sin_correctedPsi*cos_correctedPhi + sin_correctedPhi*sin_correctedTheta*cos_correctedPsi) - ax*(cos_correctedPhi*cos_correctedPsi - sin_correctedPhi*sin_correctedPsi*sin_correctedTheta), 0, 1, 0, 0, 0, 0, - ax*(sin_correctedPhi*sin_correctedPsi - sin_correctedTheta*cos_correctedPhi*cos_correctedPsi) - ay*(sin_correctedPhi*cos_correctedPsi + sin_correctedPsi*sin_correctedTheta*cos_correctedPhi) - az*cos_correctedPhi*cos_correctedTheta, az*sin_correctedPhi*sin_correctedTheta + ax*sin_correctedPhi*cos_correctedPsi*cos_correctedTheta - ay*sin_correctedPhi*sin_correctedPsi*cos_correctedTheta, ax*(cos_correctedPhi*cos_correctedPsi - sin_correctedPhi*sin_correctedPsi*sin_correctedTheta) - ay*(sin_correctedPsi*cos_correctedPhi + sin_correctedPhi*sin_correctedTheta*cos_correctedPsi),
                         az*sin_correctedPhi*cos_correctedTheta - ay*(cos_correctedPhi*cos_correctedPsi - sin_correctedPhi*sin_correctedPsi*sin_correctedTheta) - ax*(sin_correctedPsi*cos_correctedPhi + sin_correctedPhi*sin_correctedTheta*cos_correctedPsi), az*sin_correctedTheta*cos_correctedPhi + ax*cos_correctedPhi*cos_correctedPsi*cos_correctedTheta - ay*sin_correctedPsi*cos_correctedPhi*cos_correctedTheta, ay*(sin_correctedPhi*sin_correctedPsi - sin_correctedTheta*cos_correctedPhi*cos_correctedPsi) - ax*(sin_correctedPhi*cos_correctedPsi + sin_correctedPsi*sin_correctedTheta*cos_correctedPhi), 0, 0, 1, 0, 0, 0,   ax*(sin_correctedPsi*cos_correctedPhi + sin_correctedPhi*sin_correctedTheta*cos_correctedPsi) + ay*(cos_correctedPhi*cos_correctedPsi - sin_correctedPhi*sin_correctedPsi*sin_correctedTheta) - az*sin_correctedPhi*cos_correctedTheta, ay*sin_correctedPsi*cos_correctedPhi*cos_correctedTheta - ax*cos_correctedPhi*cos_correctedPsi*cos_correctedTheta - az*sin_correctedTheta*cos_correctedPhi, ax*(sin_correctedPhi*cos_correctedPsi + sin_correctedPsi*sin_correctedTheta*cos_correctedPhi) - ay*(sin_correctedPhi*sin_correctedPsi - sin_correctedTheta*cos_correctedPhi*cos_correctedPsi),
                                                                                                                                                                                                                                                        0,                                                 hx*sin_correctedTheta*cos_correctedPsi - hz*cos_correctedTheta - hy*sin_correctedPsi*sin_correctedTheta,                                                                                                         hy*cos_correctedPsi*cos_correctedTheta + hx*sin_correctedPsi*cos_correctedTheta, 0, 0, 0, 1, 0, 0,                                                                                                                                                                                                                                  0,                                                 hz*cos_correctedTheta - hx*sin_correctedTheta*cos_correctedPsi + hy*sin_correctedPsi*sin_correctedTheta,                                                                                                       - hy*cos_correctedPsi*cos_correctedTheta - hx*sin_correctedPsi*cos_correctedTheta,
                         hx*(sin_correctedPhi*sin_correctedPsi - sin_correctedTheta*cos_correctedPhi*cos_correctedPsi) + hy*(sin_correctedPhi*cos_correctedPsi + sin_correctedPsi*sin_correctedTheta*cos_correctedPhi) + hz*cos_correctedPhi*cos_correctedTheta, hy*sin_correctedPhi*sin_correctedPsi*cos_correctedTheta - hx*sin_correctedPhi*cos_correctedPsi*cos_correctedTheta - hz*sin_correctedPhi*sin_correctedTheta, hy*(sin_correctedPsi*cos_correctedPhi + sin_correctedPhi*sin_correctedTheta*cos_correctedPsi) - hx*(cos_correctedPhi*cos_correctedPsi - sin_correctedPhi*sin_correctedPsi*sin_correctedTheta), 0, 0, 0, 0, 1, 0, - hx*(sin_correctedPhi*sin_correctedPsi - sin_correctedTheta*cos_correctedPhi*cos_correctedPsi) - hy*(sin_correctedPhi*cos_correctedPsi + sin_correctedPsi*sin_correctedTheta*cos_correctedPhi) - hz*cos_correctedPhi*cos_correctedTheta, hz*sin_correctedPhi*sin_correctedTheta + hx*sin_correctedPhi*cos_correctedPsi*cos_correctedTheta - hy*sin_correctedPhi*sin_correctedPsi*cos_correctedTheta, hx*(cos_correctedPhi*cos_correctedPsi - sin_correctedPhi*sin_correctedPsi*sin_correctedTheta) - hy*(sin_correctedPsi*cos_correctedPhi + sin_correctedPhi*sin_correctedTheta*cos_correctedPsi),
                         hz*sin_correctedPhi*cos_correctedTheta - hy*(cos_correctedPhi*cos_correctedPsi - sin_correctedPhi*sin_correctedPsi*sin_correctedTheta) - hx*(sin_correctedPsi*cos_correctedPhi + sin_correctedPhi*sin_correctedTheta*cos_correctedPsi), hz*sin_correctedTheta*cos_correctedPhi + hx*cos_correctedPhi*cos_correctedPsi*cos_correctedTheta - hy*sin_correctedPsi*cos_correctedPhi*cos_correctedTheta, hy*(sin_correctedPhi*sin_correctedPsi - sin_correctedTheta*cos_correctedPhi*cos_correctedPsi) - hx*(sin_correctedPhi*cos_correctedPsi + sin_correctedPsi*sin_correctedTheta*cos_correctedPhi), 0, 0, 0, 0, 0, 1,   hx*(sin_correctedPsi*cos_correctedPhi + sin_correctedPhi*sin_correctedTheta*cos_correctedPsi) + hy*(cos_correctedPhi*cos_correctedPsi - sin_correctedPhi*sin_correctedPsi*sin_correctedTheta) - hz*sin_correctedPhi*cos_correctedTheta, hy*sin_correctedPsi*cos_correctedPhi*cos_correctedTheta - hx*cos_correctedPhi*cos_correctedPsi*cos_correctedTheta - hz*sin_correctedTheta*cos_correctedPhi, hx*(sin_correctedPhi*cos_correctedPsi + sin_correctedPsi*sin_correctedTheta*cos_correctedPhi) - hy*(sin_correctedPhi*sin_correctedPsi - sin_correctedTheta*cos_correctedPhi*cos_correctedPsi),
                                                                                                                                                                                                                                                        1,                                                                                                                                                        0,                                                                                                                                                                                       0, 0, 0, 0, 0, 0, 0,                                                                                                                                                                                                                                 -1,                                                                                                                                                        0,                                                                                                                                                                                       0,
                                                                                                                                                                                                                                                        0,                                                                                                                                                        1,                                                                                                                                                                                       0, 0, 0, 0, 0, 0, 0,                                                                                                                                                                                                                                  0,                                                                                                                                                       -1,                                                                                                                                                                                       0,
                                                                                                                                                                                                                                                        0,                                                                                                                                                        0,                                                                                                                                                                                       1, 0, 0, 0, 0, 0, 0,                                                                                                                                                                                                                                  0,                                                                                                                                                        0,                                                                                                                                                                                      -1};

    arm_mat_init_f32(&H, a, n, H_f32);

	/* Matriz Jacobiana transposta para cálculo da confiabilidade do erro . */
    float Ht_f32[n*a];
    arm_mat_init_f32(&Ht, n, a, Ht_f32);

    arm_mat_trans_f32(&H, &Ht);

	//Matriz de variâncias
	float Racel = buffer_filtro->R_acel;
	float Rmag = buffer_filtro->R_mag;	 //Variância inicial do magnetômetro.
    float Rangles = buffer_filtro->R_angles;

    float acelModulus = getVectorModulus(medida_accel, 3);
//    float magModulus = getVectorModulus(medida_mag, 3);
//    float magInitialModulus = getVectorModulus(buffer_filtro->MagInicial, 3);

    //Racel += 100*fabsf(1 - acelModulus);
    //Rmag += 1*fabs(magInitialModulus - magModulus);


    float R_f32[a*a] = {(Racel), 0, 0, 0, 0, 0, 0, 0, 0,
                        0, (Racel), 0, 0, 0, 0, 0, 0, 0,
                        0, 0, (Racel), 0, 0, 0, 0, 0, 0,
                        0, 0, 0, (Rmag),  0, 0, 0, 0, 0,
                        0, 0, 0, 0, (Rmag),	 0, 0, 0, 0,
                        0, 0, 0, 0, 0, (Rmag), 0, 0, 0,
                        0, 0, 0, 0, 0, 0, (Rangles), 0, 0,
                        0, 0, 0, 0, 0, 0, 0, (Rangles), 0,
                        0, 0, 0, 0, 0, 0, 0, 0, (Rangles)};

    arm_mat_init_f32(&R, a, a, R_f32);

	//Cálculos do filtro de Kalman

	//S = H*P*H' + R
    if(arm_mat_mult_f32(&H, &P, &temp_calc_an_0) != ARM_MATH_SUCCESS)
        while(1);

    if(arm_mat_mult_f32(&temp_calc_an_0, &Ht, &temp_calc_aa_0) != ARM_MATH_SUCCESS)
        while(1);

    if(arm_mat_add_f32(&temp_calc_aa_0, &R, &S) != ARM_MATH_SUCCESS)
        while(1);

    //Sinv = inv(S);
    //if(arm_mat_inverse_f32(&S, &Sinv) == ARM_MATH_SINGULAR)
    //    while(1);
    arm_mat_inverse_f32(&S, &Sinv);

    //Kk = P*Ht*S^(-1)
		//P*Ht
    if(arm_mat_mult_f32(&P, &Ht, &temp_calc_na_0) != ARM_MATH_SUCCESS)
        while(1);

    if(arm_mat_mult_f32(&temp_calc_na_0, &Sinv, &K) != ARM_MATH_SUCCESS)
        while(1);
	
    //temp_calc_n11 = Kk*y
    if(arm_mat_mult_f32(&K, &ykMatrix, &temp_calc_n1_0) != ARM_MATH_SUCCESS)
        while(1);

    //X = X + temp_calc_n1_1;
    if(arm_mat_add_f32(&X, &temp_calc_n1_0, &temp_calc_n1_1) != ARM_MATH_SUCCESS)
        while(1);

    arm_copy_f32(temp_calc_n1_1_f32, X_f32, n);

	//P = (I-K*H)*P
	
	//Matriz identidade para atualização da matriz P à posteriori.
    float I_f32[n*n] = {	1,0,0,0,0,0,0,0,0,0,0,0,
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

    arm_mat_init_f32(&I, n, n, I_f32);


    if(arm_mat_mult_f32(&K, &H, &temp_calc_nn_0) != ARM_MATH_SUCCESS)
        while(1);

    if(arm_mat_sub_f32(&I, &temp_calc_nn_0, &temp_calc_nn_1) != ARM_MATH_SUCCESS)
        while(1);

    if(arm_mat_mult_f32(&temp_calc_nn_1, &P, &temp_calc_nn_0) != ARM_MATH_SUCCESS)
        while(1);

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
    arm_mat_set_element(rotationMatrix, 0, 0, temp);

    temp = (2*q0*q3 + 2*q1*q2);
    arm_mat_set_element(rotationMatrix, 0, 1, temp);

    temp = (2*q1*q3 - 2*q0*q2);
    arm_mat_set_element(rotationMatrix, 0, 2, temp);

    temp = (2*q1*q2 - 2*q0*q3);
    arm_mat_set_element(rotationMatrix, 1, 0, temp);

    temp = (- 2*powf(q1,2) - 2*powf(q3,2) + 1);
    arm_mat_set_element(rotationMatrix, 1, 1, temp);

    temp = (2*q0*q1 + 2*q2*q3);
    arm_mat_set_element(rotationMatrix, 1, 2, temp);

    temp = (2*q0*q2 + 2*q1*q3);
    arm_mat_set_element(rotationMatrix, 2, 0, temp);

    temp = (2*q2*q3 - 2*q0*q1);
    arm_mat_set_element(rotationMatrix, 2, 1, temp);

    temp = (- 2*powf(q1,2) - 2*powf(q2,2) + 1);
    arm_mat_set_element(rotationMatrix, 2, 2, temp);
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

void getABCRotMatFromEulerAngles(float phi, float theta, float psi, arm_matrix_instance_f32 *rotationMatrix)
{
    float32_t tempRotationVector[9] = {       f_cos(psi)*f_cos(theta),                              f_cos(theta)*f_sin(psi),         -f_sin(theta),
                                              f_cos(psi)*f_sin(phi)*f_sin(theta) - f_cos(phi)*f_sin(psi), f_cos(phi)*f_cos(psi) + f_sin(phi)*f_sin(psi)*f_sin(theta), f_cos(theta)*f_sin(phi),
                                              f_sin(phi)*f_sin(psi) + f_cos(phi)*f_cos(psi)*f_sin(theta), f_cos(phi)*f_sin(psi)*f_sin(theta) - f_cos(psi)*f_sin(phi), f_cos(phi)*f_cos(theta)};

    arm_copy_f32(tempRotationVector, rotationMatrix->pData, 9);
}

void getRotMatOrthogonality(arm_matrix_instance_f32 *output, arm_matrix_instance_f32 *rotationMatrix)
{
    float temp = 0.0;

    float a11, a12, a13;
    float a21, a22, a23;
    float a31, a32, a33;

    a11 = arm_mat_get_element(rotationMatrix, 0, 0);
    a12 = arm_mat_get_element(rotationMatrix, 0, 1);
    a13 = arm_mat_get_element(rotationMatrix, 0, 2);
    a21 = arm_mat_get_element(rotationMatrix, 1, 0);
    a22 = arm_mat_get_element(rotationMatrix, 1, 1);
    a23 = arm_mat_get_element(rotationMatrix, 1, 2);
    a31 = arm_mat_get_element(rotationMatrix, 2, 0);
    a32 = arm_mat_get_element(rotationMatrix, 2, 1);
    a33 = arm_mat_get_element(rotationMatrix, 2, 2);

    //Produto escalar (Dot Product) entre a primeira e a segunda coluna da matriz de rotação.
    temp = a12*(a11) + a22*(a21) + a32*(a31);
    arm_mat_set_element(output, 0, 0, temp);

    //Produto escalar entre a primeira a terceira coluna da matriz de rotação
    temp = a13*(a11) + a23*(a21) + a33*(a31);
    arm_mat_set_element(output, 1, 0, temp);

    //Produto vetorial entre a primeira e a segunda coluna para geraro vetor normal à ambos e depois produto escalar entre este novo vetor e a terceira coluna da matriz de rotação.
    temp = a33*((a11)*(a22) - (a12)*(a21)) - a23*((a11)*(a32) - (a12)*(a31)) + a13*((a21)*(a32) - (a22)*(a31));
    arm_mat_set_element(output, 2, 0, temp);
}

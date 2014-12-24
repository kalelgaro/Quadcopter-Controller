/* * kalman_filter.h
 *
 *  Created on: Sep 8, 2013
 *      Author: Bruno
 */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "stm32f4_discovery.h"
#include "arm_math.h"
#include "tratamento_sinal.h"


	#define bias 1
	#define angle 0

    #define NUMBER_OF_STATES 13
    #define NUMBER_OF_CONTROL_INPUTS 0
    #define NUMBER_OF_MEASURES 6

    #define n NUMBER_OF_STATES
    #define b NUMBER_OF_MEASURES
    #define a NUMBER_OF_MEASURES

	typedef struct
	{
        float ultimo_estado[n];     //Salva o estado da ultima iteração do filtro Xk-1|k-1
        float P[n*n];               //Matriz de Covariância do erro da ultima iterção (Pk|k-1)
        float Qk[n*n];              //Matriz de covariâncias de processo salvas ao final de cada iteração.
        float Q_quat;
        float Q_bias_acel;
        float Q_bias_mag;
        float Q_bias_gyro;
		float R_acel;
		float R_mag;
		float dt;
        float AcelInicial[3];
		float MagInicial[3];
	} kalman_filter_state;

	void kalman_filter(kalman_filter_state *buffer_filtro, float medida_gyro[], float medida_accel[], float medida_mag[], uint16_t estado_motores);

    void getRotMatFromQuaternion(float quaternion[4], arm_matrix_instance_f32 *rotationMatrix);
    void arm_mat_set_element(arm_matrix_instance_f32 *entrada, uint16_t row, uint16_t column, float newValue);
    float arm_mat_get_element(const arm_matrix_instance_f32 *entrada, uint16_t row, uint16_t column);


#endif /* KALMAN_FILTER_H_ */

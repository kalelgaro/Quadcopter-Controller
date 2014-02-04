/*
 * kalman_filter.h
 *
 *  Created on: Sep 8, 2013
 *      Author: Bruno
 */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

	#define bias 1
	#define angle 0

	typedef struct
	{
		float ultimo_estado[3]; 				//Salva o estado da ultima iteração do filtro Xk-1|k-1

		float P[9];							//Matriz de Covariância do erro da ultima iterção (Pk|k-1)

		float Q;

		float R;

		float dt;

	} kalman_filter_state;


	float kalman_filter(kalman_filter_state *buffer_filtro, float medida_gyro[], float medida_accel[]);

#endif /* KALMAN_FILTER_H_ */

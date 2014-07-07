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
		float ultimo_estado[9]; 				//Salva o estado da ultima iteração do filtro Xk-1|k-1

		float P[81];							//Matriz de Covariância do erro da ultima iterção (Pk|k-1)

		float Q_ang;

		float Q_bias_mag;

		float Q_bias_gyro;

		float R_acel;

		float R_mag;

		float R_det;

		float R_orth;

		float dt;

		float MagInicial[3];

	} kalman_filter_state;


	void kalman_filter(kalman_filter_state *buffer_filtro, float medida_gyro[], float medida_accel[], float medida_mag[], uint16_t estado_motores);

#endif /* KALMAN_FILTER_H_ */

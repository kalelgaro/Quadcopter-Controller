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
		float ultimo_estado[6]; 				//Salva o estado da ultima itera��o do filtro Xk-1|k-1

		float P[36];							//Matriz de Covari�ncia do erro da ultima iter��o (Pk|k-1)

		float Q_acel;

		float Q_mag;

		float Q_bias;

		float R_acel;

		float R_mag;

		float dt;

		float MagInicial[3];

	} kalman_filter_state;


	void kalman_filter(kalman_filter_state *buffer_filtro, float medida_gyro[], float medida_accel[], float medida_mag[], uint16_t estado_motores);

#endif /* KALMAN_FILTER_H_ */

/*
 * controle_motores.h
 *
 *  Created on: Apr 26, 2013
 *      Author: Bruno
 */

#ifndef CONTROLE_MOTORES_H_
#define CONTROLE_MOTORES_H_

	void configurar_PWM();
	void ajustar_velocidade(uint8_t motor, uint16_t velocidade);
	void iniciar_ESC();
	uint16_t equilibrar_esc(float);

#endif /* CONTROLE_MOTORES_H_ */

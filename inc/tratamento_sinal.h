/*
 * tratamento_sinal.h
 *
 *  Created on: Mar 24, 2013
 *      Author: Bruno
 */

#ifndef TRATAMENTO_SINAL_H_
#define TRATAMENTO_SINAL_H_

//	float filtro_passa_baixa(float leitura, float *buffer_filtro, float *buffer_leitura);
	void acel_2_angulos(float acel_x, float acel_y, float acel_z, float angulos[2]);	
	
	void inserir_ajuster_motores(float pitch_pid, float roll_pid, float yaw_pid, uint16_t rotacao_constante);
	float filtro_correlacao(float constant, float leitura_giro, float leitura_acell, float buffer, float dt);
	float filtro_correlacao_2nd(float constant, float leitura_giro, float leitura_acell, float buffer[2], float dt);
	float filtro_fir(float leitura, float buffer_leitura[], uint16_t numero_coeficientes, float buffer_coeficientes[]);
	float media_rotativa(float nova_leitura, float buffer_leituras[], uint16_t numero_pontos);
	float arredondar_float(float entrada, uint8_t numero_entrada);
	void normalizar_vetor_R3(float vetor[3]);

	typedef struct 
	{
		float buffer_erro;
		float buffer_vel_angular;
		float buffer_saida;
	} PID_state;

	double calcular_PID(float entrada, float kp, float ki, float kd, double *buffer_pid, float dt);

#endif /* TRATAMENTO_SINAL_H_ */

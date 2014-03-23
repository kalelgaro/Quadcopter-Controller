#define ARM_MATH_CM4
  
#ifndef __FPU_USED
  #define __FPU_USED 1
#endif

#ifndef __FPU_PRESENT
  #define __FPU_PRESENT 1
#endif

/*
 * tratamento_sinal.c
 *
 *  Created on: Mar 24, 2013
 *      Author: Bruno
 */

#include <stm32f4xx_conf.h>
#include "arm_math.h"
#include "controle_motores.h"
#include "math.h"
#include "tratamento_sinal.h"

#define fator_correcao 0.35							//1/(2*sqrt(2)) -> para correção do valor aplicado aos motores -> Motor equivalente.

uint16_t buffer_velocidade_motores[4] = {0, 0, 0, 0};

float media_rotativa(float nova_leitura, float buffer_leituras[], uint16_t numero_pontos)
{
	uint16_t counter = 0;

	float temp = 0;

	for(counter = (numero_pontos-1); counter > 0; counter--)
	{
		buffer_leituras[counter] = buffer_leituras[counter-1];
	}

	buffer_leituras[0] = nova_leitura;

	for(counter = 0; counter < numero_pontos; counter++)
	{
		temp = temp + buffer_leituras[counter];
	}

	return temp/((float)numero_pontos);
}

float filtro_correlacao(float constant, float leitura_giro, float leitura_acell, float buffer, float dt)
{
	return constant*(buffer+leitura_giro*dt) + (1-constant)*(leitura_acell);
}

float filtro_correlacao_2nd(float constant, float leitura_giro, float leitura_acell, float buffer[2], float dt)
{

	float temp = 0;

	temp = leitura_acell - buffer[0];
	buffer[1] = buffer[1] + temp*constant*constant*dt;
	temp = leitura_giro + buffer[1] + temp*2*constant;

	buffer[0] = temp*dt + buffer[0];

	return buffer[0];
}

void acel_2_angulos(float acel_x, float acel_y, float acel_z, float angulos[2])
{
	//Conversão de inclinação com base no artigo
		/*Implementing a Tilt-Compensated eCompass using Accelerometer and Magnetometer Sensors*/
	float32_t temp;
	angulos[0] = (float)atan2(acel_y,acel_z);

	temp = acel_y*sin(angulos[0])+acel_z*cos(angulos[0]); 					
	
	angulos[1] = (float)(atan(-acel_x/temp));

	angulos[1] = angulos[1]*57.3;					//180/pi ~= 57.3 -> Conversão rad p/ graus. (57.295 (180/pi))
	angulos[0] = angulos[0]*57.3;					//180/pi ~= 57.3 -> Conversão rad p/ graus. (57.295 (180/pi))
}

///*Função que calcula o PID discreto para o sinal de erro */
///*Utiliza o giroscópio para cálculo do termo derivativo -> Versão de 23/11/2013*/
//
//double calcular_PID(float erro, float vel_angular , float kp, float ki, float kd, float dt, PID_state *estado_PID)
//{
//	float temp,temp1,temp2;
//
//	temp =  (erro - estado_PID->buffer_erro)*kp;
//	temp1 = (erro + estado_PID->buffer_erro)*ki*dt/2;
//	temp2 = (estado_PID->buffer_vel_angular - vel_angular)*kd;
//
//	temp = temp + temp1 + temp2 + (estado_PID->buffer_saida);
//
//	estado_PID->buffer_erro = erro;
//	estado_PID->buffer_vel_angular = vel_angular;
//	estado_PID->buffer_saida = temp;
//
//	return temp;
//}

/* - Back-up Função de cálculo PID padrão -> 23/11/2013*/
/*Nova versão implementa a velocidade angular com o giroscopio*/


double calcular_PID(float entrada, float kp, float ki, float kd, double *buffer_pid, float dt)
{
	// float derivacao;
	// float integracao;

	// integracao = buffer_pid[0] + (entrada*dt);				//buffer_pid[0] + leitura*dt -> Atualiza o integrador.
	// derivacao = (entrada-buffer_pid[1])/dt;


	// buffer_pid[1] = entrada;					//Xk-1 = Xk -> Atualiza para o próximo ciclo;
	// buffer_pid[0] = integracao;

	// return ((kp*entrada) + (kd*derivacao) + (ki*integracao));
	
	//buffer_pid[0] -> Resultado do pid anterior; y[n-1]
	//buffer_pid[1] -> Ultima leitura; x[n-1]
	//buffer_pid[2] -> Penultima leitura; x[n-2]

	// //entrada -> x[n]
	
	float temp;

	kd = kd/dt;
	ki = ki*dt/2;

	float A0 = kp + kd + ki;
	float A1 = -(kp + 2*kd) +ki;
	float A2 = kd;

	temp = buffer_pid[0] + A0*(entrada) + A1*buffer_pid[1] + A2*buffer_pid[2];

	buffer_pid[0] = temp;			 //y[n-1] = y[n] -> Próxima iteração
	buffer_pid[2] = buffer_pid[1];   //x[n-2] = x[n-1];
	buffer_pid[1] = entrada;		 //x[n-1] = x[n];

	return temp;

	//PID do simulink - Integral Trapezoidal e derivada Fowkward Euler
	//   y(z)   = Kp +Ki*dt*(z+1) + Kd*order
	//   err(z)           2 (z-1)   1+order*dt*1/(z-1)
	//

	// double temp;
	// float gama = ki*dt/2;
	// float beta = kd*order;
	// float alpha = order*dt;

	// temp = entrada*(kp+gama+beta) + buffer_pid[0]*(kp*(alpha-2)+gama*alpha-2*beta)+buffer_pid[1]*((1-alpha)*(kp-gama)+beta);
	// temp += buffer_pid[2]*(2-alpha) + buffer_pid[3]*(alpha-1);
	
	// buffer_pid[1] = buffer_pid[0];			// u[n-2] = u[n-1]
	// buffer_pid[0] = entrada;				// u[n-1] = u[n]

	// buffer_pid[3] = buffer_pid[2];			// y[n-2] = y[n-1]
	// buffer_pid[2] = temp;					// y[n-1] = y[n]

	//return temp;
}


void inserir_ajuster_motores(float pitch_pid, float roll_pid, float yaw_pid, uint16_t rotacao_constante)
{
//	static float fator_correcao = (1/(2*sqrt(2)));

	float32_t velocidade_m1;
	float32_t velocidade_m2;
	float32_t velocidade_m3;
	float32_t velocidade_m4;

	float delta_m1 = 0;
	float delta_m2 = 0;
	float delta_m3 = 0;
	float delta_m4 = 0;

	delta_m1 = (fator_correcao*(+pitch_pid - roll_pid));
	delta_m2 = (fator_correcao*(+pitch_pid + roll_pid));
	delta_m3 = (fator_correcao*(-pitch_pid + roll_pid));
	delta_m4 = (fator_correcao*(-pitch_pid - roll_pid));

	velocidade_m1 = rotacao_constante + delta_m1 - yaw_pid;
	velocidade_m2 = rotacao_constante + delta_m2 + yaw_pid;
	velocidade_m3 = rotacao_constante + delta_m3 - yaw_pid;
	velocidade_m4 = rotacao_constante + delta_m4 + yaw_pid;

	if(velocidade_m1 < 0)
		velocidade_m1 = 0;

	if(velocidade_m2 < 0)
		velocidade_m2 = 0;

	if(velocidade_m3 < 0)
		velocidade_m3 = 0;

	if(velocidade_m4 < 0)
		velocidade_m4 = 0;

	velocidade_m1 = sqrt(velocidade_m1);
	velocidade_m2 = sqrt(velocidade_m2);
	velocidade_m3 = sqrt(velocidade_m3);
	velocidade_m4 = sqrt(velocidade_m4);

	//Ajuste da velocidae dos motores calculadas acima.
	ajustar_velocidade(4, (uint16_t)round((velocidade_m1)));
	ajustar_velocidade(8, (uint16_t)round((velocidade_m2)));
	ajustar_velocidade(1, (uint16_t)round((velocidade_m3)));
	ajustar_velocidade(2, (uint16_t)round((velocidade_m4)));
}

float filtro_fir(float leitura, float buffer_leitura[], uint16_t numero_coeficientes, float buffer_coeficientes[])
{
	float temp = 0.0;
	uint8_t counter = 1;

	temp = leitura*buffer_coeficientes[0];

	for(counter = 0; counter < (numero_coeficientes-2); counter ++)
	{
		temp += (buffer_leitura[counter]*buffer_coeficientes[counter+1]);
	}

	for(counter = (numero_coeficientes-1); counter >= 1; counter--)
	{
		buffer_leitura[counter] = buffer_leitura[counter-1];
	}

	buffer_leitura[0] = leitura;

	return temp;
}


//----Função para arredondamento de pontos flutuantes.----//
//--- Bruno F. M. Callegaro - 10/09/2013---//
//--Arredonda com o número de casas decimais desejadas. Numero de casas decimais padrão são 8 casas--//

float arredondar_float(float entrada, uint8_t numero_entrada)
{
	uint8_t i,j = 0;

	double temp_calc = entrada*1000000000;

	for(i = 8-numero_entrada; i > 0; i--, j++)
	{
		temp_calc = round(temp_calc);
		temp_calc = temp_calc/10;
	}

	temp_calc = temp_calc/(pow(10,7-j));

	return temp_calc;
}

void normalizar_vetor_R3(float vetor[3])
{
	float modulo = pow(vetor[0],2) + pow(vetor[1],2) + pow(vetor[2],2);
	vetor[0] = vetor[0]/modulo;
	vetor[1] = vetor[1]/modulo;
	vetor[2] = vetor[2]/modulo;
}

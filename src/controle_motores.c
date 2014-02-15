#define ARM_MATH_CM4
  
#ifndef __FPU_USED
  #define __FPU_USED 1
#endif

#ifndef __FPU_PRESENT
  #define __FPU_PRESENT 1
#endif

/*
 * controle_motores.c
 *
 *  Created on: Apr 26, 2013
 *      Author: Bruno
 */
#include "stm32f4xx_it.h"
#include <stm32f4xx_conf.h>
#include "controle_motores.h"
#include "arm_math.h"

void iniciar_ESC()
{
	configurar_PWM();				//Pulsos utilizados para controle do ESC.
}

void configurar_PWM() //Controle de velocidade do ESC.
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);

	uint16_t PrescalerValue = 0;

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 100000) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 500;											//Periodo de 10 ms => 100/(100e3) ->
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);

	TIM4->CCR1 = 400;				//Pwm tem 1mS de pulso baixo -> 500 - 400 = 100*1/(100e3Hz)
	TIM4->CCR2 = 400;
	TIM4->CCR3 = 400;
	TIM4->CCR4 = 400;
	/* TIM3 enable counter */
	TIM_Cmd(TIM4, ENABLE);
}


//Ajusta o Duty Cicle dos pwms, e consecutivamente, a velocidade dos motores ligados nos ESCS

//0 - Velocidade mínima, e 100 velocidade máxima

void ajustar_velocidade(uint8_t motor, uint16_t velocidade) {

	if(velocidade > 100)
		velocidade = 100;

	//Saída no pwm é invertida, buffer com mosfet source comum, logo pulso é invertido.
	if((motor&0x01) != 0)
		TIM4->CCR1 = 400-velocidade;

	if((motor&0x02) != 0)
		TIM4->CCR2 = 400-velocidade;

	if((motor&0x04) != 0)
		TIM4->CCR3 = 400-velocidade;

	if((motor&0x08) != 0)
		TIM4->CCR4 = 400-velocidade;
}


//Função baseado na curva de respostas dos escs EMAX 18A e HK_ss_series 30A
//Corrige o valor inserido no ESC da HK de forma que a resposta deste , em regime permanente, seja igual aos escs Emax.
//O valor proveniente desta função é o necessário de forma que a velocidade no esc HK seja a mesma para a mesma entrada
// em um esc da Emax .
//Coeficientes retirados da função de "fitar" polinômios do MatLab com base nos testes de velocidade realizados.

uint16_t equilibrar_esc(float entrada)
{
	return pow(entrada,2)*((double)0.0104)+((float)0.3544*entrada)+(float)8.9854;
}

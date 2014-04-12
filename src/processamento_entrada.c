#include "stm32f4_discovery.h"

#include <processamento_entrada.h>

#define REF_ANGULAR_MAX

void configurar_timers_PWM_I(void)
{
	
  /* - Configuração do Timer 5 -> Input do controle para YAW - */

  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

	TIM_ICInitTypeDef  TIM_ICInitStructure;
  	
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);							//Liga o timer 2 ao barramento.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);							//Liga o timer 3 ao barramento.
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);							//Liga o timer 5 ao barramento.
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);							//Liga o timer 5 ao barramento.

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);							//PInos de input dos PWMS.

  uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 1000000)- 1;		//1000000 Contagens por segundo    //SystemcoreCLock/2 -> 84 Mhz

  /*Configuração de temporização básica do timer*/

  //Configurações dos timers de 32 bits - TIM5 e TIM2 - Clock do barramento de 84 MHz
  TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  	
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  //Configurações do timer de 16 bits - TIM3 - Clock do barramento de 84 MHz
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;	
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  //Configurações do timer de 16bits - TIM9 - Clock do barramento de 168 MHz
	PrescalerValue = (uint16_t) ((SystemCoreClock) / 1000000)- 1;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);  	

  /*Configuração do pino utilizado como entrada para a leitura do PWM.*/

 	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_0 | GPIO_Pin_7;				//PA1 ->Tim5_Ch2, PA0 -> Tim2_Ch2, PA7-> Tim3_Ch2, PA2 -> Tim9_Ch1
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;        //Pinos no modo de Função alternativa.
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   //Pinos com velocidade de 100 MHz.
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //Configuração de saída -> Push Pull
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;       //Pino com pull Up.
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  	
	/*ATiva a função alternativa do pino*/
  	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM9);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
  
  //Configurações dos pinos 2 e 6  	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //Pinos no modo de entrada.
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /*Configuração do interrupção.*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
	NVIC_Init(&NVIC_InitStructure);

  /*Configuração dos registradores de "capture" do timer 5*/
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

 	/*Configura a função de PWM Input */ 
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);
  	
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
  TIM_PWMIConfig(TIM5, &TIM_ICInitStructure);
  TIM_PWMIConfig(TIM9, &TIM_ICInitStructure);
  	
  /* Select the TIM5 Input Trigger: TI2FP2 */
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI1FP1);
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);
  TIM_SelectInputTrigger(TIM5, TIM_TS_TI2FP2);
  TIM_SelectInputTrigger(TIM9, TIM_TS_TI2FP2);

  /* Select the slave Mode: Reset Mode */
 	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);

 	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);

  TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM5,TIM_MasterSlaveMode_Enable);

  TIM_SelectSlaveMode(TIM9, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM9,TIM_MasterSlaveMode_Enable);

  /* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM5, ENABLE);
  TIM_Cmd(TIM9, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
  TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);
  TIM_ITConfig(TIM9, TIM_IT_CC2, ENABLE);
  	
}
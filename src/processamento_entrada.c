#include "stm32f4xx.h"
#include <stm32f4xx_conf.h>
#include <processamento_entrada.h>

//Constante -> Deslocamento angular absoluto máximo de 15 graus
#define REF_ANGULAR_MAX 15

//Constante -> Valor máximo inserido na rotaçaõ constante
#define MAX_ROT_CONSTANTE 2150

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

  TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  	
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;	
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	PrescalerValue = (uint16_t) ((SystemCoreClock) / 1000000)- 1;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);  	

  /*Configuração do pino utilizado como entrada para a leitura do PWM.*/

 	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_0 | GPIO_Pin_7;				//PA1 ->Tim5_Ch2, PA0 -> Tim2_Ch2, PA7-> Tim3_Ch2, PA2 -> Tim9_Ch1
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  	
	/*ATiva a função alternativa do pino*/
  	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM9);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
  	

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

void tratar_referencias(canais_entrada entradas, referencias *saida_referencias) {


  if(entradas.ch2 >= -1.1 && entradas.ch2 <= 1.1)
    saida_referencias->Ref_pitch = (entradas.ch2*REF_ANGULAR_MAX);
  else
    saida_referencias->Ref_pitch = 0;

  if(entradas.ch1 >= -1.1 && entradas.ch1 <= 1.1)
    saida_referencias->Ref_roll =  (entradas.ch1*REF_ANGULAR_MAX);
  else
    saida_referencias->Ref_roll = 0;

  if(entradas.ch4 >= -1.1 && entradas.ch4 <= 1.1)
    saida_referencias->Ref_yaw = (entradas.ch4 *REF_ANGULAR_MAX);
  else
    saida_referencias->Ref_yaw = 0;
  

  //Checa a posição da alavanca de aceleração -> 
    //Entre 0 e 0.15 e o controlador esta desligado -> Mantém o controlador desligado -> Segurança de inicialização.
    //Entre 0.15 e 2.2 - Controlador ligado e insere o valor mulitplicado por 850 no motor


  if(entradas.ch3 < 0.15)
  {
    saida_referencias->Rotacao_constante = 0;

  }else if((entradas.ch3 > 0.15) && (entradas.ch3 <= 2.2))
  {
    saida_referencias->Rotacao_constante = (entradas.ch3*(MAX_ROT_CONSTANTE/2)); //Insere o valor de rotação dos motores entre 146,25 e 2145

  }
}
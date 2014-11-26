/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"

#include "array_functions.h"
#include "funcoes_spi.h"
#include "string.h"
#include "processo_controle.h"
#include "stm32f4xx_it.h"
#include "tratamento_sinal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* - Defines para facilitar indexação dos vetores de dados - */

#define medias_controle 2

/* Private variables ---------------------------------------------------------*/

uint16_t variavel_delay_100ms = 100;

float dc_ch_1;
float dc_ch_2;
float dc_ch_3;
float dc_ch_4;

float medio_ch1 = 0.0;
float medio_ch2 = 0.0;
float medio_ch3 = 0.0;
float medio_ch4 = 0.0;


float buffer_ch1[medias_controle];
float buffer_ch2[medias_controle];
float buffer_ch3[medias_controle];
float buffer_ch4[medias_controle];

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

///**
//  * @brief   This function handles NMI exception.
//  * @param  None
//  * @retval None
//  */

//void NMI_Handler(void)
//{
//}
//
///**
//  * @brief  This function handles Hard Fault exception.
//  * @param  None
//  * @retval None
//  */
void HardFault_Handler(void)
{
 /* Go to infinite loop when Hard Fault exception occurs */
	uint32_t delay_hard_fault;
	while (1)
	{
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		
		delay_hard_fault = 0xFFFFFFFF;
		while(delay_hard_fault--);
		
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		GPIO_SetBits(GPIOD, GPIO_Pin_13);

		delay_hard_fault = 0xFFFFFFFF;
		while(delay_hard_fault--);

		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
		GPIO_SetBits(GPIOD, GPIO_Pin_14);

		delay_hard_fault = 0xFFFFFFFF;
		while(delay_hard_fault--);

		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		GPIO_SetBits(GPIOD, GPIO_Pin_15);

		delay_hard_fault = 0xFFFFFFFF;
		while(delay_hard_fault--);

		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
	
	}
}

//
///**
//  * @brief  This function handles Memory Manage exception.
//  * @param  None
//  * @retval None
//  */
//void MemManage_Handler(void)
//{
//  /* Go to infinite loop when Memory Manage exception occurs */
//  while (1)
//  {
//  }
//}
//
///**
//  * @brief  This function handles Bus Fault exception.
//  * @param  None
//  * @retval None
//  */
//void BusFault_Handler(void)
//{
//  /* Go to infinite loop when Bus Fault exception occurs */
//  while (1)
//  {
//  }
//}
//
///**
//  * @brief  This function handles Usage Fault exception.
//  * @param  None
//  * @retval None
//  */
//void UsageFault_Handler(void)
//{
//  /* Go to infinite loop when Usage Fault exception occurs */
//  while (1)
//  {
//  }
//}
//
///**
//  * @brief  This function handles SVCall exception.
//  * @param  None
//  * @retval None
//  */
//void SVC_Handler(void)
//{
//}
//
///**
//  * @brief  This function handles Debug Monitor exception.
//  * @param  None
//  * @retval None
//  */
//void DebugMon_Handler(void)
//{
//}
//
///**
//  * @brief  This function handles PendSVC exception.
//  * @param  None
//  * @retval None
//  */
//void PendSV_Handler(void)
//{
//}
//

/*Rotina de interrupção do Timer5 -> Interrupção para medição do duty cicle do PWM */
void TIM5_IRQHandler(void)
{
	uint32_t IC2Value = TIM_GetCapture2(TIM5);

	if (IC2Value != 0)
  	{
  		
  		dc_ch_1 = ((float)(((float)TIM_GetCapture1(TIM5))-1480)/570);

      medio_ch1 = media_rotativa(dc_ch_1, buffer_ch1, medias_controle);
  	}
  	/* Clear TIM5 Capture compare interrupt pending bit */
	TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);
}

void TIM3_IRQHandler(void)
{
	uint32_t IC2Value = TIM_GetCapture2(TIM3);

	if (IC2Value != 0)
  	{
  		dc_ch_2 = ((float)(((float)TIM_GetCapture1(TIM3))-1480)/570);

      medio_ch2 = media_rotativa(dc_ch_2, buffer_ch2, medias_controle);
  	}
  	/* Clear TIM5 Capture compare interrupt pending bit */
  	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
	uint32_t IC2Value = TIM_GetCapture2(TIM9);

	if (IC2Value != 0)
  	{
      dc_ch_3 = ((float)(((float)TIM_GetCapture1(TIM9))-1480)/570+1);

      medio_ch3 = media_rotativa(dc_ch_3, buffer_ch3, medias_controle);
  	}
  	/* Clear TIM5 Capture compare interrupt pending bit */
	TIM_ClearITPendingBit(TIM9, TIM_IT_CC2);
}

void TIM2_IRQHandler(void)
{
	uint32_t IC2Value = TIM_GetCapture1(TIM2);

	if (IC2Value != 0)
  	{
  		dc_ch_4 = ((float)(((float)TIM_GetCapture2(TIM2))-1480)/570);

      medio_ch4 = media_rotativa(dc_ch_4, buffer_ch4, medias_controle);
  	}
  	/* Clear TIM5 Capture compare interrupt pending bit */
  	TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
}


/*Rotina de interrupçaõ do Timer 6 -> Interrupção à cada 1,25mS para aquisição e controle*/

void TIM6_DAC_IRQHandler(void)
{
  //Checa se a intrrupção disparada foi resultado do overflow do timer 6.
  if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);		//clear interrupt and start counting again to get precise freq

    //Setar referência dos controladores (Pitch, Yaw e Roll) e da rotação constante do motor, respectivamente.
    setar_referencia(medio_ch2, medio_ch1, medio_ch4, medio_ch3);
    
    //Aquisição,processamento e controle utilizando os sensores.
    processo_controle();              

  }
  
}


//Timer de erro da recepção.

void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
    {
    	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);		//clear interrupt and start counting again to get precise freq

    	GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
    }
}


/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */


//Interrupção de overflow do timer de tempo real
    //Overflow à cada 100uS
void SysTick_Handler(void)
{
	decrementar_var_delay();

	if(variavel_delay_100ms != 0)
		variavel_delay_100ms--;
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


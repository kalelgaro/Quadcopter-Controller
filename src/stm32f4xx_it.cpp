/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "stm32f4xx_it.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* - Defines para facilitar indexação dos vetores de dados - */

/* Private variables ---------------------------------------------------------*/

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

///*Rotina de interrupção do Timer5 -> Interrupção para medição do duty cicle do PWM */
//void TIM5_IRQHandler(void)
//{
//	TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);
//}

//void TIM3_IRQHandler(void)
//{
//  	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
//}

//void TIM1_BRK_TIM9_IRQHandler(void)
//{
//	TIM_ClearITPendingBit(TIM9, TIM_IT_CC2);
//}

//void TIM2_IRQHandler(void)
//{
//  	TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
//}


///*Rotina de interrupçaõ do Timer 6 -> Interrupção �  cada 1,25mS para aquisição e controle*/

//void TIM6_DAC_IRQHandler(void)
//{
//}


////Timer de erro da recepção.

//void TIM7_IRQHandler(void)
//{
//}


/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

//Interrupção de overflow do timer de tempo real
    //Overflow �  cada 100uS
void SysTick_Handler(void)
{
	//systickInterrupt();
	BaseTimeControl::systickCallback();
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


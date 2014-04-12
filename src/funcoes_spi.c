/*
 * funcoes_spi.c
 *
 *  Created on: Feb 22, 2013
 *      Author: Bruno
 */
#include "stm32f4_discovery.h"

#include "funcoes_spi.h"

static __IO uint32_t variavel_delay;

void iniciar_spi2()
{
	GPIO_InitTypeDef estrutura_conf_gpio;
	SPI_InitTypeDef SPI2_estrutura_conf;

	//Inicialização dos pinos utilizados pelo RF.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	//Liga o clock para os pinos no porto B -> 10 SCK , 12 CS, 14 CE
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	estrutura_conf_gpio.GPIO_Pin = GPIO_Pin_10; //Pin 10 CS
	estrutura_conf_gpio.GPIO_Mode = GPIO_Mode_OUT;
	estrutura_conf_gpio.GPIO_OType = GPIO_OType_PP;
	estrutura_conf_gpio.GPIO_Speed = GPIO_Speed_25MHz;
	estrutura_conf_gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &estrutura_conf_gpio);

	Desselecionar_RF;									//Coloca o CS em alto.

	estrutura_conf_gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;			//Pino SCK
	estrutura_conf_gpio.GPIO_Mode = GPIO_Mode_AF;
	estrutura_conf_gpio.GPIO_OType  = GPIO_OType_PP;
	estrutura_conf_gpio.GPIO_Speed = GPIO_Speed_25MHz;
	estrutura_conf_gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &estrutura_conf_gpio);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);


	//Inicialização do módulo SPI.
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);


	SPI2_estrutura_conf.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI2_estrutura_conf.SPI_Mode = SPI_Mode_Master;
	SPI2_estrutura_conf.SPI_DataSize = SPI_DataSize_8b;
	SPI2_estrutura_conf.SPI_CPOL =  SPI_CPOL_Low;
	SPI2_estrutura_conf.SPI_CPHA = SPI_CPHA_1Edge;
	SPI2_estrutura_conf.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
	SPI2_estrutura_conf.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI2_estrutura_conf.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI2, &SPI2_estrutura_conf);

	SPI_Cmd(SPI2, ENABLE);


}

void enviar_dados_SPI(SPI_TypeDef* SPIx, uint8_t buffer_envio[], uint8_t buffer_recepcao[], uint8_t tamanho)
{
	uint8_t index = 0;

	Selecionar_RF;
	for(; index < tamanho; index++)
	{
		SPIx->DR = buffer_envio[index];
		while( !(SPIx->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
		while( !(SPIx->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
		while( SPIx->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
		buffer_recepcao[index] = SPIx->DR;
	}
	Desselecionar_RF;

	delay(1);
}

void delay(uint32_t tempo_delay) //Delay em 10uS
{
	variavel_delay = tempo_delay;
	while(variavel_delay > 0);
}

void decrementar_var_delay()
{
	if(variavel_delay != 0)
		variavel_delay--;
}

/*
 * funcoes_spi.h
 *
 *  Created on: Feb 22, 2013
 *      Author: Bruno
 */

#ifndef FUNCOES_SPI_H_
#define FUNCOES_SPI_H_

#define PINO_CSN GPIO_Pin_10
#define Selecionar_RF GPIOB->BSRRH|=GPIO_Pin_10
#define Desselecionar_RF GPIOB->BSRRL|=GPIO_Pin_10

  void enviar_dados_SPI(SPI_TypeDef*, uint8_t[], uint8_t[]  , uint8_t);
  void iniciar_spi2();
  void delay(uint32_t);
  void decrementar_var_delay();

#endif /* FUNCOES_SPI_H_ */

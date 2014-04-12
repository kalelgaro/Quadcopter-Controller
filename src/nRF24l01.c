#include "stm32f4_discovery.h"


#include "array_functions.h"
#include "nRF24l01.h"
#include "funcoes_spi.h"


uint8_t pino_CE;

void iniciar_placa(SPI_TypeDef* SPIx) //Inicia à placa supondo que esta estava inicialmente desligada -> Modo standby1
{
	uint8_t buffer_conf[2];

	uint8_t mascara = 0x00;

	alterar_estado_CE(0);										//Garante que o CE esteja desligado.

	mascara = (PWR_UP);

	buffer_conf[0] = mascara;
	escrever_registrador(SPIx, CONFIG, buffer_conf,1);		//Modo Standby-1 -> Apenas power UP setado.

	delay(20);												//Delay de 2mS

	mascara = 0x00;
	mascara = (PWR_UP|CRCO|EN_CRC);
	buffer_conf[0] = mascara;
	escrever_registrador(SPIx, CONFIG, buffer_conf,1);		//Ativar o CRC check e o habilita como 2 bytes.

	return;
}


uint8_t retornar_status(SPI_TypeDef* SPIx)
{
	uint8_t buffer_spi_tx[1];					//Buffer que será utilizado para os dados recebidos.
	uint8_t buffer_spi_rx[1];

	buffer_spi_tx[0] = NOP;

	enviar_dados_SPI(SPIx,buffer_spi_tx,buffer_spi_rx,1);

	return buffer_spi_rx[0];
}

void escrever_registrador(SPI_TypeDef* SPIx, uint8_t endereco,uint8_t buffer_dados[], uint8_t numero_bytes)
{
	uint8_t buffer_spi_tx[33];					//Buffer que será utilizado para os dados recebidos.
	uint8_t buffer_spi_rx[33];

	alterar_estado_CE(0);

	uint8_t mascara_endereco = W_REGISTER;
	mascara_endereco |= endereco;

	buffer_spi_tx[0] = mascara_endereco;
	copiar((buffer_spi_tx+1),buffer_dados,numero_bytes);

	numero_bytes++;

	enviar_dados_SPI(SPIx,buffer_spi_tx,buffer_spi_rx,numero_bytes);
}

void ler_registrador(SPI_TypeDef* SPIx, uint8_t endereco, uint8_t buffer_dados[], uint8_t numero_bytes)
{
	uint8_t buffer_spi_tx[33];					//Buffer que será utilizado para os dados recebidos.
	uint8_t buffer_spi_rx[33];

	uint8_t mascara_endereco = R_REGISTER;
	mascara_endereco |= endereco;

	buffer_spi_tx[0] = mascara_endereco;

	numero_bytes++;

	enviar_dados_SPI(SPIx, buffer_spi_tx, buffer_spi_rx, numero_bytes);

	copy_to(buffer_dados, (buffer_spi_rx+1), 0,(numero_bytes-1));
}

void configurar_nRF24L01(SPI_TypeDef* SPIx, uint8_t ativar_pipes, uint8_t ativar_aa_pipes, uint8_t largura_endereco, uint8_t hab_auto_resend, uint8_t delay_auto_resend, uint8_t byterate_h, uint8_t byterate_l, uint8_t tamanho_payload_pipes[])
{
	uint8_t buffer_conf[25];

	ativar_pipes = ativar_pipes;
	ativar_aa_pipes = ativar_aa_pipes;

	uint8_t contador = 0;
	uint8_t endereco = 0;

	GPIO_InitTypeDef estrutura_conf_gpio;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

	estrutura_conf_gpio.GPIO_Pin = GPIO_Pin_12;					//Pino CE
	estrutura_conf_gpio.GPIO_Mode = GPIO_Mode_OUT;
	estrutura_conf_gpio.GPIO_OType = GPIO_OType_PP;
	estrutura_conf_gpio.GPIO_Speed = GPIO_Speed_25MHz;
	estrutura_conf_gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &estrutura_conf_gpio);

	GPIOB->BSRRH |= GPIO_Pin_12;

	estrutura_conf_gpio.GPIO_Mode = GPIO_Mode_IN;
	estrutura_conf_gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	estrutura_conf_gpio.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &estrutura_conf_gpio);


	buffer_conf[0] = 3;			//	<<<<<<<<<<<----------- Problema aqui
	escrever_registrador(SPIx, EN_AA, buffer_conf, 1);

	buffer_conf[0] = 3;			//	<<<<<<<<<<<----------- Problema aqui
	escrever_registrador(SPIx, EN_RXADDR, buffer_conf,1);

	buffer_conf[0] = (uint8_t)(largura_endereco-2);
	escrever_registrador(SPIx, SETUP_AW, buffer_conf, 1);

	buffer_conf[0] = 0x00;												//Máscara para configuração do AA
	buffer_conf[0] |= hab_auto_resend;									//4LSB -> Habilitam número de tentativas de AA no duto desejado.
	buffer_conf[0] |= (uint8_t)delay_auto_resend*16;					//4MSB -> Tempo entre as tentativas de retransmissão.
	escrever_registrador(SPIx, SETUP_RETR, buffer_conf, 1);

	buffer_conf[0] = 0x06;
	buffer_conf[0] |= (byterate_h*8)|(byterate_l*32);	//Deslocamento de três bits.
	escrever_registrador(SPIx, RF_SETUP, buffer_conf, 1);

	copy_to(buffer_conf, tamanho_payload_pipes,0, ativar_pipes+1);

	endereco = 0x11;
	for(; contador< (ativar_pipes+1); contador++,endereco++)
	{
		buffer_conf[0] = 32; //<<<<<<<<<<- Programa aqui.
		escrever_registrador(SPIx, endereco, buffer_conf,1);
	}
}

void alterar_estado_CE(uint8_t novo_estado)
{
	if(novo_estado != 0)
		GPIOB->BSRRL |= GPIO_Pin_12;
	else
		GPIOB->BSRRH |= GPIO_Pin_12;

}

void modo_rx(SPI_TypeDef* SPIx)								//Partindo do modo Standby-1 -> Partindo de CE = 0;
{
	uint8_t buffer_conf[2];

	uint8_t mascara = 0x00;

	mascara = (PWR_UP|PRIM_RX|CRCO|EN_CRC);
	buffer_conf[0] = mascara;
	escrever_registrador(SPIx, CONFIG, buffer_conf,1);		//Seta o PRIM_RX  e deixa CE = 0;

	delay(2);												//Delay entre o Chip-select e o CE deve ser de no m�nimo 10u segundos.

	alterar_estado_CE(1);									//Seta o pino CE -> modo RX e deixa CE

	delay(20);												//Delay de 200uS
}

void modo_tx(SPI_TypeDef* SPIx)								//Modo TX à partir do modo RX -> Pulso vem da funçaõ de envio do payload.
{
	uint8_t buffer_conf[2];

	alterar_estado_CE(0);

	uint8_t mascara = 0x00;

	mascara = (PWR_UP|CRCO|EN_CRC);
	buffer_conf[0] = mascara;

	escrever_registrador(SPIx, CONFIG, buffer_conf,1);		//Limpa o PRIM_RX. e deixa CE = 0;

	delay(2);												//Delay entre o Chip-select e o CE.
}

void leitura_dados(SPI_TypeDef* SPIx, uint8_t buffer_dados[], uint8_t tamanho_payload)
{
	uint8_t buffer_spi_tx[33];					//Buffer que será utilizado para os dados recebidos.
	uint8_t buffer_spi_rx[33];

	alterar_estado_CE(0);					//Volta para o stand-by do modo RX (Modo-RX -> Stand_by-1)

	buffer_spi_tx[0] = R_RX_PAYLOAD;	

	enviar_dados_SPI(SPIx,buffer_spi_tx, buffer_spi_rx, tamanho_payload+1);

	copiar(buffer_dados,buffer_spi_rx+1,tamanho_payload);
}

void configurar_endereco(SPI_TypeDef* SPIx, uint8_t endereco_tx[], uint8_t endereco_rx[], uint8_t duto, uint8_t nro_bytes_end_tx, uint8_t nro_bytes_end_rx)
{
	escrever_registrador(SPIx,TX_ADDR, endereco_tx, nro_bytes_end_tx);
	escrever_registrador(SPIx, (RX_ADDR_P0+duto), endereco_rx, nro_bytes_end_rx);
}

uint8_t escrita_dados(SPI_TypeDef* SPIx, uint8_t *buffer_dados, uint8_t tamanho_payload)
{
	uint8_t buffer_conf[2];

	uint8_t status_rf = 0;

	modo_tx(SPIx);																//->PRIM_RX = 0 e CE = 0

	escrever_tx_payload(SPIx, buffer_dados, tamanho_payload);

	delay(2);																	//Delay entre Chip-select e CE -> M�nimo de 10 uS

	alterar_estado_CE(1);
	delay(3);																	//Delay de 300 uS
	alterar_estado_CE(0);

	while(Checar_IRQ() != 0);													//Ao final do envio o pino IRQ irá indicar uma mudança no flag TX_DS



	status_rf = retornar_status(SPIx);

	if((status_rf & TX_DS) != 0)
	{
		buffer_conf[0] = TX_DS|MAX_RT;
		escrever_registrador(SPIx,STATUS, buffer_conf, 1);

		return 1;
	}else
	{
		buffer_conf[0] = MAX_RT;
		limpar_fifo_tx(SPIx);
		
		escrever_registrador(SPIx,STATUS, buffer_conf, 1);

		return 0;
	}
}

void limpar_fifo_tx(SPI_TypeDef* SPIx)
{
	uint8_t buffer_spi_tx[2];					//Buffer que será utilizado para os dados recebidos.
	uint8_t buffer_spi_rx[2];

	alterar_estado_CE(0);

	buffer_spi_tx[0] = FLUSH_TX;

	enviar_dados_SPI(SPIx, buffer_spi_tx, buffer_spi_rx,1);
}

void limpar_fifo_rx(SPI_TypeDef* SPIx)
{
	uint8_t buffer_spi_tx[2];					//Buffer que será utilizado para os dados recebidos.
	uint8_t buffer_spi_rx[2];

	alterar_estado_CE(0);

	buffer_spi_tx[0] = FLUSH_RX;

	enviar_dados_SPI(SPIx, buffer_spi_tx, buffer_spi_rx,1);
}

void limpar_flags_status(SPI_TypeDef* SPIx, uint8_t flags_para_limpar)
{
	uint8_t buffer_spi_tx[1];					//Buffer que será utilizado para os dados recebidos.
	//uint8_t buffer_spi_rx[1];

	buffer_spi_tx[0] = retornar_status(SPI2);
	buffer_spi_tx[0] |= flags_para_limpar;

	escrever_registrador(SPIx,STATUS, buffer_spi_tx, 1);
}

void escrever_tx_payload(SPI_TypeDef* SPIx, uint8_t *buffer_dados, uint8_t tamanho_payload)
{
	uint8_t buffer_spi_tx[33];					//Buffer que será utilizado para os dados recebidos.
	uint8_t buffer_spi_rx[33];

	buffer_spi_tx[0] = W_TX_PAYLOAD;

	copiar((buffer_spi_tx+1),buffer_dados,tamanho_payload);

	enviar_dados_SPI(SPIx, buffer_spi_tx, buffer_spi_rx, tamanho_payload+1);		//Escrita no buffer TX
}

void transmitir_payload()
{
	alterar_estado_CE(1);
	delay(2);							//Pulso no CE de 200uS
	alterar_estado_CE(0);
}

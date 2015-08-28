#ifndef _nRF24l01_H_
#define _nRF24l01_H_

	#define NOP 0xFF

	#define W_REGISTER 0b00100000
	#define R_REGISTER 0b00000000

	#define R_RX_PAYLOAD 0b01100001
	#define W_TX_PAYLOAD 0b10100000

	#define FLUSH_TX 0b11100001
	#define FLUSH_RX 0b11100010

	#define REUSE_TX_PL 0b11100011

	#define CONFIG 		0x00
	#define EN_AA 		0x01
	#define EN_RXADDR 	0x02
	#define SETUP_AW 	0x03
	#define SETUP_RETR 	0x04
	#define RF_SETUP 	0x06
	#define STATUS		0x07
	#define TX_ADDR		0x10
	#define RX_ADDR_P0	0x0A

	//Defines do registrador de CONFIG
	#define PRIM_RX 	0x01
	#define PWR_UP 		0x02
	#define CRCO 		0x04
	#define EN_CRC 		0x08
	#define MASK_MAX_RT 0x10
	#define MASK_TX_DS 	0x20
	#define MASK_RX_DR	0x40

	//Defines dos bits do registrador CONFIG

	#define RX_DR 0x40
	#define TX_DS 0x20
	#define MAX_RT 0x10
	#define TX_FULL 0x01

	#define Checar_IRQ() GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)

	//---------------------------------

		uint8_t retornar_status(SPI_TypeDef*);
		void escrever_registrador(SPI_TypeDef*, uint8_t endereco,uint8_t *buffer_dados, uint8_t numero_bytes);
		void ler_registrador(SPI_TypeDef*, uint8_t endereco, uint8_t buffer_dados[], uint8_t numero_bytes);
		void configurar_nRF24L01(SPI_TypeDef*, uint8_t ativar_pipes, uint8_t ativar_aa_pipes, uint8_t largura_endereco, uint8_t hab_auto_resend, uint8_t delay_auto_resend, uint8_t byterate_h, uint8_t byterate_l, uint8_t tamanho_payload_pipes[]);
		void leitura_dados(SPI_TypeDef*, uint8_t buffer_dados[], uint8_t tamanho_payload);
		uint8_t escrita_dados(SPI_TypeDef*, uint8_t buffer_dados[], uint8_t tamanho_payload);
		void alterar_estado_CE(uint8_t novo_estado);
		void iniciar_placa(SPI_TypeDef*);
		void modo_rx(SPI_TypeDef*);
		void modo_tx(SPI_TypeDef*);
		void configurar_endereco(SPI_TypeDef*, uint8_t endereco_tx[], uint8_t endereco_rx[], uint8_t duto, uint8_t nro_bytes_end_tx, uint8_t nro_bytes_end_rx);
		void limpar_fifo_tx(SPI_TypeDef*);
		void limpar_fifo_rx(SPI_TypeDef*);
		void limpar_flags_status(SPI_TypeDef*, uint8_t flags_para_limpar);
		void escrever_tx_payload(SPI_TypeDef* SPIx, uint8_t *dados, uint8_t tamanho_payload);
		void transmitir_payload();
		void limpar_flags_rf(SPI_TypeDef* SPIx, uint8_t flags);

#endif

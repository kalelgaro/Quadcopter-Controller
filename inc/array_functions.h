#ifndef _ARRAY_FUNCTIONS_H_
#define _ARRAY_FUNCTIONS_H_
	
#define CR 13
	#define LF 10

		void limpar_buffer(uint8_t *,uint8_t);
		void inverter_vetor(uint8_t *,uint8_t);
		void copy_to(uint8_t *,uint8_t *,uint8_t,uint8_t);
		void escrever_string_buffer(uint8_t *,const uint8_t *);
		uint8_t receber_byte_UART(void);
		uint8_t numToASCII(int16_t,uint8_t *);
		void printfint(double,uint8_t *);
		void copiar(uint8_t destino[], uint8_t origem[], uint8_t tamanho);

#endif
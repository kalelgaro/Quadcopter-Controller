/**
  ******************************************************************************
  * @file    IO_Toggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"

#include "arm_math.h"
#include "funcoes_spi.h"
#include "nRF24l01.h"
#include "array_functions.h"
#include "string.h"
#include "stdio.h"
#include "controle_motores.h"
#include "aquisicao_IMU.h"
#include "ADXL345.h"
#include "L3G4200D.h"
#include "HMC5883L.h"
#include "tratamento_sinal.h"
#include "stdlib.h"
#include "kalman_filter.h"
#include "processo_controle.h"
#include "processamento_entrada.h"

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/

GPIO_InitTypeDef  GPIO_InitStructure;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

#define MSG_Recebida() (status_RF & RX_DR)
#define Buffer_TX_cheio() (status_RF & TX_FULL)

#define M1_f1 GPIO_Pin_2
#define M1_f2 GPIO_Pin_4
#define M1_f3 GPIO_Pin_6

#define M2_f1 GPIO_Pin_1
#define M2_f2 GPIO_Pin_3

/* Private variables ---------------------------------------------------------*/

uint8_t buffer_dados_tx[33] = "";
uint8_t buffer_dados_rx[33] = "";	

extern uint16_t variavel_delay_100ms;

extern float dc_ch_1;
extern float dc_ch_2;
extern float dc_ch_3;
extern float dc_ch_4;

uint8_t start_logging_final = 0;
uint8_t start_logging_sensores = 1;

float telemetria_kalman[3];
float telemetria_pid[3];

float telemetria_acelerometro[3];
float telemetria_giroscopio[3];
float telemetria_magnetometro[3];

//Variáveis para armazenar os valores das constantes do pid para telemetria.

float kp,ki,kd, kp_yaw, ki_yaw, kd_yaw;		

//Variáveis de bias teste para o giroscópio

float bx,by,bz,bmx,bmy,bmz;

//Variávies para armazenar os valores das constantes do FK para telemetria.

float32_t Q_acelerometro, Q_magnetometro, Q_bias, R_acelerometro, R_magnetometro;

//extern float32_t variancia_roll;
//extern float32_t variancia_pitch;

union byte_converter
{
  uint32_t armazenamento_flash;
  float flutuante_entrada;
  uint8_t bytes[4];

}conversor;

//Coeficientes filtros FIR
//Frequência de amostragem - 800 Hz
//Final da banda de passagem - 5Hz
//Atenuação máxima na banda de passagem de 5dB
//Banda de rejeição - 15Hz - Atenuação no fim da banda de rejeição - 40dB
//Ordem - 91

//float coeficientes_FIR[91] = {0.0056999,0.001782,0.0020507,0.0023411,0.0026491,0.00298,0.003328,0.0036964,0.0040826,0.0044878,0.0049083,0.0053472,0.0058007,0.0062714,0.006754,0.0072498,0.0077544,0.0082689,0.0087882,0.0093145,0.0098432,0.010377,0.01091,0.01144,0.011963,0.01248,0.012989,0.013495,0.013979,0.014446,0.014909,0.01534,0.015758,0.016151,0.016518,0.016862,0.017177,0.017463,0.017718,0.017943,0.018134,0.018292,0.018415,0.018504,0.018558,0.018577,0.018558,0.018504,0.018415,0.018292,0.018134,0.017943,0.017718,0.017463,0.017177,0.016862,0.016518,0.016151,0.015758,0.01534,0.014909,0.014446,0.013979,0.013495,0.012989,0.01248,0.011963,0.01144,0.01091,0.010377,0.0098432,0.0093145,0.0087882,0.0082689,0.0077544,0.0072498,0.006754,0.0062714,0.0058007,0.0053472,0.0049083,0.0044878,0.0040826,0.0036964,0.003328,0.00298,0.0026491,0.0023411,0.0020507,0.001782,0.0056999};

/* Private function prototypes -----------------------------------------------*/

static void configurar_acelerometro();
void iniciar_RF();
void iniciar_giroscopio();
void configurar_bussola();
void blinkDebugLeds();
void iniciar_timer_processamento(void);
void iniciar_timer_controle(void);
void enviar_log(SPI_TypeDef* SPIx, uint8_t identificador, float dados[2]);
void delay_startup(void);
void iniciar_leds_debug(void);
void teste_filtro_de_kalman();

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

int main(void)
{
	iniciar_leds_debug();

	//teste_filtro_de_kalman();

	setar_parametros_PID(60, 15, 15, 100, 1, 30);								//Ajusta as constantes do PID para Roll e Pitch.

	//Qang, Qbiasmag, Racel, Rmag, Rorth
	setar_parametros_Kalman(1e-6, 5e-10, 0.5, 1.5, 1e-6);						//Ajusta as covariâncias do filtro de Kalman.
	//Melhores parametreos testados até o momento - 2e-9, 5e-8, 5e-12, 2.e-2, 2e-1, 1e-10, 1e-10
	
	uint16_t counter_recebidos = 0;												//Variável para contagem do número de mensagens recebidas.

	uint8_t status_RF = 0;														//Variável que aloca o estado do link RF.

	SysTick_Config(168e6/10000);		 										//Frequência de 100uS no systick.

	blinkDebugLeds();															//Pisca os leds
	
	iniciar_ESC();																//Inicia PWM para controle dos ESCS.

	ajustar_velocidade(15,0);													//15 -> 0b1111 -> todos os motores -> Velocidade 0 -> Motores parados.

	delay_startup();															//Delay de inicialização dos ESCS.

	iniciar_RF();																//Inicar a placa nRF24L01p

	//delay(100);																//Delay de 100*100us = 10mS

	configurar_I2C();															//Configurar o periférico I²C da placa.

	//delay(1000);																//Delay de 10*100us = 100mS

	iniciar_giroscopio();														//Iniciar o Giroscópio.

	//delay(100);																	//Delay de 100*100us = 10mS

	configurar_acelerometro();													//Iniciar o acelerômetro.

	//delay(100);																	//Delay de 10mS
	
	configurar_bussola();														//Inicia o magnetômetro.
	
	//delay(100);																	//Delay de 10mS

	//iniciar_timer_controle();													//Timer responsável por checar se houve recepção de controel nos últimos 2 segundos.

	//delay(100);																	//Delay de 100*100us = 10mS

	escrita_dados(SPI2, (uint8_t *)"Iniciado.", 32);							//Mensagem que informa que o procimento de inicialização foi concluído.

	//delay(100);																	//Delay de 100*100us = 10mS

	modo_rx(SPI2);																//Habilita recepção de novas mensagens.

	//delay(100);																	//Delay de 100*100us = 10mS

	configurar_timers_PWM_I();													//Configura os timers utilizados para PWMinput do controle.

	iniciar_estado_Kalman();

	iniciar_timer_processamento();												//Iniciar o timer responsável pelo controle do PID -> TIM6.

	while (1)																	//Processo contínuo de checagem do transmissor RF.
	{
		if (!Checar_IRQ())														//Checa eventos do transmissor. -> 0 Novo evento | 1 Não há novos eventos.
		{
			status_RF = retornar_status(SPI2);									//Registrador de Status do transmissor -> Causa da interrupção.

			if (MSG_Recebida())													//Checa o 6º bit -> Mensagem recebida
			{

	  			GPIO_ToggleBits(GPIOD, GPIO_Pin_13);							//Toggle no LED que indica a recepção de novas mensagens.

				TIM_SetCounter(TIM7,0);											//Reseta o contador do timer -> Inicia nova contagem de 2 segundos.

				/*Recupera as msgs enquanto houverem dados à serem recuperados*/

				while((status_RF & 0x0E) != 0x0E)								//Bits 1,2 e 3 -> Número do duto que recebeu mensagem no RF.
				{																//0b1110 -> Sem dados do duto. Repete o processo enquanto tiver dados.

    				counter_recebidos++;										//Contador para teste - Comparação entre número de msg enviadas / recebidas.

					limpar_buffer(buffer_dados_tx, 33);							//Limpa os buffers para troca de dados.
					limpar_buffer(buffer_dados_rx, 33);

					leitura_dados(SPI2, buffer_dados_rx, 32);

					buffer_dados_tx[0] = status_RF;								//Limpa o FLAG de msg recebida no registrador de status.
					buffer_dados_tx[0] |= RX_DR;

					escrever_registrador(SPI2, STATUS, buffer_dados_tx, 1);

					status_RF = retornar_status(SPI2);							//Checagem se há mais dados nos disponíveis para serem tratados na próxima iteração.

					switch(buffer_dados_rx[0])									//Primeiro caractér do vetor recebido determina a operação à ser realizada.
					{
						case 't':
							GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
						break;


						//Função de teste do LINK RF -> Retorna o número de payloads recebidos
						//Utilizada para checagem no número de pacotes perdidos.
						case 'C':
							numToASCII(counter_recebidos, buffer_dados_tx);
							escrita_dados(SPI2, buffer_dados_tx, 32);
						break;
						
						case 'R':
							buffer_dados_tx[0] = 'R';

							conversor.flutuante_entrada = dc_ch_1;
							copy_to(buffer_dados_tx, conversor.bytes, 1, 4);

							conversor.flutuante_entrada = dc_ch_2;
							copy_to(buffer_dados_tx, conversor.bytes, 5, 4);

							conversor.flutuante_entrada = dc_ch_3;
							copy_to(buffer_dados_tx, conversor.bytes, 9, 4);

							conversor.flutuante_entrada = dc_ch_4;
							copy_to(buffer_dados_tx, conversor.bytes, 13, 4);

							escrita_dados(SPI2, buffer_dados_tx, 32);
						break;

						case 'S':

							if(start_logging_sensores == 0)
								start_logging_sensores = 1;
							else if(start_logging_sensores == 1)
								start_logging_sensores = 0;
						break;
						case 'L':

							if(start_logging_final == 0)
								start_logging_final = 1;

							else if(start_logging_final == 1)
								start_logging_final = 0;

						break;


					    //Retorna as constantes utilizadas no PID.
						case 'Y':

							limpar_buffer(buffer_dados_tx,33);

							retornar_parametros_pid(&kp, &ki, &kd);

							buffer_dados_tx[0] = '%';

							conversor.flutuante_entrada = kp;
							copy_to(buffer_dados_tx, conversor.bytes, 1, 4);

							conversor.flutuante_entrada = kd;
							copy_to(buffer_dados_tx, conversor.bytes, 5, 4);

							conversor.flutuante_entrada = ki;
							copy_to(buffer_dados_tx, conversor.bytes, 9, 4);

							escrita_dados(SPI2, buffer_dados_tx, 32);

						break;


						//Retorna as convariâncias e variância utilizadas no filtro de Kalman.
						case 'J':

							limpar_buffer(buffer_dados_tx,33);

							retornar_parametros_Kalman(&Q_acelerometro, &Q_magnetometro, &Q_bias, &R_acelerometro, &R_magnetometro);

							buffer_dados_tx[0] = '^';

							conversor.flutuante_entrada = Q_acelerometro;
							copy_to(buffer_dados_tx, conversor.bytes, 1, 4);

							conversor.flutuante_entrada = Q_magnetometro;
							copy_to(buffer_dados_tx, conversor.bytes, 5, 4);

							conversor.flutuante_entrada = Q_bias;
							copy_to(buffer_dados_tx, conversor.bytes, 9, 4);

							conversor.flutuante_entrada = R_acelerometro;
							copy_to(buffer_dados_tx, conversor.bytes, 13, 4);

							conversor.flutuante_entrada = R_magnetometro;
							copy_to(buffer_dados_tx, conversor.bytes, 17, 4);

							escrita_dados(SPI2, buffer_dados_tx, 32);

					    break;

					    //Insere um nível de bias de teste no giroscópio
					    case 'b' :
					    	conversor.bytes[0] = buffer_dados_rx[1];
							conversor.bytes[1] = buffer_dados_rx[2];
							conversor.bytes[2] = buffer_dados_rx[3];
							conversor.bytes[3] = buffer_dados_rx[4];

							bx = conversor.flutuante_entrada;

							conversor.bytes[0] = buffer_dados_rx[5];
							conversor.bytes[1] = buffer_dados_rx[6];
							conversor.bytes[2] = buffer_dados_rx[7];
							conversor.bytes[3] = buffer_dados_rx[8];

							by = conversor.flutuante_entrada;

							conversor.bytes[0] = buffer_dados_rx[9];
							conversor.bytes[1] = buffer_dados_rx[10];
							conversor.bytes[2] = buffer_dados_rx[11];
							conversor.bytes[3] = buffer_dados_rx[12];

							bz = conversor.flutuante_entrada;

							conversor.bytes[0] = buffer_dados_rx[13];
							conversor.bytes[1] = buffer_dados_rx[14];
							conversor.bytes[2] = buffer_dados_rx[15];
							conversor.bytes[3] = buffer_dados_rx[16];

							bmx = conversor.flutuante_entrada;

							conversor.bytes[0] = buffer_dados_rx[17];
							conversor.bytes[1] = buffer_dados_rx[18];
							conversor.bytes[2] = buffer_dados_rx[19];
							conversor.bytes[3] = buffer_dados_rx[20];

							bmy = conversor.flutuante_entrada;

							conversor.bytes[0] = buffer_dados_rx[21];
							conversor.bytes[1] = buffer_dados_rx[22];
							conversor.bytes[2] = buffer_dados_rx[23];
							conversor.bytes[3] = buffer_dados_rx[24];

							bmz = conversor.flutuante_entrada;
	
							setar_bias(bx, by, bz, bmx, bmy, bmz);

						//Altera as constantes utilizadas no PID.
						case 'U':

							conversor.bytes[0] = buffer_dados_rx[1];
							conversor.bytes[1] = buffer_dados_rx[2];
							conversor.bytes[2] = buffer_dados_rx[3];
							conversor.bytes[3] = buffer_dados_rx[4];

							kp = conversor.flutuante_entrada;

							conversor.bytes[0] = buffer_dados_rx[5];
							conversor.bytes[1] = buffer_dados_rx[6];
							conversor.bytes[2] = buffer_dados_rx[7];
							conversor.bytes[3] = buffer_dados_rx[8];

							kd = conversor.flutuante_entrada;

							conversor.bytes[0] = buffer_dados_rx[9];
							conversor.bytes[1] = buffer_dados_rx[10];
							conversor.bytes[2] = buffer_dados_rx[11];
							conversor.bytes[3] = buffer_dados_rx[12];

							ki = conversor.flutuante_entrada;

							conversor.bytes[0] = buffer_dados_rx[13];
							conversor.bytes[1] = buffer_dados_rx[14];
							conversor.bytes[2] = buffer_dados_rx[15];
							conversor.bytes[3] = buffer_dados_rx[16];

							kp_yaw = conversor.flutuante_entrada;

							conversor.bytes[0] = buffer_dados_rx[17];
							conversor.bytes[1] = buffer_dados_rx[18];
							conversor.bytes[2] = buffer_dados_rx[19];
							conversor.bytes[3] = buffer_dados_rx[20];

							kd_yaw = conversor.flutuante_entrada;

							conversor.bytes[0] = buffer_dados_rx[21];
							conversor.bytes[1] = buffer_dados_rx[22];
							conversor.bytes[2] = buffer_dados_rx[23];
							conversor.bytes[3] = buffer_dados_rx[24];

							ki_yaw = conversor.flutuante_entrada;

							setar_parametros_PID(kp,ki,kd, kp_yaw, ki_yaw, kd_yaw); //Insere os parametros no processo de controle.

						break;


						//Altera as constantes utilizadas no filtro de Kalman.
						case 'T':

							conversor.bytes[0] = buffer_dados_rx[1];
							conversor.bytes[1] = buffer_dados_rx[2];
							conversor.bytes[2] = buffer_dados_rx[3];
							conversor.bytes[3] = buffer_dados_rx[4];

							Q_acelerometro = conversor.flutuante_entrada;

							conversor.bytes[0] = buffer_dados_rx[5];
							conversor.bytes[1] = buffer_dados_rx[6];
							conversor.bytes[2] = buffer_dados_rx[7];
							conversor.bytes[3] = buffer_dados_rx[8];

							Q_magnetometro = conversor.flutuante_entrada;

							conversor.bytes[0] = buffer_dados_rx[9];
							conversor.bytes[1] = buffer_dados_rx[10];
							conversor.bytes[2] = buffer_dados_rx[11];
							conversor.bytes[3] = buffer_dados_rx[12];

							Q_bias = conversor.flutuante_entrada;

							conversor.bytes[0] = buffer_dados_rx[13];
							conversor.bytes[1] = buffer_dados_rx[14];
							conversor.bytes[2] = buffer_dados_rx[15];
							conversor.bytes[3] = buffer_dados_rx[16];

							R_acelerometro = conversor.flutuante_entrada;

							conversor.bytes[0] = buffer_dados_rx[17];
							conversor.bytes[1] = buffer_dados_rx[18];
							conversor.bytes[2] = buffer_dados_rx[19];
							conversor.bytes[3] = buffer_dados_rx[20];

							R_magnetometro = conversor.flutuante_entrada;							

							//Qang, Qbias, Qbiasmag, Racel, Rmag
							setar_parametros_Kalman(Q_acelerometro, Q_magnetometro, R_acelerometro, R_magnetometro, 1e-10); //Insere os parametros no processo de controle.

						break;


						//Se a sequência de caracteres recebida não possui um comando reconhecido, retorna o número de caracteres
						//contidos no pacote.
						default:

							numToASCII((strlen((char *) buffer_dados_rx)), buffer_dados_tx);
							escrita_dados(SPI2, buffer_dados_tx, 32);

						break;

					}
				}

				//Retorna ao modo de recepção.

				modo_rx(SPI2);

			}
		}

		//Função para envio dos dados da telemetria : Se a telemetria estiver ativada e o tempo entre o envio se passou.

		if(variavel_delay_100ms == 0)
		{
			buffer_dados_tx[0] = 'L';

			retornar_estado(telemetria_kalman, telemetria_pid);

			conversor.flutuante_entrada = telemetria_kalman[0];
			copy_to(buffer_dados_tx, conversor.bytes, 1, 4);


			conversor.flutuante_entrada = telemetria_kalman[1];
			copy_to(buffer_dados_tx, conversor.bytes, 5, 4);


			conversor.flutuante_entrada = telemetria_kalman[2];
			copy_to(buffer_dados_tx, conversor.bytes, 9, 4);


			conversor.flutuante_entrada = telemetria_pid[0];
			copy_to(buffer_dados_tx, conversor.bytes, 13, 4);


			conversor.flutuante_entrada = telemetria_pid[1];
			copy_to(buffer_dados_tx, conversor.bytes, 17, 4);


			conversor.flutuante_entrada = telemetria_pid[2];
			copy_to(buffer_dados_tx, conversor.bytes, 21, 4);

			escrita_dados(SPI2, buffer_dados_tx, 32);
			
			//Retorna ao modo de recepção para habilitar a chegada de novos pacotes.

//			modo_rx(SPI2);

			//Variável para controle de tempo entre os envios dos dados da telemetria.

			//variavel_delay_100ms = 500; 	//500* 100 uS -> 1 Ponto a cada 50 mS

		//}else if(variavel_delay_100ms == 0 && start_logging_sensores == 1 && start_logging_final == 0)
		//{
			buffer_dados_tx[0] = 'S';

			retornar_estado_sensores(telemetria_acelerometro, telemetria_giroscopio, telemetria_magnetometro);

			conversor.flutuante_entrada = telemetria_acelerometro[0];
			copy_to(buffer_dados_tx, conversor.bytes, 1, 4);


			conversor.flutuante_entrada = telemetria_acelerometro[1];
			copy_to(buffer_dados_tx, conversor.bytes, 5, 4);


			conversor.flutuante_entrada = telemetria_acelerometro[2];
			copy_to(buffer_dados_tx, conversor.bytes, 9, 4);


			conversor.flutuante_entrada = telemetria_giroscopio[0];
			copy_to(buffer_dados_tx, conversor.bytes, 13, 4);


			conversor.flutuante_entrada = telemetria_giroscopio[1];
			copy_to(buffer_dados_tx, conversor.bytes, 17, 4);


			conversor.flutuante_entrada = telemetria_giroscopio[2];
			copy_to(buffer_dados_tx, conversor.bytes, 21, 4);


			conversor.flutuante_entrada = telemetria_magnetometro[0];
			copy_to(buffer_dados_tx, conversor.bytes, 25, 4);


			//conversor.flutuante_entrada = telemetria_giroscopio[1];
			//copy_to(buffer_dados_tx, conversor.bytes, 29, 4);
			
			escrita_dados(SPI2, buffer_dados_tx, 32);
			
			//Retorna ao modo de recepção para habilitar a chegada de novos pacotes.

			modo_rx(SPI2);
			variavel_delay_100ms = 250; 	//500* 100 uS -> 1 Ponto a cada 50 mS
		}
	}
}


//Rotina para inicialização do link RF com os parâmetros utilizados neste

void iniciar_RF() 
{

	uint8_t buffer_conf_rf[11] = "";

	iniciar_spi2();				//Inicia o periférico de comunicação serial.

	iniciar_placa(SPI2);		//Coloca a placa (nRF24l01p) em modo stand-by 1 para configuração dos registradores.

	//Configurações globais da placa.

	buffer_conf_rf[0] = 32;
	buffer_conf_rf[1] = 32;
	configurar_nRF24L01(SPI2, 1, 1, 5, 15, 15, 0, 1, buffer_conf_rf);

	//Configuração do endereço do duto 0

	buffer_conf_rf[0] = 10;
	buffer_conf_rf[1] = 11;
	buffer_conf_rf[2] = 12;
	buffer_conf_rf[3] = 13;
	buffer_conf_rf[4] = 14;
	configurar_endereco(SPI2, buffer_conf_rf, buffer_conf_rf, 0, 5, 5); //Endereço Duto 0 Auto_ACK

	//Configuração do endereço do duto 1

	buffer_conf_rf[5] = 15;
	buffer_conf_rf[6] = 16;
	buffer_conf_rf[7] = 17;
	buffer_conf_rf[8] = 18;
	buffer_conf_rf[9] = 19;
	configurar_endereco(SPI2, buffer_conf_rf, buffer_conf_rf + 5, 1, 5, 5); //Endereço Duto 1

	//Limpa as fifos de envio e recepção.

	limpar_fifo_tx(SPI2);
	limpar_fifo_rx(SPI2);

	//Limpa todos os flags do registrador de status.

  	buffer_dados_tx[0] = 0x70;
  	escrever_registrador(SPI2, 0x07, buffer_dados_tx, 1);

  	//Delay de 1mS
  	delay(10);

  	//Coloca a placa em modo de recepção.

	modo_rx(SPI2);
}

//Rotina para exibir, visualmente, que os dispostivios foram iniciados

void blinkDebugLeds() {

	GPIO_ToggleBits(GPIOD, GPIO_Pin_12);

	delay(2500);

	GPIO_ToggleBits(GPIOD, GPIO_Pin_12);

	delay(2500);
	
	GPIO_ToggleBits(GPIOD, GPIO_Pin_13);

	delay(2500);

	GPIO_ToggleBits(GPIOD, GPIO_Pin_13);

	delay(2500);

	GPIO_ToggleBits(GPIOD, GPIO_Pin_14);

	delay(2500);

	GPIO_ToggleBits(GPIOD, GPIO_Pin_14);

	delay(2500);

	GPIO_ToggleBits(GPIOD, GPIO_Pin_15);

	delay(2500);

	GPIO_ToggleBits(GPIOD, GPIO_Pin_15);

	delay(2500);
}

//Rotina para inicialização do giroscópio.

void iniciar_giroscopio()
{ 
  	L3G4200D_InitTypeDef Configuracao_gyro;

  	Configuracao_gyro.Axes_Enable = XYZ_EN;				//Ativação dos três eixos
  	Configuracao_gyro.Power_Mode = NORMAL_MODE;			//Modo de operação normal
  	Configuracao_gyro.Output_DataRate = DR1;    		//DR = 400 Hz
  	Configuracao_gyro.bandwidth = 0;        			//Frequência de corte = 20 Hz
  	Configuracao_gyro.Self_Test = ST_NORMAL;			//Self-Teste desativado
  	Configuracao_gyro.BDU_Enabled = BDU;				//Block Data Update - Impede a reescrita dos registradores antes da leitura.
  	Configuracao_gyro.Full_Scale = FS500DPS;			//Fundo de escala de 250 graus por segundo

  	L3G4200D_Init(I2C3, &Configuracao_gyro);

  	L3G4200D_HPFilterConfigTypeDef Configuracao_HPF_gyro;

  	Configuracao_HPF_gyro.HP_cutoff_freq = 0b1001;
  	Configuracao_HPF_gyro.HP_mode = REFERENCE;

  	Configuracao_HPF_gyro.Filter_Enable = 1;
  	Configuracao_HPF_gyro.Output__Selection = Out_Sel1 | Out_Sel0;

  	L3G4200D_HP_Init(I2C3, &Configuracao_HPF_gyro);
  	
}

//Rotina para inicialização do acelerômetro
//Adicionalmente, obtêm o 'zero g level' do sensor (Offset).

void configurar_acelerometro()
{
  	uint16_t counter_offset_accel;

  	float teste[3] = {0,0,0};

  	float offset_accel[3] = {0,0,0};

  	ADXL345_InitTypeDef configuracao_inicial;

	//Configuração do ADXL345.

  	configuracao_inicial.Power_Mode = Measure;							//Coloca a placa em modo de aquisição contínua.
  	configuracao_inicial.bandwidth = Rate_D3 | Rate_D2;					//Aquisição da placa em 400 Hz.
  	configuracao_inicial.Full_Scale = Range_D1;							//Fundo de escala em 8G
  	configuracao_inicial.Resolution = 0;								//Resolução da placa em 10 bits.
  	configuracao_inicial.Self_Test = 0;									//Desliga o "self-test"

  	ADXL345_Init(I2C3, &configuracao_inicial);							
  	
  	//-------Obtenção do "Zero G level" , OFFSET da placa-------//

  	counter_offset_accel = 400;

  	delay(1000);

  	offset_accel[0] = 0; offset_accel[1] = 0; offset_accel[2] = 0;

  	while(counter_offset_accel--)
  	{

    	GPIO_ToggleBits(GPIOD, GPIO_Pin_13);

    	ADXL345_Read_Data(I2C3, teste);

    	offset_accel[0] += teste[0];
    	offset_accel[1] += teste[1];
    	offset_accel[2] += (teste[2]-1);

    	delay(50);      //0.05S
  	}	

  	offset_accel[0] = (offset_accel[0]/(float)400);
  	offset_accel[1] = (offset_accel[1]/(float)400);
  	offset_accel[2] = (offset_accel[2]/(float)400);

  	setar_offset_acel(offset_accel);
}

//Inicia o Magnetômetro para obtenção orientação magnética.

void configurar_bussola()
{	
	HMC5883L_InitTypeDef configuracao_inicial;

	configuracao_inicial.Samples = _8_samples;
	configuracao_inicial.Output_DataRate = _75_0_HZ;
	//configuracao_inicial.Meas_mode = Positive_bias;
	//configuracao_inicial.Meas_mode = Negative_bias;
	configuracao_inicial.Meas_mode = Default_Meas;
	configuracao_inicial.Gain = Gain_5;
	configuracao_inicial.Mode = Countinuous;
	configuracao_inicial.HS_I2C = 0;
	
	HMC5883L_Init(I2C3, &configuracao_inicial);
}

//Inicia o timer principal utilizado na aquisição de dados (TIM6)
//A frequência de interrupção é de 400 Hz (2,5mS)

void iniciar_timer_processamento()
{

  	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  	uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 100000) - 1;		//100.000 Contagens por segundo

  	TIM_TimeBaseStructure.TIM_Period = 250;											//(1/100.000)*250 segundos por "overflow" -> 0.0025 segundos por overflow
  	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

  	NVIC_InitTypeDef NVIC_InitStructure;
  	/* Enable the TIM6 gloabal Interrupt */
  	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

  	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
  	/* TIM6 enable counter */
	TIM_Cmd(TIM6, ENABLE);

  	//STM_EVAL_LEDInit(LED4);
}

//O timer abaixo será utilizado para checagem se há houve recepção de dados num intervalo pré-definido
//Se o timer abaixo disparar a interrupção , houveram 2 segundos sem recepção de sinal, e desta forma,
//há o procedimento de desligamento

void iniciar_timer_controle()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 10000) - 1;		//10000 Contagens por segundo

	TIM_TimeBaseStructure.TIM_Period = 20000;										//(1/10000)*20000 segundos por "overflow" -> 0.5 Segundos
  	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;							//Insere o prescale calculo acima
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;								
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;						//Apena conta de maneira ascendente -> Overflow ao atingir o fim do contador.
  	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);


  	NVIC_InitTypeDef NVIC_InitStructure;
  	/* Enable the TIM7 gloabal Interrupt */
  	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

  	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
  	/* TIM7 enable counter */
	TIM_Cmd(TIM7, ENABLE);
}

//Rotina de pausa inicial para estabilização dos PWMS utilizados nos ESCS.

void delay_startup(void)
{
  uint32_t var_contador = 1000000;
  while(var_contador--);
}

//Rotina que inicia as portas conectadas aos leds de debug

void iniciar_leds_debug(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
}

 // void teste_filtro_de_kalman(void)
 // {
 // 	kalman_filter_state estado_teste = {  {0,0,0,0,0,0},

 // 										  {100,0,0,0,0,0,
 // 										   0,100,0,0,0,0,
 // 										   0,0,100,0,0,0,
 // 										   0,0,0,100,0,0,
 // 										   0,0,0,0,100,0,
 // 										   0,0,0,0,0,100},

 // 								           1e-7, 1e-4, 1e-5, 2e1, 4e1, 1e-10, 1e-10, 0.0025, {1, 0, 0}};

 // 	float teste_medida_gyro[3] = {20, 10, 45};
 // 	float teste_medida_acel[3] = {0.3, 0.45, 0.85};
 // 	float teste_medida_mag[3] = {1.3, 1, -0.3};

 // 	kalman_filter(&(estado_teste), teste_medida_gyro, teste_medida_acel, teste_medida_mag, 1);

 // 	kalman_filter(&(estado_teste), teste_medida_gyro, teste_medida_acel, teste_medida_mag, 1);

 // 	float teste = 4.5;
 // }

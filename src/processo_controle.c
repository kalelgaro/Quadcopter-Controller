#define ARM_MATH_CM4
  
#ifndef __FPU_USED
  #define __FPU_USED 1
#endif

#ifndef __FPU_PRESENT
  #define __FPU_PRESENT 1
#endif

#include <stm32f4xx_conf.h>

#include "aquisicao_IMU.h"
#include "L3G4200D.h"
#include "ADXL345.h"
#include "HMC5883L.h"

#include "arm_math.h"
#include "kalman_filter.h"
#include "processo_controle.h"
#include "tratamento_sinal.h"

#define yaw 2
#define pitch 1
#define roll 0


#define acel_x 0
#define acel_y 1
#define acel_z 2

/* ----------------------------------------------------------  */
#define numero_medias_PID 4


/*-------Variáveis globais que serão utilizadas no processo-------*/

float kp,ki,kd; //Constantes que serão utilizadas no controle PID de Pitch e Roll;

float kp_yaw,ki_yaw,kd_yaw; //Constantes que serão utilizadas no controle PID do Yaw;

/*----Flags para controle do processo de controle----*/
uint8_t controlador_ligado = 0;

/*----Variávies utilizadas como referência para o controlador-----*/

float ref_pitch = 0.0;
float ref_roll = 0.0;
float ref_yaw = 0.0;

uint16_t rotacao_constante = 0;

/*------------------------------------------------------*/

float offset_accel[3];

float saida_gyro_dps_pf[3] = {0,0,0};		//Buffer para valores do gyroscopio antes da filtragem.

float acelerometro_adxl345[3];

float angulos_inclinacao[2];

float magnetometro[3];
float orientacao;

//Buffers de estado dos controles PID para pitch e roll.

double buffer_pid_pitch[4] = {0, 0, 0, 0};
double buffer_pid_roll[4] =  {0, 0, 0, 0};
double buffer_pid_yaw[4] =   {0, 0, 0, 0};

float buffer_pid_pitch_media[numero_medias_PID];
float buffer_pid_roll_media[numero_medias_PID];
float buffer_pid_yaw_media[numero_medias_PID];

//Variáveis com os valores de resposta obtidos dos controladores PID.

float saida_pitch_pid = 0;
float saida_roll_pid =  0;
float saida_yaw_pid =   0;

//Variáveis de saída do PID após a média realizada no laço de controle.

float saida_pitch_pid_final = 0;
float saida_roll_pid_final =  0;
float saida_yaw_pid_final =   0;

//Variáveis utilizadas para armazenar o resultado do filtro de Kalman.

float roll_pos_filtro;
float pitch_pos_filtro;
float yaw_pos_filtro;

//Estruturas de buffer utilizadas para cálculo das estimativas do Filtro de Kalman.

kalman_filter_state EstadoFiltroKalman = {{0,0,1,0,0,0,0,0,0}, 

									  {100,0,0,0,0,0,0,0,0,
									   0,100,0,0,0,0,0,0,0,
									   0,0,100,0,0,0,0,0,0,
									   0,0,0,100,0,0,0,0,0,
									   0,0,0,0,100,0,0,0,0,
									   0,0,0,0,0,100,0,0,0,
									   0,0,0,0,0,0,100,0,0,
									   0,0,0,0,0,0,0,100,0,
									   0,0,0,0,0,0,0,0,100}, 1e-8, 1e-1, 1e-5, 2e2, 2e3, 0.0025};

//Erros utilizados nos controladores PID
									   
float erro_pitch ,erro_roll ,erro_yaw;

//Vetor de testes

float zeros[3] = {0, 0, 0};

//--------------------------------------------------------------------//


//Altera os valores de referência utilizados no controlador PID.
	//Os angulos de referência - Pitch,Roll e Yaw vão de -1 a 1
	//O valor da rotação constante - W_cte - Vai de 0 até 2.
void setar_referencia(float Ref_pitch, float Ref_roll, float Ref_yaw, float W_cte)
{
	//Altera a referência dos controladores
		//A constante de 10 define que a excursão dos ângulos vai de - 10 até 10 graus.
		//As condições abaixo checam se os valores inseridos são validos.

	if((Ref_pitch >= -1.1) && (Ref_pitch <= 1.1))
		ref_pitch = (Ref_pitch*15);
	else
		ref_pitch = 0;

	if((Ref_roll >= -1.1) && (Ref_roll <= 1.1))
		ref_roll =  (Ref_roll*15);
	else
		ref_roll = 0;

	if((Ref_yaw >= -1.1) && (Ref_yaw <= 1.1))
		ref_yaw = (Ref_yaw*15);
	else
		ref_yaw = 0;
	

	//Checa a posição da alavanca de aceleração -> 
		//Entre 0 e 0.15 e o controlador esta desligado -> Mantém o controlador desligado -> Segurança de inicialização.
		//Entre 0.15 e 2.2 - Controlador ligado e insere o valor mulitplicado por 850 no motor


	if(W_cte < 0.15)
	{
		rotacao_constante = 0;
		controlador_ligado = 0;

	}else if((W_cte > 0.15) && (W_cte <= 2.2))
	{
		controlador_ligado = 1;
		rotacao_constante = (W_cte*975); //Insere o valor de rotação dos motores entre 146,25 e 2145
	}
}

//Altera as contastes do controlador PID. - Roll, Pitch e Yaw.

void setar_parametros_PID(float Kp, float Ki, float Kd, float Kp_yaw, float Ki_yaw, float Kd_yaw)
{
	kp = Kp;
	ki = Ki;
	kd = Kd;

	kp_yaw = Kp_yaw;
	ki_yaw = Ki_yaw;
	kd_yaw = Kd_yaw;
}

//Altera os valores das constantes utilizados no filtro de Kalman.

void setar_parametros_Kalman(float32_t Q_acelerometro, float32_t Q_magnetometro, float32_t Q_bias, float32_t R_acelerometro, float32_t R_magnetometro)
{
	EstadoFiltroKalman.Q_acel = Q_acelerometro;
	EstadoFiltroKalman.Q_mag = Q_magnetometro;
	EstadoFiltroKalman.Q_bias = Q_bias;
	
	EstadoFiltroKalman.R_acel = R_acelerometro;
	EstadoFiltroKalman.R_mag = R_magnetometro;
}

//Retorna o offset que foi obtido para o acelerometro
void retornar_offset_acel(float *offsetX, float *offsetY, float *offsetZ)
{
	*offsetX = offset_accel[acel_x];
	*offsetY = offset_accel[acel_y];
	*offsetZ = offset_accel[acel_z]; 
}

//Retorna os parametros utilizados no PID (Telemetria)

void retornar_parametros_pid(float *Kp, float *Ki, float *Kd)
{
	*Kp = kp;
	*Ki = ki;
	*Kd = kd;
}

//Retrona os parametros utilizados no Filtro de Kalman (Telemetria)

void retornar_parametros_Kalman(float32_t *Q_acelerometro, float32_t *Q_magnetometro, float32_t *Q_bias, float32_t *R_acelerometro, float32_t *R_magnetometro)
{
	*Q_acelerometro = EstadoFiltroKalman.Q_acel;
	*Q_magnetometro = EstadoFiltroKalman.Q_mag;
	*Q_bias = EstadoFiltroKalman.Q_bias;

	*R_acelerometro = EstadoFiltroKalman.R_acel;
	*R_magnetometro = EstadoFiltroKalman.R_mag;
}

//Altera o valor de offset do acelerômetro (Zero G Level).

void setar_offset_acel(float offset[3])
{
	offset_accel[0] = offset[0];
	offset_accel[1] = offset[1];
	offset_accel[2] = offset[2];
}

/*----Procedimentos utilizados durante a rotina de controle-----*/
//Aquisição dos sensores.

void processar_acelerometro()
{
	//Aquisição das leituras do acelerômetro.
	ADXL345_Read_Data(I2C3,acelerometro_adxl345);

	//Remove o offset, desvio, do acelerômetro das leituras.
    acelerometro_adxl345[acel_x] -= offset_accel[acel_x];
	acelerometro_adxl345[acel_y] -= offset_accel[acel_y];
	acelerometro_adxl345[acel_z] -= offset_accel[acel_z];
}

void processar_magnetometro()
{
	uint8_t status;

	//Leitura do registrador de STATUS do magnetômetro
	I2C_ler_registradores(I2C3, end_HMC5883L, STATUS_MG, 1, &status);


	//Chega no registrador de status se há leituras prontas no magnetômetro.
	if((status&0x01)==0x01)
	{
		orientacao = HMC5883L_Read_Data(I2C3, magnetometro);
	}
}

void processar_giroscopio()
{
	L3G4200D_Read_Data(I2C3, saida_gyro_dps_pf);

	//Conversao de unidades -> deg/s -> rad/s
	saida_gyro_dps_pf[0] = saida_gyro_dps_pf[0]*0.0174532925;
	saida_gyro_dps_pf[1] = saida_gyro_dps_pf[1]*0.0174532925;
	saida_gyro_dps_pf[2] = saida_gyro_dps_pf[2]*0.0174532925;
}

//Retorna as variáveis de estado utilizadas para telemetria.

void retornar_estado(float estado_KF[], float estado_PID[])
{
	estado_KF[roll] =  angulos_inclinacao[roll];
	estado_KF[pitch] = angulos_inclinacao[pitch];
	estado_KF[yaw] =   orientacao;

	// estado_KF[roll] =  ref_roll;
	// estado_KF[pitch] = ref_pitch;
	// estado_KF[yaw] =   ref_yaw;

	estado_PID[roll] =  saida_pitch_pid_final;
	estado_PID[pitch] = saida_roll_pid_final;
	estado_PID[yaw] =   saida_yaw_pid_final;
}

void retornar_estado_sensores(float Acelerometro[], float Giroscopio[], float Magnetometro[])
{
	Acelerometro[0] = EstadoFiltroKalman.ultimo_estado[0];
	Acelerometro[1] = EstadoFiltroKalman.ultimo_estado[1];
	Acelerometro[2] = EstadoFiltroKalman.ultimo_estado[2];

	Giroscopio[0] = acelerometro_adxl345[0];
	Giroscopio[1] = acelerometro_adxl345[1];
	Giroscopio[2] = acelerometro_adxl345[2];

	Magnetometro[0] = EstadoFiltroKalman.ultimo_estado[3];
	Magnetometro[1] = EstadoFiltroKalman.ultimo_estado[4];
	Magnetometro[2] = EstadoFiltroKalman.ultimo_estado[5];
}


//Cálculo do YAW, orientção, com base no documento
	/*Implementing a Tilt-Compensated eCompass using Accelerometer and Magnetometer Sensors*/
float calcular_orientacao(float leituras_mag[], float Pitch, float Roll)
{
	//Theta -> Pitch
	//Phi -> Roll

	/*Valores de offset obtidos através do sphereFIT no matlab*/
	float Vx = -0.1610 ;
	float Vy = -0.0581;
	float Vz = 0.0872;

	float MagX = leituras_mag[0]-Vx;
	float MagY = leituras_mag[1]-Vy;
	float MagZ = leituras_mag[2]-Vz;	

	//Conversão de graus para radianos.
	Pitch = (Pitch/57.3);
	Roll = (Roll/57.3);

	/*Graus para Radianos*/
	float heading;

	float temp1;
	float temp2;

	temp1 = MagZ*sin(Roll) - MagY*cos(Roll);
	temp2 = MagX*cos(Pitch) + MagY*sin(Roll)*sin(Pitch) + MagZ*sin(Pitch)*cos(Roll); 

	heading = 57.3*atan2(temp1,temp2);

	return heading;
}

//Procedimento de controle principal executado no overflow no Timer 3 à cada 1,25mS (800 Hz)
void processo_controle()
{
	GPIO_SetBits(GPIOD, GPIO_Pin_12);   			//Led ajuda na hora de debbugar - ACende no início do processo e apaga ao seu final, permitindo obtenção do tempo com um osc. ou analizador lógico.

	static uint16_t contador_ativacao = 0;

	static uint8_t flag_inicializacao = 0;

    //Lê os dados do giroscópio, acelerômetro e magnetômetro.

    processar_giroscopio();

    processar_acelerometro();

    processar_magnetometro();

    //Insere os valores da leituras dentro do filtro de Kalman.
	kalman_filter(&EstadoFiltroKalman, saida_gyro_dps_pf, acelerometro_adxl345, magnetometro);


    //Cálculos dos ângulos de rotação do referêncial no corpo do veículo em relação ao referêncial inercial (superfície)   
    //Roll e Pitch
	acel_2_angulos(EstadoFiltroKalman.ultimo_estado[acel_x], EstadoFiltroKalman.ultimo_estado[acel_y], EstadoFiltroKalman.ultimo_estado[acel_z], angulos_inclinacao);
	
	//Offset angular observado na telemetria.
	//angulos_inclinacao[pitch] -= -0.3;
	//angulos_inclinacao[roll] -= 0.3;
	
	//Yaw
	orientacao = calcular_orientacao((EstadoFiltroKalman.ultimo_estado)+3, angulos_inclinacao[pitch], -angulos_inclinacao[roll]);

	/*Ajuste de sentidos dos angulos*/
	angulos_inclinacao[roll] = -angulos_inclinacao[roll];

	if(flag_inicializacao == 1 && controlador_ligado == 1)
	{
		//Controlador PID com o resultado do filtro de Kalman

		/* Cálcula o erro  bufferque será utilizado no PID -> Referência - Feedback */
		erro_pitch = (ref_pitch - angulos_inclinacao[pitch]);
		erro_roll =  (ref_roll - angulos_inclinacao[roll]);
		erro_yaw =   (ref_yaw - orientacao);

		/* Cálculo do PID */
			//Pitch & Roll
		saida_pitch_pid = calcular_PID(erro_pitch, kp, 	ki, 	kd, 	buffer_pid_pitch, 0.0025); //Controlador PI para cada eixo.
		saida_roll_pid  = calcular_PID(erro_roll,  kp, 	ki, 	kd,		buffer_pid_roll,  0.00125);
		
			//Yaw
		saida_yaw_pid 	= calcular_PID(erro_yaw,	kp_yaw, ki_yaw, kd_yaw, buffer_pid_yaw,   0.0025);

		
		//Realiza média rotativa dos últimos n processos de cálculo do PID.
		saida_pitch_pid_final = media_rotativa(saida_pitch_pid, buffer_pid_pitch_media, numero_medias_PID); 
		saida_roll_pid_final  = media_rotativa(saida_roll_pid,  buffer_pid_roll_media,  numero_medias_PID);
		saida_yaw_pid_final	  = media_rotativa(saida_yaw_pid,   buffer_pid_yaw_media,   numero_medias_PID);

		//Insere os valores provnientes dos PID's dentro da função que divide a carga para cada um dos motores. -- Arredondamento pois esta só trabalha com inteiro (Duty Cicle é uint16_t)
		inserir_ajuster_motores(saida_pitch_pid_final, saida_roll_pid_final, saida_yaw_pid_final, round(rotacao_constante));

	}else
	{
		//Saída dos PIDS zeradas.
		saida_pitch_pid = 0;
		saida_roll_pid = 0;
		saida_yaw_pid = 0;

		saida_pitch_pid_final = 0;
		saida_roll_pid_final  = 0;
		saida_yaw_pid_final   = 0;

		inserir_ajuster_motores(0, 0, 0, 0);

		if(flag_inicializacao == 0)
			contador_ativacao++;
			
		if(contador_ativacao == 4000)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_14);

			flag_inicializacao = 1;
		}
	}
	//Salva valores de interesse nas estrutura que é enviada para telemetria.

	GPIO_ResetBits(GPIOD, GPIO_Pin_12); 
}

#include "processo_controle.h"

#define yaw 2
#define pitch 1
#define roll 0

#define acel_x 0
#define acel_y 1
#define acel_z 2

/* ----------------------------------------------------------  */
#define numero_medias_PID 1
#define ordem_filtro 203

//Nro de aquisições antes de realizar o processamento no filtro de Kalman.
#define NRO_AQUISICOES_PRE_KF 1

//Nro de aquisições utilizadas para média rotativa.
#define NRO_MEDIA_AQUISICAO 1

/*-------Tempo de amostragem------------*/

#define dt 0.005

/*-------Taxa de rotação constante--------*/

#define CONSTANT_RATE 72 //72 graus por segundo (360/5)


/*-------Inclinações máximas (Roll e Pitch)-------*/
#define MAX_INCLINATION 20

/*------Constante para conversão de Graus para Rad-----*/
#define DEG_TO_RAD 0.01745201

/*-------Contagem de ativação-------*/
#define nro_contagem_ativacao (uint16_t)(1200/NRO_AQUISICOES_PRE_KF)

/*-------Variáveis globais que serão utilizadas no processo-------*/

float kp,ki,kd; //Constantes que serão utilizadas no controle PID de Pitch e Roll;

float kp_yaw,ki_yaw,kd_yaw; //Constantes que serão utilizadas no controle PID do Yaw;

/*----Flags para controle do processo de controle----*/
uint8_t controlador_ligado = 0;

/*----Variávies utilizadas como referência para o controlador-----*/

float ref_pitch = 0.0;
float ref_roll = 0.0;
float ref_yaw = 0.0;
float ref_rate_yaw = 0.0;

uint16_t rotacao_constante = 0;

/*------------------------------------------------------*/

float offset_accel[3] = {0.0, 0.0, 0.0};
float offset_gyro[3] = {0.0, 0.0, 0.0};

float saida_gyro_dps_pf[3] = {0,0,0};		//Buffer para valores do gyroscopio antes da filtragem.

float acelerometro_adxl345[3];

float angulos_inclinacao[2];

float magnetometro[3];
float magOriginal[3];
float orientacao;

float orientacao_inicial = 0.0;
float temp_orientacao;

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

//Buffers para o filtro FIR do acelerômetro.
	//Filtro com banda de passagem de 4Hz e atenuação de 0.1 dB e banda de parada de 40 Hz com atenuação de 120 dB( Filter Builder)
/*
float coeficientes_FIR[ordem_filtro] = {-2.9594e-06,-2.6145e-06,-3.7377e-06,-5.1529e-06,-6.904e-06,-9.04e-06,-1.161e-05,-1.4668e-05,-1.8265e-05,-2.2453e-05,-2.7284e-05,-3.2808e-05,-3.9071e-05,-4.6114e-05,-5.3969e-05,-6.2664e-05,
										-7.2214e-05,-8.2626e-05,-9.3885e-05,-0.00010597,-0.00011882,-0.00013239,-0.00014658,-0.00016128,-0.00017634,-0.00019159,-0.00020683,-0.00022182,-0.00023629,-0.00024993,-0.00026238,-0.00027325,
										-0.00028213,-0.00028854,-0.00029197,-0.00029187,-0.00028765,-0.00027869,-0.00026432,-0.00024385,-0.00021655,-0.00018167,-0.00013843,-8.6029e-05,-2.3667e-05,4.9479e-05,0.00013423,0.0002314,
										0.00034179,0.0004662,0.00060539,0.00076009,0.00093098,0.0011187,0.0013239,0.001547,0.0017884,0.0020487,0.0023279,0.0026264,0.0029442,0.0032813,0.0036375,0.0040126,0.0044062,0.0048178,0.0052467,
										0.0056923,0.0061536,0.0066296,0.0071192,0.007621,0.0081338,0.0086559,0.0091858,0.0097217,0.010262,0.010804,0.011347,0.011888,0.012426,0.012957,0.01348,0.013993,0.014494,0.01498,0.015449,
										0.015899,0.016328,0.016734,0.017116,0.017471,0.017797,0.018094,0.018359,0.018591,0.01879,0.018954,0.019082,0.019174,0.01923,0.019248,0.01923,0.019174,0.019082,0.018954,0.01879,0.018591,0.018359,
										0.018094,0.017797,0.017471,0.017116,0.016734,0.016328,0.015899,0.015449,0.01498,0.014494,0.013993,0.01348,0.012957,0.012426,0.011888,0.011347,0.010804,0.010262,0.0097217,0.0091858,0.0086559,
										0.0081338,0.007621,0.0071192,0.0066296,0.0061536,0.0056923,0.0052467,0.0048178,0.0044062,0.0040126,0.0036375,0.0032813,0.0029442,0.0026264,0.0023279,0.0020487,0.0017884,0.001547,0.0013239,
										0.0011187,0.00093098,0.00076009,0.00060539,0.0004662,0.00034179,0.0002314,0.00013423,4.9479e-05,-2.3667e-05,-8.6029e-05,-0.00013843,-0.00018167,-0.00021655,-0.00024385,-0.00026432,-0.00027869,
										-0.00028765,-0.00029187,-0.00029197,-0.00028854,-0.00028213,-0.00027325,-0.00026238,-0.00024993,-0.00023629,-0.00022182,-0.00020683,-0.00019159,-0.00017634,-0.00016128,-0.00014658,-0.00013239,
										-0.00011882,-0.00010597,-9.3885e-05,-8.2626e-05,-7.2214e-05,-6.2664e-05,-5.3969e-05,-4.6114e-05,-3.9071e-05,-3.2808e-05,-2.7284e-05,-2.2453e-05,-1.8265e-05,-1.4668e-05,-1.161e-05,-9.04e-06,
										-6.904e-06,-5.1529e-06,-3.7377e-06,-2.6145e-06,-2.9594e-06};
*/

//float buffer_filtro_magX[ordem_filtro];
//float buffer_filtro_magY[ordem_filtro];
//float buffer_filtro_magZ[ordem_filtro];

// float buffer_filtro_acelX[ordem_filtro];
// float buffer_filtro_acelY[ordem_filtro];
// float buffer_filtro_acelZ[ordem_filtro];

/*
float buffer_media_acelX[NRO_MEDIA_AQUISICAO];
float buffer_media_acelY[NRO_MEDIA_AQUISICAO];
float buffer_media_acelZ[NRO_MEDIA_AQUISICAO];

float buffer_media_gyroX[NRO_MEDIA_AQUISICAO];
float buffer_media_gyroY[NRO_MEDIA_AQUISICAO];
float buffer_media_gyroZ[NRO_MEDIA_AQUISICAO];
*/

//Estruturas de buffer utilizadas para cálculo das estimativas do Filtro de Kalman.

kalman_filter_state EstadoFiltroKalman = {{1,0,0,0,0,0,0,0,0,0},

                                      {1e-8,    0,		0,		0,		0,		0,      0,      0,      0,      0,      0,      0,      0,
                                       0,		1e-8,	0,		0,		0,		0,      0,      0,      0,      0,      0,      0,      0,
                                       0,		0,		1e-8,	0,		0,		0,      0,      0,      0,      0,      0,      0,      0,
                                       0,		0,		0,		1e-8,	0,		0,      0,      0,      0,      0,      0,      0,      0,
                                       0,		0,		0,		0,		1e-8,	0,      0,      0,      0,      0,      0,      0,      0,
                                       0,		0,		0,		0,		0,		1e-8,   0,      0,      0,      0,      0,      0,      0,
                                       0,       0,      0,      0,      0,      0,      1e-8,   0,      0,      0,      0,      0,      0,
                                       0,       0,      0,      0,      0,      0,      0,      1e-8,   0,      0,      0,      0,      0,
                                       0,       0,      0,      0,      0,      0,      0,      0,      1e-8,   0,      0,      0,      0,
                                       0,       0,      0,      0,      0,      0,      0,      0,      0,      1e-8,   0,      0,      0,
                                       0,       0,      0,      0,      0,      0,      0,      0,      0,      0,      1e-8,   0,      0,
                                       0,       0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      1e-8,   0,
                                       0,       0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      1e-8},

                                       5e-7, 2e-9, 1e-2, 1e-3, 45, 45, dt, {0, 0, 1} ,{0.14, 0.05, -0.0155}};

//Erros utilizados nos controladores PID
									  
float erro_pitch ,erro_roll ,erro_yaw;

//--------------------------------------------------------------------//

//Altera os valores de referência utilizados no controlador PID.
	//Os angulos de referência - Pitch,Roll e Yaw vão de -1 a 1
	//O valor da rotação constante - W_cte - Vai de 0 até 2.
void setar_referencia(float Ref_pitch, float Ref_roll, float Ref_rate_yaw, float W_cte)
{
	//Altera a referência dos controladores
		//A constante de 10 define que a excursão dos ângulos vai de - 10 até 10 graus.
		//As condições abaixo checam se os valores inseridos são validos.

	if((Ref_pitch >= -1.1) && (Ref_pitch <= 1.1))
		ref_pitch = (Ref_pitch*MAX_INCLINATION);
	else
		ref_pitch = 0;

	if((Ref_roll >= -1.1) && (Ref_roll <= 1.1))
		ref_roll =  (Ref_roll*MAX_INCLINATION);
	else
		ref_roll = 0;

	if((Ref_rate_yaw >= -1.1) && (Ref_rate_yaw <= 1.1))
		ref_rate_yaw = Ref_rate_yaw*CONSTANT_RATE*dt;
	else
		ref_rate_yaw = 0;
	

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
		W_cte -= 0.15;
		rotacao_constante = (W_cte*2500); //Insere o valor de rotação dos motores entre 146,25 e 2145
	}
}

void iniciar_estado_Kalman() {

	uint8_t status;

	float mag_init_buffer[3] = {0.0, 0.0, 0.0};
	float yaw_init_buffer = 0;
	float mag_init[3];
	
	//Leitura do registrador de STATUS do magnetômetro
	uint16_t counter = 400;

	while(counter--) {
		do {
            status = TM_I2C_Read(I2C3, end_HMC5883L, STATUS_MG);
			//Aguarda enquanto não houverem novos dados dentro dos registradores de leitura.
			GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
        }while((status&0x01)!= 0x01);

		HMC5883L_Read_Data(I2C3, mag_init);

		mag_init_buffer[0] += mag_init[0];
		mag_init_buffer[1] += mag_init[1];
		mag_init_buffer[2] += mag_init[2];
	}

	mag_init_buffer[0] = mag_init_buffer[0]/(float)400;
	mag_init_buffer[1] = mag_init_buffer[1]/(float)400;
	mag_init_buffer[2] = mag_init_buffer[2]/(float)400;

    yaw_init_buffer += calcular_orientacao(mag_init_buffer, 0, 0);
    //EstadoFiltroKalman.ultimo_estado[2] = yaw_init_buffer;
	
    processar_mpu6050();

    EstadoFiltroKalman.AcelInicial[0] = 0;
    EstadoFiltroKalman.AcelInicial[1] = 0;
    EstadoFiltroKalman.AcelInicial[2] = 1;

	EstadoFiltroKalman.MagInicial[0] = mag_init_buffer[0];
	EstadoFiltroKalman.MagInicial[1] = mag_init_buffer[1];
	EstadoFiltroKalman.MagInicial[2] = mag_init_buffer[2];

    //Valor inicial do quaternion
    EstadoFiltroKalman.ultimo_estado[0] = 1;
    EstadoFiltroKalman.ultimo_estado[1] = 0;
    EstadoFiltroKalman.ultimo_estado[2] = 0;
    EstadoFiltroKalman.ultimo_estado[3] = 0;

    //Valor inicial do bias do Acelerômetro
    EstadoFiltroKalman.ultimo_estado[4] = offset_accel[0];
    EstadoFiltroKalman.ultimo_estado[5] = offset_accel[1];
    EstadoFiltroKalman.ultimo_estado[6] = offset_accel[2];

    //Valor inicial do bias do magnetômetro
//    EstadoFiltroKalman.ultimo_estado[7] = 1;
//    EstadoFiltroKalman.ultimo_estado[8] = 0;
//    EstadoFiltroKalman.ultimo_estado[9] = 0;

    //Valor inicial do bias do giroscópio.
    EstadoFiltroKalman.ultimo_estado[10] = offset_gyro[0];
    EstadoFiltroKalman.ultimo_estado[11] = offset_gyro[1];
    EstadoFiltroKalman.ultimo_estado[12] = offset_gyro[2];

    //EstadoFiltroKalman.ultimo_estado[2] = yaw_init_buffer;
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
void setar_parametros_Kalman(float32_t Q_quat, float32_t Q_biasacel, float32_t Q_biasmag, float32_t Q_biasGyro, float32_t R_acelerometro, float32_t R_magnetometro)
{
    EstadoFiltroKalman.Q_quat = Q_quat;
    EstadoFiltroKalman.Q_bias_acel = Q_biasacel;
	EstadoFiltroKalman.Q_bias_mag = Q_biasmag;
    EstadoFiltroKalman.Q_bias_gyro = Q_biasGyro;
	
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

void retornar_parametros_Kalman(float32_t *Q_quaternion, float32_t *Q_biasacel, float32_t *Q_biasmag, float32_t *R_acelerometro, float32_t *R_magnetometro)
{
    *Q_quaternion = EstadoFiltroKalman.Q_quat;
    *Q_biasmag = EstadoFiltroKalman.Q_bias_mag;
    *Q_biasacel = EstadoFiltroKalman.Q_bias_acel;

	*R_acelerometro = EstadoFiltroKalman.R_acel;
	*R_magnetometro = EstadoFiltroKalman.R_mag;
}

//Alterar o valor de offset do giroscópio

void setar_offset_gyro(float offset[3]) 
{
    offset_gyro[0] = offset[0]*DEG_TO_RAD;
    offset_gyro[1] = offset[1]*DEG_TO_RAD;
    offset_gyro[2] = offset[2]*DEG_TO_RAD;
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

void processar_mpu6050() {
    //Checa a disponibilidade de novos dados para leitura.
    while(MPU6050_checkDataReadyIntPin() == Bit_RESET);
    MPU6050_readData(I2C3, acelerometro_adxl345, saida_gyro_dps_pf);

//    saida_gyro_dps_pf[0] -= offset_gyro[0];
//    saida_gyro_dps_pf[1] -= offset_gyro[1];
//    saida_gyro_dps_pf[2] -= offset_gyro[2];

    saida_gyro_dps_pf[0] = (saida_gyro_dps_pf[0])*DEG_TO_RAD;
    saida_gyro_dps_pf[1] = (saida_gyro_dps_pf[1])*DEG_TO_RAD;
    saida_gyro_dps_pf[2] = (saida_gyro_dps_pf[2])*DEG_TO_RAD;

//    acelerometro_adxl345[acel_x] -= offset_accel[acel_x];
//    acelerometro_adxl345[acel_y] -= offset_accel[acel_y];
//    acelerometro_adxl345[acel_z] -= offset_accel[acel_z];
}

void processar_magnetometro()
{
	uint8_t status;

	//Leitura do registrador de STATUS do magnetômetro
    status = TM_I2C_Read(I2C3, end_HMC5883L, STATUS_MG);

	//Chega no registrador de status se há leituras prontas no magnetômetro.
	if((status&0x01)==0x01)
	{
		HMC5883L_Read_Data(I2C3, magnetometro);
		//normalizar_vetor_R3(magnetometro);
        magnetometro[0] = magnetometro[0];
        magnetometro[1] = magnetometro[1];
        magnetometro[2] = magnetometro[2];

	}
}

//Retorna as variáveis de estado utilizadas para telemetria.

void retornar_estado(float estado_KF[], float estado_PID[])
{
	estado_KF[roll] =  angulos_inclinacao[roll];
	estado_KF[pitch] = angulos_inclinacao[pitch];
	estado_KF[yaw] =   orientacao - orientacao_inicial;

	estado_PID[roll] =  ref_roll;
	estado_PID[pitch] = ref_pitch;
	estado_PID[yaw] =   ref_yaw;
}

void retornar_estado_sensores(float Acelerometro[], float Giroscopio[], float Magnetometro[])
{
	Acelerometro[0] = acelerometro_adxl345[acel_x];
	Acelerometro[1] = acelerometro_adxl345[acel_y];
	Acelerometro[2] = acelerometro_adxl345[acel_z];

    Giroscopio[0] = saida_gyro_dps_pf[0];
    Giroscopio[1] = saida_gyro_dps_pf[1];
    Giroscopio[2] = saida_gyro_dps_pf[2];

	Magnetometro[0] = magnetometro[0];
	Magnetometro[1] = magnetometro[2];
	Magnetometro[2] = magnetometro[1];
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


//Processar referência do yaw 
	//Cálcula a referência de yaw com base na taxa de variação inserida
void calculate_Yaw_Ref(float yaw_Rate) {
	ref_yaw = ref_yaw + yaw_Rate;
	ref_yaw = 57.3*tratar_intervalo_Angulo(ref_yaw*0.017452);
}

//Procedimento de controle principal executado no overflow no Timer 3 à cada 1,25mS (800 Hz)
void processo_controle()
{
	static uint8_t contador_aquisicao = 0;

	static uint16_t contador_ativacao = 0;

	static uint8_t flag_inicializacao = 0;

    uint16_t delayCounter = 10;

    //Lê os dados do giroscópio, acelerômetro e magnetômetro.
    processar_magnetometro();
    //GPIO_SetBits(GPIOD, GPIO_Pin_12);   			//Led ajuda na hora de debbugar - ACende no início do processo e apaga ao seu final, permitindo obtenção do tempo com um osc. ou analizador lógico.

    processar_mpu6050();

    contador_aquisicao++;

    //Reinicia o contador de segurança.
    TIM_SetCounter(TIM7, 0);

    if(contador_aquisicao == NRO_AQUISICOES_PRE_KF) {

    	contador_aquisicao = 0;
	       
	    //Insere os valores da leituras dentro do filtro de Kalman.
		kalman_filter(&EstadoFiltroKalman, saida_gyro_dps_pf, acelerometro_adxl345, magnetometro, rotacao_constante);	
        EulerAngles angles = getEulerFromQuaternion(EstadoFiltroKalman.ultimo_estado);

        angulos_inclinacao[roll] = 57.3*angles.phi;
        angulos_inclinacao[pitch] = 57.3*angles.theta;
        orientacao = 57.3*angles.psi;

		if(flag_inicializacao == 1 && controlador_ligado == 1)
		{
			//Controlador PID com o resultado do filtro de Kalman

			/* Cálcula o erro  bufferque será utilizado no PID -> Referência - Feedback */
			ref_roll = -ref_roll;

			/*Cálcular a referência do yaw com base na taxa inserida pelo controle remoto*/
			calculate_Yaw_Ref(ref_rate_yaw);

			erro_pitch = (ref_pitch - angulos_inclinacao[pitch]);
			erro_roll =  (ref_roll - angulos_inclinacao[roll]);
			erro_yaw =   (ref_yaw - orientacao + orientacao_inicial);

			/* Cálculo do PID */
				//Pitch & Roll
			saida_pitch_pid = calcular_PID(erro_pitch, kp, 	ki, 	kd, 	buffer_pid_pitch, dt); //Controlador PI para cada eixo.
			saida_roll_pid  = calcular_PID(erro_roll,  kp, 	ki, 	kd,		buffer_pid_roll,  dt);
			
				//Yaw
			saida_yaw_pid 	= calcular_PID(erro_yaw,	kp_yaw, ki_yaw, kd_yaw, buffer_pid_yaw,   dt);

			
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
			{
				contador_ativacao++;
				orientacao_inicial = orientacao_inicial + orientacao/nro_contagem_ativacao;
			}
				
			if(contador_ativacao == nro_contagem_ativacao)
			{
				flag_inicializacao = 1;
			}
		}
		//Salva valores de interesse nas estrutura que é enviada para telemetria.
	}
    GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}

#ifndef _processo_controle_
#define _processo_controle_

	void setar_referencia(referencias*);
	void setar_parametros_PID(float Kp, float Ki, float Kd, float Kp_yaw, float Ki_yaw, float Kd_yaw);
	void setar_parametros_Kalman(float Q_angle, float Q_bias, float R);
	void setar_offset_acel(float offset[3]);
	void retornar_parametros_pid(float *Kp, float *Ki, float *Kd);
	void retornar_parametros_Kalman(float *Q_angle, float *Q_bias, float *R);
	void processo_controle(void);
	void retornar_estado(float estado_KF[], float estado_PID[]);
	void retornar_estado_sensores(float Acelerometro[], float Giroscopio[], float Magnetometro[]);
	void retornar_offset_acel(float *offsetX, float *offsetY, float *offsetZ);
	
#endif
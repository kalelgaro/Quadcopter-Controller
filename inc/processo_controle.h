#ifndef _processo_controle_
#define _processo_controle_

	void setar_referencia(float Ref_pith, float Ref_roll, float Ref_yaw, float W_cte);
	void setar_parametros_PID(float Kp, float Ki, float Kd, float Kp_yaw, float Ki_yaw, float Kd_yaw);
	void setar_parametros_Kalman(float Q_angulos, float Q_biasmag, float Q_biasgyro, float R_acelerometro, float R_magnetometro, float R_determinant, float R_orthogonal);
	void setar_offset_acel(float offset[3]);
	void retornar_parametros_pid(float *Kp, float *Ki, float *Kd);
	void retornar_parametros_Kalman(float *Q_acelerometro, float *Q_magnetometro, float *Q_bias, float *R_acelerometro, float *R_magnetometro);
	void processo_controle(void);
	void retornar_estado(float estado_KF[], float estado_PID[]);
	void retornar_estado_sensores(float Acelerometro[], float Giroscopio[], float Magnetometro[]);
	void retornar_offset_acel(float *offsetX, float *offsetY, float *offsetZ);
	void calculate_Yaw_Ref(float yaw_Rate);
	void setar_bias(float Bx, float By, float Bz, float Bmx, float Bmy, float Bmz);
	
#endif
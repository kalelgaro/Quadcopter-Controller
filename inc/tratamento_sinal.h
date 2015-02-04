/*
 * tratamento_sinal.h
 *
 *  Created on: Mar 24, 2013
 *      Author: Bruno
 */

#ifndef TRATAMENTO_SINAL_H_
#define TRATAMENTO_SINAL_H_

//	float filtro_passa_baixa(float leitura, float *buffer_filtro, float *buffer_leitura);
	void acel_2_angulos(float acel_x, float acel_y, float acel_z, float angulos[2]);	
	
	void inserir_ajuster_motores(float pitch_pid, float roll_pid, float yaw_pid, uint16_t rotacao_constante);
	float filtro_correlacao(float constant, float leitura_giro, float leitura_acell, float buffer, float dt);
	float filtro_correlacao_2nd(float constant, float leitura_giro, float leitura_acell, float buffer[2], float dt);
	float filtro_fir(float leitura, float buffer_leitura[], uint16_t numero_coeficientes, float buffer_coeficientes[]);
	float media_rotativa(float nova_leitura, float buffer_leituras[], uint16_t numero_pontos);
	float arredondar_float(float entrada, uint8_t numero_entrada);
	void normalizar_vetor_R3(float vetor[3]);
	float calcular_norma_R3(float vetor[3]);
	void Rotate3dVector(float vector[3], float roll, float pitch, float yaw, float Retorno[3]);
	float f_sin(float);
	float f_cos(float);
	float f_tan(float);
	float f_sec(float);
	float tratar_intervalo_Angulo(float angles);
    float getVectorModulus(const float vector[], u8 numberOfElements);
    void normalizeVector(float vector[], u8 numberOfElements);
    void complementaryFilter(float angles[], float gyro[], float accel[], float mag[], float dt, float gain, float accelOffset[], float magOffset[]);

    typedef struct {
        float Kp;
        float Ki;
        float Kd;
        float N;
        float Ts;

        float uk1; //Ação de controle anterior u[k-1];
        float uk2; //Acção de controle de duas iterações passadas u[k-2];
        float ek1; //Erro anterior e[k-1];
        float ek2; //Erro de duas iterações anteriores e[k-2];
    } PIDControllerState;

    typedef struct {
        float phi;
        float theta;
        float psi;
    }EulerAngles;

    double calcular_PID(float entrada, float kp, float ki, float kd, double *buffer_pid, float dt, float dErrorCoeficientes[], float dErrorBuffer[], uint16_t dFilterOrder);
    float updatePIDController(PIDControllerState *state, float error);
    void initPIDControllerState(PIDControllerState *state, float kp, float kd, float ki, float N, float dt);
    void adjustPIDConstants(PIDControllerState *state, float error, float threshold);
    EulerAngles getEulerFromQuaternion(float quaternion[]);
    EulerAngles getEarthFrameRates(EulerAngles bodyFrameAngles, EulerAngles angles);
    float max(float previousMax, float newMeasure);
    float min(float previousMin, float newMeasure);
    float constrainAngle(float deegreesAngles);
    float calcular_orientacao(float leituras_mag[], float Pitch, float Roll, float offsetMag[]);


#endif /* TRATAMENTO_SINAL_H_ */

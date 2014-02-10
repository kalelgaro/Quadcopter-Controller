#ifndef PROCESSAMENTO_ENTRADA_H_
#define PROCESSAMENTO_ENTRADA_H_

	typedef struct {
		float ch1;
		float ch2;
		float ch3;
		float ch4;

	} canais_entrada;

	typedef struct {
		float Ref_pitch;
		float Ref_roll;
		float Ref_yaw;
		float Rotacao_constante;
	} referencias;

	void configurar_timers_PWM_I(void);
	void tratar_referencias(canais_entrada, referencias*);

#endif /* PROCESSAMENTO_ENTRADA_H_ */

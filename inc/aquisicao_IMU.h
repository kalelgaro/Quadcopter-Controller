#ifndef _aquisicao_IMU_H_
#define _aquisicao_IMU_H

	#define end_L3G4200 	0xD2
	#define end_ADXL345 	0xA6
	#define end_HMC5883L 	0x3C 
	#define end_BMP082 		0xEE

	void configurar_I2C(void);
	void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
	void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
	uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
	uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
	void I2C_stop(I2C_TypeDef* I2Cx);

	void I2C_escrever_registrador(I2C_TypeDef* I2Cx, uint8_t slave_address, uint8_t register_address, uint8_t num_bytes, uint8_t buffer_escrita[]);
	void I2C_ler_registradores(I2C_TypeDef* I2Cx, uint8_t slave_address, uint8_t register_address, uint8_t num_bytes, uint8_t buffer_leitura[]);

#endif

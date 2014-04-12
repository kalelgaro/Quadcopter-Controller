#include "stm32f4_discovery.h"

#include "stm32f4xx_i2c.h"
#include "aquisicao_IMU.h"


void configurar_I2C(void)
{
	//checar_scl_sda();

	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

		/*   PA8 i2c3 scl e PC9 i2c3 sda*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

		/* Liga o clock para o módulo I²C */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 ;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF ;								//Pinos configurados para função alternativo.
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;							//Pinos configurados para 50MHz.
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD ;							//Saída configurada em dreno aberto.
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;

	GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3);					//SCL I2C
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3);					//SDA I2C

	I2C_InitStruct.I2C_ClockSpeed = 600000; 								// 400kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;									// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;							// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0xFF;									// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;								// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; 	// set address length to 7 bit addresses

	I2C_Init(I2C3, &I2C_InitStruct);										// init I2C1
	// enable I2C1
	I2C_Cmd(I2C3, ENABLE);

}


/* This function issues a start condition and
 * transmits the slave address + R/W bit
 *
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */

void I2C_r_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
	// Send I2C1 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	/* wait for I2C1 EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */

	if(direction == I2C_Direction_Transmitter)
	{
		while((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
	}

	else if(direction == I2C_Direction_Receiver)
	{
		while((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)));
	}
}

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

	// Send I2C1 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	/* wait for I2C1 EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */



	if(direction == I2C_Direction_Transmitter)
	{
		while((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
	}

	else if(direction == I2C_Direction_Receiver)
	{
		while((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)));
	}
}


/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}


/* This function reads one byte from the slave device
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx)
{
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}


/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx)
{
	// disabe acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}


/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx)
{
	// Send I2C1 STOP Condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void I2C_escrever_registrador(I2C_TypeDef* I2Cx, uint8_t slave_address, uint8_t register_address, uint8_t num_bytes, uint8_t buffer_escrita[])
{
	uint8_t contador_dados = 0;

	I2C_start(I2Cx, slave_address, I2C_Direction_Transmitter);
	I2C_write(I2Cx, register_address);

	for(; contador_dados < num_bytes; contador_dados++)
	{
		I2C_write(I2Cx, buffer_escrita[contador_dados]);
	}

	I2C_stop(I2Cx);
}

void I2C_ler_registradores(I2C_TypeDef* I2Cx, uint8_t slave_address, uint8_t register_address, uint8_t num_bytes, uint8_t buffer_leitura[])
{
	uint8_t contador_dados = 0;

	I2C_start(I2Cx, slave_address, I2C_Direction_Transmitter);
	I2C_write(I2Cx, register_address);
	I2C_r_start(I2Cx, slave_address, I2C_Direction_Receiver);

	for(; contador_dados < (num_bytes-1); contador_dados++)
	{
		buffer_leitura[contador_dados] = I2C_read_ack(I2Cx);
	}

	buffer_leitura[contador_dados] = I2C_read_nack(I2Cx);

	I2C_stop(I2Cx);
}


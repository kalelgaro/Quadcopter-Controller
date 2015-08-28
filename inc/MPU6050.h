#ifndef MPU6050_H_
#define MPU6050_H_

#include "gpiodevice.h"
#include "i2cdevice.h"

#include "accelerometerdevice.h"
#include "gyroscopedevice.h"
#include "magnetometerdevice.h"

#include "ThreeAxisSensors.h"

    #define MPU6050_ADDRESS 0xD0

	#define SELF_TEST_X		0x0D
	#define SELF_TEST_Y 	0x0E
	#define SELF_TEST_Z 	0x0F
	#define SELF_TEST_A 	0x10
	
	#define SMPLRT_DIV 		0x19
    #define CONFIG_MPU6050  0x1A
	#define GYRO_CONFIG		0x1B
	#define ACCEL_CONFIG 	0x1C
	#define MOT_THR			0x1F
	#define FIFO_EN_REG		0x23
	
	#define I2C_MST_CTRL	0x24
	
	#define I2C_SLV0_ADDR	0x25
	#define I2C_SLV0_REG	0x26
	#define I2C_SLV0_CTRL	0x27
	
	#define I2C_SLV1_ADDR	0x28
	#define I2C_SLV1_REG	0x29
	#define I2C_SLV1_CTRL	0x2A
	
	#define I2C_SLV2_ADDR	0x2B
	#define I2C_SLV2_REG	0x2C
	#define I2C_SLV2_CTRL	0x2D
	
	#define I2C_SLV3_ADDR	0x2E	
	#define I2C_SLV3_REG	0x2F
	#define I2C_SLV3_CTRL	0x30

	#define I2C_SLV4_ADDR	0x31	
	#define I2C_SLV4_REG	0x32
	#define I2C_SLV4_D0		0x33
	#define I2C_SLV4_CTRL	0x34
	#define I2C_SLV4_DI		0x35
	
	#define I2C_MST_STATUS	0x36
	
	#define INT_PIN_CFG		0x37
    #define INT_ENABLE_MPU6050 0x38
	#define INT_STATUS		0x3A
	
	#define ACCEL_XOUT_H	0x3B
	#define ACCEL_XOUT_L	0x3C
		
	#define ACCEL_YOUT_H	0x3D
	#define ACCEL_YOUT_L	0x3E
	
	#define ACCEL_ZOUT_H	0x3F
	#define ACCEL_ZOUT_L	0x40

	#define TEMP_OUT_H		0x41
	#define TEMP_OUT_L		0x42

	#define GYRO_XOUT_H		0x43
	#define GYRO_XOUT_L		0x44

	#define GYRO_YOUT_H		0x45
	#define GYRO_YOUT_L		0x46

	#define GYRO_ZOUT_H		0x47
	#define GYRO_ZOUT_L		0x48

	#define EXT_SENS_DATA_00	0x49
	#define EXT_SENS_DATA_01	0x4A
	#define EXT_SENS_DATA_02	0x4B
	#define EXT_SENS_DATA_03	0x4C
	#define EXT_SENS_DATA_04	0x4D
	#define EXT_SENS_DATA_05	0x4E
	#define EXT_SENS_DATA_06	0x4F
	#define EXT_SENS_DATA_07	0x50
	#define EXT_SENS_DATA_08	0x51
	#define EXT_SENS_DATA_09	0x52
	#define EXT_SENS_DATA_10	0x53
	#define EXT_SENS_DATA_11	0x54
	#define EXT_SENS_DATA_12	0x55
	#define EXT_SENS_DATA_13	0x56
	#define EXT_SENS_DATA_14	0x57
	#define EXT_SENS_DATA_15	0x58
	#define EXT_SENS_DATA_16	0x59
	#define EXT_SENS_DATA_17	0x5A
	#define EXT_SENS_DATA_18	0x5B
	#define EXT_SENS_DATA_19	0x5C
	#define EXT_SENS_DATA_20	0x5D
	#define EXT_SENS_DATA_21	0x5E
	#define EXT_SENS_DATA_22	0x5F
	#define EXT_SENS_DATA_23	0x60

	#define I2C_SLV0_D0			0x63
	#define I2C_SLV1_D0			0x64
	#define I2C_SLV2_D0			0x65
	#define I2C_SLV3_D0			0x66

	#define I2C_MST_DELAY_CTRL	0x67
	#define SIGNAL_PATH_RESET	0x68

	#define MOT_DETECT_CTRL		0x69
	#define USER_CTRL			0x7A
	#define	PWR_MGMT_1			0x6B
	#define	PWR_MGMT_2			0x6C
	#define FIFO_COUNTH			0x72
	#define FIFO_COUNTL			0x73

	#define FIFO_R_W			0x74
    #define WHO_AM_I_MPU6050	0x75

	/** Valores padrões dos registradores para os quais este é diferente de zero **/
	#define DFLT_PWR_MGMT_1		0x40
	#define DFLT_WHO_AM_I		0x68


	/** SELF TEST REGISTERS **/
	

	/** ----------------SMPLRT_DIV ------------------
	This register specifies the divider from the gyroscope output rate used to generate the Sample Rate 
	for the MPU-60X0. 
	The sensor register output, FIFO output, DMP sampling and Motion detection are all based on the 
	Sample Rate.
	The Sample Rate is generated by dividing the gyroscope output rate by SMPLRT_DIV:
	Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz 
	when the DLPF is enabled (see Register 26).
	Note:
	For a diagram of the gyroscope and accelerometer signal paths, see Section 8 of the MPU-
	6000/MPU-6050 Product Specification document. 
	The accelerometer output rate is 1kHz. This means that for a Sample Rate greater than 1kHz, 
	the same accelerometer sample may be output to the FIFO, DMP, and sensor registers more than 
	once
	//-----------------------------------------------------------------------------*/


	/**----------------CONFIG------------------------
'	This register configures the external Frame Synchronization (FSYNC) pin sampling and the Digital 
	Low Pass Filter (DLPF) setting for both the gyroscopes and accelerometers.
	Description:
	An external signal connected to the FSYNC pin can be sampled by configuring EXT_SYNC_SET. 
	Signal changes to the FSYNC pin are latched so that short strobes may be captured. The latched 
	FSYNC signal will be sampled at the Sampling Rate, as defined in register 25. After sampling, the 
	latch will reset to the current FSYNC signal state.
	The sampled value will be reported in place of the least significant bit in a sensor data register
	determined by the value of EXT_SYNC_SET according to the following table.
	*/

	//FSYNC Bit Location
	#define FSYNC_DISABLED		0x00
	#define FSYNC_TEMP_OUT_L0 	0x08
	#define FSYNC_GYRO_XOUT_L0	0x10
	#define FSYNC_GYRO_YOUT_L0	0x18
	#define FSYNC_GYRO_ZOUT_L0	0x20
	#define FSYNC_ACCEL_XOUT_L0	0x28
	#define FSYNC_ACCEL_YOUT_L0	0x30
	#define FSYNC_ACCEL_ZOUT_L0	0x38

	//DLPF_CFG - Digital Low Pass Filter Config 


	//-----------------------------------------------------------------------------*/


	/**----------------GYRO_CONFIG----------------------
	This register is used to trigger gyroscope self-test and configure the gyroscopes’ full scale range.
	Gyroscope self-test permits users to test the mechanical and electrical portions of the
	gyroscope. The self-test for each gyroscope axis can be activated by controlling the XG_ST,
	YG_ST, and ZG_ST bits of this register. Self-test for each axis may be performed independently
	or all at the same time.
	When self-test is activated, the on-board electronics will actuate the appropriate sensor. This
	actuation will move the sensor’s proof masses over a distance equivalent to a pre-defined
	Coriolis force. This proof mass displacement results in a change in the sensor output, which is
	reflected in the output signal. The output signal is used to observe the self-test response.
	The self-test response is defined as follows:
	Self-test response = Sensor output with self-test enabled – Sensor output without self-
	test enabled
	The self-test limits for each gyroscope axis is provided in the electrical characteristics tables of
	the MPU-6000/MPU-6050 Product Specification document. When the value of the self-test
	response is within the min/max limits of the product specification, the part has passed self test.
	When the self-test response exceeds the min/max values specified in the document, the part is
	deemed to have failed self-test.	
	 */
	
	//Gyro self test trigger config
	#define XG_ST 0x20
	#define YG_ST 0x40
	#define ZG_ST 0x60

	//Gyro full scale sel
	#define FS_250DPS 	0x00
	#define FS_500DPS 	0x08
	#define FS_1000DPS 	0x10
	#define FS_2000DPS	0x18

	//-----------------------------------------------------------------------------*/

	/**----------------ACCEL_CONFIG----------------------
	This register is used to trigger accelerometer self test and configure the accelerometer full scale
	range. This register also configures the Digital High Pass Filter (DHPF).
	Accelerometer self-test permits users to test the mechanical and electrical portions of the
	accelerometer. The self-test for each accelerometer axis can be activated by controlling the XA_ST,
	YA_ST, and ZA_ST bits of this register. Self-test for each axis may be performed independently or
	all at the same time.
	When self-test is activated, the on-board electronics will actuate the appropriate sensor. This
	actuation simulates an external force. The actuated sensor, in turn, will produce a corresponding
	output signal. The output signal is used to observe the self-test response.
	The self-test response is defined as follows:
	Self-test response = Sensor output with self-test enabled – Sensor output without self-test
	enabled
	The self-test limits for each accelerometer axis is provided in the electrical characteristics tables of
	the MPU-6000/MPU-6050 Product Specification document. When the value of the self-test response
	is within the min/max limits of the product specification, the part has passed self test. When the self-
	test response exceeds the min/max values specified in the document, the part is deemed to have
	failed self-test.
	 */

	//Accel Self test trigger config
	#define XA_ST 0x20
	#define YA_ST 0x40
	#define ZA_ST 0x60

	//Accel full scale sel
	#define AFS_2G		0x00
	#define AFS_4G		0x08
	#define AFS_8G		0x10
	#define AFS_16G		0x18

	//-----------------------------------------------------------------------------*/

	/**----------------MOT_THR----------------------
	This register configures the detection threshold for Motion interrupt generation. The mg per LSB
	increment for MOT_THR can be found in the Electrical Specifications table of the MPU-6000/MPU-
	6050 Product Specification document.
	Motion is detected when the absolute value of any of the accelerometer measurements exceeds this
	Motion detection threshold.
	The Motion interrupt will indicate the axis and polarity of detected motion in MOT_DETECT
	_STATUS (Register 97)
	*/

	//-----------------------------------------------------------------------------*/	

	/**----------------FIFO_EN----------------------
	This register determines which sensor measurements are loaded into the FIFO buffer.
	Data stored inside the sensor data registers (Registers 59 to 96) will be loaded into the FIFO buffer if
	a sensor’s respective FIFO_EN bit is set to 1 in this register.
	When a sensor’s FIFO_EN bit is enabled in this register, data from the sensor data registers will be
	loaded into the FIFO buffer. The sensors are sampled at the Sample Rate as defined in Register 25.
	When an external Slave’s corresponding FIFO_EN bit (SLVx_FIFO_EN, where x=0, 1, or 2) is set to
	1, the data stored in its corresponding data registers (EXT_SENS_DATA registers, Registers 73 to
	96) will be written into the FIFO buffer at the Sample Rate. EXT_SENS_DATA register association
	with I2C Slaves is determined by the I2C_SLVx_CTRL registers (where x=0, 1, or 2; Registers 39,
	42, and 45). For information regarding EXT_SENS_DATA registers, please refer to Registers 73 to
	96.
	Note that the corresponding FIFO_EN bit (SLV3_FIFO_EN) is found in I2C_MST_CTRL (Register
	36). Also note that Slave 4 behaves in a different manner compared to Slaves 0-3. Please refer to
	Registers 49 to 53 for further information regarding Slave 4 usage
	*/

	//TEMP_FIFO_ENFIFO_EN
	#define TEMP_FIFO_EN 		0b00000001
	//XG_FIFO_EN		
	#define XG_FIFO_EN			0b00000010
	//YG_FIFO_EN
	#define YG_FIFO_EN			0b00000100
	//ZG_FIFO_EN
	#define ZG_FIFO_EN			0b00001000
	//ACCEL_FIFO_EN
	#define ACCEL_FIFO_EN		0b00010000
	//SLV2_FIFO_EN
	#define SLV2_FIFO_EN		0b00100000
	//SLV1_FIFO_EN
	#define SLV1_FIFO_EN		0b01000000
	//SLV0_FIFO_EN
	#define SLV0_FIFO_EN		0b10000000

	//-----------------------------------------------------------------------------*/	

	/**----------------I2C_MST_CTRL----------------------
	This register configures the auxiliary I2C bus for single-master or multi-master control. In addition, the
	register is used to delay the Data Ready interrupt, and also enables the writing of Slave 3 data into the 
	FIFO buffer. The register also configures the auxiliary I2C Master’s transition from one slave read
	to the next, as well as the MPU-60X0’s 8MHz internal clock.
	Multi-master capability allows multiple I2C masters to operate on the same bus. In circuits where
	multi-master capability is required, set MULT_MST_EN to 1. This will increase current drawn by
	approximately 30μA.
	In circuits where multi-master capability is required, the state of the I2C bus must always be
	monitored by each separate I2C Master. Before an I2C Master can assume arbitration of the bus, it
	must first confirm that no other I 2 C Master has arbitration of the bus. When MULT_MST_EN is set to
	1, the MPU-60X0’s bus arbitration detection logic is turned on, enabling it to detect when the bus is
	available.
	When the WAIT_FOR_ES bit is set to 1, the Data Ready interrupt will be delayed until External
	Sensor data from the Slave Devices are loaded into the EXT_SENS_DATA registers. This is used to
	ensure that both the internal sensor data (i.e. from gyro and accel) and external sensor data have
	been loaded to their respective data registers (i.e. the data is synced) when the Data Ready interrupt
	is triggered.
	When the Slave 3 FIFO enable bit (SLV_3_FIFO_EN) is set to 1, Slave 3 sensor measurement data
	will be loaded into the FIFO buffer each time. EXT_SENS_DATA register association with I2C Slaves
	is determined by I2C_SLV3_CTRL (Register 48).
	For further information regarding EXT_SENS_DATA registers, please refer to Registers 73 to 96.
	The corresponding FIFO_EN bits for Slave 0, Slave 1, and Slave 2 can be found in Register 35.
	The I2C_MST_P_NSR bit configures the I2C Master’s transition from one slave read to the next
	slave read. If the bit equals 0, there will be a restart between reads. If the bit equals 1, there will be a
	stop followed by a start of the following read. When a write transaction follows a read transaction, the
	stop followed by a start of the successive write will be always used.
	*/

	//MUL_MST_EN - Ativa o modo multi master
	#define MUL_MST_EN 				0x80

	//WAIT_FOR_ES - Aguardar o dado do sensor externo - Sincronização com Gyro e acel
	#define WAIT_FOR_ES				0x40

	//SLV3_FIFO_EN - Ativa a bufferização dos dados do dispositivo I2C Slave 
	#define SLV3_FIFO_EN 			0x20

	//I2C_MST_P_NSR - Controla a transição da recepção de dados de um Slave I2C para outro
		// 0 - Existe um comando de restart no I2C entre cada uma das leituras em cada slave
		// 1 - Existe um Stop seguido de um Start entre cada leitura de cada um dos slaves
	#define I2C_MST_P_NSR			0x10


	//I2C_MST_CLK - Clock quando o MPU6050 é utilizado em modo master
	#define I2C_MST_CLK_348KHZ		0x00
	#define I2C_MST_CLK_333KHZ		0x01
	#define I2C_MST_CLK_320KHZ		0x02
	#define I2C_MST_CLK_308KHZ		0x03
	#define I2C_MST_CLK_296KHZ		0x04
	#define I2C_MST_CLK_286KHZ		0x05
	#define I2C_MST_CLK_276KHZ		0x06
	#define I2C_MST_CLK_267KHZ		0x07
	#define I2C_MST_CLK_258KHZ		0x08
	#define I2C_MST_CLK_500KHZ		0x09
	#define I2C_MST_CLK_471KHZ		0x0A
	#define I2C_MST_CLK_444KHZ		0x0B
	#define I2C_MST_CLK_421KHZ		0x0C
	#define I2C_MST_CLK_400KHZ		0x0D
	#define I2C_MST_CLK_381KHZ		0x0E
	#define I2C_MST_CLK_364KHZ		0x0F


	//-----------------------------------------------------------------------------*/	


	/**----------Registers 37 to 39 – I C Slave 0 Control
	-----------I2C_SLV0_ADDR, I2C_SLV0_REG, and I2C_SLV0_CTRL---------------------
	These registers configure the data transfer sequence for Slave 0. Slaves 1, 2, and 3 also behave in a
	similar manner to Slave 0. However, Slave 4’s characteristics differ greatly from those of Slaves 0-3.
	For further information regarding Slave 4, please refer to registers 49 to 53.
	I2C slave data transactions between the MPU-60X0 and Slave 0 are set as either read or write
	operations by the I2C_SLV0_RW bit. When this bit is 1, the transfer is a read operation. When the bit
	is 0, the transfer is a write operation.
	I2C_SLV0_ADDR is used to specify the I2C slave address of Slave 0.
	Data transfer starts at an internal register within Slave 0. This address of this register is specified by
	I2C_SLV0_REG.
	The number of bytes transferred is specified by I2C_SLV0_LEN. When more than 1 byte is
	transferred (I2C_SLV0_LEN > 1), data is read from (written to) sequential addresses starting from
	I2C_SLV0_REG.
	In read mode, the result of the read is placed in the lowest available EXT_SENS_DATA register. For
	further information regarding the allocation of read results, please refer to the EXT_SENS_DATA
	register description (Registers 73 – 96).
	In write mode, the contents of I2C_SLV0_DO (Register 99) will be written to the slave device.
	I2C_SLV0_EN enables Slave 0 for I2C data transaction. A data transaction is performed only if more
	than zero bytes are to be transferred (I2C_SLV0_LEN > 0) between an enabled slave device
	(I2C_SLV0_EN = 1).
	I2C_SLV0_BYTE_SW configures byte swapping of word pairs. When byte swapping is enabled, the
	high and low bytes of a word pair are swapped. Please refer to I2C_SLV0_GRP for the pairing
	convention of the word pairs. When this bit is cleared to 0, bytes transferred to and from Slave 0 will
	be written to EXT_SENS_DATA registers in the order they were transferred.
	When I2C_SLV0_REG_DIS is set to 1, the transaction will read or write data only. When cleared to
	0, the transaction will write a register address prior to reading or writing data. This bit should equal 0
	when specifying the register address within the Slave device to/from which the ensuing datar
	transaction will take place.
	I2C_SLV0_GRP specifies the grouping order of word pairs received from registers. When cleared to
	0, bytes from register addresses 0 and 1, 2 and 3, etc (even, then odd register addresses) are paired
	to form a word. When set to 1, bytes from register addresses are paired 1 and 2, 3 and 4, etc. (odd,
	then even register addresses) are paired to form a word.
	I2C data transactions are performed at the Sample Rate, as defined in Register 25. The user is
	responsible for ensuring that I2C data transactions to and from each enabled Slave can be
	completed within a single period of the Sample Rate.
	The I2C slave access rate can be reduced relative to the Sample Rate. This reduced access rate is
	determined by I2C_MST_DLY (Register 52). Whether a slave’s access rate is reduced relative to the
	Sample Rate is determined by I2C_MST_DELAY_CTRL (Register 103).
	The processing order for the slaves is fixed. The sequence followed for processing the slaves is
	Slave 0, Slave 1, Slave 2, Slave 3 and Slave 4. If a particular Slave is disabled it will be skipped.
	Each slave can either be accessed at the sample rate or at a reduced sample rate. In a case where
	some slaves are accessed at the Sample Rate and some slaves are accessed at the reduced rate,
	the sequence of accessing the slaves (Slave 0 to Slave 4) is still followed. However, the reduced rate
	slaves will be skipped if their access rate dictates that they should not be accessed during that
	particular cycle. For further information regarding the reduced access rate, please refer to Register
	52. Whether a slave is accessed at the Sample Rate or at the reduced rate is determined by the
	Delay Enable bits in Register 103.
	*/

	//Definiões que são padrões para utilização nos dispositivos SLV0 - SLV3
	#define I2C_SLVn_EN 		0x80			//Habilita (1) / Desabilita (0) o dispotivo SLV0
	#define I2C_SLVn_BYTE_SW	0x40			//Habilita o Byte Swap - High e Low bytes de um par de dados é trocado
	#define I2C_SLVn_RW 		0x80			//Modo de leitura (1) ou gravação (0) para o SLV0
	#define I2C_SLVn_REG_DIS	0x20			//Em 1 - Modo de escrita ou leitura do dispotivo SLV. Quando em zero específica 
	//o  endereço para qual a operação será executada.
	#define I2C_SLVn_GRP		0x10			//Específica a ordem dos pares de grupos dos registradores para os dados 
	//recebidos - 0 = (0 e 1, 2 e 3, ...); 1 = (1 e 2, 3 e 4 ... );



	/*-----------Registers 49 to 53 – I2C Slave 4 Control
	---------I2C_SLV4_ADDR, I2C_SLV4_REG, I2C_SLV4_DO, I2C_SLV4_CTRL, and I2C_SLV4_DI
		These registers describe the data transfer sequence for Slave 4. The characteristics of Slave 4 differ 
	greatly from those of Slaves 0-3. For further information regarding the characteristics of Slaves 0-3, 
	please refer to Registers 37 to 48.
	Description:
	I2C slave data transactions between the MPU-60X0 and Slave 4 are set as either read or write 
	operations by the I2C_SLV4_RW bit. When this bit is 1, the transfer is a read operation. When the bit 
	is 0, the transfer is a write operation. 
	I2C_SLV4_ADDR is used to specify the I2C slave address of Slave 4.
	Data transfer starts at an internal  register within Slave 4. This register address is specified by 
	I2C_SLV4_REG. 
	In read mode, the result of the read will be available in I2C_SLV4_DI. In write mode, the contents of 
	I2C_SLV4_DO will be written into the slave device. 
	A data transaction is performed only if the I2C_SLV4_EN bit is set to 1. The data transaction should 
	be enabled once its parameters are configured in the _ADDR and _REG registers. For write, the 
	_DO register is also required. I2C_SLV4_EN will be cleared after the transaction is performed once. 
	An interrupt is triggered at the completion of a Slave 4 data transaction if the interrupt is enabled .
	The status of this interrupt can be observed in Register 54. 
	When I2C_SLV4_REG_DIS is set to 1, the transaction will read or write data instead of writing a 
	register address. This bit should equal 0 when specifying the register address within the Slave 
	device to/from which the ensuing data transaction will take place.
	I2C_MST_DLY configures the reduced access rate of I2C slaves relative to the Sample Rate. When 
	a slave’s access rate is decreased relative to the Sample Rate, the slave is accessed every 
	1 / (1 + I2C_MST_DLY) samples 
	This base Sample Rate in turn is determined by SMPLRT_DIV  (register 25) and  DLPF_CFG
	(register 26). Whether a slave’s access rate is reduced relative to the Sample Rate is determined by 
	I2C_MST_DELAY_CTRL (register 103). 
	For further information regarding the Sample Rate, please refer to register 25. 
	Slave 4 transactions are performed after Slave 0, 1, 2 and 3 transactions have been completed. 
	Thus the maximum rate for Slave 4 transactions is determined by the Sample Rate as defined in 
-	Register 25-----------------------------------------------------------------------------------------*/

	#define I2C_SLV4_RW 0x80			//Define se é uma operação de leitura ou escrita.
	#define I2C_SLV4_EN 0x80			//Inicia uma transação quando é setado para 1. Registrador limpo após o final da transação.
	#define I2C_SLV4_INT_EN 0x40		//Ativa / Desativa a interrupção causada no final de cada transação do SLV4.
	#define I2C_SLV4_REG_DIS 0x20		//Quando este BIT é igual a 1, a transação irã escrever ou ler dados ao 
	//invés de escrever o end. de um registrador.


	//--------------------------------------------------------------------------------------------------------------

	/*---------Register 54 – I2C Master Status
	-------------I2C_MST_STATUS--------*/

	#define PASS_THROUGH 0x80
	#define I2C_SLV4_DONE 0x40
	#define I2C_LOST_ARB 0x20
	#define I2C_SLV4_NACK 0x10
	#define I2C_SLV3_NACK 0x08
	#define I2C_SLV2_NACK 0x04
	#define I2C_SLV1_NACK 0x02
	#define I2C_SLV0_NACK 0x01

	//---------------------------------------------------------

	/* -----------Register 55 – INT Pin / Bypass Enable Configuration---------------
	------------- INT_PIN_CFG---------------*/

	#define INT_LEVEL 0x80
	#define INT_OPEN 0x40
	#define LATCH_INT_EN 0x20
	#define INT_RD_CLEAR 0x10
	#define FSYNC_INT_LEVEL 0x08
	#define FSYNC_INT_EN 0x04
	#define I2C_BYPASS_EN 0x02

	//----------------------------------------------------------------

	/* -----------------Register 56 – Interrupt Enable
	----------------INT_ENABLE--------------*/

	#define MOT_EN 0x40
	#define FIFO_OFLOW_EN 0x10
	#define I2C_MST_INT_EN 0x08
	#define DATA_RDY_EN 0x01

	//-----------------------------------------------------------------

	/*-----------------------Register 58 – Interrupt Status
	------------------------INT_STATUS-------------*/

	#define MOT_INT 0x40
	#define FIFO_OFLOW_INT 0x10
	#define I2C_MST_INT_INT 0x08
	#define DATA_RDY_INT 0x01

	//------------------------------------------------------------------

	/*-------------------Register 103 – I2C Master Delay Control-------
	----------------------------I2C_MST_DELAY_CTRL-------------------*/

	#define DELAY_ES_SHADOW 0x80
	#define I2C_SLV4_DLY_EN 0x10
	#define I2C_SLV3_DLY_EN 0x08
	#define I2C_SLV2_DLY_EN 0x04
	#define I2C_SLV1_DLY_EN 0x02
	#define I2C_SLV0_DLY_EN 0x01


	//-------------------------------------------------------------------

	/*-------------Register 104 – Signal Path Reset--------------------
	---------------------SIGNAL_PATH_RESET--------------------------*/

	#define GYRO_RESET 0x04
	#define ACCEL_RESET 0x02
	#define TEMP_RESET 0x01

	//-------------------------------------------------------------------

	/*-----------------Register 106 – User Control------------------------
	----------------------USER_CTRL---------------------------------*/
    #define FIFO_EN_MPU6050 0x40
	#define I2C_MST_EN 0x20
	#define I2C_IF_DIS 0x10
	#define FIFO_RESET 0x04
	#define I2C_MST_RESET 0x02
	#define SIG_COND_RESET 0x01

	//-------------------------------------------------------------------

	/*--------------------Register 107 – Power Management 1-------------
	-----------------------------PWR_MGMT_1----------------------------*/

    #define MPU6050_DEVICE_RESET 0x80
    #define MPU6050_SLEEP 0x40
    #define MPU6050_CYCLE 0x20
    #define MPU6050_TEMP_DIS 0x08

    #define MPU6050_CLK_INTERNAL_8MHZ_OSC       0x00
    #define MPU6050_CLK_GYRO_X_PLL              0x01
    #define MPU6050_CLK_GYRO_Y_PLL              0x02
    #define MPU6050_CLK_GYRO_Z_PLL              0x03
    #define MPU6050_CLK_EXTERNAL_32768HZ_OSC    0x04
    #define MPU6050_CLK_EXTERNAL_192MHZ_OSC     0x05

	/*-----------------Register 108 – Power Management 2 ----------------
	----------------------------PWR_MGMT_2------------------------------*/

	#define STBY_XA 0x20
	#define STBY_YA 0x10
	#define STBY_ZA 0x08

	#define STBY_XG 0x04
	#define STBY_YG 0x02
	#define STBY_ZG 0x01

	//--------------------------------------------------------------------

	typedef struct {
		uint8_t gyroFullScale;
		uint8_t accelFullScale;
        uint8_t temperatureSensorDisabled   ;
		uint8_t clockSource;
		uint8_t fifoEnabled;
		uint8_t sampleRateDivider;
		uint8_t digitalLowPassConfig;
        uint8_t interruptsConfig;
        uint8_t intPinConfig;
	} MPU6050_InitStruct;

    void MPU6050_Init(I2C_TypeDef *I2Cx, MPU6050_InitStruct *initialConfig);
    float MPU6050_readData(I2C_TypeDef* I2Cx, float accelBuffer[3], float gyroBuffer[3]);
    void MPU6050_configIntPin(uint32_t RCC_AHB1Periph, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
    uint8_t MPU6050_checkDataReadyIntPin();

    uint8_t MPU6050_checkConectivity(I2C_TypeDef *I2Cx);

namespace Sensors {

class MPU6050 : public Gyroscope<float, unsigned int>, public Accelerometer<float, unsigned int> {
public:
    MPU6050(I2CDevice<uint8_t, uint8_t, uint8_t> &i2c) :
        Gyroscope()
      , Accelerometer()
      , m_i2c(i2c)
    {}

    void updateData( void ) {
        int j;

        for(int i = 10; i > 0; --i) {
            j = i;
        }
    }

private:
    I2CDevice<uint8_t, uint8_t, uint8_t> &m_i2c;


};

}
#endif /* MPU6050_H_ */

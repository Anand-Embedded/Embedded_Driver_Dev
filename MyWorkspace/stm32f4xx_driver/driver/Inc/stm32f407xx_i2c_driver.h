/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Sep 26, 2021
 *      Author: Admin
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddr;
	uint8_t I2C_ACKCtrl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;


typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t pI2C_Config;
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SPEED_STD_MODE		100000
#define I2C_SPEED_FAST_MODE		400000

/*
 * @I2C_ACKCtrl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1


/*
 * I2C Flag Mask Macro's
 */
#define I2C_FLAG_SB				 (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR			 (1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF			 (1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF			 (1 << I2C_SR1_STOPF)
#define I2C_FLAG_TXE			 (1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE			 (1 << I2C_SR1_RXNE)
#define I2C_FLAG_BERR			 (1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO			 (1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF 			 (1 << I2C_SR1_AF)
#define I2C_FLAG_OVR			 (1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR			 (1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT		 (1 << I2C_SR1_TIMEOUT)

#define REPEATED_START_ENABLE	1
#define REPEATED_START_DISABLE	0
/*
 * Peripheral clock enable or disable API
 */
void I2C_PeriClock_Control(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);


/*
 * I2Cx init and De-init function
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * I2C send and receive data
 */
void I2C_MasterSenddata(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len, uint8_t SlaveAddress,uint8_t RepeatedStart);
void I2C_MasterReceivedata(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer,uint32_t Len, uint8_t SlaveAddress,uint8_t RepeatedStart);
/*
 * I2Cx IRQ configuration API's
 */
void I2C_IRQConfig(uint8_t IRQNumber,uint8_t EnorDi);
void I2C_IRQ_PriorityConfig(uint8_t IRQNumber,uint32_t IRQ_Priority);
void I2C_IRQHandling(I2C_Handle_t *pSPIHandle);

/*
 * I2C Get Status API's
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t flagread);

/*
 * other I2C peripheral API's
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);
void I2C_ManageACK(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);

/*
 * I2C peripheral reset macro's
 */
#define I2C1_REG_RST()			do { RCC->APB1RSTR |= (1 << 21); RCC->APB1RSTR &= ~(1 << 21); } while(0)
#define I2C2_REG_RST()			do { RCC->APB1RSTR |= (1 << 22); RCC->APB1RSTR &= ~(1 << 22); } while(0)
#define I2C3_REG_RST()			do { RCC->APB1RSTR |= (1 << 23); RCC->APB1RSTR &= ~(1 << 23); } while(0)

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
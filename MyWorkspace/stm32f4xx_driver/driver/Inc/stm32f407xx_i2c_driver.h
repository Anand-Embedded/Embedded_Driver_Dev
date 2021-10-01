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
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t Device_Addr;
	uint32_t RxSize;
	uint8_t Repeated_Start;
}I2C_Handle_t;


typedef enum
{
	I2C_STATE_IDLE,
	I2C_STATE_BUSY_IN_TX,
	I2C_STATE_BUSY_IN_RX
}I2C_State_e;


/*
 * I2C application events
 */
#define I2C_APP_EVT_TX_CMPLT	0
#define I2C_APP_EVT_RX_CMPLT	1
#define I2C_APP_EVT_STOP		2
#define I2C_ERROR_BERR  		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_SLAVE_REQ_DATA		8
#define I2C_SLAVE_REC_DATA		9



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

I2C_State_e I2C_MasterSenddataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t Len, uint8_t SlaveAddress,uint8_t RepeatedStart);
I2C_State_e I2C_MasterReceivedataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer,uint32_t Len, uint8_t SlaveAddress,uint8_t RepeatedStart);


/*
 * I2C slave send and receive API
 */

void I2C_SlaveSenddata(I2C_RegDef_t *pI2C,uint8_t data);
uint8_t I2C_SlaveReceivedata(I2C_RegDef_t *pI2C);

/*
 * I2Cx IRQ configuration API's
 */
void I2C_IRQConfig(uint8_t IRQNumber,uint8_t EnorDi);
void I2C_IRQ_PriorityConfig(uint8_t IRQNumber,uint32_t IRQ_Priority);
void I2C_Event_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_Error_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_SlavePeripheralInterrupt(I2C_RegDef_t *pI2C,uint8_t EnorDi);


/*
 * I2C Get Status API's
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t flagread);

/*
 * other I2C peripheral API's
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);
void I2C_ManageACK(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);
void I2C_CloseTranmission(I2C_Handle_t *pI2CHandle);
void I2C_CloseReception(I2C_Handle_t *pI2CHandle);




void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);

/*
 * I2C peripheral reset macro's
 */
#define I2C1_REG_RST()			do { RCC->APB1RSTR |= (1 << 21); RCC->APB1RSTR &= ~(1 << 21); } while(0)
#define I2C2_REG_RST()			do { RCC->APB1RSTR |= (1 << 22); RCC->APB1RSTR &= ~(1 << 22); } while(0)
#define I2C3_REG_RST()			do { RCC->APB1RSTR |= (1 << 23); RCC->APB1RSTR &= ~(1 << 23); } while(0)

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */

/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Sep 20, 2021
 *      Author: Admin
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f4xx.h"
/*
 * SPI Configuration structure definition
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_ClkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


/*
 *  SPI Handle structure definition
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t TxState;
	uint8_t RxState;
	uint32_t TxLen;
	uint32_t RxLen;
}SPI_Handle_t;

typedef enum
{
	SPI_STATE_IDLE,
	SPI_STATE_BUSY_IN_TX,
	SPI_STATE_BUSY_IN_RX
}SPI_State_e;

typedef enum
{
	SPI_APP_EVT_TX_CMPLT,
	SPI_APP_EVT_RX_CMPLT,
	SPI_APP_EVT_ERR_CMPLT
}SPI_Event_e;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

/*
 *@SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/*
 * @SPI_ClkSpeed
 */
#define SPI_CLK_SPEED_DIV2			0
#define SPI_CLK_SPEED_DIV4			1
#define SPI_CLK_SPEED_DIV8			2
#define SPI_CLK_SPEED_DIV16			3
#define SPI_CLK_SPEED_DIV32			4
#define SPI_CLK_SPEED_DIV64			5
#define SPI_CLK_SPEED_DIV128		6
#define SPI_CLK_SPEED_DIV256		7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BIT	0
#define SPI_DFF_16BIT	1


/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN	1
#define SPI_SSM_DI	0


/*
 * SPI Status Flag's
 */
#define SPI_TXE_FLAG   	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   (1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG   	(1 << SPI_SR_BSY)
#define SPI_MODF_FLAG   (1 << SPI_SR_MODF)
#define SPI_FRE_FLAG   	(1 << SPI_SR_FRE)
#define SPI_OVR_FLAG   	(1 << SPI_SR_OVR)


/*
 * Peripheral clock enable or disable API
 */
void SPI_PeriClock_Control(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

/*
 * SPIx init and De-init function
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseRecepition(SPI_Handle_t *pSPIHandle);
/*
 * SPI send and receive data API's
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len);
void SPI_SendandReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint8_t *pRxBuffer,uint32_t Len);

/*
 * SPI send and receive data with interrupt
 */
SPI_State_e SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len);
SPI_State_e SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len);

/*
 * SPI Get Status API's
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t flagread);


/*
 * other SPI peripheral API's
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);


/*
 * SPI IRQ configuration API's
 */
void SPI_IRQConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQ_PriorityConfig(uint8_t IRQNumber,uint32_t IRQ_Priority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Callback function
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,SPI_Event_e AppEV);
/*
 * SPI Reset Macro's
 */
#define SPI1_REG_RST()			do { RCC->APB2RSTR |= (1 << 12); RCC->APB2RSTR &= ~(1 << 12); } while(0)
#define SPI2_REG_RST()			do { RCC->APB1RSTR |= (1 << 14); RCC->APB1RSTR &= ~(1 << 14); } while(0)
#define SPI3_REG_RST()			do { RCC->APB1RSTR |= (1 << 15); RCC->APB1RSTR &= ~(1 << 15); } while(0)
#define SPI4_REG_RST()			do { RCC->APB2RSTR |= (1 << 13); RCC->APB2RSTR &= ~(1 << 13); } while(0)
#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */

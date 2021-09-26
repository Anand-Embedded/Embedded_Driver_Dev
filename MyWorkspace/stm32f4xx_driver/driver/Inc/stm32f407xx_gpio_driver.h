/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Sep 8, 2021
 *      Author: Admin
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f4xx.h"


/*
 * This structure hold the configuration details for GPIO pins
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			/*!< Possible value from @GPIO_PIN_NUMBER */
	uint8_t GPIO_PinMode;			/*!< Possible value from @GPIO_PIN_MODE */
	uint8_t GPIO_PinSpeed;			/*!< Possible value from @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;	/*!< Possible value from @GPIO_PIN_PUPD */
	uint8_t GPIO_PinOPType;			/*!< Possible value from @GPIO_PIN_OUT_CONFIG */
	uint8_t	GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/*
 * This is a handle structure to GPIO pin
 */

typedef struct
{
	GPIOx_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBER
 * GPIO Pin Numbers
 */
#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12		12
#define GPIO_PIN_13		13
#define GPIO_PIN_14		14
#define GPIO_PIN_15		15


/*
 *@GPIO_PIN_MODE
 *GPIO pin mode
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_RT		4
#define GPIO_MODE_IT_FT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_PIN_SPEED
 *GPIO pin speed
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_HIGH			2
#define GPIO_SPEED_VERY_HIGH	3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin PUSH-PULL
 */
#define GPIO_PUPD_NOPUPD		0
#define GPIO_PUPD_PU			1
#define GPIO_PUPD_PD			2

/*
 * @GPIO_PIN_OUT_CONFIG
 * GPIO pin Output configuration
 */
#define GPIO_PIN_OUT_PUPL		0
#define GPIO_PIN_OUT_OD			1

/*
 *
 */
#define GPIOA_REG_RST()			do { RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOB_REG_RST()			do { RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1); } while(0)
#define GPIOC_REG_RST()			do { RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2); } while(0)
#define GPIOD_REG_RST()			do { RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3); } while(0)
#define GPIOE_REG_RST()			do { RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4); } while(0)
#define GPIOF_REG_RST()			do { RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5); } while(0)
#define GPIOG_REG_RST()			do { RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6); } while(0)
#define GPIOH_REG_RST()			do { RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7); } while(0)
#define GPIOI_REG_RST()			do { RCC->AHB1RSTR |= (1 << 8); RCC->AHB1RSTR &= ~(1 << 8); } while(0)


/*
 *
 */
#define PORTCODE_FROM_GPIO_PORTREG(x) ((x == GPIOA) ? 0 :\
									  (x == GPIOB) ? 1 :\
									  (x == GPIOC) ? 2 :\
									  (x == GPIOD) ? 3 :\
									  (x == GPIOE) ? 4 :\
									  (x == GPIOF) ? 5 :\
									  (x == GPIOG) ? 6 :\
									  (x == GPIOH) ? 7 :\
									  (x == GPIOI) ? 8 : 0)
/*
 * GPIO Driver level API's declaration
 */

/*
 * GPIO Peripheral clock control
 */
void GPIO_PeriClock_Control(GPIOx_RegDef_t *pGPIOx,uint8_t EnorDi);

/*
 * GPIO init and De-init function
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIOx_RegDef_t *pGPIOx);

/*
 * GPIO write and read from Pin and Port
 */
uint8_t GPIO_ReadFromInputPin(GPIOx_RegDef_t *pGPIOx,uint16_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIOx_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIOx_RegDef_t *pGPIOx,uint16_t PinNumber,uint8_t value);
void GPIO_WriteToOutputPort(GPIOx_RegDef_t *pGPIOx,uint8_t value);
void GPIO_PinToggle(GPIOx_RegDef_t *pGPIOx,uint16_t PinNumber);
/*
 * GPIO IRQ Configuration
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQ_PriorityConfig(uint8_t IRQNumber,uint32_t IRQ_Priority);
void GPIO_IRQHandling(uint16_t PinNumber);






























#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */

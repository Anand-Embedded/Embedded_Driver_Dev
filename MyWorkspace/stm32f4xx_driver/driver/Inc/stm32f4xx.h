/*
 * stm32f4xx.h
 *
 *  Created on: Sep 7, 2021
 *      Author: Admin
 */

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#define __vo 		volatile
#define __weak 		__attribute__ ((weak))
/*
 * Processor specific declaration's starts from here
 */

/*
 *  NVIC Interrupt Set-Enable Registers
 */
#define NVIC_ISER0			((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t *)0xE000E10C)


/*
 *  NVIC Interrupt Clear-Enable Registers
 */
#define NVIC_ICER0			((__vo uint32_t *)0xE000E180)
#define NVIC_ICER1			((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2			((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3			((__vo uint32_t *)0xE000E18C)



/*
 *  NVIC Interrupt Priority Register
 */
#define NVIC_IPRN			((__vo uint32_t *)0xE000E400)


/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     	31
#define IRQ_NO_I2C1_ER     	32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15




/*
 * Base address of Flash and SRAM Memory
 */
#define FLASH_BASEADDR			0x08000000U		/*! Base address of the Flash memory */
#define SRAM1_BASEADDR			0x20000000U		/*! Base address of the SRAM1 memory */
#define SRAM2_BASEADDR			0x2001C000U		/*! Base address of the SRAM2 memory */
#define SRAM_BASEADDR			SRAM1_BASEADDR	/*! Base address of the SRAM memory */
#define ROM_BASEADDR			0x1FFF0000U		/*! Base address of the ROM memory */
#define OTP_BASEADDR			0x1FFF7800U		/*! Base address of the OTP memory */
#define OPTION_BYTE_BASEADDR	0x1FFFC000U		/*! Base address of the Option byte memory */


/*
 * Base address of AHBx and APBx bus
 */
#define PERI_BASEADDR			0X40000000U		/*! Base address of the Peripheral */
#define APB1_BASEADDR			PERI_BASEADDR	/*! Base address of the APB1 */
#define APB2_BASEADDR			0x40010000U		/*! Base address of the APB2 */
#define AHB1_BASEADDR			0x40020000U		/*! Base address of the AHB1 */
#define AHB2_BASEADDR			0x50000000U		/*! Base address of the AHB2 */
#define AHB3_BASEADDR			0xA0000000U		/*! Base address of the AHB3 */


/*
 * Base address of peripheral which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR			(AHB1_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR			(AHB1_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR			(AHB1_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR			(AHB1_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR			(AHB1_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR			(AHB1_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR			(AHB1_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR			(AHB1_BASEADDR + 0x1C00U)
#define GPIOI_BASEADDR			(AHB1_BASEADDR + 0x2000U)

#define RCC_BASEADDR			(AHB1_BASEADDR + 0x3800U)

/*
 * Base address of peripheral which are hanging on APB1 bus
 */

#define I2C1_BASEADDR			(APB1_BASEADDR + 0x5400U)
#define I2C2_BASEADDR			(APB1_BASEADDR + 0x5800U)
#define I2C3_BASEADDR			(APB1_BASEADDR + 0x5C00U)

#define SPI2_BASEADDR			(APB1_BASEADDR + 0x3800U)
#define SPI3_BASEADDR			(APB1_BASEADDR + 0x3C00U)


#define USART2_BASEADDR			(APB1_BASEADDR + 0x4400U)
#define USART3_BASEADDR			(APB1_BASEADDR + 0x4800U)
#define UART4_BASEADDR			(APB1_BASEADDR + 0x4C00U)
#define UART5_BASEADDR			(APB1_BASEADDR + 0x5000U)


/*
 * Base address of peripheral which are hanging on APB2 bus
 */
#define SPI1_BASEADDR			(APB2_BASEADDR + 0x3000U)
#define SPI4_BASEADDR			(APB2_BASEADDR + 0x3400U)

#define USART1_BASEADDR			(APB2_BASEADDR + 0x1000U)
#define USART6_BASEADDR			(APB2_BASEADDR + 0x1400U)


#define EXTI_BASEADDR			(APB2_BASEADDR + 0x3C00U)
#define SYSCFG_BASEADDR			(APB2_BASEADDR + 0x3800U)


/*
 * GPIOx register of peripheral
 */

typedef struct
{
	__vo uint32_t MODER;			/*GPIO port mode register,					ADDRESS OFFSET 0x00*/
	__vo uint32_t OTYPER;			/*GPIO port output type register,			ADDRESS OFFSET 0x04*/
	__vo uint32_t OSPEEDR;			/*GPIO port output speed register,			ADDRESS OFFSET 0x08*/
	__vo uint32_t PUPDR;			/*GPIO port pull-up/pull-down register,		ADDRESS OFFSET 0x0C*/
	__vo uint32_t IDR;				/*GPIO port input data register,			ADDRESS OFFSET 0x10*/
	__vo uint32_t ODR;				/*GPIO port output data register,			ADDRESS OFFSET 0x14*/
	__vo uint32_t BSRR;				/*GPIO port bit set/reset register,			ADDRESS OFFSET 0x18*/
	__vo uint32_t LCKR;				/*GPIO port configuration lock register,	ADDRESS OFFSET 0x1C*/
	__vo uint32_t AFR[2];			/*AFR[0] : GPIO alternate function low register  AFR[1] : GPIO alternate function high register,	ADDRESS OFFSET 0x20*/
}GPIOx_RegDef_t;


/*
 * RCC register of peripheral
 */

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

/*
 * Peripheral structure definition for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;


/*
 * Peripheral structure definition for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;


/*
 * Peripheral structure definition for SPI
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;


/*
 * Peripheral structure definition for I2C
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;


/*
 * Peripheral Definition Macro
 */

#define GPIOA 		((GPIOx_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB 		((GPIOx_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC 		((GPIOx_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD 		((GPIOx_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE 		((GPIOx_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF 		((GPIOx_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG 		((GPIOx_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH 		((GPIOx_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI 		((GPIOx_RegDef_t *)GPIOI_BASEADDR)

#define RCC			((RCC_RegDef_t *)RCC_BASEADDR)
#define EXTI 		((EXTI_RegDef_t *)EXTI_BASEADDR)
#define SYSCFG		((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t *) SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t *) SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t *) SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t *) SPI4_BASEADDR)

#define I2C1		((I2C_RegDef_t *) I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t *) I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t *) I2C3_BASEADDR)


/***********************************************************************************
 * Bit position definition macros for SPI peripherals
 ***********************************************************************************/

/*
 * Bit position definition for SPI_CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSB_FIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RX_ONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRC_NEXT	12
#define SPI_CR1_CRC_EN		13
#define SPI_CR1_BIDI_EN		14
#define SPI_CR1_BIDI_MODE	15


/*
 * Bit position definition for SPI_CR2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * Bit position definition for SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/***********************************************************************************
 * Bit position definition macros for I2C peripherals
 ***********************************************************************************/

/*
 * Bit position definition for I2C_CR1 register
 */
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NO_STRECH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

/*
 * Bit position definition for I2C_CR2 register
 */
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

/*
 * Bit position definition for I2C_SR1 register
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15


/*
 * Bit position definition for I2C_SR2 register
 */
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8

/*
 * Bit position definition for I2C_CCR register
 */
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15

/*
 * GPIOx Clock enable macro's
 */
#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1 << 8))

/*
 * SPI Clock enable macro
 */

#define SPI1_PCLK_EN()			RCC->APB2ENR |= (1 << 12)
#define SPI2_PCLK_EN()			RCC->APB1ENR |= (1 << 14)
#define SPI3_PCLK_EN()			RCC->APB1ENR |= (1 << 15)
#define SPI4_PCLK_EN()			RCC->APB2ENR |= (1 << 13)


/*
 * I2C Clock enable macro's
 */
#define I2C1_PCLK_EN()			RCC->APB1ENR |= (1 << 21)
#define I2C2_PCLK_EN()			RCC->APB1ENR |= (1 << 22)
#define I2C3_PCLK_EN()			RCC->APB1ENR |= (1 << 23)

/*
 * USART Clock enable macro
 */
#define USART1_PCLK_EN()		RCC->APB2ENR |= (1 << 4)
#define USART2_PCLK_EN()		RCC->APB1ENR |= (1 << 17)
#define USART3_PCLK_EN()		RCC->APB1ENR |= (1 << 18)
#define UART4_PCLK_EN()			RCC->APB1ENR |= (1 << 19)
#define UART5_PCLK_EN()			RCC->APB1ENR |= (1 << 20)

/*
 * SYSCFG Clock enable Macro's
 */
#define SYSCFG_PCLK_EN()		RCC->APB2ENR |= (1 << 14)

/*
 * GPIOx Clock disable macro's
 */
#define GPIOA_PCLK_DI()			RCC->AHB1ENR &= ~(1 << 0)
#define GPIOB_PCLK_DI()			RCC->AHB1ENR &= ~(1 << 1)
#define GPIOC_PCLK_DI()			RCC->AHB1ENR &= ~(1 << 2)
#define GPIOD_PCLK_DI()			RCC->AHB1ENR &= ~(1 << 3)
#define GPIOE_PCLK_DI()			RCC->AHB1ENR &= ~(1 << 4)
#define GPIOF_PCLK_DI()			RCC->AHB1ENR &= ~(1 << 5)
#define GPIOG_PCLK_DI()			RCC->AHB1ENR &= ~(1 << 6)
#define GPIOH_PCLK_DI()			RCC->AHB1ENR &= ~(1 << 7)
#define GPIOI_PCLK_DI()			RCC->AHB1ENR &= ~(1 << 8)


/*
 * SPI Clock disable macro
 */

#define SPI1_PCLK_DI()			RCC->APB2ENR &= ~(1 << 12)
#define SPI2_PCLK_DI()			RCC->APB1ENR &= ~(1 << 14)
#define SPI3_PCLK_DI()			RCC->APB1ENR &= ~(1 << 15)
#define SPI4_PCLK_DI()			RCC->APB2ENR &= ~(1 << 13)


/*
 * I2C Clock disable macro's
 */
#define I2C1_PCLK_DI()			RCC->APB1ENR &= ~(1 << 21)
#define I2C2_PCLK_DI()			RCC->APB1ENR &= ~(1 << 22)
#define I2C3_PCLK_DI()			RCC->APB1ENR &= ~(1 << 23)


/*
 * USART Clock disable macro
 */
#define USART1_PCLK_DI()		RCC->APB2ENR &= ~(1 << 4)
#define USART2_PCLK_DI()		RCC->APB1ENR &= ~(1 << 17)
#define USART3_PCLK_DI()		RCC->APB1ENR &= ~(1 << 18)
#define UART4_PCLK_DI()			RCC->APB1ENR &= ~(1 << 19)
#define UART5_PCLK_DI()			RCC->APB1ENR &= ~(1 << 20)

/*
 * SYSCFG Clock enable Macro's
 */
#define SYSCFG_PCLK_DI()		RCC->APB2ENR &= ~(1 << 14)

/*
 * Some generic Macros
 */

#define ENABLE 			1
#define DISABLE			0
#define	SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#define FLAG_RESET		RESET
#define FLAG_SET		SET


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"

#endif /* INC_STM32F4XX_H_ */

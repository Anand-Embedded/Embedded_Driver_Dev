/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Sep 8, 2021
 *      Author: Admin
 */


#include "stm32f407xx_gpio_driver.h"


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClock_Control(GPIOx_RegDef_t *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * GPIO init and De-init function
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0 ;

	//Enable the GPIO peripheral clock
	GPIO_PeriClock_Control(pGPIOHandle->pGPIOx, ENABLE);

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN)
	{
		//Non-interrupt functionality
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else
	{
		//Alternate functionality
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//Enable the Falling edge interrupt and clear the rising edge interrupt
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//Enable the Rising edge interrupt and clear the falling edge interrupt
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//Enable the Rising edge interrupt and falling edge interrupt
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//Enable the respective port for the pin number to NVIC
		SYSCFG_PCLK_EN();
		uint8_t index = 0;
		uint8_t pos	= 0 ;
		uint8_t portcode = PORTCODE_FROM_GPIO_PORTREG(pGPIOHandle->pGPIOx);
		index = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		pos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		SYSCFG->EXTICR[index] |= (portcode << (4 * pos));


		//Enable the GPIO peripheral interrupt
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t Reg_index = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 8;
		uint8_t Bit_pos	= (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 8;

		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode;
		pGPIOHandle->pGPIOx->AFR[Reg_index] |= (temp << (4 * Bit_pos) );
		temp = 0;
	}

}
void GPIO_DeInit(GPIOx_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RST();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RST();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RST();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RST();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RST();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RST();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RST();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RST();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RST();
	}
}

/*
 * GPIO write and read from Pin and Port
 */
uint8_t GPIO_ReadFromInputPin(GPIOx_RegDef_t *pGPIOx,uint16_t PinNumber)
{
	uint8_t status = 0;
	status = (((pGPIOx->IDR) >> PinNumber) & 0x1);
	return status;
}


uint16_t GPIO_ReadFromInputPort(GPIOx_RegDef_t *pGPIOx)
{
	return (uint16_t)pGPIOx->IDR;
}

void GPIO_WriteToOutputPin(GPIOx_RegDef_t *pGPIOx,uint16_t PinNumber,uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIOx_RegDef_t *pGPIOx,uint8_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_PinToggle(GPIOx_RegDef_t *pGPIOx,uint16_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber );
}

/*
 * GPIO IRQ Configuration
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if((IRQNumber > 31) && (IRQNumber < 64))
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if((IRQNumber >= 64) && (IRQNumber < 96))
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if((IRQNumber > 31) && (IRQNumber < 64))
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if((IRQNumber >= 64) && (IRQNumber < 96))
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}


void GPIO_IRQ_PriorityConfig(uint8_t IRQNumber,uint32_t IRQ_Priority)
{
	uint8_t index = 0;
	uint8_t Pos	= 0;
	uint8_t Bit_width = 8;

	index = IRQNumber / 4;
	Pos = IRQNumber % 4;

	*(NVIC_IPRN + index) = ((IRQ_Priority << 4) << (Bit_width * Pos));

}

void GPIO_IRQHandling(uint16_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		//Clear the pending interrupt from peripheral side ,
		//Processor side pending interrupt bit will get clear once the Interrupt is started its execution
		EXTI->PR |= (1 << PinNumber);
	}
}

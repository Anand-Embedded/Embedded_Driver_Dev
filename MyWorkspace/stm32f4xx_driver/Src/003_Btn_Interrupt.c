/*
 * 001LedToggle.c
 *
 *  Created on: Sep 11, 2021
 *      Author: Admin
 */
#include "stm32f4xx.h"

#define HIGH 1
#define BUTTON_PRESSED HIGH

void GPIO_init_Led_Toggle(void);
void GPIO_init_Button(void);

void delay(void)
{
	for (uint32_t i = 0;i < 500000/2 ;i++);
}

int main(void)
{
	GPIO_init_Led_Toggle();
	GPIO_init_Button();
	while(1)
	{
//		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) == BUTTON_PRESSED)
//		{
//			delay();
//			GPIO_PinToggle(GPIOD, GPIO_PIN_12);
//		}
	}

	return 0;
}


void GPIO_init_Led_Toggle(void)
{
	GPIO_Handle_t GpioLed;
	memset(&GpioLed,0,sizeof(GpioLed));

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_PIN_OUT_PUPL;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NOPUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_VERY_HIGH;

	//Enable the peripheral clock
	GPIO_PeriClock_Control(GPIOD, ENABLE);

	//Init the GPIO
	GPIO_Init(&GpioLed);

}


void GPIO_init_Button(void)
{

	GPIO_Handle_t GpioBtn;
	memset(&GpioBtn,0,sizeof(GpioBtn));

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NOPUPD;

	//Enable the peripheral clock
	GPIO_PeriClock_Control(GPIOA, ENABLE);

	//Init the GPIO
	GPIO_Init(&GpioBtn);

	//Configure the IRQ priority in NVIC
	GPIO_IRQ_PriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);

	//Enable the IRQ in NVIC
	GPIO_IRQConfig(IRQ_NO_EXTI0, ENABLE);

}


void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_0);
	GPIO_PinToggle(GPIOD, GPIO_PIN_12);
}

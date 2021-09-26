/*
 * 001LedToggle.c
 *
 *  Created on: Sep 11, 2021
 *      Author: Admin
 */
#include "stm32f4xx.h"

void GPIO_init_Led_Toggle(void);

void delay(void)
{
	for (uint32_t i = 0;i < 500000/2 ;i++);
}

int main(void)
{
	GPIO_init_Led_Toggle();

	while(1)
	{
		GPIO_PinToggle(GPIOD, GPIO_PIN_12);
		delay();
	}

	return 0;
}


void GPIO_init_Led_Toggle(void)
{
	GPIO_Handle_t GpioLed;

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

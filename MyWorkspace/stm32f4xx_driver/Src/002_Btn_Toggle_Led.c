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
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) == BUTTON_PRESSED)
		{
			delay();
			GPIO_PinToggle(GPIOD, GPIO_PIN_12);
		}
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


void GPIO_init_Button(void)
{

	GPIO_Handle_t GpioBtn;

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NOPUPD;

	//Enable the peripheral clock
	GPIO_PeriClock_Control(GPIOA, ENABLE);

	//Init the GPIO
	GPIO_Init(&GpioBtn);
}

/*
 * 004_spi_senddata.c
 *
 *  Created on: Sep 23, 2021
 *      Author: Admin
 */



#include "stm32f4xx.h"
#include <string.h>

#define HIGH 1
#define BUTTON_PRESSED HIGH


void SPI2_GpioInit(void);
void SPI2_Init(void);
void GPIO_ButtonInit(void);

void delay(void)
{
	for (uint32_t i = 0;i < 500000 ;i++);
}


int main(void)
{
	char usr_msg[] = "Interfacing Pulse Sensor with Arduino - Step by Step Guide with Code";

	//Initialize and configure the GPIO pin as input for button
	GPIO_ButtonInit();

	//Configure the GPIO pins for SPI functionality
	SPI2_GpioInit();

	//Initialize the SPI peripheral as Master mode
	SPI2_Init();

	//@note, to behave NSS as output in master mode SSM = 0 and SSOE = 1(configure NSS as output)
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		//Wait until user press the button
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay();

		//Enable the SPI peripheral, this makes the NSS pin to Low
		SPI_PeripheralControl(SPI2,ENABLE);

		uint8_t len = strlen(usr_msg);

		SPI_SendData(SPI2, &len, 1);

		//Send the data to Arduino UNO board
		SPI_SendData(SPI2, (uint8_t *)&usr_msg[0], strlen(usr_msg));

		//wait until SPI become free
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		//Disable the SPI peripheral, this makes the NSS pin to High
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}

/*
 * PB12 -> SPI2_NSS
 * PB13 -> SPI2_SCK
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 * Alternate functionality 5
 */
void SPI2_GpioInit(void)
{
	GPIO_Handle_t SPIpins = {0};

	SPIpins.pGPIOx = GPIOB;

	SPIpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIpins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_PIN_OUT_PUPL;
	SPIpins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NOPUPD;
	SPIpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	SPIpins.GPIO_PinConfig.GPIO_PinNumber = (  GPIO_PIN_13); //GPIO_PIN_12 | GPIO_PIN_14 |
	//Peripheral clock will be enabled inside the Gpio_init function
	GPIO_Init(&SPIpins);

	SPIpins.GPIO_PinConfig.GPIO_PinNumber = (  GPIO_PIN_15); //GPIO_PIN_12 | GPIO_PIN_14 |
	//Peripheral clock will be enabled inside the Gpio_init function
	GPIO_Init(&SPIpins);

	SPIpins.GPIO_PinConfig.GPIO_PinNumber = (  GPIO_PIN_12); //GPIO_PIN_12 | GPIO_PIN_14 |
	//Peripheral clock will be enabled inside the Gpio_init function
	GPIO_Init(&SPIpins);
}


void SPI2_Init(void)
{
	SPI_Handle_t SPI2Config = {0};

	SPI2Config.pSPIx = SPI2;
	SPI2Config.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Config.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Config.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Config.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Config.SPI_Config.SPI_DFF = SPI_DFF_8BIT;
	SPI2Config.SPI_Config.SPI_ClkSpeed = SPI_CLK_SPEED_DIV8; // Configure SPI clk for 2MHz. since (PCLK = 16MHz)
	SPI2Config.SPI_Config.SPI_SSM = SPI_SSM_DI;//Using Hardware slave select management

	SPI_Init(&SPI2Config);
}

void GPIO_ButtonInit(void)
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

/*
 * 004_spi_senddata.c
 *
 *  Created on: Sep 23, 2021
 *      Author: Admin
 */



#include "stm32f4xx.h"
#include <string.h>


void SPI2_GpioInit(void);
void SPI2_Init(void);

int main(void)
{
	char usr_msg[] = "Hello world from Anand";

	SPI2_GpioInit();
	SPI2_Init();

	//Enable the SSI bit to high, to avoid MODF error,this error occured for multi-master error(if SPI is
	//in master mode, NSS if pull to low indicates multi-master collision error)
	SPI_SSIConfig(SPI2, ENABLE);

	SPI_PeripheralControl(SPI2,ENABLE);

	SPI_SendData(SPI2, (uint8_t *)usr_msg, strlen(usr_msg));

	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

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
	SPI2Config.SPI_Config.SPI_ClkSpeed = SPI_CLK_SPEED_DIV2;
	SPI2Config.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2Config);
}


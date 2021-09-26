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

#define NACK 0xA5
#define ACK 0xF5


//command codes
#define COMMAND_LED_CTRL          	0x50
#define COMMAND_SENSOR_READ       	0x51
#define COMMAND_LED_READ          	0x52
#define COMMAND_PRINT           	0x53
#define COMMAND_ID_READ         	0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

#define LED_PIN 	9

void SPI2_GpioInit(void);
void SPI2_Init(void);
void GPIO_ButtonInit(void);
uint8_t SPI_Verifyresponse(uint8_t ackbyte);

void delay(void)
{
	for (uint32_t i = 0;i < 500000 ;i++);
}


int main(void)
{

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read = 0xff;
	uint8_t cmd_code = 0;
	uint8_t ackbyte = 0;
	uint8_t arg[2] = {0};

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
		dummy_write = 0xff;
		dummy_read = 0xff;

		//Wait until user press the button
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		//Introduced delay to avoid the button de-bouncing
		delay();

		//Enable the SPI peripheral, this makes the NSS pin to Low
		SPI_PeripheralControl(SPI2,ENABLE);

		//1. send the command led and arg
		cmd_code = COMMAND_LED_CTRL;
#if 0
		//Send command code
		SPI_SendData(SPI2, &cmd_code, 1);
		//To clear the RXNE bit , this avoid overrun error in SPI
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Read the Ackbyte
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		//check the ackbyte , used to confirm slave can support the command
		if(SPI_Verifyresponse(ackbyte))
		{
			arg[0] = LED_PIN;
			arg[1] ^= LED_ON;

			//Send the Arg
			SPI_SendData(SPI2, arg, 2);
			SPI_ReceiveData(SPI2, &dummy_read, 1);
		}
#else
		//Send command code
		SPI_SendandReceiveData(SPI2, &cmd_code, &dummy_read, 1);

		//Read the Ackbyte
		SPI_SendandReceiveData(SPI2, &dummy_write, &ackbyte, 1);

		//check the ackbyte , used to confirm slave can support the command
		if(SPI_Verifyresponse(ackbyte))
		{
			arg[0] = LED_PIN;
			arg[1] ^= LED_ON;

			//Send the Arg
			SPI_SendandReceiveData(SPI2, arg, &dummy_read, 2);
		}
#if 1
		//2. CMD_SENOSR_READ   <analog pin number(1) >
		//Wait until user press the button
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		//Introduced delay to avoid the button de-bouncing
		delay();

		//1. send the command led and arg
		cmd_code = COMMAND_SENSOR_READ;

		//Send command code
		SPI_SendandReceiveData(SPI2, &cmd_code, &dummy_read, 1);

		//Read the Ackbyte
		SPI_SendandReceiveData(SPI2, &dummy_write, &ackbyte, 1);

		//check the ackbyte , used to confirm slave can support the command
		if(SPI_Verifyresponse(ackbyte))
		{
			arg[0] = ANALOG_PIN0;

			//Send the Arg
			SPI_SendandReceiveData(SPI2, arg, &dummy_read, 1);

			//this delay used for slave to complete the ADC conversion
			for(uint32_t i = 0;i < 50000/2 ;i++);

			uint8_t adc_value = 0;
			//Send command code
			SPI_SendandReceiveData(SPI2, &adc_value, &dummy_read, 1);

		}

		//3. CMD_PRINT 		<len(2)>  <message(len) >
		//Wait until user press the button
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		//Introduced delay to avoid the button de-bouncing
		delay();

		//1. send the command led and arg
		cmd_code = COMMAND_PRINT;

		//Send command code
		SPI_SendandReceiveData(SPI2, &cmd_code, &dummy_read, 1);

		//Read the Ackbyte
		SPI_SendandReceiveData(SPI2, &dummy_write, &ackbyte, 1);

		//check the ackbyte , used to confirm slave can support the command
		if(SPI_Verifyresponse(ackbyte))
		{
			char usr_msg[] = "Hello world from STM32 by Anand";
			arg[0] = strlen(usr_msg);

			//Send the Arg
			SPI_SendandReceiveData(SPI2, arg, &dummy_read, 1);

			//for loop is introduced due to dummy array increment to may lead to exception
			for(uint32_t i = 0; i < arg[0] ; i++ )
				SPI_SendandReceiveData(SPI2, (uint8_t *)&usr_msg[i], &dummy_read, 1);

		}

		//4. CMD_ID_READ
		//Wait until user press the button
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		//Introduced delay to avoid the button de-bouncing
		delay();

		//1. send the command led and arg
		cmd_code = COMMAND_ID_READ;

		//Send command code
		SPI_SendandReceiveData(SPI2, &cmd_code, &dummy_read, 1);

		//Read the Ackbyte
		SPI_SendandReceiveData(SPI2, &dummy_write, &ackbyte, 1);

		//check the ackbyte , used to confirm slave can support the command
		if(SPI_Verifyresponse(ackbyte))
		{
			char id[20] = {0};
			//for loop is introduced due to dummy array increment to may lead to exception
			for(uint32_t i = 0; i < 10 ; i++ )
				SPI_SendandReceiveData(SPI2, &dummy_write, (uint8_t *)&id[i], 1);

			id[10] = '\0';
		}
#endif

#endif
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

	SPIpins.GPIO_PinConfig.GPIO_PinNumber = (  GPIO_PIN_13);
	//Peripheral clock will be enabled inside the Gpio_init function
	GPIO_Init(&SPIpins);

	SPIpins.GPIO_PinConfig.GPIO_PinNumber = (  GPIO_PIN_15);
	//Peripheral clock will be enabled inside the Gpio_init function
	GPIO_Init(&SPIpins);

	SPIpins.GPIO_PinConfig.GPIO_PinNumber = (  GPIO_PIN_12);
	//Peripheral clock will be enabled inside the Gpio_init function
	GPIO_Init(&SPIpins);

	SPIpins.GPIO_PinConfig.GPIO_PinNumber = (  GPIO_PIN_14);
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

	//Init the GPIO
	GPIO_Init(&GpioBtn);
}


uint8_t SPI_Verifyresponse(uint8_t ackbyte)
{
	if(ackbyte == ACK)
	{
		return 1;
	}
	return 0;
}

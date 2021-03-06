/*
 * 004_spi_senddata.c
 *
 *  Created on: Sep 23, 2021
 *      Author: Admin
 */



#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>

#define HIGH 1
#define BUTTON_PRESSED HIGH

#define MY_ADDR 		0x61
#define MY_SLAVE_ADDR	0x68

#define DEBUG_LOG_CODE		DISABLE


I2C_Handle_t I2C_Config;

void GPIO_ButtonInit(void);
void I2C1_GpioInit(void);
void I2C1_Init(void);
extern uint8_t RxCmplt;
extern void initialise_monitor_handles();

void delay(void)
{
	for (uint32_t i = 0;i < 500000 ;i++);
}


int main(void)
{
	char usr_msg[100] = {0};
	uint8_t command_code = 0;
	uint8_t Len = 0;


	initialise_monitor_handles();
	//Initialize and configure the GPIO pin as input for button
	GPIO_ButtonInit();

	I2C1_GpioInit();

	I2C1_Init();

	//enable the I2C NVIC interrupt
	I2C_IRQConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_PeripheralControl(I2C1, ENABLE);

	//This bit is set and cleared by software and cleared by hardware when PE=0. so after PE=1 in above statement
	I2C_ManageACK(I2C_Config.pI2Cx, ENABLE);


	while(1)
	{
		//Wait until user press the button
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay();
		//printf(" I2C starts...\n");
		command_code = 0x51;// Used to Len information

		I2C_MasterSenddataIT(&I2C_Config,&command_code ,1, MY_SLAVE_ADDR,REPEATED_START_ENABLE);
		while(I2C_Config.TxRxState != I2C_STATE_IDLE);

		I2C_MasterReceivedataIT(&I2C_Config, &Len, 1, MY_SLAVE_ADDR,REPEATED_START_ENABLE);
		while(I2C_Config.TxRxState != I2C_STATE_IDLE);

		command_code = 0x52;// Used to Len information
		I2C_MasterSenddataIT(&I2C_Config,&command_code ,1, MY_SLAVE_ADDR,REPEATED_START_ENABLE);
		while(I2C_Config.TxRxState != I2C_STATE_IDLE);

		I2C_MasterReceivedataIT(&I2C_Config, (uint8_t *)&usr_msg[0], Len, MY_SLAVE_ADDR,REPEATED_START_DISABLE);
		while(I2C_Config.TxRxState != I2C_STATE_IDLE);

		usr_msg[Len +1]= '\0';
#if 1
		//while(!RxCmplt);
		printf("%s\n",usr_msg);
#endif
	}


	return 0;
}

/*
 * PB6 -> I2C1_SCL
 * PB7 -> I2C1_SDA
 * Alternate functionality 4
 */
void I2C1_GpioInit(void)
{
	GPIO_Handle_t I2Cpins = {0};

	I2Cpins.pGPIOx = GPIOB;
	I2Cpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2Cpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_PIN_OUT_OD;
	I2Cpins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;
	I2Cpins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2Cpins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//scl
	I2Cpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2Cpins);

	//sda
	I2Cpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2Cpins);

}


void I2C1_Init(void)
{


	I2C_Config.pI2Cx =  I2C1;
	I2C_Config.pI2C_Config.I2C_ACKCtrl = I2C_ACK_ENABLE;
	I2C_Config.pI2C_Config.I2C_DeviceAddr = MY_ADDR;
	I2C_Config.pI2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C_Config.pI2C_Config.I2C_SCLSpeed = I2C_SPEED_STD_MODE;

	I2C_Init(&I2C_Config);
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


void I2C1_EV_IRQHandler(void)
{
	I2C_Event_IRQHandling(&I2C_Config);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_Error_IRQHandling(&I2C_Config);
}

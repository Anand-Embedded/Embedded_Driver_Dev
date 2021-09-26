/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Sep 20, 2021
 *      Author: Admin
 */
#include "stm32f407xx_spi_driver.h"


/*
 * Private function declaration
 */
void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*
 * Peripheral clock enable or disable API
 */
void SPI_PeriClock_Control(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/*
 * SPIx init and De-init function
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t temreg = 0;

	//Enable the SPI peripheral clock
	SPI_PeriClock_Control(pSPIHandle->pSPIx, ENABLE);

	//Configure SPI as Mater
	temreg = (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//Clear the BIDI mode to configure SPI as full-Duplex communication
		temreg &= ~(1 << SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//Set the BIDI mode to configure the SPI as half-duplex communication
		temreg |= (1 << SPI_CR1_BIDI_MODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//Clear the BIDI mode to configure SPI as full-Duplex communication
		temreg &= ~(1 << SPI_CR1_BIDI_MODE);

		//Set the RXONLY bit to configure SPI as simplex receive only mode
		temreg |= (1 << SPI_CR1_RX_ONLY);
	}

	//Configure SPI data frame format
	temreg |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	//Configure SPI CPOL
	temreg |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	//Configure SPI CPHA
	temreg |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	//Configure SPI clock speed
	temreg |= (pSPIHandle->SPI_Config.SPI_ClkSpeed << SPI_CR1_BR);

	//Configure SPI software slave management
	temreg |= (pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = temreg;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RST();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RST();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RST();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RST();
	}
}



/*
 * SPI send and receive data API's
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len)
{
	while(Len > 0)
	{
		//wait until TXE flag to set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) ==  FLAG_RESET);

		//Check DFF for 8bit or 16bit data format
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//write 16bit data into SPI Data register
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			//write 8bit data into SPI Data register
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len)
{
	while(Len > 0)
	{
		//wait until TXE flag to set
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) ==  FLAG_RESET);

		//Check DFF for 8bit or 16bit data format
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//write 16bit data into SPI Data register
			*((uint16_t *)pRxBuffer) =  pSPIx->DR;
			Len--;
			Len--;
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			//write 8bit data into SPI Data register
			*pRxBuffer =  pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

void SPI_SendandReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint8_t *pRxBuffer,uint32_t Len)
{
	while(Len > 0)
	{
		//wait until TXE flag to set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) ==  FLAG_RESET);

		//Check DFF for 8bit or 16bit data format
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//write 16bit data into SPI Data register
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			//write 8bit data into SPI Data register
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

		//wait until RXNE flag to set
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) ==  FLAG_RESET);

		//Check DFF for 8bit or 16bit data format
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//write 16bit data into SPI Data register
			*((uint16_t *)pRxBuffer) =  pSPIx->DR;
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			//write 8bit data into SPI Data register
			*pRxBuffer =  pSPIx->DR;
			pRxBuffer++;
		}

	}
}


/*
 * SPI send and receive data with interrupt
 */
SPI_State_e SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len)
{
	SPI_State_e state;

	//1. Check whether SPI state is IDLE or not , if idle follow the below steps , else exist
	state = pSPIHandle->TxState;
	if(state != SPI_STATE_BUSY_IN_TX)
	{
		//2. Copy the Transmit buffer address and length byte information
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//3. Change the SPI state as busy in Tx
		pSPIHandle->TxState = SPI_STATE_BUSY_IN_TX;

		//4. Enable the SPI TXE interrupt bit to trigger the SPI_ISR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}


SPI_State_e SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len)
{
	SPI_State_e state;

	//1. Check whether SPI state is IDLE or not , if idle follow the below steps , else exist
	state = pSPIHandle->RxState;

	if(state != SPI_STATE_BUSY_IN_RX)
	{
		//2. Copy the Transmit buffer address and length byte information
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//3. Change the SPI state as busy in Tx
		pSPIHandle->RxState = SPI_STATE_BUSY_IN_RX;

		//4. Enable the SPI TXE interrupt bit to trigger the SPI_ISR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}


/*
 * SPI IRQ Configuration
 */
void SPI_IRQConfig(uint8_t IRQNumber,uint8_t EnorDi)
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


void SPI_IRQ_PriorityConfig(uint8_t IRQNumber,uint32_t IRQ_Priority)
{
	uint8_t index = 0;
	uint8_t Pos	= 0;
	uint8_t Bit_width = 8;

	index = IRQNumber / 4;
	Pos = IRQNumber % 4;

	*(NVIC_IPRN + index) = ((IRQ_Priority << 4) << (Bit_width * Pos));

}

/*
 * SPI Get Status API's
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t flagread)
{
	if(pSPIx->SR & flagread)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp = 0;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1 , temp2;

	//Check for TXE and TXEIE it is set
	temp1 = (pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE));
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE));

	if( temp1 && temp2 )
	{
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//Check for TXE and TXEIE it is set
	temp1 = (pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE));
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE));

	if( temp1 && temp2 )
	{
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//Check for TXE and TXEIE it is set
	temp1 = (pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR));
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE));

	if( temp1 && temp2 )
	{
		spi_ovr_interrupt_handle(pSPIHandle);
	}

}


void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	//Check DFF for 8bit or 16bit data format
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//write 16bit data into SPI Data register
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//write 8bit data into SPI Data register
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	//check for close the SPI transmission
	if( !pSPIHandle->TxLen )
	{
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_APP_EVT_TX_CMPLT);
	}
}


void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	//Check DFF for 8bit or 16bit data format
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//write 16bit data into SPI Data register
		*((uint16_t *)pSPIHandle->pRxBuffer) =  pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t *)pSPIHandle->pRxBuffer++;
	}
	else
	{
		//write 8bit data into SPI Data register
		*pSPIHandle->pRxBuffer =  pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	//check for close the SPI Recepition
	if( !pSPIHandle->RxLen )
	{
		SPI_CloseRecepition(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_APP_EVT_RX_CMPLT);
	}
}


void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp = 0;
	if(pSPIHandle->TxState != SPI_STATE_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
		(void)temp;
	}
	SPI_ApplicationEventCallback(pSPIHandle,SPI_APP_EVT_ERR_CMPLT);
}



__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,SPI_Event_e AppEV)
{

}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	//Disable the TXXIE bit
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_STATE_IDLE;
}

void SPI_CloseRecepition(SPI_Handle_t *pSPIHandle)
{
	//Disable the TXXIE bit
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_STATE_IDLE;
}


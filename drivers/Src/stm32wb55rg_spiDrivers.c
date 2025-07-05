/*
 * stm32wb55rg_spiDrivers.c
 *
 *  Created on: Jun 13, 2025
 *      Author: n1lby73
 */

#include "stm32wb55rg_spiDrivers.h"

/* Helper function */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName){

	if(pSPIx->SPIx_SR & flagName){

		return FlagSet;
	}

	return FlagReset;
}

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI){

	switch (ENorDI){

	case ENABLE:

		switch ((uint32_t)pSPIx){

			case (uint32_t)SPI1: SPI1_PCLK_EN(); break;

			case (uint32_t)SPI2: SPI2_PCLK_EN(); break;

		}

		break;

	default:

		switch ((uint32_t)pSPIx){

			case (uint32_t)SPI1: SPI1_PCLK_DI(); break;

			case (uint32_t)SPI2: SPI2_PCLK_DI(); break;

		}

		break;
	}

}

void SPI_Init(SPI_Handle_t *pSPIxHandle){

	// Configure the SPI_CR1 register

	uint32_t tempReg = 0;

	//1. Configure device mode

	tempReg |= pSPIxHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure Bus config

	switch (pSPIxHandle->SPIConfig.SPI_BusConfig){

		case SPI_BUS_CONFIG_FD:

//			bidi (bi-directional mode) should be cleared

			tempReg &= ~(1 << SPI_CR1_BIDIMODE);

			break;

		case SPI_BUS_CONFIG_HD:

//			bidi (bi-directional mode) should be set

			tempReg |= (1 << SPI_CR1_BIDIMODE);

			break;

		case SPI_BUS_CONFIG_SIMPLEX_RXONLY:

//			bidi (bi-directional mode) should be cleared
//			RXONLY bit should be set

			tempReg &= ~(1 << SPI_CR1_BIDIMODE);
			tempReg |= (1 << SPI_CR1_RXONLY);

			break;

	}

	//3. Configure Clock Speed

	tempReg |= pSPIxHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure DFF

	tempReg |= pSPIxHandle->SPIConfig.SPI_DFF << SPI_CR1_CRCL;

	//5. Configure CPOL

	tempReg |= pSPIxHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure CPHA

	tempReg |= pSPIxHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7. Configure SSM

	tempReg |= pSPIxHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIxHandle->pSPIx->SPIx_CR1 = tempReg;

}

void SPI_Deinit(SPI_Handle_t *pSPIx){

}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len){

	while (len>0){

		//1. wait until TXE is set

		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FlagReset);

		//2. check the DFF bit in CR1

		if (pSPIx->SPIx_CR1 & (1 << SPI_CR1_CRCL)){

			//16 bit DFF
			//1. Load the data into the DR

			pSPIx->SPIx_DR = *((uint16_t*)pTxBuffer);
			len = len - 2;
			(uint16_t*)pTxBuffer++;

		}else{

			//8 but DFF

			pSPIx->SPIx_DR = *pTxBuffer;
			len --;
			pTxBuffer++;

		}

	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len){

}


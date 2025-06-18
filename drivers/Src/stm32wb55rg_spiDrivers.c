/*
 * stm32wb55rg_spiDrivers.c
 *
 *  Created on: Jun 13, 2025
 *      Author: n1lby73
 */

#include "stm32wb55rg_spiDrivers.h"

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

	tempReg |= pSPIxHandle->SPIConfig.SPI_DeviceMode << 2;

	//2. Configure Bus config

	switch (pSPIxHandle->SPIConfig.SPI_BusConfig){

		case SPI_BUS_CONFIG_FD:

//			bidi (bi-directional mode) should be cleared

			tempReg &= ~(1 << 15);

			break;

		case SPI_BUS_CONFIG_HD:

//			bidi (bi-directional mode) should be set

			tempReg |= (1 << 15);

			break;

		case SPI_BUS_CONFIG_SIMPLEX_RXONLY:

//			bidi (bi-directional mode) should be cleared
//			RXONLY bit should be set

			tempReg &= ~(1 << 15);
			tempReg |= (1 << 10);

			break;

	}

	//3. Configure Clock Speed

	tempReg |= pSPIxHandle->SPIConfig.SPI_SclkSpeed << 3;

	//4. Configure DFF

	tempReg |= pSPIxHandle->SPIConfig.SPI_DFF << 11;

	//5. Configure CPOL

	tempReg |= pSPIxHandle->SPIConfig.SPI_CPOL << 1;

	//6. Configure CPHA

	tempReg |= pSPIxHandle->SPIConfig.SPI_CPHA << 0;

	//7. Configure SSM

	tempReg |= pSPIxHandle->SPIConfig.SPI_SSM << 9;

	pSPIxHandle->pSPIx->SPIx_CR1 = tempReg;

}

void SPI_Deinit(SPI_Handle_t *pSPIx){

}

void SPI_SendData(SPI_Handle_t *pSPIx, uint8_t *pTxBuffer, uint32_t len){

}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len){

}


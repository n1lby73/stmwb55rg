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

}

void SPI_Deinit(SPI_Handle_t *pSPIx){

}

void SPI_SendData(SPI_Handle_t *pSPIx, uint8_t *pTxBuffer, uint32_t len){

}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len){

}


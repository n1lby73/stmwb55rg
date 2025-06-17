/*
 * stm32wb55rg_spiDrivers.h
 *
 *  Created on: Jun 13, 2025
 *      Author: n1lby73
 */

#ifndef INC_STM32WB55RG_SPIDRIVERS_H_
#define INC_STM32WB55RG_SPIDRIVERS_H_

#include "stm32wb55rg.h"


/*
 * Configuration structure for SPIx peripheral
 */

typedef struct{

	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

} SPI_CONFIG_t;

/*
 * Handle structure for SPIx peripheral
 */

typedef struct{

	SPI_RegDef_t				*pSPIx;		/*!< This holds the base address of SPIx(x; 0,1,2) peripheral >*/
	SPI_CONFIG_t			SPIConfig;

} SPI_Handle_t;


/* Driver supported API's */

// Initialize and De-initialize

void SPI_Init(SPI_Handle_t *pGPIOHandle);
void SPI_DeInit(SPI_RegDef_t *pGPIOx);

// Peripheral Clock Setup

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);

/* Data send and Receive */

void SPI_SendData(SPI_Handle_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t len);

/* IRQ Configuration and IRQ Handling */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/* Other peripheral control */


#endif /* INC_STM32WB55RG_SPIDRIVERS_H_ */

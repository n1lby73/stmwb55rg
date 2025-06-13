/*
 * stm32wb55rg_spiDrivers.h
 *
 *  Created on: Jun 13, 2025
 *      Author: n1lby73
 */

#ifndef INC_STM32WB55RG_SPIDRIVERS_H_
#define INC_STM32WB55RG_SPIDRIVERS_H_

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
	SPI_Config_t				SPIConfig;

} SPI_Handle_t;

#endif /* INC_STM32WB55RG_SPIDRIVERS_H_ */

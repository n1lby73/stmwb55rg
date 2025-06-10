 /*
 * stm32wb55rg_gpioDrivers.c
 *
 *  Created on: May 23, 2025
 *      Author: n1lby73
 */

#include "stm32wb55rg_gpioDrivers.h"

/* Function Declaration */

// Initialize and De-initialize a pin

/**********FUNCTION DOCUMENTATION*****************
 *
 * @fn			-	GPIO_Init
 *
 * @brief		-	This function initializes the given GPIO pin
 *
 * @param[in]	-	An array of GPIO port base address and pin configuration setting
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- 	none
 *k
 * @Note		-	none
*/

void GPIO_Init(GPIO_HANDLE_t *pGPIOHandle){

	uint32_t temporaryVar = 0;

	//1. Configure the mode of GPIO pin

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){

		temporaryVar = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));

		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temporaryVar;

	}else{

		// Step 1: Determine the preferred edge detection and configure it

		switch ((uint32_t)pGPIOHandle->GPIO_PinConfig.GPIO_PinMode){

			case ((uint32_t)GPIO_MODE_IT_FT):

				EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			break;

			case ((uint32_t)GPIO_MODE_IT_RT):

				EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			break;

			case ((uint32_t)GPIO_MODE_IT_RFT):

				EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			break;
		}

		// Step 2: Configure the GPIO port selection in SYSCFG_EXTICR

		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		uint8_t tempVar1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t tempVar2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		SYSCFG_PCLK_EN();

		SYSCFG->EXTICR[tempVar1] = portCode << (tempVar2 * 4);

		// Step 3: Enable the EXTI interrupt delivery using IMR

		EXTI->IMR1 |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	//2. Configure the speed

	temporaryVar = 0;

	temporaryVar = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temporaryVar;

	//3. Configure the PuPd settings

	temporaryVar = 0;

	temporaryVar = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temporaryVar;
}
/*
 *
 * @Note		-	none
*/

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	switch ((uint32_t)pGPIOx){

		case (uint32_t)GPIOA: GPIOA_REG_RESET(); break;

		case (uint32_t)GPIOB: GPIOB_REG_RESET(); break;

		case (uint32_t)GPIOC: GPIOC_REG_RESET(); break;

		case (uint32_t)GPIOD: GPIOD_REG_RESET(); break;

		case (uint32_t)GPIOE: GPIOE_REG_RESET(); break;

	}
}

// Peripheral Clock Setup

/**********FUNCTION DOCUMENTATION*****************
 *
 * @fn			-	GPIO_PeriClockControl
 *
 * @brief		-	This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]	-	Base address of the GPIO peripheral
 * @param[in]	-	ENABLE or DISABLE macros
 * @param[in]	-
 *
 * @return		- 	none
 *
 * @Note		-	none
*/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI){

	switch (ENorDI){

	case ENABLE:

		switch ((uint32_t)pGPIOx){

			case (uint32_t)GPIOA: GPIOA_PCLK_EN(); break;

			case (uint32_t)GPIOB: GPIOB_PCLK_EN(); break;

			case (uint32_t)GPIOC: GPIOC_PCLK_EN(); break;

			case (uint32_t)GPIOD: GPIOD_PCLK_EN(); break;

			case (uint32_t)GPIOE: GPIOE_PCLK_EN(); break;

		}

		break;

	default:

		switch ((uint32_t)pGPIOx){

			case (uint32_t)GPIOA: GPIOA_PCLK_DI(); break;

			case (uint32_t)GPIOB: GPIOB_PCLK_DI(); break;

			case (uint32_t)GPIOC: GPIOC_PCLK_DI(); break;

			case (uint32_t)GPIOD: GPIOD_PCLK_DI(); break;

			case (uint32_t)GPIOE: GPIOE_PCLK_DI(); break;

			case (uint32_t)GPIOH: GPIOH_PCLK_DI(); break;

		}

		break;
	}

}

// Data Read and Write

/**********FUNCTION DOCUMENTATION*****************
 *
 * @fn			-	GPIO_ToggleOutputPin
 *
 * @brief		-	This function toggles the state for the given GPIO pin
 *
 * @param[in]	-	Base address of the GPIO peripheral
 * @param[in]	-	GPIO Pin Number
 * @param[in]	-
 *
 * @return		- 	none
 *
 * @Note		-	none
*/

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1 << PinNumber);

}

/**********FUNCTION DOCUMENTATION*****************
 *
 * @fn			-	GPIO_ReadFromInputPin
 *
 * @brief		-	This function retrieves the state for the given GPIO pin
 *
 * @param[in]	-	Base address of the GPIO peripheral
 * @param[in]	-	GPIO Pin Number
 * @param[in]	-
 *
 * @return		- 	uint8_t of value 0 or 1
 *
 * @Note		-	none
*/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t temp = (uint8_t)( (pGPIOx->IDR >> PinNumber) & 0x1 );
	return temp;

}

/**********FUNCTION DOCUMENTATION*****************
 *
 * @fn			-	GPIO_ReadFromInputPort
 *
 * @brief		-	This function retrieves the state for the given GPIO port
 *
 * @param[in]	-	Base address of the GPIO peripheral
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- 	uint16_t
 *
 * @Note		-	none
*/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t temp = (uint16_t)pGPIOx->IDR;
	return temp;

}

/**********FUNCTION DOCUMENTATION*****************
 *
 * @fn			-	GPIO_WriteToOutputPin
 *
 * @brief		-	This function writes a state for the given GPIO pin
 *
 * @param[in]	-	Base address of the GPIO peripheral
 * @param[in]	-	GPIO Pin Number
 * @param[in]	-	value to write
 *
 * @return		- 	none
 *
 * @Note		-	none
*/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if (Value == GPIO_PinSet){

		pGPIOx->ODR |= (1 << PinNumber);

	}else{

		pGPIOx->ODR &= ~(1 << PinNumber);

	}
}

/**********FUNCTION DOCUMENTATION*****************
 *
 * @fn			-	GPIO_WriteToOutputPort
 *
 * @brief		-	This function writes a state for the given GPIO port
 *
 * @param[in]	-	Base address of the GPIO peripheral
 * @param[in]	-	value to write
 * @param[in]	-
 *  *
 * @return		- 	none
 *
 * @Note		-	none
*/

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value){

	pGPIOx->ODR = Value;

}

// IRQ Configuration and IRQ Handling

/**********FUNCTION DOCUMENTATION*****************
 *
 * @fn			-	GPIO_IRQConfig
 *
 * @brief		-	This function configures Interrupt request
 *
 * @param[in]	-	Interrupt Request Number
 * @param[in]	-	Interrupt Priority
 * @param[in]	-	Enable or Disable Macro
 *
 * @return		- 	none
 *
 * @Note		-	none
*/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI){

	switch (ENorDI) {
	    case ENABLE:
	        switch (IRQNumber) {
	            case 0 ... 31:
	                *NVIC_ISER0 |= (1 << IRQNumber);
	                break;

	            case 32 ... 63:
	                *NVIC_ISER1 |= (1 << IRQNumber % 32);
	                break;

	            case 64 ... 95:
	                *NVIC_ISER2 |= (1 << IRQNumber % 64);
	                break;
	        }
	        break;

	    case DISABLE:
	        switch (IRQNumber) {
	            case 0 ... 31:
	                *NVIC_ICER0 |= (1 << IRQNumber);
	                break;

	            case 32 ... 63:
	                *NVIC_ICER1 |= (1 << IRQNumber % 32);
	                break;

	            case 64 ... 95:
	                *NVIC_ICER2 |= (1 << IRQNumber % 64);
	                break;
	        }
	        break;
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << ((8 * iprx_section)+(8-NO_PR_BITS_IMPLEMENTED)));
}

/**********FUNCTION DOCUMENTATION*****************
 *
 * @fn			-	GPIO_IRQHandle
 *
 * @brief		-	This function handles the interrupt request for a specified GPIO pin
 *
 * @param[in]	-	GPIO Pin Number
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- 	none
 *
 * @Note		-	none
*/

void GPIO_IRQHandling(uint8_t PinNumber){

	if (EXTI->PR1 & (1 << PinNumber)){

		EXTI->PR1 |= (1 << PinNumber);
	}
}

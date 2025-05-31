/*
 * stm32wb55rg_gpioDrivers.h
 *
 *  Created on: May 23, 2025
 *      Author: n1lby73
 */

#ifndef INC_STM32WB55RG_GPIODRIVERS_H_
#define INC_STM32WB55RG_GPIODRIVERS_H_

#include "stm32wb55rg.h"

/*
 * @GPIO_PinNumber
 * GPIO pins number
 *
 * @note: GPIO port E has 0-4 while port H has 0-1,3
 *
*/

#define GPIO_PIN_NO_0							0
#define GPIO_PIN_NO_1							1
#define GPIO_PIN_NO_2							2
#define GPIO_PIN_NO_3							3
#define GPIO_PIN_NO_4							4
#define GPIO_PIN_NO_5							5
#define GPIO_PIN_NO_6							6
#define GPIO_PIN_NO_7							7
#define GPIO_PIN_NO_8							8
#define GPIO_PIN_NO_9							9
#define GPIO_PIN_NO_10							10
#define GPIO_PIN_NO_11							11
#define GPIO_PIN_NO_12							12
#define GPIO_PIN_NO_13							13
#define GPIO_PIN_NO_14							14
#define GPIO_PIN_NO_15							15


/*
 * @GPIO_PinMode
 * GPIO pins possible mode
 *
*/

#define GPIO_MODE_IN							0
#define GPIO_MODE_OUT							1
#define GPIO_MODE_ALTFN							2
#define GPIO_MODE_ANALOG						3
#define GPIO_MODE_IT_FT							4
#define GPIO_MODE_IT_RT							5
#define GPIO_MODE_IT_RFT						6

/*
 * @GPIO_PinSpeed
 * GPIO pins possible speed
 *
 */

#define GPIO_LOW_SPEED							0
#define GPIO_MEDIUM_SPEED						1
#define GPIO_FAST_SPEED							2
#define GPIO_HIGH_SPEED							3

/*
 * @GPIO_PinPuPd_Control
 * GPIO pins possible PuPd setting
 *
 */

#define GPIO_NO_PUPD							0
#define GPIO_PIN_PU								1
#define GPIO_PIN_PD								2


/*
 * @GPIO_PinOPType
 * GPIO pins possible output type
 *
 */

#define GPIO_OP_TYPE_PP							0
#define GPIO_OP_TYPE_OD							1


/* Configuration option of a GPIO pin */

typedef struct {

	uint8_t GPIO_PinNumber;						/*!< possible values from @GPIO_PinNumber >*/
	uint8_t GPIO_PinMode;						/*!< possible values from @GPIO_PinMode >*/
	uint8_t GPIO_PinSpeed;						/*!< possible values from @GPIO_PinSpeed >*/
	uint8_t GPIO_PinPuPdControl;				/*!< possible values from @GPIO_PinPuPd_Control >*/
	uint8_t GPIO_PinOPType;						/*!< possible values from @GPIO_PinOPType >*/
	uint8_t GPIO_PinAltFunMode;					/*!< possible values from @GPIO_PinAltFunMode >*/

} GPIO_PIN_CONFIG_t;

/* Handle struct for GPIO pin*/

typedef struct{

	GPIO_RegDef_t *pGPIOx;						/* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PIN_CONFIG_t GPIO_PinConfig;			/* This holds the GPIO pin configuration setting */

}GPIO_HANDLE_t;

/* Driver supported API's */

// Initialize and De-initialize a pin

void GPIO_Init(GPIO_HANDLE_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// Peripheral Clock Setup

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);

// Data Read and Write

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);

// IRQ Configuration and IRQ Handling

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32WB55RG_GPIODRIVERS_H_ */

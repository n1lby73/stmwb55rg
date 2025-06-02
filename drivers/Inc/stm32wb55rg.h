 /*
 * stm32wb55rg.h
 *
 *  Created on: May 21, 2025
 *      Author: n1lby73
 */

#ifndef INC_STM32WB55RG_H_
#define INC_STM32WB55RG_H_

#include <stdint.h>
/* Custom Macros */

#define _vo 						volatile
#define ENABLE						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PinSet					ENABLE
#define GPIO_PinReset				DISABLE

/* Memory Address Definition */

#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U
#define SRAM2A_BASEADDR				0x20030000U
#define SRAM2B_BASEADDR				0x20038000U
#define ROM_BASEADDR				0x1FFF0000U

/* Bus Address Definition */

#define APB1PERIPH_BASE				0x40000000U
#define APB2PERIPH_BASE				0x40010000U
#define AHB1PERIPH_BASE				0x40020000U
#define AHB2PERIPH_BASE				0x48000000U
#define AHB4PERIPH_BASE				0x58000000U

/* Base Address of APB1 Peripherals */

#define SPI2_BASEADDR 				(APB1PERIPH_BASE + 0x3800)
#define I2C1_BASEADDR 				(APB1PERIPH_BASE + 0x5400)
#define I2C3_BASEADDR 				(APB1PERIPH_BASE + 0x5C00)

/* Base Address of APB2 Peripherals */

#define SYSCFG_BASEADDR				(APB2PERIPH_BASE + 0x0000)
#define SPI1_BASEADDR 				(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR 			(APB2PERIPH_BASE + 0x3800)

/* Base Address of AHB1 Peripherals */

// System peripherals only - RCC CRC....

/* Base Address of AHB2 Peripherals */

// GPIO's

#define GPIOA_BASEADDR				(AHB2PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR				(AHB2PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR				(AHB2PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR				(AHB2PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR				(AHB2PERIPH_BASE + 0x1000)
#define GPIOH_BASEADDR				(AHB2PERIPH_BASE + 0x1C00)

/* Base Address of AHB4 Peripherals */

#define RCC_BASEADDR				(AHB4PERIPH_BASE + 0X0000)
#define EXTI_BASEADDR				(AHB4PERIPH_BASE + 0x0800)

/* Peripheral register definition structures */

typedef struct{

	_vo uint32_t MODER;				// GPIO port mode register
	_vo uint32_t OTYPER;			// GPIO port output type register
	_vo uint32_t OSPEEDR;			// GPIO port output speed register
	_vo uint32_t PUPDR;				// GPIO port pull-up/pull-down register
	_vo uint32_t IDR;				// GPIO port input data register
	_vo uint32_t ODR;				// GPIO port output data register
	_vo uint32_t BSRR;				// GPIO port bit set/reset register
	_vo uint32_t LCKR;				// GPIO port configuration lock register
	_vo uint32_t AFR[2];			// GPIO alternate function low register for AFR[0]; GPIO alternate function high register for AFR[1]
	_vo uint32_t BRR;				// GPIO port bit reset register

} GPIO_RegDef_t;

typedef struct {

	_vo uint32_t CR;				// RCC clock control register
	_vo uint32_t ICSCR;				// RCC internal clock sources calibration register
	_vo uint32_t CFGR;				// RCC clock configuration register
	_vo uint32_t PLLCFGR;			// RCC PLL configuration register
	_vo uint32_t PLLSAI1CFGR;		// RCC PLLSAI1 configuration register
	_vo uint32_t RESERVED;			// First RESERVED NIBBLE
	_vo uint32_t CIER;				// RCC clock interrupt enable register
	_vo uint32_t CIFR;				// RCC clock interrupt flag register
	_vo uint32_t CICR;				// RCC clock interrupt clear register
	_vo uint32_t SMPSCR;			// RCC SMPS step-down converter control register
	_vo uint32_t AHB1RSTR;			// RCC AHB1 peripheral reset register
	_vo uint32_t AHB2RSTR;			// RCC AHB2 peripheral reset register
	_vo uint32_t AHB3RSTR;			// RCC AHB3 and AHB4 peripheral reset register
	_vo uint32_t RESERVED2;			// Second RESERVED NIBBLE
	_vo uint32_t APB1RSTR1;			// RCC APB1 peripheral reset register 1
	_vo uint32_t APB1RSTR2;			// RCC APB1 peripheral reset register 2
	_vo uint32_t APB2RSTR;			// RCC APB2 peripheral reset register
	_vo uint32_t APB3RSTR;			// RCC APB3 peripheral reset register
	_vo uint32_t AHB1ENR;			// RCC AHB1 peripheral clock enable register
	_vo uint32_t AHB2ENR;			// RCC AHB2 peripheral clock enable register
	_vo uint32_t AHB3ENR;			// RCC AHB3 and AHB4 peripheral clock enable register
	_vo uint32_t APB1ENR1;		// RCC APB1 peripheral clock enable register 1
	_vo uint32_t APB1ENR2;		// RCC APB1 peripheral clock enable register 2
	_vo uint32_t APB2ENR;		// RCC APB2 peripheral clock enable register
	_vo uint32_t RESERVED3;			// THIRD RESERVED NIBBLE
	_vo uint32_t AHB1SMENR;			// RCC AHB1 peripheral clocks enable in Sleep modes register
	_vo uint32_t AHB2SMENR;			// RCC AHB2 peripheral clocks enable in Sleep modes register
	_vo uint32_t AHB3SMENR;			// RCC AHB3 and AHB4 peripheral clocks enable in Sleep and Stop modes register
	_vo uint32_t RESERVED4;			// FOURTH RESERVED NIBBLE
	_vo uint32_t APB1SMENR1;		// RCC APB1 peripheral clocks enable in Sleep mode register 1
	_vo uint32_t APB1SMENR2;		// RCC APB1 peripheral clocks enable in Sleep mode register 2
	_vo uint32_t APB2SMENR;			// RCC APB2 peripheral clocks enable in Sleep mode register
	_vo uint32_t RESERVED5;			// FIFTH RESERVED NIBBLE
	_vo uint32_t CCIPR;				// RCC peripherals independent clock configuration register
	_vo uint32_t RESERVED6;			// SIXITH RESERVED NIBBLE
	_vo uint32_t BDCR;				// RCC backup domain control register
	_vo uint32_t CSR;				// RCC control/status register
	_vo uint32_t CRRCR;				// RCC clock recovery RC register
	_vo uint32_t HSECR;				// RCC clock HSE register
	_vo uint32_t RESERVED7;			// SEVENTH RESERVED NIBBLE
	_vo uint32_t EXTCFGR;			// RCC extended clock recovery register
	_vo uint32_t RESERVED8;			// EIGHT RESRVED NIBBLE
	_vo uint32_t C2AHB1ENR;			// RCC CPU2 AHB1 peripheral clock enable register
	_vo uint32_t C2AHB2ENR;			// RCC CPU2 AHB2 peripheral clock enable register
	_vo uint32_t C2AHB3ENR;			// RCC CPU2 AHB3 and AHB4 peripheral clock enable register
	_vo uint32_t C2APB1ENR1;		// RCC CPU2 APB1 peripheral clock enable register 1
	_vo uint32_t C2APB1ENR2;		// RCC CPU2 APB1 peripheral clock enable register 2
	_vo uint32_t C2APB2ENR;			// RCC CPU2 APB2 peripheral clock enable register
	_vo uint32_t C2APB3ENR;			// RCC CPU2 APB3 peripheral clock enable register
	_vo uint32_t C2AHB1SMENR;		// RCC CPU2 AHB1 peripheral clocks enable in Sleep modes register
	_vo uint32_t C2AHB2SMENR;		// RCC CPU2 AHB2 peripheral clocks enable in Sleep mode
	_vo uint32_t C2AHB3SMENR;		// RCC CPU2 AHB3 and AHB4 peripheral clocks enable in Sleep mode register
	_vo uint32_t RESERVED9;			// NINETH RESERVED NIBBLE
	_vo uint32_t C2APB1SMENR1;		// RCC CPU2 APB1 peripheral clocks enable in Sleep mode register 1
	_vo uint32_t C2APB1SMENR2;		// RCC CPU2 APB1 peripheral clocks enable in Sleep mode register 2
	_vo uint32_t C2APB2SMENR;		// RCC CPU2 APB2 peripheral clocks enable in Sleep mode register
	_vo uint32_t C2APB3SMENR;		// RCC CPU2 APB3 peripheral clock enable in Sleep mode register
	_vo uint32_t RESERVED10;		// TENTH RESERVED NIBBLE

} RCC_RegDef_t;

typedef struct {

    _vo uint32_t RTSR1;        		// 0x00: Rising trigger selection register 1
    _vo uint32_t FTSR1;        		// 0x04: Falling trigger selection register 1
    _vo uint32_t SWIER1;       		// 0x08: Software interrupt event register 1
    _vo uint32_t PR1;          		// 0x0C: Pending register 1

    _vo uint32_t RESERVED0[4];     	// 0x10–0x1C: Reserved gap

    _vo uint32_t RTSR2;        		// 0x20: Rising trigger selection register 2
    _vo uint32_t FTSR2;        		// 0x24: Falling trigger selection register 2
    _vo uint32_t SWIER2;       		// 0x28: Software interrupt event register 2
    _vo uint32_t PR2;          		// 0x2C: Pending register 2

    _vo uint32_t RESERVED[20];		// 0x30–0x7C: Reserved gap

    _vo uint32_t IMR1;         		// 0x30: Interrupt mask register 1
    _vo uint32_t EMR1;         		// 0x34: Event mask register 1

    _vo uint32_t RESERVED1[2];     	// 0x88–0x8C: Reserved gap

    _vo uint32_t IMR2;         		// 0x40: Interrupt mask register 2
    _vo uint32_t EMR2;         		// 0x44: Event mask register 2

    _vo uint32_t RESERVED2[10];    	// 0x98–0xBC: Reserved gap (0x48 is next usable, C1IMR1 starts at 0x80)

    _vo uint32_t C2IMR1;       		// 0x80: CPU1 interrupt mask register 1
    _vo uint32_t C2EMR1;       		// 0x84: CPU1 event mask register 1

    _vo uint32_t RESERVED3[2];		// 0xC8–0xCC: Reserved gap

    _vo uint32_t C2IMR2;       		// 0x98: CPU2 interrupt mask register 2
    _vo uint32_t C2EMR2;       		// 0x9C: CPU2 event mask register 2

} EXTI_RegDef_t;

typedef struct{

	_vo uint32_t MEMRMP;
	_vo uint32_t CFGR1;
	_vo uint32_t EXTICR[4];
	_vo uint32_t SCSR;
	_vo uint32_t CFGR2;
	_vo uint32_t SWPR;
	_vo uint32_t SKR;
	_vo uint32_t SWPR2;
	_vo uint32_t IMR[2];
	_vo uint32_t C2IMR[2];
	_vo uint32_t SIPCR;

} SYSCFG_RegDef_t;

/* Peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t */

#define GPIOA						((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOH						((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC							((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI						((EXTI_RegDef_t*) EXTI_BASEADDR)

/* Clock Enable Macros for GPIOx Peripheral */

#define GPIOA_PCLK_EN()				(RCC -> AHB2ENR |= (1 << 0))
#define GPIOB_PCLK_EN()				(RCC -> AHB2ENR |= (1 << 1))
#define GPIOC_PCLK_EN()				(RCC -> AHB2ENR |= (1 << 2))
#define GPIOD_PCLK_EN()				(RCC -> AHB2ENR |= (1 << 3))
#define GPIOE_PCLK_EN()				(RCC -> AHB2ENR |= (1 << 4))
#define GPIOH_PCLK_EN()				(RCC -> AHB2ENR |= (1 << 7))

/* Clock Enable Macros for I2Cx Peripheral */

#define I2C1_PCLK_EN()				(RCC -> APB1ENR1 |= (1 << 21))
#define I2C3_PCLK_EN()				(RCC -> APB1ENR1 |= (1 << 23))

/* Clock Enable Macros for SPIx Peripheral */

#define SPI1_PCLK_EN()				(RCC -> APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()				(RCC -> APB1ENR1 |= (1 << 14))

/* Clock Enable Macros for USARTx Peripheral */

#define USART1_PCLK_EN()			(RCC -> APB2ENR |= (1 << 14))

/* Clock Enable Macros for SYSCFG Peripheral */

#define SYSCFG_PCLK_EN()			(RCC -> APB2ENR |= (1 << 14))

/* Clock Disable Macros for GPIOx Peripheral */

#define GPIOA_PCLK_DI()				(RCC -> AHB2ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()				(RCC -> AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()				(RCC -> AHB2ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()				(RCC -> AHB2ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()				(RCC -> AHB2ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()				(RCC -> AHB2ENR &= ~(1 << 7))

/* Clock Disable Macros for I2Cx Peripheral */

#define I2C1_PCLK_DI()				(RCC -> APB1ENR1 &= ~(1 << 21))
#define I2C3_PCLK_DI()				(RCC -> APB1ENR1 &= ~(1 << 23))

/* Clock Disable Macros for SPIx Peripheral */

#define SPI1_PCLK_DI()				(RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()				(RCC -> APB1ENR1 &= ~(1 << 14))

/* Clock Disable Macros for USARTx Peripheral */

#define USART1_PCLK_DI()			(RCC -> APB2ENR &= ~(1 << 14))

/* Clock Disable Macros for SYSCFG Peripheral */


/* Clock Disable Macros for SYSCFG Peripheral */

#define GPIOA_REG_RESET()			do{ (RCC -> AHB2RSTR |= (1 << 0)); (RCC -> AHB2RSTR &= ~(1 << 0)); } while (0)
#define GPIOB_REG_RESET()			do{ (RCC -> AHB2RSTR |= (1 << 1)); (RCC -> AHB2RSTR &= ~(1 << 1)); } while (0)
#define GPIOC_REG_RESET()			do{ (RCC -> AHB2RSTR |= (1 << 2)); (RCC -> AHB2RSTR &= ~(1 << 2)); } while (0)
#define GPIOD_REG_RESET()			do{ (RCC -> AHB2RSTR |= (1 << 3)); (RCC -> AHB2RSTR &= ~(1 << 3)); } while (0)
#define GPIOE_REG_RESET()			do{ (RCC -> AHB2RSTR |= (1 << 4)); (RCC -> AHB2RSTR &= ~(1 << 4)); } while (0)
#define GPIOH_REG_RESET()			do{ (RCC -> AHB2RSTR |= (1 << 7)); (RCC -> AHB2RSTR &= ~(1 << 7)); } while (0)

#endif /* INC_STM32WB55RG_H_ */

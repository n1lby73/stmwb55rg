/*
 * toggleLed.c
 *
 *  Created on: May 27, 2025
 *      Author: n1lby73
 */


#include "stm32wb55rg.h"
#include "stm32wb55rg_gpioDrivers.h"

void delay(void){

	for (uint32_t i = 0; i<100000; i++){

		//do nothing;
	}
}

int main(void){

	GPIO_HANDLE_t externalPb;

	externalPb.pGPIOx = GPIOC;
	externalPb.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	externalPb.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
//	configGpioD.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	externalPb.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;
	externalPb.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_HANDLE_t greenLed;

	greenLed.pGPIOx = GPIOB;
	greenLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	greenLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	greenLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	greenLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;
//	greenLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_HANDLE_t redLed;

	redLed.pGPIOx = GPIOB;
	redLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	redLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	redLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	redLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;
	redLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_HANDLE_t blueLed;

	blueLed.pGPIOx = GPIOB;
	blueLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	blueLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	blueLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	blueLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;
	blueLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_HANDLE_t externalBlueLed;

	externalBlueLed.pGPIOx = GPIOB;
	externalBlueLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	externalBlueLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	externalBlueLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	externalBlueLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;
	externalBlueLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_HANDLE_t pb2;

	pb2.pGPIOx = GPIOD;
	pb2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	pb2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	pb2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pb2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;
	pb2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_HANDLE_t pb3;

	pb3.pGPIOx = GPIOD;
	pb3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	pb3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	pb3.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	pb3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;
	pb3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

//	configGpioB.GPIO_PinConfig.n
//	GpioLed.pGPIOx = GPIOB;
//
//	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
//	GpioLed.GPIO_PinConfig.GPIO_PinNumber
//	uint8_t pinNumber = 1;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&externalPb);
	GPIO_Init(&greenLed);
	GPIO_Init(&redLed);
	GPIO_Init(&blueLed);
	GPIO_Init(&externalBlueLed);
	GPIO_Init(&pb2);
	GPIO_Init(&pb3);

	uint8_t prevState = 0;
	uint8_t prevBtnState = 0;

	while(1){

//		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_5);

//		uint8_t status = GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_0);
		uint8_t externalStatus = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_4);
		uint8_t pb2Status = GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_0);
		uint8_t pb3Status = GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_1);

//		Toggle blue led

		if (externalStatus == 0 && prevBtnState == 1){ // Pin D10

//			newState = 1;
			prevState = !prevState;
//			prevBtnState = externalStatus;
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_5, prevState); // Pin 26 - BLUE LED
//			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_15, 1);
		}
		prevBtnState = externalStatus;
//
//		}else{
//
//			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_5, 1); // Pin 26
////			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_15, 0);
//
//		}

//		Toggle Green led

		if (pb2Status == 1){ // Pin 36

			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, 0); // Pin 22 - GREEN LED

		}else{

			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, 1); // Pin 22

		}

//		Toggle Red led

		if (pb3Status == 1){ // Pin 38

			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_1, 0); // Pin 24 - RED LED

		}else{

			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_1, 1); // Pin 24

		}

//		delay();
//		uint8_t status2 = GPIO_ReadFromi
//		if (status == 1)
//
// 		    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, 1);
//
//		else{
//
//		    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, 0);
//		}

//		if (status2 == 1){
//
//			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, 1);
//
//		}else{
//
//			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, 0);
//		}
////		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, status);

		delay();

	}
}

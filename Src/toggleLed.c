/*
 * toggleLed.c
 *
 *  Created on: May 27, 2025
 *      Author: n1lby73
 */

#include "stm32wb55rg.h"
#include "stm32wb55rg_gpioDrivers.h"

uint8_t prevState = 0;
uint8_t prevBtnState = 0;

void delay(void){

	for (uint32_t i = 0; i<100000; i++){

		//do nothing;
	}
}

void detentSwitch(){

	uint8_t externalStatus = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_4);

	if (externalStatus == 0 && prevBtnState == 1){ // Pin D10

		prevState = !prevState;
		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_5, prevState); // Pin 26 - BLUE LED
	}

	prevBtnState = externalStatus;

}

void toggleGreenLed(){

	uint8_t pb2Status = GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_0);

	if (pb2Status == 1){ // Pin 36

		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, 0); // Pin 22 - GREEN LED

	}else{

		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, 1); // Pin 22

	}
}

void toggleRedLed(){

	uint8_t pb3Status = GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_1);

	if (pb3Status == 1){ // Pin 38

		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_1, 0); // Pin 24 - RED LED

	}else{

		GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_1, 1); // Pin 24
	}
}

void ledChaser(){

	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_1, 1);
	delay();
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_1, 0);
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, 1);
	delay();
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, 0);
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_5, 1);
	delay();
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_5, 0);



}

int main(void){

	GPIO_HANDLE_t externalPb;

	externalPb.pGPIOx = GPIOA;
	externalPb.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	externalPb.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	externalPb.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;
	externalPb.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

	GPIO_HANDLE_t greenLed;

	greenLed.pGPIOx = GPIOB;
	greenLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	greenLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	greenLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	greenLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;

	GPIO_HANDLE_t redLed;

	redLed.pGPIOx = GPIOB;
	redLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	redLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	redLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	redLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;

	GPIO_HANDLE_t blueLed;

	blueLed.pGPIOx = GPIOB;
	blueLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	blueLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	blueLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	blueLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;

	GPIO_HANDLE_t externalBlueLed;

	externalBlueLed.pGPIOx = GPIOB;
	externalBlueLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	externalBlueLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	externalBlueLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	externalBlueLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;

	GPIO_HANDLE_t pb1;

	pb1.pGPIOx = GPIOC;
	pb1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	pb1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	pb1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;
	pb1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_HANDLE_t pb2;

	pb2.pGPIOx = GPIOD;
	pb2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	pb2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	pb2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;
	pb2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_HANDLE_t pb3;

	pb3.pGPIOx = GPIOD;
	pb3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	pb3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	pb3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_FAST_SPEED;
	pb3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&externalPb);
	GPIO_Init(&greenLed);
	GPIO_Init(&redLed);
	GPIO_Init(&blueLed);
	GPIO_Init(&externalBlueLed);
	GPIO_Init(&pb1);
	GPIO_Init(&pb2);
	GPIO_Init(&pb3);


	while(1){

		// Turn on blue led
		detentSwitch();

		// Toggle green led
		toggleGreenLed();

		// Toggle red led
		toggleRedLed();

		while(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_4) == 1){

			ledChaser();
		}
//		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_5);

		delay();

	}
}

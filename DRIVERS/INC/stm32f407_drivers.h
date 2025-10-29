#ifndef STM32F407_DRIVER_H
#define STM32F407_DRIVER_H


#include<stdint.h>
#include "stm32f407_xx.h"


// This is a configuration structure for a GPIO pin


typedef struct{

	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode; 			/* @GPIO_PIN_MODE */
	uint8_t GPIO_PinSpeed; 			/*@GPIO_PIN_SPEED_MODES*/
	uint8_t GPIO_PinPuPdControl;	/* @GPIO_PULL_UP_AND_PULL_DOWN_RESISTOR*/
	uint8_t GPIO_PinOPType;			/*@GPIO_OUTPUT_TYPES*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


// This is a Handle structure for a GPIO pin

typedef struct{

	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;



/*
 * @GPIO_PIN_NUMBERS
*/

#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15


/*
 * @GPIO_PIN_MODE
*/

#define GPIO_MODE_IN    		0
#define GPIO_MODE_OUT    		1
#define GPIO_MODE_ALTFN    		2
#define GPIO_MODE_ANALOG    	3
#define INTR_F_EDGE   	    	4
#define INTR_R_EDGE   	    	5
#define INTR_BOTH_EDGE   		6


/*
 * @GPIO_OUTPUT_TYPES
*/

#define GPIO_OP_TYPE_PP         0
#define GPIO_OP_TYPE_OD         1


/*
 * @GPIO_PIN_SPEED_MODES
*/

#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM	    1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3



/*
 * @GPIO_PULL_UP_AND_PULL_DOWN_RESISTOR
*/

#define GPIO_NO_PUPD  			0
#define GPIO_PIN_PU  			1
#define GPIO_PIN_PD  			2


// APIs supported by this driver

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */

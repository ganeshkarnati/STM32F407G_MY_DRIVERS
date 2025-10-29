/*
 * i2c_send_data.c
 *
 *  Created on: Oct 28, 2025
 *      Author: karnati.ganesh
 */

#include <string.h>
#include "stm32f407_xx.h"
#include "stm32f407_i2c.h"
#include<stdio.h>

I2C_Handle_t I2C_handle;

void I2C_GPIO_Init()
{
    GPIO_Handle_t I2C_PINS;
    // SPI2 uses GPIOB pins
       I2C_PINS.pGPIOx = GPIOB;
       I2C_PINS.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;      // Alternate function mode
       I2C_PINS.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;      // Fast enough for SPI
       I2C_PINS.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU ;   // No pull-up/pull-down
       I2C_PINS.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;     // Push-pull output
       I2C_PINS.GPIO_PinConfig.GPIO_PinAltFunMode = 4;               // AF5 → SPI2

       // 1. SDA
       		I2C_PINS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
       		GPIO_Init(&I2C_PINS);

       		// 2. SCL
       		I2C_PINS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
       		GPIO_Init(&I2C_PINS);

}

//-------------------------------------------------------------//
// Function: SPIx_INIT
// Purpose : Configure and enable SPI2 peripheral in Master mode
//-------------------------------------------------------------//
void I2Cx_INIT()
{

	I2C_handle.pI2Cx=I2C2;
	I2C_handle.I2C_Config.I2C_ACKControl=I2C_ACK_ENABLE ;
	I2C_handle.I2C_Config.I2C_SCLSpeed=I2C_SCL_SPEED_SM;
	I2C_handle.I2C_Config.I2C_FMDutyCycle=I2C_FM_DUTY_2;
	I2C_handle.I2C_Config.I2C_DeviceAddress=0X61;
	I2C_Init(&I2C_handle);


                                           // Apply settings
}

void button_config()
{
	GPIO_Handle_t button;
	button.pGPIOx=GPIOE;
	      button.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	      button.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	      button.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_MEDIUM;
	      button.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;
	      GPIO_Init(&button);
}

int main()
{
	char data[]="ganesh&53462";
	I2C_GPIO_Init();
	I2Cx_INIT();
	I2C_PeripheralControl(I2C2,ENABLE);
	button_config();
	while(1)
	{
		uint8_t flag;
		if(!(flag=GPIO_ReadFromInputPin(GPIOE,5)))
	    {
		   for(volatile int i=0;i<100000;i++);
		   I2C_MasterSendData(&I2C_handle,data,strlen(data),0x68);
		   printf("%d\n",flag);
	    }

	}

}

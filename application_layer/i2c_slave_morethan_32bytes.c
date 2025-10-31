/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Feb 24, 2019
 *      Author: admin
 */



//extern void initialise_monitor_handles();


#include <string.h>
#include "stm32f407_xx.h"
#include "stm32f407_i2c.h"
#include<stdio.h>

I2C_Handle_t I2C_handle;
uint8_t CommandCode;
 uint32_t data_len=0;
 //very large message
 char Tx_buf[] = "HiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHiHi...123";

void I2C_GPIO_Init()
{
    GPIO_Handle_t I2C_PINS;
    // SPI2 uses GPIOB pins
       I2C_PINS.pGPIOx = GPIOB;
       I2C_PINS.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;      // Alternate function mode
       I2C_PINS.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;      // Fast enough for SPI
       I2C_PINS.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;   // No pull-up/pull-down
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
	I2C_handle.I2C_Config.I2C_DeviceAddress=0X68;
	I2C_Init(&I2C_handle);


                                           // Apply settings
}


int main()
{
	data_len = strlen((char*)Tx_buf);
	I2C_GPIO_Init();
	I2Cx_INIT();

	I2C_PeripheralControl(I2C2,ENABLE);
	I2C_ManageAcking(I2C2,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C2_EV,ENABLE);
		I2C_IRQInterruptConfig(IRQ_NO_I2C2_ER,ENABLE);

		I2C_Slave_EnableDisablecallbackevents(I2C2,ENABLE);

	while(1)
	{

	}

}
void  I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{


	static uint32_t cnt = 0;
	static uint32_t w_ptr = 0;



	if(AppEv == I2C_ERROR_AF)
	{
		//This will happen during slave transmitting data to master .
		// slave should understand master needs no more data
		//slave concludes end of Tx


		//if the current active code is 0x52 then dont invalidate
		if(! (CommandCode == 0x52))
			CommandCode = 0XFF;

		//reset the cnt variable because its end of transmission
		cnt = 0;

		//Slave concludes it sent all the bytes when w_ptr reaches data_len
		if(w_ptr >= (data_len))
		{
			w_ptr=0;
			CommandCode = 0xff;
		}

	}else if (AppEv == I2C_EV_STOP)
	{
		//This will happen during end slave reception
		//slave concludes end of Rx

		cnt = 0;

	}else if (AppEv == I2C_EV_DATA_REQ)
	{
		//Master is requesting for the data . send data
		if(CommandCode == 0x51)
		{
			//Here we are sending 4 bytes of length information
			I2C_SlaveSendData(I2C2,((data_len >> ((cnt%4) * 8)) & 0xFF));
		    cnt++;
		}else if (CommandCode == 0x52)
		{
			//sending Tx_buf contents indexed by w_ptr variable
			I2C_SlaveSendData(I2C2,Tx_buf[w_ptr++]);
		}
	}else if (AppEv == I2C_EV_DATA_RCV)
	{
		//Master has sent command code, read it
		 CommandCode = I2C_SlaveReceiveData(I2C2);

	}
}


void I2C2_EV_IRQHandler(void)
{

	I2C_EV_IRQHandling(&I2C_handle);
}



void I2C2_ER_IRQHandler(void)
{

	I2C_ER_IRQHandling(&I2C_handle);
}

/*
 * i2c_recv_data_intr.c
 *
 *  Created on: Oct 30, 2025
 *      Author: karnati.ganesh
 */


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
volatile uint8_t Rcv_complete;

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
	char Rcv_data[32];
	I2C_GPIO_Init();
	I2Cx_INIT();
	I2C_IRQInterruptConfig(IRQ_NO_I2C2_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C2_ER,ENABLE);
	I2C_PeripheralControl(I2C2,ENABLE);
	I2C_ManageAcking(I2C2,ENABLE);

	button_config();
	uint8_t command;
	uint8_t len;
	while(1)
	{
		uint8_t flag;
		if(!(flag=GPIO_ReadFromInputPin(GPIOE,5)))
	    {
		   for(volatile int i=0;i<100000;i++);
           printf("%d\n",flag);
		   command=0x51;
		  while (I2C_MasterSendData_INTR(&I2C_handle,&command,1,0x68,ENABLE)!=I2C_READY);
		  while (I2C_MasterReceiveData_INTR(&I2C_handle,&len,1,0x68,ENABLE)!=I2C_READY);
		  while(!Rcv_complete);
		  Rcv_complete=0;
		   command=0x52;

			  while (I2C_MasterSendData_INTR(&I2C_handle,&command,1,0x68,ENABLE)!=I2C_READY);

		   while(I2C_MasterReceiveData_INTR(&I2C_handle,Rcv_data,len,0x68,DISABLE)!=I2C_READY);
		   while(!Rcv_complete);
		   		  Rcv_complete=0;
		   	Rcv_data[len+1]='\0';
		   	printf("Received data ='%s'\n",Rcv_data);

	    }

	}

}

void I2C2_EV_IRQHandler()
{
	 I2C_EV_IRQHandling(&I2C_handle);
}
void I2C2_ER_IRQHandler()
{
	I2C_ER_IRQHandling(&I2C_handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
    switch (AppEv)
    {
        case I2C_EV_TX_CMPLT:
            printf("I2C Transmission Complete\n");
            // Transmission finished — you could set a flag here
            // txCompleteFlag = 1;
            break;

        case I2C_EV_RX_CMPLT:
            printf("I2C Reception Complete\n");
            Rcv_complete=1;
            // Reception finished — process received data here
            // rxCompleteFlag = 1;
            break;

        case I2C_EV_STOP:
            printf("I2C STOP condition detected (Slave mode)\n");
            break;

        case I2C_EV_DATA_REQ:
            printf("Master requested data (Slave Transmit)\n");
            // Master is asking for data — send next byte
            // uint8_t data = get_next_byte();
            // I2C_SlaveSendData(pI2CHandle->pI2Cx, data);
            break;

        case I2C_EV_DATA_RCV:
            printf("Data received from Master (Slave Receive)\n");
            // Read received data
            // uint8_t value = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
            // process_data(value);
            break;

        case I2C_ERROR_BERR:
            printf("❌ I2C Bus Error detected\n");
            break;

        case I2C_ERROR_ARLO:
            printf("❌ I2C Arbitration Lost Error\n");
            break;

        case I2C_ERROR_AF:
            printf("❌ I2C Acknowledge Failure Error\n");
            I2C_CloseSendData(pI2CHandle);
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
            while(1);
            break;

        case I2C_ERROR_OVR:
            printf("❌ I2C Overrun/Underrun Error\n");
            break;

        case I2C_ERROR_TIMEOUT:
            printf("❌ I2C Timeout Error\n");
            break;

        default:
            // Unknown or unsupported event
            break;
    }
}


/*
 * i2c_slave_mode.c
 *
 *  Created on: Oct 31, 2025
 *      Author: karnati.ganesh
 */


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
	I2C_handle.I2C_Config.I2C_DeviceAddress=0X68;
	I2C_Init(&I2C_handle);


                                           // Apply settings
}

char Tx_data[32]="Course about to complete";
int main()
{

	I2C_GPIO_Init();
	I2Cx_INIT();
	I2C_IRQInterruptConfig(IRQ_NO_I2C2_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C2_ER,ENABLE);
	I2C_Slave_EnableDisablecallbackevents(I2C2,ENABLE);
	I2C_PeripheralControl(I2C2,ENABLE);
	I2C_ManageAcking(I2C2,ENABLE);

	while(1)
	{

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

	static uint8_t command;
	static uint8_t cnt=0;
    switch (AppEv)
    {




        case I2C_EV_DATA_REQ:
            printf("Master requested data (Slave Transmit)\n");
            if(command==0x51)
            {
            	I2C_SlaveSendData(pI2CHandle->pI2Cx,strlen(Tx_data));
            }
            else if(command==0x52)
            {
            	I2C_SlaveSendData(pI2CHandle->pI2Cx,Tx_data[cnt++]);
            }
            break;

        case I2C_EV_DATA_RCV:

            // Read received data
             command = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
             printf("Data received from Master (Slave Receive)\n");

            break;


        case I2C_ERROR_AF:
            command=0;
            cnt=0;

            break;

        case I2C_EV_STOP:
                    printf("I2C STOP condition detected (Slave mode)\n");
                    break;

        case I2C_ERROR_TIMEOUT:
            printf("❌ I2C Timeout Error\n");
            break;

        default:
            // Unknown or unsupported event
            break;
    }
}


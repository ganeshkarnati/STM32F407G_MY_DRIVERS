/*
 * spi_interrupt_RX_TX.c
 *
 *  Created on: Oct 17, 2025
 *      Author: karnati.ganesh
 */


/*
 * Basic_spi.c
 *
 *  Created on: Oct 14, 2025
 *      Author: karnati.ganesh
 */

#include <string.h>
#include "stm32f407_xx.h"
#include "stm32f407_spi_header.h"
#include<stdio.h>
#define MAX_LEN   500
SPI_Handle_t SPI2Handle;
char Rcvbuffer[500];
volatile uint8_t data_available = 0;
volatile uint8_t Rcv_stop=0;
volatile char Read_byte;
uint16_t i=0;
// Used to toggle LED ON/OFF state each button press

//-------------------------------------------------------------//
// Function: SPI_GPIO_Init
// Purpose : Configure GPIO pins for SPI2 (PB12–PB15)
//-------------------------------------------------------------//
void SPI_GPIO_Init()
{
    GPIO_Handle_t SPI_port;

    // SPI2 uses GPIOB pins
    SPI_port.pGPIOx = GPIOB;
    SPI_port.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;      // Alternate function mode
    SPI_port.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;      // Fast enough for SPI
    SPI_port.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;   // No pull-up/pull-down
    SPI_port.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;     // Push-pull output
    SPI_port.GPIO_PinConfig.GPIO_PinAltFunMode = 5;               // AF5 → SPI2

    // Configure SPI2 Pins: NSS, SCK, MISO, MOSI
    SPI_port.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;      // NSS (Slave Select)
    GPIO_Init(&SPI_port);

    SPI_port.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;      // SCK (Clock)
    GPIO_Init(&SPI_port);

    SPI_port.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;      // MISO (Master In Slave Out)
    GPIO_Init(&SPI_port);

    SPI_port.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;      // MOSI (Master Out Slave In)
    GPIO_Init(&SPI_port);
}

//-------------------------------------------------------------//
// Function: SPIx_INIT
// Purpose : Configure and enable SPI2 peripheral in Master mode
//-------------------------------------------------------------//
void SPIx_INIT()
{


    SPI2Handle.pSPIx = SPI2;
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MASTER;      // Master mode
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_FD;              // Full duplex
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_DIV8;           // SPI Clock = PCLK / 8 (~2 MHz)
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;                 // 8-bit data frame
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;                 // Clock idle low
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_FIRST;               // Capture data on 1st edge
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;                    // Hardware slave management

    SPI_Init(&SPI2Handle);                                        // Apply settings
}

//-------------------------------------------------------------//
// Function: interrupt_Button_init
// Purpose : Configure button on PE5 to generate EXTI interrupt
//-------------------------------------------------------------//
void interrupt_pin_init()
{
    GPIO_Handle_t pin;
    memset(&pin, 0, sizeof(pin));

    pin.pGPIOx = GPIOD;
    pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    pin.GPIO_PinConfig.GPIO_PinMode = INTR_F_EDGE;             // Trigger on falling edge
    pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
    pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_Init(&pin);

    // Enable interrupt in NVIC (EXTI lines 9–5)
    GPIO_IRQInterruptConfig(EXTI9_5_IRQn, ENABLE);
    GPIO_IRQPriorityConfig(EXTI9_5_IRQn, 5);
}

//-------------------------------------------------------------//
// Function: main
// Purpose : Initialize button, SPI and enable SSOE
//-------------------------------------------------------------//
int main()
{
    interrupt_pin_init();          // Configure interrupt for button
    SPI_GPIO_Init();                  // Configure GPIO pins for SPI
    SPIx_INIT();                      // Configure SPI2 peripheral
    SPI_SSOE_CONFIG(SPI2, ENABLE);    // Enable Slave Select Output (NSS control by hardware)
    SPI_IRQInterruptConfig( SPI2_IRQn,ENABLE);
    uint8_t dummy=0XE6;

	while (1) // MCU waits in idle loop; SPI handled by ISR
    {
    	Rcv_stop=0;
      //  printf("hello\n");
    	if(data_available)
    	{
    		data_available=0;
    	GPIO_IRQInterruptConfig( EXTI9_5_IRQn,DISABLE);
    	 SPI_PeripheralControl(SPI2, ENABLE);

    	 while(!Rcv_stop)
    	 {
    		 while(SPI_DATA_TX_INTR(&SPI2Handle,&dummy,1)==SPI_TX_BUSY);
    		while(SPI_DATA_RX_INTR(&SPI2Handle,&Read_byte,1)==SPI_RX_BUSY);

    	 }
    	 while(Return_flag(SPI2,SPI_SR_BSY ));
    	 SPI_PeripheralControl(SPI2,DISABLE);
    	 printf("SLAVE DATA : %s\n",Rcvbuffer);
    	 data_available=0;
    	 GPIO_IRQInterruptConfig( EXTI9_5_IRQn,ENABLE);
    	}


    }
}

void SPI2_IRQHandler()
{
	SPI_IRQHandling(&SPI2Handle);
	//printf("%skkkk  \n",Rcvbuffer);
}
//-------------------------------------------------------------//
// Function: EXTI9_5_IRQHandler
// Purpose : Button interrupt handler — send SPI command to slave
//-------------------------------------------------------------//
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(6);
	data_available=1;
}
void SPI_ApplicationEventCallback(SPI_Handle_t *pHandler, uint8_t flag)
{
    // Application can override this function to handle events
    // For example: Toggle LED on TX/RX complete or handle errors

	if(flag==RX_EVENT_COMPLETED)
	{
		Rcvbuffer[i++]=Read_byte;
		if(Read_byte=='\0'||i==MAX_LEN)
		{
			Rcv_stop=1;
			Rcvbuffer[i-1]='\0';
			i=0;
		}
	}
}


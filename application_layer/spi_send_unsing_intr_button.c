/*
 * Basic_spi.c
 *
 *  Created on: Oct 14, 2025
 *      Author: karnati.ganesh
 */
#include<string.h>
#include "stm32f407_xx.h"

char buffer[]="HAPPY BIRTHDAY GYANAPRAKASH";
void SPI_GPIO_Init()
{
	// Create a GPIO handle structure for SPI pins
		GPIO_Handle_t SPI_port;

		// Select port B for SPI2 pins (PB12 → NSS, PB13 → SCK, PB14 → MISO, PB15 → MOSI)
		SPI_port.pGPIOx = GPIOB;

		// Configure all pins for Alternate Function mode (to connect to SPI peripheral)
		SPI_port.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;

		// Set medium speed for SPI signals (enough for most applications)
		SPI_port.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

		// No pull-up or pull-down resistors (SPI lines are actively driven)
		SPI_port.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		// Output type as Push-Pull (standard for SPI)
		SPI_port.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

		SPI_port.GPIO_PinConfig.GPIO_PinAltFunMode=5;

		//--- Configure individual SPI2 pins one by one ---//

		// 1. Configure NSS (Slave Select) → PB12
		SPI_port.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		GPIO_Init(&SPI_port);

		// 2. Configure SCK (Clock) → PB13
		SPI_port.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
		GPIO_Init(&SPI_port);

		// 3. Configure MISO (Master In Slave Out) → PB14
	//	SPI_port.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//	GPIO_Init(&SPI_port);

		// 4. Configure MOSI (Master Out Slave In) → PB15
		SPI_port.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
		GPIO_Init(&SPI_port);

}

/*
 * Function: SPI2_INIT
 * -------------------
 * Configures SPI2 peripheral as a master with basic settings.
 * This function creates a handle for SPI2, fills in the configuration
 * structure, and calls SPI_Init() to apply the settings.
 *
 * Configuration:
 *   - SPI2 peripheral
 *   - Master mode
 *   - Full duplex communication
 *   - Baud rate: f_PCLK / 2
 *   - 8-bit data frame
 *   - CPOL = 0 (clock idle low)
 *   - CPHA = 0 (data captured on first edge)
 *   - Software slave management enabled (SSM)
 */
void SPIx_INIT()
{
    // 1. Create an SPI handle structure
    SPI_Handle_t SPI2Handle;

    // 2. Specify which SPI peripheral to configure
    SPI2Handle.pSPIx = SPI2;

    // 3. Select SPI device mode: Master or Slave
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MASTER;

    // 4. Select communication type: Full duplex / Half duplex / Simplex
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_FD;

    // 5. Set the SPI serial clock speed (f_PCLK / 2)
    //    The faster the prescaler, the faster the SPI clock.
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_DIV8;//2MHZ(just for fun)

    // 6. Set data frame format (8-bit or 16-bit)
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;

    // 7. Set clock polarity (CPOL = 0 → clock idle state is LOW)
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;

    // 8. Set clock phase (CPHA = 0 → data captured on 1st edge)
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_FIRST;

    // 9. Enable software slave management (manual control of NSS)
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

    // 10. Call the initialization function to apply all settings
    SPI_Init(&SPI2Handle);
}

void interrupt_Button_init()
{
	GPIO_Handle_t button;
	memset(&button,0,sizeof(button));
	button.pGPIOx=GPIOE;
button.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
button.GPIO_PinConfig.GPIO_PinMode=INTR_R_EDGE;
button.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_MEDIUM;
button.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PD;
GPIO_Init(&button);
GPIO_IRQInterruptConfig( EXTI9_5_IRQn,ENABLE);
GPIO_IRQPriorityConfig(EXTI9_5_IRQn,5);
}
int main()
{
   interrupt_Button_init();
	SPI_GPIO_Init();
	SPIx_INIT();
	SPI_SSOE_CONFIG(SPI2,ENABLE);
	while(1);
}
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(5);
	for( int i=0;i<20000;i++);
	  SPI_PeripheralControl(SPI2,ENABLE);
		  uint8_t len=strlen(buffer);
		  printf("%d\n",len);
		  SPI_DATA_TX(SPI2,&len,1);//send length information to the slave
		  SPI_DATA_TX(SPI2,buffer,strlen(buffer));
		  SPI_PeripheralControl(SPI2,DISABLE);

}




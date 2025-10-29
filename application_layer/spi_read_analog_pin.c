/*
 * Basic_spi.c
 *
 *  Created on: Oct 14, 2025
 *      Author: karnati.ganesh
 */

#include <string.h>
#include "stm32f407_xx.h"

//--------------------- SPI Command Codes ---------------------//
#define LED_CNTR      0x50
#define SENSOR_READ   0x51
#define LED_READ      0x52
#define LED_PRINT     0x53
#define ID_READ       0x54

//--------------------- LED Control Values --------------------//
#define LED_ON        1
#define LED_OFF       0
#define LED_PIN       7

//--------------------- Sensor Config -------------------------//
#define ANALOG_PIN_0  0

//--------------------- Globals -------------------------------//
char buffer[] = "HAPPY BIRTHDAY GYANAPRAKASH";
uint8_t flag = 1;

//-------------------------------------------------------------//
// Function: SPI_GPIO_Init
// Purpose : Configure GPIO pins for SPI2 (PB12–PB15)
//-------------------------------------------------------------//
void SPI_GPIO_Init()
{
    GPIO_Handle_t SPI_port;

    // SPI2 uses port B (NSS, SCK, MISO, MOSI)
    SPI_port.pGPIOx = GPIOB;
    SPI_port.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;      // Alternate function
    SPI_port.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;      // Fast speed for SPI
    SPI_port.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;   // No pull-up/pull-down
    SPI_port.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;     // Push-pull output
    SPI_port.GPIO_PinConfig.GPIO_PinAltFunMode = 5;               // AF5 for SPI2

    // Configure individual pins for SPI2
    SPI_port.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;  // NSS
    GPIO_Init(&SPI_port);

    SPI_port.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;  // SCK
    GPIO_Init(&SPI_port);

    SPI_port.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;  // MISO
    GPIO_Init(&SPI_port);

    SPI_port.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;  // MOSI
    GPIO_Init(&SPI_port);
}

//-------------------------------------------------------------//
// Function: SPIx_INIT
// Purpose : Initialize SPI2 peripheral as Master
//-------------------------------------------------------------//
void SPIx_INIT()
{
    SPI_Handle_t SPI2Handle;

    SPI2Handle.pSPIx = SPI2;
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MASTER;
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_FD;          // Full duplex
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_DIV8;       // fPCLK/8 (~2 MHz)
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;             // 8-bit frame
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;             // Clock idle low
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_FIRST;           // Capture on 1st edge
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;                // Hardware slave management

    SPI_Init(&SPI2Handle);
}

//-------------------------------------------------------------//
// Function: interrupt_Button_init
// Purpose : Configure button on PE5 to trigger EXTI interrupt
//-------------------------------------------------------------//
void interrupt_Button_init()
{
    GPIO_Handle_t button;
    memset(&button, 0, sizeof(button));

    button.pGPIOx = GPIOE;
    button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    button.GPIO_PinConfig.GPIO_PinMode = INTR_R_EDGE;          // Interrupt on rising edge
    button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
    button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;   // Pull-down resistor

    GPIO_Init(&button);

    // Enable interrupt line in NVIC
    GPIO_IRQInterruptConfig(EXTI9_5_IRQn, ENABLE);
    GPIO_IRQPriorityConfig(EXTI9_5_IRQn, 5);
}

//-------------------------------------------------------------//
// Function: main
// Purpose : Entry point, initialize SPI & button
//-------------------------------------------------------------//
int main()
{
    interrupt_Button_init();      // Enable button interrupt
    SPI_GPIO_Init();              // Configure SPI2 pins
    SPIx_INIT();                  // Configure SPI2 peripheral
    SPI_SSOE_CONFIG(SPI2, ENABLE);// Enable slave select output

    while (1); // Wait forever, actual work happens in ISR
}

//-------------------------------------------------------------//
// Function: EXTI9_5_IRQHandler
// Purpose : Handles button press interrupt and performs SPI
//-------------------------------------------------------------//
void EXTI9_5_IRQHandler(void)
{
    uint8_t dummy_read;
    uint8_t dummy_write = 0xE5;   // Dummy data for SPI full-duplex
    uint8_t arg[2];
    uint8_t ack_bit;

    // Clear EXTI interrupt pending bit
    GPIO_IRQHandling(5);

    // Simple debounce delay
    for (int i = 0; i < 20000; i++);

    // Enable SPI2 peripheral
    SPI_PeripheralControl(SPI2, ENABLE);

    // Command to send to slave
    uint8_t command = SENSOR_READ;

    //------------------- Send Command -------------------//
    SPI_DATA_TX(SPI2, &command, 1);
    SPI_DATA_RX(SPI2, &dummy_read, 1);   // Read dummy to clear RXNE

    //------------------- Get Acknowledgement ------------//
    SPI_DATA_TX(SPI2, &dummy_write, 1);  // Send dummy to clock-in ACK
    SPI_DATA_RX(SPI2, &ack_bit, 1);      // Receive ACK byte from slave

    //------------------- If ACK Valid -------------------//
    if (ack_bit == 0xE5)
    {
        arg[0] = ANALOG_PIN_0;           // Argument: analog channel
        SPI_DATA_TX(SPI2, arg, 1);       // Send argument
    }
    else
    {
        printf("bokka\n");               // Debug message if ACK invalid
    }

    SPI_DATA_RX(SPI2, &dummy_read, 1);   // Clear RXNE again

    //------------------- ADC Conversion Delay for Aurdino -----------//
    for (uint32_t i = 0; i < 200000; i++); // Delay for Arduino ADC

    //------------------- Read ADC Value -----------------//
    SPI_DATA_TX(SPI2, &dummy_write, 1);  // Send dummy to read analog
    uint8_t analog_value;
    SPI_DATA_RX(SPI2, &analog_value, 1); // Get analog value

    printf("%d\n", analog_value);        // Print ADC value on SWV console

    //------------------- Disable SPI --------------------//
    SPI_PeripheralControl(SPI2, DISABLE);
}

/*
 * Basic_spi.c
 *
 *  Created on: Oct 14, 2025
 *      Author: karnati.ganesh
 */

#include <string.h>
#include "stm32f407_xx.h"

//---------------------- SPI Command Codes ----------------------//
#define LED_CNTR      0x50
#define SENSOR_READ   0x51
#define LED_READ      0x52
#define LED_PRINT     0x53
#define ID_READ       0x54

//---------------------- LED Control Values ----------------------//
#define LED_ON        1
#define LED_OFF       0
#define LED_PIN       7

//---------------------- Sensor Configuration --------------------//
#define ANALOG_PIN_0  0

//---------------------- Global Variables ------------------------//
char buffer[] = "HAPPY BIRTHDAY GYANAPRAKASH";
uint8_t flag = 1;   // Used to toggle LED ON/OFF state each button press

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
    SPI_Handle_t SPI2Handle;

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
void interrupt_Button_init()
{
    GPIO_Handle_t button;
    memset(&button, 0, sizeof(button));

    button.pGPIOx = GPIOE;
    button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    button.GPIO_PinConfig.GPIO_PinMode = INTR_R_EDGE;             // Trigger on rising edge
    button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
    button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;      // Internal pull-down

    GPIO_Init(&button);

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
    interrupt_Button_init();          // Configure interrupt for button
    SPI_GPIO_Init();                  // Configure GPIO pins for SPI
    SPIx_INIT();                      // Configure SPI2 peripheral
    SPI_SSOE_CONFIG(SPI2, ENABLE);    // Enable Slave Select Output (NSS control by hardware)

    while (1); // MCU waits in idle loop; SPI handled by ISR
}

//-------------------------------------------------------------//
// Function: EXTI9_5_IRQHandler
// Purpose : Button interrupt handler — send SPI command to slave
//-------------------------------------------------------------//
void EXTI9_5_IRQHandler(void)
{
    uint8_t dummy_read;
    uint8_t dummy_write = 0xE5;   // Dummy byte used to generate clock
    uint8_t arg[2];
    uint8_t ack_bit;

    // 1. Clear EXTI pending flag (acknowledge interrupt)
    GPIO_IRQHandling(5);

    // 2. Simple debounce delay to avoid multiple triggers
    for (int i = 0; i < 20000; i++);

    // 3. Enable SPI peripheral
    SPI_PeripheralControl(SPI2, ENABLE);

    // 4. Choose command to send — here SENSOR_READ or LED control
    uint8_t command = SENSOR_READ;
    // uint8_t command = LED_CNTR;

    //---------------- Send Command Byte ----------------//
    SPI_DATA_TX(SPI2, &command, 1);

    // After sending one byte in full duplex mode,
    // the RX buffer will also have garbage data.
    // Read it to clear RXNE flag.
    SPI_DATA_RX(SPI2, &dummy_read, 1);

    //---------------- Get Acknowledgement --------------//
    // Send a dummy byte to generate clock for the slave
    SPI_DATA_TX(SPI2, &dummy_write, 1);
    SPI_DATA_RX(SPI2, &ack_bit, 1); // Receive slave ACK byte

    //---------------- Process ACK ----------------------//
    if (ack_bit == 0xE5)
    {
        // Toggle LED state on each button press
        if (flag)
        {
            arg[0] = LED_PIN;
            arg[1] = LED_ON;
            SPI_DATA_TX(SPI2, arg, 2); // Send LED ON command
            flag = 0;
        }
        else
        {
            arg[0] = LED_PIN;
            arg[1] = LED_OFF;
            SPI_DATA_TX(SPI2, arg, 2); // Send LED OFF command
            flag = 1;
        }
    }
    else
    {
        printf("bokka\n"); // Debug message (ACK not received)
    }

    // 5. Disable SPI peripheral after communication
    SPI_PeripheralControl(SPI2, DISABLE);
}

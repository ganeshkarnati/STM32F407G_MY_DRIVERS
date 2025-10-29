/*
 * stm32f407_spi_header.h
 *
 *  Created on: Oct 14, 2025
 *      Author: karnati.ganesh
 */

#ifndef INC_STM32F407_SPI_HEADER_H_
#define INC_STM32F407_SPI_HEADER_H_

#include<stdint.h>
#define __VO  volatile

//SPI PERIPHERAL CLOCK ENABLE MACROS
#define SPI1_PCLK_EN()  (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()  (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()  (RCC->APB1ENR |= (1<<15))


//SPI CLOCK DISABLE MACROS
#define SPI1_PCLK_DI()  (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()  (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()  (RCC->APB1ENR &= ~(1<<15))


// SPIx peripherals reset

#define SPI1_REG_RESET()	do{ (RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<12)); }  while(0)
#define SPI2_REG_RESET()	do{ (RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14)); } while(0)
#define SPI3_REG_RESET()	do{ (RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15)); } while(0)


///SPI PERIPHERAL BASE ADDRESSES
#define SPI1_BASEADDR     0x40013000U
#define SPI2_BASEADDR     0x40003800U
#define SPI3_BASEADDR     0x40003C00U


//SIMPLE MACROS
#define SPI1          	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2          	((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3          	((SPI_RegDef_t*)SPI3_BASEADDR)


// Device mode
#define SPI_DEVICE_MASTER    1
#define SPI_DEVICE_SLAVE     0

// Bus configuration
#define SPI_BUS_FD              1   // Full-duplex
#define SPI_BUS_HD              2   // Half-duplex
#define SPI_BUS_SIMPLEX_RXONLY  3 // Simplex RX only

// Clock speed (prescaler)
#define SPI_SCLK_DIV2        0
#define SPI_SCLK_DIV4        1
#define SPI_SCLK_DIV8        2
#define SPI_SCLK_DIV16       3
#define SPI_SCLK_DIV32       4
#define SPI_SCLK_DIV64       5
#define SPI_SCLK_DIV128      6
#define SPI_SCLK_DIV256      7

// Data frame format
#define SPI_DFF_8BITS        0
#define SPI_DFF_16BITS       1

// Clock polarity
#define SPI_CPOL_LOW         0
#define SPI_CPOL_HIGH        1

// Clock phase
#define SPI_CPHA_FIRST       0
#define SPI_CPHA_SECOND      1

// Software slave management
#define SPI_SSM_DI           0
#define SPI_SSM_EN           1



typedef struct
{
    __VO uint32_t SPI_CR1;       /*!< SPI control register 1,               Address offset: 0x00 */
    __VO uint32_t SPI_CR2;       /*!< SPI control register 2,               Address offset: 0x04 */
    __VO uint32_t SPI_SR;        /*!< SPI status register,                  Address offset: 0x08 */
    __VO uint32_t SPI_DR;        /*!< SPI data register,                    Address offset: 0x0C */
    __VO uint32_t SPI_CRCPR;     /*!< SPI CRC polynomial register,          Address offset: 0x10 */
    __VO uint32_t SPI_RXCRCR;    /*!< SPI RX CRC register,                  Address offset: 0x14 */
    __VO uint32_t SPI_TXCRCR;    /*!< SPI TX CRC register,                  Address offset: 0x18 */
    __VO uint32_t SPI_I2SCFGR;   /*!< SPI_I2S configuration register,       Address offset: 0x1C */
    __VO uint32_t SPI_I2SPR;     /*!< SPI_I2S prescaler register,           Address offset: 0x20 */
} SPI_RegDef_t;

//configuration register
typedef struct
{
    uint8_t SPI_DeviceMode;   // Master/Slave
    uint8_t SPI_BusConfig;    // Full-duplex/Half-duplex/Simplex
    uint8_t SPI_SclkSpeed;    // Clock prescaler
    uint8_t SPI_DFF;          // 8-bit or 16-bit
    uint8_t SPI_CPOL;         // Clock polarity
    uint8_t SPI_CPHA;         // Clock phase
    uint8_t SPI_SSM;          // Software slave management
} SPI_Config_t;

/*
 * SPI_Handle_t
 * -------------
 * This structure acts as a "handle" for managing an SPI peripheral instance.
 * It holds both:
 *   1. A pointer to the SPI peripheral base address.
 *   2. Configuration settings (from initialization).
 *   3. Data transfer state information (buffers, lengths, and status flags).
 *
 * The handle is typically passed to driver functions like:
 *   - SPI_Init()
 *   - SPI_SendData()
 *   - SPI_ReceiveData()
 *   - SPI_IRQHandling()
 */

typedef struct
{
    SPI_RegDef_t *pSPIx;       // Pointer to SPI peripheral base address (e.g., SPI1, SPI2, SPI3)
                              // Used to access the SPI registers directly.

    SPI_Config_t SPIConfig;    // Structure containing SPI configuration settings like:
                              // - Device mode (Master/Slave)
                              // - Bus configuration (Full/Half duplex)
                              // - Clock speed
                              // - Data frame format
                              // - CPOL & CPHA

    uint8_t *TX_buffer_addrs;  // Pointer to transmit (TX) buffer
                              // Holds the address of data to be sent.

    uint8_t *RX_buffer_addrs;  // Pointer to receive (RX) buffer
                              // Holds the address where received data will be stored.

    uint8_t TX_len;            // Length of data to be transmitted (in bytes)
                              // Used by driver to keep track of remaining bytes during transmission.

    uint8_t RX_len;            // Length of data to be received (in bytes)
                              // Used by driver during reception.

    uint8_t TX_state;          // Transmission state flag:
                              // - 0 → SPI ready for new TX
                              // - 1 → SPI currently transmitting data

    uint8_t RX_state;          // Reception state flag:
                              // - 0 → SPI ready for new RX
                              // - 1 → SPI currently receiving data

} SPI_Handle_t;



void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_Init(SPI_Handle_t *pSPIHandle);

void SPI_DeInit(SPI_RegDef_t *pSPIx);

uint8_t Return_flag(SPI_RegDef_t* pSPIx, uint8_t Bit_no);

void SPI_DATA_TX(SPI_RegDef_t *pSPIx,uint8_t* data,uint32_t data_len);

void SPI_DATA_RX(SPI_RegDef_t *pSPIx,char* data,uint32_t data_len);

uint8_t SPI_DATA_TX_INTR(SPI_Handle_t *pHandler,uint8_t* data,uint32_t data_len);

uint8_t SPI_DATA_RX_INTR(SPI_Handle_t *pHandler, uint8_t* data,uint32_t data_len);


void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void SPI_IRQHandling();
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSI_CONFIG(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOE_CONFIG(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_ApplicationEventCallback(SPI_Handle_t *pHandler,uint8_t flag);

//CONTROL REGISTER

#define SPI_CR1_CPHA       0   // Clock Phase (0: 1st edge, 1: 2nd edge)
#define SPI_CR1_CPOL       1   // Clock Polarity (0: Idle low, 1: Idle high)
#define SPI_CR1_MSTR       2   // Master Selection (0: Slave, 1: Master)
#define SPI_CR1_BR0        3   // Baud rate control bit 0
#define SPI_CR1_BR1        4   // Baud rate control bit 1
#define SPI_CR1_BR2        5   // Baud rate control bit 2
#define SPI_CR1_SPE        6   // SPI Enable (0: Disabled, 1: Enabled)
#define SPI_CR1_DFF        11  // Data Frame Format (0: 8-bit, 1: 16-bit)
#define SPI_CR1_SSM        9   // Software Slave Management (0: Hardware NSS, 1: Software NSS)
#define SPI_CR1_SSI        8   // Internal slave select (used with SSM)
#define SPI_CR1_BIDIMODE   15  // Bit for bidirectional mode
#define SPI_CR1_RXONLY     10  // Bit for RX-only mode

//SPI CR2

#define SPI_CR2_RXDMAEN      0
#define SPI_CR2_TXDMAEN      1
#define SPI_CR2_SSOE         2
#define SPI_CR2_NSSP         3
#define SPI_CR2_FRF          4
#define SPI_CR2_ERRIE        5
#define SPI_CR2_RXNEIE       6
#define SPI_CR2_TXEIE        7
#define SPI_CR2_DS           8
#define SPI_CR2_FRXTH        12
#define SPI_CR2_LDMA_RX      14
#define SPI_CR2_LDMA_TX      15

//STATUS REGISTER

#define SPI_SR_RXNE      0   // Receive buffer not empty
#define SPI_SR_TXE       1   // Transmit buffer empty
#define SPI_SR_CHSIDE    2   // Channel side (I2S mode)
#define SPI_SR_UDR       3   // Underrun flag (slave mode)
#define SPI_SR_CRCERR    4   // CRC error flag
#define SPI_SR_MODF      5   // Mode fault
#define SPI_SR_OVR       6   // Overrun flag
#define SPI_SR_BSY       7   // Busy flag
#define SPI_SR_FRE       8   // Frame format error

//IRQ number for SPI
#define SPI1_IRQn       35
#define SPI2_IRQn       36
#define SPI3_IRQn       51

//SPI STATE
#define SPI_READY      0
#define SPI_TX_BUSY    1
#define SPI_RX_BUSY    2

#define TX_EVENT_COMPLETED 0
#define RX_EVENT_COMPLETED 1
#define OVR_EVENT_COMPLETED 2

#define CLEAR    0

#endif /* INC_STM32F407_SPI_HEADER_H_ */

/*
 * stm32f407_i2c.h
 *
 *  Created on: Oct 27, 2025
 *      Author: karnati.ganesh
 */

#ifndef INC_STM32F407_I2C_H_
#define INC_STM32F407_I2C_H_

#include "stm32f407_xx.h"

/***********************************************
 * I2C Register Definition Structure for STM32F407
 ***********************************************/
typedef struct
{
    volatile uint32_t I2C_CR1;      // 0x00: Control Register 1
    volatile uint32_t I2C_CR2;      // 0x04: Control Register 2
    volatile uint32_t I2C_OAR1;     // 0x08: Own Address Register 1
    volatile uint32_t I2C_OAR2;     // 0x0C: Own Address Register 2
    volatile uint32_t I2C_DR;       // 0x10: Data Register
    volatile uint32_t I2C_SR1;      // 0x14: Status Register 1
    volatile uint32_t I2C_SR2;      // 0x18: Status Register 2
    volatile uint32_t I2C_CCR;      // 0x1C: Clock Control Register
    volatile uint32_t I2C_TRISE;    // 0x20: TRISE Register
    volatile uint32_t I2C_FLTR;     // 0x24: Filter Register
} I2C_Regdef_t;


/*******************************************************
 * @I2C_Config_t
 * Configuration structure for I2C peripheral
 *******************************************************/
typedef struct
{
    uint32_t I2C_SCLSpeed;       // Serial clock speed (Standard/Fast mode)
    uint8_t  I2C_DeviceAddress;  // Own slave address
    uint8_t  I2C_ACKControl;     // Enable/Disable ACK bit
    uint16_t I2C_FMDutyCycle;    // Fast Mode duty cycle (Tlow/Thigh = 2 or 16/9)
} I2C_Config_t;

/*******************************************************
 * @I2C_Handle_t
 * Handle structure for I2C peripheral
 *******************************************************/
typedef struct
{
    I2C_Regdef_t   *pI2Cx;        // Base address of I2C peripheral registers
    I2C_Config_t    I2C_Config;   // I2C configuration settings

    uint8_t        *pTxBuffer;    // Pointer to transmission buffer
    uint8_t        *pRxBuffer;    // Pointer to reception buffer

    uint32_t        TxLen;        // Transmission length
    uint32_t        RxLen;        // Reception length

    uint8_t         TxRxState;    // Communication state (Tx/Rx/Ready)
    uint8_t         DevAddr;      // Target slave address
    uint32_t        RxSize;       // Size of reception (for repeated start)
    uint8_t         Sr;           // Repeated start flag
} I2C_Handle_t;


#define I2C_CR1_PE          0   // Peripheral enable
#define I2C_CR1_SMBUS       1   // SMBus mode
#define I2C_CR1_ENARP       4   // ARP enable
#define I2C_CR1_ENPEC       5   // PEC enable
#define I2C_CR1_ENGC        6   // General call enable
#define I2C_CR1_NOSTRETCH   7   // Clock stretching disable
#define I2C_CR1_START       8   // Start generation
#define I2C_CR1_STOP        9   // Stop generation
#define I2C_CR1_ACK         10  // Acknowledge enable
#define I2C_CR1_POS         11  // ACK/PEC position
#define I2C_CR1_PEC         12  // Packet error checking
#define I2C_CR1_ALERT       13  // SMBus alert
#define I2C_CR1_SWRST       15  // Software reset



#define I2C_CR2_FREQ        0   // Frequency bits start (bits 5:0)
#define I2C_CR2_ITERREN     8   // Error interrupt enable
#define I2C_CR2_ITEVTEN     9   // Event interrupt enable
#define I2C_CR2_ITBUFEN     10  // Buffer interrupt enable
#define I2C_CR2_DMAEN       11  // DMA requests enable
#define I2C_CR2_LAST        12  // DMA last transfer


#define I2C_SR1_SB           0   // Start bit (Master mode) – set after START condition generated
#define I2C_SR1_ADDR         1   // Address sent (Master) / matched (Slave)
#define I2C_SR1_BTF          2   // Byte transfer finished
#define I2C_SR1_ADD10        3   // 10-bit header sent (Master mode)
#define I2C_SR1_STOPF        4   // Stop detection (Slave mode)
#define I2C_SR1_RXNE         6   // Data register not empty (data received)
#define I2C_SR1_TXE          7   // Data register empty (data can be written)
#define I2C_SR1_BERR         8   // Bus error
#define I2C_SR1_ARLO         9   // Arbitration lost
#define I2C_SR1_AF           10  // Acknowledge failure
#define I2C_SR1_OVR          11  // Overrun/Underrun
#define I2C_SR1_PECERR       12  // PEC error in reception
#define I2C_SR1_TIMEOUT      14  // Timeout or Tlow error
#define I2C_SR1_SMBALERT     15  // SMBus alert


#define I2C_SR2_MSL          0   // Master/slave – 1: Master mode
#define I2C_SR2_BUSY         1   // Bus busy
#define I2C_SR2_TRA          2   // Transmitter/receiver – 1: Transmitter mode
#define I2C_SR2_GENCALL      4   // General call address (Slave mode)
#define I2C_SR2_SMBDEFAULT   5   // SMBus device default address
#define I2C_SR2_SMBHOST      6   // SMBus host header
#define I2C_SR2_DUALF        7   // Dual flag (Dual addressing mode)
#define I2C_SR2_PEC          8   // Packet error checking register (8-bit value)


#define I2C_CCR_CCR          0   // Clock control bits [11:0]
#define I2C_CCR_DUTY         14  // Fast mode duty cycle – 0: 2, 1: 16/9
#define I2C_CCR_FS           15  // I2C master mode selection – 0: Standard mode, 1: Fast mode

// @I2C_SCLSpeed (Clock speed options)
#define I2C_SCL_SPEED_SM     100000   // 100 kHz Standard mode
#define I2C_SCL_SPEED_FM2K   200000   // 200 kHz Fast mode
#define I2C_SCL_SPEED_FM4K   400000   // 400 kHz Fast mode

// @I2C_ACKControl (ACK enable/disable)
#define I2C_ACK_ENABLE       1
#define I2C_ACK_DISABLE      0

// @I2C_FMDutyCycle (Fast Mode duty ratio)
#define I2C_FM_DUTY_2        0    // Tlow/Thigh = 2
#define I2C_FM_DUTY_16_9     1    // Tlow/Thigh = 16/9

/********************** I2C SR1 Flag Macros **********************/
#define I2C_FLAG_SB          (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR        (1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF         (1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10       (1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF       (1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE        (1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE         (1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR        (1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO        (1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF          (1 << I2C_SR1_AF)
#define I2C_FLAG_OVR         (1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR      (1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT     (1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT    (1 << I2C_SR1_SMBALERT)

/********************** I2C SR2 Flag Macros **********************/
#define I2C_FLAG_MSL         (1 << I2C_SR2_MSL)
#define I2C_FLAG_BUSY        (1 << I2C_SR2_BUSY)
#define I2C_FLAG_TRA         (1 << I2C_SR2_TRA)
#define I2C_FLAG_GENCALL     (1 << I2C_SR2_GENCALL)
#define I2C_FLAG_SMBDEFAULT  (1 << I2C_SR2_SMBDEFAULT)
#define I2C_FLAG_SMBHOST     (1 << I2C_SR2_SMBHOST)
#define I2C_FLAG_DUALF       (1 << I2C_SR2_DUALF)

#define read   1
#define write  0

// 1. Peripheral Clock Control
void I2C_PeriClockControl(I2C_Regdef_t *pI2Cx, uint8_t EnOrDi);

// 2. Initialization and De-initialization
void I2C_Init(I2C_Handle_t *pI2CHandle);
//void I2C_DeInit(I2C_Regdef_t *pI2Cx);
void I2C_PeripheralControl(I2C_Regdef_t *pI2Cx, uint8_t EnOrDi);

// 3. Data Send and Receive
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr);

//void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
//uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

// 4. Interrupt Configuration and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);   // Event interrupt handler
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);   // Error interrupt handler

// 5. Peripheral Enable/Disable
void I2C_ManageAcking(I2C_Regdef_t *pI2Cx, uint8_t EnOrDi);

// 6. Application Callback (for events like Tx complete, Rx complete, errors)
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#include "stm32f407_drivers.h"

#endif /* INC_STM32F407_I2C_H_ */

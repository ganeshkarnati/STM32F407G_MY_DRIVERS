/*
 * stm32f407_i2c.c
 *
 *  Created on: Oct 27, 2025
 *      Author: karnati.ganesh
 */
#include "stm32f407_xx.h"
#include "stm32f407_drivers.h"
#include "stm32f407_i2c.h"

#include "stm32f407_xx.h"

uint16_t AHB_PreScaler[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint8_t APB_PreScaler[4] = { 2, 4, 8, 16 };

static uint8_t I2C_GetFlagStatus(I2C_Regdef_t *pI2Cx, uint32_t FlagName) {

	// Check SR1 flags
	if (pI2Cx->I2C_SR1 & FlagName)
		return SET;

	return CLEAR;
}

static void I2C_GenerateStartCondition(I2C_Regdef_t *pI2Cx) {

	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhase(I2C_Regdef_t *pI2Cx, uint8_t SlaveAddr,
		uint8_t RW) {
	/*
	 * SlaveAddr : 7-bit slave address
	 * RW        : 0 → Write, 1 → Read
	 *
	 * SDA line will send [SlaveAddr << 1 | RW]
	 */

	uint8_t addr = (SlaveAddr << 1) | (RW & 0x01); // append R/W bit
	pI2Cx->I2C_DR = addr;  // load into data register to start address phase
}

void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle) {
	uint32_t dummy_read;

	// In master mode
	if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL)) {
		// Master transmitting or receiving
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			// If only 1 byte to receive
			if (pI2CHandle->RxSize == 1) {
				// Disable ACK before clearing ADDR flag
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
			}
		}
	}

	// Read SR1 followed by SR2 to clear ADDR flag
	dummy_read = pI2CHandle->pI2Cx->I2C_SR1;
	dummy_read = pI2CHandle->pI2Cx->I2C_SR2;
	(void) dummy_read;  // prevent compiler warning
}

 void I2C_GenerateStopCondition(I2C_Regdef_t *pI2Cx) {
	// Set the STOP bit in CR1 register
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_ManageAcking(I2C_Regdef_t *pI2Cx, uint8_t EnOrDi) {
	/*
	 * EnOrDi: ENABLE = 1 → ACK after each byte
	 *          DISABLE = 0 → Send NACK after current byte
	 */

	if (EnOrDi) {
		// Enable ACK
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
	} else {
		// Disable ACK
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
	}
}
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	// Check if user wants to ENABLE the interrupt
	if (EnOrDi == ENABLE)
	{
		// ------------- ENABLE SECTION -------------
		if (IRQNumber <= 31)
		{
			// NVIC_ISER0 (Interrupt Set-Enable Register 0) handles IRQs 0–31
			*NVIC_ISER0 |= (1 << IRQNumber);       // Set bit to 1 → enable that interrupt line
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			// NVIC_ISER1 (Interrupt Set-Enable Register 1) handles IRQs 32–63
			*NVIC_ISER1 |= (1 << (IRQNumber % 32)); // Mod 32 gives position within this register
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// NVIC_ISER2 (Interrupt Set-Enable Register 2) handles IRQs 64–95
			*NVIC_ISER2 |= (1 << (IRQNumber % 64)); // Again, calculate bit position
		}
	}
	else
	{
		// ------------- DISABLE SECTION -------------
		if (IRQNumber <= 31)
		{
			// NVIC_ICER0 (Interrupt Clear-Enable Register 0) disables IRQs 0–31
			*NVIC_ICER0 |= (1 << IRQNumber);       // Write 1 → disables that interrupt line
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			// NVIC_ICER1 handles IRQs 32–63
			*NVIC_ICER1 |= (1 << (IRQNumber % 32)); // Disable corresponding IRQ
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// NVIC_ICER2 handles IRQs 64–95
			*NVIC_ICER2 |= (1 << (IRQNumber % 64)); // Disable corresponding IRQ
		}
	}
}

void I2C_PeriClockControl(I2C_Regdef_t *pI2Cx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE)   // If user wants to enable the I2C peripheral clock
	{
		// Check which I2C peripheral is selected and enable its clock
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();   // Enable clock for I2C1
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();   // Enable clock for I2C2
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();   // Enable clock for I2C3
		}
	} else   // If user wants to disable the I2C peripheral clock
	{
		// Check which I2C peripheral is selected and disable its clock
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();   // Disable clock for I2C1
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();   // Disable clock for I2C2
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();   // Disable clock for I2C3
		}
	}
}

void I2C_Slave_EnableDisablecallbackevents(I2C_Regdef_t *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        // Enable error, event and buffer interrupts
        pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);  // Error interrupt enable
        pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);  // Event interrupt enable
        pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);  // Buffer interrupt enable
    }
    else
    {
        // Disable error, event and buffer interrupts
        pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITERREN);
        pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);
        pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);
    }
}


void I2C_PeripheralControl(I2C_Regdef_t *pI2Cx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		// Enable the I2C peripheral by setting the PE (Peripheral Enable) bit
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
	} else {
		// Disable the I2C peripheral by clearing the PE bit
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
	}
}

uint8_t RCC_GetPLLOutputClock() {
	return 16000000;
}
uint32_t RCC_GetPCLK1Value(void) {
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1p;

	// 1. Get system clock source
	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if (clksrc == 0)
		SystemClk = 16000000;       // HSI
	else if (clksrc == 1)
		SystemClk = 8000000;        // HSE
	else if (clksrc == 2)
		SystemClk = RCC_GetPLLOutputClock(); // PLL output (user-defined function)
	else
		SystemClk = 16000000;       // Default fallback

	// 2. Compute AHB prescaler
	temp = ((RCC->CFGR >> 4) & 0xF);
	if (temp < 8)
		ahbp = 1;
	else
		ahbp = AHB_PreScaler[temp - 8];

	// 3. Compute APB1 prescaler
	temp = ((RCC->CFGR >> 10) & 0x7);
	if (temp < 4)
		apb1p = 1;
	else
		apb1p = APB_PreScaler[temp - 4];

	// 4. Final PCLK1 frequency
	pclk1 = (SystemClk / ahbp) / apb1p;
	return pclk1;
}

void I2C_Init(I2C_Handle_t *pI2CHandle) {
	uint32_t temp = 0;
	uint16_t ccr_value = 0;

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	/******************** 1. Configure ACK Control ********************/
	/*
	 * Bit 10: ACK (Acknowledge enable)
	 * 0: No acknowledge returned
	 * 1: Acknowledge returned after a byte is received (if matched)
	 *
	 * - Used only in master receiver mode or slave mode
	 */
	temp |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->I2C_CR1 |= temp;

	/******************** 2. Configure Peripheral Clock Frequency in CR2 ********************/
	/*
	 * Bits [5:0] FREQ[5:0]: Peripheral clock frequency (in MHz)
	 * This tells I2C peripheral what is the APB1 frequency.
	 * For example, if PCLK1 = 42 MHz → FREQ = 42
	 */
	temp = 0;
	temp = RCC_GetPCLK1Value() / 1000000U;     // Convert to MHz
	pI2CHandle->pI2Cx->I2C_CR2 |= (temp & 0x3F);  // Only lower 6 bits valid

	/******************** 3. Configure Own Device Address (OAR1) ********************/
	/*
	 * Bits [7:1]: Interface address
	 * Bit 14: Should always be kept at 1 (per reference manual)
	 *
	 * Example: If device address = 0x52 → write 0xA4 (shifted left by 1)
	 */
	temp = 0;
	temp = (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1); // 7-bit address shifted
	temp |= (1 << 14);                                       // Bit 14 must be 1
	pI2CHandle->pI2Cx->I2C_OAR1 |= temp;

	/******************** 4. Configure CCR (Clock Control Register) ********************/
	/*
	 * CCR determines the SCL clock speed.
	 * Formula depends on whether it’s Standard Mode (≤100kHz)
	 * or Fast Mode (>100kHz)
	 */
	temp = 0;

	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		/************** Standard Mode (≤100kHz) **************/
		/*
		 * Formula: CCR = Fpclk1 / (2 * Fscl)
		 * Fpclk1 = Peripheral clock (PCLK1)
		 * Fscl = Desired SCL frequency (e.g., 100kHz)
		 */
		ccr_value = RCC_GetPCLK1Value()
				/ (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		temp |= (ccr_value & 0x0FFF); // CCR[11:0]
	} else {
		/************** Fast Mode (>100kHz) **************/
		/*
		 * Set F/S bit = 1 → Fast mode
		 * Choose Duty cycle (Tlow/Thigh ratio):
		 *   DUTY=0 → 2
		 *   DUTY=1 → 16/9
		 */
		temp |= (1 << I2C_CCR_FS);  // Fast mode

		temp |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			// DUTY=0 → CCR = Fpclk1 / (3 * Fscl)
			ccr_value = RCC_GetPCLK1Value()
					/ (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		} else {
			// DUTY=1 → CCR = Fpclk1 / (25 * Fscl)
			ccr_value = RCC_GetPCLK1Value()
					/ (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}

		temp |= (ccr_value & 0x0FFF); // Write lower 12 bits
	}

	// Write final CCR value to register
	pI2CHandle->pI2Cx->I2C_CCR |= temp;

	/******************** 5. (Optional) Configure TRISE ********************/
	/*
	 * TRISE defines the maximum rise time of SCL signal.
	 * Formula (from datasheet):
	 *   For Standard Mode: TRISE = Freq + 1
	 *   For Fast Mode: TRISE = (Freq * 300ns) + 1
	 * This ensures correct SCL timing shape.
	 */
	uint32_t freq_mhz = RCC_GetPCLK1Value() / 1000000U;

	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		pI2CHandle->pI2Cx->I2C_TRISE |= ((freq_mhz + 1) & 0X3F);
	} else {
		pI2CHandle->pI2Cx->I2C_TRISE |=
				((((freq_mhz * 300) / 1000U) + 1) & 0X3F);
	}
}

/**
 * @brief  Sends data from Master to Slave over I2C
 * @param  pI2CHandle : Pointer to I2C handle structure
 * @param  pTxBuffer  : Pointer to transmit buffer
 * @param  len        : Number of bytes to send
 * @param  SlaveAddr  : 7-bit slave address
 * @param  Sr         : Repeated start enable/disable (0 = STOP after transfer, 1 = Repeated START)
 * @retval None
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t len, uint8_t SlaveAddr, uint8_t Sr) {    //Sr= "REPEATED START"
	// 1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Wait until START condition is generated (SB flag = 1)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
		;

	// 3. Send the slave address with write bit (R/W = 0)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, write);

	// 4. Wait until address phase is completed (ADDR flag = 1)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))
		;

	// 5. Clear ADDR flag by reading SR1 followed by SR2
	I2C_ClearADDRFlag(pI2CHandle);

	// 6. Send data bytes until all bytes are transmitted
	while (len > 0) {
		// Wait until TXE (Transmit Data Register Empty) is set
		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
			;

		// Write data to data register
		pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;

		pTxBuffer++;
		len--;
	}

	// 7. Wait for TXE = 1 and BTF = 1 before generating STOP
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
		;
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)) {
		;
	}

	// 8. Generate STOP condition if repeated start is not requested
	if (Sr == DISABLE) {
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}

}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
	// 1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Wait until START condition is generated (SB flag = 1)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))
		;

	// 3. Send the slave address with write bit (R/W = 1)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, read);

	// 4. Wait until address phase is completed (ADDR flag = 1)
	while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	if (Len == 1) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

		// 5. Clear ADDR flag by reading SR1 followed by SR2
		I2C_ClearADDRFlag(pI2CHandle);

		while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
			;
		if (Sr == DISABLE) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

	} else {
		I2C_ClearADDRFlag(pI2CHandle);

		for (uint8_t i = Len; i; i--) {
			while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
				;

			if (i == 2) {
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				if (Sr == DISABLE) {
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}
			*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

			pRxBuffer++;

		}

	}
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

uint8_t I2C_MasterSendData_INTR(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);

		//Implement code to Generate START Condition
						I2C_GenerateStartCondition(pI2CHandle->pI2Cx);



	}

	return busystate;
}

uint8_t  I2C_MasterReceiveData_INTR(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;


		//Implement code to Generate START Condition
				I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);



	}

	return busystate;
}

void I2C_SlaveSendData(I2C_Regdef_t *pI2Cx, uint8_t data)
{
	pI2Cx->I2C_DR=data;
}
uint8_t I2C_SlaveReceiveData(I2C_Regdef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->I2C_DR;
}


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {
	uint32_t temp1, temp2, temp3;

	// Interrupt enable bits
	temp1 = (pI2CHandle->pI2Cx->I2C_CR2) & (1 << I2C_CR2_ITEVTEN);
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & (1 << I2C_CR2_ITBUFEN);

	// --------------------------- MASTER MODE --------------------------- //
	// 1. Handle Start Bit (SB) event: generated when master sends START

	temp3 = (pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_SB));
	if ((temp1) && temp3) {
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, write);
		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, read);

		}
	}

	// 2. Handle Address (ADDR) event: Address phase complete
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ADDR);
	if ((temp1) && temp3) {
		I2C_ClearADDRFlag(pI2CHandle);
	}

	// 3. Handle Byte Transfer Finished (BTF)
	temp3 = (pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BTF));
	if ((temp1) && temp3) {
		// Transmission complete and no more data left
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {

			if ((pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TXE))) {

				if (pI2CHandle->TxLen == 0) {
					if (pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					I2C_CloseSendData(pI2CHandle);
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}

			}

		}
	}

	// 4. Handle TXE (Transmit buffer empty)
	temp3 = (pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TXE));
	if ((temp1) && (temp2) && temp3) {

		if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// In master mode
			I2C_MasterHandling_TX_intr(pI2CHandle);
			}
		}
		else
		{
			if(pI2CHandle->pI2Cx->I2C_SR2&(1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ );
			}
		}

	}

	// 5. Handle RXNE (Receive buffer not empty)
	temp3 = (pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RXNE));
	if ((temp1) && (temp2) && temp3)
	{

		if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{
		  if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		  {
			// In master mode
			  I2C_MasterHandling_RX_intr(pI2CHandle);

		  }
	    }
		else
		{
			if(!(pI2CHandle->pI2Cx->I2C_SR2&(1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV );
			}

		}
	}

	// --------------------------- SLAVE MODE --------------------------- //
	// 6. Handle STOP detection (STOPF): only valid in slave mode
	temp3 = (pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_STOPF));
	if ((temp1) && temp3) {

		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;

		// Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t temp1, temp2;

    // Check if Error Interrupt is enabled
    temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & (1 << I2C_CR2_ITERREN);

    /*********************** Check for Bus error ************************************/
    temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & (1 << I2C_SR1_BERR);
    if (temp1 && temp2)
    {
        // Bus error: misplaced Start/Stop detected
        // Clear the Bus Error flag by writing 0
        pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_BERR);

        // Notify application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }

    /*********************** Check for Arbitration Lost error ************************************/
    temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & (1 << I2C_SR1_ARLO);
    if (temp1 && temp2)
    {
        // Arbitration lost (multi-master situation)
        // Clear the Arbitration Lost flag
        pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_ARLO);

        // Notify application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
    }

    /*********************** Check for ACK Failure error ************************************/
    temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & (1 << I2C_SR1_AF);
    if (temp1 && temp2)
    {
        // ACK failure (when master sends data but slave doesn’t acknowledge)
        // Clear the ACK failure flag by writing 0
        pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_AF);

        // Notify application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
    }

    /*********************** Check for Overrun/Underrun error ************************************/
    temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & (1 << I2C_SR1_OVR);
    if (temp1 && temp2)
    {
        // Overrun/Underrun: when DR not read/written in time
        // Clear the flag
        pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_OVR);

        // Notify application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
    }

    /*********************** Check for Timeout error ************************************/
    temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & (1 << I2C_SR1_TIMEOUT);
    if (temp1 && temp2)
    {
        // Timeout: SCL held low too long (in SMBus mode)
        // Clear the flag
        pI2CHandle->pI2Cx->I2C_SR1 &= ~(1 << I2C_SR1_TIMEOUT);

        // Notify application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
    }
}


void I2C_MasterHandling_TX_intr(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->TxLen > 0) {
						pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);
						pI2CHandle->TxLen--;
						pI2CHandle->pTxBuffer++;
					}

}

void I2C_MasterHandling_RX_intr(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->RxSize == 1)
	{
						*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->I2C_DR;
						pI2CHandle->RxLen--;
	}
	else {
						if (pI2CHandle->RxLen == 2) {
							I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
						}
						*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->I2C_DR;
						pI2CHandle->RxLen--;
						pI2CHandle->pRxBuffer++;

					}
					if (pI2CHandle->RxLen == 0)
					{
						if (pI2CHandle->Sr == I2C_DISABLE_SR)
							I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

						I2C_CloseReceiveData(pI2CHandle);
						I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
					}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    // 1. Disable buffer interrupt (ITBUFEN)
    pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    // 2. Disable event interrupt (ITEVTEN)
    pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    // 3. Disable error interrupt (ITERREN)
    pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITERREN);

    // 4. Reset handle parameters
    pI2CHandle->pRxBuffer = 0;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;
    pI2CHandle->TxRxState = I2C_READY;
    if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
    		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
    	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    // 1. Disable buffer interrupt (ITBUFEN)
    pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    // 2. Disable event interrupt (ITEVTEN)
    pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    // 3. Disable error interrupt (ITERREN)
    pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITERREN);

    // 4. Reset handle parameters
    pI2CHandle->pTxBuffer = 0;
    pI2CHandle->TxLen = 0;
    pI2CHandle->TxRxState = I2C_READY;
    if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
    		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
    	}

}


/*
 * stm32f407_spi_driver.c
 *
 *  Created on: Oct 14, 2025
 *      Author: karnati.ganesh
 */
#include "stm32f407_spi_header.h"
#include "stm32f407_xx.h"

//these are local to this file shoould not use outside
static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_rxe_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_overrun_interrupt_handler(SPI_Handle_t *pSPIHandle);
/**
 * @brief  Enables or disables the peripheral clock for a given SPI peripheral.
 * @param  pSPIx: Pointer to SPI peripheral (SPI1, SPI2, SPI3)
 * @param  EnOrDi: ENABLE or DISABLE macro
 * @retval None
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)   // If user wants to enable the SPI peripheral clock
    {
        // Check which SPI peripheral is selected and enable its clock
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_EN();   // Enable clock for SPI1
        }
        else if(pSPIx == SPI2)
        {
            SPI2_PCLK_EN();   // Enable clock for SPI2
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_EN();   // Enable clock for SPI3
        }
    }
    else  // If user wants to disable the SPI peripheral clock
    {
        // Check which SPI peripheral is selected and disable its clock
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_DI();   // Disable clock for SPI1
        }
        else if(pSPIx == SPI2)
        {
            SPI2_PCLK_DI();   // Disable clock for SPI2
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_DI();   // Disable clock for SPI3
        }
    }
}


void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    uint32_t tempreg = 0;
    SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);
    // 1. Configure Device Mode: Master / Slave
    if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_DEVICE_MASTER)
    {
        tempreg |= (1 << SPI_CR1_MSTR);
    }

    // 2. Configure Bus Configuration
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_FD)
    {
        // Full duplex: default, nothing to change
        // Optional: ensure BIDIMODE = 0
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_HD)
    {
        // Half duplex: BIDIMODE = 1
        tempreg |= (1 << SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RXONLY)
    {
        // Simplex RX only: BIDIMODE = 0, RXONLY = 1
        tempreg &= ~(1 << SPI_CR1_BIDIMODE);
        tempreg |= (1 << SPI_CR1_RXONLY);
    }

    // 3. Configure SCLK Speed (BR2:BR0)
    tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR0);

    // 4. Configure DFF: 8-bit or 16-bit
    if(pSPIHandle->SPIConfig.SPI_DFF == SPI_DFF_16BITS)
    {
        tempreg |= (1 << SPI_CR1_DFF);
    }
    else
    {
        tempreg &= ~(1 << SPI_CR1_DFF);
    }

    // 5. Configure CPOL
    if(pSPIHandle->SPIConfig.SPI_CPOL == SPI_CPOL_HIGH)
    {
        tempreg |= (1 << SPI_CR1_CPOL);
    }
    else
    {
        tempreg &= ~(1 << SPI_CR1_CPOL);
    }

    // 6. Configure CPHA
    if(pSPIHandle->SPIConfig.SPI_CPHA == SPI_CPHA_SECOND)
    {
        tempreg |= (1 << SPI_CR1_CPHA);
    }
    else
    {
        tempreg &= ~(1 << SPI_CR1_CPHA);
    }

    // 7. Configure Software Slave Management (SSM)
    if(pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_EN)
    {
        tempreg |= (1 << SPI_CR1_SSM);
        tempreg |= (1 << SPI_CR1_SSI); // SSI must be set when SSM enabled
    }
    else
    {
        tempreg &= ~(1 << SPI_CR1_SSM);
    }

    // 8. Write configuration to SPI_CR1
    pSPIHandle->pSPIx->SPI_CR1 = tempreg;
}


/**
 * @brief  Resets the given SPI peripheral registers to default reset state.
 * @param  pSPIx: Pointer to SPI peripheral (SPI1, SPI2, SPI3)
 * @retval None
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    // Check which SPI peripheral is selected and perform reset
    if(pSPIx == SPI1)
    {
        SPI1_REG_RESET();    // Reset SPI1 peripheral registers
    }
    else if(pSPIx == SPI2)
    {
        SPI2_REG_RESET();    // Reset SPI2 peripheral registers
    }
    else if(pSPIx == SPI3)
    {
        SPI3_REG_RESET();    // Reset SPI3 peripheral registers
    }
}

/**
 * @brief  Sends data over SPI (blocking mode)
 * @param  pSPIx: Pointer to SPI peripheral (SPI1, SPI2, SPI3)
 * @param  data: Pointer to data buffer to transmit
 * @param  data_len: Number of bytes to transmit
 * @retval None
 */
void SPI_DATA_TX(SPI_RegDef_t *pSPIx, uint8_t* data, uint32_t data_len)
{
    // Loop until all data is transmitted
    while(data_len)
    {
        // 1. Wait until TX buffer is empty (TXE = 1)
        while(Return_flag(pSPIx, SPI_SR_TXE) == CLEAR);

        // 2. Check Data Frame Format (DFF) - 16-bit or 8-bit
        if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))   // 16-bit mode
        {
            // Combine two 8-bit bytes into one 16-bit word
            uint16_t temp = (*data << 8);   // First byte → MSB
            data++;

            if(data_len > 1)
                temp |= (*data);            // Second byte → LSB
            else
                temp |= 0;                  // Pad 0 if odd length

            // 3. Write 16-bit word to SPI data register
            pSPIx->SPI_DR = temp;
            data++;
            data_len -= 2;                  // Reduce data length by 2
        }
        else   // 8-bit mode
        {
            // Write single byte to SPI data register
            pSPIx->SPI_DR = *data;
            data_len--;
            data++;// Reduce data length by 1
        }
    }

    // 4. Wait until SPI is not busy (BSY = 0) after last transmission
    //while(Return_flag(pSPIx, SPI_SR_BSY) == SET);
}
/*********************************************************************
 * @fn      		  - SPI_DATA_RX
 *
 * @brief             - Receive data from SPI peripheral (blocking mode)
 *
 * @param[in]         - pSPIx : Pointer to SPI peripheral base address (e.g., SPI1, SPI2, etc.)
 * @param[out]        - data  : Pointer to receive buffer
 * @param[in]         - data_len : Number of bytes to receive
 *
 * @return            - None
 *
 * @Note              - Handles both 8-bit and 16-bit data frame formats.
 **********************************************************************/
void SPI_DATA_RX(SPI_RegDef_t *pSPIx, char *data, uint32_t data_len)
{
    // 1️⃣ Check Data Frame Format (DFF) bit in SPI_CR1
    //    DFF = 0 → 8-bit data frame
    //    DFF = 1 → 16-bit data frame


    while (data_len > 0)
    {
        // 2️⃣ Wait until RXNE (Receive buffer not empty) flag is set
    	  while(Return_flag(pSPIx, SPI_SR_RXNE) == CLEAR);

        if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
        {
            // ---------- 16-Bit Data Frame ----------
            // Read 2 bytes (16 bits) from the data register
            *((uint16_t*)data) = (uint16_t)pSPIx->SPI_DR;

            // Move buffer pointer ahead by 2 bytes
            data += 2;

            // Reduce data length by 2
            data_len -= 2;
        }
        else
        {
            // ---------- 8-Bit Data Frame ----------
            // Read 1 byte (8 bits) from the data register
            *data = (uint8_t)pSPIx->SPI_DR;

            // Move buffer pointer ahead by 1 byte
            data++;

            // Reduce data length by 1
            data_len--;
        }
    }

    // 3️⃣ Wait until SPI is not busy before exiting (BSY = 0)
    while (pSPIx->SPI_SR & (1 << SPI_SR_BSY));
}

/**
 * @brief  Checks the status of a specific SPI_SR bit
 * @param  pSPIx: Pointer to SPI peripheral
 * @param  Bit_no: Bit position in SPI_SR register
 * @retval SET if bit is 1, CLEAR if bit is 0
 */
uint8_t Return_flag(SPI_RegDef_t* pSPIx, uint8_t Bit_no)
{
    if(pSPIx->SPI_SR & (1 << Bit_no))
        return SET;   // Bit is set
    return CLEAR;     // Bit is cleared
}
/**
 * @brief  Enables or disables the SPI peripheral
 * @param  pSPIx: Pointer to SPI peripheral (SPI1, SPI2, SPI3)
 * @param  EnOrDi: ENABLE to enable, DISABLE to disable
 * @retval None
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        // Set the SPE bit (CR1, bit 6) to enable SPI
        pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
    }
    else
    {
        // Clear the SPE bit to disable SPI
        pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

void SPI_SSI_CONFIG(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        // Set the SPE bit (CR1, bit 6) to enable SPI
        pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
    }
    else
    {
        // Clear the SPE bit to disable SPI
        pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

void SPI_SSOE_CONFIG(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        // Set the SPE bit (CR1, bit 6) to enable SPI
        pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);
    }
    else
    {
        // Clear the SPE bit to disable SPI
        pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}

/*
 * Function: SPI_IRQInterruptConfig
 * --------------------------------
 * Enables or disables a given IRQ number in the NVIC (Nested Vector Interrupt Controller).
 *
 * Parameters:
 *   IRQNumber - The interrupt request (IRQ) number for the peripheral.
 *   EnOrDi    - ENABLE or DISABLE macro to control whether the interrupt should be enabled or disabled.
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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


/*
 * Function: SPI_IRQPriorityConfig
 * -------------------------------
 * Sets the priority of a specific IRQ number in the NVIC priority register.
 *
 * Parameters:
 *   IRQNumber   - The interrupt request (IRQ) number for the peripheral.
 *   IRQPriority - The priority level (lower number → higher priority).
 *
 * Notes:
 *   - Each NVIC priority register (IPR) is 32 bits wide and holds priority for 4 IRQs.
 *   - Each IRQ priority is stored in 8 bits, but not all bits are implemented in hardware.
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Each IPR register holds 4 priority fields (1 byte each)
	uint8_t iprx = IRQNumber / 4;         // Find which IPR register this IRQ belongs to
	uint8_t iprx_section = IRQNumber % 4; // Find the section (0–3) inside that register

	// STM32 implements only upper bits for priority (e.g., 4 bits out of 8)
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	// Write the priority value shifted into the correct field
	// (NVIC_PR_BASEADDR points to the start of NVIC priority registers)
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*
 * Function: SPI_DATA_TX_TNTR
 * --------------------------
 * Initiates SPI data transmission using interrupt mode.
 *
 * This function sets up the SPI handle with the transmit buffer, data length,
 * and enables the TXE (Transmit buffer empty) interrupt so that the actual
 * data transmission is handled inside the SPI interrupt handler.
 *
 * Parameters:
 *   pHandler  - Pointer to the SPI handle structure (contains SPI registers & config info)
 *   data      - Pointer to the data buffer that needs to be transmitted
 *   data_len  - Number of bytes to send
 *
 * Returns:
 *   SPI_TX state:
 *      - SPI_TX_BUSY  → Transmission in progress
 *      - SPI_READY    → Transmission not started or completed
 */

uint8_t SPI_DATA_TX_INTR(SPI_Handle_t *pHandler, uint8_t *data, uint32_t data_len)
{
    // Check if SPI is NOT already busy in transmission
	uint8_t state=pHandler->TX_state;
    if (state != SPI_TX_BUSY)
    {
        // 1. Save the TX buffer address (source of data to be sent)
        pHandler->TX_buffer_addrs = data;      // NOTE: This seems like a typo — should ideally be TX_buffer_addrs

        // 2. Store total data length to send
        pHandler->TX_len = data_len;

        // 3. Mark the SPI as busy in transmission to prevent re-entry
        pHandler->TX_state = SPI_TX_BUSY;

        // 4. Enable TXE interrupt (Transmit buffer empty interrupt)
        //    This interrupt triggers when SPI's TXE flag is set,
        //    meaning the data register (DR) is empty and ready for next byte.
       // SPI_PeripheralControl(pHandler->pSPIx, DISABLE);
        pHandler->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);
       // SPI_PeripheralControl(pHandler->pSPIx, ENABLE);


    }

    // Return the current transmission state (BUSY or READY)
    return state;
}

/*
 * Function: SPI_DATA_TX_TNTR
 * --------------------------
 * Initiates SPI data transmission using interrupt mode.
 *
 * This function sets up the SPI handle with the transmit buffer, data length,
 * and enables the TXE (Transmit buffer empty) interrupt so that the actual
 * data transmission is handled inside the SPI interrupt handler.
 *
 * Parameters:
 *   pHandler  - Pointer to the SPI handle structure (contains SPI registers & config info)
 *   data      - Pointer to the data buffer that needs to be transmitted
 *   data_len  - Number of bytes to send
 *
 * Returns:
 *   SPI_TX state:
 *      - SPI_TX_BUSY  → Transmission in progress
 *      - SPI_READY    → Transmission not started or completed
 */

uint8_t SPI_DATA_RX_INTR(SPI_Handle_t *pHandler, uint8_t *data, uint32_t data_len)
{
    // Check if SPI is NOT already busy in transmission
	uint8_t state=pHandler->RX_state;
    if ( state!= SPI_RX_BUSY)
    {
        // 1. Save the TX buffer address (source of data to be sent)
        pHandler->RX_buffer_addrs = data;      // NOTE: This seems like a typo — should ideally be TX_buffer_addrs

        // 2. Store total data length to send
        pHandler->RX_len = data_len;

        // 3. Mark the SPI as busy in transmission to prevent re-entry
        pHandler->RX_state = SPI_RX_BUSY;

        // 4. Enable TXE interrupt (Transmit buffer empty interrupt)
        //    This interrupt triggers when SPI's TXE flag is set,
        //    meaning the data register (DR) is empty and ready for next byte.
        pHandler->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);
    }

    // Return the current transmission state (BUSY or READY)
    return state;
}
/*
 * Function: SPI_IRQHandling
 * --------------------------
 * Handles all SPI-related interrupt events.
 *
 * This function checks which interrupt flag (TXE, RXNE, or OVR) caused the interrupt,
 * and then calls the corresponding handler function.
 *
 * Parameters:
 *   pHandler - Pointer to the SPI handle structure (contains SPI registers, buffers, states, etc.)
 *
 * Interrupt Sources:
 *   1. TXE  (Transmit Buffer Empty)   → SPI ready to send next byte
 *   2. RXNE (Receive Buffer Not Empty) → SPI has received a byte
 *   3. OVR  (Overrun Error)           → Data lost because previous data wasn’t read in time
 */

void SPI_IRQHandling(SPI_Handle_t *pHandler)
{
    uint8_t temp1, temp2;

    /*************** 1. Check for TXE interrupt ***************/
    temp1 = pHandler->pSPIx->SPI_SR & (1 << SPI_SR_TXE);       // Read TXE flag (set when TX buffer is empty)
    temp2 = pHandler->pSPIx->SPI_CR2 & (1 << SPI_CR2_TXEIE);   // Check if TXE interrupt is enabled(if it is enable then only interrupt will generate )
     // printf("%d %d\n",temp1,temp2);
    if (temp1 && temp2)    // If both flag and enable bit are set w
    {
        // Call the transmit buffer empty interrupt handler
        // This function will send the next byte (from TX buffer to SPI_DR)
        spi_txe_interrupt_handler(pHandler);
    }

    /*************** 2. Check for RXNE interrupt ***************/
    temp1 = pHandler->pSPIx->SPI_SR & (1 << SPI_SR_RXNE);      // Read RXNE flag (set when data is received)
    temp2 = pHandler->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE);  // Check if RXNE interrupt is enabled
    //printf("%d %d\n",temp1,temp2);
    if (temp1 && temp2)
    {
        // Call the receive buffer not empty interrupt handler
        // This function will read received data from SPI_DR into RX buffer
        spi_rxe_interrupt_handler(pHandler);
    }

    /*************** 3. Check for Overrun Error (OVR) interrupt ***************/
    temp1 = pHandler->pSPIx->SPI_SR & (1 << SPI_SR_OVR);       // Read OVR flag (set when data is lost)
    temp2 = pHandler->pSPIx->SPI_CR2 & (1 << SPI_CR2_ERRIE);   // Check if error interrupt is enabled

    if (temp1 && temp2)
    {
        // Call the overrun error interrupt handler
        // This function clears the OVR flag and handles the error condition
        spi_overrun_interrupt_handler(pHandler);
    }
}

/**
 * @brief  Handle TXE (Transmit buffer empty) interrupt.
 * @param  pHandle: Pointer to SPI handle structure.
 * @note   Sends next byte/word from TX buffer when TXE flag is set.
 */
static void spi_txe_interrupt_handler(SPI_Handle_t *pHandle)
{
    // Check Data Frame Format (DFF) to determine 16-bit or 8-bit transmission
    if (pHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))   // 16-bit mode
    {
        // Prepare a 16-bit word to send
        uint16_t temp = (*(pHandle->TX_buffer_addrs) << 8); // First byte → MSB
        pHandle->TX_buffer_addrs++;

        // If more data remains, include second byte as LSB
        if (pHandle->TX_len > 1)
            temp |= (*(pHandle->TX_buffer_addrs));
        else
            temp |= 0;  // Pad with 0 if remaining data length is odd

        // Write 16-bit word to SPI data register (DR)
        pHandle->pSPIx->SPI_DR = temp;

        // Move buffer pointer and update remaining data length
        pHandle->TX_buffer_addrs++;
        pHandle->TX_len -= 2;
    }
    else // 8-bit mode
    {
        // Write a single byte to SPI data register
        pHandle->pSPIx->SPI_DR = *(pHandle->TX_buffer_addrs);

        // Move buffer pointer and decrease remaining length
        pHandle->TX_buffer_addrs++;
        pHandle->TX_len--;
    }

    // If transmission is complete
    if (pHandle->TX_len<=0)
    {
        // Disable TXE interrupt to prevent further interrupts
        pHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);

        // Reset buffer pointer, length, and state
        pHandle->TX_buffer_addrs = 0;
        pHandle->TX_len = 0;
        pHandle->TX_state = SPI_READY;

        // Notify application about transmission completion
        SPI_ApplicationEventCallback(pHandle, TX_EVENT_COMPLETED);
    }
}

/**
 * @brief  Handle RXNE (Receive buffer not empty) interrupt.
 * @param  pHandle: Pointer to SPI handle structure.
 * @note   Reads incoming data from SPI data register into RX buffer.
 */
static void spi_rxe_interrupt_handler(SPI_Handle_t *pHandle)
{
    // Check Data Frame Format (DFF) to determine 16-bit or 8-bit reception
    if (pHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))   // 16-bit mode
    {
        // Read 16-bit word from data register
        *((uint16_t*)pHandle->RX_buffer_addrs) = (uint16_t)pHandle->pSPIx->SPI_DR;

        // Move buffer pointer and update remaining length
        pHandle->RX_buffer_addrs += 2;
        pHandle->RX_len -= 2;
    }
    else // 8-bit mode
    {
        // Read a single byte from data register
        *(pHandle->RX_buffer_addrs) = (uint8_t)pHandle->pSPIx->SPI_DR;

        // Move buffer pointer and decrease remaining length
        pHandle->RX_buffer_addrs++;
        pHandle->RX_len--;
    }

    // If reception is complete
    if (pHandle->RX_len<=0)
    {
        // Disable RXNE interrupt to prevent further interrupts
        pHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);

        // Reset buffer pointer, length, and state
        pHandle->RX_buffer_addrs = 0;
        pHandle->RX_len = 0;
        pHandle->RX_state = SPI_READY;

        // Notify application about reception completion
        SPI_ApplicationEventCallback(pHandle, RX_EVENT_COMPLETED);
    }
}

/**
 * @brief  Handle Overrun (OVR) error interrupt.
 * @param  pHandle: Pointer to SPI handle structure.
 * @note   Clears OVR flag and notifies application if TX is not busy.
 */
static void spi_overrun_interrupt_handler(SPI_Handle_t *pHandle)
{
    // Only clear OVR flag if SPI is not transmitting (TX not busy)
    if (pHandle->TX_state != SPI_TX_BUSY)
    {
        // Read DR and SR to clear OVR flag (required sequence)
        uint8_t temp = pHandle->pSPIx->SPI_DR;
        temp = pHandle->pSPIx->SPI_SR;
        (void)temp;  // Suppress unused variable warning
    }

    // Notify application about overrun error
    SPI_ApplicationEventCallback(pHandle, OVR_EVENT_COMPLETED);
}

/**
 * @brief  SPI application callback function.
 * @param  pHandler: Pointer to SPI handle structure.
 * @param  flag: Event type (TX/RX/OVR)
 * @note   Implement this function in application code to handle SPI events.
 */
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pHandler, uint8_t flag)
{
    // Application can override this function to handle events
    // For example: Toggle LED on TX/RX complete or handle errors
}

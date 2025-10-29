/*
 * stm32f407xx.h
 *
 *  Created on: Sep 22, 2025
 *      Author: venkatesaperumal.r
 */

#ifndef STM32F407XX_H
#define STM32F407XX_H
#include<stdint.h>
#include<string.h>
#define _VO    volatile

/*
 *  ARM CORTEX Mx NVIC ISERx Register Address
 * */

#define NVIC_ISER0     						(( volatile uint32_t*)0xE000E100)
#define NVIC_ISER1     						(( volatile uint32_t*)0xE000E104)
#define NVIC_ISER2     						(( volatile uint32_t*)0xE000E108)
#define NVIC_ISER3     						(( volatile uint32_t*)0xE000E10C)
#define NVIC_ISER4     						(( volatile uint32_t*)0xE000E110)
#define NVIC_ISER5     						(( volatile uint32_t*)0xE000E114)

#define NVIC_ICER0     						(( volatile uint32_t*)0XE000E180)
#define NVIC_ICER1     						(( volatile uint32_t*)0xE000E184)
#define NVIC_ICER2     						(( volatile uint32_t*)0xE000E188)
#define NVIC_ICER3     						(( volatile uint32_t*)0xE000E18C)
#define NVIC_ICER4     						(( volatile uint32_t*)0xE000E190)
#define NVIC_ICER5     						(( volatile uint32_t*)0xE000E194)


#define NVIC_PR_BASEADDR					(( volatile uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED               4



#define FLASH_BASEADDR						0x08000000U
#define SRAM1_BASEADDR						0x20000000U
#define SRAM2_BASEADDR						0x2001C000U
#define ROM_BASEADDR						0x1FFF0000U
#define SRAM								SRAM1_BASEADDR


#define PERIPH_BASEADDR						0x40000000U
#define APB1_BASEADDR				     	PERIPH_BASEADDR
#define APB2_BASEADDR				    	0x40010000U
#define AHB1_BASEADDR					    0x40020000U
#define AHB2_BASEADDR					    0x50000000U


// GPIO Base Addresses for STM32F407VG (AHB1 Bus)
#define GPIOA_BASEADDR    0x40020000U
#define GPIOB_BASEADDR    0x40020400U
#define GPIOC_BASEADDR    0x40020800U
#define GPIOD_BASEADDR    0x40020C00U
#define GPIOE_BASEADDR    0x40021000U
#define GPIOF_BASEADDR    0x40021400U
#define GPIOG_BASEADDR    0x40021800U
#define GPIOH_BASEADDR    0x40021C00U
#define GPIOI_BASEADDR    0x40022000U



// APB1 Peripheral Base Addresses
#define I2C1_BASEADDR     0x40005400U
#define I2C2_BASEADDR     0x40005800U
#define I2C3_BASEADDR     0x40005C00U
#define SPI2_BASEADDR     0x40003800U
#define SPI3_BASEADDR     0x40003C00U
#define USART2_BASEADDR   0x40004400U
#define USART3_BASEADDR   0x40004800U
#define UART4_BASEADDR    0x40004C00U
#define UART5_BASEADDR    0x40005000U


// APB2 Peripheral Base Addresses
#define SPI1_BASEADDR     0x40013000U
#define SPI4_BASEADDR     0x40013400U
#define SPI5_BASEADDR     0x40015000U
#define SPI6_BASEADDR     0x40015400U
#define USART1_BASEADDR   0x40011000U
#define USART6_BASEADDR   0x40011400U
#define EXTI_BASEADDR     0x40013C00U
#define SYSCFG_BASEADDR   0x40013800U

// RCC Base Address (AHB1)
#define RCC_BASEADDR      0x40023800U

typedef struct
{
	volatile uint32_t MODER;     // GPIO port mode register (2 bits per pin)
	                              // 00: Input mode, 01: General purpose output mode
	                              // 10: Alternate function mode, 11: Analog mode

	volatile uint32_t OTYPER;    // GPIO port output type register (1 bit per pin)
	                              // 0: Push-pull, 1: Open-drain

	volatile uint32_t OSPEEDR;   // GPIO port output speed register (2 bits per pin)
	                              // 00: Low speed, 01: Medium speed
	                              // 10: High speed, 11: Very high speed

	volatile uint32_t PUPDR;     // GPIO port pull-up/pull-down register (2 bits per pin)
	                              // 00: No pull-up/pull-down
	                              // 01: Pull-up, 10: Pull-down

	volatile uint32_t IDR;       // GPIO port input data register (read-only)
	                              // Contains input values of all pins (1 bit per pin)

	volatile uint32_t ODR;       // GPIO port output data register (read/write)
	                              // Controls output level of each pin (1 bit per pin)

	volatile uint32_t BSRR;      // GPIO port bit set/reset register
	                              // Lower 16 bits: set pin (1 = set)
	                              // Upper 16 bits: reset pin (1 = reset)

	volatile uint32_t LCKR;      // GPIO port configuration lock register
	                              // Locks the configuration of the port pins until next reset

	volatile uint32_t AFR[2];    // GPIO alternate function registers
	                              // AFR[0]: Pins 0–7 (low)
	                              // AFR[1]: Pins 8–15 (high)
} GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t RESERVED4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t RESERVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
}RCC_RegDef_t;


//EXTI REGISTER
typedef struct
{
  _VO uint32_t IMR;
  _VO uint32_t EMR;
  _VO uint32_t RTSR;
  _VO uint32_t FTSR;
  _VO uint32_t SWIER;
  _VO uint32_t PR;
} EXTI_RegDef_t;
//SYSTEM CONFIGURE REGISTER FOR EXTI LINE CONTROL
typedef struct
{
    volatile uint32_t MEMRMP;     /*!< Memory remap register,             Address offset: 0x00 */
    volatile uint32_t PMC;        /*!< Peripheral mode configuration,    Address offset: 0x04 */
    volatile uint32_t EXTICR[4];  /*!< EXTI configuration registers 1-4, Address offset: 0x08-0x14 */
    uint32_t RESERVED[2];         /*!< Reserved 0x18–0x1C */
    volatile uint32_t CMPCR;      /*!< Compensation cell control register, Address offset: 0x20 */
} SYSCFG_Regdef_t;



#define GPIOA          	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB          	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC          	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD          	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE          	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF          	((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG          	((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH          	((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI          	((GPIO_RegDef_t*)GPIOI_BASEADDR)



#define I2C1			((I2C_Regdef_t*)I2C1_BASEADDR)
#define I2C2			((I2C_Regdef_t*)I2C2_BASEADDR)
#define I2C3			((I2C_Regdef_t*)I2C3_BASEADDR)


#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG			((SYSCFG_Regdef_t*)SYSCFG_BASEADDR)


#define GPIOA_PCLK_EN()  (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()  (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()  (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()  (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()  (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()  (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()  (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()  (RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()  (RCC->AHB1ENR |= (1<<8))


#define I2C1_PCLK_EN()  (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()  (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()  (RCC->APB1ENR |= (1<<23))



#define USART1_PCLK_EN() (RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN() (RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1<<5))
#define UART7_PCLK_EN() (RCC->APB1ENR |= (1<<30))
#define UART8_PCLK_EN() (RCC->APB1ENR |= (1<<31))

#define SYSCFG_PCLK_EN()  (RCC->APB2ENR |= (1<<14))


#define GPIOA_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<8))


#define I2C1_PCLK_DI()  (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()  (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()  (RCC->APB1ENR &= ~(1<<23))




#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI() (RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI() (RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1<<5))
#define UART7_PCLK_DI() (RCC->APB1ENR &= ~(1<<30))
#define UART8_PCLK_DI() (RCC->APB1ENR &= ~(1<<31))


#define SYSCFG_PCLK_DI()  (RCC->APB2ENR &= ~(1<<14))

// GPIOx peripherals reset

#define GPIOA_REG_RESET()     do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); } while(0)
#define GPIOB_REG_RESET()     do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); } while(0)
#define GPIOC_REG_RESET()     do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); } while(0)
#define GPIOD_REG_RESET()     do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); } while(0)
#define GPIOE_REG_RESET()     do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); } while(0)
#define GPIOF_REG_RESET()     do{ (RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5)); } while(0)
#define GPIOG_REG_RESET()     do{ (RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6)); } while(0)
#define GPIOH_REG_RESET()     do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7)); } while(0)
#define GPIOI_REG_RESET()     do{ (RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8)); } while(0)





#define GPIO_BASEADDR_TO_CODE(x)  ( (x==GPIOA) ? 0:\
									(x==GPIOB) ? 1:\
									(x==GPIOC) ? 2:\
									(x==GPIOD) ? 3:\
									(x==GPIOE) ? 4:\
									(x==GPIOF) ? 5:\
									(x==GPIOG) ? 6:\
									(x==GPIOH) ? 7:0 )


#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET  	SET
#define GPIO_PIN_RESET  RESET
#define FLAG_SET  		SET
#define FLAG_RESET  	RESET

#define EXTI0_IRQn          6
#define EXTI1_IRQn          7
#define EXTI2_IRQn          8
#define EXTI3_IRQn          9
#define EXTI4_IRQn          10
#define EXTI9_5_IRQn        23
#define EXTI15_10_IRQn      40

#include "stm32f407_drivers.h"
#include "stm32f407_spi_header.h"
#include "stm32f407_i2c.h"
#endif /* INC_STM32F407XX_H_ */

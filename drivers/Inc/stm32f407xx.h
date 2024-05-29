/*
 * stm32f407xx.h
 *
 *  Created on: May 28, 2024
 *      Author: nesh
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define _VO volatile

/*
 * Base addresses of Flash & SRAM
 * */
#define FLASH_BASE_ADDR             0x08000000U                 /* FLASH area base address */
#define SRAM1_BASE_ADDR             0x20000000U                 /* SRAM1 area base address */
#define SRAM2_BASE_ADDR             0x2001C000U                 /* SRAM2 area base address (SRAM1 base address + SRAM1 size) */

#define ROM_BASE_ADDR               0x1FFF0000U                 /* System memory base address*/
#define SRAM                        SRAM1_BASE_ADDR             /* Static RAM base address*/

/*
 * Base addresses of AHBx & APBx
 * */
#define PERIPH_BASE_ADDR            0x40000000U                 /* Peripheral base address, Starts from TIM2 base */

#define APB1_BASE_ADDR              PERIPH_BASE_ADDR            /* APB1 base address, Starts from TIM2 base */
#define APB2_BASE_ADDR              0x40010000U                 /* APB2 base address, Starts from TIM1 base */

#define AHB1_BASE_ADDR              0x40020000U                 /* AHB1 base address, Starts from GPIOA base */
#define AHB2_BASE_ADDR              0x50000000U                 /* AHB2 base address, Starts from USB OTG FS base */
#define AHB3_BASE_ADDR              0xA0000000U                 /* AHB1 base address, Starts from FSMC control register base */

/*
 * Base addresses of peripherals which are hanging on AHB1 bus 
 * */
#define GPIOA_BASE_ADDR			    (AHB1_BASE_ADDR)            /* GPIOA base address */
#define GPIOB_BASE_ADDR			    (AHB1_BASE_ADDR + 0x0400U)  /* GPIOB base address */
#define GPIOC_BASE_ADDR			    (AHB1_BASE_ADDR + 0x0800U)  /* GPIOC base address */
#define GPIOD_BASE_ADDR			    (AHB1_BASE_ADDR + 0x0C00U)  /* GPIOD base address */
#define GPIOE_BASE_ADDR			    (AHB1_BASE_ADDR + 0x1000U)  /* GPIOE base address */
#define GPIOF_BASE_ADDR			    (AHB1_BASE_ADDR + 0x1400U)  /* GPIOF base address */
#define GPIOG_BASE_ADDR 		    (AHB1_BASE_ADDR + 0x1800U)  /* GPIOG base address */
#define GPIOH_BASE_ADDR 		    (AHB1_BASE_ADDR + 0x1C00U)  /* GPIOH base address */
#define GPIOI_BASE_ADDR 		    (AHB1_BASE_ADDR + 0x2000U)  /* GPIOI base address */
#define GPIOJ_BASE_ADDR 		    (AHB1_BASE_ADDR + 0x2400U)  /* GPIOJ base address */
#define GPIOK_BASE_ADDR 		    (AHB1_BASE_ADDR + 0x2800U)  /* GPIOK base address */
#define CRC_BASE_ADDR   		    (AHB1_BASE_ADDR + 0x3000U)  /* CRC base address */
#define RCC_BASE_ADDR   		    (AHB1_BASE_ADDR + 0x3800U)  /* RCC base address */
#define FLASH_INT_REG_BASE_ADDR     (AHB1_BASE_ADDR + 0x3800U)  /* Flash interface register base address */
#define BKPSRAM_BASE_ADDR		    (AHB1_BASE_ADDR + 0x3C00U)  /* Backup SRAM base address */
#define DMA1_BASE_ADDR			    (AHB1_BASE_ADDR + 0x6000U)  /* DMA1 base address */
#define DMA2_BASE_ADDR			    (AHB1_BASE_ADDR + 0x6400U)  /* DMA2 base address */
#define ETHERNET_MAC_BASE_ADDR	    (AHB1_BASE_ADDR + 0x8000U)  /* Ethernet MAC base address */
#define DMA2D_BASE_ADDR			    (AHB1_BASE_ADDR + 0xB000U)  /* DMA2D base address */
#define USB_OTG_HS_BASE_ADDR	    (AHB1_BASE_ADDR + 0x20000U) /* USB OTG HS base address */

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 * */
#define USB_OTG_FS_BASE_ADDR        (AHB2_BASE_ADDR)            /* USB OTG FS base address */
#define DCMI_BASE_ADDR              (AHB2_BASE_ADDR + 0x50000U) /* DCMI base address */
#define CRYP_BASE_ADDR              (AHB2_BASE_ADDR + 0x60000U) /* CRYP base address */
#define HASH_BASE_ADDR              (AHB2_BASE_ADDR + 0x60400U) /* HASH base address */
#define RNG_BASE_ADDR               (AHB2_BASE_ADDR + 0x60800U) /* RNG base address */

/*
 * Base addresses of peripherals which are hanging on AHB3 bus
 * */
#define FSMC_CNTRL_REG_BASE_ADDR    (AHB3_BASE_ADDR)            /* FSMC base address */

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * */
#define TIM2_BASE_ADDR              (APB1_BASE_ADDR)            /* TIM2 base address */
#define TIM3_BASE_ADDR              (APB1_BASE_ADDR + 0x0400U)  /* TIM3 base address */
#define TIM4_BASE_ADDR              (APB1_BASE_ADDR + 0x0800U)  /* TIM4 base address */
#define TIM5_BASE_ADDR              (APB1_BASE_ADDR + 0x0C00U)  /* TIM5 base address */
#define TIM6_BASE_ADDR              (APB1_BASE_ADDR + 0x1000U)  /* TIM6 base address */
#define TIM7_BASE_ADDR              (APB1_BASE_ADDR + 0x1400U)  /* TIM7 base address */
#define TIM12_BASE_ADDR             (APB1_BASE_ADDR + 0x1800U)  /* TIM12 base address */
#define TIM13_BASE_ADDR             (APB1_BASE_ADDR + 0x1C00U)  /* TIM13base address */
#define TIM14_BASE_ADDR             (APB1_BASE_ADDR + 0x2000U)  /* TIM14 base address */
#define RTC_BKP_REG_BASE_ADDR       (APB1_BASE_ADDR + 0x2800U)  /* RTC backup registers base address */
#define WWDG_BASE_ADDR              (APB1_BASE_ADDR + 0x2C00U)  /* WWDG base address */
#define IWDG_BASE_ADDR              (APB1_BASE_ADDR + 0x3000U)  /* IWDG base address */
#define I2S2EXT_BASE_ADDR           (APB1_BASE_ADDR + 0x3400U)  /* I2S2EXT base address */
#define SPI2_BASE_ADDR              (APB1_BASE_ADDR + 0x3800U)  /* SPI2 base address */
#define I2S2_BASE_ADDR              (APB1_BASE_ADDR + 0x3800U)  /* I2S2 base address */
#define SPI3_BASE_ADDR              (APB1_BASE_ADDR + 0x3C00U)  /* SPI3 base address */
#define I2S3_BASE_ADDR              (APB1_BASE_ADDR + 0x3C00U)  /* I2S3 base address */
#define I2S3EXT_BASE_ADDR           (APB1_BASE_ADDR + 0x4000U)  /* I2S3 External base address */
#define USART2_BASE_ADDR            (APB1_BASE_ADDR + 0x4400U)  /* USART2 base address */
#define USART3_BASE_ADDR            (APB1_BASE_ADDR + 0x4800U)  /* USART3 base address */
#define UART4_BASE_ADDR             (APB1_BASE_ADDR + 0x4C00U)  /* UART4 base address */
#define UART5_BASE_ADDR             (APB1_BASE_ADDR + 0x5000U)  /* UART5 base address */
#define I2C1_BASE_ADDR              (APB1_BASE_ADDR + 0x5400U)  /* I2C1 base address */
#define I2C2_BASE_ADDR              (APB1_BASE_ADDR + 0x5800U)  /* I2C2 base address */
#define I2C3_BASE_ADDR              (APB1_BASE_ADDR + 0x5C00U)  /* I2C3 base address */
#define CAN1_BASE_ADDR              (APB1_BASE_ADDR + 0x6400U)  /* CAN1 base address */
#define CAN2_BASE_ADDR              (APB1_BASE_ADDR + 0x6800U)  /* CAN2 base address */
#define PWR_BASE_ADDR               (APB1_BASE_ADDR + 0x7000U)  /* PWR base address */
#define DAC_BASE_ADDR               (APB1_BASE_ADDR + 0x7400U)  /* DAC base address */
#define UART7_BASE_ADDR             (APB1_BASE_ADDR + 0x7800U)  /* UART7 base address */
#define UART8_BASE_ADDR             (APB1_BASE_ADDR + 0x7C00U)  /* UART8 base address */

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * */
#define TIM1_BASE_ADDR             (APB2_BASE_ADDR)             /* TIM1 base address */
#define TIM8_BASE_ADDR             (APB2_BASE_ADDR + 0x0400U)   /* TIM9 base address */
#define USART1_BASE_ADDR           (APB2_BASE_ADDR + 0x1000U)   /* USART1 base address */
#define USART6_BASE_ADDR           (APB2_BASE_ADDR + 0x1400U)   /* USART6 base address */

#define ADC1_BASE_ADDR             (APB2_BASE_ADDR + 0x2000U)   /* ADC1 base address */
#define ADC2_BASE_ADDR             (ADC1_BASE_ADDR)             /* ADC2 base address */
#define ADC3_BASE_ADDR             (ADC1_BASE_ADDR)             /* ADC3 base address */

#define SDIO_BASE_ADDR             (APB2_BASE_ADDR + 0x2C00U)   /* SDIO base address */
#define SPI1_BASE_ADDR             (APB2_BASE_ADDR + 0x3000U)   /* SPI1 base address */
#define SPI4_BASE_ADDR             (APB2_BASE_ADDR + 0x3400U)   /* SPI4 base address */
#define SYSCFG_BASE_ADDR           (APB2_BASE_ADDR + 0x3800U)   /* SYSCFG base address */
#define EXTI_BASE_ADDR             (APB2_BASE_ADDR + 0x3C00U)   /* EXTI base address */
#define TIM9_BASE_ADDR             (APB2_BASE_ADDR + 0x4000U)   /* TIM9 base address */
#define TIM10_BASE_ADDR            (APB2_BASE_ADDR + 0x4400U)   /* TIM10 base address */
#define TIM11_BASE_ADDR            (APB2_BASE_ADDR + 0x4800U)   /* TIM11 base address */
#define SPI5_BASE_ADDR             (APB2_BASE_ADDR + 0x5000U)   /* SPI5 base address */
#define SPI6_BASE_ADDR             (APB2_BASE_ADDR + 0x5400U)   /* SPI6 base address */
#define SAI1_BASE_ADDR             (APB2_BASE_ADDR + 0x5800U)   /* SAI1 base address */
#define LCD_TFT_BASE_ADDR          (APB2_BASE_ADDR + 0x6800U)   /* LCD TFT base address */


/*** Peripheral Register Definition Structures ***/

/**
 * @brief Reset and clock configuration registers
 * @param CR RCC clock control register
 * @param PLLCFGR RCC PLL configuration register
 * @param CFGR RCC clock configuration register
 * @param CIR RCC clock interrupt register
 * @param AHB1RSTR RCC AHB1 peripheral reset register
 * @param AHB2RSTR RCC AHB2 peripheral reset register
 * @param AHB3RSTR RCC AHB3 peripheral reset register
 * @param APB1RSTR RCC APB1 peripheral reset register
 * @param APB2RSTR RCC APB2 peripheral reset register
 * @param AHB1ENR RCC AHB1 peripheral clock enable register
 * @param AHB2ENR RCC AHB2 peripheral clock enable register
 * @param AHB3ENR RCC AHB3 peripheral clock enable register
 * @param APB1ENR RCC APB1 peripheral clock enable register
 * @param APB2ENR RCC APB2 peripheral clock enable register
 * @param AHB1LPENR RCC AHB1 peripheral clock enable in low power mode register
 * @param AHB2LPENR RCC AHB2 peripheral clock enable in low power mode register
 * @param AHB3LPENR RCC AHB3 peripheral clock enable in low power mode register
 * @param APB1LPENR RCC APB1 peripheral clock enable in low power mode register
 * @param APB2LPENR RCC APB2 peripheral clock enable in low power mode register
 * @param BDCR RCC Backup domain control register
 * @param CSR RCC clock control & status register
 * @param SSCGR RCC spread spectrum clock generation register
 * @param PLLI2SCFGR RCC PLLI2S configuration register
 */
typedef struct __attribute__((packed, aligned(1)))
{
	_VO uint32_t CR;
	_VO uint32_t PLLCFGR;
	_VO uint32_t CFGR;
	_VO uint32_t CIR;
	_VO uint32_t AHB1RSTR;
	_VO uint32_t AHB2RSTR;
	_VO uint32_t AHB3RSTR;
        uint32_t :32;
	_VO uint32_t APB1RSTR;
	_VO uint32_t APB2RSTR;
        uint32_t :32;
        uint32_t :32;
	_VO uint32_t AHB1ENR;
	_VO uint32_t AHB2ENR;
	_VO uint32_t AHB3ENR;
        uint32_t :32;
	_VO uint32_t APB1ENR;
	_VO uint32_t APB2ENR;
        uint32_t :32;
        uint32_t :32;
	_VO uint32_t AHB1LPENR;
	_VO uint32_t AHB2LPENR;
	_VO uint32_t AHB3LPENR;
        uint32_t :32;
	_VO uint32_t APB1LPENR;
	_VO uint32_t APB2LPENR;
    _VO uint32_t :32;
    _VO uint32_t :32;
    _VO uint32_t BDCR;
    _VO uint32_t CSR;
        uint32_t :32;
        uint32_t :32;
    _VO uint32_t SSCGR;
    _VO uint32_t PLLI2SCFGR;
} RCC_Reg_t;

/*
 * RCC registers handle definition
 * */
#define RCC ((RCC_Reg_t*)RCC_BASE_ADDR)

/**
 * @brief General purpose input output perirpheral registers
 * @param MODER GPIO port mode register
 * @param OTYPER GPIO port output type register
 * @param OSPEEDR GPIO port output speed register
 * @param PUPDR GPIO port pull-up/pull-down register
 * @param IDR GPIO port input data register
 * @param ODR GPIO port output data register
 * @param BSRR GPIO port bit set/reset register
 * @param LCKR GPIO port configuration lock register
 * @param AFR[] GPIO alternate function registers, AFR[0] -> low register, AFR[1] -> high register
 */
typedef struct __attribute__((packed, aligned(1)))
{
	_VO uint32_t MODER;
	_VO uint32_t OTYPER;
	_VO uint32_t OSPEEDR;
	_VO uint32_t PUPDR;
	_VO uint32_t IDR;
	_VO uint32_t ODR;
    _VO uint32_t BSRR;
    _VO uint32_t LCKR;
    _VO uint32_t AFR[2];
} GPIO_Reg_t;

/*
 * GPIO registers handle definitions
 * */
#define GPIOA ((GPIO_Reg_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_Reg_t*)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_Reg_t*)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_Reg_t*)GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_Reg_t*)GPIOE_BASE_ADDR)
#define GPIOF ((GPIO_Reg_t*)GPIOF_BASE_ADDR)
#define GPIOG ((GPIO_Reg_t*)GPIOG_BASE_ADDR)
#define GPIOH ((GPIO_Reg_t*)GPIOH_BASE_ADDR)
#define GPIOI ((GPIO_Reg_t*)GPIOI_BASE_ADDR)
#define GPIOJ ((GPIO_Reg_t*)GPIOJ_BASE_ADDR)
#define GPIOK ((GPIO_Reg_t*)GPIOK_BASE_ADDR)

/**
 * @brief External interrupt perirpheral registers
 * @param IMR Interrupt mask register
 * @param IMR Event mask register
 * @param RTSR Rising trigger selection register
 * @param FTSR Falling trigger selection register
 * @param SWIER Software interrupt event register
 * @param PR Pending register
 */
typedef struct __attribute__((packed, aligned(1)))
{
	_VO uint32_t IMR;
	_VO uint32_t EMR;
	_VO uint32_t RTSR;
	_VO uint32_t FTSR;
	_VO uint32_t SWIER;
	_VO uint32_t PR;
} EXTI_Reg_t;

/*
 * External interrupt registers handle definitions
 * */
#define EXTI ((EXTI_Reg_t*)EXTI_BASE_ADDR)

/**
 * @brief System configuration controller registers
 * @param MEMRMP SYSCFG memory remap register
 * @param PMC SYSCFG peripheral mode configuration register
 * @param EXTICR1 SYSCFG external interrupt configuration register 1
 * @param EXTICR2 SYSCFG external interrupt configuration register 2
 * @param EXTICR3 SYSCFG external interrupt configuration register 3
 * @param EXTICR4 SYSCFG external interrupt configuration register 4
 * @param CMPCR Compensation cell control register
 */
typedef struct __attribute__((packed, aligned(1)))
{
	_VO uint32_t MEMRMP;
	_VO uint32_t PMC;
	_VO uint32_t EXTICR[4];
	uint32_t :32;
	uint32_t :32;
	_VO uint32_t CMPCR;
} SYSCFG_Reg_t;

/*
 * System configuration controller registers handle definitions
 * */
#define SYSCFG ((SYSCFG_Reg_t*)SYSCFG_BASE_ADDR)

/*** Clock enabling & disabling macros ***/

/*
 * Clock enable & disable macros for GPIOx peripherals
 * */
#define __GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define __GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define __GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define __GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define __GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define __GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define __GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define __GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define __GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1 << 8))

#define __GPIOA_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 0))
#define __GPIOB_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 1))
#define __GPIOC_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 2))
#define __GPIOD_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 3))
#define __GPIOE_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 4))
#define __GPIOF_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 5))
#define __GPIOG_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 6))
#define __GPIOH_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 7))
#define __GPIOI_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock enable & disable macro for CRC peripheral
 * */
#define __CRC_PCLK_EN() (RCC->AHB1ENR |= (1 << 12))
#define __CRC_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 12))

/*
 * Clock enable & disable macro for BKPSRAM (Backup SRAM interface) peripheral
 * */
#define __BKPSRAM_PCLK_EN() (RCC->AHB1ENR |= (1 << 18))
#define __BKPSRAM_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 18))

/*
 * Clock enable & disable macro for CCMDATARAM (CCM data RAM) peripheral
 * */
#define __CCMDATARAM_PCLK_EN() (RCC->AHB1ENR |= (1 << 20))
#define __CCMDATARAM_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 20))

/*
 * Clock enable & disable macro for DMA peripherals
 * */
#define __DMA1_PCLK_EN() (RCC->AHB1ENR |= (1 << 21))
#define __DMA2_PCLK_EN() (RCC->AHB1ENR |= (1 << 22))

#define __DMA1_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 21))
#define __DMA2_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 22))

/*
 * Clock enable & disable macro for ETHMAC (Ethernet MAC) peripherals
 * */
#define __ETHMAC_PCLK_EN() (RCC->AHB1ENR |= (1 << 25))
#define __ETHMACTX_PCLK_EN() (RCC->AHB1ENR |= (1 << 26))
#define __ETHMACRX_PCLK_EN() (RCC->AHB1ENR |= (1 << 27))
#define __ETHMACPTP_PCLK_EN() (RCC->AHB1ENR |= (1 << 28))

#define __ETHMAC_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 25))
#define __ETHMACTX_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 26))
#define __ETHMACRX_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 27))
#define __ETHMACPTP_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 28))

/*
 * Clock enable & disable macro for OTGHS (USB OTG HS) peripheral
 * */
#define __OTGHS_PCLK_EN() (RCC->AHB1ENR |= (1 << 29))
#define __OTGHSULPI_PCLK_EN() (RCC->AHB1ENR |= (1 << 30))

#define __OTGHS_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 29))
#define __OTGHSULPI_PCLK_DIS() (RCC->AHB1ENR &= ~(1 << 30))

/*
 * Clock enable & disable macro for DCMI (Camera interface) peripheral
 * */
#define __DCMI_PCLK_EN() (RCC->AHB2ENR |= (1 << 0))
#define __DCMI_PCLK_DIS() (RCC->AHB2ENR &= ~(1 << 0))

/*
 * Clock enable & disable macro for CRYP (Cryptographic modules) peripheral
 * */
#define __CRYP_PCLK_EN() (RCC->AHB2ENR |= (1 << 4))
#define __CRYP_PCLK_DIS() (RCC->AHB2ENR &= ~(1 << 4))

/*
 * Clock enable & disable macro for HASH (Hash modules) peripheral
 * */
#define __HASH_PCLK_EN() (RCC->AHB2ENR |= (1 << 5))
#define __HASH_PCLK_DIS() (RCC->AHB2ENR &= ~(1 << 5))

/*
 * Clock enable & disable macro for RNG (Random number generator) peripheral
 * */
#define __RNG_PCLK_EN() (RCC->AHB2ENR |= (1 << 6))
#define __RNG_PCLK_DIS() (RCC->AHB2ENR &= ~(1 << 6))

/*
 * Clock enable & disable macro for OTGFS (USB OTG FS) peripheral
 * */
#define __OTGFS_PCLK_EN() (RCC->AHB2ENR |= (1 << 7))
#define __OTGFS_PCLK_DIS() (RCC->AHB2ENR &= ~(1 << 7))

/*
 * Clock enable & disable macro for FSMC (Flexible static memory controller) peripheral
 * */
#define __FSMC_PCLK_EN() (RCC->AHB3ENR |= (1 << 0))
#define __FSMC_PCLK_DIS() (RCC->AHB3ENR &= ~(1 << 0))

/*
 * Clock enable & disable macro for Timer peripherals
 * */
#define __TIM1_PCLK_EN() (RCC->APB2ENR |= (1 << 0))
#define __TIM2_PCLK_EN() (RCC->APB1ENR |= (1 << 0))
#define __TIM3_PCLK_EN() (RCC->APB1ENR |= (1 << 1))
#define __TIM4_PCLK_EN() (RCC->APB1ENR |= (1 << 2))
#define __TIM5_PCLK_EN() (RCC->APB1ENR |= (1 << 3))
#define __TIM6_PCLK_EN() (RCC->APB1ENR |= (1 << 4))
#define __TIM7_PCLK_EN() (RCC->APB1ENR |= (1 << 0))
#define __TIM8_PCLK_EN() (RCC->APB2ENR |= (1 << 1))
#define __TIM9_PCLK_EN() (RCC->APB2ENR |= (1 << 16))
#define __TIM10_PCLK_EN() (RCC->APB2ENR |= (1 << 17))
#define __TIM11_PCLK_EN() (RCC->APB2ENR |= (1 << 18))
#define __TIM12_PCLK_EN() (RCC->APB1ENR |= (1 << 5))
#define __TIM13_PCLK_EN() (RCC->APB1ENR |= (1 << 6))
#define __TIM14_PCLK_EN() (RCC->APB1ENR |= (1 << 8))

#define __TIM1_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 0))
#define __TIM2_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 0))
#define __TIM3_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 1))
#define __TIM4_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 2))
#define __TIM5_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 3))
#define __TIM6_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 4))
#define __TIM7_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 0))
#define __TIM8_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 1))
#define __TIM9_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 16))
#define __TIM10_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 17))
#define __TIM11_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 18))
#define __TIM12_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 5))
#define __TIM13_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 6))
#define __TIM14_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 8))

/*
 * Clock enable & disable macro for WWDG peripheral
 * */
#define __WWDG_PCLK_EN() (RCC->APB1ENR |= (1 << 11))
#define __WWDG_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 11))

/*
 * Clock enable & disable macro for SPI peripherals
 * */
#define __SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define __SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define __SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))

#define __SPI1_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 12))
#define __SPI2_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 14))
#define __SPI3_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock enable & disable macro for UART & USART peripherals
 * */
#define __USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define __USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define __USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define __UART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define __UART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))
#define __USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

#define __USART1_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 4))
#define __USART2_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 17))
#define __USART3_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 18))
#define __UART4_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 19))
#define __UART5_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 20))
#define __USART6_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock enable & disable macro for I2C peripherals
 * */
#define __I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define __I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define __I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

#define __I2C1_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 21))
#define __I2C2_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 22))
#define __I2C3_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock enable & disable macro for CAN peripherals
 * */
#define __CAN1_PCLK_EN() (RCC->APB1ENR |= (1 << 25))
#define __CAN2_PCLK_EN() (RCC->APB1ENR |= (1 << 26))

#define __CAN1_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 25))
#define __CAN2_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 26))

/*
 * Clock enable & disable macro for PWR peripheral
 * */
#define __PWR_PCLK_EN() (RCC->APB1ENR |= (1 << 28))

#define __PWR_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 28))

/*
 * Clock enable & disable macro for DAC peripheral
 * */
#define __DAC_PCLK_EN() (RCC->APB1ENR |= (1 << 29))

#define __DAC_PCLK_DIS() (RCC->APB1ENR &= ~(1 << 29))

/*
 * Clock enable & disable macro for ADC peripherals
 * */
#define __ADC1_PCLK_EN() (RCC->APB2ENR |= (1 << 8))
#define __ADC2_PCLK_EN() (RCC->APB2ENR |= (1 << 9))
#define __ADC3_PCLK_EN() (RCC->APB2ENR |= (1 << 10))

#define __ADC1_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 8))
#define __ADC2_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 9))
#define __ADC3_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 10))

/*
 * Clock enable & disable macro for SDIO peripheral
 * */
#define __SDIO_PCLK_EN() (RCC->APB2ENR |= (1 << 11))

#define __SDIO_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 11))

/*
 * Clock enable & disable macro for SYSCFG (System configuration controller) peripheral
 * */
#define __SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

#define __SYSCFG_PCLK_DIS() (RCC->APB2ENR &= ~(1 << 14))


#endif /* INC_STM32F407XX_H_ */

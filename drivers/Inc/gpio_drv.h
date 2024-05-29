/*
 * gpio_drv.h
 *
 *  Created on: May 28, 2024
 *      Author: nesh
 */

#ifndef INC_GPIO_DRV_H_
#define INC_GPIO_DRV_H_

#include "stm32f407xx.h"

#include <string.h>
#include <stdbool.h>

/* MACROs */

/* ENUMs */

typedef enum
{
	_GPIO_PIN_0  = 0x00,
	_GPIO_PIN_1        ,
	_GPIO_PIN_2        ,
	_GPIO_PIN_3        ,
	_GPIO_PIN_4        ,
	_GPIO_PIN_5        ,
	_GPIO_PIN_6        ,
	_GPIO_PIN_7        ,
	_GPIO_PIN_8        ,
	_GPIO_PIN_9        ,
	_GPIO_PIN_10       ,
	_GPIO_PIN_11       ,
	_GPIO_PIN_12       ,
	_GPIO_PIN_13       ,
	_GPIO_PIN_14       ,
	_GPIO_PIN_15
} GPIO_Pin_number_t;

typedef enum
{
	_GPIO_PIN_MODE_INPUT    = 0x00,
	_GPIO_PIN_MODE_OUTPUT   = 0x01,
	_GPIO_PIN_MODE_ALT_FUNC = 0x02,
	_GPIO_PIN_MODE_ANALOG   = 0x03
} GPIO_Pin_modes_t;

typedef enum
{
	_GPIO_PIN_OUTPUT_PUSH_PULL    = 0x00,
	_GPIO_PIN_OUTPUT_OPEN_DRAIN
} GPIO_Pin_output_type_t;

typedef enum
{
	_GPIO_PIN_OUTPUT_LOW_SPEED    = 0x00,
	_GPIO_PIN_OUTPUT_MEDIUM_SPEED       ,
	_GPIO_PIN_OUTPUT_HIGH_SPEED         ,
	_GPIO_PIN_OUTPUT_VERY_HIGH_SPEED
} GPIO_Pin_output_speed_t;

typedef enum
{
	_GPIO_PIN_NO_PULL_UP_PULL_DOWN    = 0x00,
	_GPIO_PIN_PULL_UP                       ,
	_GPIO_PIN_PULL_DOWN
} GPIO_Pin_pull_type_t;

typedef enum
{
	_GPIO_PIN_ALT_FUNC_0 = 0x00, // SYS 
	_GPIO_PIN_ALT_FUNC_1       , // TIM1 - TIM2
	_GPIO_PIN_ALT_FUNC_2       , // TIM3 - TIM4 - TIM5
	_GPIO_PIN_ALT_FUNC_3       , // TIM8 - TIM9 - TIM10 - TIM11
	_GPIO_PIN_ALT_FUNC_4       , // I2C1 - I2C2 - I2C3
	_GPIO_PIN_ALT_FUNC_5       , // SPI1 - SPI2 - I2S2 - I2S2ext
	_GPIO_PIN_ALT_FUNC_6       , // SPI3 - I2Sext - I2S3
	_GPIO_PIN_ALT_FUNC_7       , // USART1 - USART2 - USART3 - I2S3ext
	_GPIO_PIN_ALT_FUNC_8       , // UART4 - UART5 - USART6
	_GPIO_PIN_ALT_FUNC_9       , // CAN1 - CAN2 - TIM12 - TIM13 - TIM14
	_GPIO_PIN_ALT_FUNC_10      , // OTG_FS - OTG_HS
	_GPIO_PIN_ALT_FUNC_11      , // ETH
	_GPIO_PIN_ALT_FUNC_12      , // FSMC - SDIO - OTG_FS
	_GPIO_PIN_ALT_FUNC_13      , // DCMI
	_GPIO_PIN_ALT_FUNC_14      , // 
	_GPIO_PIN_ALT_FUNC_15      , // 
} GPIO_Pin_alt_func_t;

typedef enum
{
	_GPIO_PIN_STATE_LOW    = 0,
	_GPIO_PIN_STATE_HIGH   = 1
} GPIO_Pin_state_t;

typedef enum
{
	_GPIO_WRITE_RES_NOT_PERMISION = -2,
	_GPIO_WRITE_RES_NOT_SUCCES    = -1,
	_GPIO_WRITE_RES_SUCCES        =  0
} GPIO_Pin_write_res_t;

/* STRUCTs */

typedef struct
{
	uint8_t number;			// Pin position as `GPIO_Pin_number_t`
	uint8_t mode;			// Pin mode tpye as `GPIO_Pin_modes_t`
	uint8_t output_type;	// Output tpye as `GPIO_Pin_output_type_t`
	uint8_t speed;			// Output pin speed type as `GPIO_Pin_output_speed_t`
	uint8_t pull_type;		// Pin pull type as `GPIO_Pin_pull_type_t`
	uint8_t alt_func_mode;	// Alternate function as `GPIO_Pin_alt_func_t`
} GPIO_Pin_config_t;

typedef struct
{
	GPIO_Reg_t *gpiox; /* The param keep the base address of GPIO port*/
	GPIO_Pin_config_t pin_config;
} GPIO_Handle_t;

/* FUNCTIONs */

/**
 * @brief Peripheral clock setups
 * @param gpiox GPIO port base address
 * @param state GPIO port clock output state (true: enable, false: disable)
 * @return bool When the set operation successfully return true, else false
 */
bool gpio_drv_PeripheralClockControl(GPIO_Reg_t *gpiox, bool state);

/**
 * @brief GPIO Pin configuration and initialization
 * @param self GPIO pin handle base address
 * @param gpiox GPIO port base address
 * @param pin GPIO pin number as `GPIO_Pin_number_t`
 * @param mode GPIO pin mode as `GPIO_Pin_modes_t`
 * @param output_type GPIO pin output type (if the pin is an output pin) as `output_type`
 * @param speed GPIO pin speed type (if the pin is an output pin) as `GPIO_Pin_output_speed_t`
 * @param pull_type GPIO pin pull type as `GPIO_Pin_pull_type_t`
 * @param alt_func GPIO pin alternate function (When the pin use as alternate) type as `GPIO_Pin_alt_func_t`
 * @return bool When the set operation successfully return true, else false
 */
bool gpio_drv_Init(GPIO_Handle_t *self, GPIO_Reg_t *gpiox, GPIO_Pin_number_t pin, GPIO_Pin_modes_t mode,
	GPIO_Pin_output_type_t output_type, GPIO_Pin_output_speed_t speed, GPIO_Pin_pull_type_t pull_type,
	GPIO_Pin_alt_func_t alt_func);

/**
 * @brief GPIO Pin de-initialization
 * @param self GPIO pin handle base address
 * @return bool When the set operation successfully return true, else false
 */
bool gpio_drv_DeInit(GPIO_Handle_t *self);

/**
 * @brief GPIO Pin state read
 * @param self GPIO pin handle base address
 * @return GPIO_Pin_state_t When the pin is high, return `_GPIO_PIN_STATE_HIGH`
 */
GPIO_Pin_state_t gpio_drv_Read(GPIO_Handle_t *self);

/**
 * @brief GPIO Pin state write
 * @param self GPIO pin handle base address
 * @param state GPIO pin target state `GPIO_Pin_state_t`
 * @return GPIO_Pin_write_res_t When the writing operation is successfully, return `_GPIO_WRITE_RES_SUCCES`
 */
GPIO_Pin_write_res_t gpio_drv_Write(GPIO_Handle_t *self, GPIO_Pin_state_t state);

/**
 * @brief GPIO Pin state toggle
 * @param self GPIO pin handle base address
 * @return GPIO_Pin_write_res_t When the writing operation is successfully, return `_GPIO_WRITE_RES_SUCCES`
 */
GPIO_Pin_write_res_t gpio_drv_Toggle(GPIO_Handle_t *self);

#endif /* INC_GPIO_DRV_H_ */

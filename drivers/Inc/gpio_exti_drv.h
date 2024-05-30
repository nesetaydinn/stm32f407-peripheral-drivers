/*
 * gpio_exti_drv.h
 *
 *  Created on: May 29, 2024
 *      Author: nesh
 */

#ifndef INC_GPIO_EXTI_DRV_H_
#define INC_GPIO_EXTI_DRV_H_

#include "stm32f407xx.h"
#include "gpio_drv.h"

#include <string.h>
#include <stdbool.h>

/* MACROs */

/* ENUMs */

typedef enum
{
    _GPIO_EXTI_PIN_TRIG_FALLING_EDGE        = 0x00,
    _GPIO_EXTI_PIN_TRIG_RISING_EDGE               ,
    _GPIO_EXTI_PIN_TRIG_RISING_FALLING_EDGE
} GPIO_EXTI_Pin_trigging_t;

typedef enum
{
    _GPIO_EXTI_NO_0     = 0x06,
    _GPIO_EXTI_NO_1     = 0x07,
    _GPIO_EXTI_NO_2     = 0x08,
    _GPIO_EXTI_NO_3     = 0x09,
    _GPIO_EXTI_NO_4     = 0x0A,
    _GPIO_EXTI_NO_9_5   = 0x17,
    _GPIO_EXTI_NO_15_10 = 0x28
} GPIO_EXTI_IRQ_nubmers_t;

/* STRUCTs */

typedef struct
{
	uint8_t number;			// Pin position as `GPIO_Pin_number_t`
    uint8_t trigging_type;  // Pin trigging type as `GPIO_EXTI_Pin_trigging_t`
    uint8_t irq_num;        // Pin IRQ type as `GPIO_EXTI_IRQ_nubmers_t`
    uint8_t irq_priority;   // Pin IRQ priority
} GPIO_EXTI_Pin_config_t;

typedef struct
{
    GPIO_Handle_t gpio_handle;
    GPIO_EXTI_Pin_config_t pin_config;
    void (*irq_event)(void);
} GPIO_EXTI_Handle_t;

/* FUNCTIONs */

/**
 * @brief GPIO Pin configuration and initialization
 * @param self GPIO pin handle base address
 * @param gpiox GPIO port base address
 * @param pin GPIO pin number as `GPIO_Pin_number_t`
 * @param pull_type GPIO pin pull type as `GPIO_Pin_pull_type_t`
 * @param trigging GPIO pin number as `GPIO_EXTI_Pin_trigging_t`
 * @param irq_priority IRQ priority
 * @param irq_event IRQ triggered event function
 * @return bool When the set operation successfully return true, else false
 */
bool gpio_exti_drv_Init(GPIO_EXTI_Handle_t *self, GPIO_Reg_t *gpiox, GPIO_Pin_number_t pin,
    GPIO_Pin_pull_type_t pull_type, GPIO_EXTI_Pin_trigging_t trigging, uint8_t irq_priority,
	void (*irq_event)(void));

/**
 * @brief GPIO External Interrupt handler
 * @param self GPIO pin handle base address
 * @return void
 * */
void gpio_exti_drv_IRQHandler(GPIO_EXTI_Handle_t *self);

/**
 * @brief GPIO Pin configuration and de-initialization
 * @param self GPIO pin handle base address
 * @return bool When the operation is successfully return true, else false
 * */
bool gpio_exti_drv_DeInit(GPIO_EXTI_Handle_t *self);

#endif /* INC_GPIO_EXTI_DRV_H_ */

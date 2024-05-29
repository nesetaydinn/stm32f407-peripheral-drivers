/*
 * gpio_exti_drv.c
 *
 *  Created on: May 29, 2024
 *      Author: nesh
 */

#include "gpio_exti_drv.h"

/*
 * EXTI interrupt config,
 * 1. The pin should be input config,
 * 2. Config the edge trigger,
 * 3. Enable interrupt delivery from peripheral to mcu (on peripheral side),
 * 4. Identify the IRQ number on which the mcu accepts the interrupt from that pin,
 * 5. Configure the IRQ priority for the identified IRQ number (proccessor side),
 * 6. Enable interrupt reception on that IRQ number (proccessor side),
 * 7. Implement IRQ handler
 * */

bool gpio_exti_drv_Init(GPIO_EXTI_Handle_t *self, GPIO_Reg_t *gpiox, GPIO_Pin_number_t pin,
    GPIO_Pin_pull_type_t pull_type, GPIO_EXTI_Pin_trigging_t trigging, uint8_t irq_priority,
	void (*irq_event)(void))
{
	if (NULL == self)
		return false;
	memset(self, 0, sizeof(*self));

	if (_GPIO_PIN_15 < pin)
		return false;

	if (EXTI->IMR &= (1 << pin))
		return false;

	self->irq_event = irq_event;

	/* GPIO Pin initializing */
	gpio_drv_PeripheralClockControl(gpiox, true);
	gpio_drv_Init(&self->gpio_handle, gpiox, pin, _GPIO_PIN_MODE_INPUT, 0, 0, pull_type, 0);

	/* Trigging type selection */
	if (_GPIO_EXTI_PIN_TRIG_FALLING_EDGE == trigging)
	{
		EXTI->FTSR |= (1 << self->gpio_handle.pin_config.number);
		EXTI->RTSR &= ~(1 << self->gpio_handle.pin_config.number);
	}
	else if (_GPIO_EXTI_PIN_TRIG_RISING_EDGE == trigging)
	{
		EXTI->RTSR |= (1 << self->gpio_handle.pin_config.number);
		EXTI->FTSR &= ~(1 << self->gpio_handle.pin_config.number);

	}
	else if (_GPIO_EXTI_PIN_TRIG_RISING_FALLING_EDGE == trigging)
	{
		EXTI->FTSR |= (1 << self->gpio_handle.pin_config.number);
		EXTI->RTSR |= (1 << self->gpio_handle.pin_config.number);
	}
	else
		return false;

	/* Configure the GPIO port selection */
	uint8_t port_code = (((volatile uint32_t*)gpiox - (volatile uint32_t*)GPIOA_BASE_ADDR) >> 8);;
	__SYSCFG_PCLK_EN(); // Enable SYSCFG clock
	SYSCFG->EXTICR[self->gpio_handle.pin_config.number / 4] |=
		(port_code << ((self->gpio_handle.pin_config.number % 4) * 4));


	/* Enable the EXTI interrupt delivery */
	EXTI->IMR |= (1 << self->gpio_handle.pin_config.number);

	/* Enable NVIC IRQ*/
	if (0 == self->gpio_handle.pin_config.number)
		self->pin_config.irq_num = _GPIO_EXTI_NO_0;
	else if (1 == self->gpio_handle.pin_config.number)
		self->pin_config.irq_num = _GPIO_EXTI_NO_1;
	else if (2 == self->gpio_handle.pin_config.number)
		self->pin_config.irq_num = _GPIO_EXTI_NO_2;
	else if (3 == self->gpio_handle.pin_config.number)
		self->pin_config.irq_num = _GPIO_EXTI_NO_3;
	else if (4 == self->gpio_handle.pin_config.number)
		self->pin_config.irq_num = _GPIO_EXTI_NO_4;
	else if ((5 <= self->gpio_handle.pin_config.number) &&
			 (10 > self->gpio_handle.pin_config.number))
		self->pin_config.irq_num = _GPIO_EXTI_NO_9_5;
	else if ((10 <= self->gpio_handle.pin_config.number) &&
			 (15 > self->gpio_handle.pin_config.number))
		self->pin_config.irq_num = _GPIO_EXTI_NO_15_10;

	if (_GPIO_EXTI_NO_9_5 >= self->pin_config.irq_num)
	{
		volatile uint32_t *arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E100;
		*arm_nvic_iser_base_addr |= (1 << self->pin_config.irq_num);
	}
	else
	{
		volatile uint32_t *arm_nvic_iser1 = (volatile uint32_t *)0xE000E104;
		*arm_nvic_iser1 |= (1 << (self->pin_config.irq_num % 32));
	}

	uint8_t shift_amount = (8 * (self->pin_config.irq_num)) +4;
	volatile uint32_t *arm_nvic_pr_base_addr = (volatile uint32_t *)0xE000E400;
	*(arm_nvic_pr_base_addr + (self->pin_config.irq_num / 4)) |=
		(irq_priority << shift_amount);

	return true;
}

void gpio_exti_drv_IRQHandler(GPIO_EXTI_Handle_t *self)
{
	if (EXTI->PR & (1 << self->gpio_handle.pin_config.number))
	{
		if (NULL != self->irq_event)
			self->irq_event();
		EXTI->PR |= 1 << self->gpio_handle.pin_config.number;
	}
}

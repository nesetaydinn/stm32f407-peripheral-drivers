/*
 * gpio_drv.c
 *
 *  Created on: May 28, 2024
 *      Author: nesh
 */

#include "gpio_drv.h"

bool gpio_drv_Init(GPIO_Handle_t *self, GPIO_Reg_t *gpiox, GPIO_Pin_number_t pin, GPIO_Pin_modes_t mode,
	GPIO_Pin_output_type_t output_type, GPIO_Pin_output_speed_t speed, GPIO_Pin_pull_type_t pull_type,
	GPIO_Pin_alt_func_t alt_func)
{

	if ((NULL == self) || (((volatile uint32_t*)gpiox < (volatile uint32_t*)GPIOA_BASE_ADDR) ||
	    	((volatile uint32_t*)gpiox > (volatile uint32_t*)GPIOK_BASE_ADDR)))
		return false;
	memset(self, 0, sizeof(*self));
	self->gpiox = gpiox;
	self->pin_config.number = pin;
	self->pin_config.pull_type = pull_type;
	self->pin_config.mode = mode;

	if (_GPIO_PIN_MODE_OUTPUT == self->pin_config.mode)
	{
		self->pin_config.output_type = output_type;
		self->pin_config.speed = speed;
	}
	else if (_GPIO_PIN_MODE_ALT_FUNC == self->pin_config.mode)
	{
		self->pin_config.output_type = output_type;
		self->pin_config.speed = speed;
		self->pin_config.alt_func_mode = alt_func;
	}

	self->gpiox->LCKR &= ~(1 << self->pin_config.number);
	self->gpiox->MODER |= (self->pin_config.mode << (2 * self->pin_config.number));
	self->gpiox->OTYPER |= (self->pin_config.output_type << self->pin_config.number);
	self->gpiox->OSPEEDR |= (self->pin_config.speed << (2 * self->pin_config.number));
	self->gpiox->PUPDR |= (self->pin_config.pull_type << (2 * self->pin_config.number));
	if (_GPIO_PIN_MODE_ALT_FUNC == self->pin_config.mode)
	{
		self->gpiox->AFR[(self->pin_config.number > 7) ? 1 : 0] |=
			(self->pin_config.alt_func_mode << (4 * self->pin_config.number));
	}
	self->gpiox->LCKR |= self->pin_config.number;
	return true;
}

GPIO_Pin_state_t gpio_drv_Read(GPIO_Handle_t *self)
{
	return ((self->gpiox->IDR & (1 << self->pin_config.number)) >> self->pin_config.number);
}

GPIO_Pin_write_res_t gpio_drv_Write(GPIO_Handle_t *self, GPIO_Pin_state_t state)
{
	if ((_GPIO_PIN_MODE_ANALOG == self->pin_config.mode) ||
		(_GPIO_PIN_MODE_INPUT == self->pin_config.mode))
		return _GPIO_WRITE_RES_NOT_PERMISION;
	self->gpiox->BSRR |= (1 << (self->pin_config.number + (state ? 0 : 16)));
	if (self->gpiox->ODR & (state << self->pin_config.number))
		return _GPIO_WRITE_RES_SUCCES;
	return _GPIO_WRITE_RES_NOT_SUCCES;
}

GPIO_Pin_write_res_t gpio_drv_Toggle(GPIO_Handle_t *self)
{
	if ((_GPIO_PIN_MODE_ANALOG == self->pin_config.mode) ||
		(_GPIO_PIN_MODE_INPUT == self->pin_config.mode))
		return _GPIO_WRITE_RES_NOT_PERMISION;
	bool state = (bool)(self->gpiox->ODR & (1 << self->pin_config.number));
	self->gpiox->BSRR |= (1 << (self->pin_config.number + (state ? 16 : 0)));
	if (self->gpiox->ODR & (state << self->pin_config.number))
		return _GPIO_WRITE_RES_SUCCES;
	return _GPIO_WRITE_RES_NOT_SUCCES;
}

bool gpio_drv_PeripheralClockControl(GPIO_Reg_t *gpiox, bool state)
{
    if (((volatile uint32_t*)gpiox < (volatile uint32_t*)GPIOA_BASE_ADDR) ||
    	((volatile uint32_t*)gpiox > (volatile uint32_t*)GPIOK_BASE_ADDR))
    {
        return false;
    }
	uint8_t pos = (((volatile uint32_t*)gpiox - (volatile uint32_t*)GPIOA_BASE_ADDR) >> 8);
    if (state)
    {
        RCC->AHB1ENR |= (1 << pos);
        return true;
    }
    RCC->AHB1ENR &= ~(1 << pos);
    return true;
}

bool gpio_drv_DeInit(GPIO_Handle_t *self)
{
	if (NULL == self)
		return false;
	self->gpiox->LCKR &= ~(1 << self->pin_config.number);

	self->gpiox->MODER &= ~(self->pin_config.mode << (2 * self->pin_config.number));
	self->gpiox->OTYPER &= ~(self->pin_config.output_type << self->pin_config.number);
	self->gpiox->OSPEEDR &= ~(self->pin_config.speed << (2 * self->pin_config.number));
	self->gpiox->PUPDR &= ~(self->pin_config.pull_type << (2 * self->pin_config.number));
	if (_GPIO_PIN_MODE_ALT_FUNC == self->pin_config.mode)
	{
		self->gpiox->AFR[(self->pin_config.number > 7) ? 1 : 0] &=
			~(self->pin_config.alt_func_mode << (4 * self->pin_config.number));
	}
	self->gpiox->LCKR |= self->pin_config.number;
	self->gpiox = NULL;
	memset(self, 0, sizeof(*self));
	return true;
}

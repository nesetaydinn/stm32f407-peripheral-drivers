/*
 * spi_drv.c
 *
 *  Created on: May 30, 2024
 *	Author: nesh
 */

#include "spi_drv.h"

/**
 * @brief SPI Peripheral Clock Control
 * @param self SPI handle base address
 * @param state Peripheral clock enabling control, true: enable
 * @return bool When the operation is successfully; return true 
 */
bool spi_drv_PeripheralClockControl(SPI_Reg_t *spix, bool state);

/**
 * @brief SPI Transmit interrupt
 * @param self SPI handle base address
 * @return
 */
static void spi_drv_TXEInterruptHandler(SPI_Handle_t *self);

/**
 * @brief SPI Receive interrupt
 * @param self SPI handle base address
 * @return
 */
static void spi_drv_RXNEInterruptHandler(SPI_Handle_t *self);

/**
 * @brief SPI Master mode fault interrupt
 * @param self SPI handle base address
 * @return
 */
static void spi_drv_MODFInterruptHandler(SPI_Handle_t *self);

/**
 * @brief SPI Overrun error interrupt
 * @param self SPI handle base address
 * @return
 */
static void spi_drv_OVRInterruptHandler(SPI_Handle_t *self);

/**
 * @brief SPI CRC error interrupt
 * @param self SPI handle base address
 * @return
 */
static void spi_drv_CRCERRInterruptHandler(SPI_Handle_t *self);

/**
 * @brief SPI TI mode frame format error interrupt
 * @param self SPI handle base address
 * @return
 */
static void spi_drv_FREInterruptHandler(SPI_Handle_t *self);

/**
 * @brief SPI Check the flag is set on Status register
 * @param self SPI handle base address
 * @param flag Interested flag
 * @return When the interested flag is set return true
 */
bool spi_drv_GetFlagStatus(SPI_Reg_t *spix, uint32_t flag);

bool spi_drv_Init(SPI_Handle_t *self, SPI_Reg_t *spix, SPI_config_t config)
{
	if (NULL == self)
		return false;

	if (!(((volatile SPI_Reg_t*)SPI1_BASE_ADDR == (volatile SPI_Reg_t*)spix) ||
		  ((volatile SPI_Reg_t*)SPI2_BASE_ADDR == (volatile SPI_Reg_t*)spix) ||
		  ((volatile SPI_Reg_t*)SPI3_BASE_ADDR == (volatile SPI_Reg_t*)spix)))
		return false;

	spi_drv_PeripheralClockControl(spix, true);

	memset(self, 0, sizeof(*self));
	self->config = config;
	self->spix = spix;

	/* Set config */
	self->spix->CR1 |= (self->config.mode << SPI_CR1_MSTR);
	if (self->config.mode && !self->config.ssm)
		self->spix->CR2 |= (1 << SPI_CR2_SSOE);

	if (_SPI_BUS_CONFIG_FULL_DULEX == self->config.bus_config)
	{
		self->spix->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (_SPI_BUS_CONFIG_HALF_DULEX == self->config.bus_config)
	{
		self->spix->CR1 |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (_SPI_BUS_CONFIG_SIMPLEX_RONLY == self->config.bus_config)
	{
		self->spix->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
		self->spix->CR1 |= (1 << SPI_CR1_RXONLY);
	}
	self->spix->CR1 |= (self->config.speed << SPI_CR1_BR);
	self->spix->CR1 |= (self->config.first_bit << SPI_CR1_LSBFIRST);
	self->spix->CR1 |= (self->config.dff << SPI_CR1_DFF);
	self->spix->CR1 |= (self->config.cpol << SPI_CR1_CPOL);
	self->spix->CR1 |= (self->config.cpha << SPI_CR1_CPHA);

	self->spix->CR1 |= (self->config.ssm << SPI_CR1_SSM);
	if (self->config.ssm)
	{
		self->spix->CR1 |= (1 << SPI_CR1_SSI);
	}

	return true;
}

bool spi_drv_SetInterrupts(SPI_Handle_t *self, uint8_t irq_priority, void (*irq_event)(void *self, uint8_t event))
{
	if (NULL == self->spix)
		return false;

	self->config.irq_priority = irq_priority;
	self->config.is_irq_enable = true;
	uint8_t irq_number = 0;
	if ((volatile SPI_Reg_t *)SPI1_BASE_ADDR == (volatile SPI_Reg_t *)self->spix)
		irq_number = _SPI_IRQ_NO_1;
	else if ((volatile SPI_Reg_t *)SPI2_BASE_ADDR == (volatile SPI_Reg_t *)self->spix)
		irq_number = _SPI_IRQ_NO_2;
	else if ((volatile SPI_Reg_t *)SPI3_BASE_ADDR == (volatile SPI_Reg_t *)self->spix)
		irq_number = _SPI_IRQ_NO_3;
	else
		return false;

	if (NULL != irq_event)
		self->irq_event = irq_event;

	self->spix->CR2 |= (1 << SPI_CR2_ERRIE);

	volatile uint32_t *arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E104;
	*arm_nvic_iser_base_addr |= (1 << (irq_number % 32));

	uint8_t iprx = irq_number / 4;
	uint8_t iprx_section = irq_number % 4;
	uint8_t shift_amount = (8 * iprx_section) + 4;
	volatile uint32_t *arm_nvic_pr_base_addr = (volatile uint32_t *)(0xE000E400 + (iprx * 4));
	*arm_nvic_pr_base_addr |= (irq_priority << shift_amount);
	return true;
}

bool spi_drv_SendData(SPI_Handle_t *self, uint8_t *data, uint32_t data_len)
{
	if (!data_len)
		return false;
	if ((_SPI_DFF_8BITS != self->config.dff) && (data_len % 2))
		return false;

	while (data_len)
	{
		while (!spi_drv_GetFlagStatus(self->spix, SPI_SR_TXE_FLAG));
		if (_SPI_DFF_8BITS == self->config.dff)
		{
			self->spix->DR = *data;
			data_len--;
			data++;
		}
		else
		{
			self->spix->DR = *((uint16_t*)data);
			data_len-= 2;
			(uint16_t*)data++;
		}
	}
	while (spi_drv_GetFlagStatus(self->spix, SPI_SR_BSY_FLAG));

	return true;
}

bool spi_drv_SendDataIT(SPI_Handle_t *self, uint8_t *data, uint32_t data_len)
{
	if ((!data_len) || (NULL == data))
		return false;

	if (_SPI_IRQ_STATE_BUSY_IN_TX == self->tx.state)
		return false;

	self->tx.buffer = data;
	self->tx.len = data_len;
	self->tx.state = _SPI_IRQ_STATE_BUSY_IN_TX;

	self->spix->CR2 |= (1 << SPI_CR2_TXEIE);

	return true;
}

bool spi_drv_ReceiveData(SPI_Handle_t *self, uint8_t *data, uint32_t data_len)
{
	if (!data_len)
		return false;
	if ((_SPI_DFF_8BITS != self->config.dff) && (data_len % 2))
		return false;

	while (data_len)
	{
		while (!spi_drv_GetFlagStatus(self->spix, SPI_SR_RXNE_FLAG));
		if (_SPI_DFF_8BITS == self->config.dff)
		{
			*data = self->spix->DR;
			data_len--;
			data++;
		}
		else
		{
			*((uint16_t*)data) = self->spix->DR;
			data_len-= 2;
			(uint16_t*)data++;
		}
	}
	while (spi_drv_GetFlagStatus(self->spix, SPI_SR_BSY_FLAG));

	return true;
}

bool spi_drv_ReceiveDataIT(SPI_Handle_t *self, uint8_t *data, uint32_t data_len)
{
	if ((!data_len) || (NULL == data))
		return false;

	if (_SPI_IRQ_STATE_BUSY_IN_RX == self->rx.state)
		return false;

	self->rx.buffer = data;
	self->rx.len = data_len;
	self->rx.state = _SPI_IRQ_STATE_BUSY_IN_RX;

	self->spix->CR2 |= (1 << SPI_CR2_RXNEIE);

	return true;
}

void spi_drv_PeripheralControl(SPI_Handle_t *self, bool state)
{
	if (state)
		self->spix->CR1 |= (1 << SPI_CR1_SPE);
	else
		self->spix->CR1 &= ~(1 << SPI_CR1_SPE);
}

bool spi_drv_PeripheralClockControl(SPI_Reg_t *spix, bool state)
{
	if (!(((volatile SPI_Reg_t*)SPI1_BASE_ADDR == (volatile SPI_Reg_t*)spix) ||
		  ((volatile SPI_Reg_t*)SPI2_BASE_ADDR == (volatile SPI_Reg_t*)spix) ||
		  ((volatile SPI_Reg_t*)SPI3_BASE_ADDR == (volatile SPI_Reg_t*)spix)))
		return false;
	if (state)
	{
		if ((volatile SPI_Reg_t*)SPI1_BASE_ADDR == (volatile SPI_Reg_t*)spix)
			__SPI1_PCLK_EN();
		else if ((volatile SPI_Reg_t*)SPI2_BASE_ADDR == (volatile SPI_Reg_t*)spix)
			__SPI2_PCLK_EN();
		else if ((volatile SPI_Reg_t*)SPI3_BASE_ADDR == (volatile SPI_Reg_t*)spix)
			__SPI3_PCLK_EN();
	}
	else
	{
		if ((volatile SPI_Reg_t*)SPI1_BASE_ADDR == (volatile SPI_Reg_t*)spix)
			__SPI1_PCLK_DIS();
		else if ((volatile SPI_Reg_t*)SPI2_BASE_ADDR == (volatile SPI_Reg_t*)spix)
			__SPI2_PCLK_DIS();
		else if ((volatile SPI_Reg_t*)SPI3_BASE_ADDR == (volatile SPI_Reg_t*)spix)
			__SPI3_PCLK_DIS();
	}
	return true;
}

void spi_drv_TXEInterruptHandler(SPI_Handle_t *self)
{
	if (_SPI_DFF_8BITS == self->config.dff)
	{
		self->spix->DR = *self->tx.buffer;
		self->tx.len--;
		self->tx.buffer++;
	}
	else
	{
		self->spix->DR = *((uint16_t*)self->tx.buffer);
		self->tx.len-= 2;
		(uint16_t*)self->tx.buffer++;
	}

	if (!self->tx.len)
	{
		self->spix->CR2 &= ~(1 << SPI_CR2_TXEIE);
		self->tx.buffer = NULL;
		self->tx.state = _SPI_IRQ_STATE_READY;
		if (NULL != self->irq_event)
			self->irq_event((void*)self, _SPI_IRQ_EVENT_TX_CMPLT);
	}
}

void spi_drv_RXNEInterruptHandler(SPI_Handle_t *self)
{
	if (_SPI_DFF_8BITS == self->config.dff)
	{
		*self->rx.buffer = self->spix->DR;
		self->rx.len--;
		self->rx.buffer++;
	}
	else
	{
		*((uint16_t*)self->rx.buffer) = self->spix->DR;
		self->rx.len-= 2;
		(uint16_t*)self->rx.buffer++;
	}

	if (!self->rx.len)
	{
		self->spix->CR2 &= ~(1 << SPI_CR2_RXNEIE);
		self->rx.buffer = NULL;
		self->rx.state = _SPI_IRQ_STATE_READY;
		if (NULL != self->irq_event)
			self->irq_event((void*)self, _SPI_IRQ_EVENT_TX_CMPLT);
	}
}

void spi_drv_MODFInterruptHandler(SPI_Handle_t *self)
{
	uint32_t dummy = self->spix->SR;
	(void)dummy;

	/* Set config */
	self->spix->CR1 |= (self->config.mode << SPI_CR1_MSTR);
	if (self->config.mode && !self->config.ssm)
		self->spix->CR2 |= (1 << SPI_CR2_SSOE);

	if (_SPI_BUS_CONFIG_FULL_DULEX == self->config.bus_config)
	{
		self->spix->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (_SPI_BUS_CONFIG_HALF_DULEX == self->config.bus_config)
	{
		self->spix->CR1 |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (_SPI_BUS_CONFIG_SIMPLEX_RONLY == self->config.bus_config)
	{
		self->spix->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
		self->spix->CR1 |= (1 << SPI_CR1_RXONLY);
	}
	self->spix->CR1 |= (self->config.speed << SPI_CR1_BR);
	self->spix->CR1 |= (self->config.first_bit << SPI_CR1_LSBFIRST);
	self->spix->CR1 |= (self->config.dff << SPI_CR1_DFF);
	self->spix->CR1 |= (self->config.cpol << SPI_CR1_CPOL);
	self->spix->CR1 |= (self->config.cpha << SPI_CR1_CPHA);

	self->spix->CR1 |= (self->config.ssm << SPI_CR1_SSM);
	if (self->config.ssm)
	{
		self->spix->CR1 |= (1 << SPI_CR1_SSI);
	}

	if (self->config.is_irq_enable)
	{
		spi_drv_SetInterrupts(self, self->config.irq_priority, self->irq_event);
	}

	if (NULL != self->irq_event)
		self->irq_event((void*)self, _SPI_IRQ_EVENT_MODF_ERR);
}

void spi_drv_OVRInterruptHandler(SPI_Handle_t *self)
{
	uint32_t dummy = self->spix->DR;
	dummy = self->spix->SR;
	(void)dummy;
	if (NULL != self->irq_event)
		self->irq_event((void*)self, _SPI_IRQ_EVENT_OVR_ERR);
}

void spi_drv_CRCERRInterruptHandler(SPI_Handle_t *self)
{
	if (NULL != self->irq_event)
		self->irq_event((void*)self, _SPI_IRQ_EVENT_CRC_ERR);
}

void spi_drv_FREInterruptHandler(SPI_Handle_t *self)
{
	uint32_t dummy = self->spix->SR;
	(void)dummy;
	if (NULL != self->irq_event)
		self->irq_event((void*)self, _SPI_IRQ_EVENT_CRC_ERR);
}

void spi_drv_IRQHandler(SPI_Handle_t *self)
{
	if ((self->spix->SR & (1 << SPI_SR_TXE)) && (self->spix->CR2 & (1 << SPI_CR2_TXEIE)))
	{
		spi_drv_TXEInterruptHandler(self);
	}

	if ((self->spix->SR & (1 << SPI_SR_RXNE)) && (self->spix->CR2 & (1 << SPI_CR2_RXNEIE)))
	{
		spi_drv_RXNEInterruptHandler(self);
	}

	// Checking Errors

	/* Master mode fault event */
	if ((self->spix->SR & (1 << SPI_SR_MODF)) && (self->spix->CR2 & (1 << SPI_CR2_ERRIE)))
	{
		spi_drv_MODFInterruptHandler(self);
	}

	/* Overrun error */
	if ((self->spix->SR & (1 << SPI_SR_OVR)) && (self->spix->CR2 & (1 << SPI_CR2_ERRIE)))
	{
		spi_drv_OVRInterruptHandler(self);
	}

	/* CRC error */
	if ((self->spix->SR & (1 << SPI_SR_CRCERR)) && (self->spix->CR2 & (1 << SPI_CR2_ERRIE)))
	{
		spi_drv_CRCERRInterruptHandler(self);
	}

	/* TI frame format error */
	if ((self->spix->SR & (1 << SPI_SR_FRE)) && (self->spix->CR2 & (1 << SPI_CR2_ERRIE)))
	{
		spi_drv_FREInterruptHandler(self);
	}
}

bool spi_drv_GetFlagStatus(SPI_Reg_t *spix, uint32_t flag)
{
	return ((spix->SR & flag) == flag);
}

bool spi_drv_DeInit(SPI_Handle_t *self)
{
	if (NULL == self->spix)
		return false;

	if (self->config.is_irq_enable)
	{
		/* Disable Interrupts */
		uint8_t irq_number = 0;
		if ((volatile SPI_Reg_t *)SPI1_BASE_ADDR == (volatile SPI_Reg_t *)self->spix)
			irq_number = _SPI_IRQ_NO_1;
		else if ((volatile SPI_Reg_t *)SPI2_BASE_ADDR == (volatile SPI_Reg_t *)self->spix)
			irq_number = _SPI_IRQ_NO_2;
		else if ((volatile SPI_Reg_t *)SPI3_BASE_ADDR == (volatile SPI_Reg_t *)self->spix)
			irq_number = _SPI_IRQ_NO_3;
		else
			return false;

		/* Disable NVIC IRQ*/
		volatile uint32_t *arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E104;
		*arm_nvic_iser_base_addr &= ~(1 << (irq_number % 32));

		uint8_t iprx = irq_number / 4;
		uint8_t iprx_section = irq_number % 4;
		uint8_t shift_amount = (8 * iprx_section) + 4;
		volatile uint32_t *arm_nvic_pr_base_addr = (volatile uint32_t *)(0xE000E400 + (iprx * 4));
		*arm_nvic_pr_base_addr &= ~(self->config.irq_priority << shift_amount);

		self->spix->CR2 &= ~(1 << SPI_CR2_ERRIE);
		self->spix->CR2 &= ~(1 << SPI_CR2_TXEIE);
		self->spix->CR2 &= ~(1 << SPI_CR2_RXNEIE);

		self->irq_event = NULL;
	}

	/* Disable config */
	self->spix->CR1 = 0x0;
	self->spix->CR2 = 0x0;

	spi_drv_PeripheralClockControl(self->spix, false);

	memset(self, 0, sizeof(*self));

	return true;
}

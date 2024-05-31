/*
 * spi_drv.c
 *
 *  Created on: May 30, 2024
 *      Author: nesh
 */

#include "spi_drv.h"

bool spi_drv_PeripheralClockControl(SPI_Reg_t *self, bool state);

bool spi_drv_GetFlagStatus(SPI_Reg_t *self, uint32_t flag);

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
    spix->CR1 |= (self->config.mode << SPI_CR1_MSTR);
    if (self->config.mode && !self->config.ssm)
        self->spix->CR2 |= (1 << SPI_CR2_SSOE);

    if (_SPI_BUS_CONFIG_FULL_DULEX == self->config.bus_config)
    {
    	spix->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
    }
    else if (_SPI_BUS_CONFIG_HALF_DULEX == self->config.bus_config)
    {
        spix->CR1 |= (1 << SPI_CR1_BIDIMODE);
    }
    else if (_SPI_BUS_CONFIG_SIMPLEX_RONLY == self->config.bus_config)
    {
        spix->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
        spix->CR1 |= (1 << SPI_CR1_RXONLY);
    }
    spix->CR1 |= (self->config.speed << SPI_CR1_BR);
    spix->CR1 |= (self->config.first_bit << SPI_CR1_LSBFIRST);
    spix->CR1 |= (self->config.dff << SPI_CR1_DFF);
    spix->CR1 |= (self->config.cpol << SPI_CR1_CPOL);
    spix->CR1 |= (self->config.cpha << SPI_CR1_CPHA);

    self->spix->CR1 |= (self->config.ssm << SPI_CR1_SSM);
    if (self->config.ssm)
    {
        self->spix->CR1 |= (1 << SPI_CR1_SSI);
    }

	return true;
}

bool spi_drv_SendData(SPI_Handle_t *self, uint8_t *data, uint32_t data_len)
{
    if (!data_len)
    	return false;

	spi_drv_PeripheralControl(self, true);

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

	spi_drv_PeripheralControl(self, false);

    return true;
}

bool spi_drv_ReceiveData(SPI_Handle_t *self, uint8_t *data, uint32_t data_len)
{
    if (!data_len)
    	return false;

	spi_drv_PeripheralControl(self, true);
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

	spi_drv_PeripheralControl(self, false);

    return true;
}

void spi_drv_PeripheralControl(SPI_Handle_t *self, bool state)
{
	if (state)
		self->spix->CR1 |= (1 << SPI_CR1_SPE);
	else
		self->spix->CR1 &= ~(1 << SPI_CR1_SPE);
}

bool spi_drv_PeripheralClockControl(SPI_Reg_t *self, bool state)
{
	if (!(((volatile SPI_Reg_t*)SPI1_BASE_ADDR == (volatile SPI_Reg_t*)self) ||
		  ((volatile SPI_Reg_t*)SPI2_BASE_ADDR == (volatile SPI_Reg_t*)self) ||
		  ((volatile SPI_Reg_t*)SPI3_BASE_ADDR == (volatile SPI_Reg_t*)self)))
		return false;
	if (state)
	{
		if ((volatile SPI_Reg_t*)SPI1_BASE_ADDR == (volatile SPI_Reg_t*)self)
			__SPI1_PCLK_EN();
		else if ((volatile SPI_Reg_t*)SPI2_BASE_ADDR == (volatile SPI_Reg_t*)self)
			__SPI2_PCLK_EN();
		else if ((volatile SPI_Reg_t*)SPI3_BASE_ADDR == (volatile SPI_Reg_t*)self)
			__SPI3_PCLK_EN();
	}
	else
	{
		if ((volatile SPI_Reg_t*)SPI1_BASE_ADDR == (volatile SPI_Reg_t*)self)
			__SPI1_PCLK_DIS();
		else if ((volatile SPI_Reg_t*)SPI2_BASE_ADDR == (volatile SPI_Reg_t*)self)
			__SPI2_PCLK_DIS();
		else if ((volatile SPI_Reg_t*)SPI3_BASE_ADDR == (volatile SPI_Reg_t*)self)
			__SPI3_PCLK_DIS();
	}
	return true;
}

bool spi_drv_GetFlagStatus(SPI_Reg_t *self, uint32_t flag)
{
    return ((self->SR & flag) == flag);
}

bool spi_drv_DeInit(SPI_Handle_t *self)
{
	return true;
}

/*
 * i2c_drv.c
 *
 *  Created on: Jun 2, 2024
 *      Author: nesh
 */

#include "i2c_drv.h"


/*
 * I2C error interrupt handling
 * I2C event interrupt handling
 * */

/**
 * @brief I2C Peripheral Clock Control
 * @param self I2C handle base address
 * @param state Peripheral clock enabling control, true: enable
 * @return bool When the operation is successfully; return true 
 */
bool i2c_drv_PeripheralClockControl(I2C_Reg_t *i2cx, bool state);

void i2c_drv_GenerateStartCondition(I2C_Reg_t *i2cx);

void i2c_drv_SendAddress(I2C_Reg_t *i2cx, uint8_t slave_addr, I2C_Addr_operation_t operation);

void i2c_drv_ControlACK(I2C_Reg_t *i2cx, I2C_Ack_state_t state);

void i2c_drv_ClearAddrFlag(I2C_Reg_t *i2cx);

void i2c_drv_GenerateStopCondition(I2C_Reg_t *i2cx);

bool i2c_drv_GetFlagStatus(I2C_Reg_t *i2cx, uint32_t flag);

bool i2c_drv_Init(I2C_Handle_t *self, I2C_Reg_t *i2cx, I2C_config_t config)
{
	if (NULL == self)
		return false;
	if (!(((volatile I2C_Reg_t*)I2C1_BASE_ADDR == (volatile I2C_Reg_t*)i2cx) ||
		  ((volatile I2C_Reg_t*)I2C2_BASE_ADDR == (volatile I2C_Reg_t*)i2cx) ||
		  ((volatile I2C_Reg_t*)I2C3_BASE_ADDR == (volatile I2C_Reg_t*)i2cx)))
		return false;

    i2c_drv_PeripheralClockControl(i2cx, true);

	memset(self, 0, sizeof(*self));
	self->config = config;
	self->i2cx = i2cx;

	/* Set config */

    // Set enable to peripheral
    self->i2cx->CR1 |= (1 << I2C_CR1_PE);

    // Set ack configuration
    self->i2cx->CR1 |= (self->config.ack_control << I2C_CR1_ACK);

    // Set freq configuration
    uint32_t apb1_speed_mhz = __RCC_GET_APB1_SPEED() / 1000000;
    self->i2cx->CR2 |= (apb1_speed_mhz & 0x3F);

    // Set own address configuration
    self->i2cx->OAR1 |= (self->config.device_addr & 0x7F) << I2C_OAR1_ADD7_1;

    // Set Clock control register configuration
    uint16_t ccr_val = 0;
    uint8_t trise = 0;
    if (self->config.scl_speed > 100000)
    {
        self->i2cx->CCR |= (1 << I2C_CCR_F_S);
        if (self->config.fm_duty_cycle)
            ccr_val = __RCC_GET_APB1_SPEED() / (25 * self->config.scl_speed);
        else
            ccr_val = __RCC_GET_APB1_SPEED() / (3 * self->config.scl_speed);
        trise = ((apb1_speed_mhz * 300) / 1000) + 1;
    }
    else
    {
        ccr_val = __RCC_GET_APB1_SPEED() / (2 * self->config.scl_speed);
        trise = apb1_speed_mhz + 1;
    }
    self->i2cx->CCR |= (self->config.fm_duty_cycle << I2C_CCR_DUTY);
    self->i2cx->CCR |= (ccr_val & 0x0FFF);

    // Set Trise configuration
    self->i2cx->TRISE |= (trise & 0x3F);

    return true;
}

bool i2c_drv_SetInterrupts(I2C_Handle_t *self)
{
    return true;
}

bool i2c_drv_MasterSendData(I2C_Handle_t *self, uint8_t slave_addr, uint8_t *data, uint32_t data_len)
{
	if ((NULL == self) || (NULL == data))
        return false;

    // Start condition generating
    i2c_drv_GenerateStartCondition(self->i2cx);

    // Start Bit checking
    while (!i2c_drv_GetFlagStatus(self->i2cx, I2C_SR1_SB_FLAG));

    // Send slave address
    i2c_drv_SendAddress(self->i2cx, slave_addr, _I2C_ADDR_OPERATION_WRITE);

    // Address sent bit checking
    while (!i2c_drv_GetFlagStatus(self->i2cx, I2C_SR1_ADDR_FLAG));

    // Clear the ADDR flag
    i2c_drv_ClearAddrFlag(self->i2cx);

    // Send the data
    while (data_len)
    {
        // Check TXE flag
        while (!i2c_drv_GetFlagStatus(self->i2cx, I2C_SR1_TXE_FLAG));
        self->i2cx->DR = *data;
        data++;
        data_len--;
    }

    // Check TXE flag
    while (!i2c_drv_GetFlagStatus(self->i2cx, I2C_SR1_TXE_FLAG));
    
    // Check Byte transfer finished flag
    while (!i2c_drv_GetFlagStatus(self->i2cx, I2C_SR1_BTF_FLAG));

    // Stop condition generating
    i2c_drv_GenerateStopCondition(self->i2cx);

    return true;
}

bool i2c_drv_MasterReceiveData(I2C_Handle_t *self, uint8_t slave_addr, uint8_t *data, uint32_t data_len)
{
    if ((NULL == self) || (NULL == data))
        return false;

    // Start condition generating
    i2c_drv_GenerateStartCondition(self->i2cx);

    // Start Bit checking
    while (!i2c_drv_GetFlagStatus(self->i2cx, I2C_SR1_SB_FLAG));

    // Send slave address 
    i2c_drv_SendAddress(self->i2cx, slave_addr, _I2C_ADDR_OPERATION_READ);

    // Address sent bit checking
    while (!i2c_drv_GetFlagStatus(self->i2cx, I2C_SR1_ADDR_FLAG));

    while (data_len)
    {
        if (1 == data_len)
        {
            // Disable ACK
            i2c_drv_ControlACK(self->i2cx, _I2C_ACK_DISABLE);
        }

        // Clear the ADDR flag
        i2c_drv_ClearAddrFlag(self->i2cx);

        // Check RXNE flag
        while (!i2c_drv_GetFlagStatus(self->i2cx, I2C_SR1_RXNE_FLAG));

        *data = self->i2cx->DR;
        data_len--;
        data++;
    }

    // Stop condition generating
    i2c_drv_GenerateStopCondition(self->i2cx);

    // Set default state to ACK
    i2c_drv_ControlACK(self->i2cx, self->config.ack_control);

    return true;
}

bool i2c_drv_SlaveTransmit(I2C_Handle_t *self)
{
    return true;
}

bool i2c_drv_SlaveReceive(I2C_Handle_t *self)
{
    return true;
}

void i2c_drv_PeripheralControl(I2C_Handle_t *self, bool state)
{
	if (state);
		// self->spix->CR1 |= (1 << SPI_CR1_SPE);
	else
    ;
		// self->spix->CR1 &= ~(1 << SPI_CR1_SPE);
}

bool i2c_drv_PeripheralClockControl(I2C_Reg_t *i2cx, bool state)
{
	if (!(((volatile I2C_Handle_t*)I2C1_BASE_ADDR == (volatile I2C_Handle_t*)i2cx) ||
		  ((volatile I2C_Handle_t*)I2C2_BASE_ADDR == (volatile I2C_Handle_t*)i2cx) ||
		  ((volatile I2C_Handle_t*)I2C3_BASE_ADDR == (volatile I2C_Handle_t*)i2cx)))
		return false;
	if (state)
	{
		if ((volatile SPI_Reg_t*)I2C1_BASE_ADDR == (volatile SPI_Reg_t*)i2cx)
			__I2C1_PCLK_EN();
		else if ((volatile SPI_Reg_t*)I2C2_BASE_ADDR == (volatile SPI_Reg_t*)i2cx)
			__I2C2_PCLK_EN();
		else if ((volatile SPI_Reg_t*)I2C3_BASE_ADDR == (volatile SPI_Reg_t*)i2cx)
			__I2C3_PCLK_EN();
	}
	else
	{
		if ((volatile SPI_Reg_t*)I2C1_BASE_ADDR == (volatile SPI_Reg_t*)i2cx)
			__I2C1_PCLK_DIS();
		else if ((volatile SPI_Reg_t*)I2C2_BASE_ADDR == (volatile SPI_Reg_t*)i2cx)
			__I2C2_PCLK_DIS();
		else if ((volatile SPI_Reg_t*)I2C3_BASE_ADDR == (volatile SPI_Reg_t*)i2cx)
			__I2C3_PCLK_DIS();
	}
	return true;
}

bool i2c_drv_GetFlagStatus(I2C_Reg_t *i2cx, uint32_t flag)
{
	return ((i2cx->SR1 & flag) == flag);
}

void i2c_drv_GenerateStartCondition(I2C_Reg_t *i2cx)
{
    i2cx->CR1 |= (1 << I2C_CR1_START);
}

void i2c_drv_SendAddress(I2C_Reg_t *i2cx, uint8_t slave_addr, I2C_Addr_operation_t operation)
{
    slave_addr <<= 1;
    slave_addr |= operation;
    i2cx->DR = slave_addr;
}

void i2c_drv_ControlACK(I2C_Reg_t *i2cx, I2C_Ack_state_t state)
{
    if (state)
    {
        i2cx->CR1 |= (1 << I2C_CR1_ACK);
        return;
    }
    i2cx->CR1 &= ~(1 << I2C_CR1_ACK);
}

void i2c_drv_ClearAddrFlag(I2C_Reg_t *i2cx)
{
    uint32_t dumy = i2cx->SR1;
    dumy = i2cx->SR2;
    (void)dumy;
}

void i2c_drv_GenerateStopCondition(I2C_Reg_t *i2cx)
{
    i2cx->CR1 |= (1 << I2C_CR1_STOP);
}

bool i2c_drv_DeInit(I2C_Handle_t *self)
{
    return true;
}


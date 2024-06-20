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

void i2c_drv_ClearAddrFlag(I2C_Handle_t *self);

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

bool i2c_drv_SetInterrupts(I2C_Handle_t *self, uint8_t irq_priority, void (*irq_event)(void *self, uint8_t event))
{
	if (NULL == self->i2cx)
		return false;

	uint8_t irq_ev_number = 0;
	uint8_t irq_err_number = 0;

	if ((volatile I2C_Reg_t *)I2C1_BASE_ADDR == (volatile I2C_Reg_t *)self->i2cx)
    {
        irq_ev_number = _I2C_IRQ_NO_1_EV;
        irq_err_number = _I2C_IRQ_NO_1_ER;
    }
	else if ((volatile I2C_Reg_t *)I2C2_BASE_ADDR == (volatile I2C_Reg_t *)self->i2cx)
    {
        irq_ev_number = _I2C_IRQ_NO_2_EV;
        irq_err_number = _I2C_IRQ_NO_2_ER;
    }
	else if ((volatile I2C_Reg_t *)I2C3_BASE_ADDR == (volatile I2C_Reg_t *)self->i2cx)
    {
        irq_ev_number = _I2C_IRQ_NO_3_EV;
        irq_err_number = _I2C_IRQ_NO_3_ER;
    }
	else
		return false;

	self->config.irq_priority = irq_priority;
	self->config.is_irq_enable = true;

    // Enable ITERREN control bit
    self->i2cx->CR2 |= (1 << I2C_CR2_ITERREN);

	if (NULL != irq_event)
		self->irq_event = irq_event;

    /* Enable events */
    volatile uint32_t *arm_nvic_iser_base_addr;
    if (irq_ev_number < 32)
    {
    	arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E100;
        *arm_nvic_iser_base_addr |= (1 << irq_ev_number);
    }
    else if ((irq_ev_number >= 32) && (irq_ev_number < 64))
    {
    	arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E104;
        *arm_nvic_iser_base_addr |= (1 << (irq_ev_number % 32));
    }
    else
    {
    	arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E108;
        *arm_nvic_iser_base_addr |= (1 << (irq_ev_number % 64));
    }

	uint8_t iprx = irq_ev_number / 4;
	uint8_t iprx_section = irq_ev_number % 4;
	uint8_t shift_amount = (8 * iprx_section) + 4;
	volatile uint32_t *arm_nvic_pr_base_addr = (volatile uint32_t *)(0xE000E400 + (iprx * 4));
	*arm_nvic_pr_base_addr |= (irq_priority << shift_amount);

    /* Enable errors */
    if (irq_err_number < 32)
    {
    	arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E100;
        *arm_nvic_iser_base_addr |= (1 << irq_err_number);
    }
    else if ((irq_err_number >= 32) && (irq_err_number < 64))
    {
    	arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E104;
        *arm_nvic_iser_base_addr |= (1 << (irq_err_number % 32));
    }
    else
    {
    	arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E108;
        *arm_nvic_iser_base_addr |= (1 << (irq_err_number % 64));
    }

	iprx = irq_err_number / 4;
	iprx_section = irq_err_number % 4;
	shift_amount = (8 * iprx_section) + 4;
    arm_nvic_pr_base_addr = (volatile uint32_t *)(0xE000E400 + (iprx * 4));
    *arm_nvic_pr_base_addr |= (irq_priority << shift_amount);
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
    i2c_drv_ClearAddrFlag(self);

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

bool i2c_drv_MasterSendDataIT(I2C_Handle_t *self, uint8_t slave_addr, uint8_t *data, uint32_t data_len)
{
	if ((NULL == self) || (NULL == data))
        return false;

    if (_I2C_IRQ_STATE_READY == self->state)
    {
        self->state = _I2C_IRQ_STATE_BUSY_IN_TX;
        self->tx.buffer = data;
        self->tx.len = data_len;
        self->slave_addr = slave_addr;

        // Start condition generating
        i2c_drv_GenerateStartCondition(self->i2cx);

        // Enable ITBUFEN control bit
        self->i2cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        // Enable ITEVTEN control bit
        self->i2cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        return true;
    }
    return false;
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

    if (1 == data_len)
    {
        // Disable ACK
        i2c_drv_ControlACK(self->i2cx, _I2C_ACK_DISABLE);
    }
    while (data_len)
    {
        if (2 == data_len)
        {
            // Disable ACK
            i2c_drv_ControlACK(self->i2cx, _I2C_ACK_DISABLE);
        }
        // Clear the ADDR flag
        i2c_drv_ClearAddrFlag(self);

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

bool i2c_drv_MasterReceiveDataIT(I2C_Handle_t *self, uint8_t slave_addr, uint8_t *data, uint32_t data_len)
{
	if ((NULL == self) || (NULL == data))
        return false;

    if (_I2C_IRQ_STATE_READY == self->state)
    {
        self->state = _I2C_IRQ_STATE_BUSY_IN_RX;
        self->rx.buffer = data;
        self->rx.len = data_len;
        self->slave_addr = slave_addr;

        // Start condition generating
        i2c_drv_GenerateStartCondition(self->i2cx);

        // Enable ITBUFEN control bit
        self->i2cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        // Enable ITEVTEN control bit
        self->i2cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        return true;
    }
    return false;
}

bool i2c_drv_SlaveTransmit(I2C_Handle_t *self)
{
    return true;
}

bool i2c_drv_SlaveReceive(I2C_Handle_t *self)
{
    return true;
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

void i2c_drv_SBInterruptHandler(I2C_Handle_t *self)
{
    // Send slave address
    if (_I2C_IRQ_STATE_BUSY_IN_TX == self->state)
        i2c_drv_SendAddress(self->i2cx, self->slave_addr, _I2C_ADDR_OPERATION_WRITE);
    else
        i2c_drv_SendAddress(self->i2cx, self->slave_addr, _I2C_ADDR_OPERATION_READ);
}

void i2c_drv_ADDRInterruptHandler(I2C_Handle_t *self)
{
    // Clear the ADDR flag
    i2c_drv_ClearAddrFlag(self);
}

void i2c_drv_BTFInterruptHandler(I2C_Handle_t *self)
{
    if (_I2C_IRQ_STATE_BUSY_IN_TX == self->state)
    {
        if (0x00 == self->tx.len)
        {
            // Check TXE event
            if (self->i2cx->SR1 & I2C_SR1_TXE_FLAG)
            {
                // Stop condition generating
                i2c_drv_GenerateStopCondition(self->i2cx);

                self->state = _I2C_IRQ_STATE_READY;
                self->tx.buffer = NULL;
                self->slave_addr = 0x00;

                if (NULL != self->irq_event)
                    self->irq_event((void*)self, _I2C_IRQ_EVENT_TX_CMPLT);
            }
        }
    }
}

void i2c_drv_STOPFInterruptHandler(I2C_Handle_t *self)
{
    // Clear the STOPF
    self->i2cx->CR1 |= 0x0000;

    if (NULL != self->irq_event)
        self->irq_event((void*)self, _I2C_IRQ_EVENT_STOP);
}

void i2c_drv_TXEInterruptHandler(I2C_Handle_t *self)
{
    if (self->i2cx->SR2 & I2C_SR2_MSL_FLAG)
    {
        if (_I2C_IRQ_STATE_BUSY_IN_TX == self->state)
        {
            if (self->tx.len > 0)
            {
                self->i2cx->DR = *self->tx.buffer;
                self->tx.buffer++;
                self->tx.len--;
            }
        }
    }
}

void i2c_drv_RXNEInterruptHandler(I2C_Handle_t *self)
{
    if (self->i2cx->SR2 & I2C_SR2_MSL_FLAG)
    {
        if (_I2C_IRQ_STATE_BUSY_IN_RX == self->state)
        {
            if (self->rx.len > 0)
            {
                if (2 == self->rx.len)
                {
                    // Disable ACK
                    i2c_drv_ControlACK(self->i2cx, _I2C_ACK_DISABLE);
                }
                *self->rx.buffer = self->i2cx->DR;
                self->rx.buffer++;
                self->rx.len--;
                if (0 == self->rx.len)
                {
                    // Stop condition generating
                    i2c_drv_GenerateStopCondition(self->i2cx);

                    // Disable ITBUFEN control bit
                    self->i2cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

                    // Disable ITEVTEN control bit
                    self->i2cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

                    self->state = _I2C_IRQ_STATE_READY;
                    self->rx.buffer = NULL;
                    self->slave_addr = 0x00;

                    // Set default state to ACK
                    i2c_drv_ControlACK(self->i2cx, self->config.ack_control);

                    if (NULL != self->irq_event)
                        self->irq_event((void*)self, _I2C_IRQ_EVENT_RX_CMPLT);
                }
            }
        }
    }
}

void i2c_drv_EventIRQHandler(I2C_Handle_t *self)
{
    // Check ITEVTEN control bit
    if (self->i2cx->CR2 & (1 << I2C_CR2_ITEVTEN))
    {
        // Check SB event (SB flag is only active in master mode)
        if (self->i2cx->SR1 & I2C_SR1_SB_FLAG)
        {
            i2c_drv_SBInterruptHandler(self);
        }

        // Check ADDR event
        if (self->i2cx->SR1 & I2C_SR1_ADDR_FLAG)
        {
            i2c_drv_ADDRInterruptHandler(self);
        }

        // Check BTF event
        if (self->i2cx->SR1 & I2C_SR1_BTF_FLAG)
        {
            i2c_drv_BTFInterruptHandler(self);
        }

        // Check STOPF event
        if (self->i2cx->SR1 & I2C_SR1_STOPF_FLAG)
        {
            i2c_drv_STOPFInterruptHandler(self);
        }

        // Check ITBUFEN control bit
        if (self->i2cx->CR2 & (1 << I2C_CR2_ITBUFEN))
        {
            // Check TXE event
            if (self->i2cx->SR1 & I2C_SR1_TXE_FLAG)
            {
                i2c_drv_TXEInterruptHandler(self);
            }

            // Check RXNE event
            if (self->i2cx->SR1 & I2C_SR1_RXNE_FLAG)
            {
                i2c_drv_RXNEInterruptHandler(self);
            }
        }
    }
}

void i2c_drv_BERRInterruptHandler(I2C_Handle_t *self)
{
    self->i2cx->SR1 &= ~(0x0000FFFF & I2C_SR1_BERR_FLAG);

    if (NULL != self->irq_event)
        self->irq_event((void*)self, _I2C_IRQ_EVENT_BERR);
}

void i2c_drv_ARLOInterruptHandler(I2C_Handle_t *self)
{
    self->i2cx->SR1 &= ~(0x0000FFFF & I2C_SR1_ARLO_FLAG);

    if (NULL != self->irq_event)
        self->irq_event((void*)self, _I2C_IRQ_EVENT_ARLO);
}

void i2c_drv_AFInterruptHandler(I2C_Handle_t *self)
{
    self->i2cx->SR1 &= ~(0x0000FFFF & I2C_SR1_AF_FLAG);

    if (NULL != self->irq_event)
        self->irq_event((void*)self, _I2C_IRQ_EVENT_AF);
}

void i2c_drv_OVRInterruptHandler(I2C_Handle_t *self)
{
    self->i2cx->SR1 &= ~(0x0000FFFF & I2C_SR1_OVR_FLAG);

    if (NULL != self->irq_event)
        self->irq_event((void*)self, _I2C_IRQ_EVENT_OVR);
}

void i2c_drv_PECERRInterruptHandler(I2C_Handle_t *self)
{
    self->i2cx->SR1 &= ~(0x0000FFFF & I2C_SR1_PECERR_FLAG);

    if (NULL != self->irq_event)
        self->irq_event((void*)self, _I2C_IRQ_EVENT_PECERR);
}

void i2c_drv_TIMEOUTInterruptHandler(I2C_Handle_t *self)
{
    self->i2cx->SR1 &= ~(0x0000FFFF & I2C_SR1_TIMEOUT_FLAG);

    if (NULL != self->irq_event)
        self->irq_event((void*)self, _I2C_IRQ_EVENT_TIMEOUT);
}

void i2c_drv_ErrorIRQHandler(I2C_Handle_t *self)
{
    // Check ITERREN control bit
    if (self->i2cx->CR2 & (1 << I2C_CR2_ITERREN))
    {
        // Check Bus Error 
        if (self->i2cx->SR1 & I2C_SR1_BERR_FLAG)
        {
            i2c_drv_BERRInterruptHandler(self);
        }

        // Check Arbitration lost Error 
        if (self->i2cx->SR1 & I2C_SR1_ARLO_FLAG)
        {
            i2c_drv_ARLOInterruptHandler(self);
        }

        // Check Acknowledge failure Error 
        if (self->i2cx->SR1 & I2C_SR1_AF_FLAG)
        {
            i2c_drv_AFInterruptHandler(self);
        }

        // Check Overrun or Underrun Error 
        if (self->i2cx->SR1 & I2C_SR1_OVR_FLAG)
        {
            i2c_drv_OVRInterruptHandler(self);
        }

        // Check PEC Error in reception Error 
        if (self->i2cx->SR1 & I2C_SR1_PECERR_FLAG)
        {
            i2c_drv_PECERRInterruptHandler(self);
        }

        // Check Timeout or Tlow error 
        if (self->i2cx->SR1 & I2C_SR1_TIMEOUT_FLAG)
        {
            i2c_drv_TIMEOUTInterruptHandler(self);
        }
    }
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

void i2c_drv_ClearAddrFlag(I2C_Handle_t *self)
{
    uint32_t dumy;

    // Check device mode
    if ((self->i2cx->SR2 & I2C_SR2_MSL_FLAG) && 
        (_I2C_IRQ_STATE_BUSY_IN_RX == self->state) &&
        (1 == self->rx.len))
        // Disable ACK
        i2c_drv_ControlACK(self->i2cx, _I2C_ACK_DISABLE);
    // Clear the ADDR Flag
    dumy = self->i2cx->SR1;
    dumy = self->i2cx->SR2;
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

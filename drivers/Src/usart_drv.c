/*
 * usart_drv.c
 *
 *  Created on: Jun 22, 2024
 *      Author: nesh
 */
#include "usart_drv.h"

/**
 * @brief USART Peripheral Clock Control
 * @param usartx USART peripheral handle base address
 * @param state Peripheral clock enabling control, true: enable
 * @return bool When the operation is successfully; return true 
 */
bool usart_drv_PeripheralClockControl(USART_Reg_t *usartx, bool state);

/**
 * @brief Transmission complete interrupt handling
 * @param self USART handle base address
 * @return
 */
void usart_drv_TCInterruptHandler(USART_Handle_t *self);

/**
 * @brief Transmit data register empty interrupt handling
 * @param self USART handle base address
 * @return
 */
void usart_drv_TXEInterruptHandler(USART_Handle_t *self);

/**
 * @brief Read data register not empty interrupt handling
 * @param self USART handle base address
 * @return
 */
void usart_drv_RXNEInterruptHandler(USART_Handle_t *self);

/**
 * @brief CTS flag is set interrupt handling
 * @param self USART handle base address
 * @return
 */
void usart_drv_CTSInterruptHandler(USART_Handle_t *self);

/**
 * @brief Overrun error interrupt handling
 * @param self USART handle base address
 * @return
 */
void usart_drv_OREInterruptHandler(USART_Handle_t *self);

/**
 * @brief IDLE line detected interrupt handling
 * @param self USART handle base address
 * @return
 */
void usart_drv_IDLEInterruptHandler(USART_Handle_t *self);

/**
 * @brief Parity error interrupt handling
 * @param self USART handle base address
 * @return
 */
void usart_drv_PEInterruptHandler(USART_Handle_t *self);

/**
 * @brief LIN break detection flag interrupt handling
 * @param self USART handle base address
 * @return
 */
void usart_drv_LBDInterruptHandler(USART_Handle_t *self);

/**
 * @brief Noise detected flag interrupt handling
 * @param self USART handle base address
 * @return
 */
void usart_drv_NFInterruptHandler(USART_Handle_t *self);

/**
 * @brief Framing error interrupt handling
 * @param self USART handle base address
 * @return
 */
void usart_drv_FEInterruptHandler(USART_Handle_t *self);

/**
 * @brief USART Check the flag is set on Status register
 * @param usartx USART handle base address
 * @param flag Interested flag
 * @return When the interested flag is set return true
 */
bool usart_drv_GetFlagStatus(USART_Reg_t *usartx, uint32_t flag);

bool usart_drv_Init(USART_Handle_t *self, USART_Reg_t *usartx, USART_config_t config)
{
    if (!(((volatile USART_Reg_t*)USART1_BASE_ADDR == (volatile USART_Reg_t*)usartx) ||
          ((volatile USART_Reg_t*)USART2_BASE_ADDR == (volatile USART_Reg_t*)usartx) ||
          ((volatile USART_Reg_t*)USART3_BASE_ADDR == (volatile USART_Reg_t*)usartx) ||
          ((volatile USART_Reg_t*)UART4_BASE_ADDR  == (volatile USART_Reg_t*)usartx) ||
          ((volatile USART_Reg_t*)UART5_BASE_ADDR  == (volatile USART_Reg_t*)usartx) ||
          ((volatile USART_Reg_t*)USART6_BASE_ADDR == (volatile USART_Reg_t*)usartx)))
    return false;

    usart_drv_PeripheralClockControl(usartx, true);

	memset(self, 0, sizeof(*self));
	self->config = config;
	self->usartx = usartx;

    /* Set config */
    if (_USART_MODE_ONLY_TX == self->config.mode)
    {
        self->usartx->CR1 &= ~(1 << USART_CR1_RE);
        self->usartx->CR1 |= (1 << USART_CR1_TE);
    }
    else if (_USART_MODE_ONLY_RX == self->config.mode)
    {
        self->usartx->CR1 &= ~(1 << USART_CR1_TE);
        self->usartx->CR1 |= (1 << USART_CR1_RE);
    }
    else if (_USART_MODE_TX_RX == self->config.mode)
        self->usartx->CR1 |= (1 << USART_CR1_TE) | (1 << USART_CR1_RE);
    else
    {
        self->usartx = NULL;
        return false;
    }

    uint32_t bus_speed;
    if (((volatile USART_Reg_t*)USART1_BASE_ADDR == (volatile USART_Reg_t*)usartx) ||
        ((volatile USART_Reg_t*)USART6_BASE_ADDR == (volatile USART_Reg_t*)usartx))
        bus_speed = __RCC_GET_APB2_SPEED();
    else
        bus_speed = __RCC_GET_APB1_SPEED();

    uint32_t brr = (bus_speed / self->config.baudrate);
    if (brr > 0xFFFF)
    {
        self->usartx = NULL;
        return false;
    }
    self->usartx->BRR |= (uint16_t)brr;

    if (((_USART_STOP_BITS_0_5 == self->config.nosb) ||
         (_USART_STOP_BITS_1_5 == self->config.nosb)) &&
        (((volatile USART_Reg_t*)UART4_BASE_ADDR  == (volatile USART_Reg_t*)usartx) ||
         ((volatile USART_Reg_t*)UART5_BASE_ADDR  == (volatile USART_Reg_t*)usartx)))
    {
        self->usartx = NULL;
        return false;
    }
    self->usartx->CR2 |= (self->config.nosb << USART_CR2_STOP);

    self->usartx->CR1 |= ((self->config.word_length & 0x1) << USART_CR1_M);

    if (_USART_PARITY_CONTROL_DIS != self->config.parity_control)
    {
        self->usartx->CR1 |= (1 << USART_CR1_PCE);
        bool ps;
        if (_USART_PARITY_CONTROL_EVEN == self->config.parity_control)
            ps = false;
        else if (_USART_PARITY_CONTROL_ODD == self->config.parity_control)
            ps = true;
        else
        {
            self->usartx = NULL;
            return false;
        }
        self->usartx->CR1 |= (ps << USART_CR1_PS);
    }

    if (_USART_HW_FLOW_CONTROL_NONE != self->config.hw_flow_control)
    {
        if (((volatile USART_Reg_t*)UART4_BASE_ADDR  == (volatile USART_Reg_t*)usartx) ||
            ((volatile USART_Reg_t*)UART5_BASE_ADDR  == (volatile USART_Reg_t*)usartx))
        {
            self->usartx = NULL;
            return false;
        }
        if (_USART_HW_FLOW_CONTROL_CTS == self->config.hw_flow_control)
        {
            self->usartx->CR3 &= ~(1 << USART_CR3_RTSE);
            self->usartx->CR3 |= (1 << USART_CR3_CTSE);
        }
        else if (_USART_HW_FLOW_CONTROL_CTS == self->config.hw_flow_control)
        {
            self->usartx->CR3 &= ~(1 << USART_CR3_CTSE);
            self->usartx->CR3 |= (1 << USART_CR3_RTSE);
        }
        else if (_USART_HW_FLOW_CONTROL_CTS_RTS == self->config.hw_flow_control)
        {
            self->usartx->CR3 |= (1 << USART_CR3_CTSE);
            self->usartx->CR3 |= (1 << USART_CR3_RTSE);
        }
    }

    self->usartx->CR1 |= (1 << USART_CR1_UE);
    return true;
}

bool usart_drv_SetInterrupts(USART_Handle_t *self, uint8_t irq_priority, void (*irq_event)(void *self, uint8_t event))
{
    if (NULL == self)
    {
        return false;
    }

	uint8_t irq_number = 0;

	if ((volatile I2C_Reg_t *)USART1_BASE_ADDR == (volatile I2C_Reg_t *)self->usartx)
        irq_number = _USART_IRQ_NO_1;
	else if ((volatile I2C_Reg_t *)USART2_BASE_ADDR == (volatile I2C_Reg_t *)self->usartx)
        irq_number = _USART_IRQ_NO_2;
	else if ((volatile I2C_Reg_t *)USART3_BASE_ADDR == (volatile I2C_Reg_t *)self->usartx)
        irq_number = _USART_IRQ_NO_3;
	else if ((volatile I2C_Reg_t *)UART4_BASE_ADDR == (volatile I2C_Reg_t *)self->usartx)
        irq_number = _USART_IRQ_NO_4;
	else if ((volatile I2C_Reg_t *)UART5_BASE_ADDR == (volatile I2C_Reg_t *)self->usartx)
        irq_number = _USART_IRQ_NO_5;
	else if ((volatile I2C_Reg_t *)USART6_BASE_ADDR == (volatile I2C_Reg_t *)self->usartx)
        irq_number = _USART_IRQ_NO_6;
	else
		return false;

    self->config.irq_priority = irq_priority;
    self->config.is_irq_enable = true;

    if ((_USART_HW_FLOW_CONTROL_CTS_RTS == self->config.hw_flow_control) ||
        (_USART_HW_FLOW_CONTROL_CTS == self->config.hw_flow_control))
    {
        // Enable CTSIE control bit
        self->usartx->CR3 |= ( 1 << USART_CR3_CTSIE);
    }

    // Enable IDLEIE control bit
    self->usartx->CR1 |= ( 1 << USART_CR1_IDLEIE);

    if (_USART_PARITY_CONTROL_DIS != self->config.parity_control)
    {
        // Enable PEIE control bit
        self->usartx->CR1 |= ( 1 << USART_CR1_PEIE);
    }

    // Enable EIE  control bit
    self->usartx->CR3 |= ( 1 << USART_CR3_EIE);

	if (NULL != irq_event)
		self->irq_event = irq_event;

    /* Enable events */
    volatile uint32_t *arm_nvic_iser_base_addr;
    if (irq_number < 32)
    {
    	arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E100;
        *arm_nvic_iser_base_addr |= (1 << irq_number);
    }
    else if ((irq_number >= 32) && (irq_number < 64))
    {
    	arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E104;
        *arm_nvic_iser_base_addr |= (1 << (irq_number % 32));
    }
    else
    {
    	arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E108;
        *arm_nvic_iser_base_addr |= (1 << (irq_number % 64));
    }

	uint8_t iprx = irq_number / 4;
	uint8_t iprx_section = irq_number % 4;
	uint8_t shift_amount = (8 * iprx_section) + 4;
	volatile uint32_t *arm_nvic_pr_base_addr = (volatile uint32_t *)(0xE000E400 + (iprx * 4));
	*arm_nvic_pr_base_addr |= (irq_priority << shift_amount);
	return true;
}

bool usart_drv_SendData(USART_Handle_t *self, uint8_t *data, uint32_t data_len)
{
    if ((NULL == self) || (NULL == data))
        return false;

    while (data_len)
    {
        while (!usart_drv_GetFlagStatus(self->usartx, USART_SR_TXE_FLAG));
        if (self->config.word_length)
        {
            uint16_t *dummy = (uint16_t*)data;
            self->usartx->DR = (*dummy & (uint16_t)0x01FF);
            if (_USART_PARITY_CONTROL_DIS == self->config.parity_control)
            {
                (uint16_t*)data++;
                data_len -= 2;
                continue;
            }
            data++;
            data_len--;
            continue;
        }
        self->usartx->DR = (*data & 0xFF);
        data_len--;
        data++;
    }
    while (usart_drv_GetFlagStatus(self->usartx, USART_SR_TC_FLAG));
    return true;
}

bool usart_drv_SendDataIT(USART_Handle_t *self, uint8_t *data, uint32_t data_len)
{
    if (_USART_IRQ_STATE_READY == self->state)
    {
        self->tx.buffer = data;
        self->tx.len = data_len;
        self->state = _USART_IRQ_STATE_BUSY_IN_TX;

        // Enable TXEIE control bit
        self->usartx->CR1 |= ( 1 << USART_CR1_TXEIE);

        // Enable TCIE control bit
        self->usartx->CR1 |= ( 1 << USART_CR1_TCIE);

        return true;
    }
    return false;
}

bool usart_drv_ReceiveData(USART_Handle_t *self, uint8_t *data, uint32_t data_len)
{
    if ((NULL == self) || (NULL == data))
        return false;

    while (data_len)
    {
        while (!usart_drv_GetFlagStatus(self->usartx, USART_SR_RXNE_FLAG));

        if (self->config.word_length)
        {
            if (_USART_PARITY_CONTROL_DIS == self->config.parity_control)
            {
                *(uint16_t*)data = (uint16_t)(self->usartx->DR & 0x01FF);
                (uint16_t*)data++;
                data_len -= 2;
                continue;
            }
            *data = (uint16_t)(self->usartx->DR & 0x01FF);
            data++;
            data_len--;
            continue;
        }
        if (_USART_PARITY_CONTROL_DIS == self->config.parity_control)
            *data = (uint8_t)(self->usartx->DR & 0x0FF);
        else
            *data = (uint8_t)(self->usartx->DR & 0x07F);
        data++;
        data_len--;
    }
    return true;
}

bool usart_drv_ReceiveDataIT(USART_Handle_t *self, uint8_t *data, uint32_t data_len)
{
    if (_USART_IRQ_STATE_READY == self->state)
    {
        self->rx.buffer = data;
        self->rx.len = data_len;
        self->state = _USART_IRQ_STATE_BUSY_IN_RX;

        // Enable RXNEIE control bit
        self->usartx->CR1 |= ( 1 << USART_CR1_RXNEIE);

        return true;
    }
    return false;
}

bool usart_drv_PeripheralClockControl(USART_Reg_t *usartx, bool state)
{
    if (!(((volatile USART_Reg_t*)USART1_BASE_ADDR == (volatile USART_Reg_t*)usartx) ||
          ((volatile USART_Reg_t*)USART2_BASE_ADDR == (volatile USART_Reg_t*)usartx) ||
          ((volatile USART_Reg_t*)USART3_BASE_ADDR == (volatile USART_Reg_t*)usartx) ||
          ((volatile USART_Reg_t*)UART4_BASE_ADDR  == (volatile USART_Reg_t*)usartx) ||
          ((volatile USART_Reg_t*)UART5_BASE_ADDR  == (volatile USART_Reg_t*)usartx) ||
          ((volatile USART_Reg_t*)USART6_BASE_ADDR == (volatile USART_Reg_t*)usartx)))
    return false;
	if (state)
	{
		if ((volatile USART_Reg_t*)USART1_BASE_ADDR == (volatile USART_Reg_t*)usartx)
			__USART1_PCLK_EN();
        else if ((volatile USART_Reg_t*)USART2_BASE_ADDR == (volatile USART_Reg_t*)usartx)
			__USART2_PCLK_EN();
        else if ((volatile USART_Reg_t*)USART3_BASE_ADDR == (volatile USART_Reg_t*)usartx)
			__USART3_PCLK_EN();
        else if ((volatile USART_Reg_t*)UART4_BASE_ADDR == (volatile USART_Reg_t*)usartx)
			__UART4_PCLK_EN();
        else if ((volatile USART_Reg_t*)UART5_BASE_ADDR == (volatile USART_Reg_t*)usartx)
			__UART5_PCLK_EN();
        else if ((volatile USART_Reg_t*)USART6_BASE_ADDR == (volatile USART_Reg_t*)usartx)
			__USART6_PCLK_EN();
	}
	else
	{
		if ((volatile USART_Reg_t*)USART1_BASE_ADDR == (volatile USART_Reg_t*)usartx)
			__USART1_PCLK_DIS();
        else if ((volatile USART_Reg_t*)USART2_BASE_ADDR == (volatile USART_Reg_t*)usartx)
			__USART2_PCLK_DIS();
        else if ((volatile USART_Reg_t*)USART3_BASE_ADDR == (volatile USART_Reg_t*)usartx)
			__USART3_PCLK_DIS();
        else if ((volatile USART_Reg_t*)UART4_BASE_ADDR == (volatile USART_Reg_t*)usartx)
			__UART4_PCLK_DIS();
        else if ((volatile USART_Reg_t*)UART5_BASE_ADDR == (volatile USART_Reg_t*)usartx)
			__UART5_PCLK_DIS();
        else if ((volatile USART_Reg_t*)USART6_BASE_ADDR == (volatile USART_Reg_t*)usartx)
			__USART6_PCLK_DIS();
	}
	return true;
}

void usart_drv_TCInterruptHandler(USART_Handle_t *self)
{
    if (_USART_IRQ_STATE_BUSY_IN_TX == self->state)
    {
        if (!self->tx.len)
        {
            // Disable TCIE control bit
            self->usartx->CR1 &= ~( 1 << USART_CR1_TCIE);

            self->tx.buffer = NULL;
            self->state = _USART_IRQ_STATE_READY;

            if (NULL != self->irq_event)
                self->irq_event(self, _USART_IRQ_EVENT_TX_CMPLT);
        }
    }
}

void usart_drv_TXEInterruptHandler(USART_Handle_t *self)
{
    if (_USART_IRQ_STATE_BUSY_IN_TX == self->state)
    {
        if (self->tx.len > 0)
        {
            if (self->config.word_length)
            {
                self->usartx->DR = (*(uint16_t*)self->tx.buffer & (uint16_t)0x01FF);
                if (_USART_PARITY_CONTROL_DIS == self->config.parity_control)
                {
                    (uint16_t*)self->tx.buffer++;
                    self->tx.len -= 2;
                    return;
                }
                self->tx.buffer++;
                self->tx.len--;
                return;
            }
            self->usartx->DR = (*self->tx.buffer & 0xFF);
            self->tx.len--;
            self->tx.buffer++;
            return;
        }
        // Disable TXEIE control bit
        self->usartx->CR1 &= ~( 1 << USART_CR1_TXEIE);
    }
}

void usart_drv_RXNEInterruptHandler(USART_Handle_t *self)
{
    if (_USART_IRQ_STATE_BUSY_IN_RX == self->state)
    {
        if (self->rx.len > 0)
        {
            if (self->config.word_length)
            {
                if (_USART_PARITY_CONTROL_DIS == self->config.parity_control)
                {
                    *(uint16_t*)self->rx.buffer = (uint16_t)(self->usartx->DR & 0x01FF);
                    (uint16_t*)self->rx.buffer++;
                    self->rx.len -= 2;
                    return;
                }
                *self->rx.buffer = (uint16_t)(self->usartx->DR & 0x01FF);
                self->rx.buffer++;
                self->rx.len--;
                return;
            }
            if (_USART_PARITY_CONTROL_DIS == self->config.parity_control)
                *self->rx.buffer = (uint8_t)(self->usartx->DR & 0x0FF);
            else
                *self->rx.buffer = (uint8_t)(self->usartx->DR & 0x07F);
            self->rx.buffer++;
            self->rx.len--;
            return;
        }
        // Disable RXNEIE control bit
        self->usartx->CR1 &= ~(1 << USART_CR1_RXNEIE);
        self->rx.buffer = NULL;
        self->state = _USART_IRQ_STATE_READY;

        if (NULL != self->irq_event)
            self->irq_event(self, _USART_IRQ_EVENT_RX_CMPLT);
    }
}

void usart_drv_CTSInterruptHandler(USART_Handle_t *self)
{

    self->usartx->SR &= ~(0x0000FFFF & USART_SR_CTS_FLAG);

	if (NULL != self->irq_event)
		self->irq_event(self, _USART_IRQ_EVENT_CTS_ERROR);
}

void usart_drv_OREInterruptHandler(USART_Handle_t *self)
{
    uint32_t dummy;
    dummy = self->usartx->SR;
    dummy = self->usartx->DR;
    (void)dummy;

	if (NULL != self->irq_event)
		self->irq_event(self, _USART_IRQ_EVENT_ORE_ERROR);
}

void usart_drv_IDLEInterruptHandler(USART_Handle_t *self)
{
    uint32_t dummy;
    dummy = self->usartx->SR;
    dummy = self->usartx->DR;
    (void)dummy;

	if (NULL != self->irq_event)
		self->irq_event(self, _USART_IRQ_EVENT_IDLE_ERROR);
}

void usart_drv_PEInterruptHandler(USART_Handle_t *self)
{
    uint32_t dummy;
    dummy = self->usartx->SR;
    dummy = self->usartx->DR;
    (void)dummy;

	if (NULL != self->irq_event)
		self->irq_event(self, _USART_IRQ_EVENT_PE_ERROR);
}

void usart_drv_LBDInterruptHandler(USART_Handle_t *self)
{
    uint32_t dummy;
    dummy = self->usartx->SR;
    dummy = self->usartx->DR;
    (void)dummy;

	if (NULL != self->irq_event)
		self->irq_event(self, _USART_IRQ_EVENT_LBD_ERROR);
}

void usart_drv_NFInterruptHandler(USART_Handle_t *self)
{
    self->usartx->SR &= ~(0x0000FFFF & USART_SR_LBD_FLAG);

	if (NULL != self->irq_event)
		self->irq_event(self, _USART_IRQ_EVENT_NF_ERROR);
}

void usart_drv_FEInterruptHandler(USART_Handle_t *self)
{
    uint32_t dummy;
    dummy = self->usartx->SR;
    dummy = self->usartx->DR;
    (void)dummy;

	if (NULL != self->irq_event)
		self->irq_event(self, _USART_IRQ_EVENT_FE_ERROR);
}

void usart_drv_IRQHandler(USART_Handle_t *self)
{
    // Check transmission complete interrupt
    if (((self->usartx->CR1 & (1 << USART_CR1_TCIE)) == (1 << USART_CR1_TCIE)) &&
        usart_drv_GetFlagStatus(self->usartx, USART_SR_TC_FLAG))
    {
        usart_drv_TCInterruptHandler(self);
    }

    // Check transmit data register empty interrupt
    if (((self->usartx->CR1 & (1 << USART_CR1_TXEIE)) == (1 << USART_CR1_TXEIE)) &&
        usart_drv_GetFlagStatus(self->usartx, USART_SR_TXE_FLAG))
    {
        usart_drv_TXEInterruptHandler(self);
    }

    // Check CTS flag interrupt
    if (((self->usartx->CR3 & (1 << USART_CR3_CTSIE)) == (1 << USART_CR3_CTSIE)) &&
        usart_drv_GetFlagStatus(self->usartx, USART_SR_CTS_FLAG))
    {
        usart_drv_CTSInterruptHandler(self);
    }

    // Check RXNEIE control bit
    if ((self->usartx->CR1 & (1 << USART_CR1_RXNEIE)) == (1 << USART_CR1_RXNEIE))
    {
        // Check Received Data Ready to be Read interrupt
        if (usart_drv_GetFlagStatus(self->usartx, USART_SR_RXNE_FLAG))
        {
            usart_drv_RXNEInterruptHandler(self);
        }

        // Check Overrun Error Detected interrupt
        if (usart_drv_GetFlagStatus(self->usartx, USART_SR_ORE_FLAG))
        {
            usart_drv_OREInterruptHandler(self);
        }
    }

    // Check Idle Line Detected interrupt
    if (((self->usartx->CR1 & (1 << USART_CR1_IDLEIE)) == (1 << USART_CR1_IDLEIE)) &&
        usart_drv_GetFlagStatus(self->usartx, USART_SR_IDLE_FLAG))
    {
        usart_drv_IDLEInterruptHandler(self);
    }

    // Check Parity Error interrupt
    if (((self->usartx->CR1 & (1 << USART_CR1_PEIE)) == (1 << USART_CR1_PEIE)) &&
        usart_drv_GetFlagStatus(self->usartx, USART_SR_PE_FLAG))
    {
        usart_drv_PEInterruptHandler(self);
    }

    // Check Break Flag interrupt
    if (((self->usartx->CR2 & (1 << USART_CR2_LBDIE)) == (1 << USART_CR2_LBDIE)) &&
        usart_drv_GetFlagStatus(self->usartx, USART_SR_LBD_FLAG))
    {
        usart_drv_LBDInterruptHandler(self);
    }

    // Check EIE control bit
    if ((self->usartx->CR3 & (1 << USART_CR3_EIE)) == (1 << USART_CR3_EIE))
    {
        // Check Noise Flag interrupt
        if (usart_drv_GetFlagStatus(self->usartx, USART_SR_NF_FLAG))
        {
            usart_drv_NFInterruptHandler(self);
        }

        // Check Overrun error interrupt
        if (usart_drv_GetFlagStatus(self->usartx, USART_SR_ORE_FLAG))
        {
            usart_drv_OREInterruptHandler(self);
        }

        // Check Framing Error in multibuffer communication interrupt
        if (usart_drv_GetFlagStatus(self->usartx, USART_SR_FE_FLAG))
        {
            usart_drv_FEInterruptHandler(self);
        }
    }

}

bool usart_drv_GetFlagStatus(USART_Reg_t *usartx, uint32_t flag)
{
	return ((usartx->SR & flag) == flag);
}

bool usart_drv_DeInit(USART_Handle_t *self)
{
    /* Disable IRQ */
    if (self->config.is_irq_enable)
    {
        uint32_t irq_number;
        volatile uint32_t *arm_nvic_iser_base_addr;
        if (irq_number < 32)
        {
            arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E100;
            *arm_nvic_iser_base_addr &= ~(1 << irq_number);
        }
        else if ((irq_number >= 32) && (irq_number < 64))
        {
            arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E104;
            *arm_nvic_iser_base_addr &= ~(1 << (irq_number % 32));
        }
        else
        {
            arm_nvic_iser_base_addr = (volatile uint32_t *)0xE000E108;
            *arm_nvic_iser_base_addr &= ~(1 << (irq_number % 64));
        }

        uint8_t iprx = irq_number / 4;
        uint8_t iprx_section = irq_number % 4;
        uint8_t shift_amount = (8 * iprx_section) + 4;
        volatile uint32_t *arm_nvic_pr_base_addr = (volatile uint32_t *)(0xE000E400 + (iprx * 4));
        *arm_nvic_pr_base_addr &= ~(self->config.irq_priority << shift_amount);

        if ((_USART_HW_FLOW_CONTROL_CTS_RTS == self->config.hw_flow_control) ||
            (_USART_HW_FLOW_CONTROL_CTS == self->config.hw_flow_control))
        {
            // Disable CTSIE control bit
            self->usartx->CR3 &= ~( 1 << USART_CR3_CTSIE);
        }

        // Disable IDLEIE control bit
        self->usartx->CR1 &= ~( 1 << USART_CR1_IDLEIE);

        if (_USART_PARITY_CONTROL_DIS != self->config.parity_control)
        {
            // Disable PEIE control bit
            self->usartx->CR1 &= ~( 1 << USART_CR1_PEIE);
        }

        // Disable EIE  control bit
        self->usartx->CR3 &= ~( 1 << USART_CR3_EIE);

        // Disable RXNEIE control bit
        self->usartx->CR1 &= ~( 1 << USART_CR1_RXNEIE);

        // Disable TXEIE control bit
        self->usartx->CR1 &= ~( 1 << USART_CR1_TXEIE);

        // Disable TCIE control bit
        self->usartx->CR1 &= ~( 1 << USART_CR1_TCIE);

        self->irq_event = NULL;
    }

    /* Set config */
    if (_USART_MODE_ONLY_TX == self->config.mode)
    {
        self->usartx->CR1 &= ~(1 << USART_CR1_RE);
        self->usartx->CR1 |= (1 << USART_CR1_TE);
    }
    else if (_USART_MODE_ONLY_RX == self->config.mode)
    {
        self->usartx->CR1 &= ~(1 << USART_CR1_TE);
        self->usartx->CR1 |= (1 << USART_CR1_RE);
    }
    else if (_USART_MODE_TX_RX == self->config.mode)
        self->usartx->CR1 |= (1 << USART_CR1_TE) | (1 << USART_CR1_RE);
    else
    {
        self->usartx = NULL;
        return false;
    }

    self->usartx->BRR = 0x00;

    self->usartx->CR2 &= ~(self->config.nosb << USART_CR2_STOP);

    self->usartx->CR1 &= ~((self->config.word_length & 0x1) << USART_CR1_M);

    if (_USART_PARITY_CONTROL_DIS != self->config.parity_control)
    {
        self->usartx->CR1 |= (1 << USART_CR1_PCE);
        self->usartx->CR1 &= ~(1 << USART_CR1_PS);
    }

    if (_USART_HW_FLOW_CONTROL_NONE != self->config.hw_flow_control)
    {
        self->usartx->CR3 &= ~(1 << USART_CR3_RTSE);
        self->usartx->CR3 &= ~(1 << USART_CR3_CTSE);
    }

    self->usartx->CR1 &= ~(1 << USART_CR1_UE);

	usart_drv_PeripheralClockControl(self->usartx, false);

    self->usartx = NULL;
	memset(self, 0, sizeof(*self));

	return false;
}

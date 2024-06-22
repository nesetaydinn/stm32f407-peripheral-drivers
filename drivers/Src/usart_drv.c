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

bool usart_drv_SetInterrupts(USART_Handle_t *self, uint8_t irq_priority, void (*irq_event)(void *self, uint8_t event));

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
                (uint16_t*)data++;
            else
                data++;
            data_len -= 2;
        }
        else
        {
            self->usartx->DR = (*data & 0xFF);
            data_len--;
            data++;
        }
    }
    while (usart_drv_GetFlagStatus(self->usartx, USART_SR_TC_FLAG));
    return true;
}

bool usart_drv_SendDataIT(USART_Handle_t *self, uint8_t *data, uint32_t data_len);


bool usart_drv_ReceiveData(USART_Handle_t *self, uint8_t *data, uint32_t data_len)
{
    if ((NULL == self) || (NULL == data))
        return false;

    while (data_len)
    {
        while (!usart_drv_GetFlagStatus(self->usartx, USART_SR_RXNE_FLAG));
        *data = self->usartx->DR;
        data_len--;
        data++;
    }
    return true;
}

bool usart_drv_ReceiveDataIT(USART_Handle_t *self, uint8_t *data, uint32_t data_len);

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

bool usart_drv_GetFlagStatus(USART_Reg_t *usartx, uint32_t flag)
{
	return ((usartx->SR & flag) == flag);
}

bool usart_drv_DeInit(USART_Handle_t *self)
{

}

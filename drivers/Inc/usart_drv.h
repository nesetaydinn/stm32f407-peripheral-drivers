/*
 * usart_drv.h
 *
 *  Created on: Jun 22, 2024
 *      Author: nesh
 */

#ifndef INC_USART_DRV_H_
#define INC_USART_DRV_H_

#include "stm32f407xx.h"

#include <string.h>
#include <stdbool.h>
#include <math.h>

/* MACROs */

/* SR Register Bit(s) Positions */
#define USART_SR_PE   0x00
#define USART_SR_FE   0x01
#define USART_SR_NF   0x02
#define USART_SR_ORE  0x03
#define USART_SR_IDLE 0x04
#define USART_SR_RXNE 0x05
#define USART_SR_TC   0x06
#define USART_SR_TXE  0x07
#define USART_SR_LBD  0x08
#define USART_SR_CTS  0x09

/* BRR Register Bit(s) Positions */
#define USART_BRR_DF   0x00 // DIV FRACTION BITS
#define USART_BRR_DM   0x04 // DIV MANTISSA BITS

/* CR1 Register Bit(s) Positions */
#define USART_CR1_SBK     0x00
#define USART_CR1_RWU     0x01
#define USART_CR1_RE      0x02
#define USART_CR1_TE      0x03
#define USART_CR1_IDLEIE  0x04
#define USART_CR1_RXNEIE  0x05
#define USART_CR1_TCIE    0x06
#define USART_CR1_TXEIE   0x07
#define USART_CR1_PEIE    0x08
#define USART_CR1_PS      0x09
#define USART_CR1_PCE     0x0A
#define USART_CR1_WAKE    0x0B
#define USART_CR1_M       0x0C
#define USART_CR1_UE      0x0D
#define USART_CR1_OVER8   0x0F

/* CR2 Register Bit(s) Positions */
#define USART_CR2_ADD     0x00
#define USART_CR2_LBDL    0x05
#define USART_CR2_LBDIE   0x06
#define USART_CR2_LBCL    0x08
#define USART_CR2_CPHA    0x09
#define USART_CR2_CPOL    0x0A
#define USART_CR2_CLKEN   0x0B
#define USART_CR2_STOP    0x0C
#define USART_CR2_LINEN   0x0E

/* CR3 Register Bit(s) Positions */
#define USART_CR3_EIE     0x00
#define USART_CR3_IREN    0x01
#define USART_CR3_IRLP    0x02
#define USART_CR3_HDSEL   0x03
#define USART_CR3_NACK    0x04
#define USART_CR3_SCEN    0x05
#define USART_CR3_DMAR    0x06
#define USART_CR3_DMAT    0x07
#define USART_CR3_RTSE    0x08
#define USART_CR3_CTSE    0x09
#define USART_CR3_CTSIE   0x0A
#define USART_CR3_ONEBIT  0x0B

/* GTPR Register Bit(s) Positions */
#define USART_GTPR_PSC     0x00
#define USART_GTPR_GT      0x08

/* SR Register Bit(s) Flags */
#define USART_SR_PE_FLAG   (1 << 0x00)
#define USART_SR_FE_FLAG   (1 << 0x01)
#define USART_SR_NF_FLAG   (1 << 0x02)
#define USART_SR_ORE_FLAG  (1 << 0x03)
#define USART_SR_IDLE_FLAG (1 << 0x04)
#define USART_SR_RXNE_FLAG (1 << 0x05)
#define USART_SR_TC_FLAG   (1 << 0x06)
#define USART_SR_TXE_FLAG  (1 << 0x07)
#define USART_SR_LBD_FLAG  (1 << 0x08)
#define USART_SR_CTS_FLAG  (1 << 0x09)

/* ENUMs */

typedef enum
{
    _USART_IRQ_NO_1 = 0x25, // 37
    _USART_IRQ_NO_2 = 0x26, // 38
    _USART_IRQ_NO_3 = 0x27, // 39
    _USART_IRQ_NO_4 = 0x34, // 52
    _USART_IRQ_NO_5 = 0x35, // 53
    _USART_IRQ_NO_6 = 0x47  // 71
} USART_IRQ_nubmers_t;

typedef enum
{
    _USART_IRQ_STATE_READY      = 0x00,
    _USART_IRQ_STATE_BUSY_IN_RX = 0x01,
    _USART_IRQ_STATE_BUSY_IN_TX = 0x02
} USART_IRQ_State_t;



typedef enum
{
    _USART_IRQ_EVENT_FE_ERROR   = -6,
    _USART_IRQ_EVENT_LBD_ERROR  = -5,
    _USART_IRQ_EVENT_ORE_ERROR  = -4,
    _USART_IRQ_EVENT_PE_ERROR   = -3,
    _USART_IRQ_EVENT_IDLE_ERROR = -2,
    _USART_IRQ_EVENT_CTS_ERROR  = -1,
    _USART_IRQ_EVENT_RX_CMPLT   =  1,
    _USART_IRQ_EVENT_TX_CMPLT   =  2
} USART_IRQ_Event_t;

typedef enum
{
    _USART_MODE_ONLY_TX = 0,
    _USART_MODE_ONLY_RX = 1,
    _USART_MODE_TX_RX   = 2
} USART_Mode_t;

typedef enum
{
    _USART_STANDART_BR_1200    = 1200L,
    _USART_STANDART_BR_2400    = 2400L,
    _USART_STANDART_BR_9600    = 9600L,
    _USART_STANDART_BR_19200   = 19200L,
    _USART_STANDART_BR_38400   = 38400L,
    _USART_STANDART_BR_57600   = 57600L,
    _USART_STANDART_BR_115200  = 115200L,
    _USART_STANDART_BR_230400  = 230400L,
    _USART_STANDART_BR_460800  = 460800L,
    _USART_STANDART_BR_921600  = 921600L,
    _USART_STANDART_BR_2000000 = 2000000L,
    _USART_STANDART_BR_3000000 = 3000000L
} USART_Standart_Baudrate_t;

typedef enum
{
    _USART_STOP_BITS_1   = 0,
    _USART_STOP_BITS_0_5 = 1, // Not available for UART4 & UART5
    _USART_STOP_BITS_2   = 2,
    _USART_STOP_BITS_1_5 = 3  // Not available for UART4 & UART5
} USART_Stop_bits_t;

typedef enum
{
    _USART_WORD_LENGTH_8BITS = 0,
    _USART_WORD_LENGTH_9BITS = 1
} USART_Word_lenght_t;

typedef enum
{
    _USART_PARITY_CONTROL_DIS  = 0,
    _USART_PARITY_CONTROL_EVEN = 1,
    _USART_PARITY_CONTROL_ODD  = 2
} USART_Parity_control_t;

typedef enum
{
    _USART_HW_FLOW_CONTROL_NONE    = 0,
    _USART_HW_FLOW_CONTROL_CTS     = 1,
    _USART_HW_FLOW_CONTROL_RTS     = 2,
    _USART_HW_FLOW_CONTROL_CTS_RTS = 3
} USART_HW_Flow_control_t;

/* STRUCTs */

typedef struct
{
    uint8_t mode;            // Mode, defined as `USART_Mode_t`
    uint32_t baudrate;       // Communication baudrate, defined standart baudrate as `USART_Standart_Baudrate_t`
                             // or custom value
    uint8_t nosb;            // Number of stop bits, defined as `USART_Stop_bits_t` 
    uint8_t word_length;     // Word length, defined as `USART_Word_lenght_t`
    uint8_t parity_control;  // Parity bit control of USART, defined as `USART_Parity_control_t`
    uint8_t hw_flow_control; // Hardware flow control, defined as `USART_HW_Flow_control_t`
    bool is_irq_enable;      // USART IRQ status, true: enable
    uint8_t irq_priority;    // USART interrupt priority
} USART_config_t;

typedef struct
{
    USART_Reg_t *usartx;
    USART_config_t config;
    uint8_t state;   // USART state as `USART_IRQ_State_t`
    struct {
        uint8_t *buffer;
        uint32_t len;
    } tx, rx;
    /* @brief Interrupt callback event
     * @param self USART handle address
     * @param event interrupt event as `USART_IRQ_Event_t`
     * */
    void (*irq_event)(void *self, uint8_t event);
} USART_Handle_t;

/* FUNCTIONs */

/**
 * @brief USART configuration and initialization
 * @param self USART handle base address
 * @param usartx USART peripheral base address
 * @param config USART peripheral configuration as `USART_config_t`
 * @return bool When the initialization is succesfully return true
 */
bool usart_drv_Init(USART_Handle_t *self, USART_Reg_t *usartx, USART_config_t config);

/**
 * @brief USART interrupt enable
 * @param self USART handle base address
 * @param irq_priority IRQ priority
 * @param irq_event IRQ triggered event function
 * @return bool When the operation is success return true
 */
bool usart_drv_SetInterrupts(USART_Handle_t *self, uint8_t irq_priority, void (*irq_event)(void *self, uint8_t event));

bool usart_drv_SendData(USART_Handle_t *self, uint8_t *data, uint32_t data_len);
bool usart_drv_SendDataIT(USART_Handle_t *self, uint8_t *data, uint32_t data_len);

bool usart_drv_ReceiveData(USART_Handle_t *self, uint8_t *data, uint32_t data_len);
bool usart_drv_ReceiveDataIT(USART_Handle_t *self, uint8_t *data, uint32_t data_len);

void usart_drv_IRQHandler(USART_Handle_t *self);

/**
 * @brief USART de-initialization
 * @param self USART handle base address
 * @return bool When the de-initializated the handle return true
 */
bool usart_drv_DeInit(USART_Handle_t *self);

#endif /* INC_USART_DRV_H_ */

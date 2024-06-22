/*
 * i2c_drv.h
 *
 *  Created on: Jun 2, 2024
 *      Author: nesh
 */

#ifndef INC_I2C_DRV_H_
#define INC_I2C_DRV_H_

#include "stm32f407xx.h"

#include <string.h>
#include <stdbool.h>

/* MACROs */

/* CR1 Register Bit(s) Positions */
#define I2C_CR1_PE         0x00
#define I2C_CR1_SMBUS      0x01
#define I2C_CR1_SMBTYPE    0x03
#define I2C_CR1_ENARP      0x04
#define I2C_CR1_ENPEC      0x05
#define I2C_CR1_ENGC       0x06
#define I2C_CR1_NOSTRETCH  0x07
#define I2C_CR1_START      0x08
#define I2C_CR1_STOP       0x09
#define I2C_CR1_ACK        0x0A
#define I2C_CR1_POS        0x0B
#define I2C_CR1_PEC        0x0C
#define I2C_CR1_ALERT      0x0D
#define I2C_CR1_SWRST      0x0F

/* CR2 Register Bit(s) Positions */
#define I2C_CR2_FREQ       0x00
#define I2C_CR2_ITERREN    0x08
#define I2C_CR2_ITEVTEN    0x09
#define I2C_CR2_ITBUFEN    0x0A
#define I2C_CR2_DMAEN      0x0B
#define I2C_CR2_LAST       0x0C

/* OAR1 Register Bit(s) Positions */
#define I2C_OAR1_ADD0      0x00 // Only use this point, when enable 10 bit address
#define I2C_OAR1_ADD7_1    0x01 // If 7 bits address mode is enable use this point 
#define I2C_OAR1_ADDMODE   0x0F 

/* OAR2 Register Bit(s) Positions */
#define I2C_OAR2_ENDUAL    0x00
#define I2C_OAR2_ADD2      0x01 

/* SR1 Register Bit(s) Positions */
#define I2C_SR1_SB         0x00
#define I2C_SR1_ADDR       0x01
#define I2C_SR1_BTF        0x02
#define I2C_SR1_ADD10      0x03
#define I2C_SR1_STOPF      0x04
#define I2C_SR1_RXNE       0x06
#define I2C_SR1_TXE        0x07
#define I2C_SR1_BERR       0x08
#define I2C_SR1_ARLO       0x09
#define I2C_SR1_AF         0x0A
#define I2C_SR1_OVR        0x0B
#define I2C_SR1_PECERR     0x0C
#define I2C_SR1_TIMEOUT    0x0E
#define I2C_SR1_SMBALERT   0x0F

/* SR2 Register Bit(s) Positions */
#define I2C_SR2_MSL        0x00
#define I2C_SR2_BUSY       0x01
#define I2C_SR2_TRA        0x02
#define I2C_SR2_GENCALL    0x04
#define I2C_SR2_SMBDEFAULT 0x05
#define I2C_SR2_SMBHOST    0x06
#define I2C_SR2_DUALF      0x07
#define I2C_SR2_PEC        0x08

/* CCR Register Bit(s) Positions */
#define I2C_CCR_CCR        0x00
#define I2C_CCR_DUTY       0x0E // Fm mode duty cycle
#define I2C_CCR_F_S        0x0F // I2C master mode selection fast, standart

/* SR1 Register Bit(s) Flags */
#define I2C_SR1_SB_FLAG       (1 << 0x00)
#define I2C_SR1_ADDR_FLAG     (1 << 0x01)
#define I2C_SR1_BTF_FLAG      (1 << 0x02)
#define I2C_SR1_ADD10_FLAG    (1 << 0x03)
#define I2C_SR1_STOPF_FLAG    (1 << 0x04)
#define I2C_SR1_RXNE_FLAG     (1 << 0x06)
#define I2C_SR1_TXE_FLAG      (1 << 0x07)
#define I2C_SR1_BERR_FLAG     (1 << 0x08)
#define I2C_SR1_ARLO_FLAG     (1 << 0x09)
#define I2C_SR1_AF_FLAG       (1 << 0x0A)
#define I2C_SR1_OVR_FLAG      (1 << 0x0B)
#define I2C_SR1_PECERR_FLAG   (1 << 0x0C)
#define I2C_SR1_TIMEOUT_FLAG  (1 << 0x0E)
#define I2C_SR1_SMBALERT_FLAG (1 << 0x0F)

/* SR1 Register Bit(s) Flags */
#define I2C_SR2_MSL_FLAG        (1 << 0x00)
#define I2C_SR2_BUSY_FLAG       (1 << 0x01)
#define I2C_SR2_TRA_FLAG        (1 << 0x02)
#define I2C_SR2_GENCALL_FLAG    (1 << 0x04)
#define I2C_SR2_SMBDEFAULT_FLAG (1 << 0x05)
#define I2C_SR2_SMBHOST_FLAG    (1 << 0x06)
#define I2C_SR2_DUALF_FLAG      (1 << 0x07)
#define I2C_SR2_PEC_FLAG        (0xFF << 0x08)

/* ENUMs */

typedef enum
{
    _I2C_IRQ_NO_1_EV = 0x1F,
    _I2C_IRQ_NO_1_ER = 0x20,
    _I2C_IRQ_NO_2_EV = 0x21,
    _I2C_IRQ_NO_2_ER = 0x22,
    _I2C_IRQ_NO_3_EV = 0x48,
    _I2C_IRQ_NO_3_ER = 0x49
} I2C_IRQ_nubmers_t;

typedef enum
{
    _I2C_IRQ_STATE_READY      = 0x00,
    _I2C_IRQ_STATE_BUSY_IN_RX = 0x01,
    _I2C_IRQ_STATE_BUSY_IN_TX = 0x02
} I2C_IRQ_State_t;

typedef enum
{
    _I2C_IRQ_EVENT_STOP     = -7,
    _I2C_IRQ_EVENT_BERR     = -6,
    _I2C_IRQ_EVENT_ARLO     = -5,
    _I2C_IRQ_EVENT_AF       = -4,
    _I2C_IRQ_EVENT_OVR      = -3,
    _I2C_IRQ_EVENT_PECERR   = -2,
    _I2C_IRQ_EVENT_TIMEOUT  = -1,
    _I2C_IRQ_EVENT_TX_CMPLT =  1, // Master mode
    _I2C_IRQ_EVENT_RX_CMPLT =  2, // Master mode
    _I2C_IRQ_EVENT_DATA_REQ =  3, // Slave mode
    _I2C_IRQ_EVENT_DATA_RCV =  4  // Slave mode
} I2C_IRQ_Event_t;

typedef enum
{
    _I2C_SCL_SPEED_SM = 100000, // Standart Mode Speed
    _I2C_SCL_SPEED_FM = 400000  // Fast Mode Speed
} I2C_SCL_Standart_speeds_t;

typedef enum
{
    _I2C_ACK_DISABLE = 0,
    _I2C_ACK_ENABLE  = 1 
} I2C_Ack_state_t;

typedef enum
{
    _I2C_FM_DUTY_CYCLE_2    = 0, // Fm mode tlow/thigh = 2
    _I2C_FM_DUTY_CYCLE_16_9 = 1  // Fm mode tlow/thigh = 16/9 
} I2C_FM_duty_cycle_t;

typedef enum
{
    _I2C_ADDR_OPERATION_WRITE = 0,
    _I2C_ADDR_OPERATION_READ  = 1
} I2C_Addr_operation_t;

/* STRUCTs */

typedef struct
{
    uint32_t scl_speed;    // Clock speed defined as `I2C_SCL_Standart_speeds_t` and set-able other value
    uint16_t device_addr;  // Device address
    uint8_t ack_control;   // Acknowlage control defined as `I2C_Ack_state_t`
    uint8_t fm_duty_cycle; // Fast mode duty cycle defined as `I2C_FM_duty_cycle_t`
    bool is_irq_enable;    // I2C IRQ status, true: enable
    uint8_t irq_priority;  // I2C interrupt priority
} I2C_config_t;

typedef struct
{
    I2C_Reg_t *i2cx;
    I2C_config_t config;
    uint8_t state;   // I2C state as `I2C_IRQ_State_t`
    uint8_t slave_addr;
    struct {
        uint8_t *buffer;
        uint32_t len;
    } tx, rx;
    /* @brief Interrupt callback event
     * @param self I2C handle address
     * @param event interrupt event as `I2C_IRQ_Event_t`
     * */
    void (*irq_event)(void *self, uint8_t event);
} I2C_Handle_t;


/* FUNCTIONs */

/**
 * @brief I2C configuration and initialization
 * @param self I2C handle base address
 * @param i2cx I2C peripheral base address
 * @param config I2C peripheral configuration as `I2C_config_t`
 * @return bool When the initialization is succesfully return true
 */
bool i2c_drv_Init(I2C_Handle_t *self, I2C_Reg_t *i2cx, I2C_config_t config);

/**
 * @brief I2C interrupt enable
 * @param self I2C handle base address
 * @param irq_priority IRQ priority
 * @param irq_event IRQ triggered event function
 * @return bool When the operation is success return true
 */
bool i2c_drv_SetInterrupts(I2C_Handle_t *self, uint8_t irq_priority, void (*irq_event)(void *self, uint8_t event));

/**
 * @brief I2C send data as polling (blocking mode)
 * @param self I2C handle base address
 * @param data Base address of transmitting data
 * @param data_len Total lenght of transmitting data
 * @return bool When the operation is success return true
 */
bool i2c_drv_MasterSendData(I2C_Handle_t *self, uint8_t slave_addr, uint8_t *data, uint32_t data_len);

/**
 * @brief I2C send data as interrupt (non-blocking mode)
 * @param self I2C handle base address
 * @param data Base address of transmitting data
 * @param data_len Total lenght of transmitting data
 * @return bool When the operation is success return true
 */
bool i2c_drv_MasterSendDataIT(I2C_Handle_t *self, uint8_t slave_addr, uint8_t *data, uint32_t data_len);

/**
 * @brief I2C receive data as polling (blocking mode)
 * @param self I2C handle base address
 * @param data Base address of receiving data
 * @param data_len Total lenght of receiving data
 * @return bool When the operation is success return true
 */
bool i2c_drv_MasterReceiveData(I2C_Handle_t *self, uint8_t slave_addr, uint8_t *data, uint32_t data_len);

/**
 * @brief I2C receive data as interrupt (non-blocking mode)
 * @param self I2C handle base address
 * @param data Base address of receiving data
 * @param data_len Total lenght of receiving data
 * @return bool When the operation is success return true
 */
bool i2c_drv_MasterReceiveDataIT(I2C_Handle_t *self, uint8_t slave_addr, uint8_t *data, uint32_t data_len);

/**
 * @brief I2C slave mode receive and transmitting enable or disable control in interrupt mode 
 * @param self I2C handle base address
 * @param state Enabling (true) or disabling (false) status of interrupts
 * @return bool When the operation is success return true
 */
bool i2c_drv_SlaveInterruptsManagement(I2C_Handle_t *self, bool state);

/**
 * @brief I2C send data in slave mode
 * @param self I2C handle base address
 * @param data Transmitting data value
 * @return
 */
void i2c_drv_SlaveSendData(I2C_Handle_t *self, uint8_t data);

/**
 * @brief I2C reveive data in slave mode
 * @param self I2C handle base address
 * @param data Receiving data value
 * @return
 */
uint8_t i2c_drv_SlaveReceiveData(I2C_Handle_t *self);

/**
 * @brief I2C Event Interrupts handler
 * @param self I2C handle base address
 * @return
 */
void i2c_drv_EventIRQHandler(I2C_Handle_t *self);

/**
 * @brief I2C Error Interrupts handler
 * @param self I2C handle base address
 * @return
 */
void i2c_drv_ErrorIRQHandler(I2C_Handle_t *self);

/**
 * @brief I2C de-initialization
 * @param self I2C handle base address
 * @return bool When the de-initializated the handle return true
 */
bool i2c_drv_DeInit(I2C_Handle_t *self);

#endif /* INC_I2C_DRV_H_ */

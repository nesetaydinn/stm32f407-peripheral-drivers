/*
 * spi_drv.h
 *
 *  Created on: May 30, 2024
 *  Author: nesh
 */

#ifndef INC_SPI_DRV_H_
#define INC_SPI_DRV_H_

#include "stm32f407xx.h"
#include "gpio_drv.h"

#include <string.h>
#include <stdbool.h>

/* MACROs */

/* CR1 Register Bit(s) Positions */
#define SPI_CR1_CPHA     0x00
#define SPI_CR1_CPOL     0x01
#define SPI_CR1_MSTR     0x02
#define SPI_CR1_BR       0x03
#define SPI_CR1_SPE      0x06
#define SPI_CR1_LSBFIRST 0x07
#define SPI_CR1_SSI      0x08
#define SPI_CR1_SSM      0x09
#define SPI_CR1_RXONLY   0x0A
#define SPI_CR1_DFF      0x0B
#define SPI_CR1_CRCNEXT  0x0C
#define SPI_CR1_CRCEN    0x0D
#define SPI_CR1_BIDIOE   0x0E
#define SPI_CR1_BIDIMODE 0x0F

/* CR2 Register Bit(s) Positions */
#define SPI_CR2_RXDMAEN 0x00
#define SPI_CR2_TXDMAEN 0x01
#define SPI_CR2_SSOE    0x02
#define SPI_CR2_FRF     0x04
#define SPI_CR2_ERRIE   0x05
#define SPI_CR2_RXNEIE  0x06
#define SPI_CR2_TXEIE   0x07

/* SR Register Bit(s) Positions */
#define SPI_SR_RXNE     0x00
#define SPI_SR_TXE      0x01
#define SPI_SR_CHSIDE   0x02
#define SPI_SR_UDR      0x03
#define SPI_SR_CRCERR   0x04
#define SPI_SR_MODF     0x05
#define SPI_SR_OVR      0x06
#define SPI_SR_BSY      0x07
#define SPI_SR_FRE      0x08

/* SR Register Bit(s) Flags */
#define SPI_SR_RXNE_FLAG   (1 << 0x00)
#define SPI_SR_TXE_FLAG    (1 << 0x01)
#define SPI_SR_CHSIDE_FLAG (1 << 0x02)
#define SPI_SR_UDR_FLAG    (1 << 0x03)
#define SPI_SR_CRCERR_FLAG (1 << 0x04)
#define SPI_SR_MODF_FLAG   (1 << 0x05)
#define SPI_SR_OVR_FLAG    (1 << 0x06)
#define SPI_SR_BSY_FLAG    (1 << 0x07)
#define SPI_SR_FRE_FLAG    (1 << 0x08)

/* ENUMs */

typedef enum
{
    _SPI_IRQ_NO_1 = 0x23,
    _SPI_IRQ_NO_2 = 0x24,
    _SPI_IRQ_NO_3 = 0x33
} SPI_IRQ_nubmers_t;

typedef enum
{
    _SPI_IRQ_STATE_READY      = 0x00,
    _SPI_IRQ_STATE_BUSY_IN_RX = 0x01,
    _SPI_IRQ_STATE_BUSY_IN_TX = 0x02
} SPI_IRQ_State_t;

typedef enum
{
    _SPI_IRQ_EVENT_TX_CMPLT = 0x01,
    _SPI_IRQ_EVENT_RX_CMPLT = 0x02,
    _SPI_IRQ_EVENT_MODF_ERR = 0x04,
    _SPI_IRQ_EVENT_OVR_ERR  = 0x08,
    _SPI_IRQ_EVENT_CRC_ERR  = 0x10,
    _SPI_IRQ_EVENT_FRE_ERR  = 0x20
} SPI_IRQ_Event_t;

typedef enum
{
    _SPI_DEVICE_MODE_SLAVE  = 0x00,
    _SPI_DEVICE_MODE_MASTER
} SPI_Device_mode_t;

typedef enum
{
    _SPI_BUS_CONFIG_FULL_DULEX    = 0x00,
    _SPI_BUS_CONFIG_HALF_DULEX          ,
    _SPI_BUS_CONFIG_SIMPLEX_RONLY
} SPI_Bus_config_t;

typedef enum
{
    _SPI_BUS_SPEED_DIV_2   = 0x0,
    _SPI_BUS_SPEED_DIV_4        ,
    _SPI_BUS_SPEED_DIV_8        ,
    _SPI_BUS_SPEED_DIV_16       ,
    _SPI_BUS_SPEED_DIV_32       ,
    _SPI_BUS_SPEED_DIV_64       ,
    _SPI_BUS_SPEED_DIV_128      ,
    _SPI_BUS_SPEED_DIV_256
} SPI_Bus_speed_t;

typedef enum
{
    _SPI_DFF_8BITS  = 0x0,
    _SPI_DFF_16BITS
} SPI_DFF_t;

typedef enum
{
    _SPI_CPOL_LOW  = 0x0,
    _SPI_CPOL_HIGH
} SPI_CPOL_t;

typedef enum
{
    _SPI_CPHA_LOW  = 0x0,
    _SPI_CPHA_HIGH
} SPI_CPHA_t;

typedef enum
{
    _SPI_SSM_HW = 0x0,
    _SPI_SSM_SW
} SPI_SSM_t;

typedef enum
{
    _SPI_FF_MSB_FIRST = 0x0,
    _SPI_FF_LSB_FIRST
} SPI_FF_t;

/* STRUCTs */

typedef struct
{
    uint8_t mode;         // SPI Device mode as `SPI_Device_mode_t`
    uint8_t bus_config;   // SPI Bus configuration as `SPI_Bus_config_t`
    uint8_t speed;        // SPI Baud rate control as `SPI_Bus_speed_t`
    uint8_t dff;          // SPI Data frame format as `SPI_DFF_t`
    uint8_t cpha;         // SPI Clock phase as `SPI_CPHA_t`
    uint8_t cpol;         // SPI Clock polarity as `SPI_CPOL_t`
    uint8_t ssm;          // SPI Software slave management `SPI_SSM_t`
    uint8_t first_bit;    // SPI Frame format as `SPI_FF_t`
    bool is_irq_enable;   // SPI IRQ status, true: enable
    uint8_t irq_priority; // SPI interrupt priority
} SPI_config_t;

typedef struct
{
    SPI_Reg_t *spix;
    SPI_config_t config;
    struct {
        uint8_t *buffer;
        uint32_t len;
        uint8_t state;   // SPI state as `SPI_IRQ_State_t`
    } tx, rx;
    /* @brief Interrupt callback event
     * @param self SPI handle address
     * @param event SPI interrupt event as `SPI_IRQ_Event_t`
     * */
    void (*irq_event)(void *self, uint8_t event);
} SPI_Handle_t;

/* FUNCTIONs */

/**
 * @brief SPI configuration and initialization
 * @param self SPI handle base address
 * @param spix SPI peripheral base address
 * @param config SPI peripheral configuration as `SPI_config_t`
 * @return bool When the initialization is succesfully return true
 */
bool spi_drv_Init(SPI_Handle_t *self, SPI_Reg_t *spix, SPI_config_t config);

/**
 * @brief SPI interrupt enable
 * @param self SPI handle base address
 * @param irq_priority IRQ priority
 * @param irq_event IRQ triggered event function
 * @return bool When the operation is success return true
 */
bool spi_drv_SetInterrupts(SPI_Handle_t *self, uint8_t irq_priority, void (*irq_event)(void *self, uint8_t event));

/**
 * @brief SPI send data as polling
 * @param self SPI handle base address
 * @param data Base address of transmitting data
 * @param data_len Total lenght of transmitting data
 * @return bool When the operation is success return true
 */
bool spi_drv_SendData(SPI_Handle_t *self, uint8_t *data, uint32_t data_len);

/**
 * @brief SPI send data as interrupt
 * @param self SPI handle base address
 * @param data Base address of transmitting data
 * @param data_len Total lenght of transmitting data
 * @return bool When the operation is success return true
 */
bool spi_drv_SendDataIT(SPI_Handle_t *self, uint8_t *data, uint32_t data_len);

/**
 * @brief SPI receive data as polling
 * @param self SPI handle base address
 * @param data Base address of receiving data
 * @param data_len Total lenght of receiving data
 * @return bool When the operation is success return true
 */
bool spi_drv_ReceiveData(SPI_Handle_t *self, uint8_t *data, uint32_t data_len);

/**
 * @brief SPI receive data as interrupt
 * @param self SPI handle base address
 * @param data Base address of receiving data
 * @param data_len Total lenght of receiving data
 * @return bool When the operation is success return true
 */
bool spi_drv_ReceiveDataIT(SPI_Handle_t *self, uint8_t *data, uint32_t data_len);

/**
 * @brief Set SPI peripheral control
 * @param self SPI handle base address
 * @param state SPI peripheral status, true: enable
 * @return
 */
void spi_drv_PeripheralControl(SPI_Handle_t *self, bool state);

/**
 * @brief SPI Interrupt handler
 * @param self SPI handle base address
 * @return
 */
void spi_drv_IRQHandler(SPI_Handle_t *self);

/**
 * @brief SPI de-initialization
 * @param self SPI handle base address
 * @return bool When the de-initializated the handle return true
 */
bool spi_drv_DeInit(SPI_Handle_t *self);

#endif /* INC_SPI_DRV_H_ */

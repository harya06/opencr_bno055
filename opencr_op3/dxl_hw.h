/*
 *  dxl_hw.h
 *
 *  dynamixel hardware
 */

#ifndef DXL_HW_H
#define DXL_HW_H

#include "dxl_def.h"
#include "dxl_hw_op3.h"

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#define DXL_LED_RX            BDPIN_LED_USER_1
#define DXL_LED_TX            BDPIN_LED_USER_2

uint32_t dxl_hw_begin(uint8_t baud);
extern volatile uint32_t g_dxl_last_io_ms;

void dxl_hw_tx_enable(void);
void dxl_hw_tx_disable(void);

void dxl_hw_power_enable(void);
void dxl_hw_power_disable(void);

uint8_t dxl_hw_read(void);
void    dxl_hw_write(uint8_t value);
void    dxl_hw_write(uint8_t *p_data, uint32_t length);

uint32_t dxl_hw_available(void);

#endif

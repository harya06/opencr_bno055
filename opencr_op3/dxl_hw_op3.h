/*
 *  dxl_hw_op3.h
 *
 *  dynamixel hardware op3
 *
 *  Created on: 2016. 10. 21.
 *      Author: Baram
 */

#ifndef DXL_HW_OP3_H
#define DXL_HW_OP3_H


#include "dxl_def.h"


#ifdef __cplusplus
 extern "C" {
#endif


#ifdef __cplusplus
}
#endif

// ========================
// PIN CONFIGURATION SUMMARY
// ========================

#define DEBUG 0

#ifndef OP3_IO_MONITOR_ENABLE
#define OP3_IO_MONITOR_ENABLE 0
#endif

#define PIN_LED_R         50    // D50 - Red channel PWM
#define PIN_LED_G         51    // D51 - Green channel PWM
#define PIN_LED_B         52    // D52 - Blue channel PWM

#define PIN_LED_1         53    // D53 - Status LED 1
#define PIN_LED_2         54    // D54 - Status LED 2
#define PIN_LED_3         55    // D55 - Status LED 3

#define PIN_BUTTON_S1     56    // D56 - Mode/Config button
#define PIN_BUTTON_S2     57    // D57 - Start button
#define PIN_BUTTON_S3     58    // D58 - User button
#define PIN_BUTTON_S4     59    // D59 - External button

#define PIN_LED_ERROR     60    // D60 - Red LED (error/calib status)
#define PIN_LED_RUN       61    // D61 - Green LED (operation ready)

// Hold S3 to reset BNO calibration + clear calibration EEPROM
#ifndef PIN_BNO_RESET_CALIB
#define PIN_BNO_RESET_CALIB PIN_BUTTON_S3
#endif

#ifndef BNO_RESET_HOLD_TIME_MS
#define BNO_RESET_HOLD_TIME_MS 3000
#endif

// ===== RESERVED PINS =====
#define PIN_RESERVED_D62  62    // D62 - Available for future use
#define PIN_RESERVED_D63  63    // D63 - Available for future use
#define PIN_RESERVED_D64  64    // D64 - Available for future use
#define PIN_RESERVED_D65  65    // D65 - Available for future use
#define PIN_RESERVED_D66  66    // D66 - Available for future use
#define PIN_RESERVED_D67  67    // D67 - Available for future use

// ===== BNO055 IMU SENSOR (I2C Interface) =====
// Hardware I2C1 (STM32L476 built-in controller)
// Pin 14: SDA (I2C1_SDA) = D18
// Pin 15: SCL (I2C1_SCL) = D19
//
// Wiring:
//   VCC   → 3.3V (GPIO Header Pin 1)  [REQUIRED: 3.3V only]
//   GND   → GND (GPIO Header Pin 2)    [REQUIRED: Ground reference]
//   SDA   → D18 + 4.7kΩ pull-up        [REQUIRED: Hardware I2C1 data line]
//   SCL   → D19 + 4.7kΩ pull-up        [REQUIRED: Hardware I2C1 clock line]
//
// Configuration:
//   I2C Address: 0x28 (default - factory configured)
//   I2C Speed: 400 kHz (Hardware I2C1 standard)
//
// Note: D18/D19 is HARDWARE I2C interface only - cannot use other pins!


void dxl_hw_op3_init(void);
void dxl_hw_op3_update(void);

// button
uint8_t dxl_hw_op3_button_read(uint8_t pin_num);

// led
void dxl_hw_op3_led_set(uint8_t pin_num, uint8_t value);
void dxl_hw_op3_led_pwm(uint8_t pin_num, uint8_t value);

// voltage
uint8_t dxl_hw_op3_voltage_read(void);

// acc
int16_t dxl_hw_op3_gyro_get_x(void);
int16_t dxl_hw_op3_gyro_get_y(void);
int16_t dxl_hw_op3_gyro_get_z(void);

int16_t dxl_hw_op3_acc_get_x(void);
int16_t dxl_hw_op3_acc_get_y(void);
int16_t dxl_hw_op3_acc_get_z(void);

int16_t dxl_hw_op3_get_rpy(uint8_t rpy);
void    dxl_hw_op3_start_cali(uint8_t index);
int16_t dxl_hw_op3_get_cali(uint8_t index);
void    dxl_hw_op3_clear_cali(uint8_t index);


void  dxl_hw_op3_set_offset(uint8_t index, float offset_data);
float dxl_hw_op3_get_offset(uint8_t index);

void dxl_hw_op3_start_gyro_cali(void);
bool dxl_hw_op3_get_gyro_cali_done(void);

// BNO055 functions
void dxl_hw_op3_bno_save_calibration(void);
void dxl_hw_op3_bno_load_calibration(void);
void dxl_hw_op3_bno_reset_calibration(void);
void dxl_hw_op3_bno_get_calibration(uint8_t *system, uint8_t *gyro, uint8_t *accel, uint8_t *mag);

// LED Status functions
typedef enum {
  IMU_STATE_INIT,
  IMU_STATE_SENSOR_ERROR,
  IMU_STATE_CALIB_NOT_READY,
  IMU_STATE_MAG_NOT_READY,
  IMU_STATE_RUN_OK
} IMUSystemState;

void dxl_hw_op3_update_led_status(IMUSystemState state);
IMUSystemState dxl_hw_op3_get_imu_state(void);

#endif

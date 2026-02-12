/*
 *  board_config.h
 *
 *  ROBOTIS OpenCR Board Configuration
 *  Required when ROBOTIS board package is not available or incomplete
 *
 *  Created on: 2025. 02. 11.
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <Arduino.h>

// ============================================================
// Type Definitions (Standard C types not in some Arduino envs)
// ============================================================
#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef BOOL
typedef uint8_t BOOL;
#endif

// ============================================================
// Arduino Pin Abstraction Macros (for Adafruit library compatibility)
// ============================================================
// These are typically in pins_arduino.h but may be missing in some variants

#ifndef digitalPinToPort
#define digitalPinToPort(P) (0)  // Stub for STM32
#endif

#ifndef digitalPinToBitMask
#define digitalPinToBitMask(P) (1 << (P % 8))  // Stub for STM32
#endif

#ifndef portOutputRegister
#define portOutputRegister(P) (NULL)  // Stub - not used with I2C
#endif

#ifndef portInputRegister
#define portInputRegister(P) (NULL)  // Stub - not used with I2C
#endif

#ifndef portModeRegister
#define portModeRegister(P) (NULL)  // Stub - not used with I2C
#endif

// ============================================================
// Serial Port Definitions
// ============================================================
// Serial3: Dynamixel communication @ 2 Mbps
#ifndef Serial3
#define Serial3 Serial  // Fallback if board package not installed
#endif

// ============================================================
// Conditional Pin Definitions
// ============================================================
// Only define if not already provided by board package
#ifndef BDPIN_LED_USER_1
#define BDPIN_LED_USER_1  55  // D55 (Red LED for Dynamixel RX indicator)
#endif

#ifndef BDPIN_LED_USER_2
#define BDPIN_LED_USER_2  56  // D56 (Green LED for Dynamixel TX indicator)
#endif

#ifndef BDPIN_DXL_PWR_EN
#define BDPIN_DXL_PWR_EN  26  // D26 (Dynamixel power enable pin)
#endif

#ifndef BDPIN_BUZZER
#define BDPIN_BUZZER      21  // D21 (Buzzer output pin)
#endif

// Buzzer master switch:
// 1 = ON (tone/noTone active), 0 = OFF (muted)
#ifndef OP3_BUZZER_ENABLE
#define OP3_BUZZER_ENABLE 0
#endif

#ifndef BDPIN_BAT_PWR_ADC
#define BDPIN_BAT_PWR_ADC A5  // Analog pin for battery voltage monitoring
#endif

#ifndef BDPIN_BUTTON_S1
#define BDPIN_BUTTON_S1   57  // D57 (Button S1)
#endif

#ifndef BDPIN_BUTTON_S2
#define BDPIN_BUTTON_S2   58  // D58 (Button S2)
#endif

#ifndef BDPIN_BUTTON_S3
#define BDPIN_BUTTON_S3   59  // D59 (Button S3)
#endif

#ifndef BDPIN_BUTTON_S4
#define BDPIN_BUTTON_S4   60  // D60 (Button S4)
#endif

#ifndef BDPIN_LED_R
#define BDPIN_LED_R       50  // D50 (Red)
#endif

#ifndef BDPIN_LED_G
#define BDPIN_LED_G       51  // D51 (Green)
#endif

#ifndef BDPIN_LED_B
#define BDPIN_LED_B       52  // D52 (Blue)
#endif

#ifndef BDPIN_I2C_SDA
#define BDPIN_I2C_SDA     19  // D19 (I2C SDA)
#endif

#ifndef BDPIN_I2C_SCL
#define BDPIN_I2C_SCL     18  // D18 (I2C SCL)
#endif

#endif // BOARD_CONFIG_H

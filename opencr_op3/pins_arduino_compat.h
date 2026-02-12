/*
 *  pins_arduino_compat.h
 *
 *  OpenCR Pin Compatibility for Adafruit libraries
 *  This provides missing pin abstraction macros for STM32/OpenCR platform
 *
 *  Created on: 2025. 02. 11.
 */

#ifndef PINS_ARDUINO_COMPAT_H
#define PINS_ARDUINO_COMPAT_H

// ============================================================
// Arduino Pin Abstraction Macros (Required by Adafruit_BusIO)
// ============================================================
// These macros are typically defined in pins_arduino.h for AVR boards
// STM32 cores may not define them, so we provide stubs

#ifndef digitalPinToPort
  #define digitalPinToPort(P) (0)
#endif

#ifndef digitalPinToBitMask
  #define digitalPinToBitMask(P) (1 << ((P) & 0x07))
#endif

#ifndef portOutputRegister
  #define portOutputRegister(P) ((volatile uint8_t*)(0))
#endif

#ifndef portInputRegister
  #define portInputRegister(P) ((volatile uint8_t*)(0))
#endif

#ifndef portModeRegister
  #define portModeRegister(P) ((volatile uint8_t*)(0))
#endif

#endif // PINS_ARDUINO_COMPAT_H

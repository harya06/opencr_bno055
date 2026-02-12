 /*
 *  opencr_op3
 *
 *
 *  Created on: 2016. 10. 21.
 *      Author: Baram
 */


 /*
=========================================================
                LED STATUS INDICATION
=========================================================

State              LED_ERROR (Red)    LED_RUN (Green)    Description
---------------------------------------------------------------------------
INIT               OFF                Fast Blink         System boot & sensor initialization
SENSOR_ERROR       Slow Blink         OFF                BNO055 not detected (I2C failure)
CALIB_NOT_READY    OFF                Slow Blink         System calibration < 3
MAG_NOT_READY      Fast Blink         Fast Blink         Magnetometer calibration < 3
RUN_OK             OFF                ON (Solid)         Fully calibrated and ready

Blink Timing Definition:
- Fast Blink  : toggle every 100 ms
- Slow Blink  : toggle every 500 ms
- ON (Solid)  : digital HIGH
- OFF         : digital LOW

=========================================================
*/


// Include compatibility header for Adafruit libraries
#include "pins_arduino_compat.h"

#include "dxl_node_op3.h"



extern void dxl_hw_tx_enable(void);

void setup()
{
  Serial3.begin(2000000);
  dxl_node_op3_init();
}

void loop()
{
  dxl_node_op3_loop();
}

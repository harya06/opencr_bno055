/*
 *  dxl_hw_op3.cpp
 *
 *  dynamixel hardware op3
 *
 *  Created on: 2016. 10. 21.
 *      Author: Baram
 */

#include "dxl_hw_op3.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>


#define LED_PWM_PIN_MAX     3
#define LED_PWM_PWM_MAX     31

#define BUTTON_PIN_MAX      4

#define IMU_CALI_MAX_COUNT  512

// BNO055 Configuration
#define BNO055_I2C_ADDR     0x28
#define BNO055_CALIB_EEPROM_START  200
#define BNO055_CALIB_SIGNATURE  0xAA  // Magic signature to verify data validity

// BNO055 Operating Modes (if enum not available in library)
#define BNO055_MODE_CONFIG  0x00
#define BNO055_MODE_NDOF    0x0C  // 9 DOF Sensor Fusion (Gyro + Accel + Mag)
#define OPERATION_MODE_NDOF_FUS_IMUPLUS  0x0C  // Alias for compatibility

// LED Blink timing
#define LED_FAST_BLINK      150
#define LED_SLOW_BLINK      600

// EEPROM calibration structure with signature
struct BNO055CalibWithSignature {
  uint8_t signature;                          // Magic byte: 0xAA
  adafruit_bno055_offsets_t offsets;          // 68 bytes
  uint8_t checksum;                           // Simple checksum
};


static uint8_t led_pwm_pins[LED_PWM_PIN_MAX] = {PIN_LED_R, PIN_LED_G, PIN_LED_B};
static uint8_t led_pwm_value[LED_PWM_PIN_MAX];

static float   imu_offset[3];
static int16_t imu_cali_count[3];
static float   imu_cali_sum[3];

static uint8_t  button_value[BUTTON_PIN_MAX];
static uint32_t button_pin_num[BUTTON_PIN_MAX] = {
  PIN_BUTTON_S1,
  PIN_BUTTON_S2,
  PIN_BUTTON_S3,
  PIN_BUTTON_S4 };



#define BATTERY_POWER_OFF             0
#define BATTERY_POWER_STARTUP         1
#define BATTERY_POWER_NORMAL          2
#define BATTERY_POWER_CHECK           3
#define BATTERY_POWER_WARNNING        4


static uint8_t battery_voltage = 0;
static float   battery_valtage_raw = 0;
static uint8_t battery_state   = BATTERY_POWER_STARTUP;


// BNO055 IMU instance
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_I2C_ADDR, &Wire);

// IMU State variables
static IMUSystemState imu_current_state = IMU_STATE_INIT;
static bool imu_sensor_detected = false;
static uint8_t imu_sys_cal = 0, imu_gyro_cal = 0, imu_accel_cal = 0, imu_mag_cal = 0;

// LED status tracking
static unsigned long led_previous_millis = 0;
static bool led_state = false;

// IMU data storage for compatibility
static float imu_euler[3] = {0.0, 0.0, 0.0};
static float imu_accel[3] = {0.0, 0.0, 0.0};
static float imu_gyro[3] = {0.0, 0.0, 0.0};
static bool imu_yaw_zero_initialized = false;

HardwareTimer Timer(TIMER_CH1);

static inline void dxl_hw_op3_buzzer_tone(uint16_t frequency, uint32_t duration_ms = 0)
{
#if OP3_BUZZER_ENABLE
  if(duration_ms > 0) tone(BDPIN_BUZZER, frequency, duration_ms);
  else                tone(BDPIN_BUZZER, frequency);
#else
  (void)frequency;
  (void)duration_ms;
#endif
}

static inline void dxl_hw_op3_buzzer_stop(void)
{
#if OP3_BUZZER_ENABLE
  noTone(BDPIN_BUZZER);
#endif
}

static float dxl_hw_op3_normalize_deg180(float angle_deg)
{
  while(angle_deg > 180.0f) angle_deg -= 360.0f;
  while(angle_deg < -180.0f) angle_deg += 360.0f;
  return angle_deg;
}

static float dxl_hw_op3_normalize_deg360(float angle_deg)
{
  while(angle_deg >= 360.0f) angle_deg -= 360.0f;
  while(angle_deg < 0.0f)    angle_deg += 360.0f;
  return angle_deg;
}

static void dxl_hw_op3_clear_bno_calibration_eeprom(void)
{
  for(uint16_t i = 0; i < sizeof(BNO055CalibWithSignature); i++)
  {
    EEPROM.update(BNO055_CALIB_EEPROM_START + i, 0xFF);
  }
}

static void dxl_hw_op3_reset_yaw_zero(bool from_button)
{
  if(!imu_sensor_detected)
    return;

  imu_offset[2] = imu_euler[2];
  imu_yaw_zero_initialized = true;

  #if DEBUG
    if(from_button)
    {
      Serial.print("[YAW] S3 short press: yaw re-zeroed at heading ");
      Serial.println(imu_euler[2], 2);
    }
    else
    {
      Serial.print("[YAW] Startup auto-zero: yaw centered at heading ");
      Serial.println(imu_euler[2], 2);
    }
  #endif
}




int16_t dxl_hw_op3_acc_conv(int16_t value);
int16_t dxl_hw_op3_gyro_conv(int16_t value);
void    dxl_hw_op3_button_update();
void    dxl_hw_op3_voltage_update();
void    dxl_hw_op3_bno_update(void);
void    dxl_hw_op3_led_update(void);
void    dxl_hw_op3_io_monitor_update(void);
void    dxl_hw_op3_io_monitor_force_led_on(void);

void handler_led(void)
{
  uint32_t i;
  static uint8_t led_counter = 0;


  for( i=0; i<LED_PWM_PIN_MAX; i++ )
  {
    if( led_counter < led_pwm_value[i] && led_pwm_value[i] > 0 )
    {
      dxl_hw_op3_led_set(led_pwm_pins[i], 0); // LED ON
    }
    else
    {
      dxl_hw_op3_led_set(led_pwm_pins[i], 1); // LED OFF
    }
  }

  led_counter++;
  if( led_counter > LED_PWM_PWM_MAX ) led_counter = 0;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_init
     WORK    :
---------------------------------------------------------------------------*/
void dxl_hw_op3_init(void)
{
  uint16_t i;

  // ===== INITIALIZE SERIAL MONITOR =====
  #if (DEBUG || OP3_IO_MONITOR_ENABLE)
    Serial.begin(115200);
    delay(500);
  #endif

  #if DEBUG
    Serial.println("\n=== OP3 Robot Initialization ===");
    Serial.println("[INIT] Starting hardware initialization...");
  #endif

  // ===== INITIALIZE I2C =====
  Wire.begin();
  Wire.setClock(400000);
  #if DEBUG
    Serial.println("[I2C] Bus initialized at 400 kHz");
  #endif

  // ===== INITIALIZE BNO055 =====
  #if DEBUG
    Serial.println("[IMU] Initializing BNO055 sensor...");
  #endif
  
  if(!bno.begin())
  {
    imu_sensor_detected = false;
    imu_current_state = IMU_STATE_SENSOR_ERROR;
    #if DEBUG
      Serial.println("[ERROR] BNO055 sensor NOT detected!");
      Serial.println("[ERROR] Check I2C connections (SDA=D19, SCL=D18)");
      Serial.println("[ERROR] Check 3.3V power supply and pull-up resistors");
    #endif
  }
  else
  {
    imu_sensor_detected = true;
    imu_current_state = IMU_STATE_INIT;
    
    #if DEBUG
      Serial.println("[OK] BNO055 detected at I2C address 0x28");
    #endif
    
    // Set operating mode for 9 DOF fusion
    bno.setMode((adafruit_bno055_opmode_t)OPERATION_MODE_NDOF_FUS_IMUPLUS);
    delay(1000);
    
    #if DEBUG
      Serial.println("[IMU] Operating mode set to NDOF (9-DOF Fusion)");
      Serial.println("[CAL] Loading calibration from EEPROM...");
    #endif
    
    // Try to load saved calibration from EEPROM
    dxl_hw_op3_bno_load_calibration();
  }

  for(i=0; i<3; i++)
  {
    imu_offset[i]     = 0.0;
    imu_cali_count[i] = 0;
  }

  for(i=0; i<BUTTON_PIN_MAX; i++)
  {
    button_value[i] = 0;
  }

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  
  // ===== INITIALIZE STATUS LED =====
  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_LED_RUN, OUTPUT);
  digitalWrite(PIN_LED_ERROR, LOW);
  digitalWrite(PIN_LED_RUN, LOW);

  pinMode(PIN_BUTTON_S1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_S2, INPUT_PULLUP);
  pinMode(PIN_BUTTON_S3, INPUT_PULLUP);
  pinMode(PIN_BUTTON_S4, INPUT_PULLUP);


  dxl_hw_op3_led_set(PIN_LED_1, 1); // R
  dxl_hw_op3_led_set(PIN_LED_2, 1); // G
  dxl_hw_op3_led_set(PIN_LED_3, 1); // B


  dxl_hw_op3_led_pwm(PIN_LED_R, 0);
  dxl_hw_op3_led_pwm(PIN_LED_G, 0);
  dxl_hw_op3_led_pwm(PIN_LED_B, 0);

  Timer.pause();
  Timer.setPeriod(500);              // 500us
  Timer.attachInterrupt(handler_led);
  Timer.refresh();
  Timer.resume();
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_update
     WORK    :
---------------------------------------------------------------------------*/
void dxl_hw_op3_update(void)
{
  // ===== UPDATE BNO055 DATA =====
  dxl_hw_op3_bno_update();

  // ===== UPDATE LED STATUS =====
#if OP3_IO_MONITOR_ENABLE
  dxl_hw_op3_io_monitor_force_led_on();
#else
  dxl_hw_op3_led_update();
#endif

  dxl_hw_op3_button_update();
  dxl_hw_op3_voltage_update();
  dxl_hw_op3_io_monitor_update();
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_io_monitor_update
     WORK    : Periodic Button/LED diagnostics for Serial Monitor
---------------------------------------------------------------------------*/
void dxl_hw_op3_io_monitor_update(void)
{
#if OP3_IO_MONITOR_ENABLE
  static uint32_t last_print_time = 0;

  if(millis() - last_print_time < 1000)
    return;

  last_print_time = millis();

  const uint8_t s1 = dxl_hw_op3_button_read(PIN_BUTTON_S1);
  const uint8_t s2 = dxl_hw_op3_button_read(PIN_BUTTON_S2);
  const uint8_t s3 = dxl_hw_op3_button_read(PIN_BUTTON_S3);
  const uint8_t s4 = dxl_hw_op3_button_read(PIN_BUTTON_S4);

  // For output pins, digitalRead() returns current output latch level.
  const uint8_t led1 = digitalRead(PIN_LED_1);
  const uint8_t led2 = digitalRead(PIN_LED_2);
  const uint8_t led3 = digitalRead(PIN_LED_3);
  const uint8_t led_err = digitalRead(PIN_LED_ERROR);
  const uint8_t led_run = digitalRead(PIN_LED_RUN);

  Serial.print("[IO] BTN{S1=");
  Serial.print(s1);
  Serial.print(" S2=");
  Serial.print(s2);
  Serial.print(" S3=");
  Serial.print(s3);
  Serial.print(" S4=");
  Serial.print(s4);
  Serial.print("} LED{L1=");
  Serial.print(led1);
  Serial.print(" L2=");
  Serial.print(led2);
  Serial.print(" L3=");
  Serial.print(led3);
  Serial.print(" ERR=");
  Serial.print(led_err);
  Serial.print(" RUN=");
  Serial.print(led_run);
  Serial.print("} RGB_PWM{R=");
  Serial.print(led_pwm_value[0]);
  Serial.print(" G=");
  Serial.print(led_pwm_value[1]);
  Serial.print(" B=");
  Serial.print(led_pwm_value[2]);
  Serial.println("}");
#endif
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_io_monitor_force_led_on
     WORK    : Force all LEDs ON in I/O monitor mode
---------------------------------------------------------------------------*/
void dxl_hw_op3_io_monitor_force_led_on(void)
{
#if OP3_IO_MONITOR_ENABLE
  // RGB PWM LEDs are active-low in handler_led(): 0 = ON
  dxl_hw_op3_led_pwm(PIN_LED_R, LED_PWM_PWM_MAX);
  dxl_hw_op3_led_pwm(PIN_LED_G, LED_PWM_PWM_MAX);
  dxl_hw_op3_led_pwm(PIN_LED_B, LED_PWM_PWM_MAX);

  // Status LEDs are used as active-high in this firmware
  digitalWrite(PIN_LED_ERROR, HIGH);
  digitalWrite(PIN_LED_RUN, HIGH);

  // User LEDs are driven directly; 0 keeps behavior aligned with OpenCR board wiring used here
  dxl_hw_op3_led_set(PIN_LED_1, 0);
  dxl_hw_op3_led_set(PIN_LED_2, 0);
  dxl_hw_op3_led_set(PIN_LED_3, 0);
#endif
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_acc_conv
     WORK    :
---------------------------------------------------------------------------*/
int16_t dxl_hw_op3_acc_conv(int16_t value)
{
  int16_t  data;

  #if 0
  data = constrain(value, -2048, 2048); // 8g->4g
  data = data/4 + 512;
  data = constrain(data, 0, 1023);
  #else
  data = value;
  #endif

  return (int16_t)data;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_button_update
     WORK    :
---------------------------------------------------------------------------*/
void dxl_hw_op3_button_update()
{
  static uint8_t pin_state[BUTTON_PIN_MAX] = {0,};
  uint8_t pin_in = 0;
  uint32_t i;
  static uint32_t pin_time[BUTTON_PIN_MAX] = {0,};
  static uint32_t button_hold_time[BUTTON_PIN_MAX] = {0,};
  const char *button_names[BUTTON_PIN_MAX] = {"S1", "S2", "S3", "S4"};


  for(i=0; i<BUTTON_PIN_MAX; i++)
  {
    pin_in = !digitalRead(button_pin_num[i]);

    switch(pin_state[i])
    {
      case 0:
        if(button_value[i] != pin_in)
        {
          pin_state[i] = 1;
          pin_time[i] = millis();
          button_hold_time[i] = 0;
        }
        break;

      case 1:
        if((millis()-pin_time[i]) > 30)
        {
          if(button_value[i] != pin_in)
          {
            button_value[i] = pin_in;

            #if OP3_IO_MONITOR_ENABLE
              Serial.print("[IO] BTN ");
              Serial.print(button_names[i]);
              Serial.print(" ");
              Serial.println(button_value[i] ? "ON" : "OFF");
            #endif
            
            if(button_pin_num[i] == PIN_BNO_RESET_CALIB && button_value[i] == 1)
            {
              button_hold_time[i] = millis();
              #if DEBUG
                Serial.print("[BTN] Reset-calibration button PRESSED - hold for ");
                Serial.print(BNO_RESET_HOLD_TIME_MS / 1000);
                Serial.println(" sec");
              #endif
            }
            else if(button_pin_num[i] == PIN_BNO_RESET_CALIB && button_value[i] == 0)
            {
              if(button_hold_time[i] > 0 && (millis() - button_hold_time[i]) >= BNO_RESET_HOLD_TIME_MS)
              {
                #if DEBUG
                  Serial.println("[BTN] Long press detected - resetting BNO calibration");
                #endif

                dxl_hw_op3_bno_reset_calibration();
              }
              else
              {
                dxl_hw_op3_reset_yaw_zero(true);
                #if DEBUG
                  Serial.println("[BTN] Short press detected - yaw zero reset");
                #endif
              }
              button_hold_time[i] = 0;
            }
            else
            {
              // Other buttons
              #if DEBUG
                if(button_value[i] == 1)
                {
                  Serial.print("[BTN] Button ");
                  Serial.print(button_names[i]);
                  Serial.println(" PRESSED");
                }
                else
                {
                  Serial.print("[BTN] Button ");
                  Serial.print(button_names[i]);
                  Serial.println(" RELEASED");
                }
              #endif
            }
          }
          pin_state[i] = 0;
        }
        break;

      default:

        break;
    }
  }
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_button_read
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_hw_op3_button_read(uint8_t pin_num)
{
  uint8_t pin_in = 0;
  uint8_t i;

  for(i=0; i<BUTTON_PIN_MAX; i++)
  {
    if(button_pin_num[i] == pin_num)
    {
      break;
    }
  }

  if(i == BUTTON_PIN_MAX) return 0;

  pin_in = button_value[i];

  return pin_in;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_led_set
     WORK    :
---------------------------------------------------------------------------*/
void dxl_hw_op3_led_set(uint8_t pin_num, uint8_t value)
{
  digitalWrite(pin_num, value);
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_led_pwm
     WORK    :
---------------------------------------------------------------------------*/
void dxl_hw_op3_led_pwm(uint8_t pin_num, uint8_t value)
{
  switch(pin_num)
  {
    case PIN_LED_R:
      led_pwm_value[0] = constrain(value, 0, LED_PWM_PWM_MAX);
      break;

    case PIN_LED_G:
      led_pwm_value[1] = constrain(value, 0, LED_PWM_PWM_MAX);
      break;

    case PIN_LED_B:
      led_pwm_value[2] = constrain(value, 0, LED_PWM_PWM_MAX);
      break;
  }
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_voltage_update
     WORK    :
---------------------------------------------------------------------------*/
void dxl_hw_op3_voltage_update(void)
{
  static bool startup = false;
  static int adc_index = 0;
  static int prev_state = 0;
  static int alram_state = 0;
  static int check_index = 0;

  int i;
  int adc_value;
  int adc_sum;
  float vol_value;

  static uint32_t process_time[8] = {0,};
  static uint16_t adc_value_tbl[10] = {0,};

  float voltage_ref = 11.1;


  if(millis()-process_time[0] > 100)
  {
    process_time[0] = millis();
    adc_value = analogRead(BDPIN_BAT_PWR_ADC);

    adc_value_tbl[adc_index] = adc_value;

    adc_index++;
    adc_index %= 10;

    adc_sum = 0;
    for(i=0; i<10; i++)
    {
        adc_sum += adc_value_tbl[i];
    }
    adc_value = adc_sum/10;
    vol_value = map(adc_value, 0, 1023, 0, 331*57/10);
    battery_valtage_raw = vol_value/100;

    battery_valtage_raw += 0.5;

    //Serial.println(vol_value);

    vol_value = battery_valtage_raw * 10;
    vol_value = constrain(vol_value, 0, 255);
    battery_voltage = vol_value;
  }


  if(millis()-process_time[1] > 1000)
  {
    process_time[1] = millis();


    switch(battery_state)
    {
      case BATTERY_POWER_OFF:
        alram_state = 0;
        if(battery_valtage_raw > voltage_ref*0.20)
        {
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_STARTUP;
        }
        else
        {
          dxl_hw_op3_buzzer_stop();
        }
        break;

      case BATTERY_POWER_STARTUP:
        if(battery_valtage_raw > voltage_ref)
        {
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
        }
        else
        {
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_CHECK;
        }
        break;

      case BATTERY_POWER_NORMAL:
        alram_state = 0;
        if(battery_valtage_raw < voltage_ref)
        {
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_CHECK;
          check_index   = 0;
        }
        break;

      case BATTERY_POWER_CHECK:
        if(check_index < 5)
        {
          check_index++;
        }
        else
        {
          if(battery_valtage_raw < voltage_ref)
          {
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_WARNNING;
          }
        }
        if(battery_valtage_raw >= voltage_ref)
        {
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
        }
        break;

      case BATTERY_POWER_WARNNING:
        alram_state ^= 1;
        if(alram_state)
        {
          dxl_hw_op3_buzzer_tone(1000, 500);
        }

        if(battery_valtage_raw > voltage_ref)
        {
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
        }
        if(battery_valtage_raw < voltage_ref*0.20)
        {
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_OFF;
        }
        break;

      default:
        break;
    }
  }

  if (battery_state == BATTERY_POWER_WARNNING)
  {
    if(millis()-process_time[2] >= 200)
    {
      process_time[2] = millis();

      dxl_hw_op3_buzzer_tone(1000, 100);
    }
  }
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_voltage_read
     WORK    :
---------------------------------------------------------------------------*/
uint8_t dxl_hw_op3_voltage_read(void)
{
  /*
  int adc_value;
  float vol_value;

  adc_value = analogRead(BDPIN_BAT_PWR_ADC);

  vol_value = map(adc_value, 0, 1023, 0, 330*57/10);
  vol_value = vol_value/10;
  vol_value = constrain(vol_value, 0, 255);
  */

  return (uint8_t)battery_voltage;
}




/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_gyro_conv
     WORK    :
---------------------------------------------------------------------------*/
int16_t dxl_hw_op3_gyro_conv(int16_t value)
{
  int16_t  data;

  #if 0
  data = constrain(value, -8200, 8200); // 2000deg->500deg
  data = map(data, -8200, 8200, 0, 1023);
  #else
  data = value;
  #endif

  return (int16_t)data;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_acc_get_x
     WORK    :
---------------------------------------------------------------------------*/
int16_t dxl_hw_op3_acc_get_x(void)
{
  // Convert from m/s² to 16-bit: range ±16g = ±156.8 m/s²
  return (int16_t)(imu_accel[0] / 156.8 * 32768);
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_acc_get_y
     WORK    :
---------------------------------------------------------------------------*/
int16_t dxl_hw_op3_acc_get_y(void)
{
  return (int16_t)(imu_accel[1] / 156.8 * 32768);
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_acc_get_z
     WORK    :
---------------------------------------------------------------------------*/
int16_t dxl_hw_op3_acc_get_z(void)
{
  return (int16_t)(imu_accel[2] / 156.8 * 32768);
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_gyro_get_x
     WORK    :
---------------------------------------------------------------------------*/
int16_t dxl_hw_op3_gyro_get_x(void)
{
  // Convert from rad/s to 16-bit: range ±2000 dps = ±34.9 rad/s
  return (int16_t)(imu_gyro[0] / 34.9 * 32768);
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_gyro_get_y
     WORK    :
---------------------------------------------------------------------------*/
int16_t dxl_hw_op3_gyro_get_y(void)
{
  return (int16_t)(imu_gyro[1] / 34.9 * 32768);
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_gyro_get_z
     WORK    :
---------------------------------------------------------------------------*/
int16_t dxl_hw_op3_gyro_get_z(void)
{
  return (int16_t)(imu_gyro[2] / 34.9 * 32768);
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_get_rpy
     WORK    :
---------------------------------------------------------------------------*/
int16_t dxl_hw_op3_get_rpy(uint8_t rpy)
{
  int16_t ret = 0;
  float angle_deg = 0.0f;

  switch(rpy)
  {
    case 0:
      angle_deg = imu_euler[0] - imu_offset[0];
      break;

    case 1:
      angle_deg = imu_euler[1] - imu_offset[1];
      break;

    case 2:
      angle_deg = imu_euler[2] - imu_offset[2];
      break;

    default:
      angle_deg = imu_euler[0] - imu_offset[0];
      break;
  }

  if(rpy == 2) angle_deg = dxl_hw_op3_normalize_deg360(angle_deg);  // Yaw: 0..360
  else         angle_deg = dxl_hw_op3_normalize_deg180(angle_deg);  // Roll/Pitch: -180..180
  if(rpy == 2) ret = (int16_t)(angle_deg * 10.0f);    // Yaw uses 0.1 deg units (fits int16)
  else         ret = (int16_t)(angle_deg * 100.0f);   // Roll/Pitch use 0.01 deg units

  return ret;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_start_cali
     WORK    :
---------------------------------------------------------------------------*/
void dxl_hw_op3_start_cali(uint8_t index)
{
  imu_cali_count[index]  = IMU_CALI_MAX_COUNT;
  imu_cali_sum[index] = 0;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_clear_cali
     WORK    :
---------------------------------------------------------------------------*/
void dxl_hw_op3_clear_cali(uint8_t index)
{
  imu_cali_count[index] = 0;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_get_cali
     WORK    :
---------------------------------------------------------------------------*/
int16_t dxl_hw_op3_get_cali(uint8_t index)
{
  return imu_cali_count[index];
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_set_offset
     WORK    :
---------------------------------------------------------------------------*/
void dxl_hw_op3_set_offset(uint8_t index, float offset_data)
{
  imu_offset[index] = offset_data;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_get_offset
     WORK    :
---------------------------------------------------------------------------*/
float dxl_hw_op3_get_offset(uint8_t index)
{
  return imu_offset[index];
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_start_gyro_cali
     WORK    :
---------------------------------------------------------------------------*/
void dxl_hw_op3_start_gyro_cali(void)
{
  // BNO055 has internal auto-calibration
  // This is just a placeholder for compatibility
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_get_gyro_cali_done
     WORK    :
---------------------------------------------------------------------------*/
bool dxl_hw_op3_get_gyro_cali_done(void)
{
  // BNO055 auto calibration is continuous
  // Always return true for compatibility
  return true;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_bno_update
     WORK    : Update BNO055 sensor data
---------------------------------------------------------------------------*/
void dxl_hw_op3_bno_update(void)
{
  if(!imu_sensor_detected)
    return;

  // BNO055 VECTOR_EULER axes:
  // x = heading(yaw), y = roll, z = pitch
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu_euler[0] = euler.y();  // Roll
  imu_euler[1] = euler.z();  // Pitch
  imu_euler[2] = euler.x();  // Yaw

  if(!imu_yaw_zero_initialized)
  {
    dxl_hw_op3_reset_yaw_zero(false);  // Auto yaw-zero once after IMU starts
  }

  // Get Accelerometer (in m/s²)
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu_accel[0] = accel.x();
  imu_accel[1] = accel.y();
  imu_accel[2] = accel.z();

  // Get Gyroscope (in rad/s)
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu_gyro[0] = gyro.x();
  imu_gyro[1] = gyro.y();
  imu_gyro[2] = gyro.z();

  // Get calibration status
  bno.getCalibration(&imu_sys_cal, &imu_gyro_cal, &imu_accel_cal, &imu_mag_cal);

  // AUTO-SAVE calibration when fully calibrated
  static bool autosave_triggered = false;
  static unsigned long autosave_timer = 0;
  
  if(imu_sys_cal == 3 && !autosave_triggered)
  {
    // Sys fully calibrated, start timer
    if(autosave_timer == 0)
    {
      autosave_timer = millis();
    }
    
    // Wait 2 seconds to ensure stability
    if(millis() - autosave_timer >= 2000)
    {
      autosave_triggered = true;
      dxl_hw_op3_bno_save_calibration();
      #if DEBUG
        Serial.println("[OK] Auto-save complete! Calibration data persisted to EEPROM.");
      #endif
      autosave_timer = 0;
    }
  }
  else if(imu_sys_cal < 3)
  {
    // Reset timer if calibration drops below requirement
    autosave_triggered = false;
    autosave_timer = 0;
  }

  // Update IMU system state based on calibration
  if(!imu_sensor_detected)
  {
    imu_current_state = IMU_STATE_SENSOR_ERROR;
  }
  else if(imu_sys_cal < 3)
  {
    imu_current_state = IMU_STATE_CALIB_NOT_READY;
  }
  else if(imu_mag_cal < 3)
  {
    imu_current_state = IMU_STATE_MAG_NOT_READY;
  }
  else
  {
    imu_current_state = IMU_STATE_RUN_OK;
  }
  
  #if DEBUG
    static unsigned long last_debug_time = 0;
    static unsigned long last_detail_time = 0;
    
    if(millis() - last_debug_time >= 500)  // Print every 500ms
    {
      float roll_out  = (float)dxl_hw_op3_get_rpy(0) / 100.0f;
      float pitch_out = (float)dxl_hw_op3_get_rpy(1) / 100.0f;
      float yaw_out   = (float)dxl_hw_op3_get_rpy(2) / 10.0f;

      last_debug_time = millis();
      Serial.print("[IMU] Roll=");
      Serial.print(roll_out, 2);
      Serial.print(" Pitch=");
      Serial.print(pitch_out, 2);
      Serial.print(" Yaw=");
      Serial.print(yaw_out, 2);
      Serial.print(" | Cal[Sys=");
      Serial.print(imu_sys_cal);
      Serial.print(" Gyro=");
      Serial.print(imu_gyro_cal);
      Serial.print(" Accel=");
      Serial.print(imu_accel_cal);
      Serial.print(" Mag=");
      Serial.print(imu_mag_cal);
      Serial.println("]");
    }
    
    // Print detailed sensor values every 3 seconds for diagnostics
    if(millis() - last_detail_time >= 3000)
    {
      last_detail_time = millis();
      Serial.print("[DBG] Accel(m/s²): X=");
      Serial.print(imu_accel[0], 2);
      Serial.print(" Y=");
      Serial.print(imu_accel[1], 2);
      Serial.print(" Z=");
      Serial.print(imu_accel[2], 2);
      
      // Calculate magnitude (should be ~9.81 m/s² for gravity)
      float accel_mag = sqrt(imu_accel[0]*imu_accel[0] + 
                             imu_accel[1]*imu_accel[1] + 
                             imu_accel[2]*imu_accel[2]);
      Serial.print(" Mag=");
      Serial.println(accel_mag, 2);
    }
  #endif
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_bno_save_calibration
     WORK    : Save calibration data to EEPROM with signature and checksum
---------------------------------------------------------------------------*/
void dxl_hw_op3_bno_save_calibration(void)
{
  if(!imu_sensor_detected)
    return;

  #if DEBUG
    Serial.println("[CAL] Saving calibration data to EEPROM...");
  #endif

  // Create calibration package with signature
  BNO055CalibWithSignature calibPackage;
  calibPackage.signature = BNO055_CALIB_SIGNATURE;
  
  bno.getSensorOffsets(calibPackage.offsets);
  
  // Calculate simple checksum
  uint8_t checksum = 0;
  uint8_t* ptr = (uint8_t*)&calibPackage.offsets;
  for(int i = 0; i < sizeof(adafruit_bno055_offsets_t); i++)
  {
    checksum ^= ptr[i];
  }
  calibPackage.checksum = checksum;

  // Save to EEPROM
  EEPROM.put(BNO055_CALIB_EEPROM_START, calibPackage);
  
  // CRITICAL: Wait longer for EEPROM write to complete
  delay(500);  // ← INCREASED from 0ms
  
  // Verify write by reading back
  BNO055CalibWithSignature verifyPackage;
  EEPROM.get(BNO055_CALIB_EEPROM_START, verifyPackage);
  
  delay(100);  // Extra safety delay

  // Check signature
  if(verifyPackage.signature != BNO055_CALIB_SIGNATURE)
  {
    #if DEBUG
      Serial.println("[ERROR] Signature mismatch after save!");
      Serial.print("[ERROR] Expected: 0xAA, Got: 0x");
      Serial.println(verifyPackage.signature, HEX);
    #endif
    return;
  }
  
  // Check checksum
  uint8_t verify_checksum = 0;
  ptr = (uint8_t*)&verifyPackage.offsets;
  for(int i = 0; i < sizeof(adafruit_bno055_offsets_t); i++)
  {
    verify_checksum ^= ptr[i];
  }
  
  if(verify_checksum != verifyPackage.checksum)
  {
    #if DEBUG
      Serial.println("[ERROR] Checksum mismatch after save!");
      Serial.print("[ERROR] Expected: 0x");
      Serial.print(verifyPackage.checksum, HEX);
      Serial.print(", Got: 0x");
      Serial.println(verify_checksum, HEX);
    #endif
    return;
  }

  // Full verification passed
  #if DEBUG
    Serial.println("[OK] Calibration saved successfully!");
    Serial.print("[CAL] Current status - Sys=");
    Serial.print(imu_sys_cal);
    Serial.print(" Gyro=");
    Serial.print(imu_gyro_cal);
    Serial.print(" Accel=");
    Serial.print(imu_accel_cal);
    Serial.print(" Mag=");
    Serial.println(imu_mag_cal);
  #endif
  
  // Success - blink green LED
  digitalWrite(PIN_LED_RUN, HIGH);
  delay(100);
  digitalWrite(PIN_LED_RUN, LOW);
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_bno_load_calibration
     WORK    : Load calibration data from EEPROM with integrity check
---------------------------------------------------------------------------*/
void dxl_hw_op3_bno_load_calibration(void)
{
  if(!imu_sensor_detected)
    return;

  #if DEBUG
    Serial.println("[CAL] Reading calibration from EEPROM (offset 200)...");
  #endif

  BNO055CalibWithSignature calibPackage;
  EEPROM.get(BNO055_CALIB_EEPROM_START, calibPackage);

  // Verify signature (magic byte)
  if(calibPackage.signature != BNO055_CALIB_SIGNATURE)
  {
    #if DEBUG
      Serial.print("[ERROR] Invalid/empty EEPROM (sig: 0x");
      Serial.print(calibPackage.signature, HEX);
      Serial.println(")");
      Serial.println("[INFO] First boot - BNO055 will auto-calibrate");
      Serial.println("[INFO] **6-POINT CALIBRATION PROCEDURE:**");
      Serial.println("[INFO] 1. Place robot FLAT on ground (1 min)");
      Serial.println("[INFO] 2. Stand upright vertical (1 min)");
      Serial.println("[INFO] 3. Flip upside down (1 min)");
      Serial.println("[INFO] 4. Roll 90° right side (1 min)");
      Serial.println("[INFO] 5. Roll 90° left side (1 min)");
      Serial.println("[INFO] 6. Place FLAT again + move in 8-figure (2 min)");
      Serial.println("[INFO] **Target: Sys=3, Gyro=3, Mag=3 (Accel=0-3 ok during movement)**");
      Serial.println("[INFO] Accel drops to 0-1 during movement - this is NORMAL!");
    #endif
    
    // Clear invalid calibration package
    dxl_hw_op3_clear_bno_calibration_eeprom();
    delay(500);
    
    return;  // Don't load corrupted data
  }

  // Verify checksum
  uint8_t checksum = 0;
  uint8_t* ptr = (uint8_t*)&calibPackage.offsets;
  for(int i = 0; i < sizeof(adafruit_bno055_offsets_t); i++)
  {
    checksum ^= ptr[i];
  }
  
  if(checksum != calibPackage.checksum)
  {
    #if DEBUG
      Serial.println("[ERROR] Checksum mismatch in EEPROM!");
      Serial.print("[ERROR] Expected: 0x");
      Serial.print(calibPackage.checksum, HEX);
      Serial.print(", Got: 0x");
      Serial.println(checksum, HEX);
      Serial.println("[WARN] EEPROM data corrupted - clearing and starting fresh calibration");
    #endif
    
    // Clear corrupted calibration package
    dxl_hw_op3_clear_bno_calibration_eeprom();
    delay(500);
    
    return;  // Don't load corrupted data
  }

  // Data is valid - load into BNO055
  bno.setSensorOffsets(calibPackage.offsets);

  delay(200);  // Wait for sensor to apply offsets

  // Check what was actually loaded
  bno.getCalibration(&imu_sys_cal, &imu_gyro_cal, &imu_accel_cal, &imu_mag_cal);
  
  #if DEBUG
    Serial.print("[CAL] Loaded calibration - Sys=");
    Serial.print(imu_sys_cal);
    Serial.print(" Gyro=");
    Serial.print(imu_gyro_cal);
    Serial.print(" Accel=");
    Serial.print(imu_accel_cal);
    Serial.print(" Mag=");
    Serial.println(imu_mag_cal);
    
    if(imu_sys_cal >= 3 && imu_mag_cal >= 3)
    {
      Serial.println("[OK] Calibration is VALID - Ready for operation!");
    }
    else
    {
      Serial.println("[WARN] Calibration incomplete - Robot needs re-calibration");
      Serial.println("[INFO] Move robot in 8-figure pattern for 2-3 minutes to calibrate");
    }
  #endif
}

/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_bno_reset_calibration
     WORK    : Clear calibration data in EEPROM and reset runtime offsets
---------------------------------------------------------------------------*/
void dxl_hw_op3_bno_reset_calibration(void)
{
  dxl_hw_op3_clear_bno_calibration_eeprom();

  imu_sys_cal = 0;
  imu_gyro_cal = 0;
  imu_accel_cal = 0;
  imu_mag_cal = 0;
  imu_current_state = IMU_STATE_CALIB_NOT_READY;

  if(imu_sensor_detected)
  {
    adafruit_bno055_offsets_t zero_offsets;
    memset(&zero_offsets, 0, sizeof(zero_offsets));

    bno.setMode((adafruit_bno055_opmode_t)BNO055_MODE_CONFIG);
    delay(30);
    bno.setSensorOffsets(zero_offsets);
    delay(30);
    bno.setMode((adafruit_bno055_opmode_t)OPERATION_MODE_NDOF_FUS_IMUPLUS);
    delay(100);
  }

  imu_offset[2] = imu_euler[2];
  imu_yaw_zero_initialized = true;

  dxl_hw_op3_buzzer_tone(1600, 120);

  #if DEBUG
    Serial.println("[CAL] EEPROM calibration cleared.");
    Serial.println("[CAL] Recalibrate IMU: move robot in 8-figure until Sys=3 and Mag=3.");
  #endif
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_bno_get_calibration
     WORK    : Get current calibration status
---------------------------------------------------------------------------*/
void dxl_hw_op3_bno_get_calibration(uint8_t *system, uint8_t *gyro, uint8_t *accel, uint8_t *mag)
{
  if(system) *system = imu_sys_cal;
  if(gyro)   *gyro   = imu_gyro_cal;
  if(accel)  *accel  = imu_accel_cal;
  if(mag)    *mag    = imu_mag_cal;
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_led_update
     WORK    : Update LED blink pattern based on system state
---------------------------------------------------------------------------*/
void dxl_hw_op3_led_update(void)
{
  dxl_hw_op3_update_led_status(imu_current_state);
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_update_led_status
     WORK    : Control LED blink pattern
---------------------------------------------------------------------------*/
void dxl_hw_op3_update_led_status(IMUSystemState state)
{
  unsigned long currentMillis = millis();
  unsigned long blinkInterval = LED_SLOW_BLINK;
  bool useRed = false, useGreen = false;
  static IMUSystemState prev_state = IMU_STATE_INIT;
  const char *state_names[] = {"INIT", "SENSOR_ERROR", "CALIB_NOT_READY", "MAG_NOT_READY", "RUN_OK"};

  // Log state changes
  #if DEBUG
    if(state != prev_state)
    {
      Serial.print("[LED] State changed: ");
      Serial.print(state_names[prev_state]);
      Serial.print(" -> ");
      Serial.println(state_names[state]);
      prev_state = state;
    }
  #endif

  switch(state)
  {
    case IMU_STATE_INIT:
      // Fast blink: GREEN only (RED is OFF)
      blinkInterval = LED_FAST_BLINK;
      useRed = false;      // ← RED LED MATI saat INIT
      useGreen = true;     // ← HIJAU LED BLINK saat INIT
      #if DEBUG
        if(state != prev_state) Serial.println("[LED] GREEN BLINK - Initializing sensors...");
      #endif
      break;

    case IMU_STATE_SENSOR_ERROR:
      // Slow blink: RED only
      blinkInterval = LED_SLOW_BLINK;
      useRed = true;
      useGreen = false;
      #if DEBUG
        if(state != prev_state) Serial.println("[LED] **RED LED ERROR MODE** - Check BNO055 connection!");
      #endif
      break;

    case IMU_STATE_CALIB_NOT_READY:
      // Slow blink: GREEN only
      blinkInterval = LED_SLOW_BLINK;
      useRed = false;
      useGreen = true;
      #if DEBUG
        if(state != prev_state) Serial.println("[LED] GREEN LED - Calibration needed (Sys<3)");
      #endif
      break;

    case IMU_STATE_MAG_NOT_READY:
      // Fast blink: Both RED and GREEN
      blinkInterval = LED_FAST_BLINK;
      useRed = true;
      useGreen = true;
      #if DEBUG
        if(state != prev_state) Serial.println("[LED] Magnetometer not calibrated (Mag<3) - Move robot in 8-figure!");
      #endif
      break;

    case IMU_STATE_RUN_OK:
      // Constant ON: GREEN only
      digitalWrite(PIN_LED_ERROR, LOW);
      digitalWrite(PIN_LED_RUN, HIGH);
      #if DEBUG
        if(state != prev_state) Serial.println("[LED] GREEN LED ON - Robot ready for operation!");
      #endif
      return;

    default:
      digitalWrite(PIN_LED_ERROR, LOW);
      digitalWrite(PIN_LED_RUN, LOW);
      return;
  }

  // Handle blinking
  if(currentMillis - led_previous_millis >= blinkInterval)
  {
    led_previous_millis = currentMillis;
    led_state = !led_state;

    digitalWrite(PIN_LED_ERROR, useRed ? led_state : LOW);
    digitalWrite(PIN_LED_RUN, useGreen ? led_state : LOW);
  }
}


/*---------------------------------------------------------------------------
     TITLE   : dxl_hw_op3_get_imu_state
     WORK    : Return current IMU system state
---------------------------------------------------------------------------*/
IMUSystemState dxl_hw_op3_get_imu_state(void)
{
  return imu_current_state;
}

/*
 *  dxl_hw_op3.cpp
 *
 *  dynamixel hardware op3 (BNO055)
 */

#include "dxl_hw_op3.h"
#include "dxl_hw.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

#define LED_PWM_PIN_MAX     3
#define LED_PWM_PWM_MAX     31
#define BUTTON_PIN_MAX      4
#define IMU_CALI_MAX_COUNT  512

#define BNO055_I2C_ADDR             0x28
#define BNO055_CALIB_EEPROM_START   200
#define BNO055_CALIB_SIGNATURE      0xAA
#define BNO055_MODE_CONFIG          0x00
#define BNO055_MODE_NDOF            0x0C

#define BATTERY_POWER_OFF             0
#define BATTERY_POWER_STARTUP         1
#define BATTERY_POWER_NORMAL          2
#define BATTERY_POWER_CHECK           3
#define BATTERY_POWER_WARNNING        4

struct BNO055CalibWithSignature {
  uint8_t signature;
  adafruit_bno055_offsets_t offsets;
  uint8_t checksum;
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
  PIN_BUTTON_S4
};

static uint8_t battery_voltage = 0;
static float   battery_valtage_raw = 0;
static uint8_t battery_state   = BATTERY_POWER_STARTUP;

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_I2C_ADDR, &Wire);
HardwareTimer Timer(TIMER_CH1);

static bool imu_sensor_detected = false;
static float imu_euler[3] = {0.0f, 0.0f, 0.0f};
static float imu_accel[3] = {0.0f, 0.0f, 0.0f};
static float imu_gyro[3]  = {0.0f, 0.0f, 0.0f};
static uint8_t imu_sys_cal = 0, imu_gyro_cal = 0, imu_accel_cal = 0, imu_mag_cal = 0;
static bool gyro_cali_request = false;

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
  while(angle_deg > 180.0f)  angle_deg -= 360.0f;
  while(angle_deg < -180.0f) angle_deg += 360.0f;
  return angle_deg;
}

static float dxl_hw_op3_normalize_deg360(float angle_deg)
{
  while(angle_deg >= 360.0f) angle_deg -= 360.0f;
  while(angle_deg < 0.0f)    angle_deg += 360.0f;
  return angle_deg;
}

static uint8_t dxl_hw_op3_calib_checksum(const adafruit_bno055_offsets_t &offsets)
{
  uint8_t checksum = 0;
  const uint8_t *ptr = (const uint8_t *)&offsets;
  for(uint16_t i = 0; i < sizeof(adafruit_bno055_offsets_t); i++) checksum ^= ptr[i];
  return checksum;
}

int16_t dxl_hw_op3_acc_conv(int16_t value);
int16_t dxl_hw_op3_gyro_conv(int16_t value);
void    dxl_hw_op3_button_update();
void    dxl_hw_op3_voltage_update();
void    dxl_hw_op3_bno_update();
void    dxl_hw_op3_status_led_update();
void    dxl_hw_op3_io_monitor_update();
void    dxl_hw_op3_led_bno(uint8_t pin_num, uint8_t value);

void handler_led(void)
{
  uint32_t i;
  static uint8_t led_counter = 0;

  for(i = 0; i < LED_PWM_PIN_MAX; i++)
  {
    if(led_counter < led_pwm_value[i] && led_pwm_value[i] > 0)
      dxl_hw_op3_led_set(led_pwm_pins[i], 0);
    else
      dxl_hw_op3_led_set(led_pwm_pins[i], 1);
  }

  led_counter++;
  if(led_counter > LED_PWM_PWM_MAX) led_counter = 0;
}

void dxl_hw_op3_init(void)
{
  uint16_t i;

#if DEBUG_SERIAL_ENABLE || OP3_IO_MONITOR_ENABLE
  Serial.begin(115200);
#endif

#if OP3_BNO_ENABLE
  Wire.begin();
  Wire.setClock(400000);
  imu_sensor_detected = bno.begin();
  if(imu_sensor_detected)
  {
    bno.setMode((adafruit_bno055_opmode_t)BNO055_MODE_NDOF);
    bno.setExtCrystalUse(true);
  }
#else
  imu_sensor_detected = false;
#endif

  for(i=0; i<3; i++)
  {
    imu_offset[i] = 0.0f;
    imu_cali_count[i] = 0;
    imu_cali_sum[i] = 0.0f;
  }

  for(i=0; i<BUTTON_PIN_MAX; i++) button_value[i] = 0;

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_LED_RUN, OUTPUT);

  pinMode(PIN_BUTTON_S1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_S2, INPUT_PULLUP);
  pinMode(PIN_BUTTON_S3, INPUT_PULLUP);
  pinMode(PIN_BUTTON_S4, INPUT_PULLUP);

  dxl_hw_op3_led_set(PIN_LED_1, 1);
  dxl_hw_op3_led_set(PIN_LED_2, 1);
  dxl_hw_op3_led_set(PIN_LED_3, 1);
  dxl_hw_op3_led_bno(PIN_LED_ERROR, 0);
  dxl_hw_op3_led_bno(PIN_LED_RUN, 0);

  dxl_hw_op3_led_pwm(PIN_LED_R, 0);
  dxl_hw_op3_led_pwm(PIN_LED_G, 0);
  dxl_hw_op3_led_pwm(PIN_LED_B, 0);

  Timer.pause();
  Timer.setPeriod(500);
  Timer.attachInterrupt(handler_led);
  Timer.refresh();
  Timer.resume();
}

void dxl_hw_op3_update(void)
{
  uint8_t i;

  dxl_hw_op3_bno_update();

  for(i=0; i<3; i++)
  {
    if(imu_cali_count[i] > 0)
    {
      imu_cali_sum[i] += imu_euler[i];
      imu_cali_count[i]--;
      if(imu_cali_count[i] == 0)
      {
        imu_offset[i] = imu_cali_sum[i] / IMU_CALI_MAX_COUNT;
        imu_cali_count[i] = -1;
      }
    }
  }

  dxl_hw_op3_button_update();
  dxl_hw_op3_voltage_update();
  dxl_hw_op3_status_led_update();
  dxl_hw_op3_io_monitor_update();
}

void dxl_hw_op3_bno_update(void)
{
#if !OP3_BNO_ENABLE
  return;
#else
  static uint32_t last_update_ms = 0;
  static uint32_t last_accel_ms = 0;
  static uint32_t last_gyro_ms = 0;
  static uint32_t last_calib_poll_ms = 0;
  static uint32_t autosave_ms = 0;

  if(!imu_sensor_detected) return;

#if OP3_BNO_SKIP_WHEN_DXL_BUSY
  if(DXL_PORT.available() > 0) return;
  if((millis() - g_dxl_last_io_ms) < OP3_BNO_DXL_IDLE_GUARD_MS) return;
#endif

  if(millis() - last_update_ms < OP3_BNO_UPDATE_PERIOD_MS) return;
  last_update_ms = millis();

  // Always prioritize fused orientation update.
  {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    imu_euler[0] = euler.z();  // roll
    imu_euler[1] = euler.y();  // pitch
    imu_euler[2] = euler.x();  // yaw
  }

#if !OP3_BNO_EULER_ONLY_MODE
  if(millis() - last_accel_ms >= OP3_BNO_ACCEL_UPDATE_PERIOD_MS)
  {
    last_accel_ms = millis();
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu_accel[0] = accel.x();
    imu_accel[1] = accel.y();
    imu_accel[2] = accel.z();
  }

  if(millis() - last_gyro_ms >= OP3_BNO_GYRO_UPDATE_PERIOD_MS)
  {
    last_gyro_ms = millis();
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu_gyro[0] = gyro.x();
    imu_gyro[1] = gyro.y();
    imu_gyro[2] = gyro.z();
  }
#endif

  if(millis() - last_calib_poll_ms >= OP3_BNO_CALIB_POLL_PERIOD_MS)
  {
    last_calib_poll_ms = millis();
    bno.getCalibration(&imu_sys_cal, &imu_gyro_cal, &imu_accel_cal, &imu_mag_cal);
  }

#if OP3_BNO_AUTOSAVE_ENABLE
  if(imu_sys_cal == 3 && imu_mag_cal == 3)
  {
    if(autosave_ms == 0) autosave_ms = millis();
    if(millis() - autosave_ms > 5000)
    {
      adafruit_bno055_offsets_t offsets;
      bno.getSensorOffsets(offsets);
      BNO055CalibWithSignature pkg;
      pkg.signature = BNO055_CALIB_SIGNATURE;
      pkg.offsets = offsets;
      pkg.checksum = dxl_hw_op3_calib_checksum(offsets);
      EEPROM.put(BNO055_CALIB_EEPROM_START, pkg);
      autosave_ms = 0;
    }
  }
  else
  {
    autosave_ms = 0;
  }
#else
  (void)autosave_ms;
#endif
#endif
}

void dxl_hw_op3_status_led_update(void)
{
  static uint32_t last_blink_ms = 0;
  static bool blink = false;

  if(!imu_sensor_detected)
  {
    if(millis() - last_blink_ms >= 250)
    {
      last_blink_ms = millis();
      blink = !blink;
      dxl_hw_op3_led_bno(PIN_LED_ERROR, blink ? 1 : 0);
    }
    dxl_hw_op3_led_bno(PIN_LED_RUN, 0);
    return;
  }

  if(imu_sys_cal < 3 || imu_mag_cal < 3)
  {
    if(millis() - last_blink_ms >= 500)
    {
      last_blink_ms = millis();
      blink = !blink;
      dxl_hw_op3_led_bno(PIN_LED_RUN, blink ? 1 : 0);
    }
    dxl_hw_op3_led_bno(PIN_LED_ERROR, 0);
    return;
  }

  dxl_hw_op3_led_bno(PIN_LED_ERROR, 0);
  dxl_hw_op3_led_bno(PIN_LED_RUN, 1);
}

void dxl_hw_op3_io_monitor_update(void)
{
#if OP3_IO_MONITOR_ENABLE
  static uint32_t last_ms = 0;
  if(millis() - last_ms < 1000) return;
  last_ms = millis();

  Serial.print("[IO] BTN ");
  Serial.print(dxl_hw_op3_button_read(PIN_BUTTON_S1));
  Serial.print(dxl_hw_op3_button_read(PIN_BUTTON_S2));
  Serial.print(dxl_hw_op3_button_read(PIN_BUTTON_S3));
  Serial.print(dxl_hw_op3_button_read(PIN_BUTTON_S4));
  Serial.print(" CAL[");
  Serial.print(imu_sys_cal); Serial.print(",");
  Serial.print(imu_gyro_cal); Serial.print(",");
  Serial.print(imu_accel_cal); Serial.print(",");
  Serial.print(imu_mag_cal); Serial.println("]");
#endif
}

int16_t dxl_hw_op3_acc_conv(int16_t value)
{
  return value;
}

void dxl_hw_op3_button_update()
{
  static uint8_t pin_state[BUTTON_PIN_MAX] = {0,};
  uint8_t pin_in = 0;
  uint32_t i;
  static uint32_t pin_time[BUTTON_PIN_MAX] = {0,};

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
        }
        break;
      case 1:
        if((millis() - pin_time[i]) > 30)
        {
          if(button_value[i] != pin_in) button_value[i] = pin_in;
          pin_state[i] = 0;
        }
        break;
      default:
        break;
    }
  }
}

uint8_t dxl_hw_op3_button_read(uint8_t pin_num)
{
  uint8_t i;
  for(i=0; i<BUTTON_PIN_MAX; i++)
  {
    if(button_pin_num[i] == pin_num) break;
  }
  if(i == BUTTON_PIN_MAX) return 0;
  return button_value[i];
}

void dxl_hw_op3_led_set(uint8_t pin_num, uint8_t value)
{
  digitalWrite(pin_num, value);
}

void dxl_hw_op3_led_pwm(uint8_t pin_num, uint8_t value)
{
  switch(pin_num)
  {
    case PIN_LED_R: led_pwm_value[0] = constrain(value, 0, LED_PWM_PWM_MAX); break;
    case PIN_LED_G: led_pwm_value[1] = constrain(value, 0, LED_PWM_PWM_MAX); break;
    case PIN_LED_B: led_pwm_value[2] = constrain(value, 0, LED_PWM_PWM_MAX); break;
  }
}

void dxl_hw_op3_led_bno(uint8_t pin_num, uint8_t value)
{
  digitalWrite(pin_num, value);
}

void dxl_hw_op3_voltage_update(void)
{
  static int adc_index = 0;
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
    for(i=0; i<10; i++) adc_sum += adc_value_tbl[i];

    adc_value = adc_sum / 10;
    vol_value = map(adc_value, 0, 1023, 0, 331*57/10);
    battery_valtage_raw = vol_value / 100;
    battery_valtage_raw += 0.5;

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
          battery_state = BATTERY_POWER_STARTUP;
        else
          dxl_hw_op3_buzzer_stop();
        break;

      case BATTERY_POWER_STARTUP:
        if(battery_valtage_raw > voltage_ref) battery_state = BATTERY_POWER_NORMAL;
        else battery_state = BATTERY_POWER_CHECK;
        break;

      case BATTERY_POWER_NORMAL:
        alram_state = 0;
        if(battery_valtage_raw < voltage_ref)
        {
          battery_state = BATTERY_POWER_CHECK;
          check_index = 0;
        }
        break;

      case BATTERY_POWER_CHECK:
        if(check_index < 5) check_index++;
        else if(battery_valtage_raw < voltage_ref) battery_state = BATTERY_POWER_WARNNING;
        if(battery_valtage_raw >= voltage_ref) battery_state = BATTERY_POWER_NORMAL;
        break;

      case BATTERY_POWER_WARNNING:
        alram_state ^= 1;
        if(alram_state) dxl_hw_op3_buzzer_tone(1000, 500);
        if(battery_valtage_raw > voltage_ref) battery_state = BATTERY_POWER_NORMAL;
        if(battery_valtage_raw < voltage_ref*0.20) battery_state = BATTERY_POWER_OFF;
        break;

      default:
        break;
    }
  }

  if(battery_state == BATTERY_POWER_WARNNING)
  {
    if(millis()-process_time[2] >= 200)
    {
      process_time[2] = millis();
      dxl_hw_op3_buzzer_tone(1000, 100);
    }
  }
}

uint8_t dxl_hw_op3_voltage_read(void)
{
  return (uint8_t)battery_voltage;
}

int16_t dxl_hw_op3_gyro_conv(int16_t value)
{
  return value;
}

int16_t dxl_hw_op3_acc_get_x(void)
{
#if OP3_BNO_ENABLE
  const float raw = (imu_accel[0] / 9.80665f) * 16384.0f;
  return dxl_hw_op3_acc_conv((int16_t)constrain(raw, -32768.0f, 32767.0f));
#else
  return 0;
#endif
}

int16_t dxl_hw_op3_acc_get_y(void)
{
#if OP3_BNO_ENABLE
  const float raw = (imu_accel[1] / 9.80665f) * 16384.0f;
  return dxl_hw_op3_acc_conv((int16_t)constrain(raw, -32768.0f, 32767.0f));
#else
  return 0;
#endif
}

int16_t dxl_hw_op3_acc_get_z(void)
{
#if OP3_BNO_ENABLE
  const float raw = (imu_accel[2] / 9.80665f) * 16384.0f;
  return dxl_hw_op3_acc_conv((int16_t)constrain(raw, -32768.0f, 32767.0f));
#else
  return 0;
#endif
}

int16_t dxl_hw_op3_gyro_get_x(void)
{
#if OP3_BNO_ENABLE
  const float raw = imu_gyro[0] / 0.0010653f;
  return dxl_hw_op3_gyro_conv((int16_t)constrain(raw, -32768.0f, 32767.0f));
#else
  return 0;
#endif
}

int16_t dxl_hw_op3_gyro_get_y(void)
{
#if OP3_BNO_ENABLE
  const float raw = imu_gyro[1] / 0.0010653f;
  return dxl_hw_op3_gyro_conv((int16_t)constrain(raw, -32768.0f, 32767.0f));
#else
  return 0;
#endif
}

int16_t dxl_hw_op3_gyro_get_z(void)
{
#if OP3_BNO_ENABLE
  const float raw = imu_gyro[2] / 0.0010653f;
  return dxl_hw_op3_gyro_conv((int16_t)constrain(raw, -32768.0f, 32767.0f));
#else
  return 0;
#endif
}

int16_t dxl_hw_op3_get_rpy(uint8_t rpy)
{
#if OP3_BNO_ENABLE
  float angle = 0.0f;
  switch(rpy)
  {
    case 0: angle = imu_euler[0] - imu_offset[0]; break;
    case 1: angle = imu_euler[1] - imu_offset[1]; break;
    case 2: angle = imu_euler[2] - imu_offset[2]; break;
    default: angle = imu_euler[0] - imu_offset[0]; break;
  }

  if(rpy == 2) angle = dxl_hw_op3_normalize_deg360(angle);
  else         angle = dxl_hw_op3_normalize_deg180(angle);

  return (int16_t)(angle * 10.0f);
#else
  (void)rpy;
  return 0;
#endif
}

void dxl_hw_op3_start_cali(uint8_t index)
{
  imu_cali_count[index] = IMU_CALI_MAX_COUNT;
  imu_cali_sum[index] = 0;
}

void dxl_hw_op3_clear_cali(uint8_t index)
{
  imu_cali_count[index] = 0;
}

int16_t dxl_hw_op3_get_cali(uint8_t index)
{
  return imu_cali_count[index];
}

void dxl_hw_op3_set_offset(uint8_t index, float offset_data)
{
  imu_offset[index] = offset_data;
}

float dxl_hw_op3_get_offset(uint8_t index)
{
  return imu_offset[index];
}

void dxl_hw_op3_start_gyro_cali(void)
{
#if OP3_BNO_ENABLE
  gyro_cali_request = true;
#else
  gyro_cali_request = false;
#endif
}

bool dxl_hw_op3_get_gyro_cali_done(void)
{
#if OP3_BNO_ENABLE
  if(!gyro_cali_request) return false;
  bno.getCalibration(&imu_sys_cal, &imu_gyro_cal, &imu_accel_cal, &imu_mag_cal);
  if(imu_sys_cal == 3 && imu_mag_cal == 3)
  {
    gyro_cali_request = false;
    return true;
  }
  return false;
#else
  return true;
#endif
}

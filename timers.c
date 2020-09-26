/***
** Created by Aleksey Volkov on 13/11/2018.
***/
#include "app_timer.h"
#include "timers.h"
#include "ble_services.h"
#include "nrf_log.h"

#include "sensors.h"

#define SENSOR_READ_INTERVAL_FAST_RUN       APP_TIMER_TICKS(2000)             /* 1 sec   */
#define SENSOR_READ_INTERVAL                APP_TIMER_TICKS(1*60*1000)        /* 1 min */
#define CH_UPDATE_INTERVAL                  APP_TIMER_TICKS(10*1000)          /* 10 sec */

APP_TIMER_DEF(m_read_sensors_timer_id);
APP_TIMER_DEF(m_update_char_timer_id);

uint64_t sensor_read_interval = SENSOR_READ_INTERVAL;

/* Function for the Timer initialization. */
void timers_init()
{
  ret_code_t err_code;

  // Initialize timer module.
  err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);

  /* Create read sensors timer. */
  err_code = app_timer_create(&m_read_sensors_timer_id, APP_TIMER_MODE_REPEATED, read_sensors_timer_handler);
  APP_ERROR_CHECK(err_code);

  /* Create update Sensor characteristics timer */
  err_code = app_timer_create(&m_update_char_timer_id, APP_TIMER_MODE_REPEATED, update_characteristics_timer_handler);
  APP_ERROR_CHECK(err_code);
}

/* Slow read sensors timer start */
void read_sensor_timer_start(bool fast) {
  ret_code_t err_code;
  if (fast)
  {
    err_code = app_timer_start(m_read_sensors_timer_id, SENSOR_READ_INTERVAL_FAST_RUN, &fast);
  } else {
    err_code = app_timer_start(m_read_sensors_timer_id, sensor_read_interval, NULL);
  }
  APP_ERROR_CHECK(err_code);
}

/* Slow read sensors timer stop */
void read_sensor_timer_stop()
{
  ret_code_t err_code;
  err_code = app_timer_stop(m_read_sensors_timer_id);
  APP_ERROR_CHECK(err_code);
}

/* Periodical Update Sensor Characteristic timer start */
static void update_sensor_characteristic_timer_start()
{
  ret_code_t err_code;
  err_code = app_timer_start(m_update_char_timer_id, CH_UPDATE_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_INFO("Periodical Update Sensor Characteristic timer start");
}

/* Periodical Update Sensor Characteristic timer stop */
static void update_sensor_characteristic_timer_stop()
{
  ret_code_t err_code;
  err_code = app_timer_stop(m_update_char_timer_id);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_INFO("Periodical Update Sensor Characteristic timer stop");
}

/* Periodical Update Sensor Characteristic timer start */
void update_sensor_characteristic_start(sensor_t sensor_id)
{
  if (!is_any_sensor_notify())
  {
    read_sensor_timer_stop();
    update_sensor_characteristic_timer_start();
  }
  set_sensor_notify(sensor_id, true);
}

/* Periodical Update Sensor Characteristic timer stop */
void update_sensor_characteristic_stop(sensor_t sensor_id)
{
  set_sensor_notify(sensor_id, false);

  if (!is_any_sensor_notify())
  {
    update_sensor_characteristic_timer_stop();
    read_sensor_timer_start(NULL);
  }
}

static uint64_t app_timer_ms_to_tik(uint32_t ms)
{
  return ms * APP_TIMER_CLOCK_FREQ / 1000 * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1);
}

void set_scan_interval(uint16_t new_interval)
{
  uint64_t tik = app_timer_ms_to_tik(new_interval * 1000);

  if (tik > 0) {
    sensor_read_interval = tik;
  }
}

/***
** Created by Aleksey Volkov on 16/11/2018.
***/

#include <libraries/timer/app_timer.h>
#include <ble_bas.h>
#include <ble_services.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrfx_saadc.h"
#include "nrfx_timer.h"
#include "nrfx_rtc.h"
#include "nrfx_clock.h"
#include "nrfx_ppi.h"

#include "bat_sensor.h"
#include "ec_sensor.h"
#include "ntc_temperature_sensor.h"
#include "sensors.h"
#include "timers.h"

uint8_t sensor_index           = 0;
bool battery_sensor_ready      = false;
bool ec_sensor_ready           = false;
bool temperature_sensor_ready  = false;

bool battery_sensor_notify     = false;
bool temperature_sensor_notify = false;
bool salinity_sensor_notify    = false;


sensors_db_t sensors_db = {0, 0, 0};

/* ToDo: save sensor values */
void store_sensor_to_db(int16_t temp, uint16_t hum, uint16_t soil, uint16_t conductivity)
{
  sensors_db.sensor_values_changed = false;

  if (temp != sensors_db.temperature)
  {
    sensors_db.temperature = temp;
    sensors_db.sensor_values_changed = true;
  }

  if (conductivity != sensors_db.conductivity_us)
  {
    sensors_db.conductivity_us = conductivity;
    sensors_db.sensor_values_changed = true;
  }

}

/* Slow read sensors timer handler */
void read_sensors_timer_handler(void *p_context)
{
  if(p_context)
  {
    /* ALL sensors was asked and ready, update advertising data and setup timer to delay */
    if (all_sensors_ready())
    {
      /* restart read after delay 30 min */
      read_sensor_timer_stop();
      read_sensor_timer_start(false);

      NRF_LOG_INFO("update advertising data");

      /* update advertising data */
      advertising_update();

      /* reset sensors state */
      reset_all_sensors();
      return;
    } else {
      /* NOT all sensors ready, continue ask sensors */
      NRF_LOG_INFO("-------------- READ SENSOR --------------");
      read_sensors();
    }
  } else {
    NRF_LOG_INFO("-------------- DELAY ELAPSED ------------");
    /* delay elapsed, setup timer to fast read sensors */
    read_sensor_timer_stop();
    read_sensor_timer_start(true);
  }

}

void update_advertising_timer_handler(void *p_context)
{
  UNUSED_PARAMETER(p_context);
  if (all_sensors_ready())
  {
    advertising_update();
  }
}

/* update characteristics timer handler */
void update_characteristics_timer_handler(void *p_context) {
  UNUSED_PARAMETER(p_context);

  /* read only one sensor, and SET Sensor Ready */
  read_sensors();

  /* update only one sensor, readied in previous call */
  if (battery_sensor_notify && sensor_ready(BATTERY_SENSOR))
  {
    update_sensors_service(BATTERY_SENSOR);
    reset_sensor(BATTERY_SENSOR);
  }
  else if (temperature_sensor_notify && sensor_ready(TEMPERATURE_SENSOR))
  {
    update_sensors_service(TEMPERATURE_SENSOR);
    reset_sensor(TEMPERATURE_SENSOR);
  }
  else if (salinity_sensor_notify && sensor_ready(SALINITY_SENSOR))
  {
    update_sensors_service(SALINITY_SENSOR);
    reset_sensor(SALINITY_SENSOR);
  }

}

/* Sensors Order
 * 0 - battery SAADC
 * 1 - salinity SAADC
 * 2 - temp SAADC
 * */
void read_sensors() {
  NRF_LOG_INFO("#Sensor %d", sensor_index);

  if (nrfx_saadc_is_busy()) {
    NRF_LOG_INFO("SAADC is Busy");
    return;
  }

  if (sensor_index == 0) {
    read_battery_voltage();
    sensor_index++;
  } else if (sensor_index == 1) {
    read_ntc_sensor();
    sensor_index++;
  } else if (sensor_index == 2) {
    read_conductivity();
    sensor_index = 0;
  }
}

void set_sensor_ready(sensor_ready_t sensor)
{
  switch (sensor)
  {
    case BAT_SENSOR_READY:
      battery_sensor_ready = true;
      break;

    case EC_SENSOR_READY:
      ec_sensor_ready = true;
      break;

    case TEMPERATURE_SENSOR_READY:
      temperature_sensor_ready = true;
      break;

    default:
      break;
  }
}

/* Check if sensor data ready to read */
bool sensor_ready(sensor_t sensor_id)
{
  switch (sensor_id)
  {
    case BATTERY_SENSOR: return battery_sensor_ready;
    case TEMPERATURE_SENSOR: return temperature_sensor_ready;
    case SALINITY_SENSOR: return ec_sensor_ready;
    default: return false;
  }
}

bool all_sensors_ready()
{
  return ec_sensor_ready && temperature_sensor_ready;
}

void reset_sensor(sensor_t sensor_id)
{
  switch (sensor_id)
  {
    case BATTERY_SENSOR:
      battery_sensor_ready = false;
      break;
    case TEMPERATURE_SENSOR:
      temperature_sensor_ready = false;
      break;
    case SALINITY_SENSOR:
      ec_sensor_ready = false;
      break;
    default:
      break;
  }
}

void reset_all_sensors()
{
  battery_sensor_ready = false;
  ec_sensor_ready = false;
  temperature_sensor_ready = false;
}

bool is_any_sensor_notify()
{
  if (battery_sensor_notify || temperature_sensor_notify || salinity_sensor_notify)
    return true;
  else
    return false;
}

void set_sensor_notify(sensor_t sensor_id, bool notify)
{
  switch (sensor_id)
  {
    case BATTERY_SENSOR:
      battery_sensor_notify = notify;
      break;
    case TEMPERATURE_SENSOR:
      temperature_sensor_notify = notify;
      break;
    case SALINITY_SENSOR:
      salinity_sensor_notify = notify;
      break;
  }
}
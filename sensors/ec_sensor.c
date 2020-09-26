/***
** Created by Aleksey Volkov on 14/11/2018.
***/

#include <libraries/timer/app_timer.h>
#include <boards.h>
#include <ble_bas.h>
#include <sensors/include/ntc_temperature_sensor.h>
#include <sensors/include/bat_sensor.h>
#include <include/storage.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrfx_saadc.h"
#include "nrfx_timer.h"
#include "nrfx_rtc.h"
#include "nrfx_clock.h"
#include "nrfx_ppi.h"
#include "nrfx_gpiote.h"

#include "ble_cus.h"
#include "ble_services.h"
#include "pwm.h"
#include "timers.h"
#include "storage.h"
#include "ec_sensor.h"
#include "sensors.h"

/* Macro to convert the result of ADC conversion in millivolts. */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_MV) / ADC_RES_12BIT) * ADC_COMPENSATION)

#define SAMPLES_IN_BUFFER     8
#define ADC_COMPENSATION      6
#define ADC_RES_12BIT         4095
#define ADC_REF_VOLTAGE_MV    600
#define SINK_PIN_RESISTANCE   79
#define TEMPERATURE_COMP      0.02f
#define OPEN_CIRCUITS_MV_DROP 12

/* cell constant K = distance between the electrodes [m] / effective area of the electrodes [m2] */
#define PROBE_K               1

/* ADC */
static nrf_saadc_value_t      m_buffer_pool[2][SAMPLES_IN_BUFFER];
uint16_t ec_milli_volts     = 0;
uint16_t resistance         = 0;

/* REF calibration data */
uint16_t low_us             = 0;
uint16_t mid_us             = 1000;
uint16_t hi_us              = 10000;

/* Calibration mode */
bool need_low_calibration   = false;
bool need_mid_calibration   = false;
bool need_hi_calibration    = false;

/* PPI */
static nrf_ppi_channel_t  m_ppi_channel_ec1;    /* EC_POWER_PIN toggle */
static nrf_ppi_channel_t  m_ppi_channel_ec2;    /* EC_SINK_PIN toggle */
static nrf_ppi_channel_t  m_ppi_channel_saadc;  /* SAADC Sample Start */

static const nrfx_timer_t     m_timer = NRFX_TIMER_INSTANCE(1);

static void timer_handler(nrf_timer_event_t event_type, void * p_context) {}

/************************************************************
 * (Ground) ----\/\/\/-------|-------\/\/\/---- V_supply Pin
 *            Test liquid    |     R Balance
 *                      Analog Pin
 ************************************************************/

/* Initial state:
 * POWER pin HIGH
 * SINK pin LOW
 *
 * Wait 250us -> start SAADC -> wait 250us -> change pins polarity -> wait 250us
 *
 *
 * */
static void saadc_sampling_event_init(void)
{
  ret_code_t err_code;

  nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
  timer_cfg.frequency = NRF_TIMER_FREQ_500kHz;
  timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
  err_code = nrfx_timer_init(&m_timer, &timer_cfg, timer_handler);
  APP_ERROR_CHECK(err_code);

  /* setup m_timer for compare event to 200us, start SAADC */
  uint32_t ticks_saadc = nrfx_timer_us_to_ticks(&m_timer, 200);
  nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks_saadc, true);

  /* setup m_timer for compare event to 400us, EC1 HIGHT, EC2 LOW */
  uint32_t ticks_gpio_reverse = nrfx_timer_us_to_ticks(&m_timer, 500);
  nrfx_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL1, ticks_gpio_reverse, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, true);

  /* setup m_timer for compare event to 600us to leave reversed pin */
//  uint32_t ticks_gpio_leave = nrfx_timer_us_to_ticks(&m_timer, 1500);
//  nrfx_timer_extended_compare(&m_timer,
//                              NRF_TIMER_CC_CHANNEL2,
//                              ticks_gpio_leave,
//                              NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK,
//                              true);

  /* GPIOTE Init */
  if (!nrfx_gpiote_is_init())
  {
    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);
  }

  /* GPIOTE Toggle task EC_POWER_PIN, initial HIGH */
  nrfx_gpiote_out_config_t config_ec1 = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(true);
  err_code = nrfx_gpiote_out_init(EC_POWER_PIN, &config_ec1);
  APP_ERROR_CHECK(err_code);

  /* GPIOTE Toggle task EC_SINK_PIN, initial LOW */
  nrfx_gpiote_out_config_t config_ec2 = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);
  err_code = nrfx_gpiote_out_init(EC_SINK_PIN, &config_ec2);
  APP_ERROR_CHECK(err_code);

  /* SAADC
   * PPI Channel */
  uint32_t timer_compare_event_ch0_addr     = nrfx_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
  uint32_t saadc_sample_task_addr           = nrfx_saadc_sample_task_get();

  /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
  err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_saadc);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_assign(m_ppi_channel_saadc, timer_compare_event_ch0_addr, saadc_sample_task_addr);
  APP_ERROR_CHECK(err_code);

  /* GPIOTE */
  /* setup ppi channel for EC_POWER_PIN toggle */
  err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_ec1);
  APP_ERROR_CHECK(err_code);

  /* setup ppi channel for EC_SINK_PIN toggle */
  err_code = nrfx_ppi_channel_alloc(&m_ppi_channel_ec2);
  APP_ERROR_CHECK(err_code);

  /* EC_POWER_PIN task address */
  uint32_t ec1_gpiote_task_addr             = nrfx_gpiote_out_task_addr_get(EC_POWER_PIN);

  /* EC_SINK_PIN task address */
  uint32_t ec2_gpiote_task_addr             = nrfx_gpiote_out_task_addr_get(EC_SINK_PIN);

  /* toggle event
   * EC_POWER_PIN -> LOW -> HI ....
   * EC_SINK_PIN -> HI -> LOW .... */
  uint32_t timer_compare_event_ch1_addr     = nrfx_timer_event_address_get(&m_timer, NRF_TIMER_EVENT_COMPARE1);

  /* Assign toggle for EC_POWER_PIN on timer CC1 */
  err_code = nrfx_ppi_channel_assign(m_ppi_channel_ec1, timer_compare_event_ch1_addr, ec1_gpiote_task_addr);
  APP_ERROR_CHECK(err_code);

  /* Assign toggle for EC_SINK_PIN on timer CC1 */
  err_code = nrfx_ppi_channel_assign(m_ppi_channel_ec2, timer_compare_event_ch1_addr, ec2_gpiote_task_addr);
  APP_ERROR_CHECK(err_code);

  nrfx_gpiote_out_task_enable(EC_POWER_PIN);
  nrfx_gpiote_out_task_enable(EC_SINK_PIN);

}

static void saadc_sampling_event_deinit(void)
{
  ret_code_t err_code;

  err_code = nrfx_ppi_channel_free(m_ppi_channel_saadc);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_free(m_ppi_channel_ec1);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_free(m_ppi_channel_ec2);
  APP_ERROR_CHECK(err_code);

  nrfx_gpiote_out_task_disable(EC_POWER_PIN);
  nrfx_gpiote_out_task_disable(EC_SINK_PIN);

  nrfx_gpiote_out_uninit(EC_POWER_PIN);
  nrfx_gpiote_out_uninit(EC_SINK_PIN);

  nrfx_timer_uninit(&m_timer);
}

static void saadc_sampling_event_enable(void)
{
  ret_code_t err_code = nrfx_ppi_channel_enable(m_ppi_channel_saadc);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_enable(m_ppi_channel_ec1);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_enable(m_ppi_channel_ec2);
  APP_ERROR_CHECK(err_code);

  nrfx_timer_enable(&m_timer);
}

static void saadc_sampling_event_disable(void)
{
  ret_code_t err_code;

  err_code = nrfx_ppi_channel_disable(m_ppi_channel_saadc);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_disable(m_ppi_channel_ec1);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_disable(m_ppi_channel_ec2);
  APP_ERROR_CHECK(err_code);
}


static void saadc_callback(nrfx_saadc_evt_t const * p_event)
{
  if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE)
  {
    ret_code_t err_code;
    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("EC SAADC calibration complete!");

    saadc_sampling_event_enable();

  }
  else if (p_event->type == NRFX_SAADC_EVT_DONE)
  {
    ret_code_t err_code;

    err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    int i;
//    uint32_t temp = 0;
    for (i = 0; i < SAMPLES_IN_BUFFER; i++)
    {
      NRF_LOG_INFO("EC RAW SAADC %d", p_event->data.done.p_buffer[i]);
//      temp += p_event->data.done.p_buffer[i];
    }

//    temp = temp/(SAMPLES_IN_BUFFER/2);
//    NRF_LOG_INFO("EC RAW Accumulated %d", temp);

//    ec_milli_volts = (uint16_t)ADC_RESULT_IN_MILLI_VOLTS(temp);
    ec_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(p_event->data.done.p_buffer[2]);
    ec_milli_volts += ADC_RESULT_IN_MILLI_VOLTS(p_event->data.done.p_buffer[4]);
    ec_milli_volts += ADC_RESULT_IN_MILLI_VOLTS(p_event->data.done.p_buffer[6]);
    ec_milli_volts = ec_milli_volts / 3;

    float volts = (float)ec_milli_volts/1000;
    NRF_LOG_INFO("EC volts " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(volts));
    if (ec_milli_volts != 0) {
      uint16_t battery = get_battery_milli_volts() - OPEN_CIRCUITS_MV_DROP;
      resistance = 1000 / ((double)battery / ec_milli_volts - 1) - SINK_PIN_RESISTANCE;
    }


    NRF_LOG_INFO("EC milli volts %d", ec_milli_volts);
    NRF_LOG_INFO("Resistance %d", resistance);
    NRF_LOG_INFO("Salinity RAW %d", get_raw_salinity());
    NRF_LOG_INFO("Salinity     %d", get_salinity());
    set_sensor_ready(EC_SENSOR_READY);

    ec_sensor_deinit();

    finish_ec_calibration();
  }

}

/* SAADC setup for EC calculation */
static void ec_saadc_init(void)
{
  nrfx_saadc_uninit();

  ret_code_t err_code;
  nrf_saadc_channel_config_t channel_config =
      NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
  channel_config.burst = NRF_SAADC_BURST_ENABLED;

  /* Configure SAADC */
  nrfx_saadc_config_t saadc_config;
  saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;        //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=4096 (when input voltage is 3.6V for channel gain setting of 1/6).
  saadc_config.oversample = NRF_SAADC_OVERSAMPLE_16X;          //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
  saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;

  err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_channel_init(0, &channel_config);
  APP_ERROR_CHECK(err_code);

  /* Start calibration */
  nrfx_saadc_calibrate_offset();

//  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
//  APP_ERROR_CHECK(err_code);
//
//  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
//  APP_ERROR_CHECK(err_code);
}

void ec_sensor_init(void)
{
  ec_saadc_init();
  saadc_sampling_event_init();
//  saadc_sampling_event_enable();
}

void ec_sensor_deinit(void)
{
  saadc_sampling_event_disable();
  saadc_sampling_event_deinit();

  nrf_gpio_cfg_input(EC_POWER_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(EC_SINK_PIN, NRF_GPIO_PIN_NOPULL);

  nrf_gpio_input_disconnect(EC_POWER_PIN);
  nrf_gpio_input_disconnect(EC_SINK_PIN);

  nrfx_saadc_uninit();
  NVIC_ClearPendingIRQ(SAADC_IRQn);
}

/* EC sensor ms calculation
 * cell constant K = distance between the electrodes [m] / effective area of the electrodes [m2])
 * effective area of the electrodes [m2] = 3.14*r^2+2*3.14*h
 *
 * resistance = Resistor * (K / ((inputV / outputV) - 1));
 * mS = ((100000 * K) / resistance);
 * */
void read_conductivity()
{
  ec_sensor_init();
}

/*
 * https://en.wikipedia.org/wiki/Conductivity_%28electrolytic%29
 * https://www.omega.co.uk/techref/ph-2.html
 *
 * Return temperature compensated value in uS/sm, micro siemens
 * */
double get_raw_salinity()
{
  double uS = 0.0f;
  if (resistance > 0 && resistance < 65535)
  {
    uS = (double)(1000000 * PROBE_K) / resistance; /* 1 μS/cm = 100 μS/m;  10^6 μS/cm = 10^3 mS/cm = 1 S/cm */
  }

  return uS;
}

/* Return salinity in uS compensated by temp and corrected by one or two point calibration */
uint16_t get_salinity()
{
  double raw_salinity   = get_raw_salinity();
  double temperature    = get_ntc_temperature();
  double true_salinity  = 0;

  /* use one point zero offcet */
  if (raw_salinity <= 200)
  {
    raw_salinity = raw_salinity - low_us;
  }
  /* use two point calibration low_us (0uS) and mid_us (1000uS) */
  else if (raw_salinity <= 2000)
  {
    double reference_range = 1000;
    double raw_range       = mid_us - low_us;

    raw_salinity = (((raw_salinity - low_us) * reference_range) / raw_range) + 0;
  }
  /* use two point calibration mid_us (1000uS) and hi_us (10000uS) */
  else if (raw_salinity <= 20000)
  {
    double reference_range = 9000;
    double raw_range       = hi_us - mid_us;

    raw_salinity = (((raw_salinity - mid_us) * reference_range) / raw_range) + 1000;
  }

  /* temperature compensation */
  true_salinity = raw_salinity + (raw_salinity * ((25 - temperature) * TEMPERATURE_COMP));
  return (uint16_t)true_salinity;
}

/* calibration: initial step */
void do_ec_calibration(ec_calibration_mode_t mode)
{
  /* read all sensors to proper EC calculation */
  read_sensor_timer_stop();
  read_sensor_timer_start(true);

  need_low_calibration = false;
  need_mid_calibration = false;
  need_hi_calibration  = false;

  switch (mode)
  {
    case EC_CAL_LOW:
      need_low_calibration = true;
      led_indication_set(LED_INDICATE_FAST_RED);
      break;
    case EC_CAL_MID:
      need_mid_calibration = true;
      led_indication_set(LED_INDICATE_FAST_GREEN);
      break;
    case EC_CAL_HI:
      need_hi_calibration  = true;
      led_indication_set(LED_INDICATE_FAST_BLUE);
      break;
    default:
      break;
  }
}

/* calibration: final step
 * ToDo: temperature normalisation */
void finish_ec_calibration()
{
  double true_salinity  = 0;
  double raw_salinity  = get_raw_salinity();
  double temperature = get_ntc_temperature();

  /* temperature compensation */
  true_salinity = raw_salinity + (raw_salinity * ((25 - temperature) * TEMPERATURE_COMP));

  if (need_low_calibration)
  {
    low_us = true_salinity;
    need_low_calibration = false;
    NRF_LOG_INFO("EC Low Calibration new %d uS", low_us);

    /* save to nvram */
    set_settings(low_us, mid_us, hi_us);
    led_indication_set(LED_INDICATE_SUCCESS);
  }

  else if (need_mid_calibration)
  {
    mid_us = true_salinity;
    need_mid_calibration = false;
    NRF_LOG_INFO("EC Mid Calibration new %d uS", mid_us);

    /* save to nvram */
    set_settings(low_us, mid_us, hi_us);
    led_indication_set(LED_INDICATE_SUCCESS);
  }

  else if (need_hi_calibration)
  {
    hi_us = true_salinity;
    need_hi_calibration = false;
    NRF_LOG_INFO("EC Hi Calibration new %d uS", hi_us);

    /* save to nvram */
    set_settings(low_us, mid_us, hi_us);
    led_indication_set(LED_INDICATE_SUCCESS);
  }
}

/* return true if calibrating */
uint8_t get_ec_calibration_status()
{
  return need_low_calibration || need_mid_calibration || need_hi_calibration;
}

void set_ec_calibration(uint16_t low, uint16_t mid, uint16_t hi)
{
  if (low) { low_us = low; }
  if (mid) { mid_us = mid; }
  if (hi) { hi_us = hi; }
}
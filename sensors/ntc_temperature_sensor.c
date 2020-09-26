/***
** Created by Aleksey Volkov on 2019-02-17.
***/

#include <sensors/include/sensors.h>
#include <math.h>
#include "bsp/custom_board.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrfx_saadc.h"
#include "nrfx_timer.h"
#include "nrfx_rtc.h"
#include "nrf_gpio.h"
#include "nrfx_ppi.h"
#include "nrfx_gpiote.h"

#include "bat_sensor.h"
#include "ntc_temperature_sensor.h"

/* Macro to convert the result of ADC conversion in millivolts. */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_MV) / ADC_RES_12BIT) * ADC_COMPENSATION)

#define SAMPLES_IN_BUFFER     2
#define ADC_COMPENSATION      6
#define ADC_RES_12BIT         4095
#define ADC_REF_VOLTAGE_MV    600
#define BALANCE_RESISTOR      10000
#define BETA                  3950
#define ROOM_TEMP             298.15      /* room temperature in Kelvin 273.15 + 25 */
#define NTC_25T_RESISTANCE    10000       /* NTC resistance when temperature is 25 Celsius */

/* ADC */
static nrf_saadc_value_t      m_buffer_pool[2][SAMPLES_IN_BUFFER];
uint16_t ntc_milli_volts    = 0;

/* PPI */
static nrf_ppi_channel_t      m_ppi_channel;
static const nrfx_timer_t     m_timer = NRFX_TIMER_INSTANCE(1);

static void timer_handler(nrf_timer_event_t event_type, void * p_context) {}

/*
 * (Ground) ----\/\/\/-------|-------\/\/\/---- V_supply Pin
 *            R_balance      |    R_thermistor
 *                      Analog Pin
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

  /* setup m_timer for compare event every 1000us - 1 khz */
  uint32_t ticks = nrfx_timer_us_to_ticks(&m_timer, 2000);
  nrfx_timer_extended_compare(&m_timer,
                              NRF_TIMER_CC_CHANNEL0,
                              ticks,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                              true);
  nrfx_timer_enable(&m_timer);

  uint32_t timer_compare_event_addr = nrfx_timer_compare_event_address_get(&m_timer,
                                                                           NRF_TIMER_CC_CHANNEL0);
  uint32_t saadc_sample_task_addr   = nrfx_saadc_sample_task_get();

  /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
  err_code = nrfx_ppi_channel_alloc(&m_ppi_channel);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_assign(m_ppi_channel,
                                     timer_compare_event_addr,
                                     saadc_sample_task_addr);
  APP_ERROR_CHECK(err_code);
}

static void saadc_sampling_event_enable(void)
{
  ret_code_t err_code = nrfx_ppi_channel_enable(m_ppi_channel);

  APP_ERROR_CHECK(err_code);
}

static void saadc_sampling_event_disable(void)
{
  nrfx_timer_disable(&m_timer);
  nrfx_timer_uninit(&m_timer);

  ret_code_t err_code = nrfx_ppi_channel_disable(m_ppi_channel);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_free(m_ppi_channel);
  APP_ERROR_CHECK(err_code);
}

static void saadc_callback(nrfx_saadc_evt_t const * p_event)
{
  if (p_event->type == NRFX_SAADC_EVT_DONE)
  {
    ret_code_t err_code;

    err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    for (uint8_t i = 0; i < p_event->data.done.size; i++)
    {
      NRF_LOG_INFO("NTC SAADC RAW   %d", p_event->data.done.p_buffer[i]);
    }

    ntc_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(p_event->data.done.p_buffer[1]);

    NRF_LOG_INFO("NTC  mv : %d", ntc_milli_volts);
    NRF_LOG_INFO("NTC Temp: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(get_ntc_temperature()));

    ntc_sensor_deinit();
    set_sensor_ready(TEMPERATURE_SENSOR_READY);

  }
}

/* SAADC setup for BATTERY calculation */
static void ntc_saadc_init() {
  nrfx_saadc_uninit();

  ret_code_t err_code;
  nrf_saadc_channel_config_t channel_config =
      NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);

  /* Configure SAADC */
  nrfx_saadc_config_t saadc_config;
//  saadc_config.low_power_mode = true;
  saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;       //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
  saadc_config.oversample = NRF_SAADC_OVERSAMPLE_16X;          //Set oversample to 32x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 128 times.
  saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;

  err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
  APP_ERROR_CHECK(err_code);

  channel_config.burst = NRF_SAADC_BURST_ENABLED;
  err_code = nrfx_saadc_channel_init(0, &channel_config);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
  APP_ERROR_CHECK(err_code);

}

void ntc_sensor_init(void)
{
  nrf_gpio_cfg_output(NTC_POWER_PIN);
  nrf_gpio_pin_set(NTC_POWER_PIN);

  ntc_saadc_init();
  saadc_sampling_event_init();
  saadc_sampling_event_enable();
}

void ntc_sensor_deinit(void)
{
  saadc_sampling_event_disable();
  NVIC_ClearPendingIRQ(SAADC_IRQn);
  nrfx_saadc_uninit();

  nrf_gpio_cfg_input(NTC_POWER_PIN, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_input_disconnect(NTC_POWER_PIN);

}

void read_ntc_sensor()
{
  ntc_sensor_init();
}

/* get Battery voltage */
void read_ntc_temperature()
{
  ntc_sensor_init();
}

double get_sd_temperature()
{
  static int32_t temperature;
  ret_code_t err_code;

  err_code = sd_temp_get(&temperature); /* in points of 0.25`C, need devise by 4 */
  APP_ERROR_CHECK(err_code);

  return (double)temperature / 4.0;
}

double get_ntc_temperature()
{
  double battery_milli_volts = get_battery_milli_volts();
  if (battery_milli_volts == 0)
    battery_milli_volts = 3300.0;

  /* wrong data from ntc sensor */
  if (ntc_milli_volts > battery_milli_volts || ntc_milli_volts == 0)
  {
    return get_sd_temperature();
  }
  /* Here we calculate the thermistor’s resistance */
  double r_thermistor = BALANCE_RESISTOR * ( (float)(battery_milli_volts / ntc_milli_volts) - 1);

  double temperature_kelvin = (BETA * ROOM_TEMP) /
            (BETA + ROOM_TEMP * log(r_thermistor / NTC_25T_RESISTANCE));

  double temperature_celsius = temperature_kelvin - 273.15;  // convert kelvin to celsius

  return temperature_celsius;
}
/***
** Created by Aleksey Volkov on 2019-02-15.
***/

#include "nrfx_gpiote.h"
#include "nrf_log.h"
#include "app_timer.h"
#include "app_button.h"
#include "custom_board.h"
#include "button.h"

#define BTN_DEBOUNCE_TIME             APP_TIMER_TICKS(200)  /* milliseconds */

static void debounce_timer_handler(void *p_context);
static void buttons_debounce_start();

APP_TIMER_DEF(m_debounce_timer_id);

/* long press 2 sec */
button_t button = {IDLE, IDLE, 10, NULL};

static void debounce_timer_handler(void *p_context) {
  UNUSED_PARAMETER(p_context);

  static uint8_t long_press_counter = 0;

  /* button still pressed > 200 msec. */
  if (!nrf_gpio_pin_read(BTN_PIN))
  {
    NRF_LOG_INFO("BTN debounce still pressed > %u msec.", long_press_counter * 200);
    long_press_counter++;
    buttons_debounce_start();
  }
  else
  {
    /* button was released after short/long press */
    button.current_state = RELEASED;
    if (long_press_counter > 80)      /* > 20000 msec. */
    {
      long_press_counter = 0;
      button.current_state = LONG_LONG_PRESSED;
    }
    else if (long_press_counter > 8)  /* > 2000 msec. */
    {
      long_press_counter = 0;
      button.current_state = LONG_PRESSED;
    }

    /* Notify user function RELEASED State */
    if (button.action_handler != NULL)
    {
      button.action_handler(button.current_state);
    }
  }

//  nrfx_gpiote_in_event_enable(BTN_PIN, true);
}

static void buttons_debounce_start()
{
  ret_code_t err_code;
  err_code = app_timer_start(m_debounce_timer_id, BTN_DEBOUNCE_TIME, NULL);
  APP_ERROR_CHECK(err_code);
}

/* GPIOTE handler */
static void in_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if (pin == BTN_PIN)
  {
    /* temporary disable GPIOTE pin sense */
//    nrfx_gpiote_in_event_disable(BTN_PIN);

    bool state = !nrf_gpio_pin_read(BTN_PIN);
    NRF_LOG_INFO("BTN GPIOTE press event: %d", state);

    button.current_state = PRESSED;

    /* Start debounce timer */
    buttons_debounce_start();
  }
}

/* Function for the Button initialization. */
void button_init()
{
  ret_code_t err_code;

  /* Create debounce/long press timer */
  err_code = app_timer_create(&m_debounce_timer_id, APP_TIMER_MODE_SINGLE_SHOT, debounce_timer_handler);
  APP_ERROR_CHECK(err_code);

  if (!nrfx_gpiote_is_init())
  {
    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);
  }

  /* Button GPIOTE event on Hi state to Lo state change */
  nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  in_config.hi_accuracy = false; /* Low Power */
  in_config.pull = NRF_GPIO_PIN_PULLUP;

  err_code = nrfx_gpiote_in_init(BTN_PIN, &in_config, in_pin_handler);
  APP_ERROR_CHECK(err_code);

  nrfx_gpiote_in_event_enable(BTN_PIN, true);

}


void set_button_action_handler(button_action_t action_handler)
{
  button.action_handler = action_handler;
}
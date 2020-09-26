//
// Created by Алексей on 11/10/2018.
//

#ifndef PROJECT_PWM_H
#define PROJECT_PWM_H

#include <stdio.h>
#include "nrfx_pwm.h"

typedef enum {
  LED_INDICATE_NONE,
  LED_INDICATE_ADVERTISING,
  LED_INDICATE_ADVERTISING_SLOW,
  LED_INDICATE_IDLE,
  LED_INDICATE_CONNECTED,
  LED_INDICATE_SUCCESS,
  LED_INDICATE_ERROR,
  LED_INDICATE_FAST_RED,
  LED_INDICATE_FAST_GREEN,
  LED_INDICATE_FAST_BLUE,
  LED_INDICATE_MAX
} led_indication_t;

typedef struct {
  uint16_t r;
  uint16_t g;
  uint16_t b;
} rgb_t;

void init_led_pwm(void);
void led_blink(uint8_t count, nrf_pwm_sequence_t *sequence);

led_indication_t get_led_indication_mode();
void led_indication_set(led_indication_t mode);
void led_indication_set_color(uint16_t r, uint16_t g, uint16_t b, uint16_t w);

#endif //PROJECT_PWM_H

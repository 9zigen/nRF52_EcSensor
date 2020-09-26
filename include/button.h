/***
** Created by Aleksey Volkov on 2019-02-15.
***/

#ifndef PHSENSOR_BUTTON_H
#define PHSENSOR_BUTTON_H

typedef enum {
  IDLE, PRESSED, RELEASED, LONG_PRESSED, LONG_LONG_PRESSED
} button_state_t;

typedef void (*button_action_t)(button_state_t state);

typedef struct {
  button_state_t last_state;
  button_state_t current_state;
  uint8_t long_press_period;                /* in units of BTN_DEBOUNCE_TIME */
  button_action_t action_handler;
} button_t;

void button_init();
void set_button_action_handler(button_action_t action_handler);

#endif //PHSENSOR_BUTTON_H

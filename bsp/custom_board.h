/***
** Created by Aleksey Volkov on 23/11/2018.
***/

#ifndef ECSENSOR_CUSTOM_BOARD_H
#define ECSENSOR_CUSTOM_BOARD_H

#include "nrf_gpio.h"

/* RGB LED */
#define               LED_R         NRF_GPIO_PIN_MAP(0,24)
#define               LED_G         NRF_GPIO_PIN_MAP(0,23)
#define               LED_B         NRF_GPIO_PIN_MAP(0,22)

/* White LED */
#define               LED_W         NRF_GPIO_PIN_MAP(0,25)

/* Nordic BSP Lib support */
#define LEDS_NUMBER    4

#define LED_1         LED_R
#define LED_2         LED_G
#define LED_3         LED_B
#define LED_4         LED_W

#define LEDS_ACTIVE_STATE 0

#define LEDS_LIST { LED_1, LED_2, LED_3, LED_4 }

#define LEDS_INV_MASK  LEDS_MASK

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3
#define BSP_LED_3      LED_4

/* Button */
#define               BTN_PIN       NRF_GPIO_PIN_MAP(0,31)

/* Nordic BSP Buttons support */
#define BUTTONS_NUMBER 1

#define BUTTON_1       NRF_GPIO_PIN_MAP(1,31)
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

/* NTC Thermistor 10K */
#define               NTC_POWER_PIN NRF_GPIO_PIN_MAP(0,2)
#define               NTC_ADC_PIN   NRF_GPIO_PIN_MAP(0,3)

/* EC Sensor */
#define               EC_POWER_PIN  NRF_GPIO_PIN_MAP(0,4)
#define               EC_SINK_PIN   NRF_GPIO_PIN_MAP(0,6)

/* Serial Interface */
#define               TX_PIN        NRF_GPIO_PIN_MAP(0,14)
#define               RX_PIN        NRF_GPIO_PIN_MAP(0,13)

#endif //ECSENSOR_CUSTOM_BOARD_H

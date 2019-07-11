/***
** Created by Aleksey Volkov on 23/11/2018.
***/

#ifndef ECSENSOR_CUSTOM_BOARD_H
#define ECSENSOR_CUSTOM_BOARD_H

#include "nrf_gpio.h"

/* Button */
#define               BTN_PIN NRF_GPIO_PIN_MAP(0,31)

/* RGB LED */
#define               LED_R NRF_GPIO_PIN_MAP(0,24)
#define               LED_G NRF_GPIO_PIN_MAP(0,23)
#define               LED_B NRF_GPIO_PIN_MAP(0,22)
#define               LED_W NRF_GPIO_PIN_MAP(0,25)

/* EC Probe */
#define               EC_POWER_PIN NRF_GPIO_PIN_MAP(0,4)
#define               EC_SINK_PIN NRF_GPIO_PIN_MAP(0,6)

/* NTC Thermistor 10K */
#define               NTC_POWER_PIN NRF_GPIO_PIN_MAP(0,2)
#define               NTC_ADC_PIN   NRF_GPIO_PIN_MAP(0,3)

#endif //ECSENSOR_CUSTOM_BOARD_H

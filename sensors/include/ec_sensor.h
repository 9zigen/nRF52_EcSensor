/***
** Created by Aleksey Volkov on 14/11/2018.
***/

#ifndef SOILSENSOR_EC_SENSOR_H
#define SOILSENSOR_EC_SENSOR_H
#include <boards.h>

typedef enum {
  EC_CAL_LOW, EC_CAL_MID, EC_CAL_HI
} ec_calibration_mode_t;

/* */
void ec_sensor_init(void);
void ec_sensor_deinit(void);

/* start read sequence */
void read_conductivity();

/* get RAW current values */
double get_raw_salinity();

/* get calibrated + compensated values */
uint16_t get_salinity();

/* calibration */
void do_ec_calibration(ec_calibration_mode_t mode);
void finish_ec_calibration();
uint8_t get_ec_calibration_status();
void set_ec_calibration(uint16_t low, uint16_t mid, uint16_t hi);

#endif //SOILSENSOR_EC_SENSOR_H

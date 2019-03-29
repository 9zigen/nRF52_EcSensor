/***
** Created by Aleksey Volkov on 2019-02-17.
***/

#ifndef PHSENSOR_NTC_TEMPERATURE_SENSOR_H
#define PHSENSOR_NTC_TEMPERATURE_SENSOR_H

void ntc_sensor_deinit(void);
void read_ntc_sensor();
double ntc_get_temperature();

#endif //PHSENSOR_NTC_TEMPERATURE_SENSOR_H

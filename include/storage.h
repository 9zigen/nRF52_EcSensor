/***
** Created by Aleksey Volkov on 18.03.2020.
***/

#ifndef PHSENSOR_STORAGE_H
#define PHSENSOR_STORAGE_H

/* 4 bytes aligned */
typedef struct {
  uint32_t magic;
  uint32_t version;
  uint16_t low_us;        /* 0 uS/sm solution (pure H20) for range 0 - 200 uS */
  uint16_t mid_us;        /* 1000 uS/sm solution 491 mg/L NaCl for range 0 - 2000 uS */
  uint16_t hi_us;         /* ToDo: 10000 uS/sm solution for range 0 - 20000 uS */
  uint16_t scan_interval;
} settings_t;

uint16_t storage_init();
void set_settings(uint16_t low, uint16_t mid, uint16_t hi);
void factory_settings();
settings_t * get_settings();

#endif //PHSENSOR_STORAGE_H

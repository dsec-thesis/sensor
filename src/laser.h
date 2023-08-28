#ifndef __LASER_H__
#define __LASER_H__

#include "esp_err.h"

void laser_init();
esp_err_t laser_read_distance(uint16_t *distance);
bool laser_detect_nearby_object(uint16_t threshold);
void laser_shutdown();

#endif

#ifndef __QMC_H__
#define __QMC_H__

#include "esp_err.h"

esp_err_t qmc_init();
esp_err_t qmc_detect_anomaly(bool *detected, int bw);

#endif

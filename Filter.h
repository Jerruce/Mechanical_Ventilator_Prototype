#ifndef _FILTER_H_
#define _FILTER_H_

#include "AppDefinition.h"

void Kalman_init(KALMAN_PARAM *pKalman);
double Filtering_Kalman(KALMAN_PARAM *pKalman, double NewData);

#endif

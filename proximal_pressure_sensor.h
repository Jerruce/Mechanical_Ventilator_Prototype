
#ifndef PROXIMAL_PRESSURE_SENSOR_H_
#define PROXIMAL_PRESSURE_SENSOR_H_

#include "mbed.h"
#include "stdint.h"

/* Function declaration */
void Proximal_Pressure_Sensor_Initialize(void);
void Proximal_Pressure_Sensor_Read(void);
float Proximal_Pressure_Sensor_Get_Pressure(void);

#endif
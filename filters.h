#ifndef FILTERS_H_
#define FILTERS_H_

#include "stdint.h"

/* Function declaration */
void Filters_Initialize(void);
float Proximal_Pressure_Cheby2_LPF(float raw_value, uint8_t restart);
float Proximal_Flow_FIR_LPF(float raw_value, uint8_t restart);
float Water_Column_Pressure_Cheby2_LPF(float raw_value, uint8_t restart);
void Apply_Flow_LPF_Filter(float flow_signal);
float Get_Flow_LPF_Output(void);
void Apply_Pressure_LPF_Filter(float pressure_signal);
float Get_Pressure_LPF_Output(void);
void Apply_Water_Column_Pressure_LPF(float column_pressure_signal);
float Get_Water_Column_Pressure_LPF_Output(void);

float Flow_PID_FIR_LPF(float raw_value, uint8_t restart);
void Apply_Flow_PID_LPF_Filter(float flow_signal);
float Get_Flow_PID_LPF_Output(void);

#endif
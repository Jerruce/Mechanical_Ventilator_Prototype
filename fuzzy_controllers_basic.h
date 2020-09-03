

#ifndef FUZZY_CONTROLLERS_BASIC_H_
#define FUZZY_CONTROLLERS_BASIC_H_

/* File inclusion */
#include "stdint.h"

/* Function declaration */
void Flow_Fuzzy_Linear_PD_Controller(double error, double error_change, double *controller_output);
float Flow_Fuzzy_Incremental_Controller(float control_setpoint, float control_feedback, uint8_t control_reset);
void PIP_Fuzzy_Linear_PD_Controller(double error, double error_change, double *controller_output);
float PIP_Fuzzy_Inc_Controller(float control_setpoint, float control_feedback, uint8_t control_reset);
float PEEP_Fuzzy_Controller(float control_setpoint, float control_feedback, uint8_t control_reset);

void Flow_Fuzzy_Inc_Controller_Set_GE(float ge_value);
void Flow_Fuzzy_Inc_Controller_Set_GCE(float gce_value);
void Flow_Fuzzy_Inc_Controller_Set_GCU(float gcu_value);
void PIP_Fuzzy_Inc_Controller_Set_GE(float ge_value);
void PIP_Fuzzy_Inc_Controller_Set_GCE(float gce_value);
void PIP_Fuzzy_Inc_Controller_Set_GCU(float gcu_value);

#endif
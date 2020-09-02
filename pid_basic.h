#ifndef PID_BASIC_H_
#define PID_BASIC_H_

#include "stdint.h"

/* Function declaration */
void PID_Controllers_Initialize(void);

double Inspiration_Flow_PID_Controller(double setpoint, double feedback, uint8_t reset);
double PIP_PID_Controller(double setpoint, double feedback, uint8_t reset);
double PEEP_PID_Controller(double setpoint, double feedback, uint8_t reset);

void Inspiration_Flow_PID_Controller_Set_Kp(double kp_value);
void Inspiration_Flow_PID_Controller_Set_Ki(double ki_value);
void Inspiration_Flow_PID_Controller_Set_Kd(double kd_value);

void PIP_PID_Controller_Set_Kp(double kp_value);
void PIP_PID_Controller_Set_Ki(double ki_value);
void PIP_PID_Controller_Set_Kd(double kd_value);

void PEEP_PID_Controller_Set_Kp(double kp_value);
void PEEP_PID_Controller_Set_Ki(double ki_value);
void PEEP_PID_Controller_Set_Kd(double kd_value);


#endif /* PID_BASIC_H_ */
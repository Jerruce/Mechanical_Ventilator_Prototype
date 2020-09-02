
#ifndef VENTILATOR_ALARM_H_
#define VENTILATOR_ALARM_H_

#include "mbed.h"

extern DigitalOut  alarm_buzzer;
/* Function declaration */
void Alarm_Buzzer_Update(void);
void Alarm_Time_Count(void);

#endif
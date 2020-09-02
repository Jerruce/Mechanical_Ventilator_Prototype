#ifndef _TASK_GUI_H_
#define _TASK_GUI_H_

#include "mcu.h"

void Task_GUI(void);
void Task_Timers (void);
void InsertPoints(float *pFlow, float *pPressure, float *pVolumen);
void Nextion_SendCommamd(char *pCommand, int xTimeout);
void UpdateMaxValues(int Flow, int Pressure, int Volumen);

#endif /* _TASK_GUI_H_ */


#ifndef ELECTRIC_VALVES_H_
#define ELECTRIC_VALVES_H_

/* File inclusion */
#include "mbed.h"
#include "stdint.h"
#include "project_defines.h"


/* Function declaration*/

void Inspiration_On_Off_Valve_Initialize(void);
void Inspiration_Needle_Valve_Initialize(void);
void Expiration_Ball_Valve_Initialize(void);
void Electric_Valves_Initialize(void);

void Inspiration_On_Off_Valve_Open(void);
void Inspiration_On_Off_Valve_Close(void);

/* Functions adapted to the new code (stepper + encoder) */

void Inspiration_Needle_Valve_Write_Step_Setpoint(uint16_t setpoint);
void Inspiration_Needle_Valve_Go_To_Setpoint(void);
void Inspiration_Needle_Valve_Go_Home(void);

void Expiration_Ball_Valve_Write_Step_Setpoint(uint16_t setpoint);
void Expiration_Ball_Valve_Go_To_Setpoint(void);
void Expiration_Ball_Valve_Go_Home(void);

void Inspiration_Needle_Valve_Enable(void);
void Inspiration_Needle_Valve_Disable(void);

void Expiration_Ball_Valve_Enable(void);
void Expiration_Ball_Valve_Disable(void);



#endif

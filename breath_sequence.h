
#ifndef BREATH_SEQUENCE_H_
#define BREATH_SEQUENCE_H_

#include "stdint.h"

/* Type definition */
typedef enum{
    System_Set_Valves_Position = 0,
    System_Stand_By,
    
    System_VC_CMV_Exp_To_Ins_Transition,
    System_VC_CMV_Inspiration,
    System_VC_CMV_Ins_To_Exp_Transition,
    System_VC_CMV_Expiration,
    
    System_PC_CMV_Exp_To_Ins_Transition,
    System_PC_CMV_Inspiration,
    System_PC_CMV_Ins_To_Exp_Transition,
    System_PC_CMV_Expiration,   
    
    System_PC_CSV_Exp_To_Ins_Transition,
    System_PC_CSV_Inspiration,
    System_PC_CSV_Ins_To_Exp_Transition,
    System_PC_CSV_Expiration,  
    
}Breath_Phase_t;


typedef enum{
    VC_CMV_MODE = 0,
    PC_CMV_MODE,
    PC_CSV_MODE
}Breath_Mode_t;


/* Function declaration */
void Breath_System_Initialize(void);
void Breath_State_Machine(void);
void VC_CMV_State_Machine(void);
void PC_CMV_State_Machine(void);
void PC_CSV_State_Machine(void);
void Inspiration_Finish(void);
void Expiration_Finish(void);
void PIP_Valve_Open(void);

void Resp_Frequency_Setpoint_Update(uint8_t resp_freq_val);
void I_E_Ratio_Setpoint_Update(float ins_ratio_val, float exp_ratio_val);
void Tidal_Volume_Setpoint_Update(uint16_t vol_val);
void PIP_Setpoint_Update(uint8_t pip_val);
void PEEP_Setpoint_Update(uint8_t peep_val);
void Flow_Setpoint_Update(uint8_t flow_val);
void Time_And_Flow_Setpoint_Update(void);
void Breath_System_Set_Mode(Breath_Mode_t mode);
void Flow_PID_Tune(void);
void PIP_PID_Tune(void);
void Breath_System_Update_Parameters(void);
void Breath_System_Update_Peak_Values(void);
void Breath_System_Reset_Peak_Values(void);

float Breath_System_Get_Volume_Indicator_Value(void);
float Breath_System_Get_Flow_Indicator_Value(void);
float Breath_System_Get_Pressure_Indicator_Value(void);
float Breath_System_Get_Resp_Frequency_Setpoint(void);
float Breath_System_Get_Volume_Setpoint(void);
float Breath_System_Get_Flow_Setpoint(void);
Breath_Mode_t Breath_System_Get_Mode(void);
Breath_Phase_t Breath_System_Get_Phase(void);

uint16_t Insp_Needle_Valve_Flow_Percent_To_Steps(float percent);
uint16_t Insp_Needle_Valve_Pressure_Percent_To_Steps(float percent);
uint16_t Exp_Ball_Valve_Flow_Percent_To_Steps(float percent);

void Flow_Setpoint_Compensate(void);
void Flow_Setpoint_Compensate_2(void);



#endif
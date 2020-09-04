
/* FIle inclusion */
#include "mbed.h"
#include "stdint.h"
#include "math.h"
#include "project_defines.h"
#include "global.h"
#include "electric_valves.h"
#include "proximal_pressure_sensor.h"
#include "proximal_flow_sensor.h"
#include "filters.h"
#include "pid_basic.h"
#include "breath_sequence.h"
#include "fuzzy_controllers_basic.h"

DigitalOut led_prueba(PB_14);

/* Object definition */
Timeout inspiration_timer;
Timeout expiration_timer;
Timeout pip_valve_open_timer;

/* Variable definition */
extern volatile float valor_volumen_promedio, valor_error_volumen;// just for testing purposes
extern float flujo_pid;
extern uint8_t ventilator_start_stop;

Breath_Mode_t current_breath_mode = VC_CMV_MODE;
volatile Breath_Mode_t next_breath_mode = VC_CMV_MODE;
volatile Breath_Phase_t breath_phase = System_VC_CMV_Exp_To_Ins_Transition;

static float resp_frequency_setpoint_rpm;
static float insp_ratio, exp_ratio;
static float tidal_volume_setpoint_ml;
static float pip_setpoint_cm_h2o;
static float peep_setpoint_cm_h2o;

static float inspiration_time_setpoint_s, expiration_time_setpoint_s;
static float inspiration_flow_setpoint_slpm;

static float volume_peak_value = 0;
static float flow_peak_value = 0;
static float pressure_peak_value = 0;
static float volume_indicator_value = 0;
static float flow_indicator_value = 0;
static float pressure_indicator_value = 0;

static float volume_peak_average = 0;
static uint16_t volume_peak_counter;

                  
/* Function definition */

void Breath_System_Initialize(void){
    
    Resp_Frequency_Setpoint_Update(RESP_FREQUENCY_BY_DEFAULT_RPM);
    I_E_Ratio_Setpoint_Update(INSP_RATIO_BY_DEFAULT, EXP_RATIO_BY_DEFAULT);
    Tidal_Volume_Setpoint_Update(TIDAL_VOLUME_BY_DEFAULT_ML);
    PIP_Setpoint_Update(PIP_BY_DEFAULT_CM_H2O);
    PEEP_Setpoint_Update(PEEP_BY_DEFAULT_CM_H2O);
    Time_Setpoint_Update();
    Flow_Setpoint_Update_From_Time();
    Flow_PID_Tune();
    PID_Controllers_Initialize();
    //Electric_Valves_Initialize();
    system_flags |= (1 << ALARM_ENABLED_FLAG);
}


void Breath_State_Machine(void){

    /* Update the breath mode and phase */
    
    if(system_flags & (1 << BREATH_MODE_UPDATE_FLAG)){
        
        //__disable_irq();
        current_breath_mode = next_breath_mode;
        //__enable_irq();

        switch(current_breath_mode){
            case VC_CMV_MODE:
                breath_phase = System_VC_CMV_Exp_To_Ins_Transition;
            break;
            
            case PC_CMV_MODE:
                breath_phase = System_PC_CMV_Exp_To_Ins_Transition;
            break;
            
            case PC_CSV_MODE:
                breath_phase = System_PC_CSV_Exp_To_Ins_Transition;
            break;
            
            default:
                break;    
                    
        }        
        
        //__disable_irq();
        system_flags &= ~(1 << BREATH_MODE_UPDATE_FLAG);
        //__enable_irq();
           
    }
    
    /* Perform the corresponding action */
    
    switch(current_breath_mode){
        case VC_CMV_MODE:
            VC_CMV_State_Machine();
            break;
            
        case PC_CMV_MODE:
            PC_CMV_State_Machine();
            break;
            
        case PC_CSV_MODE:
            PC_CSV_State_Machine();
            break;
            
        default:
            break;            
    }
         
}


void VC_CMV_State_Machine(void){
  
    float current_flow_slpm;
    float current_pressure_cm_h2o;
    float inspiration_valve_setpoint;
    float expiration_valve_setpoint;
  
    /* Update peak values */
    Breath_System_Update_Peak_Values();

    /* The other actions depend on the breath phase */

    switch(breath_phase){

        case System_VC_CMV_Exp_To_Ins_Transition:

            if(ventilator_start_stop == 1){

                // Update indicator values
                volume_indicator_value = volume_peak_value;
                flow_indicator_value = flow_peak_value;
                pressure_indicator_value = pressure_peak_value;

                // Clear the PIP error flag and check for volume error
                system_flags &= ~(1 << PC_PIP_ERROR_FLAG);
                if((volume_peak_value > (tidal_volume_setpoint_ml + VOLUME_PEAK_ALARM_TOLERANCE_ML)) || (volume_peak_value < (tidal_volume_setpoint_ml - VOLUME_PEAK_ALARM_TOLERANCE_ML))){
                    system_flags |= (1 << VC_VOLUME_ERROR_FLAG);
                }else{
                    system_flags &= ~(1 << VC_VOLUME_ERROR_FLAG);
                }
                
                // Compensate the flow setpoint if required 
                //Flow_Setpoint_Compensate_2();
                // Reset peak value and volume measurement
                Breath_System_Reset_Peak_Values();
                Proximal_Flow_Sensor_Reset_Volume();
                // Update time and flow setpoint if required
                if(system_flags & (1 << PARAMETER_MODIFIED_FLAG)){
                    
                    // Resetpeak values
                    volume_peak_counter = 0;
                    volume_peak_average = 0;
                    // Recalculate inspiration time, expiration time and flow setpoint 
                    Time_Setpoint_Update();
                    //Flow_Setpoint_Update_From_Time();    
                    //Flow_PID_Tune();

                    //__disable_irq();
                    system_flags &= ~(1 << PARAMETER_MODIFIED_FLAG);
                    //__enable_irq();
                }
                // Open inspiration solenoid valve
                Inspiration_On_Off_Valve_Open();
                // Close completely the expiration needle valve
                Expiration_Ball_Valve_Write_Step_Setpoint(0);
                Expiration_Ball_Valve_Go_To_Setpoint();
                
                // Reset flow and PEEP controllers
                //Inspiration_Flow_PID_Controller(0.0, 0.0, 1);
                Flow_Fuzzy_Incremental_Controller(0.0, 0.0, 1);
                PEEP_Fuzzy_Controller(0.0, 0.0, 1);
                // Setup timer to count inspiration time
                inspiration_timer.attach(&Inspiration_Finish, inspiration_time_setpoint_s);
                breath_phase = System_VC_CMV_Inspiration;
    
            }else{
                breath_phase = System_Stand_By;
            }
            break;

            
         case System_VC_CMV_Inspiration:
         
            if(system_flags & (1 << PID_ACTION_UPDATE_FLAG)){
 
                 //current_flow_slpm = Get_Flow_PID_LPF_Output();
                current_flow_slpm = flujo_pid;              
                //inspiration_valve_setpoint = Inspiration_Flow_PID_Controller(inspiration_flow_setpoint_slpm, current_flow_slpm, 0);
                inspiration_valve_setpoint = Flow_Fuzzy_Incremental_Controller(inspiration_flow_setpoint_slpm, current_flow_slpm, 0);
                //inspiration_valve_setpoint = Insp_Needle_Valve_Flow_Percent_To_Steps(inspiration_valve_setpoint);
                //inspiration_valve_setpoint = Insp_Needle_Valve_Flow_Percent_To_Steps(0);
                Inspiration_Needle_Valve_Write_Step_Setpoint((uint16_t)inspiration_valve_setpoint);
                Inspiration_Needle_Valve_Go_To_Setpoint();
            
                //__disable_irq();
                system_flags &= ~(1 << PID_ACTION_UPDATE_FLAG);
                //__enable_irq();
            }
            
            if(system_flags & (1 << INSPIRATION_FINISH_FLAG)){
                breath_phase = System_VC_CMV_Ins_To_Exp_Transition;
                //__disable_irq();
                system_flags &= ~(1 << INSPIRATION_FINISH_FLAG);
                //__enable_irq();
            }
            
            break;
            
            
       case System_VC_CMV_Ins_To_Exp_Transition:

            Inspiration_On_Off_Valve_Close();

            //Inspiration_Needle_Valve_Write_Step_Setpoint(0);
            //Inspiration_Needle_Valve_Go_To_Setpoint();

            expiration_timer.attach(&Expiration_Finish, expiration_time_setpoint_s);
            breath_phase = System_VC_CMV_Expiration; 
            
            break;
            
         case System_VC_CMV_Expiration:

            float error_value;
            float pasos;

            if(system_flags & (1 << PID_ACTION_UPDATE_FLAG)){
                
                if((int)peep_setpoint_cm_h2o > 0){

                    current_pressure_cm_h2o = Get_Pressure_LPF_Output();              
                    //expiration_valve_setpoint = PEEP_PID_Controller(peep_setpoint_cm_h2o, current_pressure_cm_h2o, 0);
                    expiration_valve_setpoint = PEEP_Fuzzy_Controller(peep_setpoint_cm_h2o,current_pressure_cm_h2o, 0);
                    expiration_valve_setpoint = Exp_Ball_Valve_Flow_Percent_To_Steps(expiration_valve_setpoint);

                    Expiration_Ball_Valve_Write_Step_Setpoint((uint16_t)expiration_valve_setpoint);
                    Expiration_Ball_Valve_Go_To_Setpoint();
                }else{
                    Expiration_Ball_Valve_Write_Step_Setpoint(EXPIRATION_BALL_VALVE_OPEN_STEPS_LIMIT);
                    Expiration_Ball_Valve_Go_To_Setpoint();
                }
                
               //__disable_irq();
               system_flags &= ~(1 << PID_ACTION_UPDATE_FLAG);
               //__enable_irq();
            }

             if(system_flags & (1 << EXPIRATION_FINISH_FLAG)){
                breath_phase = System_VC_CMV_Exp_To_Ins_Transition;
                //__disable_irq();
                system_flags &= ~(1 << EXPIRATION_FINISH_FLAG);
                //__enable_irq();
            }        
    
            break;       

        case System_Stand_By:

            if(ventilator_start_stop == 1){
                breath_phase = System_VC_CMV_Exp_To_Ins_Transition;
            }else{
                breath_phase = System_Stand_By;
            }

            break;                   
    
        default:
            break;
    }
}


void PC_CMV_State_Machine(void){

    float current_pressure_cm_h2o;
    float inspiration_valve_setpoint;
    float expiration_valve_setpoint;

    /* Update peak values */
    Breath_System_Update_Peak_Values();

    /* The other actions depend on the breath phase */
  
    switch(breath_phase){
        case System_PC_CMV_Exp_To_Ins_Transition:

            if(ventilator_start_stop == 1){
                // Update indicator values
                volume_indicator_value = volume_peak_value;
                flow_indicator_value = flow_peak_value;
                pressure_indicator_value = pressure_peak_value;
                // Check for PIP error
                /* 
                system_flags &= ~(1 << VC_VOLUME_ERROR_FLAG);
                if((pressure_peak_value > (pip_setpoint_cm_h2o + PIP_ALARM_TOLERANCE_CM_H2O)) || (pressure_peak_value < (pip_setpoint_cm_h2o - PIP_ALARM_TOLERANCE_CM_H2O))){
                    system_flags |= (1 << PC_PIP_ERROR_FLAG);
                }else{
                    system_flags &= ~(1 << PC_PIP_ERROR_FLAG);
                }
                */
                // Reset peak value and volume measurement
                Breath_System_Reset_Peak_Values();
                Proximal_Flow_Sensor_Reset_Volume();
                // Update time and flow setpoint if required
                if(system_flags & (1 << PARAMETER_MODIFIED_FLAG)){
                    Time_Setpoint_Update();
                    //PIP_PID_Tune();

                    //__disable_irq();
                    system_flags &= ~(1 << PARAMETER_MODIFIED_FLAG);
                    //__enable_irq();
                }
                // Open inspiration solenoid valve
                Inspiration_On_Off_Valve_Open();
                // Close expiration needle valve
                Expiration_Ball_Valve_Write_Step_Setpoint(0);
                Expiration_Ball_Valve_Go_To_Setpoint();
                
                // Reset PID controllers
                PIP_PID_Controller(0.0, 0.0, 1);
                //PIP_Fuzzy_Incremental_Controller(0.0, 0.0, 1);
                PEEP_Fuzzy_Controller(0.0, 0.0, 1);
                // Setup timer to count inspiration time
                inspiration_timer.attach(&Inspiration_Finish, inspiration_time_setpoint_s);
                breath_phase = System_PC_CMV_Inspiration;
            }else{
                breath_phase = System_Stand_By;
            }
            led_prueba = 0;
            break;
            
         case System_PC_CMV_Inspiration:
         
            if(system_flags & (1 << PID_ACTION_UPDATE_FLAG)){
 
                current_pressure_cm_h2o = Get_Pressure_LPF_Output();              
                inspiration_valve_setpoint = PIP_PID_Controller(pip_setpoint_cm_h2o, current_pressure_cm_h2o, 0);
                //inspiration_valve_setpoint = PIP_Fuzzy_Incremental_Controller(pip_setpoint_cm_h2o, current_pressure_cm_h2o, 0);
                inspiration_valve_setpoint = Insp_Needle_Valve_Pressure_Percent_To_Steps(inspiration_valve_setpoint);
                Inspiration_Needle_Valve_Write_Step_Setpoint((uint16_t)inspiration_valve_setpoint);
                Inspiration_Needle_Valve_Go_To_Setpoint();

                //expiration_valve_setpoint = PIP_Release_Fuzzy_Controller(pip_setpoint_cm_h2o, current_pressure_cm_h2o, 0);
                //expiration_valve_setpoint = Exp_Ball_Valve_Flow_Percent_To_Steps(expiration_valve_setpoint);
                //Expiration_Ball_Valve_Write_Step_Setpoint((uint16_t)expiration_valve_setpoint);
                //Expiration_Needle_Valve_Go_To_Setpoint();
            
                //__disable_irq();
                system_flags &= ~(1 << PID_ACTION_UPDATE_FLAG);
                //__enable_irq();
            }
            
            if(system_flags & (1 << INSPIRATION_FINISH_FLAG)){
                breath_phase = System_PC_CMV_Ins_To_Exp_Transition;
                //__disable_irq();
                system_flags &= ~(1 << INSPIRATION_FINISH_FLAG);
                //__enable_irq();
            }
            led_prueba = 1;
            break;
            
            
       case System_PC_CMV_Ins_To_Exp_Transition:

            Inspiration_On_Off_Valve_Close();
            //pip_valve_open_timer.attach(&PIP_Valve_Open, expiration_time_setpoint_s/2.0);
            expiration_timer.attach(&Expiration_Finish, expiration_time_setpoint_s);
            breath_phase = System_PC_CMV_Expiration; 
            
            break;
            
         case System_PC_CMV_Expiration:

            float error_value;
            float pasos;

            if(system_flags & (1 << PID_ACTION_UPDATE_FLAG)){
                
                if((int)peep_setpoint_cm_h2o > 0){

                    current_pressure_cm_h2o = Get_Pressure_LPF_Output();              
                    //expiration_valve_setpoint = PEEP_PID_Controller(peep_setpoint_cm_h2o, current_pressure_cm_h2o, 0);
                    expiration_valve_setpoint = PEEP_Fuzzy_Controller(peep_setpoint_cm_h2o,current_pressure_cm_h2o, 0);
                    expiration_valve_setpoint = Exp_Ball_Valve_Flow_Percent_To_Steps(expiration_valve_setpoint);

                    Expiration_Ball_Valve_Write_Step_Setpoint((uint16_t)expiration_valve_setpoint);
                    Expiration_Ball_Valve_Go_To_Setpoint();
                }else{
                    Expiration_Ball_Valve_Write_Step_Setpoint(EXPIRATION_BALL_VALVE_OPEN_STEPS_LIMIT);
                    Expiration_Ball_Valve_Go_To_Setpoint();
                }
                
               //__disable_irq();
               system_flags &= ~(1 << PID_ACTION_UPDATE_FLAG);
               //__enable_irq();
            }


             if(system_flags & (1 << PIP_VALVE_OPEN_FLAG)){
                 if(pip_setpoint_cm_h2o >= 20.0){
                    Inspiration_Needle_Valve_Write_Step_Setpoint(INSPIRATION_NEEDLE_VALVE_OPEN_STEPS_LIMIT);
                 }else{
                     Inspiration_Needle_Valve_Write_Step_Setpoint(INSPIRATION_NEEDLE_VALVE_CLOSE_GUARD_LIMIT);
                 }

                 Inspiration_Needle_Valve_Go_To_Setpoint();
                 system_flags &= ~(1 << PIP_VALVE_OPEN_FLAG);
             }

             if(system_flags & (1 << EXPIRATION_FINISH_FLAG)){
                breath_phase = System_PC_CMV_Exp_To_Ins_Transition;
                //__disable_irq();
                system_flags &= ~(1 << EXPIRATION_FINISH_FLAG);
                //__enable_irq();
            }         
    
            break;                      

        case System_Stand_By:

            if(ventilator_start_stop == 1){
                breath_phase = System_PC_CMV_Exp_To_Ins_Transition;
            }else{
                breath_phase = System_Stand_By;
            }

            break;   

        default:
            break;
    }

}


void PC_CSV_State_Machine(void){

}


void Inspiration_Finish(void){
    system_flags |= (1 << INSPIRATION_FINISH_FLAG);
}


void Expiration_Finish(void){
    system_flags |= (1 << EXPIRATION_FINISH_FLAG);
}

void PIP_Valve_Open(void){
    system_flags |=(1 << PIP_VALVE_OPEN_FLAG);
}

void Resp_Frequency_Setpoint_Update(uint8_t resp_freq_val){

    if(resp_frequency_setpoint_rpm != resp_freq_val){
        system_flags |= (1 << PARAMETER_MODIFIED_FLAG);
    }

    if(resp_freq_val < RESP_FREQUENCY_MIN_RPM){
        resp_frequency_setpoint_rpm = RESP_FREQUENCY_MIN_RPM;
    }else if(resp_freq_val > RESP_FREQUENCY_MAX_RPM){
        resp_frequency_setpoint_rpm = RESP_FREQUENCY_MAX_RPM;
    }else{
        resp_frequency_setpoint_rpm = (float)resp_freq_val;
    }
    
}

void I_E_Ratio_Setpoint_Update(float ins_ratio_val, float exp_ratio_val){

    if((insp_ratio != ins_ratio_val) || (exp_ratio != exp_ratio_val)){
        system_flags |= (1 << PARAMETER_MODIFIED_FLAG);
    }

    insp_ratio = ins_ratio_val;
    exp_ratio = exp_ratio_val;
}

void Tidal_Volume_Setpoint_Update(uint16_t vol_val){

    if(tidal_volume_setpoint_ml != vol_val){
        system_flags |= (1 << PARAMETER_MODIFIED_FLAG);
    }

    if(vol_val < TIDAL_VOLUME_MIN_ML){
        tidal_volume_setpoint_ml = TIDAL_VOLUME_MIN_ML;
    }else if(vol_val > TIDAL_VOLUME_MAX_ML){
        tidal_volume_setpoint_ml = TIDAL_VOLUME_MAX_ML;
    }else{
        tidal_volume_setpoint_ml = (float)vol_val;
    }

}


void PIP_Setpoint_Update(uint8_t pip_val){

    if(pip_setpoint_cm_h2o != pip_val){
        system_flags |= (1 << PARAMETER_MODIFIED_FLAG);
    }

    //__disable_irq();
    if(pip_val < PIP_MIN_CM_H2O){
        pip_setpoint_cm_h2o = PIP_MIN_CM_H2O;
    }else if(pip_val > PIP_MAX_CM_H2O){
        pip_setpoint_cm_h2o = PIP_MAX_CM_H2O;
    }else{
        pip_setpoint_cm_h2o = (float)pip_val;
    }
    //__enable_irq();
}


void PEEP_Setpoint_Update(uint8_t peep_val){

    if(peep_setpoint_cm_h2o != peep_val){
        system_flags |= (1 << PARAMETER_MODIFIED_FLAG);
        system_flags |= (1 << PEEP_UPDATE_FLAG);
    }

    //__disable_irq();
    if(peep_val < PEEP_MIN_CM_H2O){
        peep_setpoint_cm_h2o = PEEP_MIN_CM_H2O;
    }else if(peep_val > PEEP_MAX_CM_H2O){
        peep_setpoint_cm_h2o = PEEP_MAX_CM_H2O;
    }else{
        peep_setpoint_cm_h2o = (float)peep_val;
    }

    //__enable_irq();
}    


void Flow_Setpoint_Update(uint8_t flow_val){

    //__disable_irq();
    if(flow_val < FLOW_MIN_SLPM){
        inspiration_flow_setpoint_slpm = FLOW_MIN_SLPM;
    }else if(flow_val > FLOW_MAX_SLPM){
        inspiration_flow_setpoint_slpm = FLOW_MAX_SLPM;
    }else{
        inspiration_flow_setpoint_slpm = (float)flow_val;
    }
    //__enable_irq();

}     
    


void Time_Setpoint_Update(void){

    float resp_period;

    if(resp_frequency_setpoint_rpm < 10.0){
        resp_frequency_setpoint_rpm = 10.0;
    }else if(resp_frequency_setpoint_rpm > 30.0){
        resp_frequency_setpoint_rpm = 30.0;
    }else{
        // Does nothing
    }

    /* Calculate period */
    resp_period = 60.0 / resp_frequency_setpoint_rpm;

    insp_ratio = 1;

    if(exp_ratio < 2.0){
        exp_ratio = 2.0;
    }else if(exp_ratio > 3.0){
        exp_ratio = 3.0;
    }else{
        // Does nothing
    }

    /* Calculate inspiration and expiration time */
    inspiration_time_setpoint_s = (resp_period * insp_ratio) / (insp_ratio + exp_ratio);
    expiration_time_setpoint_s = (resp_period * exp_ratio) / (insp_ratio + exp_ratio);
}


void Flow_Setpoint_Update_From_Time(void){

    /* Calculate required flow */
    //__disable_irq();
    inspiration_flow_setpoint_slpm = 0.06 * (tidal_volume_setpoint_ml / inspiration_time_setpoint_s);
    //__enable_irq();
}


uint16_t Insp_Needle_Valve_Flow_Percent_To_Steps(float percent){

    float steps_value;

    if(percent > 100.0){
        percent = 100;
    }else if(percent < 0.0){
        percent = 0.0;
    }else{
        //Does nothing
    }

    // Use the proper curve
    if(percent < 60){
        steps_value = (9.155765481 * percent) + 0.08572845;
    }else if(percent <= 100){
        steps_value = (18.19748862 * percent) - 523.381064;
    }else{
        steps_value = (float)INSPIRATION_NEEDLE_VALVE_OPEN_STEPS_LIMIT;
    }

    /* Ensure the valve does not close completely */
    if(steps_value < INSPIRATION_NEEDLE_VALVE_CLOSE_GUARD_LIMIT){
        steps_value = INSPIRATION_NEEDLE_VALVE_CLOSE_GUARD_LIMIT;
    }
    //return insp_needle_valve_flow_percent_table[percent];
    return (uint16_t)(steps_value * 2.5);

}


uint16_t Insp_Needle_Valve_Pressure_Percent_To_Steps(float percent){

    float steps_value;

    if(percent > 100.0){
        percent = 100;
    }else if(percent < 0){
        percent = 0.0;
    }else{
        //Does nothing
    }

    // Use the proper curve
    if(percent < 60){
        steps_value = (91.55765481 * sqrt(percent)) + 0.08572845;
    }else if(percent <= 100){
        steps_value = (181.9748862 * sqrt(percent)) - 523.381064;
    }else{
        steps_value = (float)INSPIRATION_NEEDLE_VALVE_OPEN_STEPS_LIMIT;
    }

    /* Ensure the valve does not close completely */
    if(steps_value < INSPIRATION_NEEDLE_VALVE_CLOSE_GUARD_LIMIT){
        steps_value = INSPIRATION_NEEDLE_VALVE_CLOSE_GUARD_LIMIT;
    }


    //return insp_needle_valve_pressure_percent_table[percent];
    return (uint16_t)(steps_value * 2.5);
}


void Breath_System_Set_Mode(Breath_Mode_t mode){

    if(next_breath_mode != mode){
        system_flags |= (1 << PARAMETER_MODIFIED_FLAG);
        system_flags |= (1 << BREATH_MODE_UPDATE_FLAG);
    }
      
    next_breath_mode = mode;
}

Breath_Mode_t Breath_System_Get_Mode(void){
    return current_breath_mode;
}

Breath_Phase_t Breath_System_Get_Phase(void){
    return breath_phase;
}


float Breath_System_Get_Resp_Frequency_Setpoint(void){
    return resp_frequency_setpoint_rpm;
}


float Breath_System_Get_Volume_Setpoint(void){
    return tidal_volume_setpoint_ml;
}

float Breath_System_Get_Flow_Setpoint(void){
    return inspiration_flow_setpoint_slpm;
}

void Breath_System_Update_Peak_Values(void){

    float aux;

    //aux = Get_Flow_LPF_Output();
    aux = flujo_pid;
    if(flow_peak_value < aux){
        flow_peak_value = aux;
    }

    aux = Proximal_Flow_Sensor_Get_Volume();
    if(volume_peak_value < aux){
        volume_peak_value = aux;
    }

    aux = Get_Pressure_LPF_Output();
    if(pressure_peak_value < aux){
        pressure_peak_value = aux;
    }
 
}


void Breath_System_Reset_Peak_Values(void){
    volume_peak_value = 0;
    flow_peak_value = 0;
    pressure_peak_value = 0;
}


float Breath_System_Get_Volume_Indicator_Value(void){
    return volume_indicator_value;
}


float Breath_System_Get_Flow_Indicator_Value(void){
    return flow_indicator_value;
}

float Breath_System_Get_Pressure_Indicator_Value(void){
    return pressure_indicator_value;
}

void Flow_PID_Tune(void){

if(inspiration_flow_setpoint_slpm < 5.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.17);
        Inspiration_Flow_PID_Controller_Set_Ki(0.6);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);
    }else if(inspiration_flow_setpoint_slpm < 10.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.18);
        Inspiration_Flow_PID_Controller_Set_Ki(0.6);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);
    }else if(inspiration_flow_setpoint_slpm < 15.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.13);
        Inspiration_Flow_PID_Controller_Set_Ki(0.7);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);
    }else if(inspiration_flow_setpoint_slpm < 20.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.13);
        Inspiration_Flow_PID_Controller_Set_Ki(0.7);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);
    }else if(inspiration_flow_setpoint_slpm < 25.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.11);
        Inspiration_Flow_PID_Controller_Set_Ki(0.7);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);
    }else if(inspiration_flow_setpoint_slpm < 30.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.1);
        Inspiration_Flow_PID_Controller_Set_Ki(0.7);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);
    }else if(inspiration_flow_setpoint_slpm < 35.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.1);
        Inspiration_Flow_PID_Controller_Set_Ki(0.6);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);
    }else if(inspiration_flow_setpoint_slpm < 40.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.1);
        Inspiration_Flow_PID_Controller_Set_Ki(0.55);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);
    }else if(inspiration_flow_setpoint_slpm < 45.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.08);
        Inspiration_Flow_PID_Controller_Set_Ki(0.62);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);
    }else if(inspiration_flow_setpoint_slpm < 50.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.08);
        Inspiration_Flow_PID_Controller_Set_Ki(0.61);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);
    }else if(inspiration_flow_setpoint_slpm < 55.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.08);
        Inspiration_Flow_PID_Controller_Set_Ki(0.61);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);
    }else if(inspiration_flow_setpoint_slpm < 60.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.08);
        Inspiration_Flow_PID_Controller_Set_Ki(0.55);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);
    }else if(inspiration_flow_setpoint_slpm < 65.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.08);
        Inspiration_Flow_PID_Controller_Set_Ki(0.5);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);
    }else if(inspiration_flow_setpoint_slpm < 70.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.08);
        Inspiration_Flow_PID_Controller_Set_Ki(0.55);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);
    }else if(inspiration_flow_setpoint_slpm < 75.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.05);
        Inspiration_Flow_PID_Controller_Set_Ki(0.55);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);                        
    }else if(inspiration_flow_setpoint_slpm < 80.0){
        Inspiration_Flow_PID_Controller_Set_Kp(0.045);
        Inspiration_Flow_PID_Controller_Set_Ki(0.5);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0);                        
    }else{
        Inspiration_Flow_PID_Controller_Set_Kp(0.045);
        Inspiration_Flow_PID_Controller_Set_Ki(0.5);
        Inspiration_Flow_PID_Controller_Set_Kd(0.0); 
    }
}


void PIP_PID_Tune(void){
    if(resp_frequency_setpoint_rpm < 20.0){
        if(pip_setpoint_cm_h2o < 5.0){
         PIP_PID_Controller_Set_Kp(0.0);
         PIP_PID_Controller_Set_Ki(0.0);
         PIP_PID_Controller_Set_Kd(0.0); 
        }else if(pip_setpoint_cm_h2o < 10.0){
         PIP_PID_Controller_Set_Kp(0.0);
         PIP_PID_Controller_Set_Ki(0.0);
         PIP_PID_Controller_Set_Kd(0.0);        
        }else if(pip_setpoint_cm_h2o < 15.0){
         PIP_PID_Controller_Set_Kp(0.0);
         PIP_PID_Controller_Set_Ki(0.0);
         PIP_PID_Controller_Set_Kd(0.0);        
        }else if(pip_setpoint_cm_h2o < 20.0){
         PIP_PID_Controller_Set_Kp(0.5);//4.5
         PIP_PID_Controller_Set_Ki(0.5);
         PIP_PID_Controller_Set_Kd(0.0);        
        }else if(pip_setpoint_cm_h2o < 25.0){
         PIP_PID_Controller_Set_Kp(4.5);
         PIP_PID_Controller_Set_Ki(1.5);
         PIP_PID_Controller_Set_Kd(0.0);        
        }else if(pip_setpoint_cm_h2o < 30.0){
         PIP_PID_Controller_Set_Kp(6.0);
         PIP_PID_Controller_Set_Ki(2.0);
         PIP_PID_Controller_Set_Kd(0.0);        
        }else{
         PIP_PID_Controller_Set_Kp(2.8);
         PIP_PID_Controller_Set_Ki(2.8);
         PIP_PID_Controller_Set_Kd(0.0);    
        } 

    }else if(resp_frequency_setpoint_rpm < 30.0){
         if(pip_setpoint_cm_h2o < 5.0){
         PIP_PID_Controller_Set_Kp(0.0);
         PIP_PID_Controller_Set_Ki(0.0);
         PIP_PID_Controller_Set_Kd(0.0); 
        }else if(pip_setpoint_cm_h2o < 10.0){
         PIP_PID_Controller_Set_Kp(0.0);
         PIP_PID_Controller_Set_Ki(0.0);
         PIP_PID_Controller_Set_Kd(0.0);        
        }else if(pip_setpoint_cm_h2o < 15.0){
         PIP_PID_Controller_Set_Kp(0.0);
         PIP_PID_Controller_Set_Ki(0.0);
         PIP_PID_Controller_Set_Kd(0.0);        
        }else if(pip_setpoint_cm_h2o < 20.0){
         PIP_PID_Controller_Set_Kp(0.0);
         PIP_PID_Controller_Set_Ki(0.0);//3.0
         PIP_PID_Controller_Set_Kd(0.0);        
        }else if(pip_setpoint_cm_h2o < 25.0){
         PIP_PID_Controller_Set_Kp(5.2);
         PIP_PID_Controller_Set_Ki(3.8);
         PIP_PID_Controller_Set_Kd(0.0);        
        }else if(pip_setpoint_cm_h2o < 30.0){
         PIP_PID_Controller_Set_Kp(3.2);
         PIP_PID_Controller_Set_Ki(3.1);
         PIP_PID_Controller_Set_Kd(0.0);        
        }else{
         PIP_PID_Controller_Set_Kp(2.8);
         PIP_PID_Controller_Set_Ki(2.8);
         PIP_PID_Controller_Set_Kd(0.0);    
        } 
    }
    else{
         if(pip_setpoint_cm_h2o < 5.0){
         PIP_PID_Controller_Set_Kp(0.0);
         PIP_PID_Controller_Set_Ki(0.0);
         PIP_PID_Controller_Set_Kd(0.0); 
        }else if(pip_setpoint_cm_h2o < 10.0){
         PIP_PID_Controller_Set_Kp(0.0);
         PIP_PID_Controller_Set_Ki(0.0);
         PIP_PID_Controller_Set_Kd(0.0);        
        }else if(pip_setpoint_cm_h2o < 15.0){
         PIP_PID_Controller_Set_Kp(0.0);
         PIP_PID_Controller_Set_Ki(0.0);
         PIP_PID_Controller_Set_Kd(0.0);        
        }else if(pip_setpoint_cm_h2o < 20.0){
         PIP_PID_Controller_Set_Kp(0.0);
         PIP_PID_Controller_Set_Ki(0.0);//7.0
         PIP_PID_Controller_Set_Kd(0.0);        
        }else if(pip_setpoint_cm_h2o < 25.0){
         PIP_PID_Controller_Set_Kp(5.0);
         PIP_PID_Controller_Set_Ki(18.0);
         PIP_PID_Controller_Set_Kd(0.0);        
        }else if(pip_setpoint_cm_h2o < 30.0){
         PIP_PID_Controller_Set_Kp(3.0);
         PIP_PID_Controller_Set_Ki(3.5);
         PIP_PID_Controller_Set_Kd(0.0);        
        }else{
         PIP_PID_Controller_Set_Kp(2.8);
         PIP_PID_Controller_Set_Ki(2.8);
         PIP_PID_Controller_Set_Kd(0.0);    
        } 
        
    }
    
}

void Flow_Setpoint_Compensate(void){

    float volume_error;

    volume_peak_counter++;
    volume_peak_average += volume_peak_value;

    if(volume_peak_counter >= 2){
        volume_peak_average /= 2;
        volume_error = (tidal_volume_setpoint_ml - volume_peak_average);
        valor_volumen_promedio = volume_peak_average;
        valor_error_volumen = volume_error;

        if(volume_peak_average >= VOLUME_MIN_ACCUMULATED_VALUE_ML){

            if(volume_error > VOLUME_SETPOINT_TOLERANCE_ML){
                inspiration_flow_setpoint_slpm++;
                if(inspiration_flow_setpoint_slpm > 80.0){
                    inspiration_flow_setpoint_slpm = 80.0;
                }
            }else if(volume_error < -VOLUME_SETPOINT_TOLERANCE_ML){
                inspiration_flow_setpoint_slpm--;
                if(inspiration_flow_setpoint_slpm < 1.0){
                    inspiration_flow_setpoint_slpm = 1.0;
                }
            }else{
                //Does nothing
            }
            
        }

        Flow_PID_Tune();
        volume_peak_counter = 0; 
        volume_peak_average = 0;
    }
   
}



void Flow_Setpoint_Compensate_2(void){

 float volume_error;


    volume_peak_counter++;
    volume_peak_average += volume_peak_value;

    if(volume_peak_counter >= 3){
        volume_peak_average /= 3;
        volume_error = (tidal_volume_setpoint_ml - volume_peak_average);
        valor_volumen_promedio = volume_peak_average;
        valor_error_volumen = volume_error;

        if(volume_peak_average >= VOLUME_MIN_ACCUMULATED_VALUE_ML){

            if(volume_error > VOLUME_SETPOINT_TOLERANCE_ML){
                
                //inspiration_flow_setpoint_slpm += volume_error * 0.0025;
                inspiration_flow_setpoint_slpm += volume_error * 0.5;
                if(inspiration_flow_setpoint_slpm > 90.0){
                    inspiration_flow_setpoint_slpm = 90.0;
                }
            }else if(volume_error < -VOLUME_SETPOINT_TOLERANCE_ML){
                //inspiration_flow_setpoint_slpm += volume_error * 0.0025;
                inspiration_flow_setpoint_slpm += volume_error * 0.5;
                if(inspiration_flow_setpoint_slpm < 1.0){
                    inspiration_flow_setpoint_slpm = 1.0;
                }
            }else{
                //Does nothing
            }
            
        }

        Flow_PID_Tune();
        volume_peak_counter = 0; 
        volume_peak_average = 0;
    }
   
}




uint16_t Exp_Ball_Valve_Flow_Percent_To_Steps(float percent){

    float steps = 0;
    float aux;

    if(percent < 0.0){
        percent = 0.0;
    }else if(percent > 100){
        percent = 100;
    }else if(percent < 1.4){
        steps = percent * 14.2857;
    }else if(percent < 38.8){
        aux = 1 - ((percent- 1.4) / 40.2916);
        steps = 20 - 15*log(aux);
    }else{
        aux = 1 - ((percent- 38.8) / 62.3561);
        steps = 60 - 19*log(aux);
    }

    if(steps < 0){
        steps = 0.0;
    }else if(steps > EXPIRATION_BALL_VALVE_OPEN_STEPS_LIMIT){
        steps = EXPIRATION_BALL_VALVE_OPEN_STEPS_LIMIT;
    }else{
        //Does nothing
    }

    return (uint16_t)(steps * 2.5);                                         

}





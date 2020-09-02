/* File inclusion */
#include "mbed.h"
#include "stdint.h"
#include "project_defines.h"
#include "global.h"
#include "electric_valves.h"
#include "stepper_position_driver.h"

/* Object definition */

DigitalOut   solenoid_valve_en(INSPIRATION_ON_OFF_VALVE_EN_PIN);  

// -----------------------------------------------------------------------------

/* Variable definition */

/* Angle setpoint in steps */
static volatile int16_t insp_needle_valve_setpoint_in_steps = 0;
static volatile int16_t exp_ball_valve_setpoint_in_steps = 0;


/* Function definition*/

void Electric_Valves_Initialize(void){
    Inspiration_On_Off_Valve_Initialize();  
    Inspiration_Needle_Valve_Initialize();
    Expiration_Ball_Valve_Initialize();
    thread_sleep_for(1000);
}

// --------------------------------------------------------------------------------

void Inspiration_On_Off_Valve_Initialize(void){
    Inspiration_On_Off_Valve_Open();
}


void Inspiration_On_Off_Valve_Open(void){
    //__disable_irq();
    solenoid_valve_en = 1;
    //__enable_irq();
}


void Inspiration_On_Off_Valve_Close(void){
    //__disable_irq();
    solenoid_valve_en = 0;
    //__enable_irq();
}

// --------------------------------------------------------------------------------

void Inspiration_Needle_Valve_Initialize(void){
    Motor1_Initialize();
    Motor1_SetAsOrigen();  
}


void Inspiration_Needle_Valve_Enable(void){
    Motor1_Write_Command(DRIVER_CONTROL_ON);
}


void Inspiration_Needle_Valve_Disable(void){
    Motor1_Write_Command(DRIVER_CONTROL_OFF);
}


void Inspiration_Needle_Valve_Write_Step_Setpoint(uint16_t setpoint){

    if(setpoint > INSPIRATION_NEEDLE_VALVE_OPEN_STEPS_LIMIT){
        setpoint = INSPIRATION_NEEDLE_VALVE_OPEN_STEPS_LIMIT;
    }

    //__disable_irq();
    insp_needle_valve_setpoint_in_steps = setpoint;
    //__enable_irq();  
}


void Inspiration_Needle_Valve_Go_To_Setpoint(void){
    Motor1_Write_Setpoint(insp_needle_valve_setpoint_in_steps);
}



// --------------------------------------------------------------------------------


void Expiration_Ball_Valve_Write_Step_Setpoint(uint16_t setpoint){

    if(setpoint > EXPIRATION_BALL_VALVE_OPEN_STEPS_LIMIT){
        setpoint = EXPIRATION_BALL_VALVE_OPEN_STEPS_LIMIT;
    }

    //__disable_irq();
    exp_ball_valve_setpoint_in_steps = setpoint;
    //__enable_irq();  
}


void Expiration_Ball_Valve_Initialize(void){
    Motor2_Initialize();
    Motor2_SetAsOrigen();
}


void Expiration_Ball_Valve_Go_To_Setpoint(void){
    Motor2_Write_Setpoint(exp_ball_valve_setpoint_in_steps);
}


void Expiration_Ball_Valve_Enable(void){
    Motor2_Write_Command(DRIVER_CONTROL_ON);
}


void Expiration_Ball_Valve_Disable(void){
    Motor2_Write_Command(DRIVER_CONTROL_OFF);
}



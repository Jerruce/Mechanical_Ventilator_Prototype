
/* File inclusion */
#include "mbed.h"
#include "project_defines.h"
#include "global.h"

/* Object definition*/
DigitalOut  alarm_buzzer(BUZZER_PIN);

/* Global variables  */
volatile uint32_t alarm_time_10ms = 0;

/* Function definition */
void Alarm_Buzzer_Update(void){

    if(system_flags & (1 << ALARM_ENABLED_FLAG)){
        if(system_flags & ALARM_FLAGS_MASK){
            // Check for the pressure sensor 1 alarm
            if(system_flags & (1 << PROXIMAL_FLOW_SENSOR_ERROR_FLAG)){
                alarm_time_10ms = PROX_FLOW_ALARM_TOGGLE_PERIOD_10MS;
            }else if(system_flags & (1 << VC_VOLUME_ERROR_FLAG)){
                alarm_time_10ms =  VC_VOLUME_ALARM_TOGGLE_PERIOD_10MS;
            }else if(system_flags & (1 << PC_PIP_ERROR_FLAG)){
                alarm_time_10ms =  PC_PIP_ALARM_TOGGLE_PERIOD_10MS;
            }else{
                // Does nothing
            }

            // Update buzzer output
            if(system_flags & (1 << BUZZER_TOGGLE_FLAG)){
            // __disable_irq();
                system_flags &= ~(1 << BUZZER_TOGGLE_FLAG);
                //__enable_irq();
                ALARM_BUZZER_TOGGLE();
            }
        }else{
            ALARM_BUZZER_OFF();
        }
    }else{
        ALARM_BUZZER_OFF();
    }



}



void Alarm_Time_Count(void){

    static uint32_t alarm_conta_10ms;

    alarm_conta_10ms++;

    // Determine the buzzer toggle time
    if(alarm_conta_10ms >= alarm_time_10ms){
        alarm_conta_10ms = 0;
        system_flags |= (1 << BUZZER_TOGGLE_FLAG);
    }
}
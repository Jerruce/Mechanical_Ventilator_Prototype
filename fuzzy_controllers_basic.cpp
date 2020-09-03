
/* File inclusion */
#include "mbed.h"
#include "project_defines.h"
#include "global.h"
#include "peep_fuzzy_controller.h"
#include "fuzzy_controllers_basic.h"


/* Variabledefinition */
static double flow_inc_control_ge = 0.0 , flow_inc_control_gce = 0.0 ,  flow_inc_control_gcu = 0.0;
static double pip_inc_control_ge = 0.0 , pip_inc_control_gce = 0.0 ,  pip_inc_control_gcu = 0.0;

/* Function definition */


void Flow_Fuzzy_Linear_PD_Controller(double error, double error_change, double *controller_output){

    /* Set boundaries for error input */
    if(error < -1.0){
        error = -1.0;
    }else if(error > 1.0){
        error = 1.0;
    }else{
        /* Does nothing */
    }

    /* Set boundaries for error change input */
    if(error_change < -1.0){
        error_change = -1.0;
    }else if(error_change > 1.0){
        error_change = 1.0;
    }else{
        /* Does nothing */
    }

    *controller_output = (error + error_change) / 2.0;

}


float Flow_Fuzzy_Incremental_Controller(float control_setpoint, float control_feedback, uint8_t control_reset){

    static double error[2];
    static double fuzzy_error;
    double error_change, fuzzy_error_change;
    static double fuzzy_output[2] = {0.0, 0.0};
    static double controller_output = 0.0;
    double delta_area;

    if(control_reset){
       error[0] = 0.0;
       error[1] = 0.0; 
       fuzzy_output[0] = 0.0;
       fuzzy_output[1] = 0.0;
       controller_output = 0.0;   
    }else{
        error[1] = error[0];
        error[0] = control_setpoint - control_feedback;

        error_change = (error[0] - error[1]) / FUZZY_CONTROLLER_SAMPLE_PERIOD_S;

        fuzzy_error = flow_inc_control_ge * error[0];
        fuzzy_error_change = flow_inc_control_gce * error_change;

        fuzzy_output[1] = fuzzy_output[0];
        Flow_Fuzzy_Linear_PD_Controller(fuzzy_error, fuzzy_error_change, &fuzzy_output[0]);
        fuzzy_output[0] *= flow_inc_control_gcu;

        delta_area = FUZZY_CONTROLLER_SAMPLE_PERIOD_S * (fuzzy_output[0] + fuzzy_output[1]) / 2.0;
        controller_output += delta_area;

        if(controller_output < 0.0){
            controller_output = 0.0;
        }else if(controller_output > 100.0){
            controller_output = 100.0;
        }else{
            /* Does nothing */
        }

    }   

    return (float)controller_output;

}



void PIP_Fuzzy_Linear_PD_Controller(double error, double error_change, double *controller_output){

    /* Set boundaries for error input */
    if(error < -1.0){
        error = -1.0;
    }else if(error > 1.0){
        error = 1.0;
    }else{
        /* Does nothing */
    }

    /* Set boundaries for error change input */
    if(error_change < -1.0){
        error_change = -1.0;
    }else if(error_change > 1.0){
        error_change = 1.0;
    }else{
        /* Does nothing */
    }

    *controller_output = (error + error_change) / 2.0;

}



float PIP_Fuzzy_Incremental_Controller(float control_setpoint, float control_feedback, uint8_t control_reset){

    static double error[2];
    static double fuzzy_error;
    double error_change, fuzzy_error_change;
    static double fuzzy_output[2] = {0.0, 0.0};
    static double controller_output = 0.0;
    double delta_area;

    if(control_reset){
       error[0] = 0.0;
       error[1] = 0.0; 
       fuzzy_output[0] = 0.0;
       fuzzy_output[1] = 0.0;
       controller_output = 0.0;   
    }else{
        error[1] = error[0];
        error[0] = control_setpoint - control_feedback;

        error_change = (error[0] - error[1]) / FUZZY_CONTROLLER_SAMPLE_PERIOD_S;

        fuzzy_error = flow_inc_control_ge * error[0];
        fuzzy_error_change = flow_inc_control_gce * error_change;

        fuzzy_output[1] = fuzzy_output[0];
        Flow_Fuzzy_Linear_PD_Controller(fuzzy_error, fuzzy_error_change, &fuzzy_output[0]);
        fuzzy_output[0] *= flow_inc_control_gcu;

        delta_area = FUZZY_CONTROLLER_SAMPLE_PERIOD_S * (fuzzy_output[0] + fuzzy_output[1]) / 2.0;
        controller_output += delta_area;

        if(controller_output < 0.0){
            controller_output = 0.0;
        }else if(controller_output > 100.0){
            controller_output = 100.0;
        }else{
            /* Does nothing */
        }
    }   

     return controller_output;
}



float PEEP_Fuzzy_Controller(float control_setpoint, float control_feedback, uint8_t control_reset){

    static double fuzzy_error;
    static double fuzzy_output = 0.0;

    if(control_reset){
       fuzzy_error = 0.0;
       control_setpoint = 0.0;
       fuzzy_output = 0.0;    
    }else{
        fuzzy_error = control_setpoint - control_feedback;
        peep_fuzzy_controllerInferenceEngine(fuzzy_error, &fuzzy_output);
        
        if(fuzzy_output > 100){
            fuzzy_output = 100;
        }else if (fuzzy_output < 0){
            fuzzy_output = 0;
        }else{
            // fuzzy_output no sufre cambios
        }
    
    }

    return fuzzy_output;

}



void Flow_Fuzzy_Inc_Controller_Set_GE(float ge_value){
    flow_inc_control_ge = ge_value;
}


void Flow_Fuzzy_Inc_Controller_Set_GCE(float gce_value){
    flow_inc_control_gce = gce_value;
}


void Flow_Fuzzy_Inc_Controller_Set_GCU(float gcu_value){
    flow_inc_control_gcu = gcu_value;
}


void PIP_Fuzzy_Inc_Controller_Set_GE(float ge_value){
    pip_inc_control_ge = ge_value;
}


void PIP_Fuzzy_Inc_Controller_Set_GCE(float gce_value){
    pip_inc_control_gce = gce_value;
}


void PIP_Fuzzy_Inc_Controller_Set_GCU(float gcu_value){
    pip_inc_control_gcu = gcu_value;
}
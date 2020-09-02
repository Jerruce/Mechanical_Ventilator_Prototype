
/* File inclusion */
#include "mbed.h"
#include "project_defines.h"
#include "global.h"
#include "peep_fuzzy_controller.h"
#include "flow_fuzzy_pd_controller.h"
#include "flow_fuzzy_inc_controller.h"
#include "fuzzy_controllers_basic.h"


/* Variabledefinition */
static double flow_inc_control_ge = 0.0 , flow_inc_control_gce = 0.0 ,  flow_inc_control_gcu = 0.0;

/* Function definition */

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


float Flow_Fuzzy_PD_Controller(float control_setpoint, float control_feedback, uint8_t control_reset){

    static double error[2];
    static double fuzzy_error;
    double delta_error, fuzzy_delta_error;
    static double fuzzy_output = 0.0;
    static double controller_output = 0.0;

    if(control_reset){
       error[0] = 0.0;
       error[1] = 0.0; 
       fuzzy_error = 0.0;
       control_setpoint = 0.0;
       fuzzy_output = 0.0; 
       controller_output = 0.0;   
    }else{
        error[1] = error[0];
        error[0] = control_setpoint - control_feedback;

        delta_error = error[0] - error[1];

        fuzzy_error = error[0];
        if(fuzzy_error < -FUZZY_PD_FLOW_MAX_ERROR){
            fuzzy_error = -FUZZY_PD_FLOW_MAX_ERROR;
        }else if(fuzzy_error > FUZZY_PD_FLOW_MAX_ERROR){
            fuzzy_error = FUZZY_PD_FLOW_MAX_ERROR;
        }else{
            // Does nothing
        }
        fuzzy_error /= FUZZY_PD_FLOW_MAX_ERROR;

        fuzzy_delta_error = delta_error;
        if(fuzzy_delta_error < -FUZZY_PD_FLOW_MAX_DELTA_ERROR){
            fuzzy_delta_error = -FUZZY_PD_FLOW_MAX_DELTA_ERROR;
        }else if(fuzzy_delta_error > FUZZY_PD_FLOW_MAX_DELTA_ERROR){
            fuzzy_delta_error = FUZZY_PD_FLOW_MAX_DELTA_ERROR;
        }else{
            // Does nothing
        }
        fuzzy_delta_error /= FUZZY_PD_FLOW_MAX_DELTA_ERROR;


        flow_fuzzy_pd_controllerInferenceEngine(fuzzy_delta_error, fuzzy_error, &fuzzy_output);

        controller_output = (fuzzy_output + 1.0) * 50.0;

        if(controller_output > 100){
            controller_output = 100;
        }else if (controller_output < 0){
            controller_output = 0;
        }else{
            // fuzzy_output no sufre cambios
        }
    
    }

    return controller_output;


}   


float Flow_Fuzzy_Linear_PD_Controller(float control_setpoint, float control_feedback, uint8_t control_reset){

    static double error[2];
    static double fuzzy_error;
    
    return 0;

}



float Flow_Fuzzy_Inc_Controller(float control_setpoint, float control_feedback, uint8_t control_reset){

    static double error[2];
    static double fuzzy_error;
    double delta_error, fuzzy_delta_error;
    static double fuzzy_output = 0.0;
    static double controller_output = 0.0;
    static double fuzzy_output_gcu[2];
    static double fuzzy_output_u = 0;
    double fuzzy_output_cu;
    double delta_area = 0.0;

    if(control_reset){
       error[1] = 0.0;
       error[0] = 0.0;
       control_setpoint = 0.0;
       fuzzy_output_cu = 0.0;
       fuzzy_output_u = 0.0;
       fuzzy_output_gcu[1] = 0.0; 
       fuzzy_output_gcu[0] = 0.0; 
    }else{

        /* Read the error and change in error*/
        error[1] = error[0];
        error[0] = control_setpoint - control_feedback;

        delta_error = error[0] - error[1];

        /* Apply the gain and constrain in error input */
        fuzzy_error = flow_inc_control_ge * error[0];

        if(fuzzy_error < -FUZZY_PD_FLOW_MAX_ERROR){
            fuzzy_error = -FUZZY_PD_FLOW_MAX_ERROR;
        }else if(fuzzy_error > FUZZY_PD_FLOW_MAX_ERROR){
            fuzzy_error = FUZZY_PD_FLOW_MAX_ERROR;
        }else{
            // Does nothing
        }
        fuzzy_error /= FUZZY_PD_FLOW_MAX_ERROR;

        
        /* Apply the gain and constrain in change error input */
        fuzzy_delta_error = (flow_inc_control_gce * delta_error);
        
        if(fuzzy_delta_error < -FUZZY_PD_FLOW_MAX_DELTA_ERROR){
            fuzzy_delta_error = -FUZZY_PD_FLOW_MAX_DELTA_ERROR;
        }else if(fuzzy_delta_error > FUZZY_PD_FLOW_MAX_DELTA_ERROR){
            fuzzy_delta_error = FUZZY_PD_FLOW_MAX_DELTA_ERROR;
        }else{
            // Does nothing
        }
        fuzzy_delta_error /= FUZZY_PD_FLOW_MAX_DELTA_ERROR;

        flow_fuzzy_inc_controllerInferenceEngine(fuzzy_delta_error, fuzzy_error, &fuzzy_output_cu);
        
        fuzzy_output_gcu[1] = fuzzy_output_gcu[0];
        fuzzy_output_gcu[0] = flow_inc_control_gcu * fuzzy_output_cu;

        /* Integrate and constrain the output */
        delta_area = (fuzzy_output_gcu[0] + fuzzy_output_gcu[1]) / 2.0;
        fuzzy_output_u += delta_area;

        if(fuzzy_output_u < 0){
            fuzzy_output_u = 0;
        }else if (fuzzy_output_u > 100){
            fuzzy_output_u = 100;
        }else{
            // fuzzy_output no sufre cambios
        }

        controller_output = fuzzy_output_u;

    }

    return (float)controller_output;
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
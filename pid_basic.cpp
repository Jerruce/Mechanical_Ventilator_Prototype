#include "stdint.h"
#include "project_defines.h"  
#include "global.h" 
#include "pid_basic.h"


void PID_Controllers_Initialize(void){
    
    Inspiration_Flow_PID_Controller(0.0, 0.0, 1);    
    PIP_PID_Controller(0.0, 0.0, 1);
    PEEP_PID_Controller(0.0, 0.0, 1);
}


/* Sintonizar el PID variando estos valores */
//static double flow_kp = 1.6 , flow_ki = 0.85 ,  flow_kd = 0.0 ;//ti=34.0732.0, kp=3.3382, kd=0.0081
static double flow_kp = 0.15 , flow_ki = 0.6 ,  flow_kd = 0.0 ;//ti=34.0732.0, kp=3.3382, kd=0.0081
static double pip_kp = 0.0 , pip_ki = 0.0 ,  pip_kd = 0.0 ;//ti=34.0732.0, kp=3.3382, kd=0.0081
static double peep_kp = 0.0 , peep_ki = 0.0 ,  peep_kd = 0.0 ;//ti=34.0732.0, kp=3.3382, kd=0.0081



double Inspiration_Flow_PID_Controller(double pid_setpoint, double pid_feedback, uint8_t pid_reset){
    
    static double pid_error[3];
    static double pid_output = 0.0;
    double delta_area = 0.0;
    double p_comp, d_comp;
    static double i_comp = 0;

    if(pid_reset){
       
       pid_error[2] = 0.0;
       pid_error[1] = 0.0;
       pid_error[0] = 0.0; 
       pid_setpoint = 0.0;
       pid_output = 0.0;
       i_comp = 0.0;    
        
    }else{
  
        pid_error[2] = pid_error[1];
        pid_error[1] = pid_error[0];
        pid_error[0] = pid_setpoint - pid_feedback;
        delta_area = PID_SAMPLE_PERIOD_S * ((pid_error[0] + pid_error[1])) / 2.0;

        p_comp = pid_error[0];
        i_comp = i_comp + delta_area;
        d_comp = (pid_error[0] - pid_error[1]) / PID_SAMPLE_PERIOD_S;
        
        pid_output = (flow_kp * p_comp) + (flow_ki * i_comp) + (flow_kd * d_comp);
        

        if(pid_output > FLOW_PID_MAX_OUTPUT){
            pid_output = FLOW_PID_MAX_OUTPUT;
        }else if (pid_output < FLOW_PID_MIN_OUTPUT){
            pid_output = FLOW_PID_MIN_OUTPUT;
        }else{
            // aux no sufre cambios
        }
    
    }

    return pid_output;

}



double PIP_PID_Controller(double pid_setpoint, double pid_feedback, uint8_t pid_reset){
    
    static double pid_error[3];
    static double pid_output = 0.0;
    double delta_area = 0.0;
    double p_comp, d_comp;
    static double i_comp = 0;

    if(pid_reset){
       
       pid_error[2] = 0.0;
       pid_error[1] = 0.0;
       pid_error[0] = 0.0; 
       pid_setpoint = 0.0;
       pid_output = 0.0;
       i_comp = 0.0;     
        
    }else{
  
        pid_error[2] = pid_error[1];
        pid_error[1] = pid_error[0];
        pid_error[0] = pid_setpoint - pid_feedback;
        delta_area = PID_SAMPLE_PERIOD_S * ((pid_error[0] + pid_error[1])) / 2.0;

        p_comp = pid_error[0];
        i_comp = i_comp + delta_area;
        d_comp = (pid_error[0] - pid_error[1]) / PID_SAMPLE_PERIOD_S;
        
        pid_output = (pip_kp * p_comp) + (pip_ki * i_comp) + (pip_kd * d_comp);
        

        if(pid_output > PIP_PID_MAX_OUTPUT){
            pid_output = PIP_PID_MAX_OUTPUT;
        }else if (pid_output < PIP_PID_MIN_OUTPUT){
            pid_output = PIP_PID_MIN_OUTPUT;
        }else{
            // aux no sufre cambios
        }
    
    }

    return pid_output;
}


double PEEP_PID_Controller(double pid_setpoint, double pid_feedback, uint8_t pid_reset){
    
    static double pid_error[3];
    static double pid_output = 0.0;
    double delta_area = 0.0;
    double p_comp, d_comp;
    static double i_comp = 0;

    if(pid_reset){
       
       pid_error[2] = 0.0;
       pid_error[1] = 0.0;
       pid_error[0] = 0.0; 
       pid_setpoint = 0.0;
       pid_output = 0.0;
       i_comp = 0.0;     
        
    }else{
  
        pid_error[2] = pid_error[1];
        pid_error[1] = pid_error[0];
        pid_error[0] = pid_setpoint - pid_feedback;
        //pid_error[0] =  pid_feedback - pid_setpoint;
        delta_area = PID_SAMPLE_PERIOD_S * ((pid_error[0] + pid_error[1])) / 2.0;

        p_comp = pid_error[0];
        i_comp = i_comp + delta_area;
        d_comp = (pid_error[0] - pid_error[1]) / PID_SAMPLE_PERIOD_S;
        
        pid_output = (peep_kp * p_comp) + (peep_ki * i_comp) + (peep_kd * d_comp);

    
        if(pid_output > PEEP_PID_MAX_OUTPUT){
            pid_output = PEEP_PID_MAX_OUTPUT;
        }else if (pid_output < PEEP_PID_MIN_OUTPUT){
            pid_output = PEEP_PID_MIN_OUTPUT;
        }else{
            // aux no sufre cambios
        }
    
    }

    return pid_output;
}



void Inspiration_Flow_PID_Controller_Set_Kp(double kp_value){
    flow_kp = kp_value;
}

void Inspiration_Flow_PID_Controller_Set_Ki(double ki_value){
    flow_ki = ki_value;
}

void Inspiration_Flow_PID_Controller_Set_Kd(double kd_value){
    flow_kd = kd_value;
}

void PIP_PID_Controller_Set_Kp(double kp_value){
    pip_kp = kp_value;
}

void PIP_PID_Controller_Set_Ki(double ki_value){
    pip_ki = ki_value;
}

void PIP_PID_Controller_Set_Kd(double kd_value){
    pip_kd = kd_value;
}

void PEEP_PID_Controller_Set_Kp(double kp_value){
    peep_kp = kp_value;
}

void PEEP_PID_Controller_Set_Ki(double ki_value){
    peep_ki = ki_value;
}

void PEEP_PID_Controller_Set_Kd(double kd_value){
    peep_kd = kd_value;
}

/* File inclusion */
#include "stdint.h"
#include "project_defines.h"
#include "filters.h"

/* Global variables */

static float flow_lpf_output = 0;
static float pressure_lpf_output = 0;
static float water_column_lpf_output = 0;
static float flow_pid_lpf_output = 0;

/* Function definition */

void Filters_Initialize(void){

    Proximal_Pressure_Cheby2_LPF(0.0, 1);
    Proximal_Flow_Cheby2_LPF(0.0, 1);
    Flow_PID_FIR_LPF(0.0, 1);
}


float Proximal_Pressure_Cheby2_LPF(float raw_value, uint8_t restart){
    
   const float b[PRESSURE_01_LPF_N] = {0.20077,   0.47142,   0.47142,   0.20077};                        
   const float a[PRESSURE_01_LPF_N] = {1.0000000,  -0.0398709,   0.3747845,   0.0094542};
   
   static float x[PRESSURE_01_LPF_N];
   static float y[PRESSURE_01_LPF_N];
   
   int16_t i;  

    if(restart){
        for(i = 0; i < PRESSURE_01_LPF_N; i++){
           x[i] = 0.0; 
           y[i] = 0.0; 
        }
    }else{
        
        /* Update recorded input and output values */
        for(i = PRESSURE_01_LPF_N - 1; i > 0; i--){
           x[i] = x[i - 1]; 
           y[i] = y[i - 1]; 
        }
   
        /* Update current input and output values */
        x[0] = raw_value;
        y[0] = b[0] * x[0];
        for(i = 1; i < PRESSURE_01_LPF_N; i++){
           y[0] += (b[i] * x[i]) - (a[i] * y[i]); 
        } 
        
    }

    return y[0];
}



float Proximal_Flow_Cheby2_LPF(float raw_value, uint8_t restart){
    
   const float b[FLOW_LPF_N] = {0.0585,   0.0382,   0.0382,   0.0585};                        
   const float a[FLOW_LPF_N] = {1.0000000,  -1.6511,   1.0893,   -0.2447};
   
   static float x[FLOW_LPF_N];
   static float y[FLOW_LPF_N];
   
   int16_t i;  

    if(restart){
        for(i = 0; i < FLOW_LPF_N; i++){
           x[i] = 0.0; 
           y[i] = 0.0; 
        }
    }else{
        
        /* Update recorded input and output values */
        for(i = FLOW_LPF_N - 1; i > 0; i--){
           x[i] = x[i - 1]; 
           y[i] = y[i - 1]; 
        }
   
        /* Update current input and output values */
        x[0] = raw_value;
        y[0] = b[0] * x[0];
        for(i = 1; i < FLOW_LPF_N; i++){
           y[0] += (b[i] * x[i]) - (a[i] * y[i]); 
        } 
        
    }

    return y[0];
}






float Flow_PID_FIR_LPF(float raw_value, uint8_t restart){
// Filter of 10 HZ
    const float b[FLOW_PID_LPF_N] = {
                                         0.00039283,   0.00095967,   0.00133595,   0.00128764,   0.00053529,
                                        -0.00101810,  -0.00302174,  -0.00458279, -0.00451184,  -0.00190788,
                                        0.00314241,   0.00913196,   0.01332520,   0.01270014,   0.00540223,
                                        -0.00785969,  -0.02316569,  -0.03398547,  -0.03306256,  -0.01505246,
                                        0.02103265,   0.07058419,   0.12401460,   0.16921421, 0.19510924,
                                        0.19510924,   0.16921421,   0.12401460,   0.07058419,   0.02103265,
                                        -0.01505246,  -0.03306256,  -0.03398547,  -0.02316569,  -0.00785969,
                                        0.00540223,   0.01270014,   0.01332520,   0.00913196,   0.00314241,
                                        -0.00190788,  -0.00451184,  -0.00458279,  -0.00302174,  -0.00101810,
                                        0.00053529,   0.00128764,   0.00133595,    0.00095967,   0.00039283
                                        };
 
 
 //Filter of 4HZ
/* 
 const float b[FLOW_PID_LPF_N] = {
                                         -0.00020668,  -0.00049110,  -0.00086076,  -0.00135961,  -0.00200310,
                                        -0.00276490,  -0.00356817,  -0.00428309,  -0.00473174,  -0.00470045,
                                        -0.00395897,  -0.00228458,   0.00051102,   0.00455525,   0.00989154,  
                                        0.01646259,   0.02410233,   0.03253814,   0.04140374,   0.05026218,
                                        0.05863698,   0.06604876,   0.07205393,   0.07628159,   0.07846511,
                                        0.07846511,   0.07628159,   0.07205393,   0.06604876,   0.05863698,
                                        0.05026218,   0.04140374,   0.03253814,   0.02410233,   0.01646259,
                                        0.00989154,   0.00455525,   0.00051102,  -0.00228458,  -0.00395897,
                                        -0.00470045,  -0.00473174,  -0.00428309,  -0.00356817,  -0.00276490, 
                                        -0.00200310,  -0.00135961,  -0.00086076,  -0.00049110,  -0.00020668
                                        };
   */          
    static float x[FLOW_PID_LPF_N];
    float y; 
        
    int16_t i; 
    
     if(restart){
        for(i = 0; i < FLOW_PID_LPF_N; i++){
           x[i] = 0.0; 
        }
    }else{
        
        /* Update recorded input values */
        for(i = FLOW_PID_LPF_N - 1; i > 0; i--){
           x[i] = x[i - 1]; 
        }
   
        /* Update current input and output values */
        x[0] = raw_value;
        y = 0;
        for(i = 0; i < FLOW_PID_LPF_N; i++){
           y += (b[i] * x[i]); 
        } 
        
    }

    return y;     
}





void Apply_Pressure_LPF_Filter(float pressure_signal){
    pressure_lpf_output = Proximal_Pressure_Cheby2_LPF(pressure_signal, 0);
}

void Apply_Flow_LPF_Filter(float flow_signal){
    flow_lpf_output = Proximal_Flow_Cheby2_LPF(flow_signal, 0);
}


float Get_Pressure_LPF_Output(void){
    return pressure_lpf_output;
}


float Get_Flow_LPF_Output(void){
    return  flow_lpf_output;
}


void Apply_Flow_PID_LPF_Filter(float flow_signal){
    flow_pid_lpf_output = Flow_PID_FIR_LPF(flow_signal, 0);
}


float Get_Flow_PID_LPF_Output(void){
    return flow_pid_lpf_output;
}
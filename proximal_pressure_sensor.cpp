
/* File inclusion */
#include "mbed.h"
#include "stdint.h"
#include "project_defines.h"
#include "global.h"
#include "proximal_pressure_sensor.h"


/* Object definition */
I2C proximal_pressure_sensor(PROXIMAL_PRESSURE_SENSOR_SDA_PIN, PROXIMAL_PRESSURE_SENSOR_SCL_PIN );

/* Global variable definition */
static float proximal_pressure_cm_h2o;
static char sensor_read_data_buffer[2];


/* Function definition */

void Proximal_Pressure_Sensor_Initialize(void){
    
    proximal_pressure_sensor.frequency(PROXIMAL_PRESSURE_SENSOR_I2C_CLK_HZ); 
    
}


void Proximal_Pressure_Sensor_Read(void){
    
    int16_t proximal_pressure_bin;
    float proximal_pressure_psi;
    uint8_t i2c_error;
    static uint8_t error_counter = 0;
    
    /* Read proximal pressure binary value */
    sensor_read_data_buffer[0] = 0;
    sensor_read_data_buffer[1] = 0;
    i2c_error = proximal_pressure_sensor.read(PROXIMAL_PRESSURE_SENSOR_SRA, sensor_read_data_buffer, 2);

    proximal_pressure_bin = ((sensor_read_data_buffer[0] << 8) | sensor_read_data_buffer[1]) & 0x3FFF;
    if(proximal_pressure_bin < PROX_PRESS_SENSOR_OUTPUT_MIN){
        proximal_pressure_bin = PROX_PRESS_SENSOR_OUTPUT_MIN;
    } 

    /* Check if the proximal pressure sensor is working */ 
    
    if(i2c_error){

        error_counter++;
        
        if(error_counter >= PROXIMAL_PRESSURE_SENSOR_MAX_ERROR_COUNT){
            error_counter = 0;
            __disable_irq();
            system_flags |= (1 << PROXIMAL_PRESSURE_SENSOR_ERROR_FLAG);
            __enable_irq();
        }
    }else{
        error_counter = 0;
        __disable_irq();
        system_flags &= ~(1 << PROXIMAL_PRESSURE_SENSOR_ERROR_FLAG);
        __enable_irq();

        /* Calculate pressure value in PSI */
        proximal_pressure_psi = (proximal_pressure_bin - PROX_PRESS_SENSOR_OUTPUT_MIN) * (PROX_PRESS_SENSOR_PRESS_MAX - PROX_PRESS_SENSOR_PRESS_MIN);
        proximal_pressure_psi = (proximal_pressure_psi / (PROX_PRESS_SENSOR_OUTPUT_MAX - PROX_PRESS_SENSOR_OUTPUT_MIN)) + PROX_PRESS_SENSOR_PRESS_MIN;
        
        /* Convert from PSI to cm H2O and apply a correction if necessary */
        proximal_pressure_cm_h2o = PSI_TO_CMH2O_CONSTANT * proximal_pressure_psi;
        proximal_pressure_cm_h2o = (PROXIMAL_PRESSURE_SENSOR_SCALE_FACTOR * proximal_pressure_cm_h2o) + PROXIMAL_PRESSURE_SENSOR_OFFSET_CM_H2O; 

        /* Ensure that the pressure is a non-negative value */
        if(proximal_pressure_cm_h2o < 0.0){
            proximal_pressure_cm_h2o = 0.0;
        }

    }
   
} 


float Proximal_Pressure_Sensor_Get_Pressure(void){
    return proximal_pressure_cm_h2o;
}



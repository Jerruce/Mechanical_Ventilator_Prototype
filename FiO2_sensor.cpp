
#include "mbed.h"
#include "project_defines.h"
#include "global.h"
#include "FiO2_sensor.h"

/* Object definition */
AnalogIn fio2_sensor(FIO2_SENSOR_PIN);

/* Variable definition */
static float fio2_percent = 0;


void FiO2_Sensor_Read(void){

    float sensor_volt_out;

    sensor_volt_out = 3.3 * fio2_sensor.read();
    fio2_percent = 100.0 * (sensor_volt_out - FIO2_AMP_MIN_OUT_V) / (FIO2_AMP_MAX_OUT_V - FIO2_AMP_MIN_OUT_V);
    //fio2_percent = sensor_volt_out * 100;
 
    if(fio2_percent > 100.0){
        fio2_percent = 100;
    }else if(fio2_percent < 0.0){
        fio2_percent = 0.0;
    }else{
        // Does nothing 
    }
    

}



float Get_FiO2_Value(void){

    return fio2_percent;

}
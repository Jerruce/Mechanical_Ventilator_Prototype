
#ifndef PROJECT_DEFINES_H_
#define PROJECT_DEFINES_H_

/******************************************************************************/
/*************************** PIN DEFINITION ***********************************/
/******************************************************************************/

// Serial communication with the PC 
#define PC_TX_PIN                                       PD_8
#define PC_RX_PIN                                       PD_9

// Serial communication with the Nextion HMI 
#define NEXTION_TX_PIN                                  PD_5
#define NEXTION_RX_PIN                                  PD_6

// Proximal pressure sensor 
#define PROXIMAL_PRESSURE_SENSOR_SCL_PIN                PB_8
#define PROXIMAL_PRESSURE_SENSOR_SDA_PIN                PB_9

// Proximal flow sensor 
#define PROXIMAL_FLOW_SENSOR_SCL_PIN                    PD_12
#define PROXIMAL_FLOW_SENSOR_SDA_PIN                    PD_13

// Inspiration on/off (solenoid) valve 
#define INSPIRATION_ON_OFF_VALVE_EN_PIN                 PC_9

// Inspiration needle valve
#define INSPIRATION_NEEDLE_VALVE_PULSE_PIN              PF_7
#define INSPIRATION_NEEDLE_VALVE_DIR_PIN                PF_6
#define INSPIRATION_NEEDLE_VALVE_EN_PIN                 PC_12
#define INSPIRATION_NEEDLE_VALVE_BRAKE_PIN              PE_12
#define INSPIRATION_NEEDLE_VALVE_CLOSE_SENSOR_PIN       PD_2


// Expiration ball valve 
#define EXPIRATION_BALL_VALVE_PULSE_PIN                 PF_9
#define EXPIRATION_BALL_VALVE_DIR_PIN                   PG_1
#define EXPIRATION_BALL_VALVE_EN_PIN                    PF_8
#define EXPIRATION_BALL_VALVE_BRAKE_PIN                 PE_10
#define EXPIRATION_BALL_VALVE_CLOSE_SENSOR_PIN          PB_5

// FiO2 sensor 
#define FIO2_SENSOR_PIN                                 PF_3

// Buzzer alarm
#define BUZZER_PIN                                      PG_6


/******************************************************************************/
/*************************  CONSTANTS AND MACROS *******************************/
/******************************************************************************/

// -------------------- Defines for serial communication -----------------------

#define MODSERIAL_DEFAULT_RX_BUFFER_SIZE                1024
#define MODSERIAL_DEFAULT_TX_BUFFER_SIZE                1024 

// ------------------ Defines for proximal pressure sensor ---------------------

#define PROXIMAL_PRESSURE_SENSOR_I2C_CLK_HZ             100000

#define PROXIMAL_PRESSURE_SENSOR_SLAVE_ADDRESS          0x28
#define PROXIMAL_PRESSURE_SENSOR_SWA                    (PROXIMAL_PRESSURE_SENSOR_SLAVE_ADDRESS << 1)
#define PROXIMAL_PRESSURE_SENSOR_SRA                    (PROXIMAL_PRESSURE_SENSOR_SWA  | 1)

#define PROX_PRESS_SENSOR_OUTPUT_MAX                   14745
#define PROX_PRESS_SENSOR_OUTPUT_MIN                   1638
#define PROX_PRESS_SENSOR_PRESS_MAX                    1.0f
#define PROX_PRESS_SENSOR_PRESS_MIN                    0.0f

#define PRESSURE_SENSOR_OUTPUT_MAX                      14745
#define PRESSURE_SENSOR_OUTPUT_MIN                      1638
#define PRESSURE_SENSOR_PRESSURE_MAX                    1.0f
#define PRESSURE_SENSOR_PRESSURE_MIN                    0.0f

#define PROXIMAL_PRESSURE_SENSOR_SCALE_FACTOR           1.0f
#define PROXIMAL_PRESSURE_SENSOR_OFFSET_CM_H2O          0.0f

#define PSI_TO_CMH2O_CONSTANT                           70.306957829636f

#define PROXIMAL_PRESSURE_SENSOR_MAX_ERROR_COUNT        10

// ------------------- Defines for proximal flow sensor ------------------------

#define PROXIMAL_FLOW_SENSOR_I2C_CLK_HZ                 100000

#define PROXIMAL_FLOW_SENSOR_SLAVE_ADDRESS              0x40  
#define PROXIMAL_FLOW_SENSOR_SWA                        (PROXIMAL_FLOW_SENSOR_SLAVE_ADDRESS << 1)
#define PROXIMAL_FLOW_SENSOR_SRA                        (PROXIMAL_FLOW_SENSOR_SWA  | 1) 

#define PROXIMAL_FLOW_SENSOR_OFFSET_SLPM                32768.0
#define PROXIMAL_FLOW_SENSOR_SCALE_FACTOR               120.0f
#define PROXIMAL_FLOW_SENSOR_THRESHOLD_SLPM             0.5f

#define PROXIMAL_FLOW_SENSOR_READ_PERIOD_MS             10

#define PROXIMAL_FLOW_SENSOR_CRC_POLYNOM                0x131
#define PROXIMAL_FLOW_SENSOR_COUNTER_MAX                10

#define SENSIRION_START_FLOW_MEASUREMENT                0x1000
#define SENSIRION_START_TEMP_MEASUREMENT                0x1001
#define SENSIRION_READ_SCALE_FACTOR                     0x30DE
#define SENSIRION_READ_OFFSET                           0x30DF
#define SENSIRION_READ_ARTICLE_NUMBER_P1                0x31E3
#define SENSIRION_READ_ARTICLE_NUMBER_P2                0x31E4 
#define SENSIRION_READ_SERIAL_NUMBER_P1                 0x31AE
#define SENSIRION_READ_SERIAL_NUMBER_P2                 0x31AF     
#define SENSIRION_SOFTWARE_RESET                        0x2000

#define VOLUME_SETPOINT_TOLERANCE_ML                    5
#define VOLUME_MIN_ACCUMULATED_VALUE_ML                 50

// -------------------- Defines for the electric valves ------------------------

#define INSPIRATION_NEEDLE_VALVE_DIR                0 
#define INSPIRATION_NEEDLE_VALVE_EN                 1    
#define INSPIRATION_ON_OFF_VALVE_EN                 2


#define INSPIRATION_NEEDLE_VALVE_PULSE_PERIOD_S        0.0000625
#define INSPIRATION_NEEDLE_VALVE_PULSE_DUTY_CYCLE      0.5

#define EXPIRATION_BALL_VALVE_PULSE_PERIOD_S         0.001
#define EXPIRATION_BALL_VALVE_PULSE_DUTY_CYCLE       0.5
#define EXPIRATION_BALL_VALVE_OPEN_ANGLE_STEPS        350    

#define INSPIRATION_NEEDLE_VALVE_OPEN_STEPS_LIMIT       1410
#define INSPIRATION_NEEDLE_VALVE_CLOSE_STEPS_LIMIT      1410
#define INSPIRATION_NEEDLE_VALVE_CLOSE_GUARD_LIMIT      7

#define EXPIRATION_BALL_VALVE_OPEN_STEPS_LIMIT        180//320//4580//1410,
#define EXPIRATION_BALL_VALVE_CLOSE_STEPS_LIMIT       1200//360
#define EXPIRATION_BALL_VALVE_CLOSE_GUARD_LIMIT       10

#define INSPIRATION_NEEDLE_VALVE_SPEED_SWITCH_THRESHOLD   50  
#define EXPIRATION_BALL_VALVE_SPEED_SWITCH_THRESHOLD      50
  

// ----------------------- Defines for digital filters -------------------------

#define PRESSURE_01_LPF_N                               4
#define PRESSURE_02_LPF_N                               3
#define FLOW_LPF_N                                      25

#define FLOW_PID_LPF_N                                  50

// ----------------------- Defines for PID controllers -------------------------

#define CONTROLLER_SAMPLE_PERIOD_S      0.05

#define PID_SAMPLE_PERIOD_S             0.05f

#define FLOW_PID_MAX_OUTPUT             100.0f
#define FLOW_PID_MIN_OUTPUT             0.0f

#define PIP_PID_MAX_OUTPUT              100.0f
#define PIP_PID_MIN_OUTPUT              0.0f

#define PEEP_PID_MAX_OUTPUT             100.0f
#define PEEP_PID_MIN_OUTPUT             0.0f

#define PIP_PID_EXPIRATION_VALVE_SCALE  1.0f

// ----------------------- Defines for fuzzy controllers -------------------------

#define FUZZY_PD_FLOW_MAX_ERROR             30
#define FUZZY_PD_FLOW_MAX_DELTA_ERROR       30
#define FUZZY_CONTROLLER_SAMPLE_PERIOD_S    0.05


// ---------------------- Defines for the oxygen sensor ------------------------

#define FIO2_SENSOR_MIN_OUT_V   0.000230349//0.009
#define FIO2_SENSOR_MAX_OUT_V   0.05291946
#define FIO2_AMP_GAIN           47.8085
#define FIO2_AMP_MIN_OUT_V      (FIO2_SENSOR_MIN_OUT_V * FIO2_AMP_GAIN)
#define FIO2_AMP_MAX_OUT_V      (FIO2_SENSOR_MAX_OUT_V * FIO2_AMP_GAIN)

// ------------- Defines for the Nextion display user interface ----------------

#define NEXTION_DISPLAY_BAUD_RATE               250000

#define BLACK                                   0x0000
#define BLUE                                    0x001F
#define BROWN                                   0xBC40
#define GREEN                                   0x07E0
#define YELLOW                                  0xFFE0
#define RED                                     0xF800
#define GRAY                                    0x8430
#define WHITE                                   0xFFFF
#define DARK_BLUE                               0x218F

#define NORMAL_COLOR                            WHITE
#define SELECT_COLOR                            BLUE
#define ADJUST_COLOR                            GREEN
#define BACKGROUND_COLOR                        BLACK
#define WAVEFORM_BACKGROUND_COLOR               DARK_BLUE
#define WAVEFORM_GRID_COLOR                     GRAY     

#define VOLUME_WAVEFORM_ID                      25
#define VOLUME_WAVEFORM_X0                      245
#define VOLUME_WAVEFORM_Y0                      45
#define VOLUME_WAVEFORM_WIDTH                   550
#define VOLUME_WAVEFORM_HEIGHT                  140
#define VOLUME_WAVEFORM_OFFSET                  70
#define VOLUME_WAVEFORM_GAIN                    0.07f          

#define FLOW_WAVEFORM_ID                        26
#define FLOW_WAVEFORM_X0                        245
#define FLOW_WAVEFORM_Y0                        188
#define FLOW_WAVEFORM_WIDTH                     550
#define FLOW_WAVEFORM_HEIGHT                    140
#define FLOW_WAVEFORM_OFFSET                    70
#define FLOW_WAVEFORM_GAIN                      0.875f

#define PRESSURE_WAVEFORM_ID                    27
#define PRESSURE_WAVEFORM_X0                    245
#define PRESSURE_WAVEFORM_Y0                    331
#define PRESSURE_WAVEFORM_WIDTH                 550
#define PRESSURE_WAVEFORM_HEIGHT                140
#define PRESSURE_WAVEFORM_OFFSET                35
#define PRESSURE_WAVEFORM_GAIN                  3.5f

// ------------------------ Defines for the alarm buzzer ------------------------

#define ALARM_BUZZER_ON()                               alarm_buzzer = 1
#define ALARM_BUZZER_OFF()                              alarm_buzzer = 0
#define ALARM_BUZZER_TOGGLE()                           alarm_buzzer = !alarm_buzzer  

#define VOLUME_PEAK_ALARM_TOLERANCE_ML                  10                      
#define PIP_ALARM_TOLERANCE_CM_H2O                      5

#define PROXIMAL_FLOW_SENSOR_MAX_ERROR_COUNT            25
#define PROX_FLOW_ALARM_TOGGLE_PERIOD_10MS              25
#define VC_VOLUME_ALARM_TOGGLE_PERIOD_10MS              50
#define PC_PIP_ALARM_TOGGLE_PERIOD_10MS                 50


// ----------------- Min, Max and Defaut values for parameters ------------------

#define RESP_FREQUENCY_MIN_RPM          10
#define RESP_FREQUENCY_MAX_RPM          30
#define RESP_FREQUENCY_BY_DEFAULT_RPM   20

#define TIDAL_VOLUME_MIN_ML             300
#define TIDAL_VOLUME_MAX_ML             600
#define TIDAL_VOLUME_BY_DEFAULT_ML      400

#define INSP_RATIO_MIN                  1.0f
#define INSP_RATIO_MAX                  1.0f
#define INSP_RATIO_BY_DEFAULT           1.0f

#define EXP_RATIO_MIN                   2.0f
#define EXP_RATIO_MAX                   3.0f
#define EXP_RATIO_BY_DEFAULT            2.0f

#define PIP_MIN_CM_H2O                  15
#define PIP_MAX_CM_H2O                  35
#define PIP_BY_DEFAULT_CM_H2O           20

#define PEEP_MAX_CM_H2O                 20
#define PEEP_MIN_CM_H2O                 0
#define PEEP_BY_DEFAULT_CM_H2O          0

#define FLOW_MIN_SLPM                   0
#define FLOW_MAX_SLPM                   120
#define FLOW_BY_DEFAULT_SLPM            20

// ----------------------------- System flags ----------------------------------

#define PROXIMAL_PRESSURE_SENSOR_ERROR_FLAG             0
#define PROXIMAL_FLOW_SENSOR_ERROR_FLAG                 1
#define INSP_NEEDLE_VALVE_SENSOR_ERROR_FLAG             2
#define EXP_ON_OFF_VALVE_SENSOR_ERROR_FLAG              3
#define EXP_NEEDLE_VALVE_SENSOR_ERROR_FLAG              4
#define BREATH_MODE_UPDATE_FLAG                         5
#define PID_PARAMETERS_UPDATE_FLAG                      6
#define PID_ACTION_UPDATE_FLAG                          7
#define INSPIRATION_FINISH_FLAG                         8
#define EXPIRATION_FINISH_FLAG                          9
#define SAMPLE_SENSORS_FLAG                             10
#define PLOT_WAVEFORMS_FLAG                             11
#define COMMAND_RECEIVED_DATA_FLAG                      12
#define PROXIMAL_FLOW_SENSOR_RESTARTING                 13
#define DISPLAY_MEASURES_FLAG                           14
#define PERISTALTIC_WORKING_FLAG                        15
#define PEEP_WATER_COLUMN_CHECK_FLAG                    16
#define PARAMETER_MODIFIED_FLAG                         17
#define PEEP_UPDATE_FLAG                                18
#define PIP_VALVE_OPEN_FLAG                             19
#define ALARM_ENABLED_FLAG                              20
#define BUZZER_TOGGLE_FLAG                              21
#define VC_VOLUME_ERROR_FLAG                            22
#define PC_PIP_ERROR_FLAG                               23
#define VALVES_READY_FLAG                               24

//#define ALARM_FLAGS_MASK    ((1 << PROXIMAL_FLOW_SENSOR_ERROR_FLAG) | (1 << VC_VOLUME_ERROR_FLAG) | (1 << PC_PIP_ERROR_FLAG))
#define ALARM_FLAGS_MASK    (1 << PROXIMAL_FLOW_SENSOR_ERROR_FLAG)

#endif

#ifndef _APPDEFINITION_H_
#define _APPDEFINITION_H_


typedef struct Kalman_param
{
	double A;
	double P;
	double Q;
	double H;
	double R;
	double K;
	double I;
	double X; 
}KALMAN_PARAM;



typedef struct _GUI_ModeParam
{
	int Parameter[4];
} GUI_ModeParam_Type;

typedef struct _CTRL_PARAMETER
{
	int SetPoint;
	int Minimun;
	int Maximun;
	int Over;				// 
	int Under;
} Ctrl_Parameter_Type;

typedef struct _GUI_Grid
{
	int MaxVal;
    int MinVal;
	int DivVal;
	int GridY;
} GUI_Grid_Type;
	
typedef struct _GUI_ApplicationVar
{
	GUI_ModeParam_Type Modes[3];
	Ctrl_Parameter_Type Volumen;
	Ctrl_Parameter_Type Flow;
	Ctrl_Parameter_Type Pressure;
	int ControlMode;
} GUI_ApplicationVar_Type;

typedef struct _GUI_Interface
{
	GUI_Grid_Type Grid[3];
} GUI_Interface_Type;

typedef struct _AppSetting
{
	GUI_ApplicationVar_Type ApplicationVar;
	GUI_Interface_Type Interface;
} AppSetting_Type;

typedef struct _AppStatus
{
	int Alarms;
    int AlarmCounter;
	int State;
    int Status;
    int MaxFlow;
    int MaxPressure;
    int MaxVolumen;
	int FIO2;
} AppStatus_Type;



#endif /* _APPDEFINITION_H_ */

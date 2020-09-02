#include "mcu.h"
#include "rtos.h"
#include "Driver_USART.h"
#include "Driver_GPIO.h"
#include "AppDefinition.h"
#include "Task_GUI.h"
#include "breath_sequence.h"
//#include "stdio.h"

extern int Word2StrMSB0(uint32_t Word, char *Buffer, int Digits);
extern float frec_resp_sp,  t_exp_sp, volumen_tidal_sp, peep_sp, pip_sp;//garcía
extern Breath_Mode_t modo_ventilacion;
extern volatile uint8_t alarm_enable, ventilator_start_stop;

extern AppStatus_Type m_AppStatus;
extern AppSetting_Type m_AppSetting;

#define SET_MODE_VOLUMEN_CONTROL	0
#define SET_MODE_PRESSURE_CONTROL	1
#define SET_MODE_ASYMMETRIC_PRESSURE	2

#define SET_MODE_PARAM_VOL	0
#define SET_MODE_PARAM_FREQ	1
#define SET_MODE_PARAM_IE	2
#define SET_MODE_PARAM_PEEP	3

#define SET_GRID_VOLUMEN	0
#define SET_GRID_FLOW	1
#define SET_GRID_PRESSURE	2

#define SET_GRID_YSIZE	35	// Puntos de la division de la grilla en el eje vertical
#define SET_GRID_XSIZE	550	// Puntos de la division de la grilla en el eje vertical

#define VOLUME_HIGH_LIMIT                       3000.0f    
#define VOLUME_LOW_LIMIT                        -3000.0f  
#define FLOW_HIGH_LIMIT                         120.0f    
#define FLOW_LOW_LIMIT                          -120.0f  
#define PRESSURE_HIGH_LIMIT                     60.0f    
#define PRESSURE_LOW_LIMIT                      0.0f  

static float waveform_scale_max[3] = {
                                        (VOLUME_HIGH_LIMIT),
                                        (FLOW_HIGH_LIMIT),
                                        (PRESSURE_HIGH_LIMIT)
                                        };

static float waveform_scale_min[3] = {
                                        (VOLUME_LOW_LIMIT),
                                        (FLOW_LOW_LIMIT),
                                        (PRESSURE_LOW_LIMIT)
                                        };

static float frame_signal_abs_min[3] = {
                                        (VOLUME_HIGH_LIMIT * 0.1),
                                        (FLOW_HIGH_LIMIT * 0.1),
                                        (PRESSURE_HIGH_LIMIT * 0.1)
                                        };

static float frame_signal_abs_max[3] = {
                                        (VOLUME_HIGH_LIMIT * 0.75),
                                        (FLOW_HIGH_LIMIT * 0.75),
                                        (PRESSURE_HIGH_LIMIT * 0.75)
                                        };

static float waveform_top_value[3] ={
                                        (VOLUME_HIGH_LIMIT * 0.5) ,
                                        (FLOW_HIGH_LIMIT * 0.5),
                                        (PRESSURE_HIGH_LIMIT * 0.5)
                                        };

static float waveform_bottom_value[3] = {
                                        (VOLUME_LOW_LIMIT * 0.5) ,
                                        (FLOW_LOW_LIMIT * 0.5),
                                        (PRESSURE_LOW_LIMIT * 0.5)
                                        };

//extern AppSetting_Type m_AppSetting;
//extern AppStatus_Type m_AppStatus;

int m_DataBlockReady = 0;
int m_UpdateGrid = 1;
int m_wLevel = 0;
int m_WF_LastPlot = 0;
int m_WF_EndPlot = 0;
int vXPosition=0;
int m_ResetDisplay=0;
 
/* USART Driver */
uint8_t iSignalBuffer[128];

uint8_t pTxBuffer[550];
uint8_t pRxBuffer[550];

float pCh0Buffer[600];
float pCh1Buffer[600];
float pCh2Buffer[600];

/* Inicializacion del UART */
#define BAUD_RATE (115200)

void Nextion_PlotPoints(char *pCommand, uint8_t *xPoints, int xLength)
{
	int vRxTimeOut = 0;
	int vLength = 0;
	char *pTxBuffer = pCommand;
	
	while(*pTxBuffer != 0)
	{
		vLength++;
		pTxBuffer++;
	}
	
	pTxBuffer = pCommand;
	UART2_Flush();
	UART2_SendData((uint8_t *)pTxBuffer,vLength);
	while(UART2_IsTXReady() == 0)
		osDelay(1);
	
	vLength= 0;
	vRxTimeOut = 100;
	while(vRxTimeOut > 0)
	{
		vRxTimeOut--;
		
		while(UART2_IsAvailable() > 0)
			pRxBuffer[vLength++] = UART2_ReadByte();
		
		if(vLength >= 4)
		{
			if(pRxBuffer[0]==0xFE)
			{
				if((pRxBuffer[1]==0xFF)&&(pRxBuffer[2]==0xFF)&&(pRxBuffer[3]==0xFF))
				{
					UART2_Flush();
					UART2_SendData(xPoints, xLength);
					while(UART2_IsTXReady() == 0)
						osDelay(1);
					
					vLength= 0;
					vRxTimeOut = 100;
					while(vRxTimeOut > 0)
					{
						osDelay(2);
						vRxTimeOut--;
						while(UART2_IsAvailable() > 0)
							pRxBuffer[vLength++] = UART2_ReadByte();
						
						if(vLength >= 4)
						{
							if(pRxBuffer[0]==0xFD)
							{
								;
							}
							break;
						}
					}
				}
			}
			
			break;
		}
		
		osDelay(2);
	}
}

void Nextion_SendCommamd(char *pCommand, int xTimeout)
{
	int vRxAttempts = 0;
	int vLength = 0;
	char *pTxBuffer = pCommand;
	
	while(*pTxBuffer != 0)
	{
		vLength++;
		pTxBuffer++;
	}
	
	pTxBuffer = pCommand;
	UART2_Flush();
	UART2_SendData((uint8_t *)pTxBuffer,vLength);
	while(UART2_TxLevel() > 0) osDelay(1); osDelay(1);
	
	vLength= 0;
	vRxAttempts = (xTimeout + 4)/5;
	while(vRxAttempts > 0)
	{
		vRxAttempts--;
		
		while(UART2_IsAvailable() > 0)
			pRxBuffer[vLength++] = UART2_ReadByte();
		
		if(vLength >= 4)
		{
			if(pRxBuffer[0]!=0x00)
			{
				if((pRxBuffer[1]==0xFF)&&(pRxBuffer[2]==0xFF)&&(pRxBuffer[3]==0xFF))
				{
					;
				}
			}
			
			break;
		}
		
		osDelay(5);
	}
}

int Nextion_GetNumeric(char *pCommand, int *pData, uint8_t xResponseCode, int xTimeout)
{
	volatile int vRxAttempts = 0;
	volatile int vLength = 0;
	char *pTxBuffer = pCommand;
	volatile uint32_t vNumericValue = 0;
	volatile int vError = -1;
	
	while(*pTxBuffer != 0)
	{
		vLength++;
		pTxBuffer++;
	}
	
	pTxBuffer = pCommand;
	UART2_Flush();
	UART2_SendData((uint8_t *)pTxBuffer,vLength);
	while(UART2_TxLevel() > 0) osDelay(1); osDelay(1);
	
	vLength= 0;
	vRxAttempts = (xTimeout + 4)/5;
	while(vRxAttempts > 0)
	{
		vRxAttempts--;
		
		while(UART2_IsAvailable() > 0)
			pRxBuffer[vLength++] = UART2_ReadByte();
		
		if(vLength >= 7)
		{
			// Verifica si es trama correcta
			if(pRxBuffer[0]==xResponseCode)
			{
				if((pRxBuffer[5]==0xFF)&&(pRxBuffer[6]==0xFF)&&(pRxBuffer[7]==0xFF))
				{
					vNumericValue = pRxBuffer[4];
					vNumericValue <<= 8;
					vNumericValue += pRxBuffer[3];
					vNumericValue <<= 8;
					vNumericValue += pRxBuffer[2];
					vNumericValue <<= 8;
					vNumericValue += pRxBuffer[1];
					
					*pData = vNumericValue;
					vError = 0;
				}
			}
			
			break;
		}
		else if(vLength >= 4)
		{
			// Verifica si hay algun error
			if(pRxBuffer[0]!=0x00)
			{
				if((pRxBuffer[1]==0xFF)&&(pRxBuffer[2]==0xFF)&&(pRxBuffer[3]==0xFF))
				{
					vError = -2;
				}
				else
				{
					vError = -3;
				}
				
				break;
			}
		}
		
		osDelay(5);
	}
	
	return vError;
}

int Nextion_GetPageId(int xTimeout)
{
	int vRxAttempts = 0;
	int vLength = 0;

	uint32_t vNumericValue = -1;
	
	UART2_Flush();
	UART2_SendData((uint8_t *)("sendme\xFF\xFF\xFF"),9);
	while(UART2_TxLevel() > 0) osDelay(1); osDelay(1);
	
	vLength= 0;
	vRxAttempts = (xTimeout + 4)/5;
	while(vRxAttempts > 0)
	{
		vRxAttempts--;
		
		while(UART2_IsAvailable() > 0)
			pRxBuffer[vLength++] = UART2_ReadByte();
		
		if(vLength >= 5)
		{
			// Verifica si es trama correcta
			if(pRxBuffer[0]==0x66)
			{
				if((pRxBuffer[2]==0xFF)&&(pRxBuffer[3]==0xFF)&&(pRxBuffer[4]==0xFF))
					vNumericValue = pRxBuffer[1];
			}
			
			break;
		}
		else if(vLength >= 4)
		{
			// Verifica si hay algun error
			if(pRxBuffer[0]!=0x00)
			{
				if((pRxBuffer[1]==0xFF)&&(pRxBuffer[2]==0xFF)&&(pRxBuffer[3]==0xFF))
				{
					;
				}
				
				break;
			}
		}
		
		osDelay(5);
	}
	
	return vNumericValue;
}

int GUI_GetParameters(void)
{
	volatile int vLength = 0;
	volatile int vModeId = 0;
	volatile int vParamId = 0;
	volatile int vCommFail = 0;
	volatile int vAttempts = 0;
	
	for(vModeId = 0; vModeId <= 2; vModeId++)
	{
		for(vParamId = 0; vParamId <= 3; vParamId++)
		{
			vAttempts = 10;
			do
			{
				vAttempts--;
				vLength = sprintf((char *)pTxBuffer, "get vm_m%dp%d\xFF\xFF\xFF", vModeId, vParamId + 1);
				vCommFail = Nextion_GetNumeric((char *)pTxBuffer,
                             &m_AppSetting.ApplicationVar.Modes[vModeId].Parameter[vParamId], 0x71, 100);
				osDelay(20);

				if(vCommFail == 0)
					break;
                else
                    osDelay(100);

			}while(vAttempts > 0);
			
			if(vCommFail != 0)
					break;
		}
		
		if(vCommFail != 0)
			break;
	}

	if(vCommFail == 0)	// En caso no hubo falla de comunicacion
    {
        switch(m_AppSetting.ApplicationVar.ControlMode)
        {
        case SET_MODE_VOLUMEN_CONTROL:
            modo_ventilacion= VC_CMV_MODE;
            frec_resp_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_FREQ]) / 10.0;
            t_exp_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_IE]) / 10.0;
            volumen_tidal_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_VOL]);
            peep_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_PEEP]) / 10.0;
        break;
        case SET_MODE_PRESSURE_CONTROL:
            modo_ventilacion= PC_CMV_MODE;
            frec_resp_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_FREQ]) / 10.0;
            t_exp_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_IE]) / 10.0;
            pip_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_VOL]);
            peep_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_PEEP]) / 10.0;
        break;
        case SET_MODE_ASYMMETRIC_PRESSURE:
            frec_resp_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_ASYMMETRIC_PRESSURE].Parameter[SET_MODE_PARAM_FREQ]) / 10.0;
            t_exp_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_ASYMMETRIC_PRESSURE].Parameter[SET_MODE_PARAM_IE]) / 10.0;
            volumen_tidal_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_ASYMMETRIC_PRESSURE].Parameter[SET_MODE_PARAM_VOL]);
            peep_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_ASYMMETRIC_PRESSURE].Parameter[SET_MODE_PARAM_PEEP]) / 10.0;
        break;
        default:
            frec_resp_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_FREQ]) / 10.0;
            t_exp_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_IE]) / 10.0;
            volumen_tidal_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_VOL]);
            peep_sp = ((float)m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_PEEP]) / 10.0;
        break;
        }
    }
	
	if(vCommFail == 0)	// En caso no hubo falla de comunicacion
	{
		vAttempts = 10;
		do
		{
			vAttempts--;
			vLength = sprintf((char *)pTxBuffer, "get vm_setmod\xFF\xFF\xFF");
			vCommFail = Nextion_GetNumeric((char *)pTxBuffer, &m_AppSetting.ApplicationVar.ControlMode, 0x71, 100);
			osDelay(50);

			// Salir si no hubo falla de comunicacion
			if(vCommFail == 0)
				break;
		}while(vAttempts > 0);
	}
	
	return vCommFail;
}

void EvaluateControlVar(GUI_ApplicationVar_Type *ApplicationVar)
{
	switch(ApplicationVar->ControlMode)
	{
		case SET_MODE_VOLUMEN_CONTROL:
			{
				ApplicationVar->Volumen.SetPoint = ApplicationVar->Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_VOL];
				ApplicationVar->Volumen.Maximun = (ApplicationVar->Volumen.SetPoint * 110 ) / 100;
				ApplicationVar->Volumen.Minimun = -(ApplicationVar->Volumen.SetPoint * 110 ) / 100;
				ApplicationVar->Volumen.Over = (ApplicationVar->Volumen.SetPoint * 120 ) / 100;
				ApplicationVar->Volumen.Under = -(ApplicationVar->Volumen.SetPoint * 120 ) / 100;
				
				if((ApplicationVar->Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_IE] != 0) &&
					(ApplicationVar->Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_FREQ]!=0))
					ApplicationVar->Flow.SetPoint = (100 * ApplicationVar->Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_VOL])
																				/(ApplicationVar->Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_FREQ] * 
																					ApplicationVar->Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_IE] );
				ApplicationVar->Flow.Maximun = (ApplicationVar->Flow.SetPoint * 110 ) / 100;
				ApplicationVar->Flow.Minimun = -(ApplicationVar->Flow.SetPoint * 110 ) / 100;
				ApplicationVar->Flow.Over = (ApplicationVar->Flow.SetPoint * 120 ) / 100;
				ApplicationVar->Flow.Under = -(ApplicationVar->Flow.SetPoint * 120 ) / 100;
				
				ApplicationVar->Pressure.SetPoint = ApplicationVar->Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_PEEP] * 50 / 100;
				ApplicationVar->Pressure.Maximun = (ApplicationVar->Pressure.SetPoint * 110 ) / 100;
				ApplicationVar->Pressure.Minimun = -(ApplicationVar->Pressure.SetPoint * 110 ) / 100;
				ApplicationVar->Pressure.Over = (ApplicationVar->Pressure.SetPoint * 120 ) / 100;
				ApplicationVar->Pressure.Under = -(ApplicationVar->Pressure.SetPoint * 120 ) / 100;
			}
			break;
		case SET_MODE_PRESSURE_CONTROL:
			{
				ApplicationVar->Volumen.SetPoint = ApplicationVar->Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_VOL];
				ApplicationVar->Volumen.Maximun = (ApplicationVar->Volumen.SetPoint * 110 ) / 100;
				ApplicationVar->Volumen.Minimun = -(ApplicationVar->Volumen.SetPoint * 110 ) / 100;
				ApplicationVar->Volumen.Over = (ApplicationVar->Volumen.SetPoint * 120 ) / 100;
				ApplicationVar->Volumen.Under = -(ApplicationVar->Volumen.SetPoint * 120 ) / 100;
				
				if((ApplicationVar->Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_IE] != 0) &&
					(ApplicationVar->Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_FREQ]!=0))
					ApplicationVar->Flow.SetPoint = (100 * ApplicationVar->Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_VOL])
																				/(ApplicationVar->Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_FREQ] * 
																					ApplicationVar->Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_IE] );
				ApplicationVar->Flow.Maximun = (ApplicationVar->Flow.SetPoint * 110 ) / 100;
				ApplicationVar->Flow.Minimun = -(ApplicationVar->Flow.SetPoint * 110 ) / 100;
				ApplicationVar->Flow.Over = (ApplicationVar->Flow.SetPoint * 120 ) / 100;
				ApplicationVar->Flow.Under = -(ApplicationVar->Flow.SetPoint * 120 ) / 100;
				
				ApplicationVar->Pressure.SetPoint = ApplicationVar->Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_PEEP] * 50 / 100;
				ApplicationVar->Pressure.Maximun = (ApplicationVar->Pressure.SetPoint * 110 ) / 100;
				ApplicationVar->Pressure.Minimun = -(ApplicationVar->Pressure.SetPoint * 110 ) / 100;
				ApplicationVar->Pressure.Over = (ApplicationVar->Pressure.SetPoint * 120 ) / 100;
				ApplicationVar->Pressure.Under = -(ApplicationVar->Pressure.SetPoint * 120 ) / 100;
			}
			break;
		case SET_MODE_ASYMMETRIC_PRESSURE:
			{
				ApplicationVar->Volumen.SetPoint = ApplicationVar->Modes[SET_MODE_ASYMMETRIC_PRESSURE].Parameter[SET_MODE_PARAM_VOL];
				ApplicationVar->Volumen.Maximun = (ApplicationVar->Volumen.SetPoint * 110 ) / 100;
				ApplicationVar->Volumen.Minimun = -(ApplicationVar->Volumen.SetPoint * 110 ) / 100;
				ApplicationVar->Volumen.Over = (ApplicationVar->Volumen.SetPoint * 120 ) / 100;
				ApplicationVar->Volumen.Under = -(ApplicationVar->Volumen.SetPoint * 120 ) / 100;
				
				if((ApplicationVar->Modes[SET_MODE_ASYMMETRIC_PRESSURE].Parameter[SET_MODE_PARAM_IE] != 0) &&
					(ApplicationVar->Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_FREQ]!=0))
					ApplicationVar->Flow.SetPoint = (100 * ApplicationVar->Modes[SET_MODE_ASYMMETRIC_PRESSURE].Parameter[SET_MODE_PARAM_VOL])
																				/(ApplicationVar->Modes[SET_MODE_ASYMMETRIC_PRESSURE].Parameter[SET_MODE_PARAM_FREQ] * 
																					ApplicationVar->Modes[SET_MODE_ASYMMETRIC_PRESSURE].Parameter[SET_MODE_PARAM_IE] );
				ApplicationVar->Flow.Maximun = (ApplicationVar->Flow.SetPoint * 110 ) / 100;
				ApplicationVar->Flow.Minimun = -(ApplicationVar->Flow.SetPoint * 110 ) / 100;
				ApplicationVar->Flow.Over = (ApplicationVar->Flow.SetPoint * 120 ) / 100;
				ApplicationVar->Flow.Under = -(ApplicationVar->Flow.SetPoint * 120 ) / 100;
				
				ApplicationVar->Pressure.SetPoint = ApplicationVar->Modes[SET_MODE_ASYMMETRIC_PRESSURE].Parameter[SET_MODE_PARAM_PEEP] * 50 /100;
				ApplicationVar->Pressure.Maximun = (ApplicationVar->Pressure.SetPoint * 110 ) / 100;
				ApplicationVar->Pressure.Minimun = -(ApplicationVar->Pressure.SetPoint * 110 ) / 100;
				ApplicationVar->Pressure.Over = (ApplicationVar->Pressure.SetPoint * 120 ) / 100;
				ApplicationVar->Pressure.Under = -(ApplicationVar->Pressure.SetPoint * 120 ) / 100;
			}
			break;
		default:
			break;
	}
}

int EvaluateGridDivision(int SetPoint)
{
	int vFactorX10;
	int vAbsMaxValue;
	int vTempValue;
	int vScale;
	
	if(SetPoint >= 0)
		vAbsMaxValue = SetPoint;
	else
		vAbsMaxValue = -SetPoint;
	
	vTempValue = vAbsMaxValue;
	
	// La grilla calculada tiene un factor de 100
	vScale = 100;
	while(vTempValue >0)
	{
		vScale *= 10;
		vTempValue /= 10;
	}
	
	vTempValue = vAbsMaxValue * 100;
	if(vTempValue > (vScale / 2))
	{
		;// Mantiene escala
	}
	else if(vTempValue > (vScale / 4))
	{
		vScale /= 2;	// Escala en mitad
	}
	else
	{
		vScale /= 4;	// Escala en cuarta parte
	}
	
	return vScale;
}

void EvaluateNewGrid(AppSetting_Type *ApplicationSetting)
{
	// La grilla calculada tiene un factor de x100
	ApplicationSetting->Interface.Grid[SET_GRID_VOLUMEN].DivVal = 
																EvaluateGridDivision(waveform_scale_max[0] / 2);
	ApplicationSetting->Interface.Grid[SET_GRID_FLOW].DivVal = 
																EvaluateGridDivision(waveform_scale_max[1] / 2 );
	ApplicationSetting->Interface.Grid[SET_GRID_PRESSURE].DivVal = 
																EvaluateGridDivision(waveform_scale_max[2] / 4);
	
    ApplicationSetting->Interface.Grid[SET_GRID_VOLUMEN].MaxVal = ApplicationSetting->Interface.Grid[SET_GRID_VOLUMEN].DivVal * 2;
	ApplicationSetting->Interface.Grid[SET_GRID_FLOW].MaxVal = ApplicationSetting->Interface.Grid[SET_GRID_FLOW].DivVal * 2;
	ApplicationSetting->Interface.Grid[SET_GRID_PRESSURE].MaxVal = ApplicationSetting->Interface.Grid[SET_GRID_PRESSURE].DivVal * 4;

	ApplicationSetting->Interface.Grid[SET_GRID_VOLUMEN].MinVal = -ApplicationSetting->Interface.Grid[SET_GRID_VOLUMEN].DivVal * 2;
	ApplicationSetting->Interface.Grid[SET_GRID_FLOW].MinVal = -ApplicationSetting->Interface.Grid[SET_GRID_FLOW].DivVal * 2;
	ApplicationSetting->Interface.Grid[SET_GRID_PRESSURE].MinVal = -ApplicationSetting->Interface.Grid[SET_GRID_PRESSURE].DivVal * 0;
	
	ApplicationSetting->Interface.Grid[SET_GRID_VOLUMEN].GridY = SET_GRID_YSIZE;	// Numero de puntos en el eje Y de la grilla
	ApplicationSetting->Interface.Grid[SET_GRID_FLOW].GridY = SET_GRID_YSIZE;	// Numero de puntos en el eje Y de la grilla
	ApplicationSetting->Interface.Grid[SET_GRID_PRESSURE].GridY = SET_GRID_YSIZE;	// Numero de puntos en el eje Y de la grilla
	
}

void LoadDefaultSetting(void)
{
	m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_VOL] = 750;
	m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_FREQ] = 25;
	m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_IE] = 55;
	m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_PEEP] = 80;
	
	m_AppSetting.ApplicationVar.Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_VOL] = 500;
	m_AppSetting.ApplicationVar.Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_FREQ] = 25;
	m_AppSetting.ApplicationVar.Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_IE] = 45;
	m_AppSetting.ApplicationVar.Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_PEEP] = 60;
	
	m_AppSetting.ApplicationVar.Modes[SET_MODE_ASYMMETRIC_PRESSURE].Parameter[SET_MODE_PARAM_VOL] = 500;
	m_AppSetting.ApplicationVar.Modes[SET_MODE_ASYMMETRIC_PRESSURE].Parameter[SET_MODE_PARAM_FREQ] = 25;
	m_AppSetting.ApplicationVar.Modes[SET_MODE_ASYMMETRIC_PRESSURE].Parameter[SET_MODE_PARAM_IE] = 45;
	m_AppSetting.ApplicationVar.Modes[SET_MODE_ASYMMETRIC_PRESSURE].Parameter[SET_MODE_PARAM_PEEP] = 70;
	
	m_AppSetting.ApplicationVar.ControlMode = SET_MODE_VOLUMEN_CONTROL;
	
	EvaluateControlVar(&m_AppSetting.ApplicationVar);
	EvaluateNewGrid(&m_AppSetting);
}

// La se�al debe tener un factor de 100
int ConvertSignalToY(int VarId, float *pSignalValue)
{
	int YValue;
	int iSignalValue = 0;
	
	iSignalValue = (int)(*pSignalValue * 100.00);
	
	switch(VarId)
	{
		case SET_GRID_VOLUMEN:
			{
				if(iSignalValue < m_AppSetting.Interface.Grid[SET_GRID_VOLUMEN].MinVal)
					iSignalValue = m_AppSetting.Interface.Grid[SET_GRID_VOLUMEN].MinVal;
                else if(iSignalValue > m_AppSetting.Interface.Grid[SET_GRID_VOLUMEN].MaxVal)
					iSignalValue = m_AppSetting.Interface.Grid[SET_GRID_VOLUMEN].MaxVal;
				
				if(m_AppSetting.Interface.Grid[SET_GRID_VOLUMEN].DivVal == 0)
					m_AppSetting.Interface.Grid[SET_GRID_VOLUMEN].DivVal = SET_GRID_YSIZE;
				
				YValue = (( iSignalValue - m_AppSetting.Interface.Grid[SET_GRID_VOLUMEN].MinVal) * SET_GRID_YSIZE ) /
										m_AppSetting.Interface.Grid[SET_GRID_VOLUMEN].DivVal;
			}
			break;
		case SET_GRID_FLOW:
			{
				if(iSignalValue < m_AppSetting.Interface.Grid[SET_GRID_FLOW].MinVal)
					iSignalValue = m_AppSetting.Interface.Grid[SET_GRID_FLOW].MinVal;
                else if(iSignalValue > m_AppSetting.Interface.Grid[SET_GRID_FLOW].MaxVal)
					iSignalValue = m_AppSetting.Interface.Grid[SET_GRID_FLOW].MaxVal;
				
				if(m_AppSetting.Interface.Grid[SET_GRID_FLOW].DivVal == 0)
					m_AppSetting.Interface.Grid[SET_GRID_FLOW].DivVal = SET_GRID_YSIZE;
				
				YValue = (( iSignalValue - m_AppSetting.Interface.Grid[SET_GRID_FLOW].MinVal) * SET_GRID_YSIZE ) /
										m_AppSetting.Interface.Grid[SET_GRID_FLOW].DivVal;
			}
			break;
		case SET_GRID_PRESSURE:
			{
				if(iSignalValue < m_AppSetting.Interface.Grid[SET_GRID_PRESSURE].MinVal)
					iSignalValue = m_AppSetting.Interface.Grid[SET_GRID_PRESSURE].MinVal;
                else if(iSignalValue > m_AppSetting.Interface.Grid[SET_GRID_PRESSURE].MaxVal)
					iSignalValue = m_AppSetting.Interface.Grid[SET_GRID_PRESSURE].MaxVal;
				
				if(m_AppSetting.Interface.Grid[SET_GRID_PRESSURE].DivVal == 0)
					m_AppSetting.Interface.Grid[SET_GRID_PRESSURE].DivVal = SET_GRID_YSIZE;
				
				YValue = (( iSignalValue - m_AppSetting.Interface.Grid[SET_GRID_PRESSURE].MinVal) * SET_GRID_YSIZE ) /
										m_AppSetting.Interface.Grid[SET_GRID_PRESSURE].DivVal;
			}
			break;
	}
	
	return YValue;
}

// Envia valores de las etiquetas de a grilla
void UpdateGrid(void)
{
	volatile int vPanelId = 0;
	volatile int vDivId = 0;
	volatile int vLength = 0;
	volatile int vAttempts = 0;
	volatile float vDivValue = 0;
    volatile int _vDivValue = 0;
	
	for(vPanelId = 0; vPanelId <= 2; vPanelId++)
	{
		for(vDivId = 0; vDivId <= 3; vDivId++)
		{
			vDivValue = m_AppSetting.Interface.Grid[vPanelId].MinVal + vDivId * m_AppSetting.Interface.Grid[vPanelId].DivVal;
            _vDivValue = (int)vDivValue;
			//vDivValue /= 100;
			
			if(m_AppSetting.Interface.Grid[vPanelId].DivVal >= 100)
				vLength = sprintf((char *)pTxBuffer, "p_operation.lb_g%d%d.txt=\"%d.%.2i -\"\xFF\xFF\xFF",
                                                    vPanelId+1, vDivId+1,
                                                    _vDivValue / 100,
                                                    _vDivValue % 100);
			else
				vLength = sprintf((char *)pTxBuffer, "p_operation.lb_g%d%d.txt=\"%d.%.2i -\"\xFF\xFF\xFF",
                                                    vPanelId+1,
                                                    vDivId+1,
                                                    _vDivValue / 100,
                                                    _vDivValue % 100);
			
			Nextion_SendCommamd((char *)pTxBuffer, 10);
		}
	}
	
	vLength = sprintf((char *)pTxBuffer, "p_operation.lb_sp1.txt=\"%d mL\"\xFF\xFF\xFF",
							m_AppSetting.ApplicationVar.Volumen.SetPoint);
	Nextion_SendCommamd((char *)pTxBuffer, 10);
	
	vLength = sprintf((char *)pTxBuffer, "p_operation.lb_sp2.txt=\"%d L/min\"\xFF\xFF\xFF",
							m_AppSetting.ApplicationVar.Flow.SetPoint);
	Nextion_SendCommamd((char *)pTxBuffer, 10);
	
	vLength = sprintf((char *)pTxBuffer, "p_operation.lb_sp3.txt=\"%d cm H2O\"\xFF\xFF\xFF",
							m_AppSetting.ApplicationVar.Pressure.SetPoint);
	Nextion_SendCommamd((char *)pTxBuffer, 10);
}

// Env�a valor de los parametros de los modos de operacion
void GUI_SetParameters(void)
{
	volatile int vLength = 0;
	volatile int vModeId = 0;
	volatile int vParamId = 0;

    frec_resp_sp = 20.0,  t_exp_sp= 2.0, volumen_tidal_sp= 400, peep_sp= 0.0, pip_sp= 10.0;//garcía

    m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_FREQ] = (int)(frec_resp_sp * 10.0);
    m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_IE] = (int)(t_exp_sp * 10.0);
    m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_VOL] = (int)(volumen_tidal_sp);
    m_AppSetting.ApplicationVar.Modes[SET_MODE_VOLUMEN_CONTROL].Parameter[SET_MODE_PARAM_PEEP] = (int)(peep_sp * 10.0);

    m_AppSetting.ApplicationVar.Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_FREQ] = (int)(frec_resp_sp * 10.0);
    m_AppSetting.ApplicationVar.Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_IE] = (int)(t_exp_sp * 10.0);
    m_AppSetting.ApplicationVar.Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_VOL] = (int)(pip_sp);
    m_AppSetting.ApplicationVar.Modes[SET_MODE_PRESSURE_CONTROL].Parameter[SET_MODE_PARAM_PEEP] = (int)(peep_sp * 10.0);

	for(vModeId = 0; vModeId <= 2; vModeId++)
	{
		for(vParamId = 0; vParamId <= 3; vParamId++)
		{
			
			vLength = sprintf((char *)pTxBuffer, "vm_m%dp%d=%d\xFF\xFF\xFF",
											vModeId,
											vParamId + 1,
											m_AppSetting.ApplicationVar.Modes[vModeId].Parameter[vParamId]);
			
			Nextion_SendCommamd((char *)pTxBuffer, 10);
		}
	}
	
	vLength = sprintf((char *)pTxBuffer, "vm_setmod=%d\xFF\xFF\xFF", m_AppSetting.ApplicationVar.ControlMode);
	Nextion_SendCommamd((char *)pTxBuffer, 10);
}

UART_Config_Type UsartConfig;

/* CMSIS-RTOS Thread - UART command thread */
void Task_GUI(void)
{
	int vCounter;
	int vLength;
	int vDummy;
	int vGUI_Command = 0;
	int vGUI_PageId = 0;
	int vUpdateGrid = 1;
    int vTimeout = 0;
    int vStep = 0;
    int vGUI_LastPageId = 0;
    int viTempSignal = 0;
	
	LoadDefaultSetting();
	// Leer configuracion de memoria no volatil
	
	for(vCounter=0; vCounter < SET_GRID_XSIZE; vCounter++)
	{
		pCh0Buffer[vCounter] = 0;
		pCh1Buffer[vCounter] = 0;
		pCh2Buffer[vCounter] = 0;
	}

    m_AppStatus.MaxVolumen = 0;
    m_AppStatus.MaxFlow = 0;
    m_AppStatus.MaxPressure = 0;

	osDelay(200);
	vCounter = 0;
	
	// Carga parametros de configuracion en pantalla
	GUI_SetParameters();
    m_ResetDisplay = 1;
	
	while (1)
	{
        osDelay(5);

        if(vTimeout > 0)
            vTimeout--;
        else
        {
            if(((m_wLevel % 10) < 5) || (vGUI_PageId != 0))
            {
                vTimeout = 50;

                switch(vStep)
                {
                    case 0:
                        // Verifica alarma
                        if(m_AppStatus.Alarms != 0)
                        {
                            m_AppStatus.Status |= 0x3;
                            m_AppStatus.Alarms++;
                            
                            vLength = sprintf((char *)pTxBuffer, "p_alarm.info.txt=\"Alarma detectada Nro. %d\"\xFF\xFF\xFF",
                                                                m_AppStatus.Alarms);
                            Nextion_SendCommamd((char *)pTxBuffer, 20);
                        }

                        // Actualiza estado y status
                        vLength = sprintf((char *)pTxBuffer, "vm_status=%d\xFF\xFF\xFF",m_AppStatus.Status);
                        Nextion_SendCommamd((char *)pTxBuffer, 5);
                        vLength = sprintf((char *)pTxBuffer, "vm_state=%d\xFF\xFF\xFF",m_AppStatus.State);
                        Nextion_SendCommamd((char *)pTxBuffer, 5);
                    break;
                    case 1:
                        // Lee pagina actual en pantalla
                        //vGUI_PageId = 0;
		                //vGUI_PageId = Nextion_GetPageId(100);
                        Nextion_GetNumeric((char *)("get vi_lastpage\xFF\xFF\xFF\x00"), &vGUI_PageId, 0x71, 50);
                        if(vGUI_PageId != 0)
                        {
                            m_ResetDisplay = 1;

                            // Lee solicitud de comando
                            Nextion_GetNumeric((char *)("get vm_cmd\xFF\xFF\xFF\x00"), &vGUI_Command, 0x71, 50);
                            
                            while(vGUI_Command > 0)
                            {
                                // Ejecutar comandos
                                switch(vGUI_Command)
                                {
                                    case 1: // start
                                        
                                        m_AppStatus.State = 3;
                                        ventilator_start_stop = 1;
                                        vGUI_Command = 0;
                                        break;
                                    case 2: // stop
                                        m_AppStatus.State = 1;
                                        ventilator_start_stop = 0;
                                        vGUI_Command = 0;
                                        break;
                                    case 3: // Silenciar
                                        m_AppStatus.Status &= ~(0x1 << 1);
                                        vGUI_Command = 0;
                                        break;
                                    case 10: // Guardar
                                        {
                                            m_AppStatus.Status = 0;
                                            osDelay(100);
                                            if(GUI_GetParameters() != 0)
                                                m_AppStatus.Status |= 0x3;	// En caso de error de comunicacion
                                            else
                                            {
                                                // Todos los parametros fueron l�edos
                                                EvaluateControlVar(&m_AppSetting.ApplicationVar);
                                                // Guardar configuracion en memoria no volatil
                                                vUpdateGrid = 1;
                                                vGUI_Command =0;

                                                //Nextion_SendCommamd((char *)("vm_cmd=0\xFF\xFF\xFF"), 100);
                                            }
                                        }
                                        break;
                                }

                                if(vGUI_Command == 0)
                                    Nextion_SendCommamd((char *)("vm_cmd=0\xFF\xFF\xFF"), 100);
                                
                                Nextion_GetNumeric((char *)("get vm_cmd\xFF\xFF\xFF\x00"), &vGUI_Command, 0x71, 50);
                            }
                        }
                    break;
                }

                vStep++;
                if(vStep >1)
                    vStep = 0;
            }
        }
		
		// Verifica si est� en pagina de monitoreo/operacion
		if(vGUI_PageId == 0)
		{
            if(m_ResetDisplay != 0)
            {
                osDelay(250);
                m_ResetDisplay = 0;
                vXPosition = 0;
                m_wLevel = 0;
                m_DataBlockReady = 0;
            }

            if(m_UpdateGrid != 0)
            {
                m_UpdateGrid = 0;
                UpdateGrid();
            }
            
            if(m_DataBlockReady == 0)
			    continue;
		    m_DataBlockReady = 0;

			if(vXPosition == 0)
			{
				Nextion_SendCommamd((char *)("cle 25,255\xFF\xFF\xFF"), 0);
				Nextion_SendCommamd((char *)("cle 26,255\xFF\xFF\xFF"), 0);
				Nextion_SendCommamd((char *)("cle 27,255\xFF\xFF\xFF"), 0);
			}
			
            vLength = 0;

            //viTempSignal = (int)(*(pCh0Buffer + vXPosition));
            vLength += sprintf((char *)pTxBuffer + vLength, "p_operation.v1.txt=\"%d\"\xFF\xFF\xFF", (int)m_AppStatus.MaxVolumen);
            //viTempSignal = (int)(*(pCh1Buffer + vXPosition));
            vLength += sprintf((char *)pTxBuffer + vLength, "p_operation.v2.txt=\"%d\"\xFF\xFF\xFF", (int)m_AppStatus.MaxFlow);
            //viTempSignal = (int)(*(pCh2Buffer + vXPosition));
            vLength += sprintf((char *)pTxBuffer + vLength, "p_operation.v3.txt=\"%d\"\xFF\xFF\xFF", (int)m_AppStatus.MaxPressure);
			
			vLength += sprintf((char *)pTxBuffer + vLength, "vm_fio2=%d\xFF\xFF\xFF", (int)m_AppStatus.FIO2);

            UART2_SendData(pTxBuffer, vLength);
            while(UART2_TxLevel() > 0) osDelay(1); osDelay(1);
			
			// Conviente valores de la se�al en puntos de la grilla
            vCounter = 0;
            if(m_WF_LastPlot < m_wLevel)
                m_WF_EndPlot = m_wLevel;
            else if(m_WF_LastPlot != 0)
                m_WF_EndPlot = 550;
           

            //for(vCounter = m_WF_LastPlot; vCounter < m_WF_EndPlot; vCounter++)
            while(vXPosition < m_WF_EndPlot)
            {
                vLength = 0;
                //iSignalBuffer[vCounter] = ConvertSignalToY(SET_GRID_VOLUMEN, pCh0Buffer + vXPosition + vCounter);
                iSignalBuffer[0] = ConvertSignalToY(SET_GRID_VOLUMEN, &pCh0Buffer[vXPosition]);
                vLength += sprintf((char *)pTxBuffer + vLength, "add 25,0,%d\xFF\xFF\xFF",iSignalBuffer[0]);

                //iSignalBuffer[vCounter] = ConvertSignalToY(SET_GRID_FLOW, pCh1Buffer + vXPosition + vCounter);
                iSignalBuffer[0] = ConvertSignalToY(SET_GRID_FLOW, &pCh1Buffer[vXPosition]);
                vLength += sprintf((char *)pTxBuffer + vLength, "add 26,0,%d\xFF\xFF\xFF",iSignalBuffer[0]);

                //iSignalBuffer[vCounter] = ConvertSignalToY(SET_GRID_PRESSURE, pCh2Buffer + vXPosition + vCounter);
                iSignalBuffer[0] = ConvertSignalToY(SET_GRID_PRESSURE, &pCh2Buffer[vXPosition]);
                vLength += sprintf((char *)pTxBuffer + vLength, "add 27,0,%d\xFF\xFF\xFF",iSignalBuffer[0]);

                UART2_SendData(pTxBuffer, vLength);
                while(UART2_TxLevel() > 0) osDelay(1); osDelay(1);

                vXPosition++;
            }
            m_WF_LastPlot = vXPosition;
			
			if(vXPosition >= 550)
            {
				vXPosition = 0;
                m_WF_LastPlot = 0;
            }
		}
		else if(vGUI_PageId > 0)
		{
			vXPosition = 0;
		}
		else
		{
			;
		}
	}
}

float GetMaxAbsoluteValue(float *pBuffer, int Length)
{
    volatile float vMaxValue = 0;
    volatile float vTempValue = 0;

    while(Length > 0)
    {
        Length--;
        vTempValue = *pBuffer++;
        if(vTempValue < 0)
            vTempValue = -vTempValue;

        if(vTempValue > vMaxValue)
            vMaxValue = vTempValue;
    }

    return vMaxValue;
}


void UpdateMaxValues(int Flow, int Pressure, int Volumen)
{
    m_AppStatus.MaxFlow = (int)(Flow);
    m_AppStatus.MaxPressure = (int)(Pressure);
    m_AppStatus.MaxVolumen = (int)(Volumen);
}

void InsertPoints(float *pFlow, float *pPressure, float *pVolumen)
{
    int idx = 0;

    if(m_wLevel == 0)
    {
        frame_signal_abs_max[0] = GetMaxAbsoluteValue(pCh0Buffer, SET_GRID_XSIZE);
        frame_signal_abs_max[1] = GetMaxAbsoluteValue(pCh1Buffer, SET_GRID_XSIZE);
        frame_signal_abs_max[2] = GetMaxAbsoluteValue(pCh2Buffer, SET_GRID_XSIZE);

        waveform_scale_max[0] = VOLUME_HIGH_LIMIT;
        waveform_scale_max[1] = FLOW_HIGH_LIMIT;
        waveform_scale_max[2] = PRESSURE_HIGH_LIMIT;

        for(idx = 0; idx < 3; idx++)
        {
            if(frame_signal_abs_max[idx] > frame_signal_abs_min[idx])
               waveform_scale_max[idx] = frame_signal_abs_max[idx] * 5 / 4;
        }

        waveform_scale_min[0] = -waveform_scale_max[0];
        waveform_scale_min[1] = -waveform_scale_max[1];
        waveform_scale_min[2] = PRESSURE_LOW_LIMIT;

        EvaluateNewGrid(&m_AppSetting);
        m_UpdateGrid = 1;
        //m_WF_LastPlot = 0;
    }

    pCh1Buffer[m_wLevel] = *pFlow;
    pCh2Buffer[m_wLevel] = *pPressure;
    pCh0Buffer[m_wLevel] = *pVolumen;

    m_wLevel++;

    if((m_wLevel % 10) == 0)
        m_DataBlockReady = 1;
    
    if(m_wLevel >= 550)
		m_wLevel = 0;
}

void Task_Timers (void)
{
	
  while (1)
	{
        //UART2_SendData((uint8_t *)"UU", 2); while(UART2_TxLevel() > 0) osDelay(1); osDelay(1);
        //USART2->TDR = 0x55;
		osDelay(50);
		//m_DataBlockReady = 1;
  }
}

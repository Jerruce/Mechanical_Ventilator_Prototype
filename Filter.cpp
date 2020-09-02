#include "Filter.h"


void Kalman_init(KALMAN_PARAM *pKalman)
{
		pKalman->A = 1;
		pKalman->P = 1;
	    // Kalman.Q = 0.00050;
		pKalman->Q = 0.02;		//1
		pKalman->H = 1;
		pKalman->R = 0.5;	//200
		pKalman->K = 1;
		pKalman->I = 1;
		pKalman->X = 1; 
}

double Filtering_Kalman(KALMAN_PARAM *pKalman, double NewData)
{
	if(NewData > (pKalman->X + 2500))
	{
		pKalman->X = NewData;
	}
	
	if(NewData < (pKalman->X - 2500))
	{
		pKalman->X = NewData;
	}
		
		pKalman->P = pKalman->A * pKalman->P * pKalman->A + pKalman->Q;
		pKalman->K = pKalman->P * pKalman->H * 1.0/(pKalman->H * pKalman->P * pKalman->H + pKalman->R);
		pKalman->P = (pKalman->I - pKalman->K * pKalman->H) * pKalman->P;
		pKalman->X = pKalman->A * pKalman->X;
		pKalman->X = pKalman->X + pKalman->K * (NewData - pKalman->H * pKalman->X);
		return pKalman->X;
}

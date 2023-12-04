#include "RPIWAlib.h"
#include <math.h>

void SignalNode::begin(int channel, float CAL)
{
	int i;
	for (i=0; i<DATA_ARR_LEN; i++)
	{
		centred2_arr[i] = 0.0;
		//raw_arr[i] = 0;
	}

	idx_arr = 0;
	offset = ADC_COUNTS/2;
	sum_centred2 = 0.0;
	//sum_raw = 0;
	
	RATIO = CAL * ((ADC_REF / 1000.0) / ADC_COUNTS);
	
	centred = ADC_COUNTS/2.;
	pVal = centred;

}

void SignalNode::updateRMS(int newVal)
{
	//float centred;
	//float a = 0.0001;
	//offset = offset + (newVal - offset) *a;
	
	float b0 = 0.9984316659167189;
	float b1 = -0.9984316659167189;
	float a1 = -0.9968633318334380;

	centred = b0*newVal + b1*pVal - a1*centred;
	pVal = newVal;
	offset = newVal-centred;
	/*
	sum_raw -= raw_arr[idx_arr];
	raw_arr[idx_arr] = newVal;
	sum_raw += raw_arr[idx_arr];
	offset = ((float)sum_raw)/DATA_ARR_LEN;
	*/
	
	//centred = newVal - offset;
	
	sum_centred2 -= centred2_arr[idx_arr];
	centred2_arr[idx_arr] = centred*centred;
	sum_centred2 += centred2_arr[idx_arr];
	
	RMS = RATIO * sqrt(sum_centred2/DATA_ARR_LEN);

	if (++idx_arr==DATA_ARR_LEN) 
	{
		idx_arr=0;
		warm = 1;
	}

}

void PowerNode::begin(int Ichannel, float ICAL, int Vchannel, float VCAL)
{
	int i;
	for (i=0; i<DATA_ARR_LEN; i++)
	{
		Icentred2_arr[i] = 0.0;
		Vcentred2_arr[i] = 0.0;
		Pcentred2_arr[i] = 0.0;
	}

	idx_arr = 0;
	offsetI = ADC_COUNTS/2;
	offsetV = ADC_COUNTS/2;
	sumI = 0.0;
	sumV = 0.0;
	sumP = 0.0;
	I_RATIO = ICAL * ((ADC_REF / 1000.0) / ADC_COUNTS);
	V_RATIO = VCAL * ((ADC_REF / 1000.0) / ADC_COUNTS);
	warm = 0;
	// Best we can do here is to initialise on zero line
	// Will make erroneous data only fo rthe first sample
	// First sample being part of the warmup anyway.
	prev_centredI = 0.0;

}



void PowerNode::updateRMS(int IVal, int VVal)
{
	float centredI, centredV, tmp_centredI;
	// Computing the zero line with an IIR filter
	offsetI = offsetI + (IVal - offsetI) / 1024;
	offsetV = offsetV + (VVal - offsetV) / 1024;
	// Recentring data to zero
	tmp_centredI = IVal - offsetI;
	centredV = VVal - offsetV;
	// Shifting I to V with linear interpolation
	// Assuming V has lower channel than I (otherwise we extrapolate)
	// Assuming 8 channels in total
	centredI = (prev_centredI*(Ichannel-Vchannel)+tmp_centredI*(Vchannel-Ichannel+8))/8;
	// Only storing for next computation the unshifted data
	prev_centredI = tmp_centredI;
	
	// Subtracting from the sum thwe oldest data in buffer
	sumI -= Icentred2_arr[idx_arr];
	sumV -= Vcentred2_arr[idx_arr];
	sumP -= Pcentred2_arr[idx_arr];
	// Replacing the new value in the array
	Icentred2_arr[idx_arr] = centredI*centredI;
	Vcentred2_arr[idx_arr] = centredV*centredV;
	Pcentred2_arr[idx_arr] = centredV*centredI;
	// Adding it back in the sum
	sumI += Icentred2_arr[idx_arr];
	sumV += Vcentred2_arr[idx_arr];
	sumP += Pcentred2_arr[idx_arr];
	// Converting from raw data to real world 
	Irms = I_RATIO * sqrt(sumI/DATA_ARR_LEN);
	Vrms = V_RATIO * sqrt(sumV/DATA_ARR_LEN);
	P = I_RATIO * V_RATIO * sumP / DATA_ARR_LEN;

	if (++idx_arr==DATA_ARR_LEN) 
	{
		idx_arr=0;
		warm = 1;
	}
}


void InstantNode::begin(float CAL)
{

	offset = ADC_COUNTS/2;
	RATIO = CAL * ((ADC_REF / 1000.0) / ADC_COUNTS);

}

void InstantNode::convert(int rawVal)
{
	float centred;
	offset = offset + (rawVal - offset) / 1024;
	centred = rawVal - offset;
	value = RATIO * centred;


}






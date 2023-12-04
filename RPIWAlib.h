

#ifndef RPIWAlib_h
#define RPIWAlib_h

#define DATA_ARR_LEN 1249
#define ADC_REF 4096
#define ADC_BITS 12
#define ADC_COUNTS (1<<ADC_BITS)

class SignalNode
{
	public:
		void begin(int channel, float CAL);
		void updateRMS(int  newVal);
		float RMS;
		float offset;
		int warm;

	private:
		float centred2_arr[DATA_ARR_LEN];
		int raw_arr[DATA_ARR_LEN];
		int err;
		
		float RATIO;
		int channel;
		float sum_centred2;
		//int sum_raw;
		int idx_arr;
		float centred, pVal;
};

class PowerNode
{
	public:
		void begin(int Ichannel, float ICAL, int Vchannel, float VCAL);
		void updateRMS(int IVal, int VVal);
		float getRMS();
		float Irms, Vrms, P;
		float prev_centredI;
		int warm;
		
		

	private:
		float Icentred2_arr[DATA_ARR_LEN];
		float Vcentred2_arr[DATA_ARR_LEN];
		float Pcentred2_arr[DATA_ARR_LEN];
		int err;
		float offsetI, offsetV;
		float I_RATIO, V_RATIO;
		int Ichannel, Vchannel;
		float sumI, sumV, sumP;
		int idx_arr;
};


class InstantNode
{
	public:
		void begin(float CAL);
		void convert(int rawVal);
		float value;

	private:
		
		float offset;
		float RATIO;

};































#endif

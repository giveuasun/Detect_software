#pragma once
#include <cmath>
#include <ipp.h>
class DVB
{
public:
	void init();
	void work(Ipp32f* data, int datalen, float sample_rate, float freq, float Bd, float* thread);
private:
	void resampleIPP(int inRate, int outRate, Ipp32f* inData, int inLen, Ipp32f* outData, int& outLen/*这个是输出参数,调用的时候不传或者传0就行*/);
};



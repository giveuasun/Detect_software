#pragma once
#include <cmath>
#include <ipp.h>
class DVB
{
public:
	int work(Ipp32f* data, int datalen, float sample_rate, float freq, float Bd, float thread);
};



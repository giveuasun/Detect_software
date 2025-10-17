#pragma once
#include <iostream>
#include <ipp.h>
#define IPPCHECK(func) \
{\
	IppStatus status = (func);\
	if (status != ippStsNoErr){\
		fprintf(stderr,"[IPP±¨´í] %s %d | %s failed (code:%d)\n",__FILE__,__LINE__,#func, status);\
	}\
}

class CDMA3GPP
{
public:
	int work(Ipp32f* data, int datalen, float sample_rate, float freq, float thread);
private:
	void r2iq(float freq, float fs, float* pSrc, float* pDstI, float* pDstQ, size_t len);
	void lpfCoefDesignIPP(float rFreq, int tapsLen, float* pDst);
	void doLpfIPP(float* pSrc, float* pDst, int len, float rFreq, int tapsLen);
	void resampleIPP(int inRate, int outRate, Ipp32f* inData, int inLen, Ipp32f* outData, int& outLen);
	void PNGen(float* f, float* regIni, int order, int way, float* m);
};


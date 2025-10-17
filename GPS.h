#pragma once
#include <iostream>
#include <ipp.h>
#define IPPCHECK(func) \
{\
	IppStatus status = (func);\
	if (status != ippStsNoErr){\
		fprintf(stderr,"[IPP报错] %s %d | %s failed (code:%d)\n",__FILE__,__LINE__,#func, status);\
	}\
}
class GPS
{
public:
	int work(Ipp32f* data, int datalen, float sample_rate, float freq, float thread);
private:
	void r2iq(float freq, float fs, float* pSrc, float* pDstI, float* pDstQ, size_t len);

	void lpfCoefDesignIPP(float rFreq, int tapsLen, float* pDst);

	void doLpfIPP(float* pSrc, float* pDst, int len, float rFreq, int tapsLen);

	//调用先计算：int outLen = (float)inLen * outRate / inRate;
	//并且分配outData的内存,函数执行后outLen记得检查
	void resampleIPP(int inRate, int outRate, Ipp32f* inData, int inLen, Ipp32f* outData, int& outLen);
	void PNGen(float* f, float* regIni, int order, int way, float* m);
	void PNGenGPS(float* f, float* regIni, int order, int way, int index, float* m);
};


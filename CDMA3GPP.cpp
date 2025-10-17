#include "CDMA3GPP.h"

void CDMA3GPP::r2iq(float freq, float fs, float* pSrc, float* pDstI, float* pDstQ, size_t len)
{
	float* cosData = ippsMalloc_32f(len); float* sinData = ippsMalloc_32f(len);
	Ipp32f normF = 2 * IPP_PI * freq / fs;
	for (size_t i = 0; i < len; i++)
	{
		cosData[i] = cos(i * normF);
		sinData[i] = sin(i * normF);
	}
	IPPCHECK(ippsMul_32f(pSrc, cosData, pDstI, len)); IPPCHECK(ippsMul_32f(pSrc, sinData, pDstQ, len));
	ippsFree(cosData); ippsFree(sinData);
}

void CDMA3GPP::lpfCoefDesignIPP(float rFreq, int tapsLen, float* pDst)
{
	int* pLowpassBufferSize = new int(0);
	IPPCHECK(ippsFIRGenGetBufferSize(tapsLen, pLowpassBufferSize));
	Ipp64f* pTaps = ippsMalloc_64f(tapsLen);
	Ipp8u* pBuf = ippsMalloc_8u(*pLowpassBufferSize);
	IPPCHECK(ippsFIRGenLowpass_64f((Ipp64f)rFreq, pTaps, tapsLen, ippWinHamming /*winType*/, ippTrue/*doNormal*/, pBuf));
	ippsConvert_64f32f(pTaps, pDst, tapsLen);
	ippsFree(pTaps);
	ippsFree(pBuf);
}

void CDMA3GPP::doLpfIPP(float* pSrc, float* pDst, int len, float rFreq, int tapsLen)
{
	Ipp32f* pTaps_32f = ippsMalloc_32f(tapsLen);
	lpfCoefDesignIPP(rFreq, tapsLen, pTaps_32f);
	int specSize, bufSize;
	IPPCHECK(ippsFIRSRGetSize(tapsLen, ipp32f, &specSize, &bufSize));
	float* dly = ippsMalloc_32f(tapsLen - 1);
	Ipp8u* buf = ippsMalloc_8u(bufSize);
	IppsFIRSpec_32f* pSpec = (IppsFIRSpec_32f*)ippsMalloc_8u(specSize);
	ippsFIRSRInit_32f(pTaps_32f, tapsLen, ippAlgDirect, pSpec);
	ippsFIRSR_32f(pSrc, pDst, len, pSpec, NULL, dly, buf);
	ippsFree(dly);
	ippsFree(buf);
	ippsFree(pTaps_32f);
}

//调用先计算：int outLen = (float)inLen * outRate / inRate;
//并且分配outData的内存,函数执行后outLen记得检查
void CDMA3GPP::resampleIPP(int inRate, int outRate, Ipp32f* inData, int inLen, Ipp32f* outData, int& outLen)
{
	int size, len, height;
	int fixed_len = 35;//固定因子重采样滤波器长度
	ippsResamplePolyphaseFixedGetSize_32f(inRate, outRate, fixed_len, &size, &len, &height, ippAlgHintAccurate);
	IppsResamplingPolyphaseFixed_32f* state = (IppsResamplingPolyphaseFixed_32f*)ippsMalloc_8u(size);
	ippsResamplePolyphaseFixedInit_32f(inRate, outRate, fixed_len, 0.95f, 9.0f, state, ippAlgHintAccurate);
	Ipp64f time = 0;//开始时间
	ippsResamplePolyphaseFixed_32f(inData, inLen, outData, 0.98f, &time, &outLen, state);
	ippsFree(state);
}

void CDMA3GPP::PNGen(float* f, float* regIni, int order, int way, float* m)
{
	int m_len = pow(2, order) - 1;
	int f_len = order + 1;
	float* f1 = ippsMalloc_32f(order);
	for (int i = 0; i < f_len - 1; i++)
		f1[i] = f[i + 1];
	float* reg = ippsMalloc_32f(order);
	for (int i = 0; i < order; i++)
		reg[i] = regIni[i];
	//START
	if (f[f_len - 1] != 1)
		ippsZero_32f(m, m_len);
	else {
		switch (way) {
		case 0:
		{
			for (int i = 0; i < m_len; i++)
			{
				m[i] = reg[order - 1];
				float temp = reg[order - 1];
				for (int j = order - 1; j >= 1; j--)
				{
					temp = temp + f1[j - 1] * reg[j - 1];
					if (temp == 2)temp = 0;
				}
				for (int j = order; j >= 2; j--)
				{
					reg[j - 1] = reg[j - 2];
				}
				reg[0] = temp;
			}
			break;
		}
		case 1:
		{
			for (int i = 0; i < m_len; i++)
			{
				m[i] = reg[order - 1];
				float temp = reg[order - 1];
				for (int j = order; j >= 2; j--)
				{
					reg[j - 1] = reg[j - 2] + f1[j - 2] * temp;
					if (temp == 2)temp = 0;
				}
				reg[0] = temp;
			}
			break;
		}
		default:
		{
			ippsZero_32f(m, m_len);
			break;
		}
		}
	}
	ippsFree(f1);
	ippsFree(reg);
}

int CDMA3GPP::work(Ipp32f* data, int datalen, float sample_rate, float freq, float thread)
{
	//分解
	Ipp32f* vec_i = new Ipp32f[datalen]; Ipp32f* vec_q = new Ipp32f[datalen];
	r2iq(freq, sample_rate, data, vec_i, vec_q, datalen);
	//低通滤波器设计
	float Wn = 1.2288e+6 / (sample_rate / 2);
	Wn = Wn / 2;//ipp库的滤波器输入参数和matlab不一样
	int N = 35;//滤波器抽头数
	//执行滤波
	float* lpfPass_i = ippsMalloc_32f(datalen);
	float* lpfPass_q = ippsMalloc_32f(datalen);
	doLpfIPP(vec_i, lpfPass_i, datalen, Wn, N);
	doLpfIPP(vec_q, lpfPass_q, datalen, Wn, N);
	delete[] vec_i; delete[] vec_q;
	//重采
	float Bd = 1.2288e+6; //这个是定值
	int reLen = (float)datalen * Bd * 4 / sample_rate;
	float* re_i = ippsMalloc_32f(reLen); float* re_q = ippsMalloc_32f(reLen);
	resampleIPP(sample_rate, Bd * 4, lpfPass_i, datalen, re_i, reLen);
	resampleIPP(sample_rate, Bd * 4, lpfPass_q, datalen, re_q, reLen);
	ippsFree(lpfPass_i); ippsFree(lpfPass_q);
	//解析
	Ipp32fc* cplx = ippsMalloc_32fc(reLen);
	ippsRealToCplx_32f(re_i, re_q, cplx, reLen);
	ippsFree(re_i); ippsFree(re_q);



	//伪码生成
	float f[] = { 1,0,1,0,0,0,1,1,1,0,1,0,0,0,0,1 };
	float regIni[] = { 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	int spOrder = 15;
	int m_len = pow(2, spOrder) - 1;
	float* m = ippsMalloc_32f(m_len);
	PNGen(f,regIni,spOrder,0,m);
	


	float* loc_PN = ippsMalloc_32f(32768);
	ippsZero_32f(loc_PN, 32768);
	for (int i = 1; i < 32768; i++)
	{
		loc_PN[i] = m[i - 1];
	}
	for (int i = 0; i < 32768; i++)
	{
		loc_PN[i] = 2 * loc_PN[i] - 1;
	}
	//相关
	int fralength = 1024;
	int it = 32768 / fralength;
	int corLen = reLen - 4 * 32768;
	float* corrl = ippsMalloc_32f(corLen);
	ippsZero_32f(corrl, corLen);

	Ipp32fc* huadong = ippsMalloc_32fc(32768);
	Ipp32fc* datafrag = ippsMalloc_32fc(1024);
	Ipp32f* PNfrag = ippsMalloc_32f(1024);
	Ipp32f* PNfrag_im = ippsMalloc_32f(1024);
	ippsZero_32f(PNfrag_im, 1024);
	Ipp32fc* PNfrag_c = ippsMalloc_32fc(1024);
	Ipp32fc* corrout = ippsMalloc_32fc(1024);

	Ipp32fc corrtmp;
	Ipp32f tmp2;

	for (size_t i = 0; i < corLen; i++)
	{
		for (size_t j = 0; j < 32768; j++)
		{
			huadong[j] = cplx[i + j * 4];
		}
		for (size_t k = 0; k < it; k++)
		{
			for (size_t s = 0; s < fralength; s++)
			{
				datafrag[s] = huadong[k * fralength + s];
			}
			for (size_t s = 0; s < fralength; s++)
			{
				PNfrag[s] = loc_PN[k * fralength + s];
			}
			ippsRealToCplx_32f(PNfrag,PNfrag_im,PNfrag_c,1024);
			ippsMul_32fc(PNfrag_c, datafrag, corrout, 1024);
			ippsSum_32fc(corrout, 1024, &corrtmp, ippAlgHintNone);
			ippsMagnitude_32fc(&corrtmp, &tmp2, 1);
			corrl[i] = tmp2*tmp2;
		}
	}
	//释放




	//判别
	int kind = 0;
	int num, num1;
	for (size_t i = 0; i < corLen; i++)
	{
		if (corrl[i]>thread)
		{
			num = i+1;
			num1 = num + 32768 * 4;
			if (num1 < corLen)
			{
				if (corrl[num1] > thread)
				{
					kind = 1;
					break;
				}
			}
		}
	}
	ippsFree(corrl);
	return kind;
}

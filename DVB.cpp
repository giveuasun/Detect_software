#include "DVB.h"

/// <summary>
/// DVB识别程序work
/// </summary>
/// <param name="data">数据指针</param>
/// <param name="datalen">输入数据长度</param>
/// <param name="sample_rate">采样率</param>
/// <param name="freq">频率</param>
/// <param name="Bd">符号速率</param>
/// <param name="thread">门限</param>
/// <returns>识别种类</returns>
int DVB::work(Ipp32f* data, int datalen, float sample_rate, float freq, float Bd, float thread)
{
	//正交分解
	Ipp32f* vec_i = new Ipp32f[datalen]; Ipp32f* vec_q = new Ipp32f[datalen];
	r2iq(freq, sample_rate, data, vec_i, vec_q, datalen);
	//低通滤波器设计
	float Wn = 7.5e+6 / (sample_rate / 2); 
	Wn = Wn / 2;//ipp库的滤波器输入参数和matlab不一样
	int N = 35;//滤波器抽头数
	//执行滤波
	float* lpfPass_i = ippsMalloc_32f(datalen);
	float* lpfPass_q = ippsMalloc_32f(datalen);
	doLpfIPP(vec_i, lpfPass_i, datalen, Wn, N);
	doLpfIPP(vec_q, lpfPass_q, datalen, Wn, N);
	delete[] vec_i; delete[] vec_q;
	//重采
	int reLen = (float)datalen * Bd * 4 / sample_rate;
	float* re_i = ippsMalloc_32f(reLen); float* re_q = ippsMalloc_32f(reLen);
	resampleIPP(sample_rate, Bd * 4, lpfPass_i, datalen, re_i, reLen);
	resampleIPP(sample_rate, Bd * 4, lpfPass_q, datalen, re_q, reLen);
	ippsFree(lpfPass_i); ippsFree(lpfPass_q);
	//解析
	Ipp32fc* cplx = ippsMalloc_32fc(reLen);
	ippsRealToCplx_32f(re_i, re_q, cplx, reLen);
	ippsFree(re_i); ippsFree(re_q);
	////相关
	//调制
	Ipp32fc* p_UW = ippsMalloc_32fc(26);
	modUW(p_UW);
	//相关执行
	Ipp32f* corr_m = ippsMalloc_32f(reLen - 104);//储存相关模
	corr(cplx, reLen, p_UW, corr_m);
	ippsFree(p_UW);
	ippsFree(cplx);
	////判断过程
	int kind = 0;//后续存放判断结果
	size_t num, num1, num2, num3;
	int length = reLen - 104;
	for (size_t i = 0; i < length; i++)
	{
		if (corr_m[i] > thread)
		{
			num = i + 1;
			num1 = num + 32490 * 4;
			num2 = num + 33282 * 4;
			num3 = num + 33372 * 4;
			if (num1 < length)
			{
				if (corr_m[num1] > thread)
				{
					kind = 1;
					break;
				}
			}
			if (num2 < length)
			{
				if (corr_m[num2] > thread)
					kind = 2;
			}
			if (num3 < length)
			{
				if (corr_m[num3] > thread)
				{
					kind = 3;
					break;
				}
			}
		}
	}
	ippsFree(corr_m);
	return kind;
}

void DVB::r2iq(float freq, float fs, float* pSrc, float* pDstI, float* pDstQ, size_t len)
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

void DVB::lpfCoefDesignIPP(float rFreq, int tapsLen, float* pDst)
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

void DVB::doLpfIPP(float* pSrc, float* pDst, int len, float rFreq, int tapsLen)
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
void DVB::resampleIPP(int inRate, int outRate, Ipp32f* inData, int inLen, Ipp32f* outData, int& outLen)
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

void DVB::modUW(Ipp32fc* mod)//mod长度卡死26
{
	int SOF_bit[] = { 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0 };//26bit的SOF
	Ipp32fc cplx_arr[] = { {1,1} ,{-1,1}, {-1,-1}, {1,-1} };
	for (size_t i = 0; i < 13; i++)
	{
		if (SOF_bit[2 * i] == 0)
		{
			mod[2 * i] = { 1,1 };
		}
		else
		{
			mod[2 * i] = { -1,-1 };
		}
		if (SOF_bit[2 * i + 1] == 0)
		{
			mod[2 * i + 1] = { -1,1 };
		}
		else
		{
			mod[2 * i + 1] = { 1,-1 };
		}
	}
}

//预先分配内存：Ipp32f* corr_m = ippsMalloc_32f(dataLen - 104);//储存相关模
void DVB::corr(Ipp32fc* pData, int Len, Ipp32fc* pUW, Ipp32f* corr_m)
{
	Ipp32fc* sigcr1 = ippsMalloc_32fc(104);
	Ipp32fc* sigcr2 = ippsMalloc_32fc(104);
	Ipp32fc* sigcr_out = ippsMalloc_32fc(104);
	Ipp32fc tmp;
	Ipp32f tmp2;
	ippsConj_32fc(pUW, sigcr2, 104);
	for (size_t i = 0; i < Len - 104; i++)
	{
		for (size_t j = 0; j < 26; j++)
		{
			sigcr1[j] = pData[i + 4 * j];
		}
		ippsMul_32fc(sigcr1, sigcr2, sigcr_out, 26);
		ippsSum_32fc(sigcr_out, 26, &tmp, ippAlgHintNone);
		ippsMagnitude_32fc(&tmp, &tmp2, 1);
		corr_m[i] = tmp2;
	}
	ippsFree(sigcr1);
	ippsFree(sigcr2);
	ippsFree(sigcr_out);
}
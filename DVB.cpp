#include "DVB.h"
#include <iostream>
#define IPPCHECK(func) \
{\
	IppStatus status = (func);\
	if (status != ippStsNoErr){\
		fprintf(stderr,"[IPP报错] %s %d | %s failed (code:%d)\n",__FILE__,__LINE__,#func, status);\
	}\
}
//#define IPPCHECK(func)
void DVB::work(Ipp32f* data, int datalen, float sample_rate, float freq, float Bd, float* thread)
{
	//正弦余弦波生成
	Ipp32f normF = freq / sample_rate;
	normF = 2 * IPP_PI * normF;
	Ipp32f* sinVec = new Ipp32f[datalen];
	Ipp32f* cosVec = new Ipp32f[datalen];
	for (size_t i = 0; i < datalen; i++)
	{
		sinVec[i] = sin(i * normF);
		cosVec[i] = cos(i * normF);
	}
	//与数据逐个相乘
	Ipp32f* sinVecMul = new Ipp32f[datalen];
	Ipp32f* cosVecMul = new Ipp32f[datalen];
	IPPCHECK(ippsMul_32f(data, sinVec, sinVecMul, datalen));
	IPPCHECK(ippsMul_32f(data, cosVec, cosVecMul, datalen));
	delete[] sinVec;
	delete[] cosVec;
	//低通滤波器设计
	float Wn = 7.5e+6 / (sample_rate / 2);
	int N = 35; //滤波器抽头数
	Ipp64f* pTaps = new Ipp64f[N];
	Ipp32f* pTaps_32f = new Ipp32f[N];
	int* pLowpassBufferSize = new int(0);
	IPPCHECK(ippsFIRGenGetBufferSize(N, pLowpassBufferSize));
	Ipp8u* pBuf = new Ipp8u[*pLowpassBufferSize];
	IPPCHECK(ippsFIRGenLowpass_64f((Ipp64f)Wn, pTaps, N, ippWinHamming /*winType*/, ippTrue/*doNormal*/, pBuf));
	ippsConvert_64f32f(pTaps, pTaps_32f, N);
	delete[] pTaps;
	//滤波过程
	
	float* dly;
	Ipp8u* buf;
	//get sizes of the spec structure and the work buffer
	int specSize, bufSize;
	IPPCHECK(ippsFIRSRGetSize(N, ipp32f, &specSize, &bufSize));//滤波器


	float* dstsin = ippsMalloc_32f(datalen);
	float* dstcos = ippsMalloc_32f(datalen);

	dly = ippsMalloc_32f(N - 1);

	IppsFIRSpec_32f* pSpec = (IppsFIRSpec_32f*)ippsMalloc_8u(specSize);
	buf = ippsMalloc_8u(bufSize);

	ippsFIRSRInit_32f(pTaps_32f, N, ippAlgDirect, pSpec);
	////apply the FIR filter
	ippsFIRSR_32f(sinVecMul, dstsin, datalen, pSpec, NULL, dly, buf);
	ippsFIRSR_32f(cosVecMul, dstcos, datalen, pSpec, NULL, dly, buf);

	//重采样，为相关做铺垫
	int outLenSin = 0, outLenCos = 0;
	Ipp32f* outDataSin = new Ipp32f[datalen];
	Ipp32f* outDataCos = new Ipp32f[datalen];
	resampleIPP(sample_rate, Bd * 4, dstsin, datalen, outDataSin, outLenSin);
	resampleIPP(sample_rate, Bd * 4, dstcos, datalen, outDataCos, outLenCos);
	Ipp32fc* cplx = new Ipp32fc[outLenCos];
	ippsRealToCplx_32f(outDataCos, outDataSin, cplx, outLenCos);
	////相关
	//调制
	int SOF_bit[] = { 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0 };//26bit的SOF
	Ipp32fc cplx_arr[] = { {1,1} ,{-1,1}, {-1,-1}, {1,-1} };
	Ipp32fc* mod = new Ipp32fc[26];
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
	//相关过程（结果暂时不太对）
	Ipp32fc* corr1 = new Ipp32fc[outLenCos];
	Ipp32f* corr2 = new Ipp32f[outLenCos];
	Ipp32fc* sigcr1 = new Ipp32fc[104];
	Ipp32fc* sigcr2 = new Ipp32fc[104];
	Ipp32fc* sigcr_out = new Ipp32fc[104];
	Ipp32fc tmp;
	Ipp32f tmp2;
	ippsConj_32fc(mod, sigcr2, 104);

	for (size_t i = 0; i < outLenCos - 104; i++)
	{
		for (size_t j = 0; j < 26; j++)
		{
			sigcr1[j] = cplx[i + 4 * j];
		}
		IPPCHECK(ippsMul_32fc(sigcr1, sigcr2, sigcr_out, 26));
		ippsSum_32fc(sigcr_out, 26, &tmp, ippAlgHintNone);
		ippsMagnitude_32fc(&tmp, &tmp2, 1);
		corr2[i] = tmp2;
	}
	////判断过程



	int kind = 0;
	float mean = 0;
	ippsMean_32f(corr2, outLenCos, &mean, ippAlgHintNone);
	if (thread == nullptr)
	{
		*thread = 5 * mean; // 设置一个相关值门限，这个要作为输入参数
	}
	int num = 0;
	for (size_t i1 = 0; i1 < outLenCos; i1++)
	{
		if (true)
		{

		}
	}



	//集中释放，后续可重新编排结构
	//ippsFree(src);
	//ippsFree();
	//ippsFree();

	delete pLowpassBufferSize;
	delete[] pBuf;
	delete[] pTaps_32f;

	delete[] sinVecMul;
	delete[] cosVecMul;

}



void DVB::resampleIPP(int inRate, int outRate, Ipp32f* inData, int inLen, Ipp32f* outData, int& outLen/*输出参数,调用时不传*/)
{
	//缺陷，需在外部分配输出内存
	int size, len, height;
	int fixed_len = 35;//固定因子重采样滤波器长度
	ippsResamplePolyphaseFixedGetSize_32f(inRate, outRate, fixed_len, &size, &len, &height, ippAlgHintAccurate);
	IppsResamplingPolyphaseFixed_32f* state = (IppsResamplingPolyphaseFixed_32f*)ippsMalloc_8u(size);
	ippsResamplePolyphaseFixedInit_32f(inRate, outRate, fixed_len, 0.95f, 9.0f, state, ippAlgHintAccurate);
	Ipp64f time = 0;//？似乎是开始时间
	ippsResamplePolyphaseFixed_32f(inData, inLen, outData, 0.98f, &time, &outLen, state);
	ippsFree(state);
}

//void DVB::lpfDesignIPP()
//{
//
//}
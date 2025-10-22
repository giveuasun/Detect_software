#include "GPS.h"

void GPS::r2iq(float freq, float fs, float* pSrc, float* pDstI, float* pDstQ, size_t len)
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

void GPS::lpfCoefDesignIPP(float rFreq, int tapsLen, float* pDst)
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

void GPS::doLpfIPP(float* pSrc, float* pDst, int len, float rFreq, int tapsLen)
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
void GPS::resampleIPP(int inRate, int outRate, Ipp32f* inData, int inLen, Ipp32f* outData, int& outLen)
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

//长度外部计算
void GPS::PNGen(float* f,float* regIni,int order,int way,float* m)
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
	else{
		switch (way){
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

void pnGenSSRG(std::vector<float>& ploy, std::vector<float>& reg, int order, int way, std::vector<float>& vDst)
{
	if (way == 0)
		bool isSSRG = true;

	int pnLen = pow(2, order) - 1;
	vDst.resize(pnLen);
	std::vector<float> ploy_use;
	ploy_use.resize(order);
	for (int i = 0; i < order; i++)
		ploy_use[i] = ploy[i + 1];
	for (int i = 0; i < pnLen; i++)
	{
		vDst[i] = reg[order-1];
		float temp = reg[order-1];
		for (int j = 0; j < order - 1; j++)
		{
			temp = temp + ploy_use[j] * reg[j];
			if (temp == 2) temp = 0;
		}
		for (int j = order - 1; j >= 1; j--)
			reg[j] = reg[j - 1];
		reg[0] = temp;
	}
}

//长度外部计算
void GPS::PNGenGPS(float* f, float* regIni, int order, int way, int index, float* m)
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
		if (way == 0)
		{
			for (int i = 0; i < m_len; i++)
			{
				switch (index)
				{
					case 1:
					{
						m[i] = reg[1] + reg[5];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 2:
					{
						m[i] = reg[2] + reg[6];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 3:
					{
						m[i] = reg[3] + reg[7];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 4:
					{
						m[i] = reg[4] + reg[8];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 5:
					{
						m[i] = reg[0] + reg[8];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 6:
					{
						m[i] = reg[1] + reg[9];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 7:
					{
						m[i] = reg[0] + reg[7];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 8:
					{
						m[i] = reg[1] + reg[8];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 9:
					{
						m[i] = reg[2] + reg[9];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 10:
					{
						m[i] = reg[1] + reg[2];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 11:
					{
						m[i] = reg[2] + reg[3];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 12:
					{
						m[i] = reg[4] + reg[5];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 13:
					{
						m[i] = reg[5] + reg[6];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 14:
					{
						m[i] = reg[6] + reg[7];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 15:
					{
						m[i] = reg[7] + reg[8];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 16:
					{
						m[i] = reg[8] + reg[9];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 17:
					{
						m[i] = reg[0] + reg[3];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 18:
					{
						m[i] = reg[1] + reg[4];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 19:
					{
						m[i] = reg[2] + reg[5];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 20:
					{
						m[i] = reg[3] + reg[6];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 21:
					{
						m[i] = reg[4] + reg[7];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 22:
					{
						m[i] = reg[5] + reg[8];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 23:
					{
						m[i] = reg[0] + reg[2];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 24:
					{
						m[i] = reg[3] + reg[5];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 25:
					{
						m[i] = reg[4] + reg[6];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 26:
					{
						m[i] = reg[5] + reg[7];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 27:
					{
						m[i] = reg[6] + reg[8];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 28:
					{
						m[i] = reg[7] + reg[9];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 29:
					{
						m[i] = reg[0] + reg[5];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 30:
					{
						m[i] = reg[1] + reg[6];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 31:
					{
						m[i] = reg[2] + reg[7];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 32:
					{
						m[i] = reg[3] + reg[8];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 33:
					{
						m[i] = reg[4] + reg[9];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 34:
					{
						m[i] = reg[3] + reg[9];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 35:
					{
						m[i] = reg[0] + reg[6];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 36:
					{
						m[i] = reg[1] + reg[7];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					case 37:
					{
						m[i] = reg[3] + reg[9];
						if (m[i] == 2)m[i] = 0;
						break;
					}
					default:
					{
						m[i] = reg[order - 1];
						break;
					}
				}
				float tmp = reg[order - 1];
				for (int j = order - 1; j >= 1; j--)
				{
					tmp = tmp + f1[j - 1] * reg[j - 1];
					if (tmp == 2) tmp = 0;
				}
				for (size_t j = order - 1; j >= 2; j--)
				{
					reg[j - 1] = reg[j - 2];
				}
				reg[0] = tmp;
			}
		}
		else
		{
			if (way == 1)
			{
				//用不到先不实现
			}
			else
			{
				//用不到先不实现
			}
		}
	}
}

void pnGenGPSSSRG10(std::vector<float>& ploy, std::vector<float>& reg, int way, int index, std::vector<float>& m)
{
	if (way == 0)
		bool isSSRG = true;
	int order = 10;

	int pnLen = pow(2, order) - 1;
	m.resize(pnLen);
	std::vector<float> ploy_use;
	ploy_use.resize(order);
	for (int i = 0; i < order; i++)
		ploy_use[i] = ploy[i + 1];
	for (int i = 0; i < pnLen; i++)
	{
		switch (index)
		{
		case 1:
		{
			m[i] = reg[1] + reg[5];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 2:
		{
			m[i] = reg[2] + reg[6];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 3:
		{
			m[i] = reg[3] + reg[7];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 4:
		{
			m[i] = reg[4] + reg[8];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 5:
		{
			m[i] = reg[0] + reg[8];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 6:
		{
			m[i] = reg[1] + reg[9];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 7:
		{
			m[i] = reg[0] + reg[7];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 8:
		{
			m[i] = reg[1] + reg[8];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 9:
		{
			m[i] = reg[2] + reg[9];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 10:
		{
			m[i] = reg[1] + reg[2];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 11:
		{
			m[i] = reg[2] + reg[3];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 12:
		{
			m[i] = reg[4] + reg[5];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 13:
		{
			m[i] = reg[5] + reg[6];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 14:
		{
			m[i] = reg[6] + reg[7];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 15:
		{
			m[i] = reg[7] + reg[8];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 16:
		{
			m[i] = reg[8] + reg[9];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 17:
		{
			m[i] = reg[0] + reg[3];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 18:
		{
			m[i] = reg[1] + reg[4];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 19:
		{
			m[i] = reg[2] + reg[5];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 20:
		{
			m[i] = reg[3] + reg[6];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 21:
		{
			m[i] = reg[4] + reg[7];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 22:
		{
			m[i] = reg[5] + reg[8];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 23:
		{
			m[i] = reg[0] + reg[2];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 24:
		{
			m[i] = reg[3] + reg[5];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 25:
		{
			m[i] = reg[4] + reg[6];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 26:
		{
			m[i] = reg[5] + reg[7];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 27:
		{
			m[i] = reg[6] + reg[8];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 28:
		{
			m[i] = reg[7] + reg[9];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 29:
		{
			m[i] = reg[0] + reg[5];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 30:
		{
			m[i] = reg[1] + reg[6];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 31:
		{
			m[i] = reg[2] + reg[7];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 32:
		{
			m[i] = reg[3] + reg[8];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 33:
		{
			m[i] = reg[4] + reg[9];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 34:
		{
			m[i] = reg[3] + reg[9];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 35:
		{
			m[i] = reg[0] + reg[6];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 36:
		{
			m[i] = reg[1] + reg[7];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		case 37:
		{
			m[i] = reg[3] + reg[9];
			if (m[i] == 2)m[i] = 0;
			break;
		}
		default:
		{
			m[i] = reg[order - 1];
			break;
		}
		}
		float temp = reg[order - 1];
		for (int j = 0; j < order - 1; j++)
		{
			temp = temp + ploy_use[j] * reg[j];
			if (temp == 2) temp = 0;
		}
		for (int j = order - 1; j >= 1; j--)
			reg[j] = reg[j - 1];
		reg[0] = temp;
	}
}

//分段相关，减小由载波估计误差导致的相关峰弱化的影响
void fraCorr(int fraLen/*小段长度*/, std::vector<float>& vSrc_uw/*独特码*/, std::vector<Ipp32fc>& vSrc/*待检测信号*/, 
	std::vector<float>& vDst/*相关结果*/)
{
	//独特码处理
	std::vector<float> vIm(vSrc_uw.size(), 0);
	std::vector<Ipp32fc> vUW_cplx; vUW_cplx.resize(vSrc_uw.size());
	ippsRealToCplx_32f(vSrc_uw.data(), vIm.data(), vUW_cplx.data(), vUW_cplx.size());
	//do
	int it = vUW_cplx.size() / fraLen;
	std::vector<Ipp32fc> huadong; huadong.resize(vUW_cplx.size());
	vDst.resize(vSrc.size() - 4 * vUW_cplx.size());//后续四倍抽取
	std::vector<Ipp32fc> datafrag; datafrag.resize(fraLen);
	std::vector<Ipp32fc> uwfrag; uwfrag.resize(fraLen);
	std::vector<Ipp32fc> vtmp; vtmp.resize(fraLen);
	for (int im = 0; im < vDst.size(); im++)
	{
		//抽取
		for (int i = 0; i < huadong.size(); i++)
			huadong[i] = vSrc[im + 4 * i];
		for (int i = 0; i < it; i++)
		{
			for (int s = 0; s < fraLen; s++)
				datafrag[s] = huadong[i * fraLen + s];
			for (size_t s = 0; s < fraLen; s++)
				uwfrag[s] = vUW_cplx[i * fraLen + s];
			Ipp32fc corrltmp;
			ippsMul_32fc(uwfrag.data(), datafrag.data(), vtmp.data(), fraLen);
			ippsSum_32fc(vtmp.data(), fraLen, &corrltmp, ippAlgHintNone);
			vDst[im] = pow(corrltmp.re, 2) + pow(corrltmp.im, 2);
		}
	}
}

int GPS::work(Ipp32f* data, int datalen, float sample_rate, float freq, float thread)
{
	//分解
	Ipp32f* vec_i = new Ipp32f[datalen]; Ipp32f* vec_q = new Ipp32f[datalen];
	r2iq(freq, sample_rate, data, vec_i, vec_q, datalen);
	//低通滤波器设计
	float Wn = 1e+6 / (sample_rate / 2);
	Wn = Wn / 2;//ipp库的滤波器输入参数和matlab不一样
	int N = 35;//滤波器抽头数
	//执行滤波
	float* lpfPass_i = ippsMalloc_32f(datalen);
	float* lpfPass_q = ippsMalloc_32f(datalen);
	doLpfIPP(vec_i, lpfPass_i, datalen, Wn, N);
	doLpfIPP(vec_q, lpfPass_q, datalen, Wn, N);
	delete[] vec_i; delete[] vec_q;
	//重采
	float Bd = 1.023e+6;
	int reLen = (float)datalen * Bd * 4 / sample_rate;
	float* re_i = ippsMalloc_32f(reLen); float* re_q = ippsMalloc_32f(reLen);
	resampleIPP(sample_rate, Bd * 4, lpfPass_i, datalen, re_i, reLen);
	resampleIPP(sample_rate, Bd * 4, lpfPass_q, datalen, re_q, reLen);
	ippsFree(lpfPass_i); ippsFree(lpfPass_q);
	//解析
	std::vector<Ipp32fc> cplx;
	cplx.resize(reLen);
	ippsRealToCplx_32f(re_i, re_q, cplx.data(), reLen);
	ippsFree(re_i); ippsFree(re_q);


	////伪码生成
	//参数输入
	std::vector<float> f1 = { 1,0,0,1,0,0,0,0,0,0,1 };//伪码1的本原多项式
	std::vector<float> f2 = { 1,0,1,1,0,0,1,0,1,1,1 };//伪码2的本原多项式
	std::vector<float> regIni1 = { 1,1,1,1,1,1,1,1,1,1 };//伪码1寄存器初始值
	std::vector<float> regIni2 = { 1,1,1,1,1,1,1,1,1,1 };//伪码2寄存器初始值
	int spOrder = 10;//寄存器阶数
	int way = 0;



	int m_len = pow(2, spOrder) - 1;
	std::vector<float> m;
	std::vector<float> m2;
	std::vector<float> gold;

	std::vector<Ipp32f> box;
	int kind = 0;
	std::vector<int> mark(38);
	for (size_t hi = 1; hi <= 38; hi++)
	{
		if (kind == 1)
			break;
		pnGenSSRG(f1,regIni1,spOrder,way,m);
		pnGenGPSSSRG10(f2, regIni2, 0, hi, m2);
		gold.resize(m.size() + 1);
		for (int i = 0; i < m.size(); i++)
		{
			if (m[i] == m2[i])
			{
				gold[i] = 0;
			}
			else
			{
				gold[i] = 1;
			}
			gold[i] = 2 * gold[i] - 1;
		}
		gold[gold.size() - 1] = gold[0];

		fraCorr(32, gold, cplx, box);

		int num, num1;
		
		
		for (int i = 0; i < box.size(); i++)
		{
			if (box[i] > thread)
			{
				num = i + 1;
				num1 = num + 1023 * 4;
				if (num1 < box.size())
				{
					if (box[num1] > thread)
					{
						kind = 1;
						mark[hi] = 1;
					}
				}
			}
		}


	}









	return kind;
}
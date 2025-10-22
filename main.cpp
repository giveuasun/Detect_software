#include <iostream>
#include <fstream>
#include <string>
#include <ipp.h>

#include "DVB.h"
#include "CDMA3GPP.h"
#include "GPS.h"

using namespace std;

int main()
{
	int sampleoffset = 0;//略过量

		//string filename = "D:\\Code\\mCode\\韩博\\matlab识别程序\\DVB标准\\DVB_IF.dat";
		//int max_read_count = 4000000;

		//string filename = "D:\\Code\\mCode\\韩博\\matlab识别程序\\CDMA3GPP\\CDMAIF3GPP.dat";
		//int max_read_count = 500000;

		string filename = "D:\\Code\\mCode\\韩博\\matlab识别程序\\GNSS\\GPSIFnew.dat";
		int max_read_count = 120000;




	//读取
	ifstream ifile(filename, ios::binary);//输入流对象，二进制方式打开
	if (!ifile){
		cerr << "无法打开文件" << filename << endl;
		return 1;
	}
	//信号总长度输出
	ifile.seekg(0, ios::end);//文件指针以0偏移量至末尾
	streamsize filesize = ifile.tellg();
	streamsize samplenum = filesize / sizeof(short);//信号总长度
	cout << "信号长度（点数）：" << samplenum << endl;
	//读取设定长度的数据
	streamsize offset = sampleoffset * sizeof(short);
	ifile.seekg(offset, ios::beg);//略过头部offset长度的数据
	Ipp16s* pData = ippsMalloc_16s(max_read_count);
	short value = 0;
	size_t count = 0;
	//while (count < max_read_count && ifile.read(reinterpret_cast<char*>(&value), sizeof(short)))
	while (count < max_read_count && ifile.read((char*)(&value), sizeof(short)))//read函数需要字节输入，后面一个参数是跳步长度
	{
		pData[count] = value;
		++count;
	}
	ifile.close();
	//精度转换
	Ipp32f* pData32f = ippsMalloc_32f(max_read_count);
	ippsConvert_16s32f(pData, pData32f, max_read_count);
	ippsFree(pData);



	//主程序流程
	DVB* dvb = new DVB;
	//int k1 = dvb->work(pData32f, max_read_count, 250e6, 62.5e6, 9.5e6, 94308);

	CDMA3GPP* cd = new CDMA3GPP;
	//int k2 = cd->work(pData32f, max_read_count, 5e6, 1.25e6, 8e+9);

	GPS* gp = new GPS;
	int k3 = gp->work(pData32f, max_read_count, 12e6, 3e6, 1.5e+08);

	//结束
	ippsFree(pData32f);
	return 0;
}


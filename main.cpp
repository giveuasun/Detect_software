#include <iostream>
#include <fstream>
#include <string>
#include <ipp.h>
#include "DVB.h"

using namespace std;

int main()
{
	//参数设计
	const string filename = "D:\\Code\\mCode\\韩博\\matlab识别程序\\DVB标准\\DVB_IF.dat";
	const size_t max_read_count = 4000000;
	const int sampleoffset = 0;//略过量
	//读取
	ifstream ifile(filename, ios::binary);//输入流对象，二进制方式打开
	if (!ifile)
	{
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
	//主程序流程
	DVB* dvb = new DVB;
	//dvb->work(pData32f, max_read_count, 250e6, 62.5e6, 9.5e6);

	Ipp32fc a[2] = {{ {3} ,{5} }, { {2} ,{2} }};
	Ipp32fc b[2] = { {{1},{-1}},{{3},{5}} };
	Ipp32fc c[2];

	ippsMul_32fc(&a[0], &b[0], &c[0], 2);


	//结束,释放堆
	ippsFree(pData);
	ippsFree(pData32f);
	return 0;
}


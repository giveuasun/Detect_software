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
	int sampleoffset = 0;//�Թ���

		//string filename = "D:\\Code\\mCode\\����\\matlabʶ�����\\DVB��׼\\DVB_IF.dat";
		//int max_read_count = 4000000;

		//string filename = "D:\\Code\\mCode\\����\\matlabʶ�����\\CDMA3GPP\\CDMAIF3GPP.dat";
		//int max_read_count = 500000;

		string filename = "D:\\Code\\mCode\\����\\matlabʶ�����\\GNSS\\GPSIFnew.dat";
		int max_read_count = 120000;




	//��ȡ
	ifstream ifile(filename, ios::binary);//���������󣬶����Ʒ�ʽ��
	if (!ifile){
		cerr << "�޷����ļ�" << filename << endl;
		return 1;
	}
	//�ź��ܳ������
	ifile.seekg(0, ios::end);//�ļ�ָ����0ƫ������ĩβ
	streamsize filesize = ifile.tellg();
	streamsize samplenum = filesize / sizeof(short);//�ź��ܳ���
	cout << "�źų��ȣ���������" << samplenum << endl;
	//��ȡ�趨���ȵ�����
	streamsize offset = sampleoffset * sizeof(short);
	ifile.seekg(offset, ios::beg);//�Թ�ͷ��offset���ȵ�����
	Ipp16s* pData = ippsMalloc_16s(max_read_count);
	short value = 0;
	size_t count = 0;
	//while (count < max_read_count && ifile.read(reinterpret_cast<char*>(&value), sizeof(short)))
	while (count < max_read_count && ifile.read((char*)(&value), sizeof(short)))//read������Ҫ�ֽ����룬����һ����������������
	{
		pData[count] = value;
		++count;
	}
	ifile.close();
	//����ת��
	Ipp32f* pData32f = ippsMalloc_32f(max_read_count);
	ippsConvert_16s32f(pData, pData32f, max_read_count);
	ippsFree(pData);



	//����������
	DVB* dvb = new DVB;
	//int k1 = dvb->work(pData32f, max_read_count, 250e6, 62.5e6, 9.5e6, 94308);

	CDMA3GPP* cd = new CDMA3GPP;
	//int k2 = cd->work(pData32f, max_read_count, 5e6, 1.25e6, 8e+9);

	GPS* gp = new GPS;
	int k3 = gp->work(pData32f, max_read_count, 12e6, 3e6, 1.5e+08);

	//����
	ippsFree(pData32f);
	return 0;
}


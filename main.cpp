#include <iostream>
#include <fstream>
#include <string>
#include <ipp.h>
#include "DVB.h"

using namespace std;

int main()
{
	//�������
	const string filename = "D:\\Code\\mCode\\����\\matlabʶ�����\\DVB��׼\\DVB_IF.dat";
	const size_t max_read_count = 4000000;
	const int sampleoffset = 0;//�Թ���
	//��ȡ
	ifstream ifile(filename, ios::binary);//���������󣬶����Ʒ�ʽ��
	if (!ifile)
	{
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
	//����������
	DVB* dvb = new DVB;
	//dvb->work(pData32f, max_read_count, 250e6, 62.5e6, 9.5e6);

	Ipp32fc a[2] = {{ {3} ,{5} }, { {2} ,{2} }};
	Ipp32fc b[2] = { {{1},{-1}},{{3},{5}} };
	Ipp32fc c[2];

	ippsMul_32fc(&a[0], &b[0], &c[0], 2);


	//����,�ͷŶ�
	ippsFree(pData);
	ippsFree(pData32f);
	return 0;
}


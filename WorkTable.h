#pragma once
#include<set>
#include<vector>
#include<bitset>

class WorkTable
{
	int m_ID = -1;
	int m_Type;
	std::pair<float, float> m_Pos;
	int m_CountDownFrame;
	std::bitset<7> m_MaterialState;
	int m_ProductState;
	std::set<std::pair<float, std::pair<int, int> > > m_DistanceOrder; //����̨֮��ľ��롾��������1��123���ͺ�123����֮��� 2���˴˲��ɴ�ġ�

public:
	/*-------��ѯ����------*/
	std::pair<float, float> GetPos(); //λ��
	int GetID(); //ID
	int GetType(); //����
	int GetCountDownFrame(); //����ʣ��֡���������������������-1��Ϊ��ͳһ����Ϊ15000������Ϸ����Ҳ���������ã��������ж�
	bool HaveProduct(); //�Ƿ��л�
	//��ǰ�����״̬�������������ͻ����������123�ͱ����ã�����һ��-1��vector��456789�ͷ���ȱ����vector����Ŵ�С���󣩣���ȱ����һ����vector
	std::vector<int> NeedWhat();
	std::set<std::pair<float, std::pair<int, int> > > DistanceGet(); //��ѯ����̨֮�����

	/*------IO���õ�------*/
	void Init(int id, std::pair<float, float> pos, int type); //��ʼ����������������λ�ú�id���������ٸ���
	bool UpdateInfo(int cdf, int ms, int ps);
	void DistanceSet(std::set<std::pair<float, std::pair<int, int> > > & ds);

	/*�����������*/
	int GetMaterialState(); //����������ʽ�Ĳ�Ʒ��״̬
	int CountDownFrameGet(); //��Ҫͨ��-1�ж��Ƿ�������
};
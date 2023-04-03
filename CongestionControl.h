#pragma once
#include <vector>
#include"IO.h"
class CongestionControl
{
	std::array<pair<float, float>, 4> m_LeaveCongestionPoint; //�뿪������������һ����
	IO* io;
	bool m_Flag[102][102];
	std::pair<int, int> m_pos;
public:
	CongestionControl(IO* io);
	CongestionControl();
	//�����Ƿ���Լ���ͨ��
	bool CanGo(std::pair<float, float> my_pos, int id);
	void Road_DFS(vector<std::pair<float, float>>& vt_road, vector<std::pair<float, float>>& path, std::pair<int, int> now_pos);
	float distance(std::pair<float, float>& pos1, std::pair<float, float>& pos2);

	//��������·�������Ƿ��ж�������
	bool Congestion(std::vector<std::pair<float, float> >& path1, int id1, std::vector<std::pair<float, float> >& path2, int id2);

	//�������·���������ǰλ��û���ҵ�����·�������ؿ�vector�����򷵻شӵ�ǰλ�ó����������õ�ĵ㼯
	std::vector<std::pair<float, float> > AvoidanceRoad(std::pair<float, float> my_pos, vector<std::pair<float, float>> & path);
};

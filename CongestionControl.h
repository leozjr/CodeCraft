#pragma once
#include <vector>
class CongestionControl
{
public:
	float distance(std::pair<float, float>& pos1, std::pair<float, float>& pos2);
	//��������·�������Ƿ��ж�������
	bool Congestion(std::vector<std::pair<float, float> > & path1, std::vector<std::pair<float, float> > & path2);

	//�������·���������ǰλ��û���ҵ�����·�������ؿ�vector�����򷵻شӵ�ǰλ�ó����������õ�ĵ㼯
	std::vector<std::pair<float, float> > AvoidanceRoad(std::pair<float, float> my_pos);
};

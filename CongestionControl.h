#pragma once
#include <vector>
#include"IO.h"
class CongestionControl
{

	
	bool m_Flag[102][102];
	std::pair<int, int> m_pos;
public:
	IO* io;
	vector<std::pair<float, float>> path1;
	vector<std::pair<float, float>> path2;


	std::array<pair<float, float>, 4> m_LeaveCongestionPoint; //�뿪������������һ����
	CongestionControl(IO* io);
	CongestionControl();
	//�����Ƿ���Լ���ͨ��
	bool CanGo(bool can_park, Robot& r_first_go);
	void Road_DFS(std::vector<int> iter_i, std::vector<int> iter_j,std::vector<std::pair<float, float>>& vt_road, std::vector<std::pair<float, float>>& path, std::pair<int, int> now_pos, std::pair<float, float> & my_pos, std::pair<float, float> & your_pos, vector<vector<std::pair<float, float>>>& others_road, std::vector<std::pair<float, float>> avoider_pos);
	float distance(std::pair<float, float>& pos1, std::pair<float, float>& pos2);

	//��������·�������Ƿ��ж�������
	int Congestion(std::vector<std::pair<float, float> >& path1, int id1, std::vector<std::pair<float, float> >& path2, int id2, int type);

	//�������·���������ǰλ��û���ҵ�����·�������ؿ�vector�����򷵻شӵ�ǰλ�ó����������õ�ĵ㼯
	std::vector<std::pair<float, float> > AvoidanceRoad(std::pair<float, float> my_pos, vector<std::pair<float, float>> & path, std::pair<float, float> my_pos2, std::pair<float, float> your_pos, std::vector<std::vector<std::pair<float, float>>>& others_road, std::vector<pair<float, float>> avoider_pos);
};

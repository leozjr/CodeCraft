#pragma once
#include <vector>
#include"IO.h"
class CongestionControl
{

	IO* io;
	bool m_Flag[102][102];
	std::pair<int, int> m_pos;
public:

	vector<std::pair<float, float>> path1;
	vector<std::pair<float, float>> path2;


	std::array<pair<float, float>, 4> m_LeaveCongestionPoint; //离开堵塞区域的最后一个点
	CongestionControl(IO* io);
	CongestionControl();
	//计算是否可以继续通过
	bool CanGo(std::pair<float, float> my_pos, int id);
	void Road_DFS(vector<std::pair<float, float>>& vt_road, vector<std::pair<float, float>>& path, std::pair<int, int> now_pos, std::pair<float, float> & my_pos, std::pair<float, float> & your_pos);
	float distance(std::pair<float, float>& pos1, std::pair<float, float>& pos2);

	//输入两条路，返回是否有堵塞风险
	bool Congestion(std::vector<std::pair<float, float> >& path1, int id1, std::vector<std::pair<float, float> >& path2, int id2, int type);

	//计算避让路径，如果当前位置没有找到避让路径，返回空vector，否则返回从当前位置出发，到避让点的点集
	std::vector<std::pair<float, float> > AvoidanceRoad(std::pair<float, float> my_pos, vector<std::pair<float, float>> & path, std::pair<float, float> my_pos2, std::pair<float, float> your_pos);
};

#pragma once
#include <vector>
class CongestionControl
{
public:
	float distance(std::pair<float, float>& pos1, std::pair<float, float>& pos2);
	//输入两条路，返回是否有堵塞风险
	bool Congestion(std::vector<std::pair<float, float> > & path1, std::vector<std::pair<float, float> > & path2);

	//计算避让路径，如果当前位置没有找到避让路径，返回空vector，否则返回从当前位置出发，到避让点的点集
	std::vector<std::pair<float, float> > AvoidanceRoad(std::pair<float, float> my_pos);
};

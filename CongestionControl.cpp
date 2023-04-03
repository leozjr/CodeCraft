#include "CongestionControl.h"
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;

float CongestionControl::distance(std::pair<float, float> & pos1, std::pair<float, float> & pos2) {
	return sqrt(pow(pos1.first - pos2.first, 2) + pow(pos1.second - pos2.second, 2));
}
bool CongestionControl::Congestion(std::vector<std::pair<float, float>> & path1, std::vector<std::pair<float, float>> & path2)
{
	int max_length = min(10,int(path1.size())); //最多搜索10个点
	max_length = min(max_length, int(path2.size()));
	float max_dis = 2.0; //点距阈值
	for (int i = max_length - 1; i >= 0; i--) {
		if (distance(path1[0], path2[i]) < max_dis && distance(path1[i], path2[0]) < max_dis){ //确定首尾是否重合
			int iter = i - 1;
			int iter2 = 1;
			while (iter >= iter2) {
				if (distance(path1[iter], path2[iter2]) < max_dis && distance(path1[iter2], path2[iter]) < max_dis) {
					iter--;
					iter2++;
				}
				else return false;
			}
			return true;
		}
	}
	return false;
}

std::vector<std::pair<float, float>> CongestionControl::AvoidanceRoad(std::pair<float, float> my_pos)
{
	std::vector<std::pair<float, float>> a;
	return a;
}

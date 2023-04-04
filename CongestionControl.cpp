#include "CongestionControl.h"
#include "IO.h"
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;

CongestionControl::CongestionControl(IO* io)
{
	this->io = io;
}

CongestionControl::CongestionControl()
{
}

void CongestionControl::Road_DFS(vector<std::pair<float, float>> & vt_road, vector<std::pair<float, float>> & path, pair<int, int> now_pos, pair<float, float> & my_pos, pair<float, float> & your_pos) {
	pair<float, float> p = make_pair(0.25 + 0.5 * now_pos.second, 49.75 - 0.5 * now_pos.first);
	int i = 0;
	this->m_Flag[now_pos.first][now_pos.second] = true;
	if (distance(my_pos, p) < distance(your_pos, p)) {
		for (i = 0; i < path.size(); i++) {
			if (distance(path[i], p) < 1.5) {
				break;
			}
		}
	}
	if (i == path.size()) { //找到可以躲避的点
		vt_road.push_back(p);
		return;
	}
	for (int i = -1; i < 2; i++) {
		for (int j = -1; j < 2; j++) {
			pair<int, int> pos = make_pair(now_pos.first + i, now_pos.second + j);
			if (abs(pos.first - this->m_pos.first) < 4 && abs(pos.second - this->m_pos.second) < 4 && pos.first > 0 && pos.first < 100 && pos.second > 0 && pos.second < 100 && !this->m_Flag[pos.first][pos.second] && this->io->CanGo_Map[pos.first][pos.second] != 1) {
				Road_DFS(vt_road, path, pos, my_pos, your_pos);
				if (vt_road.size() != 0) {
					vt_road.push_back(p);
					return;
				}
			}
		}
	}
	return;
}
float CongestionControl::distance(std::pair<float, float> & pos1, std::pair<float, float> & pos2) {
	return sqrt(pow(pos1.first - pos2.first, 2) + pow(pos1.second - pos2.second, 2));
}
bool CongestionControl::Congestion(std::vector<std::pair<float, float> >& path1, int id1, std::vector<std::pair<float, float> >& path2, int id2, int type)
{
	if (type == 1) {
		reverse(path1.begin(), path1.end());
		reverse(path2.begin(), path2.end());
		int max_length = min(10, int(path1.size())); //最多搜索10个点
		max_length = min(max_length, int(path2.size()));
		float max_dis = 1.0; //点距阈值
		for (int i = max_length - 1; i >= 4; i--) {
			if (distance(path1[0], path2[i]) < max_dis && distance(path1[i], path2[0]) < max_dis) { //确定首尾是否重合
				int iter = i - 1;
				int iter2 = 1;
				while (iter >= iter2) {
					if (distance(path1[iter], path2[iter2]) < max_dis && distance(path1[iter2], path2[iter]) < max_dis) {
						iter--;
						iter2++;
					}
					else
					{
						reverse(path1.begin(), path1.end());
						reverse(path2.begin(), path2.end());
						return false;
					}
				}

				this->m_LeaveCongestionPoint[id1] = path1[i];
				this->m_LeaveCongestionPoint[id2] = path2[i];
				reverse(path1.begin(), path1.end());
				reverse(path2.begin(), path2.end());
				return true;

			}
		}
		reverse(path1.begin(), path1.end());
		reverse(path2.begin(), path2.end());
		return false;
	}
	else if (type == 2) {
		int max_length = min(10, int(path1.size())); //最多搜索10个点
		int end1 = path1.size() - 1;
		int end2 = path2.size() - 1;
		max_length = min(max_length, int(path2.size()));
		float max_dis = 1.0; //点距阈值
		for (int i = max_length - 1; i >= 3; i--) {
			if (distance(path1[end1], path2[end2]) < max_dis && distance(path1[end1-i], path2[end2-i]) < max_dis) { //确定首尾是否重合
				int iter = i - 1;
				while (iter > 0) {
					if (distance(path1[end1 - iter], path2[end2 - iter]) < max_dis) {
						iter--;
					}
					else
					{
						return false;
					}
				}
				return true;
			}
		}
		return false;
	}
}

bool CongestionControl::CanGo(std::pair<float, float> my_pos, int id)
{
	return (this->distance(my_pos, this->m_LeaveCongestionPoint[id]) < 1);
}

std::vector<std::pair<float, float>> CongestionControl::AvoidanceRoad(std::pair<float, float> my_pos, vector<std::pair<float, float>> & path, std::pair<float, float> my_pos2, std::pair<float, float> your_pos)
{
	memset(this->m_Flag, false, sizeof(this->m_Flag));
	vector<pair<float, float>> vt_road;
	vt_road.clear();
	pair<int, int> now_pos = make_pair((49.75 - my_pos.second) * 2, (my_pos.first - 0.25) * 2);
	this->m_pos = now_pos;
	Road_DFS(vt_road, path, now_pos, my_pos2, your_pos);
	return vt_road;
}

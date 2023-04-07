#include "CongestionControl.h"
#include "IO.h"
#include <vector>
#include <cmath>
#include <cstring>
#include <algorithm>
using namespace std;

CongestionControl::CongestionControl(IO* io)
{
	this->io = io;
}

CongestionControl::CongestionControl()
{
}

//r现在在避让r_first_go，要确定什么时候可以走了
bool CongestionControl::CanGo(bool can_park, Robot& r_first_go)
{
	pair<float, float> pos = r_first_go.GetPos();
	int id = r_first_go.GetID();
	vector<pair<float, float>>& future_road = r_first_go.m_FutureRoad[0];

	float dis = this->distance(pos, this->m_LeaveCongestionPoint[id]);
	for (auto p_itx = future_road.rbegin(); p_itx != future_road.rend();p_itx++)
	{
		//如果future_road中有任何一个点都比我现在的位置离leave_point更近，那么就不能can_go
		if (this->distance(*p_itx, this->m_LeaveCongestionPoint[id]) < dis)
			return false;
	}
	return true;
}

void CongestionControl::Road_DFS(std::vector<int> iter_i, std::vector<int> iter_j,vector<std::pair<float, float>> & vt_road, vector<std::pair<float, float>> & path, pair<int, int> now_pos, pair<float, float> & my_pos, pair<float, float> & your_pos, vector<vector<std::pair<float, float>>>& others_road, vector<pair<float, float>> avoider_pos) {
	pair<float, float> p = make_pair(0.25 + 0.5 * now_pos.second, 49.75 - 0.5 * now_pos.first);
	bool flag_t = false;
	this->m_Flag[now_pos.first][now_pos.second] = true;
	if (distance(my_pos, p) < distance(your_pos, p)) {
		for (int i = 0; i < path.size(); i++) {
			if (distance(path[i], p) < 2) {
				flag_t = true;
				break;
			}
		}
		if (!flag_t) {
			for (int i = 0; i < others_road.size(); i++) {
				for (int j = 0; j < others_road[i].size(); j++) {
					if (distance(others_road[i][j], p) < 2) {
						flag_t = true;
						break;
					}
				}
				if (flag_t) break;
			}
		}
		if (!flag_t) { //找到可以躲避的点
			vt_road.push_back(p);
			return;
		}
	}
	for (int i : iter_i) {
		for (int j : iter_j) {
			pair<int, int> pos = make_pair(now_pos.first + i, now_pos.second + j);
			pair<float, float> pos_xy = make_pair(0.25 + 0.5 * pos.second, 49.75 - 0.5 * pos.first);
			if (distance(your_pos, pos_xy) > 2 && abs(pos.first - this->m_pos.first) < 8 && abs(pos.second - this->m_pos.second) < 8 && pos.first > 0 && pos.first < 100 && pos.second > 0 && pos.second < 100 && !this->m_Flag[pos.first][pos.second] && this->io->CanGo_Map[pos.first][pos.second] != 1) {
				bool flag_a = false;
				for (int k = 0; k < avoider_pos.size(); k++) {
					if (distance(my_pos, avoider_pos[k]) < 2) {
						flag_a = true;
						break;
					}
				}
				if (flag_a) continue;
				Road_DFS(iter_i,iter_j,vt_road, path, pos, my_pos, your_pos, others_road, avoider_pos);
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
int CongestionControl::Congestion(std::vector<std::pair<float, float> >& path1, int id1, std::vector<std::pair<float, float> >& path2, int id2, int type)
{
	if (type == 1) {
		reverse(path1.begin(), path1.end());
		reverse(path2.begin(), path2.end());
		int max_length = min(10, int(path1.size())); //最多搜索10个点
		max_length = min(max_length, int(path2.size()));
		float max_dis = 1.0; //点距阈值
		for (int i = max_length - 1; i >= 1; i--) {
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
		int iter = 0;
		int end1 = path1.size() - 1;
		int end2 = path2.size() - 1;
		float max_dis = 1.0; //点距阈值
		while (true) {
			if (iter <= end1 && iter <= end2 && distance(path1[end1 - iter], path2[end2 - iter]) < max_dis) {
				iter++;
			}
			else
			{
				break;
			}
		}
		if (iter < 5) {
			return 0;
		}
		else {
			return iter;
		}
	}
}

std::vector<std::pair<float, float>> CongestionControl::AvoidanceRoad(std::pair<float, float> my_pos, vector<std::pair<float, float>> & path, std::pair<float, float> my_pos2, std::pair<float, float> your_pos, vector<vector<std::pair<float, float>>>& others_road,std::vector<pair<float, float>> avoider_pos)
{
	vector<int> iterr = { -1,0,1 };
	vector<int> iterr_reverse = { 1,0,-1 };
	memset(this->m_Flag, false, sizeof(this->m_Flag));
	vector<pair<float, float>> vt_road;
	vector<pair<float, float>> vt_road_temp;
	vt_road.clear();
	vt_road_temp.clear();
	pair<int, int> now_pos = make_pair((49.75 - my_pos.second) * 2, (my_pos.first - 0.25) * 2);
	this->m_pos = now_pos;

	Road_DFS(iterr,iterr,vt_road_temp, path, now_pos, my_pos2, your_pos, others_road, avoider_pos);
	vt_road = vt_road_temp;

	if(vt_road.empty()) return vt_road;

	vt_road_temp.clear();
	memset(this->m_Flag, false, sizeof(this->m_Flag));
	Road_DFS(iterr, iterr_reverse, vt_road_temp, path, now_pos, my_pos2, your_pos, others_road, avoider_pos);
	if (!vt_road_temp.empty() && vt_road_temp.size() < vt_road.size()) {
		vt_road = vt_road_temp;
	}
	vt_road_temp.clear();
	memset(this->m_Flag, false, sizeof(this->m_Flag));
	Road_DFS(iterr_reverse, iterr, vt_road_temp, path, now_pos, my_pos2, your_pos, others_road, avoider_pos);
	if (!vt_road_temp.empty() && vt_road_temp.size() < vt_road.size()) {
		vt_road = vt_road_temp;
	}
	vt_road_temp.clear();
	memset(this->m_Flag, false, sizeof(this->m_Flag));
	Road_DFS(iterr_reverse, iterr_reverse, vt_road_temp, path, now_pos, my_pos2, your_pos, others_road, avoider_pos);
	if (!vt_road_temp.empty() && vt_road_temp.size() < vt_road.size()) {
		vt_road = vt_road_temp;
	}
	vt_road_temp.clear();
	return vt_road;
}

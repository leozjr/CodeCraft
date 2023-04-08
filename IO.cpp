#include <cmath>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <vector>
#include <set>
#include <queue>
#include <map>
#include "IO.h"
#include"Robot.h"
#include"WorkTable.h"
#include"RobotManager.h"
using namespace std;

float IO::distance(const pair<float, float> & pos1, const pair<float, float> & pos2) {
	return sqrt(pow(pos1.first - pos2.first, 2) + pow(pos1.second - pos2.second, 2));
}

void IO::TableIDByType(char type, int id, vector<vector<int> > & tableID_by_type)
{
	switch (type)
	{
	case '1':
		tableID_by_type[0].push_back(id);
		break;
	case '2':
		tableID_by_type[1].push_back(id);
		break;
	case '3':
		tableID_by_type[2].push_back(id);
		break;
	case '4':
		tableID_by_type[3].push_back(id);
		break;
	case '5':
		tableID_by_type[4].push_back(id);
		break;
	case '6':
		tableID_by_type[5].push_back(id);
		break;
	case '7':
		tableID_by_type[6].push_back(id);
		break;
	case '8':
		tableID_by_type[7].push_back(id);
		break;
	case '9':
		tableID_by_type[8].push_back(id);
		break;
	default:
		break;
	}
}

int IO::InListFlag(vector<A_Point *> & lt, pair<float, float> pos) {
	for (int i = 0; i < lt.size(); i++) {
		if (lt[i]->pos.first < pos.first + 0.01 && lt[i]->pos.first > pos.first - 0.01 && lt[i]->pos.second < pos.second + 0.01 && lt[i]->pos.second > pos.second - 0.01) {
			return i;
		}
	}
	return -1;
}

float IO::G_Calculate(A_Point * ap, pair<float, float> pos) {
	float g_add = this->m_gap;
	if (abs(ap->pos.first - pos.first) + abs(ap->pos.second - pos.second) > this->m_gap + 0.05) { //浮点数比较要有余量
		g_add = g_add * sqrt(2);
	}
	return ap->g + g_add;
}

float IO::H_Calculate(pair<float,float> end, pair<float, float> pos) {
	return abs(end.first - pos.first) + abs(end.second - pos.second);
}

void IO::CanGo(int Map[102][102]) //没有区分买卖前后
{
	for (int i = 0; i < 100; i++) {
		for (int j = 0; j < 100; j++) {
			int flag = 0;
			if (Map[i][j] == 2) { //有工作台在的地方
				continue;
			}
			if (i == 0 || i == 99 || j == 0 || j == 99) { //距墙0.25米
				this->CanGo_Map[i][j] = 1;
				continue;
			}
			for (int x = -1; x < 2; x++) {
				for (int y = -1; y < 2; y++) {
					if (Map[i + x][j + y] == 1) {
						flag += 1;
					}
				}
			}
			if (flag == 1) {
				if (j + 2 > 99 || i + 2 > 99 || j - 2 < 0 || i - 2 < 0) {
					this->CanGo_Map[i][j] = 1;
					continue;
				}
				if (Map[i - 1][j - 1] == 1 || Map[i + 1][j - 1] == 1) { //左边堵住
					if (Map[i - 1][j - 1] == 1 && Map[i - 1][j + 2] == 0 && Map[i][j + 2] == 0 && Map[i + 2][j - 1] == 0 && Map[i + 2][j] == 0) {
						if (Map[i + 1][j + 2] == 0 && Map[i + 2][j + 1] == 0) {
							this->CanGo_Map[i][j] = 1;
						}
						else this->CanGo_Map[i][j] = 2; //2代表半点
					}
					else if (Map[i + 1][j - 1] == 1 && Map[i + 1][j + 2] == 0 && Map[i][j + 2] == 0 && Map[i - 2][j - 1] == 0 && Map[i - 2][j] == 0) {
						if (Map[i - 1][j + 2] == 0 && Map[i - 2][j + 1] == 0) {
							this->CanGo_Map[i][j] = 1;
						}
						else this->CanGo_Map[i][j] = 2; //2代表半点
					}
					else {
						this->CanGo_Map[i][j] = 1;
					}
				}
				else if (Map[i - 1][j + 1] == 1 || Map[i + 1][j + 1] == 1) { //右边堵住
					if (Map[i - 1][j + 1] == 1 && Map[i - 1][j - 2] == 0 && Map[i][j - 2] == 0 && Map[i + 2][j + 1] == 0 && Map[i + 2][j] == 0) {
						if (Map[i + 1][j - 2] == 0 && Map[i + 2][j - 1] == 0) {
							this->CanGo_Map[i][j] = 1;
						}
						else this->CanGo_Map[i][j] = 2; //2代表半点
					}
					else if (Map[i + 1][j + 1] == 1 && Map[i + 1][j - 2] == 0 && Map[i][j - 2] == 0 && Map[i - 2][j + 1] == 0 && Map[i - 2][j] == 0) {
						if (Map[i - 1][j - 2] == 0 && Map[i - 2][j - 1] == 0) {
							this->CanGo_Map[i][j] = 1;
						}
						else this->CanGo_Map[i][j] = 2; //2代表半点
					}
					else {
						this->CanGo_Map[i][j] = 1;
					}
				}
				else {
					this->CanGo_Map[i][j] = 1;
				}
			}
			else if (flag > 1) {
				this->CanGo_Map[i][j] = 1;
			}
		}
	}
}

void IO::CanGo_NoGoods(int Map[102][102]) //买前
{
	vector<pair<int, int> > vt;
	for (int i = 0; i < 100; i++) {
		for (int j = 0; j < 100; j++) {
			int flag = 0;
			int flag_2 = 0;
			vt.clear();
			if (Map[i][j] == 2) { //有工作台在的地方
				continue;
			}
			if (Map[i][j] == 1) { //当前处于障碍物上
				this->CanGo_Map_NoGoods[i][j] = 1;
				continue;
			}
			for (int x = -1; x < 2; x++) {
				for (int y = -1; y < 2; y++) {
					if (i + x < 0 || i + x > 99 || j + y < 0 || j + y > 99) {
						flag_2++;
					}
					else if (Map[i + x][j + y] == 1) {
						flag++;
						vt.push_back(make_pair(x, y));
					}
				}
			}
			if ((flag_2 > 0 && flag > 0) || flag_2 > 1 || flag > 3) {
				this->CanGo_Map_NoGoods[i][j] = 1;
				continue;
			}
			if (flag_2 > 0 || flag < 2) {
				if (flag == 1) { //////
					if (Map[i + 1][j + 1] == 1 || Map[i + 1][j - 1] == 1 || Map[i - 1][j + 1] == 1 || Map[i - 1][j - 1] == 1) {
						this->CanGo_Map_NoGoods[i][j] = 3;
					}
				}
				continue;
			}
			else {
				if (flag == 2) {
					if ((vt[0].first == vt[1].first && vt[0].first != 0) || (vt[0].second == vt[1].second && vt[0].second != 0)) {
						continue;
					}
					else {
						this->CanGo_Map_NoGoods[i][j] = 1;
						continue;
					}
				}
				if (flag == 3) {
					if ((vt[0].first == vt[1].first && vt[1].first == vt[2].first) || (vt[0].second == vt[1].second && vt[1].second == vt[2].second)) {
						continue;
					}
					else {
						this->CanGo_Map_NoGoods[i][j] = 1;
						continue;
					}
				}
			}
		}
	}
	for (int i = 0; i < 100; i++) { //////
		for (int j = 0; j < 100; j++) {
			int flag = 0;
			if (this->CanGo_Map_NoGoods[i][j] == 3) { //////
				if (this->CanGo_Map_NoGoods[i + 1][j] == 3 || this->CanGo_Map_NoGoods[i - 1][j] == 3 || this->CanGo_Map_NoGoods[i][j + 1] == 3 || this->CanGo_Map_NoGoods[i][j - 1] == 3) {
					this->CanGo_Map_NoGoods[i][j] = 0;
				}
				else {
					this->CanGo_Map_NoGoods[i][j] = 1;
				}
			}
			if (this->CanGo_Map_NoGoods[i][j] == 0) {
				if (i == 0 || i == 99 || j == 0 || j == 99) {
					continue;
				}
				for (int x = -1; x < 2; x++) {
					for (int y = -1; y < 2; y++) {
						if (Map[i + x][j + y] != 1) {
							flag++;
						}
					}
				}
				if (flag == 9) {
					if (j + 2 > 99 || Map[i][j + 2] == 1)
					{
						this->CanGo_Map_NoGoods[i][j + 1] = 1;
					}
					if (j - 2 < 0 || Map[i][j - 2] == 1)
					{
						this->CanGo_Map_NoGoods[i][j - 1] = 1;
					}
					if (i + 2 > 99 || Map[i + 2][j] == 1)
					{
						this->CanGo_Map_NoGoods[i + 1][j] = 1;
					}
					if (i - 2 < 0 || Map[i - 2][j] == 1)
					{
						this->CanGo_Map_NoGoods[i - 1][j] = 1;
					}
				}
			}
		}
	}
}

int IO::SellorBuy(int type1, int type2) {
	if (type1 < 3) { //从123出发一定是带着东西
		return -1;
	}

	else if (type1 < 6) {
		if (type2 < 6) {
			return 1;
		}
		else {
			return -1;
		}
	}
	else if (type1 == 6) {
		if (type2 < 7) {
			return 1;
		}
		else {
			return -1;
		}
	}
	else {
		return 1;
	}
}

void IO::A_Star(Robot * robot, WorkTable * table, float n_max, float m_max, int Map[102][102], vector<pair<float, float> > & InitialRobotPos, vector<pair<float, float> > & InitialWorkTablePos, vector<vector<vector<pair<float, float> > > > & Road, map<pair<int,int>,float> & Distance)
{
	memset(this->CanGo_Map, 0, sizeof(this->CanGo_Map));
	memset(this->CanGo_Map_NoGoods, 0, sizeof(this->CanGo_Map_NoGoods));
	this->CanGo(Map);
	this->CanGo_NoGoods(Map);
	int CloseMap[102][102];
	int OpenMap[102][102];
	float gap = this->m_gap;
	pair<float, float> start_pos;
	pair<float, float> end_pos;
	for (int i_start = 0; i_start < 4 + InitialWorkTablePos.size(); i_start++) { //前四个为初始机器人位置
		int num_total = 0;
		if (i_start >= 4) {
			pair<float, float> target_now_pos = table[i_start - 4].GetPos();
			pair<int, int> pos_target = make_pair((49.75 - target_now_pos.second) * 2, (target_now_pos.first - 0.25) * 2);
			int itt = 0;
			if (Map[pos_target.first][pos_target.second + 1] == 1) itt++;
			if (Map[pos_target.first][pos_target.second - 1] == 1) itt++;
			if (Map[pos_target.first + 1][pos_target.second] == 1) itt++;
			if (Map[pos_target.first - 1][pos_target.second] == 1) itt++;
			if (itt > 2) {
				this->m_NoTable.insert(i_start - 4);
			}
			else if(itt == 2) {
				if ((Map[pos_target.first][pos_target.second + 1] == 1 && Map[pos_target.first][pos_target.second - 1] == 1) || (Map[pos_target.first + 1][pos_target.second] == 1 && Map[pos_target.first - 1][pos_target.second] == 1)) {
					this->m_NoTable.insert(i_start - 4);
				}
				else {
					if (table[i_start - 4].GetType() > 2) {
						if (Map[pos_target.first][pos_target.second + 1] == 1 && Map[pos_target.first + 1][pos_target.second] == 1) {
							if (Map[pos_target.first - 1][pos_target.second + 1] == 1 && Map[pos_target.first + 1][pos_target.second - 1] == 1) {
								this->m_NoTable.insert(i_start - 4);
							}
						}
						else if (Map[pos_target.first][pos_target.second - 1] == 1 && Map[pos_target.first + 1][pos_target.second] == 1) {
							if (Map[pos_target.first - 1][pos_target.second - 1] == 1 && Map[pos_target.first + 1][pos_target.second + 1] == 1) {
								this->m_NoTable.insert(i_start - 4);
							}
						}
						else if (Map[pos_target.first][pos_target.second - 1] == 1 && Map[pos_target.first - 1][pos_target.second] == 1) {
							if (Map[pos_target.first + 1][pos_target.second - 1] == 1 && Map[pos_target.first - 1][pos_target.second + 1] == 1) {
								this->m_NoTable.insert(i_start - 4);
							}
						}
						else if (Map[pos_target.first][pos_target.second + 1] == 1 && Map[pos_target.first - 1][pos_target.second] == 1) {
							if (Map[pos_target.first + 1][pos_target.second + 1] == 1 && Map[pos_target.first - 1][pos_target.second - 1] == 1) {
								this->m_NoTable.insert(i_start - 4);
							}
						}
					}
				}
			}
		}
		if (this->m_NoTable.count(i_start - 4)) continue;
		for (int i_end = 0; i_end < InitialWorkTablePos.size(); i_end++) {
			int Type_1 = 0;
			int Type_2 = 0;
			bool Flag;
			if (i_start < 4) {
				start_pos = InitialRobotPos[i_start];
				if (table[i_end].GetType() > 2) continue;
			}
			else {		
				start_pos = InitialWorkTablePos[i_start - 4];
				//if (i_start >= i_end + 4) continue;
				Type_1 = table[i_start - 4].GetType();
				Type_2 = table[i_end].GetType();
				if ((Type_1 < 3 && Type_2 < 3) || (i_start == i_end + 4)) {

					if (i_start == i_end + 4) { //自己到自己
						Road[i_start][i_end].push_back(table[i_end].GetPos());
					}
					continue;
				}
			}
			end_pos = InitialWorkTablePos[i_end];
			this->open.clear();
			this->close.clear();
			memset(CloseMap, 0, sizeof(CloseMap));
			memset(OpenMap, 0, sizeof(OpenMap));
			A_Point start(start_pos);
			A_Point * ap;
			this->open.push_back(&start);
			OpenMap[int(round((49.75 - start_pos.second) * 2))][int(round((start_pos.first - 0.25) * 2))] = 1;
			if (i_start < 4) {
				Flag = true; //买东西
			}
			else if (this->SellorBuy(Type_1, Type_2) > 0) {
				Flag = true;
			}
			else {
				Flag = false;
			}
			while (this->InListFlag(this->open, end_pos) < 0) {
				if (this->open.size() == 0) break;
				float temp = 100000000;
				int iter;
				for (int i = 0; i < this->open.size(); i++) {
					if (this->open[i]->f < temp) {
						temp = open[i]->f;
						iter = i;
					}
				}
				ap = this->open[iter];
				this->open.erase(open.begin() + iter);
				this->close.push_back(ap);
				CloseMap[int(round((49.75 - ap->pos.second) * 2))][int(round((ap->pos.first - 0.25) * 2))] = 1;
				for (int i = -1; i < 2; i++) {
					for (int j = -1; j < 2; j++) {
						float x = ap->pos.first + i * gap;
						float y = ap->pos.second + j * gap;
						if (x > 0.24 && x < n_max - 0.24 && y > 0.24 && y < m_max - 0.24) {
							pair<float, float> now_pos = make_pair(x, y);

							if (((!Flag && this->CanGo_Map[int(round((49.75 - y) * 2))][int(round((x - 0.25) * 2))] != 1) || (Flag && this->CanGo_Map_NoGoods[int(round((49.75 - y) * 2))][int(round((x - 0.25) * 2))] != 1)) && CloseMap[int(round((49.75 - y) * 2))][int(round((x - 0.25) * 2))] != 1) {
								float temp_g = this->G_Calculate(ap, now_pos);
								if (OpenMap[int(round((49.75 - y) * 2))][int(round((x - 0.25) * 2))] == 1) {
									int flag = this->InListFlag(this->open, now_pos);
									if (temp_g < this->open[flag]->g) {
										this->open[flag]->g = temp_g;
										this->open[flag]->father = ap;
										this->open[flag]->f = this->open[flag]->g + this->open[flag]->h;
									}
								}
								else {
									A_Point * temp_A_Point = new A_Point(now_pos);
									temp_A_Point->g = temp_g;
									temp_A_Point->h = this->H_Calculate(end_pos, now_pos);
									temp_A_Point->f = temp_g + temp_A_Point->h;
									temp_A_Point->father = ap;
									this->open.push_back(temp_A_Point);
									OpenMap[int(round((49.75 - y) * 2))][int(round((x - 0.25) * 2))] = 1;
								}
							}
						}
					}
				}
			}
			Road[i_start][i_end].clear();
			if (this->open.size() != 0) {
				int end_iter = this->InListFlag(this->open, end_pos);
				A_Point * temp_iter = this->open[end_iter];
				Distance[make_pair(i_start, i_end)] = temp_iter->g;
				while (temp_iter->pos != start.pos) {
					Road[i_start][i_end].push_back(temp_iter->pos);
					temp_iter = temp_iter->father;
				}
				Road[i_start][i_end].push_back(temp_iter->pos);
				reverse(Road[i_start][i_end].begin(), Road[i_start][i_end].end());

				if (Flag) { //买东西时用
					int it = 0;
					bool target_flag = false;
					int num = Road[i_start][i_end].size();
					pair<int, int> pre_target = make_pair((49.75 - Road[i_start][i_end][num - 2].second) * 2, (Road[i_start][i_end][num - 2].first - 0.25) * 2);
					pair<int, int> end_target = make_pair((49.75 - Road[i_start][i_end][num - 1].second) * 2, (Road[i_start][i_end][num - 1].first - 0.25) * 2);
					pair<int, int> goal_target;
					if (this->CanGo_Map[pre_target.first][pre_target.second] == 1) {
						for (int k = -1; k < 2; k++) {
							for (int q = -1; q < 2; q++) {
								if (!(k == 0 && q == 0) && end_target.first + k >= 0 && end_target.first + k < 100 && end_target.second + q >= 0 && end_target.second + q < 100 && this->CanGo_Map[end_target.first + k][end_target.second + q] != 1) {
									goal_target = make_pair(end_target.first + k, end_target.second + q);
									target_flag = true;
									break;
								}
							}
							if (target_flag) break;
						}
						if (target_flag) { //能去但是会卡住
							int gg = 0;
							Road[i_start][i_end].push_back(make_pair(0.25 + 0.5 * goal_target.second, 49.75 - 0.5 * goal_target.first));

							for (int k = -1; k < 2; k++) {
								for (int q = -1; q < 2; q++) {
									if (!(k == 0 && q == 0) && goal_target.first + k >= 0 && goal_target.first + k < 100 && goal_target.second + q >= 0 && goal_target.second + q < 100 && this->CanGo_Map[goal_target.first + k][goal_target.second + q] != 1) {
										Road[i_start][i_end].push_back(make_pair(0.25 + 0.5 * (goal_target.second + q), 49.75 - 0.5 * (goal_target.first + k)));
										gg = 1;
										break;
									}
								}
								if (gg == 1)break;
							}
							Road[i_start][i_end].push_back(make_pair(0.25 + 0.5 * end_target.second, 49.75 - 0.5 * end_target.first));
						}
						else { //该工作台不能去
							this->m_NoTable.insert(i_end);
						}
					}

				}

				num_total++;
			}

		}
		if (num_total == 0) {
			if (i_start < 4) {
				this->m_NoRobot.insert(i_start);
			}
			else {
				this->m_NoTable.insert(i_start - 4);
			}
		}
	}
}

void IO::A_Star_4D(Robot * robot, WorkTable * table, float n_max, float m_max, int Map[102][102], vector<pair<float, float> > & InitialRobotPos, vector<pair<float, float> > & InitialWorkTablePos, vector<vector<vector<pair<float, float> > > > & Road, map<pair<int, int>, float> & Distance)
{
	memset(this->CanGo_Map, 0, sizeof(this->CanGo_Map));
	this->CanGo(Map);
	int CloseMap[102][102];
	int OpenMap[102][102];
	float gap = this->m_gap;
	pair<float, float> start_pos;
	pair<float, float> end_pos;
	for (int i_start = 0; i_start < 4 + InitialWorkTablePos.size(); i_start++) { //前四个为初始机器人位置
		for (int i_end = 0; i_end < InitialWorkTablePos.size(); i_end++) {
			if (i_start < 4) {
				start_pos = InitialRobotPos[i_start];
				if (table[i_end].GetType() > 2) continue;
			}
			else {
				start_pos = InitialWorkTablePos[i_start - 4];
				if (i_start >= i_end + 4) continue;
				int Type_1 = table[i_start - 4].GetType();
				int Type_2 = table[i_end].GetType();
				if (Type_1 < 3 && Type_2 < 3) continue;
			}
			end_pos = InitialWorkTablePos[i_end];
			this->open.clear();
			this->close.clear();
			memset(CloseMap, 0, sizeof(CloseMap));
			memset(OpenMap, 0, sizeof(OpenMap));
			A_Point start(start_pos);
			A_Point * ap;
			this->open.push_back(&start);
			OpenMap[int(round((49.75 - start_pos.second) * 2))][int(round((start_pos.first - 0.25) * 2))] = 1;
			while (this->InListFlag(this->open, end_pos) < 0) {
				if (this->open.size() == 0) break;
				float temp = 100000000;
				int iter;
				for (int i = 0; i < this->open.size(); i++) {
					if (this->open[i]->f < temp) {
						temp = open[i]->f;
						iter = i;
					}
				}
				ap = this->open[iter];
				this->open.erase(open.begin() + iter);
				this->close.push_back(ap);
				CloseMap[int(round((49.75 - ap->pos.second) * 2))][int(round((ap->pos.first - 0.25) * 2))] = 1;
				for (int i = -1; i < 2; i++) {
					for (int j = -1; j < 2; j++) {
						float x = ap->pos.first + i * gap;
						float y = ap->pos.second + j * gap;
						if (x > 0.24 && x < n_max - 0.24 && y > 0.24 && y < m_max - 0.24 && (i == 0 || j == 0)) {
							pair<float, float> now_pos = make_pair(x, y);
							if (this->CanGo_Map[int(round((49.75 - y) * 2))][int(round((x - 0.25) * 2))] != 1 && CloseMap[int(round((49.75 - y) * 2))][int(round((x - 0.25) * 2))] != 1) {
								float temp_g = this->G_Calculate(ap, now_pos);
								if (OpenMap[int(round((49.75 - y) * 2))][int(round((x - 0.25) * 2))] == 1) {
									int flag = this->InListFlag(this->open, now_pos);
									if (temp_g < this->open[flag]->g) {
										this->open[flag]->g = temp_g;
										this->open[flag]->father = ap;
										this->open[flag]->f = this->open[flag]->g + this->open[flag]->h;
									}
								}
								else {
									A_Point * temp_A_Point = new A_Point(now_pos);
									temp_A_Point->g = temp_g;
									temp_A_Point->h = this->H_Calculate(end_pos, now_pos);
									temp_A_Point->f = temp_g + temp_A_Point->h;
									temp_A_Point->father = ap;
									this->open.push_back(temp_A_Point);
									OpenMap[int(round((49.75 - y) * 2))][int(round((x - 0.25) * 2))] = 1;
								}
							}
						}
					}
				}
			}
			Road[i_start][i_end].clear();
			if (this->open.size() != 0) {
				int end_iter = this->InListFlag(this->open, end_pos);
				A_Point * temp_iter = this->open[end_iter];
				Distance[make_pair(i_start, i_end)] = temp_iter->g;
				while (temp_iter->pos != start.pos) {
					Road[i_start][i_end].push_back(temp_iter->pos);
					temp_iter = temp_iter->father;
				}
				Road[i_start][i_end].push_back(temp_iter->pos);
				reverse(Road[i_start][i_end].begin(), Road[i_start][i_end].end());
			}

		}
	}
}

void IO::Initialization(RobotManager & rm, Robot * robot,WorkTable * table, vector<vector<int> > & tableID_by_type, int & WorkTableNum, vector<vector<vector<pair<float, float> > > > & Road, set<pair<float, pair<int, int> > > * DistanceOrder, set<pair<float, pair<int, int> > > * DistanceOrder_robot) {
	int mapID;
	int iter = 0;
	int iter_robot = 0;
	int iter_table = 0;
	char InitialMap[102][102];
	vector<pair<float, float> > InitialWorkTablePos;
	vector<pair<float, float> > InitialRobotPos;
	vector<int> WorkTableType;
	map<pair<int, int>, float> Distance;
	char line[105];
	while (true) {
		cin >> line;
		if (line[0] == 'O' && line[1] == 'K') {
			break;
		}
		for (int i = 0; i < 100; i++) {
			InitialMap[iter][i] = line[i];
			if (line[i] == 'A') {
				InitialRobotPos.push_back(make_pair(0.25 + 0.5*i, 49.75 - 0.5*iter));
				robot[iter_robot].Init(iter_robot); //初始化机器人位置
				iter_robot++;
			}
			else if (line[i] >= '1' && line[i] <= '9') {
				if (line[i] == '7') rm.SetSevenFlag(true); //有7号
				//rm.SetSevenFlag(false);
				InitialWorkTablePos.push_back(make_pair(0.25 + 0.5*i, 49.75 - 0.5*iter));
				WorkTableType.push_back(line[i] - '1');
				this->TableIDByType(line[i], iter_table, tableID_by_type);
				table[iter_table].Init(iter_table, make_pair(0.25 + 0.5*i, 49.75 - 0.5*iter), line[i] - '1'); //初始化工作台位置和类型
				iter_table++;
			}
		}
		iter++;
	}
	//求最短路径
	memset(Map, 0, sizeof(Map));
	for (int i = 0; i < 100; i++) {
		for (int j = 0; j < 100; j++) {
			if (InitialMap[i][j] == '#') Map[i][j] = 1;
			if (InitialMap[i][j] >= '1' && InitialMap[i][j] <= '9') Map[i][j] = 2;
		}
	}
	this->A_Star(robot, table, 50, 50, Map, InitialRobotPos, InitialWorkTablePos, Road, Distance);
	//机器人间距离计算
	
	for (int i = 0; i < 4 + InitialWorkTablePos.size(); i++)
	{
		for (int j = 0; j < InitialWorkTablePos.size(); j++)
		{
			if (Distance.count(make_pair(i, j)) && this->m_NoTable.count(j) == 0) {
				if (i < 4) { //机器人和工作台的距离
					DistanceOrder_robot[i].insert(make_pair(Distance[make_pair(i, j)], make_pair(j, WorkTableType[j])));
				}
				else if (i - 4 != j) {
					DistanceOrder[i - 4].insert(make_pair(Distance[make_pair(i, j)], make_pair(j, WorkTableType[j])));
				}
			}
		}
	}

	for (int i = 0; i < InitialWorkTablePos.size(); i++) { //需要存储距离
		table[i].DistanceSet(DistanceOrder[i]);               
	}

	for (int i = 0; i < 4; i++) {
		robot[i].DistanceSet(DistanceOrder_robot[i]);
	}

	WorkTableNum = InitialWorkTablePos.size();
	switch (WorkTableNum)
	{
	case 37:
		mapID = 1;
		break;
	case 8:
		mapID = 2;
		break;
	case 13:
		mapID = 3;
		break;
	case 26:
		mapID = 4;
		break;
	default:
		break;
	}
	rm.InitialTypeTable(WorkTableType); //初始化工作类型表
	rm.SingleManager(robot, table, Road, 0); //初始化决策
	cout << "OK";
	cout << flush;
}

int IO::readUntilOK(int & frameID, int & money, int & WorkTableNum, Robot * robot, WorkTable * table, vector<pair<float, int> > * DistanceOrder_between_robot) {
	cin >> frameID >> money;
	cin.get();
	cin >> WorkTableNum;
	cin.get();
	int Type;
	int mapID;
	pair<float, float> TablePos;
	int RemainingProductionTime;
	int MaterialStatus;
	int ProductionStatus;
	int WhichWorkTable;
	int ProductionType;
	float TimeIndex;
	float CrashIndex;
	float RotSpeed;
	pair<float, float> LineSpeed;
	float Direction;
	pair<float, float> RobotPos;
	char OK[5];
	switch (WorkTableNum)
	{
	case 37:
		mapID = 1;
		break;
	case 8:
		mapID = 2;
		break;
	case 13:
		mapID = 3;
		break;
	case 26:
		mapID = 4;
		break;
	default:
		break;
	}
	for (int i = 0; i < WorkTableNum; i++) {
		cin >> Type >> TablePos.first >> TablePos.second >> RemainingProductionTime >> MaterialStatus >> ProductionStatus;
		cin.get();
		table[i].UpdateInfo(RemainingProductionTime, MaterialStatus, ProductionStatus);
	}
	for (int i = 0; i < 4; i++) {
		cin >> WhichWorkTable >> ProductionType >> TimeIndex >> CrashIndex >> RotSpeed >> LineSpeed.first >> LineSpeed.second >> Direction >> RobotPos.first >> RobotPos.second;
		cin.get();
		robot[i].UpdateInfo(WhichWorkTable, ProductionType, TimeIndex, CrashIndex, RotSpeed, LineSpeed, Direction, RobotPos);
		robot[i].BuySellCheck(2, table);
	}

	// 机器人距离更新

	for (int i = 0; i < 4; i++)
	{
		DistanceOrder_between_robot[i].clear();
	}

	for (int i = 0; i < 4; i++)
	{
		for (int j = i + 1; j < 4; j++)
		{
			float dis = this->distance(robot[i].GetPos(), robot[j].GetPos());
			DistanceOrder_between_robot[i].push_back(make_pair(dis, j));
			DistanceOrder_between_robot[j].push_back(make_pair(dis, i));
		}
	}

	for (int i = 0; i < 4; i++)
	{
		robot[i].DistanceSet(DistanceOrder_between_robot[i]);
	}

	cin >> OK;
	cin.get();
	return mapID;
}

void IO::writeUntilOK(int & frameID, vector<pair<string, pair<int, float> > > & Order_1, vector<pair<string, int> > & Order_2) {
	cout << frameID << endl;
	while (!Order_1.empty()) {
		cout << Order_1.back().first << " " << Order_1.back().second.first << " " << Order_1.back().second.second << endl;
		Order_1.pop_back();
	}
	while (!Order_2.empty()) {
		cout << Order_2.back().first << " " << Order_2.back().second << endl;
		Order_2.pop_back();
	}
	cout << "OK";
	cout << fflush;
}
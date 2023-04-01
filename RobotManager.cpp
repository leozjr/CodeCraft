#include "RobotManager.h"
#include"Robot.h"
#include"WorkTable.h"
#include<set>
#include<vector>
#include<cmath>
#include<algorithm>

using namespace std;

void RobotManager::GiveTask(Robot * robots, int * goalTable, vector<vector<vector<pair<float, float> > > > & Road){
	for (int i = 0; i < 4; i++) {
		if (robots[i].isBusy()) continue;
		int id = robots[i].GetTask()[0];
		if (robots[i].GetBuyorSell() == -1) {
			this->m_TableFlagBuy[id] = false;
		}
		else if (robots[i].GetBuyorSell() == 1)
		{
			this->m_TableFlagSell[id][robots[i].GetGoods() - 1] = false;
			this->m_SellTable[id] -= int(pow(2, robots[i].GetGoods()));
		}
		if (goalTable[i] != -1) {
			vector<int> vt_task = { goalTable[i] };
			robots[i].SetTask(vt_task);
			if (goalTable[i] < id) {
				vector<vector<pair<float, float> > > st_road = { Road[goalTable[i] + 4][id] };
				robots[i].SetRoad(st_road);
			}
			else if (goalTable[i] > id) {
				vector<vector<pair<float, float> > > st_road = { Road[id + 4][goalTable[i]] };
				robots[i].SetRoad(st_road);
			}
		}
		else {
			robots[i].ClearTask();
		}
	}
}
void RobotManager::SingleManager(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road,int frameID)
{
	float t1;
	float t2;
	float speed = 6.0; //匀速指标
	int goalTable[5] = { -1,-1,-1,-1 };
	set<std::pair<float, std::pair<int, int> > >::iterator it;
	if (frameID == 0) { //初始化
		for (int i = 0; i < 4; i++) {
			set<std::pair<float, std::pair<int, int> > > st = robots[i].DistanceGet();
			for (it = st.begin(); it != st.end(); it++) {
				if ((*it).second.second >= 0 && (*it).second.second <= 2) {
					int temp = (*it).second.first;
					if (!this->m_TableFlagBuy[temp]) {
						goalTable[i] = temp;
						this->m_TableFlagBuy[temp] = true;
						break;
					}
				}
			}
		}
	}
	else {
		SetChooseTable(tables); //买东西只能从中选择
		for (int i = 0; i < 4; i++) {
			if (robots[i].isBusy()) continue;
			int id = robots[i].GetTask()[0];
			set<std::pair<float, std::pair<int, int> > > st = tables[id].DistanceGet();
			st.insert(make_pair(0, make_pair(id, tables[id].GetType())));
			if (robots[i].GetBuyorSell() == -1) { //刚买货
				switch (tables[id].GetType()) {
				case 0: //类型从0开始
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 3 || ty == 4) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][0]) && (count(need.begin(),need.end(),0) > 0)) { //没有被预定且有产品格
								goalTable[i] = temp;
								this->m_TableFlagSell[temp][0] = true;
								this->m_SellTable[temp] += 2;
								break;
							}
						}
					}
					break;
				case 1:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 3 || ty == 5) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][1]) && (count(need.begin(), need.end(), 1) > 0)) { //没有被预定且有产品格
								goalTable[i] = temp;
								this->m_TableFlagSell[temp][1] = true;
								this->m_SellTable[temp] += 4;
								break;
							}
						}
					}
					break;
				case 2:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 4 || ty == 5) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][2]) && (count(need.begin(), need.end(), 2) > 0)) { //没有被预定且有产品格
								goalTable[i] = temp;
								this->m_TableFlagSell[temp][2] = true;
								this->m_SellTable[temp] += 8;
								break;
							}
						}
					}
					break;
				case 3:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][3]) && (count(need.begin(), need.end(), 3) > 0)) { //没有被预定且有产品格
								goalTable[i] = temp;
								this->m_TableFlagSell[temp][3] = true;
								this->m_SellTable[temp] += 16;
								break;
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					break;
				case 4:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][4]) && (count(need.begin(), need.end(), 4) > 0)) { //没有被预定且有产品格
								goalTable[i] = temp;
								this->m_TableFlagSell[temp][4] = true;
								this->m_SellTable[temp] += 32;
								break;
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					break;
				case 5:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][5]) && (count(need.begin(), need.end(), 5) > 0)) { //没有被预定且有产品格
								goalTable[i] = temp;
								this->m_TableFlagSell[temp][5] = true;
								this->m_SellTable[temp] += 64;
								break;
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					break;
				case 6:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 7 || ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp; //不设锁
							break;
						}
					}
					break;
				default:
					break;
				}
			}
			else if (robots[i].GetBuyorSell() == 1) { //刚卖货
				int flag = 0;
				for (int j = 1; j <= 3; j++) { //搜索三次，分别为7，456，123
					if (flag == 1) break;
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (j == 1 && ty == 6) {
							int temp = (*it).second.first;
							t1 = tables[temp].CountDownFrameGet() / 50.0;
							t2 = (*it).first / speed;
							if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2))) { //没有被预定且有产品可以拿
								goalTable[i] = temp;
								this->m_TableFlagBuy[temp] = true;
								flag = 1;
								break;
							}
						}
						else if (j == 2 && (ty >= 3 && ty <= 5)) {
							if (this->m_ChooseTable.count(ty) > 0) {
								int temp = (*it).second.first;
								t1 = tables[temp].CountDownFrameGet() / 50.0;
								t2 = (*it).first / speed;
								if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2))) {
									goalTable[i] = temp;
									this->m_TableFlagBuy[temp] = true;
									int num = this->m_ChooseTable.count(ty);
									this->m_ChooseTable.erase(ty);
									for (int cc = 1; cc < num; cc++) {
										this->m_ChooseTable.insert(ty);
									}
									flag = 1;
									break;
								}
							}
						}
						else if (j == 3 && (ty >= 0 && ty <= 2)) {
							if (this->m_ChooseTable.count(ty) > 0) {
								int temp = (*it).second.first;
								t1 = tables[temp].CountDownFrameGet() / 50.0;
								t2 = (*it).first / speed;
								if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2))) {
									goalTable[i] = temp;
									this->m_TableFlagBuy[temp] = true;
									int num = this->m_ChooseTable.count(ty);
									this->m_ChooseTable.erase(ty);
									for (int cc = 1; cc < num; cc++) {
										this->m_ChooseTable.insert(ty);
									}
									flag = 1;
									break;
								}
							}
						}
					}
				}
			}
		}
	}
	this->GiveTask(robots, goalTable, Road);//如果实在是没地方去会怎么样？？？？？
}

void RobotManager::SingleManager_2(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road,int frameID) //优先级对调，不专门去拿4567
{
	float t1;
	float t2;
	float speed = 6.0; //匀速指标
	int goalTable[5] = { -1,-1,-1,-1 };
	set<std::pair<float, std::pair<int, int> > >::iterator it;
	if (frameID == 0) { //初始化
		for (int i = 0; i < 4; i++) {
			set<std::pair<float, std::pair<int, int> > > st = robots[i].DistanceGet();
			for (it = st.begin(); it != st.end(); it++) {
				if ((*it).second.second >= 0 && (*it).second.second <= 2) {
					int temp = (*it).second.first;
					if (!this->m_TableFlagBuy[temp]) {
						goalTable[i] = temp;
						this->m_TableFlagBuy[temp] = true;
						break;
					}
				}
			}
		}
	}
	else {
		SetChooseTable(tables); //买东西只能从中选择
		for (int i = 0; i < 4; i++) {
			if (robots[i].isBusy()) continue;
			int id = robots[i].GetTask()[0];
			int flag = 0;
			//this->m_TableFlag[id] = false; //买卖之后就不占用了
			set<std::pair<float, std::pair<int, int> > > st = tables[id].DistanceGet();
			st.insert(make_pair(0, make_pair(id, tables[id].GetType())));
			if (robots[i].GetBuyorSell() == -1) { //刚买货
				switch (tables[id].GetType()) {
				case 0: //类型从0开始
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 3 || ty == 4) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][0]) && (count(need.begin(), need.end(), 0) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (this->m_ChooseTable.count(ty)) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][0] = true;
						this->m_SellTable[goalTable[i]] += 2;
					}
					break;
				case 1:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 3 || ty == 5) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][1]) && (count(need.begin(), need.end(), 1) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (this->m_ChooseTable.count(ty)) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][1] = true;
						this->m_SellTable[goalTable[i]] += 4;
					}
					break;
				case 2:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 4 || ty == 5) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][2]) && (count(need.begin(), need.end(), 2) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (this->m_ChooseTable.count(ty)) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][2] = true;
						this->m_SellTable[goalTable[i]] += 8;
					}
					break;
				case 3:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][3]) && (count(need.begin(), need.end(), 3) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 16 == 112) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][3] = true;
						this->m_SellTable[goalTable[i]] += 16;
					}
					break;
				case 4:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][4]) && (count(need.begin(), need.end(), 4) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 32 == 112) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][4] = true;
						this->m_SellTable[goalTable[i]] += 32;
					}
					break;
				case 5:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][5]) && (count(need.begin(), need.end(), 5) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 64 == 112) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][5] = true;
						this->m_SellTable[goalTable[i]] += 64;
					}
					break;
				case 6:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 7 || ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp; //不设锁
							break;
						}
					}
					break;
				default:
					break;
				}
			}
			else if (robots[i].GetBuyorSell() == 1) { //刚卖货
				int GTID = robots[i].GetTask()[0];
				int GT = tables[GTID].GetType();
				if (tables[GTID].HaveProduct() && (this->m_ChooseTable.count(GT) > 0 || GT == 6)) {
					//Order_2.push_back(make_pair("buy", GTID));
					goalTable[i] = GTID;
					this->m_TableFlagBuy[GTID] = true;
					int num = this->m_ChooseTable.count(GT);
					this->m_ChooseTable.erase(GT);
					for (int cc = 1; cc < num; cc++) {
						this->m_ChooseTable.insert(GT);
					}
				}
				else {
					int tyy;
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty >= 0 && ty <= 2) {
							if (this->m_ChooseTable.count(ty) > 0) {
								int temp = (*it).second.first;
								if ((!this->m_TableFlagBuy[temp]) && tables[temp].HaveProduct()) {
									if (flag == 0) {
										goalTable[i] = temp;
										flag = 1;
										tyy = ty;
									}
									switch (ty) {
									case 0:
										if (this->m_ChooseTable.count(3) || this->m_ChooseTable.count(4)) {
											goalTable[i] = temp;
											tyy = ty;
											flag = 2;
										}
										break;
									case 1:
										if (this->m_ChooseTable.count(3) || this->m_ChooseTable.count(5)) {
											goalTable[i] = temp;
											tyy = ty;
											flag = 2;
										}
										break;
									case 2:
										if (this->m_ChooseTable.count(4) || this->m_ChooseTable.count(5)) {
											goalTable[i] = temp;
											tyy = ty;
											flag = 2;
										}
										break;
									default:
										break;
									}
									if (flag == 2) break;
								}
							}
						}
					}
					if (flag >= 1) {
						this->m_TableFlagBuy[goalTable[i]] = true;
						int num = this->m_ChooseTable.count(tyy);
						this->m_ChooseTable.erase(tyy);
						for (int cc = 1; cc < num; cc++) {
							this->m_ChooseTable.insert(tyy);
						}
					}
				}
			}
		}
	}
	this->GiveTask(robots, goalTable, Road);//如果实在是没地方去会怎么样？？？？？
}

void RobotManager::SingleManager_3(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road,int frameID) //设置了全图456的拿取
{
	float t1;
	float t2;
	float speed = 6.0; //匀速指标
	int goalTable[5] = { -1,-1,-1,-1 };
	set<std::pair<float, std::pair<int, int> > >::iterator it;
	if (frameID == 0) { //初始化
		for (int i = 0; i < 4; i++) {
			set<std::pair<float, std::pair<int, int> > > st = robots[i].DistanceGet();
			for (it = st.begin(); it != st.end(); it++) {
				if ((*it).second.second >= 0 && (*it).second.second <= 2) {
					int temp = (*it).second.first;
					if (!this->m_TableFlagBuy[temp]) {
						goalTable[i] = temp;
						this->m_TableFlagBuy[temp] = true;
						break;
					}
				}
			}
		}
	}
	else {
		SetChooseTable(tables); //买东西只能从中选择
		SetTargetTable(tables, robots);
		for (int i = 0; i < 4; i++) {
			if (robots[i].isBusy()) continue;
			int id = robots[i].GetTask()[0];
			int flag = 0;
			//this->m_TableFlag[id] = false; //买卖之后就不占用了
			set<std::pair<float, std::pair<int, int> > > st = tables[id].DistanceGet();
			st.insert(make_pair(0, make_pair(id, tables[id].GetType())));
			if (robots[i].GetBuyorSell() == -1) { //刚买货
				switch (tables[id].GetType()) {
				case 0: //类型从0开始
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 3 || ty == 4) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][0]) && (count(need.begin(), need.end(), 0) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (this->m_TargetTable[ty] > 0) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][0] = true;
						this->m_SellTable[goalTable[i]] += 2;
					}
					break;
				case 1:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 3 || ty == 5) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][1]) && (count(need.begin(), need.end(), 1) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (this->m_TargetTable[ty] > 0) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][1] = true;
						this->m_SellTable[goalTable[i]] += 4;
					}
					break;
				case 2:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 4 || ty == 5) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][2]) && (count(need.begin(), need.end(), 2) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (this->m_TargetTable[ty] > 0) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][2] = true;
						this->m_SellTable[goalTable[i]] += 8;
					}
					break;
				case 3:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][3]) && (count(need.begin(), need.end(), 3) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 16 == 112) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][3] = true;
						this->m_SellTable[goalTable[i]] += 16;
					}
					break;
				case 4:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][4]) && (count(need.begin(), need.end(), 4) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 32 == 112) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][4] = true;
						this->m_SellTable[goalTable[i]] += 32;
					}
					break;
				case 5:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][5]) && (count(need.begin(), need.end(), 5) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 64 == 112) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][5] = true;
						this->m_SellTable[goalTable[i]] += 64;
					}
					break;
				case 6:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 7 || ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp; //不设锁
							break;
						}
					}
					break;
				default:
					break;
				}
			}
			else if (robots[i].GetBuyorSell() == 1) { //刚卖货
				int flag = 0;
				int tyy;
				for (int j = 1; j <= 3; j++) { //搜索三次，分别为7，456，123
					if (flag == 1) break;
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (j == 1 && ty == 6) {
							int temp = (*it).second.first;
							t1 = tables[temp].CountDownFrameGet() / 50.0;
							t2 = (*it).first / speed;
							if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2))) { //没有被预定且有产品可以拿
								goalTable[i] = temp;
								this->m_TableFlagBuy[temp] = true;
								flag = 1;
								break;
							}
						}
						else if (j == 2 && (ty >= 3 && ty <= 5)) {
							if (this->m_ChooseTable.count(ty) > 0) {
								int temp = (*it).second.first;
								t1 = tables[temp].CountDownFrameGet() / 50.0;
								t2 = (*it).first / speed;
								if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2))) {
									goalTable[i] = temp;
									this->m_TableFlagBuy[temp] = true;
									int num = this->m_ChooseTable.count(ty);
									this->m_ChooseTable.erase(ty);
									for (int cc = 1; cc < num; cc++) {
										this->m_ChooseTable.insert(ty);
									}
									flag = 1;
									break;
								}
							}
						}
						else if (j == 3 && (ty >= 0 && ty <= 2)) {
							if (this->m_ChooseTable.count(ty) > 0) {
								int temp = (*it).second.first;
								t1 = tables[temp].CountDownFrameGet() / 50.0;
								t2 = (*it).first / speed;
								if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2))) {
									if (flag == 0) {
										goalTable[i] = temp;
										flag = 3;
										tyy = ty;
									}
									switch (ty) {
									case 0:
										if (this->m_TargetTable[3] > 0 || this->m_TargetTable[4] > 0) {
											goalTable[i] = temp;
											tyy = ty;
											flag = 2;
										}
										break;
									case 1:
										if (this->m_TargetTable[3] > 0 || this->m_TargetTable[5] > 0) {
											goalTable[i] = temp;
											tyy = ty;
											flag = 2;
										}
										break;
									case 2:
										if (this->m_TargetTable[4] > 0 || this->m_TargetTable[5] > 0) {
											goalTable[i] = temp;
											tyy = ty;
											flag = 2;
										}
										break;
									default:
										break;
									}
									if (flag == 2) {
										this->m_TableFlagBuy[goalTable[i]] = true;
										int num = this->m_ChooseTable.count(tyy);
										this->m_ChooseTable.erase(tyy);
										for (int cc = 1; cc < num; cc++) {
											this->m_ChooseTable.insert(tyy);
										}
										flag = 1;
										break;
									}
								}
							}
						}
					}
				}
				if (flag == 3) {
					this->m_TableFlagBuy[goalTable[i]] = true;
					int num = this->m_ChooseTable.count(tyy);
					this->m_ChooseTable.erase(tyy);
					for (int cc = 1; cc < num; cc++) {
						this->m_ChooseTable.insert(tyy);
					}
				}
			}
		}
	}
	this->GiveTask(robots, goalTable, Road);//如果实在是没地方去会怎么样？？？？？
}

void RobotManager::SingleManager_4(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road,int frameID) //加入顺道买东西机制
{
	float t1;
	float t2;
	float speed = 6.0; //匀速指标
	int goalTable[5] = { -1,-1,-1,-1 };
	set<std::pair<float, std::pair<int, int> > >::iterator it;
	if (frameID == 0) { //初始化
		for (int i = 0; i < 4; i++) {
			set<std::pair<float, std::pair<int, int> > > st = robots[i].DistanceGet();
			for (it = st.begin(); it != st.end(); it++) {
				if ((*it).second.second >= 0 && (*it).second.second <= 2) {
					int temp = (*it).second.first;
					if (!this->m_TableFlagBuy[temp]) {
						goalTable[i] = temp;
						this->m_TableFlagBuy[temp] = true;
						break;
					}
				}
			}
		}
	}
	else {
		SetChooseTable(tables); //买东西只能从中选择
		for (int i = 0; i < 4; i++) {
			if (robots[i].isBusy()) continue;
			int id = robots[i].GetTask()[0];
			int flag = 0;
			//this->m_TableFlag[id] = false; //买卖之后就不占用了
			set<std::pair<float, std::pair<int, int> > > st = tables[id].DistanceGet();
			st.insert(make_pair(0, make_pair(id, tables[id].GetType())));
			if (robots[i].GetBuyorSell() == -1) { //刚买货
				switch (tables[id].GetType()) {
				case 0: //类型从0开始
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 3 || ty == 4) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][0]) && (count(need.begin(), need.end(), 0) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (this->m_ChooseTable.count(ty)) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][0] = true;
						this->m_SellTable[goalTable[i]] += 2;
					}
					break;
				case 1:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 3 || ty == 5) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][1]) && (count(need.begin(), need.end(), 1) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (this->m_ChooseTable.count(ty)) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][1] = true;
						this->m_SellTable[goalTable[i]] += 4;
					}
					break;
				case 2:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 4 || ty == 5) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][2]) && (count(need.begin(), need.end(), 2) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (this->m_ChooseTable.count(ty)) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][2] = true;
						this->m_SellTable[goalTable[i]] += 8;
					}
					break;
				case 3:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][3]) && (count(need.begin(), need.end(), 3) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 16 == 112) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][3] = true;
						this->m_SellTable[goalTable[i]] += 16;
					}
					break;
				case 4:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][4]) && (count(need.begin(), need.end(), 4) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 32 == 112) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][4] = true;
						this->m_SellTable[goalTable[i]] += 32;
					}
					break;
				case 5:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][5]) && (count(need.begin(), need.end(), 5) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 64 == 112) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][5] = true;
						this->m_SellTable[goalTable[i]] += 64;
					}
					break;
				case 6:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 7 || ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp; //不设锁
							break;
						}
					}
					break;
				default:
					break;
				}
			}
			else if (robots[i].GetBuyorSell() == 1) { //刚卖货
				int GTID = robots[i].GetTask()[0];
				int GT = tables[GTID].GetType();
				if ((!this->m_TableFlagBuy[GTID]) && tables[GTID].HaveProduct() && (this->m_ChooseTable.count(GT) > 0 || GT == 6)) {
					//Order_2.push_back(make_pair("buy", GTID));
					goalTable[i] = GTID;
					this->m_TableFlagBuy[GTID] = true;
					int num = this->m_ChooseTable.count(GT);
					this->m_ChooseTable.erase(GT);
					for (int cc = 1; cc < num; cc++) {
						this->m_ChooseTable.insert(GT);
					}
				}
				else {
					int tyy;
					for (int j = 1; j <= 3; j++) {
						if (flag == 1) break;
						for (it = st.begin(); it != st.end(); it++) {
							int ty = (*it).second.second;
							if (j == 1 && ty == 6) {
								int temp = (*it).second.first;
								t1 = tables[temp].CountDownFrameGet() / 50.0;
								t2 = (*it).first / speed;
								if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2)) && tables[temp].NeedWhat().size() == 0) {
									goalTable[i] = temp;
									this->m_TableFlagBuy[temp] = true;
									flag = 1;
									break;
								}
							}
							else if (j == 2 && ty >= 3 && ty <= 5) {
								if (this->m_ChooseTable.count(ty) > 0) {
									int temp = (*it).second.first;
									t1 = tables[temp].CountDownFrameGet() / 50.0;
									t2 = (*it).first / speed;
									if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2)) && tables[temp].NeedWhat().size() == 0) {
										goalTable[i] = temp;
										this->m_TableFlagBuy[temp] = true;
										int num = this->m_ChooseTable.count(ty);
										this->m_ChooseTable.erase(ty);
										for (int cc = 1; cc < num; cc++) {
											this->m_ChooseTable.insert(ty);
										}
										flag = 1;
										break;
									}
								}
							}
							else if (j == 2 && ty >= 0 && ty <= 2) {
								if (this->m_ChooseTable.count(ty) > 0) {
									int temp = (*it).second.first;
									if ((!this->m_TableFlagBuy[temp]) && tables[temp].HaveProduct()) {
										if (flag == 0) {
											goalTable[i] = temp;
											flag = 3;
											tyy = ty;
										}
										switch (ty) {
										case 0:
											if (this->m_ChooseTable.count(3) || this->m_ChooseTable.count(4)) {
												goalTable[i] = temp;
												tyy = ty;
												flag = 2;
											}
											break;
										case 1:
											if (this->m_ChooseTable.count(3) || this->m_ChooseTable.count(5)) {
												goalTable[i] = temp;
												tyy = ty;
												flag = 2;
											}
											break;
										case 2:
											if (this->m_ChooseTable.count(4) || this->m_ChooseTable.count(5)) {
												goalTable[i] = temp;
												tyy = ty;
												flag = 2;
											}
											break;
										default:
											break;
										}
										if (flag == 2) {
											this->m_TableFlagBuy[goalTable[i]] = true;
											int num = this->m_ChooseTable.count(tyy);
											this->m_ChooseTable.erase(tyy);
											for (int cc = 1; cc < num; cc++) {
												this->m_ChooseTable.insert(tyy);
											}
											flag = 1;
											break;
										}
									}
								}

							}
						}
					}
					if (flag == 3) {
						this->m_TableFlagBuy[goalTable[i]] = true;
						int num = this->m_ChooseTable.count(tyy);
						this->m_ChooseTable.erase(tyy);
						for (int cc = 1; cc < num; cc++) {
							this->m_ChooseTable.insert(tyy);
						}
					}
				}
			}
		}
	}
	this->GiveTask(robots, goalTable, Road);//如果实在是没地方去会怎么样？？？？？
}

void RobotManager::SingleManager_5(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road,int frameID)  //只考虑买123
{
	float t1;
	float t2;
	float speed = 6.0; //匀速指标
	int goalTable[5] = { -1,-1,-1,-1 };
	set<std::pair<float, std::pair<int, int> > >::iterator it;
	if (frameID == 0) { //初始化
		for (int i = 0; i < 4; i++) {
			set<std::pair<float, std::pair<int, int> > > st = robots[i].DistanceGet();
			for (it = st.begin(); it != st.end(); it++) {
				if ((*it).second.second >= 0 && (*it).second.second <= 2) {
					int temp = (*it).second.first;
					if (!this->m_TableFlagBuy[temp]) {
						goalTable[i] = temp;
						this->m_TableFlagBuy[temp] = true;
						break;
					}
				}
			}
		}
	}
	else {
		SetChooseTable(tables); //买东西只能从中选择
		for (int i = 0; i < 4; i++) {
			if (robots[i].isBusy()) continue;
			int id = robots[i].GetTask()[0];
			//this->m_TableFlag[id] = false; //买卖之后就不占用了
			set<std::pair<float, std::pair<int, int> > > st = tables[id].DistanceGet();
			st.insert(make_pair(0, make_pair(id, tables[id].GetType())));
			if (robots[i].GetBuyorSell() == -1) { //刚买货
				switch (tables[id].GetType()) {
				case 0: //类型从0开始
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 3 || ty == 4) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][0]) && (count(need.begin(), need.end(), 0) > 0)) { //没有被预定且有产品格
								goalTable[i] = temp;
								this->m_TableFlagSell[temp][0] = true;
								this->m_SellTable[temp] += 2;
								break;
							}
						}
					}
					break;
				case 1:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 3 || ty == 5) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][1]) && (count(need.begin(), need.end(), 1) > 0)) { //没有被预定且有产品格
								goalTable[i] = temp;
								this->m_TableFlagSell[temp][1] = true;
								this->m_SellTable[temp] += 4;
								break;
							}
						}
					}
					break;
				case 2:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 4 || ty == 5) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][2]) && (count(need.begin(), need.end(), 2) > 0)) { //没有被预定且有产品格
								goalTable[i] = temp;
								this->m_TableFlagSell[temp][2] = true;
								this->m_SellTable[temp] += 8;
								break;
							}
						}
					}
					break;
				default:
					break;
				}
			}
			else if (robots[i].GetBuyorSell() == 1) { //刚卖货
				for (it = st.begin(); it != st.end(); it++) {
					int ty = (*it).second.second;
					if (ty >= 0 && ty <= 2) {
						if (this->m_ChooseTable.count(ty) > 0) {
							int temp = (*it).second.first;
							t1 = tables[temp].CountDownFrameGet() / 50.0;
							t2 = (*it).first / speed;
							if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2))) { //没有被预定且有产品可以拿
								goalTable[i] = temp;
								this->m_TableFlagBuy[temp] = true;
								int num = this->m_ChooseTable.count(ty);
								this->m_ChooseTable.erase(ty);
								for (int cc = 1; cc < num; cc++) {
									this->m_ChooseTable.insert(ty);
								}
								break;
							}
						}
					}
				}
			}
		}
	}
	this->GiveTask(robots, goalTable, Road);//如果实在是没地方去会怎么样？？？？？
}

void RobotManager::SingleManager_6(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road,int frameID) //123不上锁+顺道买东西
{
	float t1;
	float t2;
	float speed = 6.0; //匀速指标
	int goalTable[5] = { -1,-1,-1,-1 };
	set<std::pair<float, std::pair<int, int> > >::iterator it;
	if (frameID == 0) { //初始化
		for (int i = 0; i < 4; i++) {
			set<std::pair<float, std::pair<int, int> > > st = robots[i].DistanceGet();
			for (it = st.begin(); it != st.end(); it++) {
				if ((*it).second.second >= 0 && (*it).second.second <= 2) {
					int temp = (*it).second.first;
					if (!this->m_TableFlagBuy[temp]) {
						goalTable[i] = temp;
						this->m_TableFlagBuy[temp] = true;
						break;
					}
				}
			}
		}
	}
	else {
		SetChooseTable_2(tables, robots); //买东西只能从中选择
		SetTargetTable_2(tables, robots); //2
		for (int i = 0; i < 4; i++) {
			if (robots[i].isBusy()) continue;
			int id = robots[i].GetTask()[0];
			int flag = 0;
			//this->m_TableFlag[id] = false; //买卖之后就不占用了
			set<std::pair<float, std::pair<int, int> > > st = tables[id].DistanceGet();
			st.insert(make_pair(0, make_pair(id, tables[id].GetType())));
			if (robots[i].GetBuyorSell() == -1) { //刚买货
				switch (tables[id].GetType()) {
				case 0: //类型从0开始
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 3 || ty == 4) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][0]) && (count(need.begin(), need.end(), 0) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if ((this->m_TargetTable[ty] > 0 && (tables[temp].GetMaterialState() | this->m_SellTable[temp]) > 0) || this->m_TargetTable[ty] == -100) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][0] = true;
						this->m_SellTable[goalTable[i]] += 2;
					}
					break;
				case 1:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 3 || ty == 5) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][1]) && (count(need.begin(), need.end(), 1) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if ((this->m_TargetTable[ty] > 0 && (tables[temp].GetMaterialState() | this->m_SellTable[temp]) > 0) || this->m_TargetTable[ty] == -100) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][1] = true;
						this->m_SellTable[goalTable[i]] += 4;
					}
					break;
				case 2:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 4 || ty == 5) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][2]) && (count(need.begin(), need.end(), 2) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if ((this->m_TargetTable[ty] > 0 && (tables[temp].GetMaterialState() | this->m_SellTable[temp]) > 0) || this->m_TargetTable[ty] == -100) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][2] = true;
						this->m_SellTable[goalTable[i]] += 8;
					}
					break;
				case 3:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][3]) && (count(need.begin(), need.end(), 3) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 16 == m_TypeTable[temp]) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][3] = true;
						this->m_SellTable[goalTable[i]] += 16;
					}
					break;
				case 4:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][4]) && (count(need.begin(), need.end(), 4) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 32 == m_TypeTable[temp]) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][4] = true;
						this->m_SellTable[goalTable[i]] += 32;
					}
					break;
				case 5:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][5]) && (count(need.begin(), need.end(), 5) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 64 == m_TypeTable[temp]) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][5] = true;
						this->m_SellTable[goalTable[i]] += 64;
					}
					break;
				case 6:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 7 || ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp; //不设锁
							break;
						}
					}
					break;
				default:
					break;
				}
			}
			else if (robots[i].GetBuyorSell() == 1) { //刚卖货
				int GTID = robots[i].GetTask()[0];
				int GT = tables[GTID].GetType();
				if ((!this->m_TableFlagBuy[GTID]) && tables[GTID].HaveProduct() && (this->m_ChooseTable.count(GT) > 0 || GT == 6)) {
					//Order_2.push_back(make_pair("buy", GTID));
					goalTable[i] = GTID;
					this->m_TableFlagBuy[GTID] = true;
					int num = this->m_ChooseTable.count(GT);
					this->m_ChooseTable.erase(GT);
					for (int cc = 1; cc < num; cc++) {
						this->m_ChooseTable.insert(GT);
					}
				}
				else {
					int flag = 0;
					int tyy;
					for (int j = 1; j <= 3; j++) { //搜索三次，分别为7，456，123
						if (flag == 1) break;
						for (it = st.begin(); it != st.end(); it++) {
							int ty = (*it).second.second;
							if (j == 1 && ty == 6) {
								int temp = (*it).second.first;
								t1 = tables[temp].CountDownFrameGet() / 50.0;
								t2 = (*it).first / speed;
								if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() && (tables[temp].CountDownFrameGet() >= 0 && t1 < t2)) && tables[temp].NeedWhat().size() == 0) {
									goalTable[i] = temp;
									this->m_TableFlagBuy[temp] = true;
									flag = 1;
									break;
								}
							}
							else if (j == 2 && (ty >= 3 && ty <= 5)) {
								if (this->m_ChooseTable.count(ty) > 0) {
									int temp = (*it).second.first;
									t1 = tables[temp].CountDownFrameGet() / 50.0;
									t2 = (*it).first / speed;
									if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2)) && tables[temp].NeedWhat().size() == 0) {
										goalTable[i] = temp;
										this->m_TableFlagBuy[temp] = true;
										int num = this->m_ChooseTable.count(ty);
										this->m_ChooseTable.erase(ty);
										for (int cc = 1; cc < num; cc++) {
											this->m_ChooseTable.insert(ty);
										}
										flag = 1;
										break;
									}
								}
							}
							else if (j == 3 && (ty >= 0 && ty <= 2)) {
								if (this->m_ChooseTable.count(ty) > 0) {
									int temp = (*it).second.first;
									t1 = tables[temp].CountDownFrameGet() / 50.0;
									t2 = (*it).first / speed;
									if (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2)) {
										if (flag == 0) {
											goalTable[i] = temp;
											flag = 3;
											tyy = ty;
										}
										if (this->m_TargetTable[ty] > 0) {
											goalTable[i] = temp;
											flag = 2;
											this->m_TargetTable[ty] = m_TargetTable[ty] - 1;
											int num = this->m_ChooseTable.count(ty);
											this->m_ChooseTable.erase(ty);
											for (int cc = 1; cc < num; cc++) {
												this->m_ChooseTable.insert(ty);
											}
											flag = 1;
											break;
										}
									}
								}
							}
						}
					}
					if (flag == 3) {
						int num = this->m_ChooseTable.count(tyy);
						this->m_ChooseTable.erase(tyy);
						for (int cc = 1; cc < num; cc++) {
							this->m_ChooseTable.insert(tyy);
						}
					}
				}
			}
		}
	}
	this->GiveTask(robots, goalTable, Road);//如果实在是没地方去会怎么样？？？？？
}

void RobotManager::SingleManager_7(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road,int frameID) //没有7但考虑9
{
	float t1;
	float t2;
	float speed = 6.0; //匀速指标
	int goalTable[5] = { -1,-1,-1,-1 };
	set<std::pair<float, std::pair<int, int> > >::iterator it;
	if (frameID == 0) { //初始化
		for (int i = 0; i < 4; i++) {
			set<std::pair<float, std::pair<int, int> > > st = robots[i].DistanceGet();
			for (it = st.begin(); it != st.end(); it++) {
				if ((*it).second.second >= 0 && (*it).second.second <= 2) {
					int temp = (*it).second.first;
					if (!this->m_TableFlagBuy[temp]) {
						goalTable[i] = temp;
						this->m_TableFlagBuy[temp] = true;
						break;
					}
				}
			}
		}
	}
	else {
		SetChooseTable_3(tables); //买东西只能从中选择
		SetTargetTable(tables, robots);
		for (int i = 0; i < 4; i++) {
			if (robots[i].isBusy()) continue;
			int id = robots[i].GetTask()[0];
			int flag = 0;
			//this->m_TableFlag[id] = false; //买卖之后就不占用了
			set<std::pair<float, std::pair<int, int> > > st = tables[id].DistanceGet();
			st.insert(make_pair(0, make_pair(id, tables[id].GetType())));
			if (robots[i].GetBuyorSell() == -1) { //刚买货
				switch (tables[id].GetType()) {
				case 0: //类型从0开始
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 3 || ty == 4) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][0]) && (count(need.begin(), need.end(), 0) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if ((this->m_TargetTable[ty] > 0 && (tables[temp].GetMaterialState() | this->m_SellTable[temp]) > 0) || this->m_TargetTable[ty] == -100) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][0] = true;
						this->m_SellTable[goalTable[i]] += 2;
					}
					break;
				case 1:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 3 || ty == 5) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][1]) && (count(need.begin(), need.end(), 1) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if ((this->m_TargetTable[ty] > 0 && (tables[temp].GetMaterialState() | this->m_SellTable[temp]) > 0) || this->m_TargetTable[ty] == -100) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][1] = true;
						this->m_SellTable[goalTable[i]] += 4;
					}
					break;
				case 2:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 4 || ty == 5) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][2]) && (count(need.begin(), need.end(), 2) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if ((this->m_TargetTable[ty] > 0 && (tables[temp].GetMaterialState() | this->m_SellTable[temp]) > 0) || this->m_TargetTable[ty] == -100) {
									goalTable[i] = temp;
									break;
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][2] = true;
						this->m_SellTable[goalTable[i]] += 8;
					}
					break;
				case 3:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][3]) && (count(need.begin(), need.end(), 3) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 16 == m_TypeTable[temp]) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][3] = true;
						this->m_SellTable[goalTable[i]] += 16;
					}
					break;
				case 4:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][4]) && (count(need.begin(), need.end(), 4) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 32 == m_TypeTable[temp]) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][4] = true;
						this->m_SellTable[goalTable[i]] += 32;
					}
					break;
				case 5:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (this->SevenFlag && ty == 6) {
							int temp = (*it).second.first;
							vector<int> need = tables[temp].NeedWhat();
							if ((!this->m_TableFlagSell[temp][5]) && (count(need.begin(), need.end(), 5) > 0)) { //没有被预定且有产品格
								if (flag == 0) {
									goalTable[i] = temp;
									flag = 1;
								}
								if (m_SellTable[temp] + 64 == m_TypeTable[temp]) {
									goalTable[i] = temp;
									break;
								}
							}
						}
						else if (!this->SevenFlag && ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp;  //没有设置锁
							break;
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][5] = true;
						this->m_SellTable[goalTable[i]] += 64;
					}
					break;
				case 6:
					for (it = st.begin(); it != st.end(); it++) {
						int ty = (*it).second.second;
						if (ty == 7 || ty == 8) {
							int temp = (*it).second.first;
							goalTable[i] = temp; //不设锁
							break;
						}
					}
					break;
				default:
					break;
				}
			}
			else if (robots[i].GetBuyorSell() == 1) { //刚卖货
				int GTID = robots[i].GetTask()[0];
				int GT = tables[GTID].GetType();
				if ((!this->m_TableFlagBuy[GTID]) && tables[GTID].HaveProduct() && (this->m_ChooseTable.count(GT) > 0 || GT == 6) && CanSellTo9(i, tables, robots, goalTable)) {
					//Order_2.push_back(make_pair("buy", GTID));
					goalTable[i] = GTID;
					this->m_TableFlagBuy[GTID] = true;
					int num = this->m_ChooseTable.count(GT);
					this->m_ChooseTable.erase(GT);
					for (int cc = 1; cc < num; cc++) {
						this->m_ChooseTable.insert(GT);
					}
				}
				else {
					int flag = 0;
					int tyy;
					for (int j = 1; j <= 3; j++) { //搜索三次，分别为7，456，123
						if (flag == 1) break;
						for (it = st.begin(); it != st.end(); it++) {
							int ty = (*it).second.second;
							if (j == 1 && ty == 6) {
								int temp = (*it).second.first;
								t1 = tables[temp].CountDownFrameGet() / 50.0;
								t2 = (*it).first / speed;
								if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2)) && tables[temp].NeedWhat().size() == 0) { //没有被预定且有产品可以拿
									goalTable[i] = temp;
									this->m_TableFlagBuy[temp] = true;
									flag = 1;
									break;
								}
							}
							else if (j == 2 && (ty >= 3 && ty <= 5)) {
								if (this->m_ChooseTable.count(ty) > 0) {
									int temp = (*it).second.first;
									t1 = tables[temp].CountDownFrameGet() / 50.0;
									t2 = (*it).first / speed;
									if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2)) && tables[temp].NeedWhat().size() == 0) {
										goalTable[i] = temp;
										this->m_TableFlagBuy[temp] = true;
										int num = this->m_ChooseTable.count(ty);
										this->m_ChooseTable.erase(ty);
										for (int cc = 1; cc < num; cc++) {
											this->m_ChooseTable.insert(ty);
										}
										flag = 1;
										break;
									}
								}
							}
							else if (j == 3 && (ty >= 0 && ty <= 2)) {
								if (this->m_ChooseTable.count(ty) > 0) {
									int temp = (*it).second.first;
									t1 = tables[temp].CountDownFrameGet() / 50.0;
									t2 = (*it).first / speed;
									if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2))) {
										if (flag == 0) {
											goalTable[i] = temp;
											flag = 3;
											tyy = ty;
										}
										if (this->m_TargetTable[ty] > 0) {
											this->m_TableFlagBuy[temp] = true;
											goalTable[i] = temp;
											flag = 2;
											this->m_TargetTable[ty] = m_TargetTable[ty] - 1;
											int num = this->m_ChooseTable.count(ty);
											this->m_ChooseTable.erase(ty);
											for (int cc = 1; cc < num; cc++) {
												this->m_ChooseTable.insert(ty);
											}
											flag = 1;
											break;
										}
									}
								}
							}
						}
					}
					if (flag == 3) {
						this->m_TableFlagBuy[goalTable[i]] = true;
						int num = this->m_ChooseTable.count(tyy);
						this->m_ChooseTable.erase(tyy);
						for (int cc = 1; cc < num; cc++) {
							this->m_ChooseTable.insert(tyy);
						}
					}
				}
			}
		}
	}
	this->GiveTask(robots, goalTable, Road);//如果实在是没地方去会怎么样？？？？？
}

void RobotManager::SingleManager_8(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road,int frameID) //机器人分图
{
	/*set<int> AccessID_2{ 11,12,13,14,16,20,22,23,24,25,28,29,32,33,34 }; //允许前往的编号B
	set<int> AccessID_0{ 0,1,2,3,4,5,6,24 }; //允许前往的编号0
	set<int> AccessID_1{ 8,9,10,15,17,18,24,26,27,30,31 }; //允许前往的编号R
	set<int> AccessID_3{ 24,36,41,42,44,45,47,48 }; //允许前往的编号02
	set<int> AccessID_4{ 24,37,38,39,40,46,49 }; //允许前往的编号T*/
	set<int> AccessID_2{ 16,24,28,32 }; //允许前往的编号B
	set<int> AccessID_0{ 0,4,6,24 }; //允许前往的编号0
	set<int> AccessID_1{ 15,24,27,31 }; //允许前往的编号R
	set<int> AccessID_3{ 24,36,41,42 }; //允许前往的编号02
	set<int> AccessID_4{ 24,37,38,39,40,46,49 }; //允许前往的编号T
	float t1;
	float t2;
	float speed = 6.0; //匀速指标
	int goalTable[5] = { -1,-1,-1,-1 };
	set<std::pair<float, std::pair<int, int> > >::iterator it;
	if (frameID == 0) { //初始化
		for (int i = 0; i < 4; i++) {
			set<std::pair<float, std::pair<int, int> > > st = robots[i].DistanceGet();
			for (it = st.begin(); it != st.end(); it++) {
				if ((i == 0 && AccessID_0.count((*it).second.first)) || (i == 1 && AccessID_1.count((*it).second.first)) || (i == 2 && AccessID_2.count((*it).second.first)) || (i == 3 && AccessID_3.count((*it).second.first))) {
					if ((*it).second.second >= 0 && (*it).second.second <= 2) {
						int temp = (*it).second.first;
						if (!this->m_TableFlagBuy[temp]) {
							goalTable[i] = temp;
							this->m_TableFlagBuy[temp] = true;
							break;
						}
					}
				}
			}
		}
	}
	else {
		SetChooseTable_5(tables, AccessID_0, AccessID_1, AccessID_2, AccessID_3); //买东西只能从中选择
		SetTargetTable(tables, robots);
		for (int i = 0; i < 4; i++) {
			if (robots[i].isBusy()) continue;
			int id = robots[i].GetTask()[0];
			int flag = 0;
			//this->m_TableFlag[id] = false; //买卖之后就不占用了
			set<std::pair<float, std::pair<int, int> > > st = tables[id].DistanceGet();
			st.insert(make_pair(0, make_pair(id, tables[id].GetType())));
			if (robots[i].GetBuyorSell() == -1) { //刚买货
				switch (tables[id].GetType()) {
				case 0: //类型从0开始
					for (it = st.begin(); it != st.end(); it++) {
						if ((i == 0 && AccessID_0.count((*it).second.first)) || (i == 1 && AccessID_1.count((*it).second.first)) || (i == 2 && AccessID_2.count((*it).second.first)) || (i == 3 && AccessID_3.count((*it).second.first))) {
							int ty = (*it).second.second;
							if (ty == 3 || ty == 4) {
								int temp = (*it).second.first;
								vector<int> need = tables[temp].NeedWhat();
								if ((!this->m_TableFlagSell[temp][0]) && (count(need.begin(), need.end(), 0) > 0)) { //没有被预定且有产品格
									if (flag == 0) {
										goalTable[i] = temp;
										flag = 1;
									}
									if ((tables[temp].GetMaterialState() | this->m_SellTable[temp]) > 0) {
										goalTable[i] = temp;
										break;
									}
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][0] = true;
						this->m_SellTable[goalTable[i]] += 2;
					}
					break;
				case 1:
					for (it = st.begin(); it != st.end(); it++) {
						if ((i == 0 && AccessID_0.count((*it).second.first)) || (i == 1 && AccessID_1.count((*it).second.first)) || (i == 2 && AccessID_2.count((*it).second.first)) || (i == 3 && AccessID_3.count((*it).second.first))) {
							int ty = (*it).second.second;
							if (ty == 3 || ty == 5) {
								int temp = (*it).second.first;
								vector<int> need = tables[temp].NeedWhat();
								if ((!this->m_TableFlagSell[temp][1]) && (count(need.begin(), need.end(), 1) > 0)) { //没有被预定且有产品格
									if (flag == 0) {
										goalTable[i] = temp;
										flag = 1;
									}
									if ((tables[temp].GetMaterialState() | this->m_SellTable[temp]) > 0) {
										goalTable[i] = temp;
										break;
									}
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][1] = true;
						this->m_SellTable[goalTable[i]] += 4;
					}
					break;
				case 2:
					for (it = st.begin(); it != st.end(); it++) {
						if ((i == 0 && AccessID_0.count((*it).second.first)) || (i == 1 && AccessID_1.count((*it).second.first)) || (i == 2 && AccessID_2.count((*it).second.first)) || (i == 3 && AccessID_3.count((*it).second.first))) {
							int ty = (*it).second.second;
							if (ty == 4 || ty == 5) {
								int temp = (*it).second.first;
								vector<int> need = tables[temp].NeedWhat();
								if ((!this->m_TableFlagSell[temp][2]) && (count(need.begin(), need.end(), 2) > 0)) { //没有被预定且有产品格
									if (flag == 0) {
										goalTable[i] = temp;
										flag = 1;
									}
									if ((tables[temp].GetMaterialState() | this->m_SellTable[temp]) > 0) {
										goalTable[i] = temp;
										break;
									}
								}
							}
						}
					}
					if (flag == 1) {
						this->m_TableFlagSell[goalTable[i]][2] = true;
						this->m_SellTable[goalTable[i]] += 8;
					}
					break;
				case 3:
					for (it = st.begin(); it != st.end(); it++) {
						if ((i == 0 && AccessID_0.count((*it).second.first)) || (i == 1 && AccessID_1.count((*it).second.first)) || (i == 2 && AccessID_2.count((*it).second.first)) || (i == 3 && AccessID_3.count((*it).second.first))) {
							int ty = (*it).second.second;
							if (this->SevenFlag && ty == 6) {
								int temp = (*it).second.first;
								vector<int> need = tables[temp].NeedWhat();
								if ((!this->m_TableFlagSell[temp][3]) && (count(need.begin(), need.end(), 3) > 0)) { //没有被预定且有产品格
									if (flag == 0) {
										goalTable[i] = temp;
										flag = 1;
									}
									if (m_SellTable[temp] + 16 == m_TypeTable[temp]) {
										goalTable[i] = temp;
										break;
									}
								}
							}
							else if (!this->SevenFlag && ty == 8) {
								int temp = (*it).second.first;
								goalTable[i] = temp;  //没有设置锁
								break;
							}
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][3] = true;
						this->m_SellTable[goalTable[i]] += 16;
					}
					break;
				case 4:
					for (it = st.begin(); it != st.end(); it++) {
						if ((i == 0 && AccessID_0.count((*it).second.first)) || (i == 1 && AccessID_1.count((*it).second.first)) || (i == 2 && AccessID_2.count((*it).second.first)) || (i == 3 && AccessID_3.count((*it).second.first))) {
							int ty = (*it).second.second;
							if (this->SevenFlag && ty == 6) {
								int temp = (*it).second.first;
								vector<int> need = tables[temp].NeedWhat();
								if ((!this->m_TableFlagSell[temp][4]) && (count(need.begin(), need.end(), 4) > 0)) { //没有被预定且有产品格
									if (flag == 0) {
										goalTable[i] = temp;
										flag = 1;
									}
									if (m_SellTable[temp] + 32 == m_TypeTable[temp]) {
										goalTable[i] = temp;
										break;
									}
								}
							}
							else if (!this->SevenFlag && ty == 8) {
								int temp = (*it).second.first;
								goalTable[i] = temp;  //没有设置锁
								break;
							}
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][4] = true;
						this->m_SellTable[goalTable[i]] += 32;
					}
					break;
				case 5:
					for (it = st.begin(); it != st.end(); it++) {
						if ((i == 0 && AccessID_0.count((*it).second.first)) || (i == 1 && AccessID_1.count((*it).second.first)) || (i == 2 && AccessID_2.count((*it).second.first)) || (i == 3 && AccessID_3.count((*it).second.first))) {
							int ty = (*it).second.second;
							if (this->SevenFlag && ty == 6) {
								int temp = (*it).second.first;
								vector<int> need = tables[temp].NeedWhat();
								if ((!this->m_TableFlagSell[temp][5]) && (count(need.begin(), need.end(), 5) > 0)) { //没有被预定且有产品格
									if (flag == 0) {
										goalTable[i] = temp;
										flag = 1;
									}
									if (m_SellTable[temp] + 64 == m_TypeTable[temp]) {
										goalTable[i] = temp;
										break;
									}
								}
							}
							else if (!this->SevenFlag && ty == 8) {
								int temp = (*it).second.first;
								goalTable[i] = temp;  //没有设置锁
								break;
							}
						}
					}
					if (this->SevenFlag && flag == 1) {
						this->m_TableFlagSell[goalTable[i]][5] = true;
						this->m_SellTable[goalTable[i]] += 64;
					}
					break;
				case 6:
					for (it = st.begin(); it != st.end(); it++) {
						if ((i == 0 && AccessID_0.count((*it).second.first)) || (i == 1 && AccessID_1.count((*it).second.first)) || (i == 2 && AccessID_2.count((*it).second.first)) || (i == 3 && AccessID_3.count((*it).second.first))) {
							int ty = (*it).second.second;
							if (ty == 7 || ty == 8) {
								int temp = (*it).second.first;
								goalTable[i] = temp; //不设锁
								break;
							}
						}
					}
					break;
				default:
					break;
				}
			}
			else if (robots[i].GetBuyorSell() == 1) { //刚卖货
				int GTID = robots[i].GetTask()[0];
				int GT = tables[GTID].GetType();
				if (((i == 0 && AccessID_0.count(GTID)) || (i == 1 && AccessID_1.count(GTID)) || (i == 2 && AccessID_2.count(GTID)) || (i == 3 && AccessID_3.count(GTID))) && (!this->m_TableFlagBuy[GTID]) && tables[GTID].HaveProduct() > 0 && (this->m_ChooseTable2[i].count(GT) > 0 || GT == 6) && CanSellTo9(i, tables, robots, goalTable)) {
					//Order_2.push_back(make_pair("buy", GTID));
					goalTable[i] = GTID;
					this->m_TableFlagBuy[GTID] = true;
					int num = this->m_ChooseTable2[i].count(GT);
					this->m_ChooseTable2[i].erase(GT);
					for (int cc = 1; cc < num; cc++) {
						this->m_ChooseTable2[i].insert(GT);
					}
				}
				else {
					int flag = 0;
					int tyy;
					for (int j = 1; j <= 3; j++) { //搜索三次，分别为7，456，123
						if (flag == 1) break;
						for (it = st.begin(); it != st.end(); it++) {
							if ((i == 0 && AccessID_0.count((*it).second.first)) || (i == 1 && AccessID_1.count((*it).second.first)) || (i == 2 && AccessID_2.count((*it).second.first)) || (i == 3 && AccessID_3.count((*it).second.first))) {
								int ty = (*it).second.second;
								if (j == 1 && ty == 6) {
									int temp = (*it).second.first;
									t1 = tables[temp].CountDownFrameGet() / 50.0;
									t2 = (*it).first / speed;
									if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2)) && tables[temp].NeedWhat().size() == 0) { //没有被预定且有产品可以拿
										goalTable[i] = temp;
										this->m_TableFlagBuy[temp] = true;
										flag = 1;
										break;
									}
								}
								else if (j == 2 && (ty >= 3 && ty <= 5)) {
									if (this->m_ChooseTable2[i].count(ty) > 0) {
										int temp = (*it).second.first;
										t1 = tables[temp].CountDownFrameGet() / 50.0;
										t2 = (*it).first / speed;
										if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2)) && tables[temp].NeedWhat().size() == 0) {
											goalTable[i] = temp;
											this->m_TableFlagBuy[temp] = true;
											int num = this->m_ChooseTable2[i].count(ty);
											this->m_ChooseTable2[i] .erase(ty);
											for (int cc = 1; cc < num; cc++) {
												this->m_ChooseTable2[i].insert(ty);
											}
											flag = 1;
											break;
										}
									}
								}
								else if (j == 3 && (ty >= 0 && ty <= 2)) {
									if (this->m_ChooseTable2[i].count(ty) > 0) {
										int temp = (*it).second.first;
										t1 = tables[temp].CountDownFrameGet() / 50.0;
										t2 = (*it).first / speed;
										if ((!this->m_TableFlagBuy[temp]) && (tables[temp].HaveProduct() || (tables[temp].CountDownFrameGet() >= 0 && t1 < t2))) {
											if (flag == 0) {
												goalTable[i] = temp;
												flag = 3;
												tyy = ty;
											}
											if (this->m_TargetTable[ty] > 0) {
												this->m_TableFlagBuy[temp] = true;
												goalTable[i] = temp;
												flag = 2;
												this->m_TargetTable[ty] = m_TargetTable[ty] - 1;
												int num = this->m_ChooseTable2[i].count(ty);
												this->m_ChooseTable2[i].erase(ty);
												for (int cc = 1; cc < num; cc++) {
													this->m_ChooseTable2[i].insert(ty);
												}
												flag = 1;
												break;
											}
										}
									}
								}
							}
						}
					}
					if (flag == 3) {
						this->m_TableFlagBuy[goalTable[i]] = true;
						int num = this->m_ChooseTable2[i].count(tyy);
						this->m_ChooseTable2[i].erase(tyy);
						for (int cc = 1; cc < num; cc++) {
							this->m_ChooseTable2[i].insert(tyy);
						}
					}
				}
			}
		}
	}
	this->GiveTask(robots, goalTable, Road);//如果实在是没地方去会怎么样？？？？？
}

void RobotManager::SetSevenFlag(bool sf)
{
	this->SevenFlag = sf;
}

void RobotManager::InitialTypeTable(std::vector<int> tt)
{
	std::vector<int> kk;
	for (int i = 0; i < tt.size(); i++) {
		int temp = tt[i];
		switch (temp) {
		case 3:
			kk.push_back(6);
			break;
		case 4:
			kk.push_back(10);
			break;
		case 5:
			kk.push_back(12);
			break;
		case 6:
			kk.push_back(112);
			break;
			/*case 7:
			kk.push_back(128);
			break;*/
		default:
			kk.push_back(0);
			break;
		}
	}
	this->m_TypeTable = kk;
	this->m_TableNum = tt.size();
}

void RobotManager::SetChooseTable_5(WorkTable * tables, set<int> & ID_0, set<int> & ID_1, set<int> & ID_2, set<int> & ID_3) //一个机器人一个
{
	this->m_ChooseTable2.clear();
	this->m_ChooseTable2.push_back(this->mt0);
	this->m_ChooseTable2.push_back(this->mt1);
	this->m_ChooseTable2.push_back(this->mt2);
	this->m_ChooseTable2.push_back(this->mt3);
	for (int i = 0; i < this->m_TableNum; i++) {
		if(ID_0.count(i)){
			int temp = this->m_TypeTable[i] - (tables[i].GetMaterialState() | this->m_SellTable[i]);
			int iter = 0;
			while (true) {
				temp = temp >> 1;
				if (temp <= 0) break;
				if (temp % 2) {
					this->m_ChooseTable2[0].insert(iter); //类型从0开始
				}
				iter++;
			}
		}
		if (ID_1.count(i)) {
			int temp = this->m_TypeTable[i] - (tables[i].GetMaterialState() | this->m_SellTable[i]);
			int iter = 0;
			while (true) {
				temp = temp >> 1;
				if (temp <= 0) break;
				if (temp % 2) {
					this->m_ChooseTable2[1].insert(iter); //类型从0开始
				}
				iter++;
			}
		}
		if (ID_2.count(i)) {
			int temp = this->m_TypeTable[i] - (tables[i].GetMaterialState() | this->m_SellTable[i]);
			int iter = 0;
			while (true) {
				temp = temp >> 1;
				if (temp <= 0) break;
				if (temp % 2) {
					this->m_ChooseTable2[2].insert(iter); //类型从0开始
				}
				iter++;
			}
		}
		if (ID_3.count(i)) {
			int temp = this->m_TypeTable[i] - (tables[i].GetMaterialState() | this->m_SellTable[i]);
			int iter = 0;
			while (true) {
				temp = temp >> 1;
				if (temp <= 0) break;
				if (temp % 2) {
					this->m_ChooseTable2[3].insert(iter); //类型从0开始
				}
				iter++;
			}
		}
	}
	for (int i = 0; i < this->m_TableNum; i++) {
		if (ID_0.count(i)) {
			if (this->m_TableFlagBuy[i] && tables[i].GetType() < 6) {
				int GT = tables[i].GetType();
				int temp = this->m_ChooseTable2[0].count(GT);
				this->m_ChooseTable2[0].erase(GT);
				for (int j = 1; j < temp; j++) this->m_ChooseTable2[0].insert(GT);
			}
		}
		if (ID_1.count(i)) {
			if (this->m_TableFlagBuy[i] && tables[i].GetType() < 6) {
				int GT = tables[i].GetType();
				int temp = this->m_ChooseTable2[1].count(GT);
				this->m_ChooseTable2[1].erase(GT);
				for (int j = 1; j < temp; j++) this->m_ChooseTable2[1].insert(GT);
			}
		}
		if (ID_2.count(i)) {
			if (this->m_TableFlagBuy[i] && tables[i].GetType() < 6) {
				int GT = tables[i].GetType();
				int temp = this->m_ChooseTable2[2].count(GT);
				this->m_ChooseTable2[2].erase(GT);
				for (int j = 1; j < temp; j++) this->m_ChooseTable2[2].insert(GT);
			}
		}
		if (ID_3.count(i)) {
			if (this->m_TableFlagBuy[i] && tables[i].GetType() < 6) {
				int GT = tables[i].GetType();
				int temp = this->m_ChooseTable2[3].count(GT);
				this->m_ChooseTable2[3].erase(GT);
				for (int j = 1; j < temp; j++) this->m_ChooseTable2[3].insert(GT);
			}
		}
	}
	this->m_ChooseTable2[0].insert(4);
	this->m_ChooseTable2[1].insert(3);
	this->m_ChooseTable2[2].insert(5);
	this->m_ChooseTable2[3].insert(4);
}

void RobotManager::SetChooseTable_4(WorkTable * tables, Robot * robots) //没有7+123不上锁
{
	this->m_ChooseTable.clear();
	for (int i = 0; i < this->m_TableNum; i++) {
		int temp = this->m_TypeTable[i] - (tables[i].GetMaterialState() | this->m_SellTable[i]);
		int iter = 0;
		while (true) {
			temp = temp >> 1;
			if (temp <= 0) break;
			if (temp % 2) {
				this->m_ChooseTable.insert(iter); //类型从0开始
			}
			iter++;
		}
	}
	for (int i = 0; i < this->m_TableNum; i++) {
		if (this->m_TableFlagBuy[i] && tables[i].GetType() <= 5 && tables[i].GetType() >= 3) {
			int GT = tables[i].GetType();
			int temp = this->m_ChooseTable.count(GT);
			this->m_ChooseTable.erase(GT);
			for (int j = 1; j < temp; j++) this->m_ChooseTable.insert(GT);
		}
	}
	for (int i = 0; i < 4; i++) {
		int id = robots[i].GetTask()[0];
		int GTT = tables[id].GetType();
		if (GTT >= 0 && GTT <= 2) {
			int temp = this->m_ChooseTable.count(GTT);
			this->m_ChooseTable.erase(GTT);
			for (int j = 1; j < temp; j++) this->m_ChooseTable.insert(GTT);
		}
	}
	this->m_ChooseTable.insert(3);
	this->m_ChooseTable.insert(3);
	this->m_ChooseTable.insert(4);
	this->m_ChooseTable.insert(4);
	this->m_ChooseTable.insert(5);
	this->m_ChooseTable.insert(5);
}

void RobotManager::SetChooseTable_3(WorkTable * tables) //没有7的时候用的
{
	this->m_ChooseTable.clear();
	for (int i = 0; i < this->m_TableNum; i++) {
		int temp = this->m_TypeTable[i] - (tables[i].GetMaterialState() | this->m_SellTable[i]);
		int iter = 0;
		while (true) {
			temp = temp >> 1;
			if (temp <= 0) break;
			if (temp % 2) {
				this->m_ChooseTable.insert(iter); //类型从0开始
			}
			iter++;
		}
	}
	for (int i = 0; i < this->m_TableNum; i++) {
		if (this->m_TableFlagBuy[i] && tables[i].GetType() < 6) {
			int GT = tables[i].GetType();
			int temp = this->m_ChooseTable.count(GT);
			this->m_ChooseTable.erase(GT);
			for (int j = 1; j < temp; j++) this->m_ChooseTable.insert(GT);
		}
	}
	this->m_ChooseTable.insert(3);
	this->m_ChooseTable.insert(3);
	this->m_ChooseTable.insert(4);
	this->m_ChooseTable.insert(4);
	this->m_ChooseTable.insert(5);
	this->m_ChooseTable.insert(5);
}

void RobotManager::SetChooseTable(WorkTable * tables)
{
	this->m_ChooseTable.clear();
	for (int i = 0; i < this->m_TableNum; i++) {
		int temp = this->m_TypeTable[i] - (tables[i].GetMaterialState() | this->m_SellTable[i]);
		int iter = 0;
		while (true) {
			temp = temp >> 1;
			if (temp <= 0) break;
			if (temp % 2) {
				this->m_ChooseTable.insert(iter); //类型从0开始
			}
			iter++;
		}
	}
	for (int i = 0; i < this->m_TableNum; i++) {
		if (this->m_TableFlagBuy[i] && tables[i].GetType() < 6) {
			int GT = tables[i].GetType();
			int temp = this->m_ChooseTable.count(GT);
			this->m_ChooseTable.erase(GT);
			for (int j = 1; j < temp; j++) this->m_ChooseTable.insert(GT);
		}
	}
}

void RobotManager::SetChooseTable_2(WorkTable * tables, Robot * robots) //123不上锁时用的
{
	this->m_ChooseTable.clear();
	for (int i = 0; i < this->m_TableNum; i++) {
		int temp = this->m_TypeTable[i] - (tables[i].GetMaterialState() | this->m_SellTable[i]);
		int iter = 0;
		while (true) {
			temp = temp >> 1;
			if (temp <= 0) break;
			if (temp % 2) {
				this->m_ChooseTable.insert(iter); //类型从0开始
			}
			iter++;
		}
	}
	for (int i = 0; i < this->m_TableNum; i++) {
		if (this->m_TableFlagBuy[i] && tables[i].GetType() <= 5 && tables[i].GetType() >= 3) {
			int GT = tables[i].GetType();
			int temp = this->m_ChooseTable.count(GT);
			this->m_ChooseTable.erase(GT);
			for (int j = 1; j < temp; j++) this->m_ChooseTable.insert(GT);
		}
	}
	for (int i = 0; i < 4; i++) {
		int id = robots[i].GetTask()[0];
		int GTT = tables[id].GetType();
		if (GTT >= 0 && GTT <= 2) {
			int temp = this->m_ChooseTable.count(GTT);
			this->m_ChooseTable.erase(GTT);
			for (int j = 1; j < temp; j++) this->m_ChooseTable.insert(GTT);
		}
	}
}

void RobotManager::SetTargetTable(WorkTable * tables, Robot * robots) { //从全局的角度设定应该卖到哪儿
	this->m_TargetTable.clear();
	for (int i = 0; i < this->m_TableNum; i++) {
		if (tables[i].GetType() == 6) {
			m_TargetTable[3] = m_TargetTable[3] + 1;
			m_TargetTable[4] = m_TargetTable[4] + 1;
			m_TargetTable[5] = m_TargetTable[5] + 1;
			int iter = 0;
			int temp = tables[i].GetMaterialState();
			while (true) {
				temp = temp >> 1;
				if (temp <= 0) break;
				if (temp % 2) {
					m_TargetTable[iter] = m_TargetTable[iter] - 1;
				}
				iter++;
			}
		}
	}
	for (int i = 0; i < this->m_TableNum; i++) {
		if (tables[i].GetType() >= 3 && tables[i].GetType() <= 5) {
			int temp = tables[i].GetType();
			if (tables[i].HaveProduct()) m_TargetTable[temp] = m_TargetTable[temp] - 1;
			if (tables[i].CountDownFrameGet() >= 0) m_TargetTable[temp] = m_TargetTable[temp] - 1;
		}
	}
	for (int i = 0; i < 4; i++) { ///////有问题！！！！！但是分高
		if (robots[i].GetGoods() >= 4 && robots[i].GetGoods() <= 6) {
			int temp = robots[i].GetGoods();
			m_TargetTable[temp] = m_TargetTable[temp] - 1;
		}
	}
}

void RobotManager::SetTargetTable_2(WorkTable * tables, Robot * robots) { //从全局的角度设定应该买1，2，3
	int flagg[5] = { 0 };
	std::set<int> max_st;
	this->m_TargetTable.clear();
	for (int i = 0; i < this->m_TableNum; i++) {
		if (tables[i].GetType() >= 3 && tables[i].GetType() <= 5) {
			int temp = tables[i].GetType();
			if (tables[i].HaveProduct()) m_TargetTable[temp] = m_TargetTable[temp] + 1;
			if (tables[i].CountDownFrameGet() >= 0) m_TargetTable[temp] = m_TargetTable[temp] + 1;
		}
		else if (tables[i].GetType() == 6) {
			int iter = 0;
			int temp = tables[i].GetMaterialState();
			while (true) {
				temp = temp >> 1;
				if (temp <= 0) break;
				if (temp % 2) {
					m_TargetTable[iter] = m_TargetTable[iter] + 1;
				}
				iter++;
			}
		}
	}
	for (int i = 0; i < 4; i++) {
		if (robots[i].GetGoods() >= 4 && robots[i].GetGoods() <= 6) {
			int temp = robots[i].GetGoods() - 1;
			m_TargetTable[temp] = m_TargetTable[temp] + 1;
		}
	}
	for (int i = 0; i < 3; i++) {
		max_st.insert(m_TargetTable[i + 3]);
	}
	int max_TT = *(--max_st.end());
	int min_TT = *max_st.begin();
	for (int i = 0; i < 3; i++) {
		m_TargetTable[i + 3] = max_TT - m_TargetTable[i + 3];
	}
	if (max_TT == min_TT) {
		for (int i = 0; i < 3; i++) {
			m_TargetTable[i + 3] = 1;
		}
	}
	for (int i = 0; i < this->m_TableNum; i++) {
		if (tables[i].GetType() >= 3 && tables[i].GetType() <= 5 && m_TargetTable[tables[i].GetType()] > 0) {
			if ((tables[i].GetMaterialState() | this->m_SellTable[i]) > 0) {
				flagg[tables[i].GetType() - 3] = 1;
				int temp = this->m_TypeTable[i] - (tables[i].GetMaterialState() | this->m_SellTable[i]);
				int iter = 0;
				while (true) {
					temp = temp >> 1;
					if (temp <= 0) break;
					if (temp % 2) {
						m_TargetTable[iter] = m_TargetTable[iter] + 1; //类型从0开始
					}
					iter++;
				}
			}
		}
	}
	for (int i = 0; i < 3; i++) {
		if (flagg[i] == 0 && m_TargetTable[i + 3] > 0) m_TargetTable[i + 3] = -100;
	}
}

void RobotManager::SetTargetTable_3(WorkTable * tables, Robot * robots) { //从全局的角度设定应该买1，2，3(最少的那个）
	int flagg[5] = { 0 };
	std::set<int> max_st;
	this->m_TargetTable.clear();
	for (int i = 0; i < this->m_TableNum; i++) {
		if (tables[i].GetType() >= 3 && tables[i].GetType() <= 5) {
			int temp = tables[i].GetType();
			if (tables[i].HaveProduct()) m_TargetTable[temp] = m_TargetTable[temp] + 1;
			if (tables[i].CountDownFrameGet() >= 0) m_TargetTable[temp] = m_TargetTable[temp] + 1;
		}
		else if (tables[i].GetType() == 6) {
			int iter = 0;
			int temp = tables[i].GetMaterialState();
			while (true) {
				temp = temp >> 1;
				if (temp <= 0) break;
				if (temp % 2) {
					m_TargetTable[iter] = m_TargetTable[iter] + 1;
				}
				iter++;
			}
		}
	}
	for (int i = 0; i < 4; i++) {
		if (robots[i].GetGoods() >= 4 && robots[i].GetGoods() <= 6) {
			int temp = robots[i].GetGoods() - 1;
			m_TargetTable[temp] = m_TargetTable[temp] + 1;
		}
	}
	for (int i = 0; i < 3; i++) {
		max_st.insert(m_TargetTable[i + 3]);
	}
	int max_TT = *(--max_st.end());
	int min_TT = *max_st.begin();
	int gap = max_TT - min_TT;
	for (int i = 0; i < 3; i++) {
		m_TargetTable[i + 3] = max_TT - m_TargetTable[i + 3];
	}
	for (int i = 0; i < this->m_TableNum; i++) {
		if (tables[i].GetType() >= 3 && tables[i].GetType() <= 5 && m_TargetTable[tables[i].GetType()] == gap) {
			if ((tables[i].GetMaterialState() | this->m_SellTable[i]) > 0) {
				flagg[tables[i].GetType() - 3] = 1;
				int temp = this->m_TypeTable[i] - (tables[i].GetMaterialState() | this->m_SellTable[i]);
				int iter = 0;
				while (true) {
					temp = temp >> 1;
					if (temp <= 0) break;
					if (temp % 2) {
						m_TargetTable[iter] = m_TargetTable[iter] + 1; //类型从0开始
					}
					iter++;
				}
			}
		}
	}
	for (int i = 0; i < 3; i++) {
		if (flagg[i] == 0 && m_TargetTable[i + 3] == gap) m_TargetTable[i + 3] = -100;
	}
}


float distance1(const pair<float, float> & pos1, const pair<float, float> & pos2) {
	return sqrt(pow(pos1.first - pos2.first, 2) + pow(pos1.second - pos2.second, 2));
}

bool RobotManager::CanSellTo9(int num, WorkTable * tables, Robot * robots, int * goal) {
	int ID = 0;
	pair<float, float> pos1 = robots[num].GetPos();
	pair<float, float> pos2 = tables[ID].GetPos();
	float time = distance1(pos1, pos2) / 6;
	int flag = 0;
	for (int i = 0; i < 4; i++) {
		if (i != num && robots[i].GetTask()[0] == ID || goal[i] == ID) {
			pair<float, float> pos_1 = robots[i].GetPos();
			pair<float, float> pos_2 = tables[ID].GetPos();
			float time_1 = distance1(pos_1, pos_2) / 6;
			if (abs(time - time_1) < 1) {
				flag = 1;
				break;
			}
		}
	}
	if (flag == 1) return false;
	else return true;
}

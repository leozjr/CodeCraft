#pragma once
#include"Robot.h"
#include"WorkTable.h"
#include <set>
#include <vector>
#include <map>
using namespace std;
class RobotManager
{
	bool m_TableFlagBuy[52];
	bool m_TableFlagSell[52][10];
	bool SevenFlag = false;
	std::multiset<int> m_ChooseTable;
	std::vector<std::multiset<int> > m_ChooseTable2;
	multiset<int> mt0;
	multiset<int> mt1;
	multiset<int> mt2;
	multiset<int> mt3;
	std::map<int,int> m_TargetTable;
	std::vector<int> m_TypeTable;
	int m_TableNum;
	int m_SellTable[52] = { 0 };

public:
	void SingleManager(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road, int frameID); //单机决策
	void SingleManager_2(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road, int frameID);
	void SingleManager_3(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road, int frameID);
	void SingleManager_4(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road,int frameID);
	void SingleManager_5(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road,int frameID);
	void SingleManager_6(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road,int frameID);
	void SingleManager_7(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road,int frameID);
	void SingleManager_8(Robot * robots, WorkTable * tables, vector<vector<vector<pair<float, float> > > > & Road,int frameID);

	void GiveTask(Robot * robots, int * goalTable, vector<vector<vector<pair<float, float>>>>& Road);
	void SetSevenFlag(bool sf);
	void InitialTypeTable(std::vector<int> tt);
	void SetChooseTable_5(WorkTable * tables, set<int>& ID_0, set<int>& ID_1, set<int>& ID_2, set<int>& ID_3);
	void SetChooseTable_4(WorkTable * tables, Robot * robots);
	void SetChooseTable_3(WorkTable * tables);
	void SetChooseTable_2(WorkTable * tables, Robot * robots);
	void SetChooseTable(WorkTable* tables);
	void SetTargetTable(WorkTable * tables, Robot * robots);
	void SetTargetTable_2(WorkTable * tables, Robot * robots);
	void SetTargetTable_3(WorkTable * tables, Robot * robots);
	bool CanSellTo9(int num, WorkTable * tables, Robot * robots, int * goal);
};

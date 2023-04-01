#pragma once
#include <vector>
#include <map>
#include "Robot.h"
#include "WorkTable.h"
using namespace std;

struct A_Point
{
	pair<float, float> pos;
	float f = 0; //总代价
	float g = 0; //精确
	float h = 0;//预估
	A_Point * father;

	A_Point(pair<float, float> new_pos) {
		this->pos = new_pos;
	}
};
class IO
{
	float m_gap = 0.5; //格子间距
	vector<A_Point *> open;
	vector<A_Point *> close;
	int CanGo_Map[102][102];

public:
	int InListFlag(vector<A_Point*>& lt, pair<float, float> pos);
	float G_Calculate(A_Point * ap, pair<float, float> pos);
	float H_Calculate(pair <float, float> end, pair<float, float> pos);
	bool Cut(int & Type_1, int & Type_2);
	void CanGo(int Map[102][102]);
	void A_Star(Robot * robot, WorkTable * table, float n_max, float m_max, int Map[102][102], vector<pair<float, float> > & InitialRobotPos, vector<pair<float, float> > & InitialWorkTablePos, vector<vector<vector<pair<float, float> > > > & Road, map<pair<int, int>, float> & Distance);

	void Initialization(Robot * robot, WorkTable * table, vector<vector<int>>& tableID_by_type, int & WorkTableNum, vector<vector<vector<pair<float, float> > > > & Road, set<pair<float, pair<int, int> > > * DistanceOrder, set<pair<float, pair<int, int> > > * DistanceOrder_robot);
	
	float distance(const pair<float, float> & pos1, const pair<float, float> & pos2);
	void TableIDByType(char type, int id, vector<vector<int> > & tableID_by_type);

	int readUntilOK(int & frameID, int & money, int & WorkTableNum, Robot * robot, WorkTable * table, vector<pair<float, int> > * DistanceOrder_between_robot);
	void writeUntilOK(int & frameID, vector<pair<string, pair<int, float> > > & Order_1, vector<pair<string, int>>& Order_2);
};

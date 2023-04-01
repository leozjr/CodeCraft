#include <iostream>
#include <sstream>
#include <vector>
#include <set>
#include <cmath>
#include <cstring>
#include<Windows.h>
#include <chrono>
#include"Robot.h"
#include"WorkTable.h"
#include"IO.h"
#include"MotionControl.h"
using namespace std;

Robot robot[5];
WorkTable table[52];
IO io;
MotionControl mc;

int frameID;
int money;
int WorkTableNum;
vector<vector<int> > tableID_by_type(9); //��ά���飬һ����һ��id�������0��������1��id����1��������2��id
vector<pair<string, pair<int, float> > > Order_1; //ǰ������ת����
vector<pair<string,int> > Order_2; //��������������
vector<pair<float, int> > DistanceOrder_between_robot[5]; //�������໥���룬ǰһ��Ϊ���룬��һ��Ϊid
set<pair<float, pair<int, int> > > DistanceOrder[52];
set<pair<float, pair<int, int> > > DistanceOrder_robot[5];
vector<vector<vector<pair<float, float> > > > Road(60, vector<vector<pair<float, float> > >(55));

int main() {
	std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
	io.Initialization(robot, table, tableID_by_type, WorkTableNum, Road, DistanceOrder, DistanceOrder_robot);
	std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
	double elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
	std::cerr << "Elapsed time: " << elapsed_time << " seconds" << std::endl;

	//Sleep(8000);
	
	robot[0].SetRoad({ Road[0][12] });
	while (!cin.eof()) {
		int mapID = io.readUntilOK(frameID, money, WorkTableNum, robot, table, DistanceOrder_between_robot);
		//rm.SingleManager_4(robot, table, frameID);
		mc.MakeOrder(robot, Order_1, Order_2);
		io.writeUntilOK(frameID, Order_1, Order_2);
    }
    return 0;
}
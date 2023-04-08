#include <iostream>
#include <sstream>
#include <vector>
#include <set>
#include <cmath>
#include <cstring>
#include<Windows.h>
#include<chrono>
#include"Robot.h"
#include"WorkTable.h"
#include"IO.h"
#include"MotionControl.h"
#include"RobotManager.h"
using namespace std;

Robot robot[5];
WorkTable table[52];
IO io;
MotionControl mc;
RobotManager rm;
CongestionControl cc(&io);

int frameID;
int money;
int WorkTableNum;
vector<vector<int> > tableID_by_type(9); //二维数组，一行是一类id，比如第0行是所有1的id，第1行是所有2的id
vector<pair<string, pair<int, float> > > Order_1; //前进和旋转命令
vector<pair<string,int> > Order_2; //买卖和销毁命令
vector<pair<float, int> > DistanceOrder_between_robot[5]; //机器人相互距离，前一个为距离，后一个为id
set<pair<float, pair<int, int> > > DistanceOrder[52];
set<pair<float, pair<int, int> > > DistanceOrder_robot[5];
vector<vector<vector<pair<float, float> > > > Road(60, vector<vector<pair<float, float> > >(55));

int main() {
	//Sleep(9000);
	io.Initialization(rm, robot, table, tableID_by_type, WorkTableNum, Road, DistanceOrder, DistanceOrder_robot);

	while (!cin.eof()) {
		int mapID = io.readUntilOK(frameID, money, WorkTableNum, robot, table, DistanceOrder_between_robot);
		rm.SingleManager_6(robot, table, Road, frameID);
		mc.MakeOrder(robot, Order_1, Order_2, &cc);
		io.writeUntilOK(frameID, Order_1, Order_2);
    }
    return 0;
}
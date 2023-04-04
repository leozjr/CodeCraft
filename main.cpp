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
	Sleep(8000);
	io.Initialization(rm, robot, table, tableID_by_type, WorkTableNum, Road, DistanceOrder, DistanceOrder_robot);

	/*vector<vector<vector<pair<float, float>>>> r1_task = { { Road[1][3], Road[7][10] }, { Road[14][3], Road[7][10] } };
	vector<vector<vector<pair<float, float>>>> r2_task = { { Road[2][4], Road[8][8] } };

	vector<vector<int>> r1_buysell = { {10, 3} ,{10,3} };
	vector<vector<int>> r2_buysell = { {8, 4} };

	robot[0].SetRoad({ Road[0][0], Road[4][5] });
	robot[0].SetTask({5, 0});

	robot[1].SetRoad(r1_task[0]);
	robot[1].SetTask(r1_buysell[0]);
	robot[2].SetRoad(r2_task[0]);
	robot[2].SetTask(r2_buysell[0]);*/

	while (!cin.eof()) {
		int mapID = io.readUntilOK(frameID, money, WorkTableNum, robot, table, DistanceOrder_between_robot);
		rm.SingleManager_6(robot, table, Road, frameID);
		//robot[1].BuySellCheck(1, table);
		//robot[2].BuySellCheck(1, table);

		/*if (!robot[1].isBusy())
		{
			robot[1].SetRoad(r1_task[1]);
			robot[1].SetTask(r1_buysell[1]);
		}*/

		//robot[0].Stop();
		mc.MakeOrder(robot, Order_1, Order_2, &cc);
		io.writeUntilOK(frameID, Order_1, Order_2);
    }
    return 0;
}
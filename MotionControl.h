#pragma once
#include<array>
#include<set>
#include<vector>
#include<string>
#include"IO.h"
#include"Robot.h"
#include"CongestionControl.h"
#define _USE_MATH_DEFINES
#include<math.h>

class MotionControl
{
	int m_RobotsNum = 4;

	float m_CollisionCheckVFactor = 0.8; //碰撞检测 速度加权系数，速度越大，应当有更大的碰撞预警范围
	float m_MinLimitDistace = 2; //最小碰撞检测距离
	float m_CollisionTurnPower = 3; //避障转向力度。最大M_PI
	bool m_TurnRight; //避障向左还是向右

	float m_CanGoAngle = M_PI/6; //允许前进的角度偏离值，即如果与当前target角度大于这个角，会速度减为0转弯，直到小于该角度才会出发
	float m_TrackDistance = 1; //循迹pop距离，1

	int m_RollBackTime = 0;

	CongestionControl* cc;
	void WhoNeedAvoidance(Robot* robots);
	void TrackAvoidanceBack(int my_id, int first_go_id, Robot* robots);

	//导航函数，计算四个机器人的速度矩阵，第一列为线速度，第二列为角速度
	void Navigation(Robot* robots);

	//VO碰撞检测
	bool CollisionCheck(Robot* robots, int my_id);
	//是否处于VO区域的判定
	bool InVO(std::pair<std::pair<float, float>, std::pair<float, float>> vo, std::pair<float, float> relative_v, std::pair<float, float> relative_pos);
	//是否有人闯入碰撞范围的检查
	std::vector<int> CollisionDetection(Robot& robot);
	
	//循迹
	void TrackRoad(Robot& robot);
	std::pair<float, float> CalTrackRoadV(Robot& robot, std::pair<float, float> target_point);
	std::pair<float, float> NearWallSolver(pair<float, float> target_point); //将靠墙的点挪0.25的距离
	void KeepDistance(int my_id, Robot* others);
	void JumpPointSolver(Robot& r); //解决机器人提前出现在未来的点的情况，即跳点求解器

	//计算角度差
	float AngleDiff(std::pair<float, float> target_pos, std::pair<float, float> robot_pos, float robot_dir);
	//给出向量坐标计算角度差
	float AngleDiff(std::pair<float, float> relative_pos, std::pair<float, float> relative_v);
	//给出方向，计算角度差
	float AngleDiff(float dir1, float dir2);
	
	bool LagCheck(Robot& r, Robot* others);
	void LagSolver(Robot& r, Robot* others);

	// 输出角速度计算
	float AnglePDcontrol(float angle_diff);
	// 输出线速度计算
	float LinearPDcontrol(Robot& r, float angle);

public:
	// 根据速度矩阵制作Order1格式的输出命令
	void MakeOrder(Robot* robots, std::vector<std::pair<std::string, std::pair<int, float> > >& Order_1, std::vector<std::pair<std::string, int> >& Order_2, CongestionControl* cc);

};


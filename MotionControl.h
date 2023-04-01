#pragma once
#include<array>
#include<set>
#include<vector>
#include<string>
#include"Robot.h"
class MotionControl
{
	int m_RobotsNum = 1;
	float m_MaxSpeed = 6;
	float m_WallSpeed = 1; //墙边目标限速
	float m_WallSlowDownDis = 1; //提前减速距离

	float m_CollisionCheckVFactor = 0.8; //碰撞检测 速度加权系数，速度越大，应当有更大的碰撞预警范围
	float m_MinLimitDistace = 2; //最小碰撞检测距离
	float m_CollisionTurnPower = 3; //避障转向力度。最大M_PI
	bool m_TurnRight; //避障向左还是向右

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

	//计算航向差
	float AngleDiff(std::pair<float, float> target_pos, std::pair<float, float> robot_pos, float robot_dir);
	//角度差通用版本
	float AngleDiff(std::pair<float, float> relative_pos, std::pair<float, float> relative_v);
	
	//撞墙检查
	bool HitWallCheck(std::pair<float, float> target_pos, std::pair<float, float> my_pos);

	// 输出角速度计算
	float AnglePDcontrol(float angle_diff);
	// 输出线速度计算
	float LinearPDcontrol(std::pair<float, float> target_pos, std::pair<float, float> my_pos, float angle);

public:
	// 根据速度矩阵制作Order1格式的输出命令
	void MakeOrder(Robot* robots, std::vector<std::pair<std::string, std::pair<int, float> > >& Order_1, std::vector<std::pair<std::string, int> >& Order_2);

};


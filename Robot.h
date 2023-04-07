#pragma once
#include<set>
#include <cmath>
#include<vector>
#include<deque>
#include<array>
#include"WorkTable.h"
class Robot
{
	// 判题器输入参数
	int m_ID; //机器人id
	int m_WorkTableID; //所在工作台id
	int m_Goods;//所背货物
	float m_TimeValueFactor; // 时间损耗系数
	float m_CollisionValueFactor; // 碰撞损耗系数
	float m_AngularVelocity; //角速度
	std::pair<float, float> m_LinearVelocity; //线速度
	float m_Direction; //方向
	std::pair<float, float> m_Pos; //位置

	// 新增参数
	bool m_Busy = false; //忙标志位
	bool m_Stop = false; //停标志位
	bool m_SuccTrans = false; //交易成功标志位
	int m_Wait = 0; //买卖分离决策需要

	bool m_PriorityPass; //优先同行权
	bool m_Avoidance = false; //避让状态标志位
	int m_AvoidID; //避让对象的ID是谁
	bool m_CanPark = false; //搜索到停车位的标志位

	bool m_PrecisionControl = false; //精准控制标志位
	float m_PrecisionSpeed = 3;

	bool m_EarlyBrake = false; //到达工作台前提前刹车
	float m_MaxSpeed = 6; //该机器人的最大速度（初始为6）
	float m_ReachSpeed = 1; //墙边目标限速
	float m_BrakeDistance = 1; //提前减速距离

	int m_BuyOrSell = 0; //0：不买也不卖，1：卖， -1：买， 2:销毁
	std::vector<int> m_Task; //上哪儿卖，从哪儿买。0:买家id, 1：卖家id；
	std::vector<std::pair<float, int>> m_RobotsDistance; //机器人间距离
	std::set<std::pair<float, std::pair<int, int> > > m_DistanceOrder_robot; //初始机器人和工作台之间的距离

	void Buy(); //买货指令， 只给BuySellCheck()用
	void Sell(); //卖货指令，只给BuySellCheck()用
	void Destory(); //销毁货物指令，只给BuySellCheck()用
	bool Reached(); //查询是否到达目标地点， 只给BuySellCheck()用

	void EarlyBrake(std::pair<float, float> target_pos, std::pair<float, float> my_pos); // 提前刹车控制

	std::pair<float, float> m_NextV; //要输出的线速度和角速度，<linear_v, angle_v>
public:
	std::vector<std::vector<std::pair<float, float> >> m_FutureRoad; //没走的路
	std::vector<std::pair<float, float> > m_PastRoad; //走过的路，用来回退用

	std::vector<std::pair<float, float> > m_ParkFutureRoad; //前往避让点的没走到路
	std::vector<std::pair<float, float> > m_ParkPastRoad; //避让中的走过的路
	bool EndPark = false; //停止泊车，可以返回
	bool AllowJump = true; //允许跳点
	
	std::pair<float, float> EndPriorityPass; //结束优先通行权标志
	int EndPriorityNumber; //经过两个搜到的地方，就结束优先通行
	int FullyCanGo; //完全可以通行的标志位，别人需要进Leave_Point点范围，然后再出点范围，才能走
	

	/*-------查询函数------*/
	int GetID(); //ID
	std::pair<float, float> GetPos(); //位置
	float GetDir(); //方向
	std::pair<float, float> GetV(); //速度
	int GetGoods(); // 携带货物状态 0为未携带，1234567为正在携带对应类型的货物
	std::pair<float, float> GetNextV();
	bool GetTransState(); //获取交易状态
	bool GetAvoidance(); //获取避让状态
	bool GetPriorityPass();
	int GetAvoidID();
	std::vector<int> GetTask(); //任务状态
	int GetBuyorSell(); //查询买卖指令
	std::vector<std::pair<float, int>> GetRobotsDistance(); //查询机器人间距离
	bool CanPark();
	bool GetPrecisionControl();
	bool GetEarlyBrake(); //查询提前刹车
	float GetReachSpeed(); //查询到达时限速距离
	float GetMaxSpeed();
	bool isBusy(); //是否忙碌
	
	//主函数调用
	void BuySellCheck(int type, WorkTable* wts); //任务的买卖检查，到目标地点后是否需要买卖【type：1是买卖绑定；2是买卖分离】

	/*------IO类用的------*/
	void Init(int id); //初始化id
	bool UpdateInfo(int wt, int goods, float tvf, float cvf, float av, std::pair<float, float> lv, float dir, std::pair<float, float> pos);
	void DistanceSet(std::vector<std::pair<float, int>>& ds_between_robot);//更新机器人间距离
	void DistanceSet(std::set<std::pair<float, std::pair<int, int> > > & ds); //初始机器人与工作台距离


	/*------给rm用的-------*/
	void SetWait(int sw); //买卖分离决策需要
	void Stop(); //停止命令，主要给rm清货时调用
	void SetTask(std::vector<int> task); //设置任务，接收rm给出的task
	void SetRoad(std::vector<std::vector<std::pair<float, float> > > road); //设置路径
	void ClearTask(); //清空任务，可能用在清货阶段
	std::set<std::pair<float, std::pair<int, int> > > DistanceGet(); //得到初始机器人与工作台距离
	
	/*-----给mc用的-----*/
	void SetPrecisionControl(bool flag); //精准控制模式，速度为4
	void SetPriorityPass(bool flag); // 设置优先通行权
	void SetNextV(std::pair<float, float> next_v);
	void SetAvoidance(bool flag, int avoid_id); //设置机器人进入或退出避让状态
	void SetCanPark(bool flag);
	bool NeedStop(); // 是否发出停指令
	
};


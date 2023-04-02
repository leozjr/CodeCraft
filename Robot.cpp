#include<algorithm>
#include <set>
#include<vector>
#include<deque>
#include "Robot.h"

void Robot::Init(int id)
{
	this->m_ID = id;
}

void Robot::Stop()
{
	this->m_Stop = true;
}

void Robot::SetNextV(std::pair<float, float> next_v)
{
	this->m_NextV = next_v;
}

bool Robot::NeedStop()
{
	return this->m_Stop;
}

void Robot::SetTask(std::vector<int> task)
{
	this->m_Task = task;
}

void Robot::SetRoad(std::vector<std::vector<std::pair<float, float> > > road)
{
	this->m_Road.clear();
	//逆序，为了mc中好pop_back
	for (auto &i : road)
		std::reverse(i.begin(), i.end());
	this->m_Road = road;
}

void Robot::ClearTask()
{
	this->m_Task.clear();
	this->m_Busy = false;
}

std::set<std::pair<float, std::pair<int, int> > > Robot::DistanceGet()
{
	return this->m_DistanceOrder_robot;
}

void Robot::BuySellCheck(int type, WorkTable* wts)
{
	//检查是否需要买卖
	if (type == 1) { //买卖绑定
		if (!this->m_Task.empty())
		{
			this->m_Busy = true;//我忙了,因为我任务表没完成
			if (this->m_Task.size() == 2 && this->Reached())
			{
				//如果任务长度为2，而且到达，则正处于卖家地址
				if (wts[this->m_Task.back()].HaveProduct())
				{
					this->Buy();
					this->m_SuccTrans = true; // 成功交易标志位，给mc判断是否继续前进用的
					this->m_Task.pop_back();//卖家地址已到达，划掉。
				}
			}
			else if (this->m_Task.size() == 1 && this->Reached())
			{
				//如果任务长度为1，而且到达，则正处于买家地址
				this->Sell();
				if (this->GetGoods() == 0)
				{
					this->m_Task.pop_back();//买家地址已到达，划掉。
					this->m_SuccTrans = true; // 成功交易标志位，给mc判断是否继续前进用的
					this->m_Busy = false;// task列表此时为空，任务完成，Busy标志解除
				}
			}
		}
	}
	else if (type == 2) {
		if (!this->m_Task.empty())
		{
			this->m_Busy = true;//我忙了,因为我任务表没完成
			if (this->GetGoods() == 0 && this->Reached()) { //没货物，准备去买
				this->Buy();
				this->m_Busy = false;
			}
			else if (this->GetGoods() > 0 && this->Reached()){ //有货物，准备去卖
				this->Sell();
				this->m_Busy = false;
			}
		}

	}
}

int Robot::GetGoods()
{
	return this->m_Goods;
}

std::pair<float, float> Robot::GetNextV()
{
	return this->m_NextV;
}

bool Robot::GetTransState()
{
	return this->m_SuccTrans;
}

std::vector<int> Robot::GetTask()
{
	return this->m_Task;
}

bool Robot::Reached()
{
	return (this->m_WorkTableID == this->m_Task.back());
}

int Robot::GetID()
{
	return this->m_ID;
}

int Robot::GetBuyorSell()
{
	return this->m_BuyOrSell;
}

void Robot::Buy()
{
	this->m_BuyOrSell = -1;
}

void Robot::Sell()
{
	this->m_BuyOrSell = 1;
}

void Robot::Destory()
{
	this->m_BuyOrSell = 2;
}

std::pair<float, float> Robot::GetPos()
{
	return this->m_Pos;
}

float Robot::GetDir()
{
	return this->m_Direction;
}

std::pair<float, float>  Robot::GetV()
{
	return this->m_LinearVelocity;
}

void Robot::DistanceSet(std::vector<std::pair<float, int> > & ds_between_robot)
{
	this->m_RobotsDistance = ds_between_robot;
}

void Robot::DistanceSet(std::set<std::pair<float, std::pair<int, int> > > & ds)
{
	this->m_DistanceOrder_robot = ds;
}

std::vector<std::pair<float, int> > Robot::GetRobotsDistance()
{
	return this->m_RobotsDistance;
}


bool Robot::UpdateInfo(int wt, int goods, float tvf, float cvf, float av, std::pair<float, float> lv, float dir, std::pair<float, float> pos)
{
	this->m_WorkTableID = wt;
	this->m_Goods = goods;
	this->m_TimeValueFactor = tvf;
	this->m_CollisionValueFactor = cvf;
	this->m_AngularVelocity = av;
	this->m_LinearVelocity = lv;
	this->m_Direction = dir;
	this->m_Pos = pos;
	this->m_BuyOrSell = 0;
	this->m_SuccTrans = false;
	this->m_Stop = false;
	return true;
}

bool Robot::isBusy()
{
	return this->m_Busy;
}



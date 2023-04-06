#include<algorithm>
#include <set>
#include<vector>
#include<deque>
#include "Robot.h"

// �������
float TableDistance(std::pair<float, float> target_pos, std::pair<float, float> robot_pos)
{
	return sqrt(pow(target_pos.first - robot_pos.first, 2) + pow(target_pos.second - robot_pos.second, 2));
}

void Robot::Init(int id)
{
	this->m_ID = id;
}

void Robot::Stop()
{
	this->m_Stop = true;
}

void Robot::SetPrecisionControl(bool flag)
{
	if (flag)
	{
		this->m_PrecisionControl = false;
		this->m_MaxSpeed = 6;
	}
	else
	{
		this->m_PrecisionControl = true;
		this->m_MaxSpeed = this->m_PrecisionSpeed;
	}

}

void Robot::SetPriorityPass(bool flag)
{
	this->m_PriorityPass = flag;
}

void Robot::SetNextV(std::pair<float, float> next_v)
{
	this->m_NextV = next_v;
}

void Robot::SetAvoidance(bool flag, int avoid_id)
{
	this->m_Avoidance = flag;
	this->m_AvoidID = avoid_id;
}
void Robot::SetCanPark(bool flag)
{
	this->m_CanPark = flag;
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
	this->m_FutureRoad.clear();
	//����Ϊ��mc�к�pop_back
	for (auto &i : road)
		std::reverse(i.begin(), i.end());
	this->m_FutureRoad = road;
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
	//����Ƿ���Ҫ����
	if (type == 1) { //������
		if (!this->m_Task.empty())
		{
			// ����ǰ���ټ��
			this->EarlyBrake(wts[this->m_Task.back()].GetPos(), this->m_Pos);

			this->m_Busy = true;//��æ��,��Ϊ�������û���
			if (this->m_Task.size() == 2 && this->Reached())
			{
				//������񳤶�Ϊ2�����ҵ�������������ҵ�ַ
				if (wts[this->m_Task.back()].HaveProduct())
				{
					this->Buy();
					this->m_SuccTrans = true; // �ɹ����ױ�־λ����mc�ж��Ƿ����ǰ���õ�
					this->m_Task.pop_back();//���ҵ�ַ�ѵ��������
				}
			}
			else if (this->m_Task.size() == 1 && this->Reached())
			{
				//������񳤶�Ϊ1�����ҵ������������ҵ�ַ
				this->Sell();
				if (this->GetGoods() == 0)
				{
					this->m_Task.pop_back();//��ҵ�ַ�ѵ��������
					this->m_SuccTrans = true; // �ɹ����ױ�־λ����mc�ж��Ƿ����ǰ���õ�
					this->m_Busy = false;// task�б��ʱΪ�գ�������ɣ�Busy��־���
				}
			}
		}
	}
	else if (type == 2) {
		if (this->m_Wait != 0) { //�����ͣ���ȴ��µ�����
			this->m_Busy = false;
			if (this->m_Wait == 1) {
				this->Sell();
			}
			else if (this->m_Wait == -1) {
				this->Buy();
			}
		}
		else if (!this->m_Task.empty())
		{
			// ����ǰ���ټ��
			this->EarlyBrake(wts[this->m_Task.back()].GetPos(), this->m_Pos);

			this->m_Busy = true;//��æ��,��Ϊ�������û���
			if (this->GetGoods() == 0 && this->Reached() && this->m_FutureRoad.size() == 1) { //û���׼��ȥ��
				if (wts[this->m_Task.back()].HaveProduct())
				{
					this->Buy();
					//this->m_SuccTrans = true; // �ɹ����ױ�־λ����mc�ж��Ƿ����ǰ���õ�
					this->m_Busy = false;
				}
			}
			else if (this->GetGoods() > 0 && this->Reached() && this->m_FutureRoad.size() == 1) { //�л��׼��ȥ��
				this->Sell();
				//this->m_SuccTrans = true; // �ɹ����ױ�־λ����mc�ж��Ƿ����ǰ���õ�
				this->m_Busy = false;// task�б��ʱΪ�գ�������ɣ�Busy��־���
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

bool Robot::GetAvoidance()
{
	return this->m_Avoidance;
}

bool Robot::GetPriorityPass()
{
	return this->m_PriorityPass;
}

int Robot::GetAvoidID()
{
	return this->m_AvoidID;
}

std::vector<int> Robot::GetTask()
{
	return this->m_Task;
}

bool Robot::Reached()
{
	return (this->m_WorkTableID == this->m_Task.back());
}

//���﹤��̨ǰ��ǰ����
void Robot::EarlyBrake(std::pair<float, float> target_pos, std::pair<float, float> my_pos)
{
	float dis = TableDistance(target_pos, my_pos);
	if (dis < this->m_BrakeDistance)
		this->m_EarlyBrake = true;
	else
		this->m_EarlyBrake = false;

	//float stop_dis = 2;
	//if (target_pos.first < stop_dis || target_pos.first > 50 - stop_dis || target_pos.second < stop_dis || target_pos.second > 50 - stop_dis)
	//{
	//	float dis = TargetDistance(target_pos, my_pos);
	//	if (dis < this->m_BrakeDistance)
	//		this->m_EarlyBrake = true;
	//	else
	//		this->m_EarlyBrake = false;
	//}
	//else
	//	this->m_EarlyBrake = false;
}

void Robot::SetWait(int sw)
{
	this->m_Wait = sw;
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

bool Robot::CanPark()
{
	return this->m_CanPark;
}

bool Robot::GetPrecisionControl()
{
	return this->m_PrecisionControl;
}

bool Robot::GetEarlyBrake()
{
	return this->m_EarlyBrake;
}

float Robot::GetReachSpeed()
{
	return this->m_ReachSpeed;
}

float Robot::GetMaxSpeed()
{
	return this->m_MaxSpeed;
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



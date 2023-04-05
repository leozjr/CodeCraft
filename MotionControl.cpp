#include<set>
#include<vector>
#define _USE_MATH_DEFINES
#include<math.h>
#include<algorithm>
#include <cstdlib>
#include <ctime>
#include "MotionControl.h"
#include"Robot.h"
#include"CongestionControl.h"
using namespace std;

// ��������ת�������
pair<float, float> RelativePair(pair<float, float> p1, pair<float, float> p2)
{
	return make_pair(p1.first - p2.first, p1.second - p2.second);
}

// �������
float TargetDistance(std::pair<float, float> target_pos, std::pair<float, float> robot_pos)
{
	return sqrt(pow(target_pos.first - robot_pos.first, 2) + pow(target_pos.second - robot_pos.second, 2));
}

//����VO������˳��Ϊ��߽��������ұ߽�����
pair<pair<float, float>, pair<float, float>> CalculateVO(pair<float, float> pos, float r)
{
	float dis = sqrt(pow(pos.first, 2) + pow(pos.second, 2)); //��Ծ���
	float dis_2 = pow(dis, 2);
	float leg = sqrt(pow(dis, 2) - pow(r, 2)); //�߽������ĳ��ȣ���Բ���ߵ�ԭ�㳤�ȣ�
	
	//pos������ת����߽�ĵ�λ���� x' = x cos - y sin, y' = x sin + y cos
	pair<float, float> left_VO = make_pair((pos.first * leg - pos.second * r)/dis_2, (pos.first * r + pos.second * leg)/ dis_2);
	//pos������ת���ұ߽�ĵ�λ���� x' = x cos + y sin, y' = y cos - x sin
	pair<float, float> right_VO = make_pair((pos.first * leg + pos.second * r) / dis_2, (pos.second * leg - pos.first * r) / dis_2);

	return make_pair(left_VO, right_VO);
}

// ����VO�㷨�е���԰뾶
float CalculateR(int mygoods, int yourgoods)
{
	float mr, yr;
	float no_goods = 0.55;
	float has_goods = 0.63;
	if (mygoods == 0)
		mr = no_goods;
	else
		mr = has_goods; 
	if (yourgoods == 0)
		yr = no_goods;
	else
		yr = has_goods;
	return mr + yr; //��԰뾶
}

bool MotionControl::InVO(pair<pair<float, float>, pair<float, float>> vo, pair<float, float> relative_v, pair<float, float> relative_pos)
{
	float right_vo_angle = atan2(vo.second.second, vo.second.first);
	float left_vo_angle = atan2(vo.first.second, vo.first.first);
	float middle_vo_angle = atan2(relative_pos.second, relative_pos.first);
	float relative_v_angle = atan2(relative_v.second, relative_v.first);
	
	// �����X�Ḻ������һ�����������Ҫ�Ƚ����
	if (right_vo_angle > 0 && left_vo_angle < 0)
	{
		if (relative_v_angle > right_vo_angle || relative_v_angle < left_vo_angle)
		{
			//�������������ת
			if (middle_vo_angle < 0 && relative_v_angle>0)
			{
				this->m_TurnRight = true;
			}
			else if (middle_vo_angle > 0 && relative_v_angle < 0)
			{
				this->m_TurnRight = false;
			}
			else if (relative_v_angle<middle_vo_angle)
			{
				this->m_TurnRight = true;
			}
			else
			{
				this->m_TurnRight = false;
			}
			return true;
		}
	}
	//����Խ��X�ᣬ�����ж�
	if (relative_v_angle > right_vo_angle && relative_v_angle < left_vo_angle)
	{
		//����ת���ж�
		if (relative_v_angle < middle_vo_angle)
		{
			this->m_TurnRight = true;
		}
		else
		{
			this->m_TurnRight = false;
		}
		return true;
	}
	else
		return false;
}

//ͬ���ж�
bool SameDir(Robot& r1, Robot& r2)
{
	float angle_diff = abs(r1.GetDir() - r2.GetDir());
	float angle_diff_sup = abs(2 * M_PI - angle_diff);
	float min_angle_diff = (angle_diff_sup >= angle_diff) ? angle_diff : angle_diff_sup;

	return (min_angle_diff < 0.34);//20�ȴ���ͬ��
}

void MotionControl::WhoNeedAvoidance(Robot* robots)
{
	for (int i = 0; i < this->m_RobotsNum; i++)
	{
		if (robots[i].GetTask().empty()) continue;
		//����·���ȥ·�غϵ��ˣ���������ͨ��Ȩ
		if (!robots[i].GetPriorityPass())
		{
			int num = this->cc->Congestion(robots[i].m_FutureRoad[0], i, robots[i].m_PastRoad, i, 2);
			if (num > 0)
			{
				robots[i].EndPriorityPass = robots[i].m_FutureRoad[0][robots[i].m_FutureRoad[0].size() - num];
				robots[i].SetPriorityPass(true);
			}
		}
	}

	//������еĻ����ˣ�˭��Ҫ����
	for (int i = 0; i < this->m_RobotsNum; i++)
	{
		for (int j = i+1; j < this->m_RobotsNum; j++)
		{
			if (robots[i].GetTask().empty() || robots[j].GetTask().empty()) continue;

			if (!((robots[i].GetAvoidance() && robots[j].GetAvoidance()) || (robots[i].GetAvoidance() && robots[i].GetAvoidID() == j) || (robots[j].GetAvoidance() && robots[j].GetAvoidID() == i)))
			{

				bool temp = this->cc->Congestion(robots[i].m_FutureRoad[0], i, robots[j].m_FutureRoad[0], j, 1);
				if (temp)
				{
					
					this->cc->path1 = robots[i].m_FutureRoad[0];
					this->cc->path2 = robots[j].m_FutureRoad[0];

					//���ڳ�ͻ���������ȼ�ȷ��˭��Ҫ����
					if (robots[i].GetAvoidance()) {
						if (robots[j].GetPriorityPass()) {
							robots[j].Stop();
						}
						else {
							robots[j].SetAvoidance(true);
							robots[j].SetAvoidID(i);
						}
					}
					else if (robots[j].GetAvoidance()) {
						if (robots[i].GetPriorityPass()) {
							robots[i].Stop();
						}
						else {
							robots[i].SetAvoidance(true);
							robots[i].SetAvoidID(j);
						}
					}
					else if (robots[i].GetPriorityPass()) {
						robots[j].SetAvoidance(true);
						robots[j].SetAvoidID(i);
					}
					else if (robots[j].GetPriorityPass()) {
						robots[i].SetAvoidance(true);
						robots[i].SetAvoidID(j);
					}
					else if (robots[i].GetGoods() <= robots[j].GetGoods())
					{
						robots[i].SetAvoidance(true);
						robots[i].SetAvoidID(j);
					}
					else
					{
						robots[j].SetAvoidance(true);
						robots[j].SetAvoidID(i);
					}
				}
			}
		}
	}
}

void MotionControl::TrackAvoidanceBack(Robot& r, Robot& r_first_go)
{
	if (!r.CanPark())
	{	
		//����ʼѭ��
		r.SetNextV(this->CalTrackRoadV(r, r.m_PastRoad.back()));

		if (TargetDistance(r.GetPos(), r.m_PastRoad.back()) < this->m_TrackDistance)
		{
			r.m_FutureRoad[0].push_back(r.m_PastRoad.back());
			r.m_PastRoad.pop_back();

			//ͬʱ��������ͣ���ĵ�
			r.m_ParkFutureRoad = this->cc->AvoidanceRoad(r.m_PastRoad.back(), r_first_go.m_FutureRoad[0], r.GetPos(), r_first_go.GetPos());
			if (!r.m_ParkFutureRoad.empty())
			{
				this->cc->m_LeaveCongestionPoint[r_first_go.GetID()] = r.m_PastRoad.back();
				//������־λΪ�棬֮��������·��
				r.SetCanPark(true);
			}
		}
	}

	if (r.CanPark() && !r.EndPark)
	{	//��ʼѭ�������㣬�ڲ�����ȴ�
		if (cc->CanGo(r_first_go.GetPos(), r_first_go.GetID()))
			r.EndPark = true;
		else if (r.m_ParkFutureRoad.size() > 1)
		{
			r.SetNextV(this->CalTrackRoadV(r, r.m_ParkFutureRoad.back()));
			if (TargetDistance(r.GetPos(), r.m_ParkFutureRoad.back()) < this->m_TrackDistance)
			{
				r.m_ParkPastRoad.push_back(r.m_ParkFutureRoad.back());
				r.m_ParkFutureRoad.pop_back();
			}
				
		}
		else if(r.m_ParkFutureRoad.size() == 1)
		{
			//�ڲ�����ȴ�
			r.SetNextV(this->CalTrackRoadV(r, r.m_ParkFutureRoad.back()));
		}
	}
	
	if (r.EndPark)
	{
		if (!r.m_ParkFutureRoad.empty())
		{
			//���ǻ�������parkFutureRoad��Ȼ��
			r.SetNextV(this->CalTrackRoadV(r, r.m_ParkFutureRoad.back()));
			if (TargetDistance(r.GetPos(), r.m_ParkFutureRoad.back()) < this->m_TrackDistance)
			{
				r.m_ParkPastRoad.push_back(r.m_ParkFutureRoad.back());
				r.m_ParkFutureRoad.pop_back();
			}
		}
		else if(r.m_ParkPastRoad.size() > 1)
		{
			r.SetNextV(this->CalTrackRoadV(r, r.m_ParkPastRoad.back()));
			if (TargetDistance(r.GetPos(), r.m_ParkPastRoad.back()) < this->m_TrackDistance)
				r.m_ParkPastRoad.pop_back();
		}
		else if (r.m_ParkPastRoad.size() == 1)
		{
			//�������״̬������ǳ���ȷ�Ļص����õ�����Խ��
			r.SetNextV(this->CalTrackRoadV(r, r.m_ParkPastRoad.back()));
			if (TargetDistance(r.GetPos(), r.m_ParkPastRoad.back()) < 0.5)
				r.m_ParkPastRoad.pop_back();
		}
		else
		{
			//����״̬���
			r.SetAvoidance(false);
			r.SetCanPark(false);
			r.EndPark = false;
			r.m_ParkFutureRoad.clear();
			r.m_ParkPastRoad.clear();
		}
	}
	
}

//�������̿���
void MotionControl::Navigation(Robot* robots)
{
	for (int i = 0; i < this->m_RobotsNum; i++)
	{
		if (robots[i].GetTask().empty()) continue;

		if (robots[i].GetAvoidance())
		{
			//�û����˴��ڱ���״̬
			TrackAvoidanceBack(robots[i], robots[robots[i].GetAvoidID()]);
		}
		/*else if (CollisionCheck(robots, i))
		{
			//VO ��ײ���
			float linear_v = 6;

			float angle_v;
			//ȫ��ת��
			if (this->m_TurnRight)
				angle_v = -1*this->m_CollisionTurnPower;
			else
				angle_v = m_CollisionTurnPower;

			robots[i].SetNextV(make_pair(linear_v, angle_v));
			
		}*/
		else
		{
			//ѭ��
			this->TrackRoad(robots[i]);
		}

	}
}

//��ײ���
bool MotionControl::CollisionCheck(Robot* robots, int my_id)
{
	bool collision = false;
	vector<int> collision_id = this->CollisionDetection(robots[my_id]);
	pair<float, float> my_v, your_v, my_pos, your_pos, relative_v, relative_pos;
	pair<pair<float, float>, pair<float, float>> vo;
	if (!collision_id.empty())
	{

		my_v = robots[my_id].GetV();
		my_pos = robots[my_id].GetPos();
		for (int m = 0; m < collision_id.size(); m++)
		{
			your_pos = robots[collision_id[m]].GetPos();
			your_v = robots[collision_id[m]].GetV();
			relative_pos = RelativePair(your_pos, my_pos);
			relative_v = RelativePair(my_v, your_v);

			//���ݻ���Я��״̬��������ͬ����԰뾶r
			float r = CalculateR(robots[my_id].GetGoods(), robots[collision_id[m]].GetGoods());
			vo = CalculateVO(relative_pos, r);

			if (this->InVO(vo, relative_v, relative_pos) && !SameDir(robots[my_id], robots[collision_id[m]]))
			{	//˵��Ҫײ��
				collision = true;
				break;//ֻҪ֪����һ��Ҫײ�˾͹���
			}
		}
	}
	return collision;
}

//������ײԤ����Χ���
vector<int> MotionControl::CollisionDetection(Robot& robot)
{
	vector<pair<float, int> > dis_robots = robot.GetRobotsDistance();
	float v = sqrt(pow(robot.GetV().first, 2)+ pow(robot.GetV().second, 2));
	vector<int> Check_list;
	for (int i = 0; i < 3; i++)
	{
		//���ȼ���Ƿ��л���������m_CollisionCheckLimit��Χ��
		if (dis_robots[i].first < this->m_MinLimitDistace + this->m_CollisionCheckVFactor * v)
		{
			Check_list.push_back(dis_robots[i].second);
		}
	}
	return Check_list;
}

//����ѭ���ٶȿ��� <���ٶȣ����ٶ�>
pair<float, float> MotionControl::CalTrackRoadV(Robot& r, pair<float, float> target_point)
{
	//���angle_diff�Ǵ������ŵ�
	float angle_diff = this->AngleDiff(target_point, r.GetPos(), r.GetDir());

	float angle_v = this->AnglePDcontrol(angle_diff);
	float linear_v = this->LinearPDcontrol(r, abs(angle_diff));
	return make_pair(linear_v, angle_v);
}

//����������������
void MotionControl::JumpPointSolver(Robot& r)
{
	//��ǰN������ȡ�����һ��
	int n = min(6, int(r.m_FutureRoad[0].size()));
	if (n == 0)
		return;

	auto min_iter = r.m_FutureRoad[0].end();
	float min_dist = std::numeric_limits<float>::max(); // ��ʼ����С����Ϊfloat�����ֵ
	for (auto iter = r.m_FutureRoad[0].end() - n; iter != r.m_FutureRoad[0].end(); ++iter) {
		float dist = TargetDistance(r.GetPos(), (*iter));
		if (dist < min_dist) {
			min_dist = dist; // ������С����
			min_iter = iter; // ������С�����Ӧ�ĵ�����
		}
	}

	while (r.m_FutureRoad[0].end()-1 != min_iter) {
		r.m_PastRoad.push_back(r.m_FutureRoad[0].back()); 
		r.m_FutureRoad[0].pop_back(); 
	}

}

//�������ڼ�� -1���ǿ��ڣ�0�ǽ��룬1�ǳ�����
int MotionControl::NearChasm(Robot& r)
{


	//pair<int, int> now_pos = make_pair((49.75 - my_pos.second) * 2, (my_pos.first - 0.25) * 2);
	return false;
}

//�ھ������ڵ�ʱ����Ҫ�������㣬���ҿ��Ƶ�����Ҫ��С
void MotionControl::CrossChasmSolver(Robot& r)
{
	if (this->NearChasm(r) == 0)
	{
		//�ս����ڣ����㣬����׼����
	}
	else
	{
		//�����ڣ��ؾ�׼����
	}
	
}


//ѭ��
void MotionControl::TrackRoad(Robot& r)
{
	if (!r.m_FutureRoad.empty())
	{
		//�������
		vector<pair<float, float>>& future_road = r.m_FutureRoad[0]; //δ���ߵ�·
		vector<pair<float, float>>& past_road = r.m_PastRoad; //�߹���·
		vector<pair<float, float>> part_past_road; // ���ֹ�ȥ·
		
		if (r.GetTransState())
		{
			future_road.clear();
			r.m_FutureRoad.erase(r.m_FutureRoad.begin());//ɾ����·��
			r.m_PastRoad.clear(); //�����ʷ��
			return;
		}
		//���յ�ǰ
		if (future_road.size() > 1)
		{
			//ѭ��

			//ƫ��·��������ļ�鼰���
			this->JumpPointSolver(r);

			//�����ڲ���

			r.SetNextV(this->CalTrackRoadV(r, future_road.back()));
			if (TargetDistance(r.GetPos(), future_road.back()) < this->m_TrackDistance)
			{
				//�ͷ�����Ȩ
				if (r.GetPriorityPass())
				{
					int idx = min(int(r.m_PastRoad.size()-1), 30);
					part_past_road.assign(r.m_PastRoad.end()-idx, r.m_PastRoad.end());
					if (future_road.back() == r.EndPriorityPass)
					{
						r.SetPriorityPass(false);
						r.EndPriorityNumber = 0;
					}
					else if (!this->cc->AvoidanceRoad(r.m_FutureRoad[0].back(), part_past_road, r.GetPos(), make_pair(100, 100)).empty())
					{
						r.EndPriorityNumber += 1;
					}
					
					if (r.EndPriorityNumber > 3)
					{
						r.SetPriorityPass(false);
						r.EndPriorityNumber = 0;
					}
					
				}
				past_road.push_back(future_road.back()); //��¼��ȥ
				future_road.pop_back(); //ɾ��δ��
			}
		}
		//���յ���
		else if (future_road.size() == 1)
		{
			//ֻ��Ŀ�꣬��ɾ����������
			r.SetNextV(this->CalTrackRoadV(r, future_road.back()));
		}
	}
	else
	{
		//·�����ˣ�ͣ
		r.Stop();
		return;
	}
}


float MotionControl::AngleDiff(pair<float, float> target_pos, pair<float, float> robot_pos, float robot_dir)
{
	// �ٶ�������ת����ʹ���������ȷ��
	pair<float, float> a = make_pair(cos(robot_dir), sin(robot_dir));//�÷���ĵ�λ����
	pair<float, float> b = make_pair(target_pos.first - robot_pos.first,target_pos.second - robot_pos.second);
	int dir = 1;
	if (a.first * b.second - b.first * a.second < 0)
		dir = -1;

	//��λ��
	//float b_length = sqrt(pow(b.first, 2) + pow(b.second, 2));
	//b.first = b.first / b_length;
	//b.second = b.second / b.second;

	//float angle = dir *acos((a.first * b.first + a.second * b.second));

	//����Ŀ����������X��������ļн�
	float target_dir = atan2(target_pos.second-robot_pos.second, target_pos.first-robot_pos.first);

	//��������˷���������Ŀ�귽�������мнǽ�С��һ��
	float angle_diff = abs(target_dir - robot_dir);
	float angle_diff_sup = abs(2 * M_PI - angle_diff);
	float min_angle_diff = (angle_diff_sup >= angle_diff) ? angle_diff : angle_diff_sup;

	return dir * min_angle_diff;
}

//�������нǣ�0~PI��
float MotionControl::AngleDiff(std::pair<float, float> relative_pos, std::pair<float, float> relative_v)
{
	float pos_dir = atan2(relative_pos.second, relative_pos.first);
	float v_dir = atan2(relative_v.second, relative_v.first);

	//����н��н�С��һ��
	float angle_diff = abs(pos_dir - v_dir);
	float angle_diff_sup = abs(2 * M_PI - angle_diff);
	float min_angle_diff = (angle_diff_sup >= angle_diff) ? angle_diff : angle_diff_sup;

	return min_angle_diff;
}



float MotionControl::AnglePDcontrol(float angle_diff)
{

	float f_P = 6;
	float f_D = 0.2;
	//float output = f_P * angle_diff + f_D*(angle_diff - this->m_LastAngleDiff); // PD����
	float angle_output = f_P * angle_diff; // ��򵥵ı�������
	
	//�����
	if (angle_output > M_PI)
		angle_output = M_PI;
	else if (angle_output < -M_PI)
		angle_output = -M_PI;

	return angle_output;
}

bool MotionControl::LagCheck(Robot& r, Robot* others)
{
	float lag_speed = 0.2;
	for (int i = 0; i < this->m_RobotsNum; i++)
	{
		if (i != r.GetID())
		{
			if (TargetDistance(r.GetPos(), others[i].GetPos()) < 1.1 && (abs(r.GetV().first)+abs(r.GetV().second))< lag_speed && (abs(others[i].GetV().first) + abs(others[i].GetV().second)) < lag_speed)
			{
				return true;
			}
		}
	}
	return false;
}

//�ٶȿ���
float MotionControl::LinearPDcontrol(Robot& r, float angle)
{
	float lv;
	if (angle < 3/M_PI)
	{
		if (!r.GetEarlyBrake())
			lv = r.GetMaxSpeed();
		else
			lv = r.GetReachSpeed();//����Ŀ������
	}
	else
		lv = 0;
	return lv;
}


void MotionControl::MakeOrder(Robot* robots, vector<pair<string, pair<int, float> > >& Order_1, vector<pair<string, int > >& Order_2,  CongestionControl* cc)
{
	this->cc = cc;
	this->WhoNeedAvoidance(robots);
	this->Navigation(robots); 
	for (int i = 0; i < this->m_RobotsNum; i++)
	{
		if (robots[i].GetTask().empty()) continue;

		if (robots[i].NeedStop())//ֻҪǰ����������robot.stop��������ʲô������õģ�����ͣ
			Order_1.push_back(make_pair("forward", make_pair(i, 0)));
		else
			Order_1.push_back(make_pair("forward", make_pair(i, robots[i].GetNextV().first)));
		if (!LagCheck(robots[i], robots))
		{
			Order_1.push_back(make_pair("rotate", make_pair(i, robots[i].GetNextV().second)));
		}
		else
		{
			srand(time(NULL)); // ������������ӣ��Ե�ǰʱ����Ϊ����
			float r = rand() * 3.0 / RAND_MAX;
			Order_1.push_back(make_pair("rotate", make_pair(i, r)));
		}

		if (robots[i].GetBuyorSell() == -1)
		{
			Order_2.push_back(make_pair("buy",i));
		}
		else if (robots[i].GetBuyorSell() == 1)
		{
			Order_2.push_back(make_pair("sell", i));
		}
		else if (robots[i].GetBuyorSell() == 2)
		{
			Order_2.push_back(make_pair("destory", i));
		}
	}

}

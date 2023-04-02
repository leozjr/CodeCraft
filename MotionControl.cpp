#include "MotionControl.h"
#include"Robot.h"
#include<set>
#include<vector>
#define _USE_MATH_DEFINES
#include<math.h>
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

//�������̿���
void MotionControl::Navigation(Robot* robots)
{
	for (int i = 0; i < this->m_RobotsNum; i++)
	{
		//��ײ���
		bool collision = CollisionCheck(robots, i);
		if (collision)
		{
			float linear_v = 6;

			float angle_v;
			//ȫ��ת��
			if (this->m_TurnRight)
				angle_v = -1*this->m_CollisionTurnPower;
			else
				angle_v = m_CollisionTurnPower;

			robots[i].SetNextV(make_pair(linear_v, angle_v));
			
		}
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
	float linear_v = this->LinearPDcontrol(target_point, r.GetPos(), abs(angle_diff));
	return make_pair(linear_v, angle_v);
}

//ѭ��
void MotionControl::TrackRoad(Robot& r)
{
	if (!r.m_Road.empty())
	{
		//�����������Ȼ����̫����
		vector<pair<float, float>>& road = r.m_Road[0]; //һ��·
		if (r.GetTransState()) 
		{
			//���׳ɹ��������·��
			road.clear();
			r.m_Road.erase(r.m_Road.begin());//ɾ����·��
			return;
		}
		//���յ�ǰ
		if (road.size() > 1)
		{
			//����ѭ����ɾ���߹���·
			r.SetNextV(this->CalTrackRoadV(r, road.back()));
			if (TargetDistance(r.GetPos(), road.back()) < 1.5) {
				road.pop_back();
			}
		}
		//���յ���
		else if (road.size() == 1)
		{
			//ֻ��Ŀ�꣬��ɾ����������
			r.SetNextV(this->CalTrackRoadV(r, road.back()));
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

bool MotionControl::HitWallCheck(pair<float, float> target_pos, pair<float, float> my_pos)
{
	float stop_dis = 2;
	if (target_pos.first < stop_dis || target_pos.first > 50-stop_dis || target_pos.second < stop_dis || target_pos.second > 50 - stop_dis)
	{
		float dis = TargetDistance(target_pos, my_pos);
		if (dis < this->m_WallSlowDownDis)
			return true;
		else
			return false;
	}
	else
		return false;
}

//�ٶȿ���
float MotionControl::LinearPDcontrol(pair<float, float> target_pos, pair<float, float> my_pos, float angle)
{
	float lv;

	if (angle < 3/M_PI)
	{
		if (!HitWallCheck(target_pos, my_pos))
			lv = this->m_MaxSpeed;
		else
			lv = this->m_WallSpeed;//ǽ��Ŀ������
	}
	else
		lv = 0;
	return lv;
}


void MotionControl::MakeOrder(Robot* robots, vector<pair<string, pair<int, float> > >& Order_1, vector<pair<string, int > >& Order_2)
{
	Navigation(robots);
	for (int i = 0; i < this->m_RobotsNum; i++)
	{
		if (robots[i].NeedStop())//ֻҪǰ����������robot.stop��������ʲô������õģ�����ͣ
			Order_1.push_back(make_pair("forward", make_pair(i, 0)));
		else
			Order_1.push_back(make_pair("forward", make_pair(i, robots[i].GetNextV().first)));
		Order_1.push_back(make_pair("rotate", make_pair(i, robots[i].GetNextV().second)));
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

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

// 绝对坐标转相对坐标
pair<float, float> RelativePair(pair<float, float> p1, pair<float, float> p2)
{
	return make_pair(p1.first - p2.first, p1.second - p2.second);
}

// 距离计算
float TargetDistance(std::pair<float, float> target_pos, std::pair<float, float> robot_pos)
{
	return sqrt(pow(target_pos.first - robot_pos.first, 2) + pow(target_pos.second - robot_pos.second, 2));
}

//计算VO，返回顺序为左边界向量和右边界向量
pair<pair<float, float>, pair<float, float>> CalculateVO(pair<float, float> pos, float r)
{
	float dis = sqrt(pow(pos.first, 2) + pow(pos.second, 2)); //相对距离
	float dis_2 = pow(dis, 2);
	float leg = sqrt(pow(dis, 2) - pow(r, 2)); //边界向量的长度（大圆切线到原点长度）
	
	//pos向量旋转到左边界的单位向量 x' = x cos - y sin, y' = x sin + y cos
	pair<float, float> left_VO = make_pair((pos.first * leg - pos.second * r)/dis_2, (pos.first * r + pos.second * leg)/ dis_2);
	//pos向量旋转到右边界的单位向量 x' = x cos + y sin, y' = y cos - x sin
	pair<float, float> right_VO = make_pair((pos.first * leg + pos.second * r) / dis_2, (pos.second * leg - pos.first * r) / dis_2);

	return make_pair(left_VO, right_VO);
}

// 计算VO算法中的相对半径
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
return mr + yr; //相对半径
}

bool MotionControl::InVO(pair<pair<float, float>, pair<float, float>> vo, pair<float, float> relative_v, pair<float, float> relative_pos)
{
	float right_vo_angle = atan2(vo.second.second, vo.second.first);
	float left_vo_angle = atan2(vo.first.second, vo.first.first);
	float middle_vo_angle = atan2(relative_pos.second, relative_pos.first);
	float relative_v_angle = atan2(relative_v.second, relative_v.first);

	// 横跨了X轴负方向是一个特殊情况，要先解决。
	if (right_vo_angle > 0 && left_vo_angle < 0)
	{
		if (relative_v_angle > right_vo_angle || relative_v_angle < left_vo_angle)
		{
			//特殊情况的左右转
			if (middle_vo_angle < 0 && relative_v_angle>0)
			{
				this->m_TurnRight = true;
			}
			else if (middle_vo_angle > 0 && relative_v_angle < 0)
			{
				this->m_TurnRight = false;
			}
			else if (relative_v_angle < middle_vo_angle)
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
	//不穿越负X轴，正常判断
	if (relative_v_angle > right_vo_angle && relative_v_angle < left_vo_angle)
	{
		//左右转的判断
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

//同向判定
bool SameDir(Robot& r1, Robot& r2)
{
	float angle_diff = abs(r1.GetDir() - r2.GetDir());
	float angle_diff_sup = abs(2 * M_PI - angle_diff);
	float min_angle_diff = (angle_diff_sup >= angle_diff) ? angle_diff : angle_diff_sup;

	return (min_angle_diff < 0.34);//20度代表同向
}

void MotionControl::WhoNeedAvoidance(Robot* robots)
{
	for (int i = 0; i < this->m_RobotsNum; i++)
	{
		if (robots[i].GetTask().empty()) continue;
		//将来路与过去路重合的人，具有优先通行权
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

	//检查所有的机器人，谁需要避让
	for (int i = 0; i < this->m_RobotsNum; i++)
	{
		for (int j = i + 1; j < this->m_RobotsNum; j++)
		{
			if (robots[i].GetTask().empty() || robots[j].GetTask().empty()) continue;

			//避让过程中切换了任务
			if (robots[i].GetAvoidance() && robots[i].GetAvoidID() == j && !robots[j].isBusy())
			{
				robots[j].SetAvoidance(true, i);
			}
			else if (robots[j].GetAvoidance() && robots[i].GetAvoidID() == i && !robots[i].isBusy())
			{
				robots[i].SetAvoidance(true, j);
			}

			if (!((robots[i].GetAvoidance() && robots[j].GetAvoidance()) || (robots[i].GetAvoidance() && robots[i].GetAvoidID() == j) || (robots[j].GetAvoidance() && robots[j].GetAvoidID() == i)))
			{

				//检查避让者的parkroad
				if (robots[i].GetAvoidance() && !robots[i].m_ParkFutureRoad.empty())
				{
					//i避让，i走park_future_road
					if (this->cc->Congestion(robots[i].m_ParkFutureRoad, i, robots[j].m_FutureRoad[0], j, 1))
					{
						//j和i的ParkFutureRoad冲突
						robots[j].SetAvoidance(true, i);
					}

				}
				else if (robots[i].GetAvoidance() && !robots[i].m_ParkPastRoad.empty())
				{
					//i避让，i走park_past_road
					if (this->cc->Congestion(robots[i].m_ParkPastRoad, i, robots[j].m_FutureRoad[0], j, 1))
					{
						//j和i的ParkFutureRoad冲突
						robots[j].SetAvoidance(true, i);
					}

				}
				else if (robots[i].GetAvoidance() && !robots[i].m_PastRoad.empty())
				{
					if (this->cc->Congestion(robots[i].m_PastRoad, i, robots[j].m_FutureRoad[0], j, 1))
					{
						//j和i的ParkFutureRoad冲突
						robots[j].SetAvoidance(true, i);
					}
				}
				else if (robots[j].GetAvoidance() && !robots[j].m_ParkFutureRoad.empty())
				{
					//j避让，j走park_future_road
					if (this->cc->Congestion(robots[j].m_ParkFutureRoad, j, robots[i].m_FutureRoad[0], i, 1))
					{
						//j和i的ParkFutureRoad冲突
						robots[i].SetAvoidance(true, j);
					}

				}
				else if (robots[j].GetAvoidance() && !robots[j].m_ParkPastRoad.empty())
				{
					//j避让，j走park_past_road
					if (this->cc->Congestion(robots[j].m_ParkPastRoad, j, robots[i].m_FutureRoad[0], i, 1))
					{
						//j和i的ParkFutureRoad冲突
						robots[i].SetAvoidance(true, j);
					}

				}
				else if (robots[j].GetAvoidance() && !robots[j].m_PastRoad.empty())
				{
					if (this->cc->Congestion(robots[j].m_PastRoad, j, robots[i].m_FutureRoad[0], i, 1))
					{
						//j和i的ParkFutureRoad冲突
						robots[i].SetAvoidance(true, j);
					}
				}
				else if (this->cc->Congestion(robots[i].m_FutureRoad[0], i, robots[j].m_FutureRoad[0], j, 1))
				{
					
					this->cc->path1 = robots[i].m_FutureRoad[0];
					this->cc->path2 = robots[j].m_FutureRoad[0];

					//存在冲突，根据优先级确定谁需要避让
					if (robots[i].GetAvoidance()) {
						if (robots[j].GetPriorityPass()) {
							robots[j].Stop();
						}
						else {
							robots[j].SetAvoidance(true, i);
						}
					}
					else if (robots[j].GetAvoidance()) {
						if (robots[i].GetPriorityPass()) {
							robots[i].Stop();
						}
						else {
							robots[i].SetAvoidance(true, j);
						}
					}
					else if (robots[i].GetPriorityPass()) {
						robots[j].SetAvoidance(true, i);
					}
					else if (robots[j].GetPriorityPass()) {
						robots[i].SetAvoidance(true, j);
					}
					else if (robots[i].GetGoods() <= robots[j].GetGoods())
					{
						robots[i].SetAvoidance(true, j);
					}
					else
					{
						robots[j].SetAvoidance(true, i);
					}
				}
			}
		}
	}
}

void MotionControl::TrackAvoidanceBack(int my_id, int first_go_id, Robot* robots)
{
	Robot& r = robots[my_id];
	Robot& r_first_go = robots[first_go_id];
	vector<pair<float, float>> avoider_pos;
	for (int i = 0; i < this->m_RobotsNum; i++)
	{
		if (robots[i].GetAvoidance() && my_id != i)
			avoider_pos.push_back(robots[i].GetPos());
	}

	//看一下别人的路，为了避免站到别人路上
	vector<vector<pair<float, float>>> others_road;
	vector<pair<float, float>> part_of_future_road;
	for (int i = 0; i < this->m_RobotsNum; i++)
	{
		if (robots[i].GetTask().empty()) continue; //跳过没有任务的机器人

		if (i != my_id && i != first_go_id)
		{
			if (robots[i].GetAvoidance() && robots[i].CanPark())
			{
				if (!robots[i].m_ParkFutureRoad.empty())
				{
					others_road.push_back(robots[i].m_ParkFutureRoad);
				}
				else if (!robots[i].m_ParkPastRoad.empty())
				{
					others_road.push_back(robots[i].m_ParkPastRoad);
				}
			}
			else
			{
				int num = min(int(robots[i].m_FutureRoad[0].size()), 30);
				part_of_future_road.assign(robots[i].m_FutureRoad[0].end()-num, robots[i].m_FutureRoad[0].end());
				others_road.push_back(part_of_future_road);
			}
		}
	}

	if (!r.CanPark() && !r.EndPark)
	{	
		//往后开始循迹
		r.SetNextV(this->CalTrackRoadV(r, r.m_PastRoad.back()));

		this->cc->m_LeaveCongestionPoint[r_first_go.GetID()] = r.m_FutureRoad[0].back(); //还没搜到时，默认的leave_point是Future_road的第一个
		if (TargetDistance(r.GetPos(), r.m_PastRoad.back()) < this->m_TrackDistance)
		{
			r.m_FutureRoad[0].push_back(r.m_PastRoad.back());
			r.m_PastRoad.pop_back();

			//同时搜索可以停靠的点
			if (r_first_go.GetAvoidance())
			{
				if (!r_first_go.m_ParkFutureRoad.empty())
				{
					vector<pair<float, float>> all_road = r_first_go.m_FutureRoad[0];
					all_road.insert(all_road.end(), r_first_go.m_ParkPastRoad.begin(), r_first_go.m_ParkPastRoad.end());
					all_road.insert(all_road.end(), r_first_go.m_ParkFutureRoad.begin(), r_first_go.m_ParkFutureRoad.end());
					
					r.m_ParkFutureRoad = this->cc->AvoidanceRoad(r.m_PastRoad.back(), all_road, r.GetPos(), r_first_go.GetPos(), others_road, avoider_pos);
				}
				else if (!r_first_go.m_ParkPastRoad.empty())
				{
					vector<pair<float, float>> all_road = r_first_go.m_FutureRoad[0];
					all_road.insert(all_road.end(), r_first_go.m_ParkPastRoad.begin(), r_first_go.m_ParkPastRoad.end());
					r.m_ParkFutureRoad = this->cc->AvoidanceRoad(r.m_PastRoad.back(), all_road, r.GetPos(), r_first_go.GetPos(), others_road, avoider_pos);
				}
				else
				{
					int num = min(int(r_first_go.m_PastRoad.size()), 150);
					part_of_future_road.assign(r_first_go.m_PastRoad.end() - num, r_first_go.m_PastRoad.end());
					r.m_ParkFutureRoad = this->cc->AvoidanceRoad(r.m_PastRoad.back(), part_of_future_road, r.GetPos(), r_first_go.GetPos(), others_road, avoider_pos);
				}
			}
			else //如果避让的人是正常人
			{
				r.m_ParkFutureRoad = this->cc->AvoidanceRoad(r.m_PastRoad.back(), r_first_go.m_FutureRoad[0], r.GetPos(), r_first_go.GetPos(), others_road, avoider_pos);
			}
			
			if (!r.m_ParkFutureRoad.empty())
			{
				this->cc->m_LeaveCongestionPoint[r_first_go.GetID()] = *(r.m_ParkFutureRoad.begin());
				//泊车标志位为真，之后不再搜索路线
				r.SetCanPark(true);
			}
		}

		if (cc->CanGo(r.CanPark(), r_first_go))
			r.EndPark = true; //可以走了，直接结束泊车，直接走
	}
	else if (r.CanPark() && !r.EndPark)
	{	
		if (cc->CanGo(r.CanPark(), r_first_go))
			r.EndPark = true; //可以走了，直接结束泊车，直接走

		//开始循到泊车点，在泊车点等待
		if (r.m_ParkFutureRoad.size() > 1)
		{
			//循迹
			r.SetNextV(this->CalTrackRoadV(r, r.m_ParkFutureRoad.back()));
			if (TargetDistance(r.GetPos(), r.m_ParkFutureRoad.back()) < this->m_TrackDistance)
			{
				r.m_ParkPastRoad.push_back(r.m_ParkFutureRoad.back());
				r.m_ParkFutureRoad.pop_back();
			}
		}
		else if(r.m_ParkFutureRoad.size() == 1)
		{
			//在泊车点等待
			r.SetNextV(this->CalTrackRoadV(r, r.m_ParkFutureRoad.back()));
		}
	}
	
	if (r.EndPark)
	{
		// 直接回去的三种情况，在park_future_road上，在park_past_road上，连路都没搜到的情况

		if (!r.m_ParkFutureRoad.empty())
		{
			//还是会先走完parkFutureRoad，然后
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
			//解除避让状态，必须非常精确的回到避让点才予以解除
			r.SetNextV(this->CalTrackRoadV(r, r.m_ParkPastRoad.back()));
			if (TargetDistance(r.GetPos(), r.m_ParkPastRoad.back()) < 0.5)
				r.m_ParkPastRoad.pop_back();
		}
		
		//当前帧就要直接置位
		if(!r.CanPark() || (r.CanPark() && r.m_ParkFutureRoad.empty() && r.m_ParkPastRoad.empty()))
		{ 
			//路都没找到，就可以直接走了，那么也允许解除，或者找到路了，走完全程后，予以解除
			r.SetAvoidance(false, -1);
			r.SetCanPark(false);
			r.EndPark = false;
			r.m_ParkFutureRoad.clear();
			r.m_ParkPastRoad.clear();
		}
	}
	
}

//导航流程控制
void MotionControl::Navigation(Robot* robots)
{
	for (int i = 0; i < this->m_RobotsNum; i++)
	{
		if (robots[i].GetTask().empty()) continue;

		//距离控制
		this->KeepDistance(i, robots);

		if (robots[i].GetAvoidance())
		{
			//该机器人处于避让状态
			TrackAvoidanceBack(i, robots[i].GetAvoidID(), robots);
		}
		/*else if (CollisionCheck(robots, i))
		{
			//VO 碰撞检测
			float linear_v = 6;

			float angle_v;
			//全力转弯
			if (this->m_TurnRight)
				angle_v = -1*this->m_CollisionTurnPower;
			else
				angle_v = m_CollisionTurnPower;

			robots[i].SetNextV(make_pair(linear_v, angle_v));
			
		}*/
		else
		{

			//循迹
			this->TrackRoad(robots[i]);
		}

	}
}

//碰撞检测
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

			//根据货物携带状态，给出不同的相对半径r
			float r = CalculateR(robots[my_id].GetGoods(), robots[collision_id[m]].GetGoods());
			vo = CalculateVO(relative_pos, r);

			if (this->InVO(vo, relative_v, relative_pos) && !SameDir(robots[my_id], robots[collision_id[m]]))
			{	//说明要撞了
				collision = true;
				break;//只要知道有一个要撞了就够了
			}
		}
	}
	return collision;
}

//进入碰撞预警范围检查
vector<int> MotionControl::CollisionDetection(Robot& robot)
{
	vector<pair<float, int> > dis_robots = robot.GetRobotsDistance();
	float v = sqrt(pow(robot.GetV().first, 2)+ pow(robot.GetV().second, 2));
	vector<int> Check_list;
	for (int i = 0; i < 3; i++)
	{
		//首先检查是否有机器人落入m_CollisionCheckLimit范围内
		if (dis_robots[i].first < this->m_MinLimitDistace + this->m_CollisionCheckVFactor * v)
		{
			Check_list.push_back(dis_robots[i].second);
		}
	}
	return Check_list;
}

//计算循迹速度控制 <线速度，角速度>
pair<float, float> MotionControl::CalTrackRoadV(Robot& r, pair<float, float> target_point)
{
	pair<float, float> final_target_point = target_point;
	if (target_point != r.m_FutureRoad[0].front())
		final_target_point = this->NearWallSolver(target_point); // 不是工作台，就执行靠墙挪点

	//这个angle_diff是带正负号的
	float angle_diff = this->AngleDiff(final_target_point, r.GetPos(), r.GetDir());

	float angle_v = this->AnglePDcontrol(angle_diff);
	float linear_v = this->LinearPDcontrol(r, abs(angle_diff));
	return make_pair(linear_v, angle_v);
}

// 靠墙点的挪动函数
std::pair<float, float> MotionControl::NearWallSolver(pair<float, float> target_point)
{
	pair<int, int> map_pos = make_pair((49.75 - target_point.second) * 2, (target_point.first - 0.25) * 2);
	int x = map_pos.first;
	int y = map_pos.second;
	pair<float, float> op_pos;
	if (x > 0 && x < 99 && y > 0 && y < 99) //防止越界
	{
		//首先判断上下左右
		if (this->cc->io->Map[x - 1][y] == 1)
		{
			op_pos = make_pair(0.25 + 0.5 * y, 49.75 - 0.5 * (x + 1));
			return make_pair((target_point.first + op_pos.first) / 2, (target_point.second + op_pos.second) / 2);
		}
		else if (this->cc->io->Map[x + 1][y] == 1)
		{
			op_pos = make_pair(0.25 + 0.5 * y, 49.75 - 0.5 * (x - 1));
			return make_pair((target_point.first + op_pos.first) / 2, (target_point.second + op_pos.second) / 2);
		}
		else if (this->cc->io->Map[x][y - 1] == 1)
		{
			op_pos = make_pair(0.25 + 0.5 * (y + 1), 49.75 - 0.5 * x);
			return make_pair((target_point.first + op_pos.first) / 2, (target_point.second + op_pos.second) / 2);
		}
		else if (this->cc->io->Map[x][y + 1] == 1)
		{
			op_pos = make_pair(0.25 + 0.5 * (y - 1), 49.75 - 0.5 * x);
			return make_pair((target_point.first + op_pos.first) / 2, (target_point.second + op_pos.second) / 2);
		}
		else
		{
			//然后判断角格
			vector<pair<int, int>> pos_vec;
			for (int ii = -1; ii < 2; ii++) {
				for (int jj = -1; jj < 2; jj++) {
					if (ii != 0 && jj != 0 && this->cc->io->Map[x + ii][y + jj] == 1) {
						pos_vec.push_back(make_pair(x + ii, y + jj));
					}
				}
			}
			if (pos_vec.size() == 1) {

				op_pos = make_pair(0.25 + 0.5 * (2*y - pos_vec[0].second), 49.75 - 0.5 * (2 * x - pos_vec[0].first));
				return make_pair((target_point.first + op_pos.first) / 2.0, (target_point.second + op_pos.second) / 2.0);
			}
			else if (pos_vec.size() == 2) {
				pair<int, int> oop_pos = make_pair((pos_vec[0].first + pos_vec[1].first) / 2, (pos_vec[0].second + pos_vec[1].second)/2);
				op_pos = make_pair(0.25 + 0.5 * (2*y - oop_pos.second), 49.75 - 0.5 * (2*x - oop_pos.first));
				return make_pair((target_point.first + op_pos.first) / 2.0, (target_point.second + op_pos.second) / 2.0);
			}
		}
	}
	return target_point;
}

//保持间距, 反向时不应该起作用
void MotionControl::KeepDistance(int my_id, Robot* others)
{
	Robot& r = others[my_id];
	float default_range = 2; //当x或y坐标相同时，默认探测范围
	int p_gap = 2; //间隔几个点检查，间隔越大，探测到的范围最大
	int p_range = 1; //检查几个点开外的人，越大越远
	
	vector<pair<float, float>>* road = NULL; //根据机器人的当前状态选择合适的路
	if (r.GetAvoidance())
	{
		if (r.CanPark())
		{
			if (!r.m_ParkFutureRoad.empty())
			{
				road = &r.m_ParkFutureRoad;
			}
			else if(!r.m_ParkPastRoad.empty())
			{
				road = &r.m_ParkPastRoad;
			}
		}
		else
		{
			int idx = min(int(r.m_PastRoad.size() - 1), 10); //急停最多五六个都很多了，给10个
			vector<pair<float, float>> part_past_road;
			part_past_road.assign(r.m_PastRoad.end() - idx, r.m_PastRoad.end());
			road = &part_past_road;
		}
	}
	else
	{
		road = &r.m_FutureRoad[0];
	}
	int N = min(p_range, int(road->size()-p_gap));; //p_range个点的future_road探测空间，或者future_road长度不够6时，取size-1

	if (N < 1)
		return;

	//遍历N个点，检查是否有人落在这个区间
	for (auto iter = road->end() - N; iter != road->end(); ++iter)
	{
		pair<float, float> p = *iter; //当前坐标
		if (iter == road->end() - 1)
			p = r.GetPos(); //最后一个点的时候，以机器人自己坐标为准

		pair<float, float> p_next = *(iter-p_gap); //前一个坐标

		for (int i = 0; i < this->m_RobotsNum; i++)
		{
			//别人是否落入呢？
			if (i != my_id)
			{
				//反向检查，如果该人与我反向，那么下面的判断大概率两人都停，所以不做停止操作
				//if (AngleDiff(r.GetDir(), others[i].GetDir()) > 2 * M_PI / 3)
				//	return;

				pair<float, float> other_pos = others[i].GetPos(); //当前坐标
				if ((p_next.first - p.first) == 0)
				{
					//x坐标相同
					if (abs(other_pos.first - p.first) < default_range && ((other_pos.second > p.second && other_pos.second < p_next.second) || (other_pos.second < p.second && other_pos.second > p_next.second)))
					{
						//有人在我这个点的区间中
						r.Stop();
					}
				}
				else if ((p_next.second - p.second) == 0)
				{
					//y坐标相同
					if (abs(other_pos.second - p.second) < default_range && ((other_pos.first > p.first && other_pos.first < p_next.first) || (other_pos.first < p.first && other_pos.first > p_next.first)))
					{
						//有人在我这个点的区间中
						r.Stop();
					}
				}
				else
				{
					if (((other_pos.first > p.first && other_pos.first< p_next.first) || (other_pos.first < p.first && other_pos.first > p_next.first)) && ((other_pos.second > p.second && other_pos.second < p_next.second) || (other_pos.second < p.second && other_pos.second > p_next.second)))
					{
						//有人在我这个点的区间中
						r.Stop();
					}
				}
			}
		}
		
	}
}

//机器人跳点解决方法
void MotionControl::JumpPointSolver(Robot& r)
{
	//在前N个点中取最近的一个
	int n = min(5, int(r.m_FutureRoad[0].size()));
	if (n == 0)
		return;

	auto min_iter = r.m_FutureRoad[0].end();
	float min_dist = std::numeric_limits<float>::max(); // 初始化最小距离为float的最大值
	for (auto iter = r.m_FutureRoad[0].end() - n; iter != r.m_FutureRoad[0].end(); ++iter) {
		float dist = TargetDistance(r.GetPos(), (*iter));
		if (dist < min_dist) {
			min_dist = dist; // 更新最小距离
			min_iter = iter; // 更新最小距离对应的迭代器
		}
	}

	while (r.m_FutureRoad[0].end()-1 != min_iter) {
		r.m_PastRoad.push_back(r.m_FutureRoad[0].back()); 
		r.m_FutureRoad[0].pop_back(); 
	}
}

//循迹
void MotionControl::TrackRoad(Robot& r)
{
	if (!r.m_FutureRoad.empty())
	{
		//起个别名
		vector<pair<float, float>>& future_road = r.m_FutureRoad[0]; //未来走的路
		vector<pair<float, float>>& past_road = r.m_PastRoad; //走过的路
		vector<pair<float, float>> part_past_road; // 部分过去路
		
		if (r.GetTransState())
		{
			future_road.clear();
			r.m_FutureRoad.erase(r.m_FutureRoad.begin());//删除该路线
			r.m_PastRoad.clear(); //清空历史，
			r.AllowJump = true;
			return;
		}
		//到终点前
		if (future_road.size() > 1)
		{
			//正常循迹
			vector<vector<pair<float, float>>> others_road; //用了AvoidanceRoad搜空间，但这里没必要再考虑和其他路重合，先给个空的吧
			vector<pair<float, float>> avoider_pos;
			//偏离路径后，跳点的检查及解决
			if (future_road.back() == future_road.front())
				r.AllowJump = false; //到最后时刻，可能出现需要到达再返回的情况
			if(r.AllowJump)
				this->JumpPointSolver(r);

			r.SetNextV(this->CalTrackRoadV(r, future_road.back()));
			if (TargetDistance(r.GetPos(), future_road.back()) < this->m_TrackDistance)
			{
				//释放优先权
				if (r.GetPriorityPass())
				{
					int idx = min(int(r.m_PastRoad.size()-1), 30);
					part_past_road.assign(r.m_PastRoad.end()-idx, r.m_PastRoad.end());
					if (future_road.back() == r.EndPriorityPass)
					{
						r.SetPriorityPass(false);
						r.EndPriorityNumber = 0;
					}
					else if (!this->cc->AvoidanceRoad(r.m_FutureRoad[0].back(), part_past_road, r.GetPos(), make_pair(100, 100), others_road, avoider_pos).empty())
					{
						r.EndPriorityNumber += 1;
					}
					
					if (r.EndPriorityNumber > 3)
					{
						r.SetPriorityPass(false);
						r.EndPriorityNumber = 0;
					}
					
				}
				past_road.push_back(future_road.back()); //记录过去
				future_road.pop_back(); //删掉未来
			}
		}
		//到终点了
		else if (future_road.size() == 1)
		{
			//只设目标，不删除最后这个点
			r.SetNextV(this->CalTrackRoadV(r, future_road.back()));
		}
	}
	else
	{
		//路走完了，停
		r.Stop();
		return;
	}
}


float MotionControl::AngleDiff(pair<float, float> target_pos, pair<float, float> robot_pos, float robot_dir)
{
	// 速度向量旋转方向，使用向量叉乘确定
	pair<float, float> a = make_pair(cos(robot_dir), sin(robot_dir));//该方向的单位向量
	pair<float, float> b = make_pair(target_pos.first - robot_pos.first,target_pos.second - robot_pos.second);
	int dir = 1;
	if (a.first * b.second - b.first * a.second < 0)
		dir = -1;

	//计算目标向量关于X轴正方向的夹角
	float target_dir = atan2(target_pos.second-robot_pos.second, target_pos.first-robot_pos.first);

	//计算机器人方向向量和目标方向向量中夹角较小的一个
	float angle_diff = abs(target_dir - robot_dir);
	float angle_diff_sup = abs(2 * M_PI - angle_diff);
	float min_angle_diff = (angle_diff_sup >= angle_diff) ? angle_diff : angle_diff_sup;

	return dir * min_angle_diff;
}

//两向量夹角（0~PI）
float MotionControl::AngleDiff(std::pair<float, float> relative_pos, std::pair<float, float> relative_v)
{
	float pos_dir = atan2(relative_pos.second, relative_pos.first);
	float v_dir = atan2(relative_v.second, relative_v.first);

	//计算夹角中较小的一个
	float angle_diff = abs(pos_dir - v_dir);
	float angle_diff_sup = abs(2 * M_PI - angle_diff);
	float min_angle_diff = (angle_diff_sup >= angle_diff) ? angle_diff : angle_diff_sup;

	return min_angle_diff;
}
//给出方向，计算角度差
float MotionControl::AngleDiff(float dir1, float dir2)
{
	float angle_diff = abs(dir1 - dir2);
	float angle_diff_sup = abs(2 * M_PI - angle_diff);
	float min_angle_diff = (angle_diff_sup >= angle_diff) ? angle_diff : angle_diff_sup;
	return min_angle_diff;
}


float MotionControl::AnglePDcontrol(float angle_diff)
{
	float f_P = 4; //比例系数
	if (abs(angle_diff) > this->m_CanGoAngle)
	{
		int sign = std::signbit(angle_diff) ? -1 : 1; //正负号
		return sign * M_PI; 
	}
	else
	{
		return f_P * angle_diff;
	}
}

bool MotionControl::LagCheck(Robot& r, Robot* others)
{
	float lag_speed = 0.1;
	for (int i = 0; i < this->m_RobotsNum; i++)
	{
		if (i != r.GetID())
		{
			if (TargetDistance(r.GetPos(), others[i].GetPos()) < 1.1 && (abs(r.GetV().first)+abs(r.GetV().second))< lag_speed && (abs(others[i].GetV().first) + abs(others[i].GetV().second)) < lag_speed)
			{
				++this->m_RollBackTime;
			}
		}
	}
	if (this->m_RollBackTime > 300)
		return true;
	else
		return false;
}

void MotionControl::LagSolver(Robot& r, Robot* others)
{
	for (int i = 0; i < this->m_RobotsNum; i++)
	{
		if (r.GetAvoidance() && r.GetAvoidID() == others[i].GetID())
			continue;
		if (i != r.GetID() && TargetDistance(r.GetPos(), others[i].GetPos()) < 1.1)
		{
			others[i].SetAvoidance(true, r.GetID());
			this->m_RollBackTime = 0;
		}
	}
}

//速度控制
float MotionControl::LinearPDcontrol(Robot& r, float angle)
{
	float lv;
	if (angle < this->m_CanGoAngle)
	{
		if (!r.GetEarlyBrake())
			lv = r.GetMaxSpeed();
		else
			lv = r.GetReachSpeed();//到达目标限速
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
		
		if (!LagCheck(robots[i], robots)) 
		{
			//不卡的情况，正常形式
			Order_1.push_back(make_pair("rotate", make_pair(i, robots[i].GetNextV().second)));
			
			if (robots[i].NeedStop())//只要前面有人用了robot.stop，不管是什么情况下用的，都会停
				Order_1.push_back(make_pair("forward", make_pair(i, 0)));
			else
				Order_1.push_back(make_pair("forward", make_pair(i, robots[i].GetNextV().first)));
		}
		else
		{
			LagSolver(robots[i], robots);
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

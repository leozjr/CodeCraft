#include "WorkTable.h"
#include <set>
#include<vector>
using namespace std;

void WorkTable::Init(int id, std::pair<float, float> pos,int type)
{
	this->m_ID = id;
	this->m_Pos = pos;
	this->m_Type = type;
}

std::pair<float, float> WorkTable::GetPos()
{
	return this->m_Pos;
}

int WorkTable::GetID()
{
	return this->m_ID;
}

int WorkTable::GetType()
{
	return this->m_Type;
}

bool WorkTable::HaveProduct()
{
	return bool(this->m_ProductState);
}

vector<int> WorkTable::NeedWhat()
{
	vector<vector<int>> material_state = { 
		{-1},
		{-1},
		{-1},
		{ 0,1 },
		{ 0,2 },
		{ 1,2 },
		{ 3,4,5 },
		{6},
		{0,1,2,3,4,5,6}
	};

	if (this->m_Type == 0 || this->m_Type == 1 || this->m_Type == 2 || this->m_Type == 7 || this->m_Type == 8)
	{
		//特殊情况，123不缺，89永远缺
		return material_state[this->m_Type];
	}

	vector<int> lack_list;
	lack_list.clear();
	for (vector<int>::iterator it = material_state[this->m_Type].begin(); it != material_state[this->m_Type].end(); it++)
	{
		//不要迭代123缺货，没有意义
		if (*it != -1 && this->m_MaterialState[(*it)+1] == 0)
		{
			//如果对应货号的位置为0，而且当前没有机器人正在给我买这个货，说明缺这种货。把货号入栈。
			lack_list.push_back((*it));
		}
	}
	return lack_list;
}

std::set<std::pair<float, std::pair<int, int>>> WorkTable::DistanceGet()
{
	return this->m_DistanceOrder;
}

int WorkTable::GetCountDownFrame()
{
	return (this->m_CountDownFrame == -1) ? 15000 : this->m_CountDownFrame;
}

bool WorkTable::UpdateInfo(int cdf, int ms, int ps)
{
	this->m_CountDownFrame = cdf;
	this->m_MaterialState = ms;
	this->m_ProductState = ps;
	return true;
}

void WorkTable::DistanceSet(set<std::pair<float, std::pair<int, int>>> & ds)
{
	this->m_DistanceOrder = ds;
}

int WorkTable::GetMaterialState()
{
	return int(this->m_MaterialState.to_ulong());
}

int WorkTable::CountDownFrameGet()
{
	return this->m_CountDownFrame;
}

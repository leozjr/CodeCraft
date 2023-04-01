#pragma once
#include<set>
#include<vector>
#include<bitset>

class WorkTable
{
	int m_ID = -1;
	int m_Type;
	std::pair<float, float> m_Pos;
	int m_CountDownFrame;
	std::bitset<7> m_MaterialState;
	int m_ProductState;
	std::set<std::pair<float, std::pair<int, int> > > m_DistanceOrder; //工作台之间的距离【不包含：1、123类型和123类型之间的 2、彼此不可达的】

public:
	/*-------查询函数------*/
	std::pair<float, float> GetPos(); //位置
	int GetID(); //ID
	int GetType(); //类型
	int GetCountDownFrame(); //生产剩余帧数，不生产的情况本来是-1，为了统一，改为15000（即游戏结束也不会生产好），方便判断
	bool HaveProduct(); //是否有货
	//当前货物格状态（不考虑正在送货的情况），123型被调用，返回一个-1的vector，456789型返回缺货的vector（标号从小到大），不缺返回一个空vector
	std::vector<int> NeedWhat();
	std::set<std::pair<float, std::pair<int, int> > > DistanceGet(); //查询工作台之间距离

	/*------IO类用的------*/
	void Init(int id, std::pair<float, float> pos, int type); //初始化函数，传入类型位置和id，后续不再更新
	bool UpdateInfo(int cdf, int ms, int ps);
	void DistanceSet(std::set<std::pair<float, std::pair<int, int> > > & ds);

	/*买卖分离决策*/
	int GetMaterialState(); //返回整数形式的产品格状态
	int CountDownFrameGet(); //需要通过-1判断是否在生产
};
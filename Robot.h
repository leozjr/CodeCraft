#pragma once
#include<set>
#include <cmath>
#include<vector>
#include<deque>
#include<array>
#include"WorkTable.h"
class Robot
{
	// �������������
	int m_ID; //������id
	int m_WorkTableID; //���ڹ���̨id
	int m_Goods;//��������
	float m_TimeValueFactor; // ʱ�����ϵ��
	float m_CollisionValueFactor; // ��ײ���ϵ��
	float m_AngularVelocity; //���ٶ�
	std::pair<float, float> m_LinearVelocity; //���ٶ�
	float m_Direction; //����
	std::pair<float, float> m_Pos; //λ��

	// ��������
	bool m_Busy = false; //æ��־λ
	bool m_Stop = false; //ͣ��־λ
	bool m_SuccTrans = false; //���׳ɹ���־λ
	int m_Wait = 0; //�������������Ҫ

	bool m_PriorityPass; //����ͬ��Ȩ
	bool m_Avoidance = false; //����״̬��־λ
	int m_AvoidID; //���ö����ID��˭
	bool m_CanPark = false; //������ͣ��λ�ı�־λ

	bool m_PrecisionControl = false; //��׼���Ʊ�־λ
	float m_PrecisionSpeed = 3;

	bool m_EarlyBrake = false; //���﹤��̨ǰ��ǰɲ��
	float m_MaxSpeed = 6; //�û����˵�����ٶȣ���ʼΪ6��
	float m_ReachSpeed = 1; //ǽ��Ŀ������
	float m_BrakeDistance = 1; //��ǰ���پ���

	int m_BuyOrSell = 0; //0������Ҳ������1������ -1���� 2:����
	std::vector<int> m_Task; //���Ķ��������Ķ���0:���id, 1������id��
	std::vector<std::pair<float, int>> m_RobotsDistance; //�����˼����
	std::set<std::pair<float, std::pair<int, int> > > m_DistanceOrder_robot; //��ʼ�����˺͹���̨֮��ľ���

	void Buy(); //���ָ� ֻ��BuySellCheck()��
	void Sell(); //����ָ�ֻ��BuySellCheck()��
	void Destory(); //���ٻ���ָ�ֻ��BuySellCheck()��
	bool Reached(); //��ѯ�Ƿ񵽴�Ŀ��ص㣬 ֻ��BuySellCheck()��

	void EarlyBrake(std::pair<float, float> target_pos, std::pair<float, float> my_pos); // ��ǰɲ������

	std::pair<float, float> m_NextV; //Ҫ��������ٶȺͽ��ٶȣ�<linear_v, angle_v>
public:
	std::vector<std::vector<std::pair<float, float> >> m_FutureRoad; //û�ߵ�·
	std::vector<std::pair<float, float> > m_PastRoad; //�߹���·������������

	std::vector<std::pair<float, float> > m_ParkFutureRoad; //ǰ�����õ��û�ߵ�·
	std::vector<std::pair<float, float> > m_ParkPastRoad; //�����е��߹���·
	bool EndPark = false; //ֹͣ���������Է���
	bool AllowJump = true; //��������
	
	std::pair<float, float> EndPriorityPass; //��������ͨ��Ȩ��־
	int EndPriorityNumber; //���������ѵ��ĵط����ͽ�������ͨ��
	int FullyCanGo; //��ȫ����ͨ�еı�־λ��������Ҫ��Leave_Point�㷶Χ��Ȼ���ٳ��㷶Χ��������
	

	/*-------��ѯ����------*/
	int GetID(); //ID
	std::pair<float, float> GetPos(); //λ��
	float GetDir(); //����
	std::pair<float, float> GetV(); //�ٶ�
	int GetGoods(); // Я������״̬ 0ΪδЯ����1234567Ϊ����Я����Ӧ���͵Ļ���
	std::pair<float, float> GetNextV();
	bool GetTransState(); //��ȡ����״̬
	bool GetAvoidance(); //��ȡ����״̬
	bool GetPriorityPass();
	int GetAvoidID();
	std::vector<int> GetTask(); //����״̬
	int GetBuyorSell(); //��ѯ����ָ��
	std::vector<std::pair<float, int>> GetRobotsDistance(); //��ѯ�����˼����
	bool CanPark();
	bool GetPrecisionControl();
	bool GetEarlyBrake(); //��ѯ��ǰɲ��
	float GetReachSpeed(); //��ѯ����ʱ���پ���
	float GetMaxSpeed();
	bool isBusy(); //�Ƿ�æµ
	
	//����������
	void BuySellCheck(int type, WorkTable* wts); //�����������飬��Ŀ��ص���Ƿ���Ҫ������type��1�������󶨣�2���������롿

	/*------IO���õ�------*/
	void Init(int id); //��ʼ��id
	bool UpdateInfo(int wt, int goods, float tvf, float cvf, float av, std::pair<float, float> lv, float dir, std::pair<float, float> pos);
	void DistanceSet(std::vector<std::pair<float, int>>& ds_between_robot);//���»����˼����
	void DistanceSet(std::set<std::pair<float, std::pair<int, int> > > & ds); //��ʼ�������빤��̨����


	/*------��rm�õ�-------*/
	void SetWait(int sw); //�������������Ҫ
	void Stop(); //ֹͣ�����Ҫ��rm���ʱ����
	void SetTask(std::vector<int> task); //�������񣬽���rm������task
	void SetRoad(std::vector<std::vector<std::pair<float, float> > > road); //����·��
	void ClearTask(); //������񣬿�����������׶�
	std::set<std::pair<float, std::pair<int, int> > > DistanceGet(); //�õ���ʼ�������빤��̨����
	
	/*-----��mc�õ�-----*/
	void SetPrecisionControl(bool flag); //��׼����ģʽ���ٶ�Ϊ4
	void SetPriorityPass(bool flag); // ��������ͨ��Ȩ
	void SetNextV(std::pair<float, float> next_v);
	void SetAvoidance(bool flag, int avoid_id); //���û����˽�����˳�����״̬
	void SetCanPark(bool flag);
	bool NeedStop(); // �Ƿ񷢳�ָͣ��
	
};


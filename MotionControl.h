#pragma once
#include<array>
#include<set>
#include<vector>
#include<string>
#include"IO.h"
#include"Robot.h"
#include"CongestionControl.h"
class MotionControl
{
	int m_RobotsNum = 4;

	float m_CollisionCheckVFactor = 0.8; //��ײ��� �ٶȼ�Ȩϵ�����ٶ�Խ��Ӧ���и������ײԤ����Χ
	float m_MinLimitDistace = 2; //��С��ײ������
	float m_CollisionTurnPower = 3; //����ת�����ȡ����M_PI
	bool m_TurnRight; //��������������

	float m_TrackDistance = 1.0;

	CongestionControl* cc;
	void WhoNeedAvoidance(Robot* robots);
	void TrackAvoidanceBack(int my_id, int first_go_id, Robot* robots);

	//���������������ĸ������˵��ٶȾ��󣬵�һ��Ϊ���ٶȣ��ڶ���Ϊ���ٶ�
	void Navigation(Robot* robots);

	//VO��ײ���
	bool CollisionCheck(Robot* robots, int my_id);
	//�Ƿ���VO������ж�
	bool InVO(std::pair<std::pair<float, float>, std::pair<float, float>> vo, std::pair<float, float> relative_v, std::pair<float, float> relative_pos);
	//�Ƿ����˴�����ײ��Χ�ļ��
	std::vector<int> CollisionDetection(Robot& robot);
	
	//ѭ��
	void TrackRoad(Robot& robot);
	std::pair<float, float> CalTrackRoadV(Robot& robot, std::pair<float, float> target_point);
	void KeepDistance(int my_id, Robot* others);
	void JumpPointSolver(Robot& r); //�����������ǰ������δ���ĵ������������������
	void CrossChasmSolver(Robot& r); //��������˴�Խ����ʱ�Ŀ�������
	int NearChasm(Robot& r);

	//���㺽���
	float AngleDiff(std::pair<float, float> target_pos, std::pair<float, float> robot_pos, float robot_dir);
	//�ǶȲ�ͨ�ð汾
	float AngleDiff(std::pair<float, float> relative_pos, std::pair<float, float> relative_v);
	
	bool LagCheck(Robot& r, Robot* others);

	// ������ٶȼ���
	float AnglePDcontrol(float angle_diff);
	// ������ٶȼ���
	float LinearPDcontrol(Robot& r, float angle);

public:
	// �����ٶȾ�������Order1��ʽ���������
	void MakeOrder(Robot* robots, std::vector<std::pair<std::string, std::pair<int, float> > >& Order_1, std::vector<std::pair<std::string, int> >& Order_2, CongestionControl* cc);

};


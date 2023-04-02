#pragma once
#include<array>
#include<set>
#include<vector>
#include<string>
#include"Robot.h"
class MotionControl
{
	int m_RobotsNum = 1;
	float m_MaxSpeed = 6;
	float m_WallSpeed = 1; //ǽ��Ŀ������
	float m_WallSlowDownDis = 1; //��ǰ���پ���

	float m_CollisionCheckVFactor = 0.8; //��ײ��� �ٶȼ�Ȩϵ�����ٶ�Խ��Ӧ���и������ײԤ����Χ
	float m_MinLimitDistace = 2; //��С��ײ������
	float m_CollisionTurnPower = 3; //����ת�����ȡ����M_PI
	bool m_TurnRight; //��������������

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

	//���㺽���
	float AngleDiff(std::pair<float, float> target_pos, std::pair<float, float> robot_pos, float robot_dir);
	//�ǶȲ�ͨ�ð汾
	float AngleDiff(std::pair<float, float> relative_pos, std::pair<float, float> relative_v);
	
	//ײǽ���
	bool HitWallCheck(std::pair<float, float> target_pos, std::pair<float, float> my_pos);

	// ������ٶȼ���
	float AnglePDcontrol(float angle_diff);
	// ������ٶȼ���
	float LinearPDcontrol(std::pair<float, float> target_pos, std::pair<float, float> my_pos, float angle);

public:
	// �����ٶȾ�������Order1��ʽ���������
	void MakeOrder(Robot* robots, std::vector<std::pair<std::string, std::pair<int, float> > >& Order_1, std::vector<std::pair<std::string, int> >& Order_2);

};

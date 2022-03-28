#pragma once
#include "config.h"
#include "GraspMethod.h"

class ControlSystem {
private:
	string arm_ip_;
	unsigned short arm_port_;
	string hand_ip_;
	unsigned short hand_port_;

	SOCKET clientSocketArm_;
	SOCKET clientSocketHand_;

	Eigen::Vector3d TranslationOfRobotbaseToCurrentHand_;			//当前机械手相对于机械臂坐标系
	Eigen::Vector3d EularAngleOfRobotbaseToCurrentHand_;
	vector<int> CurrentPosOfHand_;

	int CurrentGraspingObjectID = -1;
public:
	ControlSystem();
	ControlSystem(string arm_ip, unsigned short arm_port, string hand_ip_, unsigned short hand_port);
	void MoveAsGraspMethod(GraspMethod GspMethod);
	void MoveToInitState();
};
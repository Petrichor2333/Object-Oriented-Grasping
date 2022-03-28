#pragma once

#include "config.h"

enum Device { Arm, Hand, Undef };

class ControlCommand {
private:
	Device device_;
	string command_type_;
	vector<double> data_;
	int sleep_time_;
public:
	ControlCommand();
	ControlCommand(Device device, string command_type, vector<double> data);
	ControlCommand(Device device, string command_type, vector<double> data, int sleep_time);
	~ControlCommand();

	Device GetDevice();
	string GetCommandType();
	vector<double> GetData();
	int GetSleepTime();
};

class GraspMethod {
private:
	int GraspMethodNum_ = -1;
	vector<ControlCommand> controlCommands_;
	vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> moveTrajectory_;	//Eigen类作为vector元素需 aligned allocator : http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
	void SetGraspMethodNum(int GraspMethodNum);
	friend class GspObj_coffeeCup;
public:
	GraspMethod();
	GraspMethod(int GraspMethodNum);
	~GraspMethod();

	vector<ControlCommand> GetControlCommands();
	vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> GetMoveTrajectory();	//Eigen类作为vector元素需 aligned allocator : http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html
	void PushBackControlCommand(ControlCommand CtrlCommand);
	void PushBackTrajectoryPoint(Eigen::Isometry3d Isometry);
	int ReturnGraspMethodNum();
};
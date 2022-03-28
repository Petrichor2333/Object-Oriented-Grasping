#pragma once
#include "pch.h"
#include "config.h"
#include "GraspMethod.h"

//--------------------ControlCommand--------------------

ControlCommand::ControlCommand() {
	device_ = Undef;
}

ControlCommand::ControlCommand(Device device, string command_type, vector<double> data) {
	device_ = device;
	command_type_ = command_type;
	data_ = data;
	sleep_time_ = 0;
}

ControlCommand::ControlCommand(Device device, string command_type, vector<double> data, int sleep_time) {
	device_ = device;
	command_type_ = command_type;
	data_ = data;
	sleep_time_ = sleep_time;
}

ControlCommand::~ControlCommand() {

}

Device ControlCommand::GetDevice() {
	return device_;
}
string ControlCommand::GetCommandType() {
	return command_type_;
}
vector<double> ControlCommand::GetData() {
	return data_;
}
int ControlCommand::GetSleepTime() {
	return sleep_time_;
}

//--------------------GraspMethod--------------------

GraspMethod::GraspMethod() {

}
GraspMethod::GraspMethod(int GraspMethodNum) {
	GraspMethodNum_ = GraspMethodNum;
}

GraspMethod::~GraspMethod() {

}

vector<ControlCommand> GraspMethod::GetControlCommands() {
	return controlCommands_;
}
vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> GraspMethod::GetMoveTrajectory() {
	return moveTrajectory_;
}

void GraspMethod::PushBackControlCommand(ControlCommand CtrlCommand) {
	controlCommands_.push_back(CtrlCommand);
}
void GraspMethod::PushBackTrajectoryPoint(Eigen::Isometry3d Isometry) {
	moveTrajectory_.push_back(Isometry);
}

int GraspMethod::ReturnGraspMethodNum() 
{
	return GraspMethodNum_;
}
void GraspMethod::SetGraspMethodNum(int GraspMethodNum)
{
	GraspMethodNum_ = GraspMethodNum;
}
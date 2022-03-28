#include "pch.h"
#include "GraspObject.h"
#include "../CoordinateCalibration.h"

GraspObject::GraspObject() {
	id_ = -1;
	center_ << 0.0, 0.0, 0.0;
	X_axis_ << 1.0, 0.0, 0.0;
	X_axis_ << 0.0, 1.0, 0.0;
	X_axis_ << 0.0, 0.0, 1.0;
	RobotbaseToGspObject_ = Eigen::Isometry3d::Identity();
	RobotbaseToTablebase_= FindIsometryofRobotbasetoTablebase();
}

GraspObject::~GraspObject() {

}

vector<GraspMethod> GraspObject::GetGraspMethods() {
	return GraspMethods_;
}

Eigen::Isometry3d GraspObject::GetIsoOfWorldToGspObject() {
	return RobotbaseToGspObject_;
}

int GraspObject::GetGraspObjectId() {
	return id_;
}
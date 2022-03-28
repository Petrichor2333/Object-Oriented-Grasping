#include "pch.h"
#include "GspObj_cokecan.h"


GspObj_cokecan::GspObj_cokecan() : GraspObject() {

}

GspObj_cokecan::GspObj_cokecan(int id, Eigen::Vector3d center, Eigen::Vector3d Z_axis) {
	id_ = id;
	double epsilon = 0.05;
	FindWorldToObj(center, Z_axis, epsilon, center_, X_axis_, Y_axis_, Z_axis_, RobotbaseToGspObject_);

	GraspMethod GspMethod1 = FindGraspMethod1(RobotbaseToGspObject_);
	cout << (GspMethod1.GetControlCommands())[1].GetCommandType() << endl;
	GraspMethods_.push_back(GspMethod1);
	GraspMethod GspMethod2 = FindGraspMethod2(RobotbaseToGspObject_);
	GraspMethods_.push_back(GspMethod2);
}

void GspObj_cokecan::FindWorldToObj(Eigen::Vector3d center, Eigen::Vector3d Z_axis, double epsilon, Eigen::Vector3d &center_, Eigen::Vector3d &X_axis_, Eigen::Vector3d &Y_axis_, Eigen::Vector3d &Z_axis_, Eigen::Isometry3d &WorldToGspObject_) {
	center_ = center;
	Z_axis_ = Z_axis;
	Z_axis_.normalize();														//确定Z轴

	double NormCrossBetweenZaxisAndZ = Z_axis_.dot(Eigen::Vector3d(0, 0, 1)); //通过NormCrossBetweenZaxisAndZ判断建系方式
	if (NormCrossBetweenZaxisAndZ <= -1 + epsilon || NormCrossBetweenZaxisAndZ >= 1 - epsilon) {
		Z_axis_ = Eigen::Vector3d(0, 0, 1);										//修正Z轴
		Y_axis_ << center[0], center[1], 0;
	}
	else {
		Y_axis_ = -Z_axis.cross(Eigen::Vector3d(0, 0, 1));
	}
	assert(0 == Z_axis_.dot(Y_axis_));
	Y_axis_.normalize();														//确定Y轴
	X_axis_ = Y_axis_.cross(Z_axis);
	X_axis_.normalize();														//确定X轴

	WorldToGspObject_ = Eigen::Isometry3d::Identity();
	Eigen::Matrix3d rotation_matrix_WorldToObj;
	rotation_matrix_WorldToObj << X_axis_, Y_axis_, Z_axis_;
	WorldToGspObject_.rotate(rotation_matrix_WorldToObj);
	WorldToGspObject_.pretranslate(center_);									//计算得到目标物体在世界坐标系下的其次变换矩阵
}

GraspMethod GspObj_cokecan::FindGraspMethod1(Eigen::Isometry3d WorldToGspObject) {				//抓取方法1
	Eigen::Matrix3d rotation_matrix_ObjToHand;
	Eigen::Vector3d center_matrix_ObjToHand;
	rotation_matrix_ObjToHand << 1, 0, 0, 0, 0, 1, 0, -1, 0;
	center_matrix_ObjToHand << -0.05, 0, 0.05;									//机械手在物体坐标系下的齐次变换矩阵， 需要调整参数

	Eigen::Isometry3d ObjToHand = Eigen::Isometry3d::Identity();
	ObjToHand.rotate(rotation_matrix_ObjToHand);
	ObjToHand.pretranslate(center_matrix_ObjToHand);

	Eigen::Isometry3d RobotbaseToHand = WorldToGspObject * ObjToHand;

	Eigen::Vector3d Z_axis_CoordInWorld = (RobotbaseToHand.rotation()).col(2);
	Eigen::Isometry3d preRobotbaseToHand = RobotbaseToHand;
	preRobotbaseToHand.pretranslate(-0.1*Z_axis_CoordInWorld);

	Eigen::Vector3d EulerAngle_RobotbaseToHand = (RobotbaseToHand.rotation()).eulerAngles(2, 1, 0);
	Eigen::Vector3d Center_RobotbaseToHand = RobotbaseToHand.translation();
	Eigen::Vector3d EulerAngle_preRobotbaseToHand = (preRobotbaseToHand.rotation()).eulerAngles(2, 1, 0);
	Eigen::Vector3d Center_preRobotbaseToHand = preRobotbaseToHand.translation();

	GraspMethod GspMethod;
	GspMethod.PushBackTrajectoryPoint(preRobotbaseToHand);
	GspMethod.PushBackTrajectoryPoint(RobotbaseToHand);

	vector<double> HandParam1 = { 0, 0, 0, 0, 0, 0 };
	vector<double> ArmParam2 = { Center_preRobotbaseToHand[0], Center_preRobotbaseToHand[1], Center_preRobotbaseToHand[2],
								EulerAngle_preRobotbaseToHand[0], EulerAngle_preRobotbaseToHand[1], EulerAngle_preRobotbaseToHand[2] };
	vector<double> ArmParam3 = { Center_RobotbaseToHand[0], Center_RobotbaseToHand[1], Center_RobotbaseToHand[2],
								EulerAngle_RobotbaseToHand[0], EulerAngle_RobotbaseToHand[1], EulerAngle_RobotbaseToHand[2] };
	vector<double> HandParam4 = { 0, 0, 0, 0, 0, 0 };
	ControlCommand CtrlCommand1(Hand, "SetCurPos", HandParam1);
	GspMethod.PushBackControlCommand(CtrlCommand1);
	ControlCommand CtrlCommand2(Arm, "move_joint", ArmParam2);
	GspMethod.PushBackControlCommand(CtrlCommand2);
	ControlCommand CtrlCommand3(Arm, "move_line", ArmParam3);
	GspMethod.PushBackControlCommand(CtrlCommand3);
	ControlCommand CtrlCommand4(Hand, "SetCurPos", HandParam4);
	GspMethod.PushBackControlCommand(CtrlCommand4);

	return GspMethod;
}

GraspMethod GspObj_cokecan::FindGraspMethod2(Eigen::Isometry3d WorldToGspObject) {				//抓取方法2
	Eigen::Matrix3d rotation_matrix_ObjToHand;
	Eigen::Vector3d center_matrix_ObjToHand;
	rotation_matrix_ObjToHand << 1, 0, 0, 0, 0, 1, 0, -1, 0;
	center_matrix_ObjToHand << -0.05, 0, 0.05;									//机械手在物体坐标系下的齐次变换矩阵， 需要调整参数

	Eigen::Isometry3d ObjToHand = Eigen::Isometry3d::Identity();
	ObjToHand.rotate(rotation_matrix_ObjToHand);
	ObjToHand.pretranslate(center_matrix_ObjToHand);

	Eigen::Isometry3d RobotbaseToHand = WorldToGspObject * ObjToHand;

	Eigen::Vector3d Z_axis_CoordInWorld = (RobotbaseToHand.rotation()).col(2);
	Eigen::Isometry3d preRobotbaseToHand = RobotbaseToHand;
	preRobotbaseToHand.pretranslate(0.1*Eigen::Vector3d(0, 0, 1));

	Eigen::Vector3d EulerAngle_RobotbaseToHand = (RobotbaseToHand.rotation()).eulerAngles(2, 1, 0);
	Eigen::Vector3d Center_RobotbaseToHand = RobotbaseToHand.translation();
	Eigen::Vector3d EulerAngle_preRobotbaseToHand = (preRobotbaseToHand.rotation()).eulerAngles(2, 1, 0);
	Eigen::Vector3d Center_preRobotbaseToHand = preRobotbaseToHand.translation();

	GraspMethod GspMethod;
	GspMethod.PushBackTrajectoryPoint(preRobotbaseToHand);
	GspMethod.PushBackTrajectoryPoint(RobotbaseToHand);

	vector<double> HandParam1 = { 1000, 1000, 1000, 1000, 1000, 600 };
	vector<double> ArmParam2 = { Center_preRobotbaseToHand[0], Center_preRobotbaseToHand[1], Center_preRobotbaseToHand[2],
								EulerAngle_preRobotbaseToHand[0], EulerAngle_preRobotbaseToHand[1], EulerAngle_preRobotbaseToHand[2] };
	vector<double> ArmParam3 = { Center_RobotbaseToHand[0], Center_RobotbaseToHand[1], Center_RobotbaseToHand[2],
								EulerAngle_RobotbaseToHand[0], EulerAngle_RobotbaseToHand[1], EulerAngle_RobotbaseToHand[2] };
	vector<double> HandParam4 = { 0, 0, 0, 0, 0, 0 };
	ControlCommand CtrlCommand1(Hand, "SetCurPos", HandParam1);
	GspMethod.PushBackControlCommand(CtrlCommand1);
	ControlCommand CtrlCommand2(Arm, "move_joint", ArmParam2);
	GspMethod.PushBackControlCommand(CtrlCommand2);
	ControlCommand CtrlCommand3(Arm, "move_line", ArmParam3);
	GspMethod.PushBackControlCommand(CtrlCommand3);
	ControlCommand CtrlCommand4(Hand, "SetCurPos", HandParam4, 500);
	GspMethod.PushBackControlCommand(CtrlCommand4);

	return GspMethod;
}
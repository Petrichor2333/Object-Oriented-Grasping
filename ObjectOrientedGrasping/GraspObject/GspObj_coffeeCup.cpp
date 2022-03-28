#include "pch.h"
#include "GspObj_coffeeCup.h"
#include "../CoordinateCalibration.h"




GspObj_coffeeCup::GspObj_coffeeCup() : GraspObject() {

}

GspObj_coffeeCup::GspObj_coffeeCup(int id, TypeOfCoffeeCup CoffeeCupeType, Eigen::Vector3d center_in_table, Eigen::Matrix3d rotation_matrix_tableToObj)
{
	id_ = id;
	coffee_cup_type_ = CoffeeCupeType;

	//物体的基础信息
	English_name_ = "coffee cup";
	Chinese_name_ = "咖啡杯";
	made_of_ = Ceramics;
	net_weight_ = 300;
	volume_ = 120;

	//物体的形状信息
	shape_ = CircularTruncatedCone;
	radius_of_big_Circle_ = 1.5;
	radius_of_small_Circle_ = 2;
	length_ = 5;

#ifndef NDEBUG

	cout << "Rotation matrix of robot base to table base:" << endl;
	cout << RobotbaseToTablebase_.rotation();
	cout << endl << endl << "--------------" << endl << endl;
	cout << "Translation vector of robot base to table base:" << endl;
	cout << RobotbaseToTablebase_.translation();
	cout << endl << endl << "--------------" << endl << endl;

#endif
	//物体的位置信息
	Eigen::Isometry3d TablebasetoObj = Eigen::Isometry3d::Identity();
	TablebasetoObj.rotate(rotation_matrix_tableToObj);
	TablebasetoObj.pretranslate(center_in_table);
	RobotbaseToGspObject_ = RobotbaseToTablebase_ * TablebasetoObj;
	Eigen::Matrix3d rotation_matrix_RobotbaseToObj = RobotbaseToGspObject_.rotation();
	Eigen::Vector3d center_in_Robotbase = RobotbaseToGspObject_.translation();
	center_ = center_in_Robotbase;
	X_axis_ = rotation_matrix_RobotbaseToObj.col(0);
	Y_axis_ = rotation_matrix_RobotbaseToObj.col(1);
	Z_axis_ = rotation_matrix_RobotbaseToObj.col(2);

	//
	volume_of_liquid_ = 0;

	//物体的抓取方法
	GraspMethod GspMethod1 = FindGraspMethod0(RobotbaseToGspObject_, volume_of_liquid_);
	GraspMethods_.push_back(GspMethod1);
}


GspObj_coffeeCup::~GspObj_coffeeCup()
{
}

Eigen::Isometry3d GspObj_coffeeCup::ReturnIsoOfObjToHandOfGraspMethod0()
{
	Eigen::Matrix3d rotation_matrix_ObjToHand;
	Eigen::Vector3d center_matrix_ObjToHand;
	Eigen::Isometry3d ObjToHand = Eigen::Isometry3d::Identity();
	if (coffee_cup_type_ == CoffeeCupType1) {
		
		rotation_matrix_ObjToHand <<	0.232639096, 0.1456156088, -0.9616003044,
										0.724185158, 0.634033968, 0.2712135401,
										0.6491801814, -0.7594715412, 0.04204842643;
		center_matrix_ObjToHand << 0.1638490211, -0.1033644605, 0.01221785997;									//机械手在物体坐标系下的齐次变换矩阵， 需要调整参数

	/*	rotation_matrix_ObjToHand << 0, 1, 0, 1, 0, 0, 0, 0, -1;
		center_matrix_ObjToHand << 0.02,-0.03,-0.01;*/
		
		ObjToHand.rotate(rotation_matrix_ObjToHand);
		ObjToHand.pretranslate(center_matrix_ObjToHand);
	}
	else if (coffee_cup_type_ == CoffeeCupType2) {
		rotation_matrix_ObjToHand <<	0.2390929015,	0.1122557948,	-0.964485988,
										0.6256601217,	0.7417938485,	0.2414359097,
										0.7425523528,	-0.6611660328,	0.1071236693;									//机械手在物体坐标系下的齐次变换矩阵， 需要调整参数
		center_matrix_ObjToHand << 0.1602749849,	-0.0918496745,	-0.008360262425;
		ObjToHand.rotate(rotation_matrix_ObjToHand);
		ObjToHand.pretranslate(center_matrix_ObjToHand);
	}
	else if (coffee_cup_type_ == CoffeeCupType3) {
		rotation_matrix_ObjToHand <<	0.08134409685,	0.02245912863,	-0.996433001,
										0.7312088389,	0.6780209626,	0.07497471736,
										0.6772863293,	-0.7346993683,	0.03873068948;									//机械手在物体坐标系下的齐次变换矩阵， 需要调整参数
		center_matrix_ObjToHand <<	0.1715593979,	-0.06115476165,	0.03451488602;
		ObjToHand.rotate(rotation_matrix_ObjToHand);
		ObjToHand.pretranslate(center_matrix_ObjToHand);
	}
	else {
		
	}
	

	
	return ObjToHand;
}

vector<ControlCommand> GspObj_coffeeCup::ReturnHandCtrlCommandOfGraspMethod0()
{
	vector<ControlCommand> HandControlCommand;
	if (coffee_cup_type_ == CoffeeCupType1) {
		vector<double> HandParam4 = { 2000, 2000, 1170, 1140, 1200, 2000 };
		ControlCommand CtrlCommand4(Hand, "SetCurPos", HandParam4, 500);
		HandControlCommand.push_back(CtrlCommand4);
		vector<double> HandParam5 = { 2000, 2000, 100, 100, 50, 2000 };
		ControlCommand CtrlCommand5(Hand, "SetSpeed", HandParam5, 100);
		HandControlCommand.push_back(CtrlCommand5);
		vector<double> HandParam6 = { 300, 300, 800, 800, 800, 300 };
		ControlCommand CtrlCommand6(Hand, "SetThrForce", HandParam6, 100);
		HandControlCommand.push_back(CtrlCommand6);
		vector<double> HandParam7 = { 2000, 2000, 1150, 1120, 1190, 2000 };
		ControlCommand CtrlCommand7(Hand, "SetCurPos", HandParam7, 800);
		HandControlCommand.push_back(CtrlCommand7);
	}
	else if (coffee_cup_type_ == CoffeeCupType2) {
		vector<double> HandParam4 = { 2000, 2000, 1125, 1080, 1230, 2000 };
		ControlCommand CtrlCommand4(Hand, "SetCurPos", HandParam4, 500);
		HandControlCommand.push_back(CtrlCommand4);
		vector<double> HandParam5 = { 2000, 2000, 100, 100, 50, 2000 };
		ControlCommand CtrlCommand5(Hand, "SetSpeed", HandParam5, 100);
		HandControlCommand.push_back(CtrlCommand5);
		vector<double> HandParam6 = { 300, 300, 1000, 1000, 1000, 300 };
		ControlCommand CtrlCommand6(Hand, "SetThrForce", HandParam6, 100);
		HandControlCommand.push_back(CtrlCommand6);
		vector<double> HandParam7 = { 2000, 2000, 925, 1020, 1130, 2000 };
		ControlCommand CtrlCommand7(Hand, "SetCurPos", HandParam7, 800);
		HandControlCommand.push_back(CtrlCommand7);
	}
	else if (coffee_cup_type_ == CoffeeCupType3) {
		vector<double> HandParam4 = { 2000, 2000, 915, 865, 1250, 2000 };
		ControlCommand CtrlCommand4(Hand, "SetCurPos", HandParam4, 500);
		HandControlCommand.push_back(CtrlCommand4);
		vector<double> HandParam5 = { 2000, 2000, 100, 100, 50, 2000 };
		ControlCommand CtrlCommand5(Hand, "SetSpeed", HandParam5, 100);
		HandControlCommand.push_back(CtrlCommand5);
		vector<double> HandParam6 = { 300, 300, 800, 800, 800, 300 };
		ControlCommand CtrlCommand6(Hand, "SetThrForce", HandParam6, 100);
		HandControlCommand.push_back(CtrlCommand6);
		vector<double> HandParam7 = { 2000, 2000, 855, 825, 1220, 2000 };
		ControlCommand CtrlCommand7(Hand, "SetCurPos", HandParam7, 800);
		HandControlCommand.push_back(CtrlCommand7);
	}
	else {
	
	}
	return HandControlCommand;
}

GraspMethod GspObj_coffeeCup::FindGraspMethod0(Eigen::Isometry3d RobotbaseToGspObject, double volume_of_liquid)		//抓取方法 number 0
{				
	Eigen::Isometry3d ObjToHand = ReturnIsoOfObjToHandOfGraspMethod0();

#ifndef NDEBUG
	cout << "Rotation matrix of object to hand is:" << endl;
	cout << ObjToHand.rotation();
	cout << endl << endl << "------------------------" << endl << endl;
	cout << "Translation vector of object to hand is : " << endl;
	cout << ObjToHand.translation();
#endif // !NDEBUG

	

	Eigen::Isometry3d RobotbaseToHand = RobotbaseToGspObject * ObjToHand;
#ifndef NDEBUG
	cout << "Rotation matrix of Robot Base to Hand:" << endl;
	cout << RobotbaseToHand.rotation();
	cout << endl << endl << "--------------" << endl << endl;
	cout << "Translation vector of Robot Base to Hand:" << endl;
	cout << RobotbaseToHand.translation();
	cout << endl << endl << "--------------" << endl << endl;
#endif

	Eigen::Vector3d X_axis_CoordInWorld = (RobotbaseToHand.rotation()).col(0);
	Eigen::Vector3d Y_axis_CoordInWorld = (RobotbaseToHand.rotation()).col(1);
	Eigen::Vector3d Z_axis_CoordInWorld = (RobotbaseToHand.rotation()).col(2);	
	Eigen::Isometry3d preRobotbaseToHand = RobotbaseToHand;
	preRobotbaseToHand.pretranslate(-0.15*Z_axis_CoordInWorld);
	preRobotbaseToHand.pretranslate(-0.03*X_axis_CoordInWorld);
	preRobotbaseToHand.pretranslate(-0.03*Y_axis_CoordInWorld);

	Eigen::Vector3d EulerAngle_RobotbaseToHand = (RobotbaseToHand.rotation()).eulerAngles(2, 1, 0);
	EulerAngle_RobotbaseToHand = EulerAngle_RobotbaseToHand * 180 / PI;
	Eigen::Vector3d Center_RobotbaseToHand = RobotbaseToHand.translation();					
	Eigen::Vector3d EulerAngle_preRobotbaseToHand = (preRobotbaseToHand.rotation()).eulerAngles(2, 1, 0);
	EulerAngle_preRobotbaseToHand = EulerAngle_preRobotbaseToHand * 180 / PI;
	Eigen::Vector3d Center_preRobotbaseToHand = preRobotbaseToHand.translation();

	int GraspMethodNum = 0;
	GraspMethod GspMethod(GraspMethodNum);
	GspMethod.PushBackTrajectoryPoint(preRobotbaseToHand);
	GspMethod.PushBackTrajectoryPoint(RobotbaseToHand);		
	
	vector<double> HandParam1 = { 2000, 2000, 2000, 2000, 1300, 2000 };
	ControlCommand CtrlCommand1(Hand, "SetCurPos", HandParam1,200);
	GspMethod.PushBackControlCommand(CtrlCommand1);
	vector<double> ArmParam2 = { Center_preRobotbaseToHand[0], Center_preRobotbaseToHand[1], Center_preRobotbaseToHand[2],
								EulerAngle_preRobotbaseToHand[0], EulerAngle_preRobotbaseToHand[1], EulerAngle_preRobotbaseToHand[2] };
	ControlCommand CtrlCommand2(Arm, "MoveJoint", ArmParam2);
	GspMethod.PushBackControlCommand(CtrlCommand2);
	vector<double> ArmParam3 = { Center_RobotbaseToHand[0], Center_RobotbaseToHand[1], Center_RobotbaseToHand[2],
								EulerAngle_RobotbaseToHand[0], EulerAngle_RobotbaseToHand[1], EulerAngle_RobotbaseToHand[2] };
	ControlCommand CtrlCommand3(Arm, "MoveLine", ArmParam3);
	GspMethod.PushBackControlCommand(CtrlCommand3);
	
	vector<ControlCommand> HandControlCommand = ReturnHandCtrlCommandOfGraspMethod0();
	for (int i = 0;i < HandControlCommand.size();i++) {
		GspMethod.PushBackControlCommand(HandControlCommand[i]);
	}

	return GspMethod;
}


GraspMethod GspObj_coffeeCup::MoveObjectToDesignatedPosition(Eigen::Isometry3d	TableBaseToDesignatedPositionOfObj)
{
	Eigen::Isometry3d RobotBaseToDesignatedPositionOfObj = RobotbaseToTablebase_ * TableBaseToDesignatedPositionOfObj;
	Eigen::Isometry3d ObjToHand = ReturnIsoOfObjToHandOfGraspMethod0();
	Eigen::Isometry3d RobotbaseToHand = RobotBaseToDesignatedPositionOfObj * ObjToHand;

	Eigen::Vector3d EulerAngle_RobotbaseToHand = (RobotbaseToHand.rotation()).eulerAngles(2, 1, 0);
	EulerAngle_RobotbaseToHand = EulerAngle_RobotbaseToHand * 180 / PI;
	Eigen::Vector3d Center_RobotbaseToHand = RobotbaseToHand.translation();

	int GraspMethodNum = 0;
	GraspMethod GspMethod(GraspMethodNum);
	//GspMethod.PushBackTrajectoryPoint(preRobotbaseToHand);
	//GspMethod.PushBackTrajectoryPoint(RobotbaseToHand);
	vector<double> ArmParam1 = { Center_RobotbaseToHand[0], Center_RobotbaseToHand[1], Center_RobotbaseToHand[2],
								EulerAngle_RobotbaseToHand[0], EulerAngle_RobotbaseToHand[1], EulerAngle_RobotbaseToHand[2] };
	ControlCommand CtrlCommand1(Arm, "MoveLine", ArmParam1);
	GspMethod.PushBackControlCommand(CtrlCommand1);

	RobotbaseToGspObject_ = TableBaseToDesignatedPositionOfObj;

	return GspMethod;
}


GraspMethod GspObj_coffeeCup::PutdownObjectToDesignatedPosition(Eigen::Isometry3d	TableBaseToDesignatedPositionOfObj)
{
	Eigen::Isometry3d RobotBaseToDesignatedPositionOfObj = RobotbaseToTablebase_ * TableBaseToDesignatedPositionOfObj;
	Eigen::Isometry3d ObjToHand = ReturnIsoOfObjToHandOfGraspMethod0();
	Eigen::Isometry3d RobotbaseToHand = RobotBaseToDesignatedPositionOfObj * ObjToHand;

	Eigen::Vector3d X_axis_CoordInWorld = (RobotbaseToHand.rotation()).col(0);
	Eigen::Vector3d Y_axis_CoordInWorld = (RobotbaseToHand.rotation()).col(1);
	Eigen::Vector3d Z_axis_CoordInWorld = (RobotbaseToHand.rotation()).col(2);

	Eigen::Vector3d EulerAngle_RobotbaseToHand = (RobotbaseToHand.rotation()).eulerAngles(2, 1, 0);
	EulerAngle_RobotbaseToHand = EulerAngle_RobotbaseToHand * 180 / PI;
	Eigen::Vector3d Center_RobotbaseToHand = RobotbaseToHand.translation();
	Eigen::Vector3d EulerAngle_preRobotbaseToHand = EulerAngle_RobotbaseToHand;
	Eigen::Vector3d prePutdown(0, 0, 0.1);
	Eigen::Vector3d Center_preRobotbaseToHand = Center_RobotbaseToHand + prePutdown;
	Eigen::Vector3d EulerAngle_AfterRobotbaseToHand = EulerAngle_RobotbaseToHand;
	Eigen::Vector3d Center_AfterRobotbaseToHand = Center_RobotbaseToHand;
	Center_AfterRobotbaseToHand = Center_AfterRobotbaseToHand - 0.1*Z_axis_CoordInWorld - 0.02*Y_axis_CoordInWorld - 0.02*X_axis_CoordInWorld;

	int GraspMethodNum = 0;
	GraspMethod GspMethod(GraspMethodNum);
	//GspMethod.PushBackTrajectoryPoint(preRobotbaseToHand);
	//GspMethod.PushBackTrajectoryPoint(RobotbaseToHand);
	vector<double> ArmParam1 = { Center_preRobotbaseToHand[0], Center_preRobotbaseToHand[1], Center_preRobotbaseToHand[2],
								EulerAngle_preRobotbaseToHand[0], EulerAngle_preRobotbaseToHand[1], EulerAngle_preRobotbaseToHand[2] };
	ControlCommand CtrlCommand1(Arm, "MoveLine", ArmParam1);
	GspMethod.PushBackControlCommand(CtrlCommand1);
	vector<double> ArmParam2 = { Center_RobotbaseToHand[0], Center_RobotbaseToHand[1], Center_RobotbaseToHand[2],
								EulerAngle_RobotbaseToHand[0], EulerAngle_RobotbaseToHand[1], EulerAngle_RobotbaseToHand[2] };
	ControlCommand CtrlCommand2(Arm, "MoveLine", ArmParam2);
	GspMethod.PushBackControlCommand(CtrlCommand2);
	vector<double> HandParam3 = { 2000, 2000, 2000, 2000, 2000, 2000 };
	ControlCommand CtrlCommand3(Hand, "SetSpeed", HandParam3,100);
	GspMethod.PushBackControlCommand(CtrlCommand3);
	vector<double> HandParam4 = { 2000, 2000, 2000, 2000, 1300, 2000 };
	ControlCommand CtrlCommand4(Hand, "SetCurPos", HandParam4, 500);
	GspMethod.PushBackControlCommand(CtrlCommand4);
	vector<double> ArmParam5 = { Center_AfterRobotbaseToHand[0], Center_AfterRobotbaseToHand[1], Center_AfterRobotbaseToHand[2],
								EulerAngle_AfterRobotbaseToHand[0], EulerAngle_AfterRobotbaseToHand[1], EulerAngle_AfterRobotbaseToHand[2] };
	ControlCommand CtrlCommand5(Arm, "MoveLine", ArmParam5);
	GspMethod.PushBackControlCommand(CtrlCommand5);

	RobotbaseToGspObject_ = TableBaseToDesignatedPositionOfObj;

	return GspMethod;
}


int GspObj_coffeeCup::AddLiquidtoCoffeeCup(double volume_of_liquid) {
	int action_config=0;
	volume_of_liquid_ = volume_of_liquid_+ volume_of_liquid;
	if (volume_of_liquid_ > volume_) {
		cout << "The liquid added exceeds the capacity, coffe cup is full of liquid";
		volume_of_liquid_ = volume_;
		action_config = 1;
	}
	GraspMethod GspMethod1 = FindGraspMethod0(RobotbaseToGspObject_, volume_of_liquid_);
	GraspMethods_.pop_back();
	GraspMethods_.push_back(GspMethod1);
	return action_config;
}

int GspObj_coffeeCup::PourOutLiquidtoCoffeeCup(double volume_of_liquid) {
	int action_config = 0;
	if (volume_of_liquid_ - volume_of_liquid < 0) {
		cout << "There is not enough liquid in coffee cup, just pour out all the liquid";
		volume_of_liquid_ = 0;
		int action_config = 1;
	}
	else {
		volume_of_liquid_ = volume_of_liquid_ - volume_of_liquid;
	}
	GraspMethod GspMethod1 = FindGraspMethod0(RobotbaseToGspObject_, volume_of_liquid_);
	GraspMethods_.pop_back();
	GraspMethods_.push_back(GspMethod1);
	return action_config;
}
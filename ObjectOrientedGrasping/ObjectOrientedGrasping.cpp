// ObjectOrientedGrasping.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include "config.h"
#include "AllGraspObjects.h"
#include "ControlSystem.h"
#include "CoordinateCalibration.h"

int main()
{

	Eigen::Isometry3d FindIsoByTranslationAndEularAngle(Eigen::Vector3d Trans, Eigen::Vector3d EA);

	//Eigen::Vector3d center_cokecan(0.5, 0.1, 0);
	//Eigen::Vector3d Z_axis_cokecan(0, 0, 1);

	Eigen::Vector3d center_coffeeCup_in_table(0.235, 0.335, 0.108);	
	Eigen::Vector3d EAofTabletoCoffeeCup(0, 0, 0);
	Eigen::Isometry3d TablebaseToCoffeeCup = FindIsoByTranslationAndEularAngle(center_coffeeCup_in_table, EAofTabletoCoffeeCup);

#ifndef NDEBUG
	cout << "Rotation matrix of table base to coffee cup:" << endl;
	cout << TablebaseToCoffeeCup.rotation();
	cout << endl << endl << "--------------" << endl << endl;
	cout << "Translation vector of table base to coffee cup:" << endl;
	cout << TablebaseToCoffeeCup.translation();
	cout << endl << endl << "--------------" << endl << endl;
#endif

	
	//GspObj_cokecan obj_cokecan(1, center_cokecan, Z_axis_cokecan);
	GspObj_coffeeCup obj_coffeCup(1, CoffeeCupType3, TablebaseToCoffeeCup.translation(), TablebaseToCoffeeCup.rotation());

	//vector<GraspMethod> GspMethos_cokecan = obj_cokecan.GetGraspMethods();
	vector<GraspMethod> GspMethos_coffeeCup = obj_coffeCup.GetGraspMethods();

	cout << "Start control system" << endl;
	ControlSystem CtrlSystem("192.168.1.101", 3001, "127.0.0.1", 3000);
	cout << "Control system ready" << endl << endl;

	cout << "Move arm and hand to initial state." << endl << endl;
	CtrlSystem.MoveToInitState();
	
	//cout << "Start to grasp cokecan:" << endl;
	//CtrlSystem.MoveAsGraspMethod(GspMethos_cokecan[0]);
	//cout << endl << endl;
	cout << "Start to grasp coffee cup:" << endl;
	CtrlSystem.MoveAsGraspMethod(GspMethos_coffeeCup[0]);
	cout << endl << endl;

	Eigen::Vector3d RaiseVec(0,0,0.1);
	Eigen::Vector3d center_coffeeCup_in_table_raised = center_coffeeCup_in_table + RaiseVec;
	Eigen::Isometry3d TablebaseToCoffeeCupRaised = FindIsoByTranslationAndEularAngle(center_coffeeCup_in_table_raised, EAofTabletoCoffeeCup);
	GraspMethod RaiseCoffeeCup = obj_coffeCup.MoveObjectToDesignatedPosition(TablebaseToCoffeeCupRaised);
	cout << "Start to raise coffee cup:" << endl;
	CtrlSystem.MoveAsGraspMethod(RaiseCoffeeCup);
	cout << endl << endl;

	Eigen::Vector3d center_coffeeCup_in_table_putdown(0.185, 0.535, 0.108);
	Eigen::Isometry3d TablebaseToCoffeeCupPutdown = FindIsoByTranslationAndEularAngle(center_coffeeCup_in_table_putdown, EAofTabletoCoffeeCup);
	GraspMethod PutdownCoffeeCup = obj_coffeCup.PutdownObjectToDesignatedPosition(TablebaseToCoffeeCupPutdown);
	cout << "Start to put down coffee cup:" << endl;
	CtrlSystem.MoveAsGraspMethod(PutdownCoffeeCup);
	cout << endl << endl;

	cout << "Move arm and hand to initial state." << endl << endl;
	CtrlSystem.MoveToInitState();

	return 0;
}

Eigen::Isometry3d FindIsoByTranslationAndEularAngle(Eigen::Vector3d Trans, Eigen::Vector3d EA)
{
	Eigen::Matrix3d rotation_matrix;
	//rotation_matrix_TabletoCoffeeCup << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	rotation_matrix = Eigen::AngleAxisd(EA[0], Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(EA[1], Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(EA[2], Eigen::Vector3d::UnitX());
	Eigen::Isometry3d Iso = Eigen::Isometry3d::Identity();
	Iso.rotate(rotation_matrix);
	Iso.pretranslate(Trans);
	return Iso;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门提示: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件

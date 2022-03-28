#include "pch.h"
#include <Eigen/SVD>
#include "config.h"
#include "CoordinateCalibration.h"
#define PI 3.14159265359

using namespace std;
//using namespace Eigen;
//using namespace Eigen::internal;
//using namespace Eigen::Architecture;

Eigen::Isometry3d FindIsometryofRobotbasetoTablebase()
{
	Eigen::Matrix3d CovarMarix;

	CovarMarix << -614.27,	-1235.76,	214.61,
				727.67,		-1169.14,	8.94,
				-151.37,	-150.11,	110.91;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(CovarMarix, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
	//Eigen::Matrix3d  S = U.inverse() * CovarMarix * (V.transpose()).inverse();

	Eigen::Matrix3d rotation_matrix_RobotbaseToTablebase = U * V.transpose();

	Eigen::Vector3d translate_vector_RobotbaseToTable;
	translate_vector_RobotbaseToTable<< 77.411846,-62.791904,-9.1199879;
	translate_vector_RobotbaseToTable = 0.01*translate_vector_RobotbaseToTable;			//单位换算，机械臂使用的单位为m，CovarMarix单位为cm

	Eigen::Isometry3d RobotbaseToTablebase = Eigen::Isometry3d::Identity();
	RobotbaseToTablebase.rotate(rotation_matrix_RobotbaseToTablebase);
	RobotbaseToTablebase.pretranslate(translate_vector_RobotbaseToTable);

	return RobotbaseToTablebase;
}
/*
Eigen::Isometry3d FindIsometryofRobotbasetoTablebase()
{
	Eigen::Matrix3d rotation_matrix_RobotbaseToTablebase;
	Eigen::Vector3d EAofRobotbaseToTablebase(PI / 2, 0, 0);
	//rotation_matrix_RobotbaseToTablebase << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	rotation_matrix_RobotbaseToTablebase = Eigen::AngleAxisd(EAofRobotbaseToTablebase[0], Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(EAofRobotbaseToTablebase[1], Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(EAofRobotbaseToTablebase[2], Eigen::Vector3d::UnitX());


	Eigen::Vector3d translate_vector_RobotbaseToTable;
	translate_vector_RobotbaseToTable << 0.01, 0, 0;

	Eigen::Isometry3d RobotbaseToTablebase = Eigen::Isometry3d::Identity();
	RobotbaseToTablebase.rotate(rotation_matrix_RobotbaseToTablebase);
	RobotbaseToTablebase.pretranslate(translate_vector_RobotbaseToTable);

	return RobotbaseToTablebase;
}*/
#pragma once

#include "../config.h"
#include "../GraspMethod.h"

//GraspObject类，所有抓取物体的基类

enum Materials { Paper, Wood, Plastic, Metal, Ceramics, UndefMaterial };
enum Shapes { Ellipsoid, Sphere, Cuboid, Cube, Cylinder, CircularTruncatedCone, UndefShape };

class GraspObject {
protected:
	friend class ControlSystem;
	
	int id_;		//场景中物体的id编码
	
	//物体的基础信息
	string English_name_;
	string Chinese_name_;
	Materials made_of_;
	double net_weight_;


	//物体的位置信息
	Eigen::Vector3d center_;
	Eigen::Vector3d X_axis_;
	Eigen::Vector3d Y_axis_;
	Eigen::Vector3d Z_axis_;
	Eigen::Isometry3d RobotbaseToGspObject_;			//物体坐标系相对于机械臂坐标系

	Eigen::Vector3d TranslationOfRobotbaseToCurrentHand_;			//当前机械手相对于机械臂坐标系
	Eigen::Vector3d EularAngleOfRobotbaseToCurrentHand_;

	//桌子参考系在机械臂坐标系下的表示
	Eigen::Isometry3d RobotbaseToTablebase_;

	//所有抓取物体的方法
	vector<GraspMethod> GraspMethods_;

public:
	GraspObject();
	~GraspObject();

	Eigen::Isometry3d GetIsoOfWorldToGspObject();
	vector<GraspMethod> GetGraspMethods();
	int GetGraspObjectId();
};


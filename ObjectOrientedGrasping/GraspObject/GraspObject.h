#pragma once

#include "../config.h"
#include "../GraspMethod.h"

//GraspObject�࣬����ץȡ����Ļ���

enum Materials { Paper, Wood, Plastic, Metal, Ceramics, UndefMaterial };
enum Shapes { Ellipsoid, Sphere, Cuboid, Cube, Cylinder, CircularTruncatedCone, UndefShape };

class GraspObject {
protected:
	friend class ControlSystem;
	
	int id_;		//�����������id����
	
	//����Ļ�����Ϣ
	string English_name_;
	string Chinese_name_;
	Materials made_of_;
	double net_weight_;


	//�����λ����Ϣ
	Eigen::Vector3d center_;
	Eigen::Vector3d X_axis_;
	Eigen::Vector3d Y_axis_;
	Eigen::Vector3d Z_axis_;
	Eigen::Isometry3d RobotbaseToGspObject_;			//��������ϵ����ڻ�е������ϵ

	Eigen::Vector3d TranslationOfRobotbaseToCurrentHand_;			//��ǰ��е������ڻ�е������ϵ
	Eigen::Vector3d EularAngleOfRobotbaseToCurrentHand_;

	//���Ӳο�ϵ�ڻ�е������ϵ�µı�ʾ
	Eigen::Isometry3d RobotbaseToTablebase_;

	//����ץȡ����ķ���
	vector<GraspMethod> GraspMethods_;

public:
	GraspObject();
	~GraspObject();

	Eigen::Isometry3d GetIsoOfWorldToGspObject();
	vector<GraspMethod> GetGraspMethods();
	int GetGraspObjectId();
};


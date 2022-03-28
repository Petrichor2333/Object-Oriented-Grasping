#pragma once
#include "../config.h"
#include "GraspObject.h"

enum TypeOfCoffeeCup { CoffeeCupType1, CoffeeCupType2, CoffeeCupType3, CoffeeCupType4, UndefType};

class GspObj_coffeeCup : public GraspObject
{
private:
	//��״
	Shapes shape_;
	double radius_of_big_Circle_;
	double radius_of_small_Circle_;
	double length_;

	//�ݻ�
	double volume_;

	TypeOfCoffeeCup coffee_cup_type_=UndefType;
	
	//����װ��Һ������
	double volume_of_liquid_;

	Eigen::Isometry3d ReturnIsoOfObjToHandOfGraspMethod0();
	vector<ControlCommand> ReturnHandCtrlCommandOfGraspMethod0();
	GraspMethod FindGraspMethod0(Eigen::Isometry3d WorldToGspObject, double volume_of_liquid);	
	
public:
	GspObj_coffeeCup();
	GspObj_coffeeCup(int id, TypeOfCoffeeCup CoffeeCupType, Eigen::Vector3d center, Eigen::Matrix3d rotation_matrix_WorldToObj);
	~GspObj_coffeeCup();

	GraspMethod MoveObjectToDesignatedPosition(Eigen::Isometry3d	TableBaseToDesignatedPositionOfObj);
	GraspMethod PutdownObjectToDesignatedPosition(Eigen::Isometry3d DesignatedPosition);

	//�򿧷ȱ��м���һ������Һ�壬ͬʱ����ץȡ�ķ���
	int AddLiquidtoCoffeeCup(double volume_of_liquid);
	//�ӿ��ȱ��е���һ������Һ�壬ͬʱ����ץȡ�ķ���
	int PourOutLiquidtoCoffeeCup(double volume_of_liquid);
};


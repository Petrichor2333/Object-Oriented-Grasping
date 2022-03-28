#pragma once
#include "../config.h"
#include "GraspObject.h"

enum TypeOfCoffeeCup { CoffeeCupType1, CoffeeCupType2, CoffeeCupType3, CoffeeCupType4, UndefType};

class GspObj_coffeeCup : public GraspObject
{
private:
	//形状
	Shapes shape_;
	double radius_of_big_Circle_;
	double radius_of_small_Circle_;
	double length_;

	//容积
	double volume_;

	TypeOfCoffeeCup coffee_cup_type_=UndefType;
	
	//杯中装有液体的体积
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

	//向咖啡杯中加入一定量的液体，同时更新抓取的方法
	int AddLiquidtoCoffeeCup(double volume_of_liquid);
	//从咖啡杯中倒出一定量的液体，同时更新抓取的方法
	int PourOutLiquidtoCoffeeCup(double volume_of_liquid);
};


#pragma once
#include "../config.h"
#include "GraspObject.h"

class GspObj_cokecan : public GraspObject {
private:
	void FindWorldToObj(Eigen::Vector3d center, Eigen::Vector3d Z_axis, double epsilon, Eigen::Vector3d &center_, Eigen::Vector3d &X_axis_, Eigen::Vector3d &Y_axis_, Eigen::Vector3d &Z_axis_, Eigen::Isometry3d &WorldToGspObject_);
	GraspMethod FindGraspMethod1(Eigen::Isometry3d WorldToGspObject);
	GraspMethod FindGraspMethod2(Eigen::Isometry3d WorldToGspObject);
public:
	GspObj_cokecan();
	GspObj_cokecan(int id, Eigen::Vector3d center, Eigen::Vector3d Z_axis);

};
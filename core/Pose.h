#ifndef __POSE_H__
#define __POSE_H__
#include <Eigen/Geometry>
#include "Skeleton.h"
#include "Joint.h"
class Pose
{
public:
	Pose(Skeleton* skeleton,const int& num_joint);

	void SetAngle(const int& index,const Eigen::Isometry3d& transform);
	void CalculateTransform();
	void CalculateTransformAll(Joint* joint,Eigen::Isometry3d transform);

	std::vector<Eigen::Isometry3d> GetJointAngles() {return mJointAngle;};
	std::vector<Eigen::Isometry3d> GetJointTransformation() {return mJointTransformation;};

	Eigen::Vector3d GetForwardDirection();

private:
	Skeleton* mSkeleton;
	Joint* mRoot;

	std::vector<Eigen::Isometry3d> mJointAngle;
	std::vector<Eigen::Isometry3d> mJointTransformation;

	int mNumJoint;
	double mScaling;

	Eigen::Vector3d mForwardDirection;
};
#endif
#include "Pose.h"
#include <iostream>
Pose::
Pose(Skeleton* skeleton,const int& num_joint)
	:mSkeleton(skeleton),mNumJoint(num_joint),mScaling(1.0)
{
	mJointAngle.resize(mNumJoint);
	mJointTransformation.resize(mNumJoint);
	mRoot = mSkeleton->GetRoot();
}
void 
Pose::
SetAngle(const int& index,const Eigen::Isometry3d& transform)
{
	mJointAngle[index] = transform;
}
void
Pose::
CalculateTransform()
{
	CalculateTransformAll(mRoot,Eigen::Isometry3d::Identity());
}
void
Pose::
CalculateTransformAll(Joint* joint,Eigen::Isometry3d transform)
{
	Eigen::Isometry3d t = Eigen::Isometry3d::Identity();

	Eigen::Isometry3d poseTransform = mJointAngle[joint->GetIndex()];
	if(joint->GetParent()==nullptr) {
		poseTransform.translation() *= mScaling;
	} 
    Eigen::Isometry3d skelTransform= Eigen::Isometry3d::Identity();
    skelTransform.translation()= mScaling*joint->GetOffset();

    t= transform* skelTransform* poseTransform;
    mJointTransformation[joint->GetIndex()]= t;

	for(const auto& child: joint->GetChildren())
		CalculateTransformAll(child,t);
}
Eigen::Vector3d
Pose::
GetForwardDirection()
{
	int left_hip = mSkeleton->GetNameIdxMap()["L_Hip"];
	int right_hip = mSkeleton->GetNameIdxMap()["R_Hip"];

	Eigen::Vector3d across_hip = mJointTransformation[left_hip].translation() - mJointTransformation[right_hip].translation();	

	Eigen::Vector3d across = across_hip.normalized();
	Eigen::Vector3d forward = across.cross(Eigen::Vector3d(0,1,0));

	return forward.normalized();
}
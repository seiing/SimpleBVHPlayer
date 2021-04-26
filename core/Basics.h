#ifndef __BASIC_H__
#define __BASIC_H__
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace Basics
{
	Eigen::Quaterniond GetClosestRotation(Eigen::Quaterniond original_rot,Eigen::Vector3d baseAxis);
	Eigen::Matrix3d R_x(double x);
    Eigen::Matrix3d R_y(double y);
    Eigen::Matrix3d R_z(double z);
    Eigen::Vector3d ExtractAngle(const Eigen::Isometry3d& transform);
    double GetBetweenAngle(Eigen::Vector3d a,Eigen::Vector3d b);
    Eigen::VectorXd to6D(const Eigen::Quaterniond& quaternion);
    Eigen::Matrix3d toRotationMatrix(const Eigen::VectorXd& rotation6d);
};
#endif
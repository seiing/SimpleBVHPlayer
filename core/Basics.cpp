#include "Basics.h"
#include <cassert>
#include <iostream>
Eigen::Quaterniond
Basics::
GetClosestRotation(Eigen::Quaterniond original_rot,Eigen::Vector3d baseAxis)
{
	double ws = original_rot.w();
	Eigen::Vector3d vs(original_rot.x(), original_rot.y(), original_rot.z());

	double w0= 1;
	Eigen::Vector3d v0(0,0,0);

	double a= ws*w0+ vs.dot(v0);
	double b= w0*(baseAxis.dot(vs)) - ws*(baseAxis.dot(v0)) + vs.dot(baseAxis.cross(v0));

	double alpha= std::atan2(a,b); //radian
	double t_minus= -2*alpha - M_PI;
	double t_plus= -2*alpha + M_PI;

	Eigen::Quaterniond q_minus(Eigen::AngleAxisd(t_minus, baseAxis));
	Eigen::Quaterniond q_plus(Eigen::AngleAxisd(t_plus, baseAxis));

	if(original_rot.dot(q_minus)> original_rot.dot(q_plus)) return q_minus;
	return q_plus;
}
Eigen::Matrix3d
Basics::
R_x(double x) 
{
    double cosa = cos(x * M_PI / 180.0);
    double sina = sin(x * M_PI / 180.0);
    Eigen::Matrix3d R;
    R << 1, 0, 0,
            0, cosa, -sina,
            0, sina, cosa;
    return R;
}

Eigen::Matrix3d 
Basics::
R_y(double y) 
{
    double cosa = cos(y * M_PI / 180.0);
    double sina = sin(y * M_PI / 180.0);
    Eigen::Matrix3d R;
    R << cosa, 0, sina,
            0, 1, 0,
            -sina, 0, cosa;
    return R;
}
Eigen::Matrix3d 
Basics::
R_z(double z) 
{
    double cosa = cos(z * M_PI / 180.0);
    double sina = sin(z * M_PI / 180.0);
    Eigen::Matrix3d R;
    R << cosa, -sina, 0,
            sina, cosa, 0,
            0, 0, 1;
    return R;
}
Eigen::Vector3d
Basics::
ExtractAngle(const Eigen::Isometry3d& transform)
{
    Eigen::Vector3d angle;
    angle[0] = std::atan2(-transform(1,2),transform(2,2));
    double cosY = std::sqrt(
        pow(transform(0,0),2)+pow(transform(0,1),2)
    );
    angle[1] = std::atan2(transform(0,2),cosY);
    double sinX = std::sin(angle[0]);
    double cosX = std::cos(angle[0]);
    angle[2] = std::atan2(cosX*transform(1,0)+sinX*transform(2,0),cosX*transform(1,1)+sinX*transform(2,1));

    return angle;
}
double 
Basics::
GetBetweenAngle(Eigen::Vector3d a,Eigen::Vector3d b)
{
    double cos= a.dot(b);
    double sin= a.cross(b).norm();
    double theta = atan2(sin, cos);

    assert(!std::isnan(theta)&&!std::isinf(theta));
    return theta;
}
Eigen::VectorXd
Basics::
to6D(const Eigen::Quaterniond& quaternion)
{
    Eigen::Matrix3d mat = quaternion.toRotationMatrix();
    Eigen::VectorXd ret(6);
    ret.block<3,1>(0,0) =  mat.col(0);
    ret.block<3,1>(3,0) =  mat.col(1);
    return ret;
}
Eigen::Matrix3d
Basics::
toRotationMatrix(const Eigen::VectorXd& rotation6d)
{
    Eigen::Matrix3d R; 

    Eigen::Vector3d x_raw = rotation6d.block<3,1>(0,0);
    Eigen::Vector3d y_raw = rotation6d.block<3,1>(3,0);

    Eigen::Vector3d x,y,z;
    x = x_raw.normalized();
    z = x.cross(y_raw);
    z = z.normalized();
    y = z.cross(x);

    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;

    return R;
}
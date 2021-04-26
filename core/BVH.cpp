#include "BVH.h"
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "Basics.h"
BVH::
BVH(std::string name)
	:mName(name),mScaling(1.0)
{
	mSkeleton = new Skeleton();
}
Eigen::Isometry3d 
Channel2Isometry3d(CHANNEL c,double v)
{
    Eigen::Isometry3d I= Eigen::Isometry3d::Identity();
    switch(c)
    {
        case Xpos: I.translation()[0]+=v; break;
        case Ypos: I.translation()[1]+=v; break;
        case Zpos: I.translation()[2]+=v; break;
        case Xrot: I.linear()= I.linear()*Basics::R_x(v); break;
        case Yrot: I.linear()= I.linear()*Basics::R_y(v); break;
        case Zrot: I.linear()= I.linear()*Basics::R_z(v); break;
        default: break;
    }
    return I;
}
void
BVH::
Parse()
{
	std::ifstream is(mName);
	char buffer[256];
	if(!is)
	{
		std::cout<<"Can't open file: "<< "\t" << mName <<std::endl;
		return;
	}
	while(is>>buffer)
	{
		if(!strcmp(buffer,"HIERARCHY"))
		{
			is>>buffer;
			is>>buffer;
			int c_offset=0;
			mSkeleton->SetRoot(ReadHierarchy(nullptr,buffer,c_offset,is));
			mNumTotalChannels = c_offset;
		}
		else if(!strcmp(buffer,"MOTION"))
		{
			is>>buffer;
			is>>buffer;
			mNumTotalFrames = atoi(buffer);
			is>>buffer;
			is>>buffer;
			is>>buffer;
			mTimeStep = atof(buffer);
			double val;

			Eigen::Isometry3d root_angle = Eigen::Isometry3d::Identity();
			Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

			for(int i=0;i<mNumTotalFrames;i++)
			{
				Eigen::VectorXd motion(mNumTotalChannels);

				for(int j=0;j<mNumTotalChannels;j++)
				{
					is>>val;
					motion[j]=val;
				}

				mPoses.emplace_back(new Pose(mSkeleton,mSkeleton->GetNumJoint()));
				auto pose = mPoses.back();
				int idx=0;

				for(const auto& m: mSkeleton->GetIdxJointMap())
				{
					Eigen::Isometry3d transform =Eigen::Isometry3d::Identity();
					
					for(const auto& c: m.second->GetChannel())
					{
						if(m.second->GetParent() == nullptr) {
							if(c == Xpos || c == Ypos || c== Zpos) {
								motion[idx] *= mScaling;
							}
						}
						transform = transform*Channel2Isometry3d(c,motion[idx]);						
						idx+=1;
					}
					pose->SetAngle(m.first,transform);
				}
				
				root_angle = pose->GetJointAngles()[0];

				if(i == 0) {
					transform = root_angle.inverse();
					transform.translation()[1] = 0.0;

					Eigen::Quaterniond inplane_transform_q= Basics::GetClosestRotation(Eigen::Quaterniond(transform.linear()),Eigen::Vector3d(0,1,0));
		            transform.linear()= inplane_transform_q.toRotationMatrix();
		            transform.translation() = -transform.linear()*root_angle.translation();
		            transform.translation()[1] = 0.0;
		        }

	            pose->SetAngle(0,transform*root_angle);
				pose->CalculateTransform();
			}
		}
	}
	is.close();

	mSkeleton->SetKeyJoint();
}
Joint* 
BVH::
ReadHierarchy(Joint* parent,const std::string& name,int& channel_offset,std::ifstream& is)
{
	char buffer[256];
	double offset[3];
	std::vector<std::string> c_name;

	Joint* new_joint = new Joint(name,mSkeleton->GetNumJoint(),parent);

	mSkeleton->SetNameIdxMap(std::pair<std::string,int>(name,mSkeleton->GetNumJoint()));
	mSkeleton->SetIdxJointMap(std::pair<int,Joint*>(mSkeleton->GetNumJoint(),new_joint));

	mSkeleton->SetNumJoint(mSkeleton->GetNumJoint()+1);

	is>>buffer;

	while(is>>buffer)
	{
		if(!strcmp(buffer,"}"))
		{
			break;
		}
		else if(!strcmp(buffer,"OFFSET"))
		{
			double x,y,z;
			is>>x; is>>y; is>>z;

			if(parent==nullptr)
			{
				mRootCOMOffset[0]=x;
				mRootCOMOffset[1]=y;
				mRootCOMOffset[2]=z;
			} 
			new_joint->SetOffset(mScaling*Eigen::Vector3d(x,y,z));	
		}
		else if(!strcmp(buffer,"CHANNELS"))
		{
			is>>buffer;
			int n;
			n= atoi(buffer);

			for(int i=0;i<n;i++)
			{
				is>>buffer;
				c_name.push_back(std::string(buffer));
			}
			
			new_joint->SetChannel(channel_offset,c_name);
			channel_offset+=n;
		}
		else if(!strcmp(buffer,"JOINT"))
		{
			is>>buffer;
			Joint* child = ReadHierarchy(new_joint,std::string(buffer),channel_offset,is);
			new_joint->AddChild(child);
		}
		else if(!strcmp(buffer,"End"))
		{
			is>>buffer;
			Joint* child = ReadHierarchy(new_joint,std::string("EndEffector"),channel_offset,is);
			new_joint->AddChild(child);
		}
	}

	return new_joint;
}
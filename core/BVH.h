#ifndef __BVH_H__
#define __BVH_H__
#include <string>
#include <Eigen/Core>
#include "Joint.h"
#include "Pose.h"
#include "Skeleton.h"
class BVH
{
public:
	BVH(std::string name);
	std::string GetName() {return mName;};

	void Parse();

	int GetNumFrame(){return mNumTotalFrames;};
	double GetTimeStep(){return mTimeStep;};

	std::vector<Pose*> GetPoses() {return mPoses;};	
	Skeleton* GetSkeleton() {return mSkeleton;};

private:
	std::string mName;

	Skeleton* mSkeleton;
	std::vector<Pose*> mPoses;

	int mNumTotalChannels;
	Eigen::Vector3d mRootCOMOffset;
	int mNumTotalFrames;
	double mTimeStep;

	Eigen::Vector3d mForward;

	double mScaling;

	Joint* ReadHierarchy(Joint* parent,const std::string& name,int& channel_offset,std::ifstream& is);
};
#endif
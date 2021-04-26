#ifndef __SKELETON_H__
#define __SKELETON_H__
#include "Joint.h"
class Skeleton
{
public:
	Skeleton();

	void SetRoot(Joint* root);
	Joint* GetRoot() {return mRoot;};
	void SetKeyJoint();
	std::vector<int> GetKeyJoints() {return mKeyJoints;};
	std::vector<std::pair<int,int>> GetKeyJointPairs(){return mKeyJointPairs;};

	std::map<std::string,int> GetNameIdxMap(){return mNameIdxMap;};
	void SetNameIdxMap(std::pair<std::string,int> map);
	std::map<int,Joint*> GetIdxJointMap(){return mIdxJointMap;};
	void SetIdxJointMap(std::pair<int,Joint*> map);

	void SetNumJoint(const int& n);
	int GetNumJoint() {return mNumJoint;};

private:
	Joint* mRoot;

	std::map<std::string,int> mNameIdxMap;
    std::map<int,Joint*> mIdxJointMap;
    std::vector<int> mKeyJoints;
    std::vector<std::pair<int,int>> mKeyJointPairs;

    int mNumJoint;
};
#endif
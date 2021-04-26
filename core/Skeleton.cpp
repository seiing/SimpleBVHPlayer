#include "Skeleton.h"
#include <iostream>
Skeleton::
Skeleton()
	:mNumJoint(0)
{
	mRoot = new Joint();
}
void
Skeleton::
SetRoot(Joint* root)
{
	mRoot = root;
}
void 
Skeleton::
SetKeyJoint()
{
    std::vector<std::string> key_joints = 
    {
        "Pelvic", //0
        "L_Hip", //1
        "L_Knee", //2
        "L_Ankle", //3
        "L_Toe", //4
        "R_Hip", //6
        "R_Knee", //7
        "R_Ankle", //8
        "R_Toe", //9
    };

    for(const auto& k: key_joints){
     for(const auto& n: mNameIdxMap){
         if(n.first == k) {
             mKeyJoints.emplace_back(n.second);
             break; 
         }
     }
    }

    mKeyJointPairs.emplace_back(std::make_pair(0, 1));
    mKeyJointPairs.emplace_back(std::make_pair(1, 2));
    mKeyJointPairs.emplace_back(std::make_pair(2, 3));
    mKeyJointPairs.emplace_back(std::make_pair(3, 4));

    mKeyJointPairs.emplace_back(std::make_pair(0, 6));
    mKeyJointPairs.emplace_back(std::make_pair(6, 7));
    mKeyJointPairs.emplace_back(std::make_pair(7, 8));
    mKeyJointPairs.emplace_back(std::make_pair(8, 9));
}
void
Skeleton::
SetNameIdxMap(std::pair<std::string,int> map)
{
	mNameIdxMap.insert(map);
}
void
Skeleton::
SetIdxJointMap(std::pair<int,Joint*> map)
{
	mIdxJointMap.insert(map);
}
void
Skeleton::
SetNumJoint(const int& n)
{
	mNumJoint=n;
}
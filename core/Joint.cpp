#include "Joint.h"
#include "Basics.h"
#include <iostream>
Joint::
Joint()
{
}
Joint::
Joint(const std::string& name,const int& index,Joint* parent)
	:mParent(parent),mName(name),mIndex(index),mNumChannels(0)
{
}
void 
Joint::
SetChannel(int c_offset,std::vector<std::string>& c_name)
{
	mChannelOffset = c_offset;
	mNumChannels = c_name.size();
	for(const auto& cn : c_name)
		mChannel.push_back(CHANNEL_NAME[cn]);
}
void 
Joint::
SetChannel(std::vector<CHANNEL> channel,int channel_offset)
{
	mChannel=channel;
	mNumChannels = mChannel.size();
	mChannelOffset = channel_offset;
}
void 
Joint::
SetOffset(Eigen::Vector3d offset)
{
	mOffset = offset;
}
void 
Joint::
AddChild(Joint* child)
{
	mChildren.push_back(child);
}
std::map<std::string,CHANNEL> Joint::CHANNEL_NAME =
{
	{"Xposition",Xpos},
	{"XPOSITION",Xpos},
	{"Yposition",Ypos},
	{"YPOSITION",Ypos},
	{"Zposition",Zpos},
	{"ZPOSITION",Zpos},
	{"Xrotation",Xrot},
	{"XROTATION",Xrot},
	{"Yrotation",Yrot},
	{"YROTATION",Yrot},
	{"Zrotation",Zrot},
	{"ZROTATION",Zrot}
};
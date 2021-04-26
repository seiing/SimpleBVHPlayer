#ifndef __JOINT_H__
#define __JOINT_H__
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
enum CHANNEL
{
	Xpos=0,
	Ypos=1,
	Zpos=2,
	Xrot=3,
	Yrot=4,
	Zrot=5
}; 
class Joint
{
public:
	
	static std::map<std::string,CHANNEL> CHANNEL_NAME;

	Joint(); 
	Joint(const std::string& name,const int& index,Joint* parent);
	
	Joint* GetParent() {return mParent;};
	void AddChild(Joint* child);
	std::vector<Joint*> GetChildren(){return mChildren;};

	std::string GetName(){return mName;};

	void SetChannel(int c_offset,std::vector<std::string>& c_name);
	void SetChannel(std::vector<CHANNEL> channel,int channel_offset);
	std::vector<CHANNEL> GetChannel(){return mChannel;};
	int GetChannelOffset(){return mChannelOffset;};
	void SetOffset(Eigen::Vector3d offset);
	Eigen::Vector3d GetOffset(){return mOffset;};

	int GetIndex() {return mIndex;};

private:
	int mIndex;

	Joint* mParent;
	std::vector<Joint*> mChildren;

	Eigen::Vector3d mOffset;

	std::string mName;
	int mNumChannels;
	int mChannelOffset;
	std::vector<CHANNEL> mChannel;
};
#endif
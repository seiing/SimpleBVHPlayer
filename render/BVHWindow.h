#ifndef __BVH_WINDOW_H__
#define __BVH_WINDOW_H__
#include "GLWindow.h"
#include "BVH.h"
class BVHWindow : public GLWindow
{
public:
	BVHWindow();

protected:
	void Display() override;
	void Keyboard(unsigned char key,int x,int y) override;
	void Special(int key,int x,int y) override;
	void Mouse(int button, int state, int x, int y) override;
	void Motion(int x, int y) override;
	void Reshape(int w, int h) override;
	void Timer(int value) override;
	void initLights();

	void DrawStringOnScreen(float _x, float _y, const std::string& _s,bool _bigFont);
	void DrawFloor();
	void DrawCylinder(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const double& line_width=1.0,const Eigen::Vector3d& color = Eigen::Vector3d(0.8,0.8,0.8));
	void DrawCylinder(double radius, double height, int slices, int stacks);

protected:
	std::vector<BVH*>	mBVHs;

	bool 				mPlay;
	int 				mCurFrame;
	int 				mTotalFrame;
	double 				mElapsedTime;

	int 				mBVHIdx;
};
#endif
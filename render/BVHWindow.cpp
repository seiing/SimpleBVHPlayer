#include "BVHWindow.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <string>
#include <vector>
#include <filesystem>
#include "Basics.h"
static GLUquadricObj *quadObj;
static void initQuadObj(void)
{
    quadObj = gluNewQuadric();
}
#define QUAD_OBJ_INIT { if(!quadObj) initQuadObj(); }
namespace fs = std::filesystem;
BVHWindow::
BVHWindow()
	:GLWindow(),mCurFrame(0),mTotalFrame(0),mElapsedTime(0.0),mBVHIdx(0),mPlay(false)
{
	std::cout << "BVH Player" << std::endl;
	mDisplayTimeout = 33;
	initLights();
	
	std::string path = std::string(BVH_DIR)+"/data/";
	std::vector<std::string> filenames;
	for (const auto& entry: fs::directory_iterator(path))
		filenames.emplace_back(entry.path().filename().string());
		
	for(const auto fn: filenames) {
		mBVHs.emplace_back(new BVH(path+fn));
		auto bvh = mBVHs.back();
		bvh->Parse();
	}
}
void
BVHWindow::
initLights()
{ 
	static float ambient[]           	 = {0.4, 0.4, 0.4, 1.0};
	static float diffuse[]             = {0.4, 0.4, 0.4, 1.0};
	static float front_mat_shininess[] = {60.0};
	static float front_mat_specular[]  = {0.2, 0.2,  0.2,  1.0};
	static float front_mat_diffuse[]   = {0.2, 0.2, 0.2, 1.0};
	static float lmodel_ambient[]      = {0.2, 0.2,  0.2,  1.0};
	static float lmodel_twoside[]      = {GL_TRUE};

	GLfloat position[] = {0.0, 1.0, 1.0, 0.0};
	GLfloat position1[] = {0.0, 1.0, -1.0, 0.0};

	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,  lmodel_ambient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position1);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);

	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, front_mat_shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  front_mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   front_mat_diffuse);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDisable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);

	glEnable(GL_FOG);
	GLfloat fogColor[] = {200.0/256.0,200.0/256.0,200.0/256.0,1};
	glFogfv(GL_FOG_COLOR,fogColor);
	glFogi(GL_FOG_MODE,GL_LINEAR);
	glFogf(GL_FOG_DENSITY,0.05);
	glFogf(GL_FOG_START,20.0);
	glFogf(GL_FOG_END,40.0);
}
void
BVHWindow::
Display()
{
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	initLights();
	glEnable(GL_LIGHTING);

	mCamera->Apply();

	this->DrawFloor();
	this->DrawStringOnScreen(0.9, 0.9, std::to_string(mCurFrame), true);

	const auto& joint_transformation = mBVHs[mBVHIdx]->GetPoses()[mCurFrame]->GetJointTransformation();
	const auto& key_joint_pair = mBVHs[mBVHIdx]->GetSkeleton()->GetKeyJointPairs();
	const auto& name_id_map = mBVHs[mBVHIdx]->GetSkeleton()->GetNameIdxMap();

	for(int i=0;i<key_joint_pair.size(); i++) 
	{
		Eigen::Vector3d p1 = joint_transformation[key_joint_pair[i].first].translation();
		Eigen::Vector3d p2 = joint_transformation[key_joint_pair[i].second].translation();
		this->DrawCylinder(p1,p2,0.02,Eigen::Vector3d(230, 133, 171));	
	}

	glutSwapBuffers();
}
void
BVHWindow::
Keyboard(unsigned char key,int x,int y)
{
	switch(key)
	{
		case 27: exit(0); break;
		case ' ': {
			mPlay = !mPlay;
			if(mPlay)
				std::cout << "Play." << std::endl;
			else 
				std::cout << "Pause." << std::endl;

			break;
		}
		case 'r': {
			break;
		}
		case '<': {
			mCurFrame-=1;
			if(mCurFrame<0) mCurFrame=0;
			std::cout << "Frame: "<<mCurFrame << std::endl;
			break;
		}
		case '>': {
			mCurFrame+=1; 
			if(mCurFrame>=mBVHs[mBVHIdx]->GetNumFrame()) mCurFrame=0;
			std::cout << "Frame: "<<mCurFrame << std::endl;
			break;
		}
	}
	glutPostRedisplay();
}
void 
BVHWindow::
Special(int key,int x,int y)
{
	switch(key)
	{
		case GLUT_KEY_LEFT: {
			mCurFrame=0;
			mBVHIdx-= 1;
			if(mBVHIdx<0) mBVHIdx=0;
			break;
		}

		case GLUT_KEY_RIGHT: {
			mCurFrame=0;
			mBVHIdx+= 1;
			if(mBVHIdx>=mBVHs.size()) mBVHIdx=0;
			break;
		}
	}
	glutPostRedisplay();
}
void
BVHWindow::
Mouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
	{
		mIsDrag = true;
		mMouseType = button;
		mPrevX = x;
		mPrevY = y;
	}
	else
	{
		mIsDrag = false;
		mMouseType = 0;
	}

	glutPostRedisplay();
}
void
BVHWindow::
Motion(int x, int y)
{
	if (!mIsDrag)
		return;

	int mod = glutGetModifiers();
	if (mMouseType == GLUT_LEFT_BUTTON)
		mCamera->Translate(x,y,mPrevX,mPrevY);

	else if (mMouseType == GLUT_MIDDLE_BUTTON)
		mCamera->Pan(x,y,mPrevX,mPrevY);

	else if (mMouseType == GLUT_RIGHT_BUTTON)
		mCamera->Rotate(x,y,mPrevX,mPrevY);
	
	mPrevX = x;
	mPrevY = y;
	glutPostRedisplay();
}
void
BVHWindow::
Reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	mCamera->Apply();
	glutPostRedisplay();
}
void
BVHWindow::
Timer(int value)
{
	if(mPlay){
		mCurFrame+=1;
		if(mCurFrame>=mBVHs[mBVHIdx]->GetNumFrame()) mCurFrame = 0;
	}

	mElapsedTime = mCurFrame;

	glutTimerFunc(mDisplayTimeout, TimerEvent,1);
	glutPostRedisplay();
}
void
BVHWindow::
DrawStringOnScreen(float _x, float _y, const std::string& _s,bool _bigFont)
{
    GLint oldMode;
    glGetIntegerv(GL_MATRIX_MODE, &oldMode);
    glMatrixMode(GL_PROJECTION);

    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0, 0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glRasterPos2f(_x, _y);
    unsigned int length = _s.length();
    for (unsigned int c = 0; c < length; c++) {
    if (_bigFont)
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, _s.at(c) );
    else
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, _s.at(c) );
    }  
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(oldMode);
}
void 
BVHWindow::
DrawFloor()
{
    glEnable(GL_LIGHTING);  
    glBegin(GL_QUADS);

    double ground_y = -0.0;
    double width_x = 50.0;
    double width_z = 50.0;
    double height_y = 100.0;

    int cnt =0;

    for(int x=-width_x;x<=width_x;x++)
    {
        for(int z=-width_z;z<=width_z;z++)
        {
            if(cnt%2==0) {
                glColor3f(150.0/256.0,150.0/256.0,150.0/256.0);
            } else {
                glColor3f(200.0/256.0,200.0/256.0,200.0/256.0);
            }
            cnt+=1;
            glNormal3d(0,1,0);
            glVertex3f(x,ground_y,z);
            glVertex3f(x+1.0,ground_y,z);
            glVertex3f(x+1.0,ground_y,z+1.0);
            glVertex3f(x,ground_y,z+1.0);
        }
    }
    glEnd();

    glBegin(GL_QUADS);
    glNormal3d(0,0,1);
    glVertex3f(-width_x,ground_y,-width_z);
    glVertex3f(width_x,ground_y,-width_z);
    glVertex3f(width_x,height_y,-width_z);
    glVertex3f(-width_x,height_y,-width_z);
    glEnd();

    glBegin(GL_QUADS);
    glNormal3d(0,0,-1);
    glVertex3f(-width_x,ground_y,width_z);
    glVertex3f(-width_x,height_y,width_z);
    glVertex3f(width_x,height_y,width_z);
    glVertex3f(width_x,ground_y,width_z);
    glEnd();

    glBegin(GL_QUADS);
    glNormal3d(1,0,0);
    glVertex3f(-width_x,ground_y,-width_z);
    glVertex3f(-width_x,height_y,-width_z);
    glVertex3f(-width_x,height_y,width_z);
    glVertex3f(-width_x,ground_y,width_z);
    glEnd();

    glBegin(GL_QUADS);
    glNormal3d(-1,0,0);
    glVertex3f(width_x,ground_y,-width_z);
    glVertex3f(width_x,ground_y,width_z);
    glVertex3f(width_x,height_y,width_z);
    glVertex3f(width_x,height_y,-width_z);
    glEnd();

    glDisable(GL_LIGHTING);
}
void 
BVHWindow::
DrawCylinder(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const double& line_width,const Eigen::Vector3d& color)
{
    glEnable(GL_LIGHTING);
    Eigen::Vector3d u(0,0,1);
    Eigen::Vector3d v = p0-p1;
    Eigen::Vector3d mid = 0.5*(p0+p1);
    double len = v.norm();
    v /= len;
    Eigen::Isometry3d T;
    T.setIdentity();
    Eigen::Vector3d axis = u.cross(v);
    axis.normalize();
    double angle = acos(u.dot(v));
    Eigen::Matrix3d w_bracket = Eigen::Matrix3d::Zero();
    w_bracket(0, 1) = -axis(2);
    w_bracket(1, 0) =  axis(2);
    w_bracket(0, 2) =  axis(1);
    w_bracket(2, 0) = -axis(1);
    w_bracket(1, 2) = -axis(0);
    w_bracket(2, 1) =  axis(0);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity()+(sin(angle))*w_bracket+(1.0-cos(angle))*w_bracket*w_bracket;
    T.linear() = R;
    T.translation() = mid;
    glPushMatrix();
    glMultMatrixd(T.data());
    glEnable(GL_BLEND);
    glColor3ub(color[0],color[1],color[2]);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    DrawCylinder(line_width,len,6,6);
    glPopMatrix();
    glDisable(GL_BLEND);
}
void 
BVHWindow::
DrawCylinder(double radius, double height, int slices, int stacks)
{
  glPushMatrix();

  // Graphics assumes Cylinder is centered at CoM
  // gluCylinder places base at z = 0 and top at z = height
  glTranslated(0.0, 0.0, -0.5*height);

  // Code taken from glut/lib/glut_shapes.c
  QUAD_OBJ_INIT;
  gluQuadricDrawStyle(quadObj, GLU_FILL);
  gluQuadricNormals(quadObj, GLU_SMOOTH);
  //gluQuadricTexture(quadObj, GL_TRUE);

  // glut/lib/glut_shapes.c
  gluSphere(quadObj,radius,stacks,stacks);
  gluCylinder(quadObj, radius, radius, height, slices, stacks);
  glTranslatef(0,0,height);
  gluSphere(quadObj,radius,stacks,stacks);

  glPopMatrix();
}
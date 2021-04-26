#include "BVHWindow.h"
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif
int main(int argc,char** argv)
{
	BVHWindow* window = new BVHWindow();
	glutInit(&argc, argv);
	window->InitWindow(1080,720,"BVH Data Viewer");
	glutMainLoop();
}
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <numeric>
#include <iostream>
#include <Windows.h>
#include <gl/GL.h>
#include <gl/glut.h>

#include "glm.h"
#include "mtxlib.h"
#include "trackball.h"

using namespace std;


//本次作业有两个版本，对应的example.cpp example1.cpp文件。
//example.cpp    针对W的权重算法是根据每一个点与所有控制点的关系计算的
//example1.cpp   针对W的算法是根据所有控制点的关系计算的，和课上的方法相似。

//从效果来看，example.cpp的效果比较好。

//因为我以前是电工系的，针对这次作业的看法，提出以下思路：
//意思就是控制点就像一个个大头针，针与针不能互相影响，
//但是钉住的人皮是根据钉它的针变形的，人皮上的点越靠近某根针，
//就越受其影响，如果就是钉住的那一点，则完全受其影响，不受其他针的影响
//然而最起码的要求，该权重的影响要平滑，权重之和为1

//提出模型： 电阻分流模型
//就像电阻并联，电阻越小，电流越大，电阻为0，电流为总电流，所有支路电流相加为总电流
//电阻 = 距离， 电流 = 权重，总电流 = 1
//然后算法就清楚了， 计算各支路的电导，然后根据总电导求其所占比例
//电导 = 距离的倒数
//就是每个点与所有控制点的权重weight都不一样



_GLMmodel *mesh;
int WindWidth, WindHeight;

int last_x , last_y;
int selectedFeature = -1;
vector<int> featureList;

int numPt = 200;			//决定周围选取多少个顶点
vector<int>  minDisIdx;	//用于顺序存储顶点序号
vector<float> minDiss;	//用于顺序存储顶点与鼠标的距离

void Reshape(int width, int height)
{
  int base = min(width , height);

  tbReshape(width, height);
  glViewport(0 , 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0,(GLdouble)width / (GLdouble)height , 1.0, 128.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0, 0.0, -3.5);

  WindWidth = width;
  WindHeight = height;
}

void Display(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();
  tbMatrix();
  
  // render solid model
  glEnable(GL_LIGHTING);
  glColor3f(1.0 , 1.0 , 1.0f);
  glPolygonMode(GL_FRONT_AND_BACK , GL_FILL);
  glmDraw(mesh , GLM_SMOOTH);

  // render wire model
  glPolygonOffset(1.0 , 1.0);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glLineWidth(1.0f);
  glColor3f(0.6 , 0.0 , 0.8);
  glPolygonMode(GL_FRONT_AND_BACK , GL_LINE);
  glmDraw(mesh , GLM_SMOOTH);

  // render features
  glPointSize(10.0);
  glColor3f(1.0 , 0.0 , 0.0);
  glDisable(GL_LIGHTING);
  glBegin(GL_POINTS);
	for (int i = 0 ; i < featureList.size() ; i++)
	{
		int idx = featureList[i];

		glVertex3fv((float *)&mesh->vertices[3 * idx]);
	}
  glEnd();
  
  glPopMatrix();

  glFlush();  
  glutSwapBuffers();
}

vector3 Unprojection(vector2 _2Dpos)
{
	float Depth;
	int viewport[4];
	double ModelViewMatrix[16];				//Model_view matrix
	double ProjectionMatrix[16];			//Projection matrix

	glPushMatrix();
	tbMatrix();

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, ModelViewMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, ProjectionMatrix);

	glPopMatrix();

	glReadPixels((int)_2Dpos.x , viewport[3] - (int)_2Dpos.y , 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &Depth);

	double X = _2Dpos.x;
	double Y = _2Dpos.y;
	double wpos[3] = {0.0 , 0.0 , 0.0};

	gluUnProject(X , ((double)viewport[3] - Y) , (double)Depth , ModelViewMatrix , ProjectionMatrix , viewport, &wpos[0] , &wpos[1] , &wpos[2]);

	return vector3(wpos[0] , wpos[1] , wpos[2]);
}

void mouse(int button, int state, int x, int y)
{
  tbMouse(button, state, x, y);

  // add feature
  if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN)
  {
	  int minIdx = 0;
	  float minDis = 9999999.0f;

	  vector3 pos = Unprojection(vector2((float)x , (float)y));

	  for (int i = 0 ; i < mesh->numvertices ; i++)
	  {
		  vector3 pt(mesh->vertices[3 * i + 0] , mesh->vertices[3 * i + 1] , mesh->vertices[3 * i + 2]);
		  float dis = (pos - pt).length();

		  if (minDis > dis)
		  {
			  minDis = dis;
			  minIdx = i;
		  }
	  }

	  featureList.push_back(minIdx);
  }

  // manipulate feature
  if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
  {
	  int minIdx = 0;
	  float minDis = 9999999.0f;

	  vector3 pos = Unprojection(vector2((float)x , (float)y));

	  for (int i = 0 ; i < featureList.size() ; i++)
	  {
		  int idx = featureList[i];
		  vector3 pt(mesh->vertices[3 * idx + 0] , mesh->vertices[3 * idx + 1] , mesh->vertices[3 * idx + 2]);
		  float dis = (pos - pt).length();

		  if (minDis > dis)
		  {
			  minDis = dis;
			  minIdx = featureList[i];
		  }
	  }

	  selectedFeature = minIdx;




	 //初始化两个向量组
	  minDisIdx.clear();
	  minDiss.clear();
	  for (int i = 0; i < numPt; i++)
	  {
		  minDiss.push_back(9999999.0f);
		  minDisIdx.push_back(0);
	  }



	  vector3 spt(mesh->vertices[3 * selectedFeature + 0], mesh->vertices[3 * selectedFeature + 1], mesh->vertices[3 * selectedFeature + 2]);
	  for (int i = 0; i < mesh->numvertices; i++)
	  {
		  vector3 pt(mesh->vertices[3 * i + 0], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);
		  float dis = (pt - spt).length();


		  for (int j = 0; j < numPt; j++)
		  {
			  //每一个顶点的距离都与当前minDiss里面储存的每一个值对比，发现比其中一个值小就插入到minDiss，实现排序
			  if (minDiss[j] >= dis)
			  {
				  minDiss.insert(minDiss.begin() + j, dis);
				  minDisIdx.insert(minDisIdx.begin() + j, i);
				  minDiss.pop_back();
				  minDisIdx.pop_back();
				  break;
			  }
			  //因为插入了一个值，所以相应的删除队列尾部的值

		  }

	  }

  }

  if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
	  selectedFeature = -1;

  last_x = x;
  last_y = y;
}

float Gaussian(float distance){	float gaussian;	float sigama = 0.1;	gaussian = exp(-pow(distance, 2) / (2 * pow(sigama, 2)));	return gaussian;}

void motion(int x, int y)
{
  tbMotion(x, y);

  if (selectedFeature != -1)
  {
	  matrix44 m;
	  vector4 vec = vector4((float)(x - last_x) / 100.0f , (float)(y - last_y) / 100.0f , 0.0 , 1.0);
	  
	  gettbMatrix((float *)&m);
	  vec = m * vec;
	  //sfp: selectedFeature 的三维坐标
	  vector3 sfp(mesh->vertices[3 * selectedFeature + 0], mesh->vertices[3 * selectedFeature + 1], mesh->vertices[3 * selectedFeature + 2]);

	  //temp: 储存当前处理的顶点
	  //fp：储存当前处理的控制点
	  vector3 temp, fp;

	  //mainDist：储存当前处理顶点与selectedFeature的距离
	  //sum：加权分母，用于计算权重
	  //weight：权重
	  float mainDist, sum,t_length, weight = 0.5;

	  //遍历selectedFeature周围所有点
	  for (unsigned int i = 0; i < minDisIdx.size();i++)
	  {
		  sum = 0;
		  //temp得到当前处理的顶点坐标
		  temp.x = mesh->vertices[3 * minDisIdx[i] + 0];
		  temp.y = mesh->vertices[3 * minDisIdx[i] + 1];
		  temp.z = mesh->vertices[3 * minDisIdx[i] + 2];
		  //计算当前顶点与selectedFeature的距离
		  mainDist = (temp - sfp).length();
		  
	  
		  //通过距离却定当前顶点是否就是selectedFeature本身？如果是，权重为1（这里把这个特殊情况揪出来并不是因为虽然后面的算法不适用这种情况，而是因为计算机无法处理分母为零的情况，然而笔算是可以约分出来的得到权重为1）
		  if (mainDist<0.001)
			  weight = 1;
		  else//不是selectedFeature本身，遍历所有控制点，求出当前顶点与他们的距离
			  for (unsigned int j = 0;j < featureList.size();j++)
		  {
			  //得到控制点坐标
			  fp.x = mesh->vertices[3 * featureList[j] + 0];
			  fp.y = mesh->vertices[3 * featureList[j] + 1];
			  fp.z = mesh->vertices[3 * featureList[j] + 2];
			  //计算距离
			  t_length = (temp - fp).length();

			  //通过距离却定当前顶点是否就是该控制点本身，如果是，权重为0，理由如上
			  if(t_length<0.0009)
			  {
				  weight = 0;
				  break;
			  }

			  //距离的倒数作为计算权重的分母。
			  sum += (1.0 / t_length);

		  }

		  //判断上述运算是否通过特殊情况已经确定了权重,(即判断weight是否还是原来的0.5)，如果未改变，则按照算法计算。
		  if (weight < 0.6 && weight > 0.4)
			  weight = 1.0 / (sum*mainDist);
		  //赋值
		  mesh->vertices[3 * minDisIdx[i] + 0] += weight*Gaussian(mainDist)*vec.x;
		  mesh->vertices[3 * minDisIdx[i] + 1] -= weight*Gaussian(mainDist)*vec.y;
		  mesh->vertices[3 * minDisIdx[i] + 2] += weight*Gaussian(mainDist)*vec.z;
	  }

  }

  last_x = x;
  last_y = y;
}

void timf(int value)
{
  glutPostRedisplay();
  glutTimerFunc(1, timf, 0);
}

int main(int argc, char *argv[])
{
  WindWidth = 400;
  WindHeight = 400;
	
  GLfloat light_ambient[] = {0.0, 0.0, 0.0, 1.0};
  GLfloat light_diffuse[] = {0.8, 0.8, 0.8, 1.0};
  GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat light_position[] = {0.0, 0.0, 1.0, 0.0};

  glutInit(&argc, argv);
  glutInitWindowSize(WindWidth, WindHeight);
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  glutCreateWindow("Trackball Example");

  glutReshapeFunc(Reshape);
  glutDisplayFunc(Display);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glClearColor(0, 0, 0, 0);

  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);

  glEnable(GL_LIGHT0);
  glDepthFunc(GL_LESS);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  tbInit(GLUT_LEFT_BUTTON);
  tbAnimate(GL_TRUE);

  glutTimerFunc(40, timf, 0); // Set up timer for 40ms, about 25 fps

  // load 3D model
  mesh = glmReadOBJ("../data/head.obj");
  
  glmUnitize(mesh);
  glmFacetNormals(mesh);
  glmVertexNormals(mesh , 90.0);

  glutMainLoop();

  return 0;

}



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

#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

_GLMmodel *mesh;
int WindWidth, WindHeight;

int last_x , last_y;
int selectedFeature = -1;

// for store distance from mouse
vector<int>  minDisIdx;	//用于顺序存储顶点序号
vector<float> minDiss;
//mine

vector<int> featureList;




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
	/*for (int i = 0; i < minDisIdx.size(); i++)
	{
		int idx = minDisIdx[i];

		glVertex3fv((float *)&mesh->vertices[3 * idx]);
	}*/


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
	

	
			  for (int i = 0; i < mesh->numvertices; i++)
			  {
				  vector3 pt(mesh->vertices[3 * i + 0], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);
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


  }

  if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
  {
	  selectedFeature = -1;
	 
  }

  last_x = x;
  last_y = y;


}





float Gaussian(float distance)

{
	float gaussian;
	float sigama = 0.2;
	gaussian = 0.5*exp(-pow(distance, 2) / (2 * pow(sigama, 2)));

	return gaussian;
}






MatrixXf getPsiMatrix(int n)

{
	//Control points
	vector<vector3> cps;
	for (int i = 0; i < n; i++)
	{
		vector3 cp(mesh->vertices[3 * featureList[i] + 0], mesh->vertices[3 * featureList[i] + 1], mesh->vertices[3 * featureList[i] + 2]);
		cps.push_back(cp);
	}

	//Psis: 储存psi的二维矩阵，temp_dist:储存控制点之间的距离的临时变量
	MatrixXf psisMatrice(cps.size(), cps.size());

	float temp_dist;
	//为二维数组动态分配空间
	//建立Psis矩阵
	for (int i = 0; i < cps.size(); i++)
	{
		for (int j = 0; j < cps.size(); j++)
		{
			if (j == i)
			{
				psisMatrice(i, j) = 1;
			}
			else
			{
				temp_dist = (cps[i] - cps[j]).length();
				psisMatrice(i, j) = Gaussian(temp_dist);
			}
		}
	}
	return psisMatrice;
}

void motion(int x, int y)
{
  tbMotion(x, y);


  if(selectedFeature != -1)
  {
	  matrix44 m;
	  vector4 vec = vector4((float)(x - last_x) / 100.0f , (float)(y - last_y) / 100.0f , 0.0 , 1.0);
	  
	  gettbMatrix((float *)&m);
	  vec = m * vec;
	  
	  vector<vector3> temVecList;
	  vector3 temp1(vec.x, vec.y, vec.z);
	  temVecList.push_back(temp1);

	  mesh->vertices[3 * selectedFeature + 0] += 0.5*vec.x;
	  mesh->vertices[3 * selectedFeature + 1] -= 0.5*vec.y;
	  mesh->vertices[3 * selectedFeature + 2] += 0.5*vec.z;

	
	

	  for (int i = 0; i < mesh->numvertices; i++)

	  {
		  vector3 sum(0, 0, 0);
		  vector3 localP(mesh->vertices[3 * i + 0], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);
		  vector3 result(0,0,0);
		  float weights = 0;
		
		
		 

					  for (int j = 0; j < featureList.size(); j++)
					  {   													  
										  vector3 contrlP(mesh->vertices[3 * featureList[j] + 0], mesh->vertices[3 * featureList[j] + 1], mesh->vertices[3 * featureList[j] + 2]);
										  float distance = (contrlP - localP).length();
										  float Psi = Gaussian(distance);
										 
									if(featureList[j] == selectedFeature )
									{

									
										MatrixXf psiMatrix = getPsiMatrix(featureList.size());
										MatrixXf psiInvMatrix = psiMatrix.inverse();
										//vector3 tempVec(vec.x, vec.y, vec.z);
										weights = psiInvMatrix(1, j) + psiInvMatrix(0, j) + psiInvMatrix(2, j);
										for (int k = 0; k < featureList.size(); k++)
										{
											weights += psiInvMatrix(k, j);
										}
									

										mesh->vertices[3 * i + 0] += Psi*weights*vec.x;
										mesh->vertices[3 * i + 1] -= Psi*weights*vec.y;
										mesh->vertices[3 * i + 2] += Psi*weights*vec.z;
									

										
									}

									
									
									
					  }

					 

					
	  }



	  //int minIdx = 0;
	  //float minDis = 9999999.0f;
	  ////选取区域范围，获得该控制点周围点的idex 以及与控制点的距离
	  //int numPt = 32;			//决定周围选取多少个顶点
			//					//vector<int>  minDisIdx;	//用于顺序存储顶点序号
			//					//vector<float> minDiss;	//用于顺序存储顶点与鼠标的距离

			//					//初始化两个向量组

	  //minDisIdx.clear();
	  //minDiss.clear();
	  //for (int i = 0; i < numPt; i++)
	  //{
		 // minDiss.push_back(9999999.0f);
		 // minDisIdx.push_back(0);
	  //}


	  //// vector3 pos = Unprojection(vector2((float)x, (float)y));
	  //vector3 controlPoint(mesh->vertices[3 * selectedFeature + 0], mesh->vertices[3 * selectedFeature + 1], mesh->vertices[3 * selectedFeature + 2]);

	  //for (int i = 0; i < mesh->numvertices; i++)
	  //{
		 // vector3 pt(mesh->vertices[3 * i + 0], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);
		 // float dis = (controlPoint - pt).length();


		 // for (int j = 0; j < numPt; j++)
		 // {
			//  //每一个顶点的距离都与当前minDiss里面储存的每一个值对比，发现比其中一个值小就插入到minDiss，实现排序
			//  if (minDiss[j] >= dis)
			//  {
			//	  minDiss.insert(minDiss.begin() + j, dis);
			//	  minDisIdx.insert(minDisIdx.begin() + j, i);
			//	  minDiss.pop_back();
			//	  minDisIdx.pop_back();
			//	  break;
			//  }
			//  //因为插入了一个值，所以相应的删除队列尾部的值

		 // }

	  //}

	  ////选取区域范围，获得该控制点周围点的idex 以及与控制点的距离
		  
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


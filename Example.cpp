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


//������ҵ�������汾����Ӧ��example.cpp example1.cpp�ļ���
//example.cpp    ���W��Ȩ���㷨�Ǹ���ÿһ���������п��Ƶ�Ĺ�ϵ�����
//example1.cpp   ���W���㷨�Ǹ������п��Ƶ�Ĺ�ϵ����ģ��Ϳ��ϵķ������ơ�

//��Ч��������example.cpp��Ч���ȽϺá�

//��Ϊ����ǰ�ǵ繤ϵ�ģ���������ҵ�Ŀ������������˼·��
//��˼���ǿ��Ƶ����һ������ͷ�룬�����벻�ܻ���Ӱ�죬
//���Ƕ�ס����Ƥ�Ǹ��ݶ���������εģ���Ƥ�ϵĵ�Խ����ĳ���룬
//��Խ����Ӱ�죬������Ƕ�ס����һ�㣬����ȫ����Ӱ�죬�����������Ӱ��
//Ȼ���������Ҫ�󣬸�Ȩ�ص�Ӱ��Ҫƽ����Ȩ��֮��Ϊ1

//���ģ�ͣ� �������ģ��
//������貢��������ԽС������Խ�󣬵���Ϊ0������Ϊ�ܵ���������֧·�������Ϊ�ܵ���
//���� = ���룬 ���� = Ȩ�أ��ܵ��� = 1
//Ȼ���㷨������ˣ� �����֧·�ĵ絼��Ȼ������ܵ絼������ռ����
//�絼 = ����ĵ���
//����ÿ���������п��Ƶ��Ȩ��weight����һ��



_GLMmodel *mesh;
int WindWidth, WindHeight;

int last_x , last_y;
int selectedFeature = -1;
vector<int> featureList;

int numPt = 200;			//������Χѡȡ���ٸ�����
vector<int>  minDisIdx;	//����˳��洢�������
vector<float> minDiss;	//����˳��洢���������ľ���

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




	 //��ʼ������������
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
			  //ÿһ������ľ��붼�뵱ǰminDiss���洢���ÿһ��ֵ�Աȣ����ֱ�����һ��ֵС�Ͳ��뵽minDiss��ʵ������
			  if (minDiss[j] >= dis)
			  {
				  minDiss.insert(minDiss.begin() + j, dis);
				  minDisIdx.insert(minDisIdx.begin() + j, i);
				  minDiss.pop_back();
				  minDisIdx.pop_back();
				  break;
			  }
			  //��Ϊ������һ��ֵ��������Ӧ��ɾ������β����ֵ

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
	  //sfp: selectedFeature ����ά����
	  vector3 sfp(mesh->vertices[3 * selectedFeature + 0], mesh->vertices[3 * selectedFeature + 1], mesh->vertices[3 * selectedFeature + 2]);

	  //temp: ���浱ǰ����Ķ���
	  //fp�����浱ǰ����Ŀ��Ƶ�
	  vector3 temp, fp;

	  //mainDist�����浱ǰ��������selectedFeature�ľ���
	  //sum����Ȩ��ĸ�����ڼ���Ȩ��
	  //weight��Ȩ��
	  float mainDist, sum,t_length, weight = 0.5;

	  //����selectedFeature��Χ���е�
	  for (unsigned int i = 0; i < minDisIdx.size();i++)
	  {
		  sum = 0;
		  //temp�õ���ǰ����Ķ�������
		  temp.x = mesh->vertices[3 * minDisIdx[i] + 0];
		  temp.y = mesh->vertices[3 * minDisIdx[i] + 1];
		  temp.z = mesh->vertices[3 * minDisIdx[i] + 2];
		  //���㵱ǰ������selectedFeature�ľ���
		  mainDist = (temp - sfp).length();
		  
	  
		  //ͨ������ȴ����ǰ�����Ƿ����selectedFeature��������ǣ�Ȩ��Ϊ1�������������������������������Ϊ��Ȼ������㷨���������������������Ϊ������޷������ĸΪ��������Ȼ�������ǿ���Լ�ֳ����ĵõ�Ȩ��Ϊ1��
		  if (mainDist<0.001)
			  weight = 1;
		  else//����selectedFeature�����������п��Ƶ㣬�����ǰ���������ǵľ���
			  for (unsigned int j = 0;j < featureList.size();j++)
		  {
			  //�õ����Ƶ�����
			  fp.x = mesh->vertices[3 * featureList[j] + 0];
			  fp.y = mesh->vertices[3 * featureList[j] + 1];
			  fp.z = mesh->vertices[3 * featureList[j] + 2];
			  //�������
			  t_length = (temp - fp).length();

			  //ͨ������ȴ����ǰ�����Ƿ���Ǹÿ��Ƶ㱾������ǣ�Ȩ��Ϊ0����������
			  if(t_length<0.0009)
			  {
				  weight = 0;
				  break;
			  }

			  //����ĵ�����Ϊ����Ȩ�صķ�ĸ��
			  sum += (1.0 / t_length);

		  }

		  //�ж����������Ƿ�ͨ����������Ѿ�ȷ����Ȩ��,(���ж�weight�Ƿ���ԭ����0.5)�����δ�ı䣬�����㷨���㡣
		  if (weight < 0.6 && weight > 0.4)
			  weight = 1.0 / (sum*mainDist);
		  //��ֵ
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


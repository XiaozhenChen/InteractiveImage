#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <numeric>
#include <iostream>
#include <limits>
#include <Windows.h>
#include <gl/GL.h>
#include <glut.h>

#include "glm.h"
#include "mtxlib.h"
#include "trackball.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace Eigen;
using namespace std;

#include <stdlib.h>
#include <iostream>
#include "LeastSquaresSparseSolver.h"
#pragma comment(lib, "legacy_stdio_definitions.lib")
#ifdef __cplusplus
extern "C"
#endif
FILE __iob_func[3] = { __acrt_iob_func(0),__acrt_iob_func(1),__acrt_iob_func(2) };

extern "C" { FILE _iob[3] = { __acrt_iob_func(0),__acrt_iob_func(1),__acrt_iob_func(2) }; }
// ----------------------------------------------------------------------------------------------------
// global variables

_GLMmodel *mesh;

int WindWidth, WindHeight;
int last_x, last_y;
int select_x, select_y;

typedef enum { SELECT_MODE, DEFORM_MODE } ControlMode;
ControlMode current_mode = SELECT_MODE;

vector<float*> colors;
vector<vector<int> > handles;
vector<int> handlesList;
int selected_handle_id = -1;
int selected_handle_id_v = -1;
int contrlPoint_ID = -1;
bool deform_mesh_flag = false;
bool enter = false;

bool checkUnique(vector<int> vec, int idx);
vector<int>* getNeighborList();
vector<int>*  neighborList;
vector<float>*  weightList;
vector<float>* getNeighW();
vector<vector<vector3>> getEdge();
vector<vector<vector3>> edgeListOld;
vector<vector<vector3>> edgeListNew;
vector<vector<vector3>> edgeListNew1;
vector<float>* neighW;
vector<vector3> lap;
int numHandles = 0; //得到handles总共有多少个数。
int numVertices = 0;
//LeastSquaresSparseSolver solver, solver1, solver2;
LeastSquaresSparseSolver solver, solver1, solver2, solverNew, solver1New, solver2New, solver0, solver10, solver20;


// ----------------------------------------------------------------------------------------------------
// render related functions

void Reshape(int width, int height)
{
	int base = min(width, height);

	tbReshape(width, height);
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (GLdouble)width / (GLdouble)height, 1.0, 128.0);
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
	glColor3f(1.0, 1.0, 1.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glmDraw(mesh, GLM_SMOOTH);

	// render wire model
	glPolygonOffset(1.0, 1.0);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glLineWidth(1.0f);
	glColor3f(0.6, 0.0, 0.8);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glmDraw(mesh, GLM_SMOOTH);

	// render handle points
	glPointSize(10.0);
	glEnable(GL_POINT_SMOOTH);
	glDisable(GL_LIGHTING);
	glBegin(GL_POINTS);
	for (int handleIter = 0; handleIter<handles.size(); handleIter++)
	{
		glColor3fv(colors[handleIter%colors.size()]);
		for (int vertIter = 0; vertIter<handles[handleIter].size(); vertIter++)
		{
			int idx = handles[handleIter][vertIter];
			glVertex3fv((float *)&mesh->vertices[3 * idx]);
		}
	}
	glEnd();

	glPopMatrix();

	glFlush();
	glutSwapBuffers();
}

// ----------------------------------------------------------------------------------------------------
// mouse related functions

vector3 Unprojection(vector2 _2Dpos)
{
	float Depth;
	int viewport[4];
	double ModelViewMatrix[16];    // Model_view matrix
	double ProjectionMatrix[16];   // Projection matrix

	glPushMatrix();
	tbMatrix();

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, ModelViewMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, ProjectionMatrix);

	glPopMatrix();

	glReadPixels((int)_2Dpos.x, viewport[3] - (int)_2Dpos.y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &Depth);

	double X = _2Dpos.x;
	double Y = _2Dpos.y;
	double wpos[3] = { 0.0 , 0.0 , 0.0 };

	gluUnProject(X, ((double)viewport[3] - Y), (double)Depth, ModelViewMatrix, ProjectionMatrix, viewport, &wpos[0], &wpos[1], &wpos[2]);

	return vector3(wpos[0], wpos[1], wpos[2]);
}

vector2 projection_helper(vector3 _3Dpos)
{
	int viewport[4];
	double ModelViewMatrix[16];    // Model_view matrix
	double ProjectionMatrix[16];   // Projection matrix

	glPushMatrix();
	tbMatrix();

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, ModelViewMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, ProjectionMatrix);

	glPopMatrix();

	double wpos[3] = { 0.0 , 0.0 , 0.0 };
	gluProject(_3Dpos.x, _3Dpos.y, _3Dpos.z, ModelViewMatrix, ProjectionMatrix, viewport, &wpos[0], &wpos[1], &wpos[2]);

	return vector2(wpos[0], (double)viewport[3] - wpos[1]);
}

bool checkUnique(vector<int> vec, int idx)
{
	int length = vec.size();
	int i = 0;
	for (i = 0; i < length; i++)
	{
		if (vec[i] == idx)
			break;
	}
	if (i == length)
		return true;
	return false;
}

//使用时返回一个vector<int>[i]
//i就是第几个顶点，请勿使用i=0;
vector<int>* getNeighborList()
{
	//创建关系表并分配内存
	vector<int>* relations = (vector<int>*)calloc(mesh->numvertices + 1, sizeof(vector<int>));
	int triangleVecIdx[3];

	//初始化关系表
	for (int i = 0; i < mesh->numvertices + 1; i++)
	{
		relations[i] = *(new vector<int>);

	}

	//遍历所有三角形	
	//	cout << "Num=" << mesh->numtriangles << endl;
	for (unsigned long i = 0; i< mesh->numtriangles; i++)
	{
		triangleVecIdx[0] = mesh->triangles[i].vindices[0];
		triangleVecIdx[1] = mesh->triangles[i].vindices[1];
		triangleVecIdx[2] = mesh->triangles[i].vindices[2];

		//添加与triangleVecIdx[0]有关系的所有顶点

		if (checkUnique(relations[triangleVecIdx[0]], triangleVecIdx[1]))
			relations[triangleVecIdx[0]].push_back(triangleVecIdx[1]);
		if (checkUnique(relations[triangleVecIdx[0]], triangleVecIdx[2]))
			relations[triangleVecIdx[0]].push_back(triangleVecIdx[2]);

		//添加与triangleVecIdx[1]有关系的所有顶点
		if (checkUnique(relations[triangleVecIdx[1]], triangleVecIdx[0]))
			relations[triangleVecIdx[1]].push_back(triangleVecIdx[0]);
		if (checkUnique(relations[triangleVecIdx[1]], triangleVecIdx[2]))
			relations[triangleVecIdx[1]].push_back(triangleVecIdx[2]);

		//添加与triangleVecIdx[2]有关系的所有顶点
		if (checkUnique(relations[triangleVecIdx[2]], triangleVecIdx[0]))
			relations[triangleVecIdx[2]].push_back(triangleVecIdx[0]);
		if (checkUnique(relations[triangleVecIdx[2]], triangleVecIdx[1]))
			relations[triangleVecIdx[2]].push_back(triangleVecIdx[1]);
	}

	return relations;
}

//lap中的有效数字从[1]开始
vector<vector3> getLap()
{
	vector3 empty;
	vector<vector3> lap;
	lap.push_back(empty);//填充lap[0]，使lap中的有效数字从[1]开始

	int count, d;
	for (int i = 1; i < mesh->numvertices + 1; i++)
	{

		d = neighborList[i].size();
		vector3 self;
		self.x = mesh->vertices[i * 3];
		self.y = mesh->vertices[i * 3 + 1];
		self.z = mesh->vertices[i * 3 + 2];

		vector3 neighbor;
		neighbor.x = neighbor.y = neighbor.z = 0;

		for (int j = 0; j < d; j++)
		{
			neighbor.x += mesh->vertices[neighborList[i][j] * 3];
			neighbor.y += mesh->vertices[neighborList[i][j] * 3 + 1];
			neighbor.z += mesh->vertices[neighborList[i][j] * 3 + 2];
		}
		self.x -= neighbor.x / d;
		self.y -= neighbor.y / d;
		self.z -= neighbor.z / d;
		lap.push_back(self);
	}
	return lap;
}
float cot(vector3 v1, vector3 v2)
{
	float x1, x2, y1, y2, z1, z2;
	x1 = v1.x;
	y1 = v1.y;
	z1 = v1.z;

	x2 = v2.x;
	y2 = v2.y;
	z2 = v2.z;

	float   dot = x1*x2 + y1*y2 + z1*z2;
	float	lenSq1 = x1*x1 + y1*y1 + z1*z1;
	float	lenSq2 = x2*x2 + y2*y2 + z2*z2;
	float	angle = acos(dot / sqrt(lenSq1 * lenSq2));
	float  cotangen = 1 / (tanf(angle));
	return cotangen;

}

//[i=1][0]就是第1个顶点1st neighbor，请勿使用i=0;
vector<float>* getNeighW()
{

	//vector<float> weightIJ;
	//weightIJ.push_back(0);
	vector<vector3> P;
	vector3 neighbor;
	vector3 p1;
	vector3 p2;
	vector<float> empty;
	vector<float>* weightIJ = (vector<float>*)calloc(mesh->numvertices + 1, sizeof(vector<float>));
	vector3 edge1i, edge1j, edge2i, edge2j;
	//初始化关系表


	for (int i = 0; i < mesh->numvertices + 1; i++)
	{
		weightIJ[i] = *(new vector<float>);

	}

	int count, d, num;
	for (int i = 1; i < mesh->numvertices + 1; i++)
	{



		for (int j = 0; j < neighborList[i].size(); j++)
		{

			neighbor.x = neighbor.y = neighbor.z = 0;

			for (int k = 0; k < neighborList[neighborList[i][j]].size(); k++)
			{
				for (int h = 0; h < neighborList[i].size(); h++)
				{
					if (neighborList[neighborList[i][j]][k] == neighborList[i][h])
					{

						neighbor.x = mesh->vertices[neighborList[i][h] * 3];
						neighbor.y = mesh->vertices[neighborList[i][h] * 3 + 1];
						neighbor.z = mesh->vertices[neighborList[i][h] * 3 + 2];
						P.push_back(neighbor);
					}


				}




			}


			edge1i.x = mesh->vertices[3 * i + 0] - P[0].x;
			edge1i.y = mesh->vertices[3 * i + 1] - P[0].y;
			edge1i.z = mesh->vertices[3 * i + 2] - P[0].z;

			edge1j.x = mesh->vertices[3 * neighborList[i][j] + 0] - P[0].x;
			edge1j.y = mesh->vertices[3 * neighborList[i][j] + 1] - P[0].y;
			edge1j.z = mesh->vertices[3 * neighborList[i][j] + 2] - P[0].z;

			edge2i.x = mesh->vertices[3 * i + 0] - P[1].x;
			edge2i.y = mesh->vertices[3 * i + 1] - P[1].y;
			edge2i.z = mesh->vertices[3 * i + 2] - P[1].z;

			edge2j.x = mesh->vertices[3 * neighborList[i][j] + 0] - P[1].x;
			edge2j.y = mesh->vertices[3 * neighborList[i][j] + 1] - P[1].y;
			edge2j.z = mesh->vertices[3 * neighborList[i][j] + 2] - P[1].z;


			//在此求Cot 

			float cotang = 0.5*(cot(edge1i, edge1j) + cot(edge2i, edge2j));


			weightIJ[i].push_back(cotang);

			P.clear();
		}


	}
	return weightIJ;

}

//按照每个点neighbor的顺序，获取edge = pi-pj,从1开始计数
vector<vector<vector3>> getEdge()
{
	vector<vector3> empty;
	vector<vector<vector3>> edgeIJ;
	edgeIJ.push_back(empty);

	for (int i = 1; i < mesh->numvertices + 1; i++)
	{
		vector<vector3> P;

		for (int j = 0; j < neighborList[i].size(); j++)
		{
			vector3 temp;
			temp.x = -mesh->vertices[neighborList[i][j] * 3] + mesh->vertices[i * 3];
			temp.y = -mesh->vertices[neighborList[i][j] * 3 + 1] + mesh->vertices[i * 3 + 1];
			temp.z = -mesh->vertices[neighborList[i][j] * 3 + 2] + mesh->vertices[i * 3 + 2];

			P.push_back(temp);
		}
		edgeIJ.push_back(P);
	}
	return edgeIJ;

}

Matrix3f GetS(int i)
{
	Matrix3f Si;
	Si.setZero();
	float wi = 1;
	MatrixXf e(3, 1), e1(3, 1);
	for (int j = 0; j < neighborList[i].size(); j++)
	{

		e(0, 0) = edgeListOld[i][j].x;
		e(1, 0) = edgeListOld[i][j].y;
		e(2, 0) = edgeListOld[i][j].z;
		e1(0, 0) = mesh->vertices[3 * i + 0] - mesh->vertices[3 * neighborList[i][j] + 0];
		e1(1, 0) = mesh->vertices[3 * i + 1] - mesh->vertices[3 * neighborList[i][j] + 1];
		e1(2, 0) = mesh->vertices[3 * i + 2] - mesh->vertices[3 * neighborList[i][j] + 2];
		Si = Si + e*e1.transpose();
	}
	return Si;
}

Matrix3f GetR(int i)
{
	Matrix3f S = GetS(i);
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
	const Eigen::Matrix3f U = svd.matrixU();
	const Eigen::Matrix3f V = svd.matrixV();
	//const Eigen::VectorXf Ss = svd.singularValues();

	return V*U.transpose();
}

vector<Matrix3f> GetRList()
{
	vector<Matrix3f> RotList;
	Matrix3f empty3m;
	RotList.push_back(empty3m);
	for (int i = 1;i < mesh->numvertices + 1;i++)
	{
		Matrix3f Ri = GetR(i);
		RotList.push_back(Ri);
	}
	return RotList;
}

void mouse(int button, int state, int x, int y)
{
	tbMouse(button, state, x, y);

	if (current_mode == SELECT_MODE && button == GLUT_RIGHT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			select_x = x;
			select_y = y;
		}
		else
		{
			vector<int> this_handle;

			// project all mesh vertices to current viewport
			for (int vertIter = 0; vertIter<mesh->numvertices; vertIter++)
			{
				vector3 pt(mesh->vertices[3 * vertIter + 0], mesh->vertices[3 * vertIter + 1], mesh->vertices[3 * vertIter + 2]);
				vector2 pos = projection_helper(pt);

				// if the projection is inside the box specified by mouse click&drag, add it to current handle
				if (pos.x >= select_x && pos.y >= select_y && pos.x <= x && pos.y <= y)
				{
					this_handle.push_back(vertIter);
				}
			}
			handles.push_back(this_handle);
		}
	}
	// select handle
	else if (current_mode == DEFORM_MODE && button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
	{
		// project all handle vertices to current viewport
		// see which is closest to selection point
		double min_dist = 999999;
		int handle_id = -1;
		for (int handleIter = 0; handleIter<handles.size(); handleIter++)
		{
			for (int vertIter = 0; vertIter<handles[handleIter].size(); vertIter++)
			{

				int idx = handles[handleIter][vertIter];
				vector3 pt(mesh->vertices[3 * idx + 0], mesh->vertices[3 * idx + 1], mesh->vertices[3 * idx + 2]);
				vector2 pos = projection_helper(pt);
				handlesList.push_back(idx);
				double this_dist = sqrt((double)(pos.x - x)*(pos.x - x) + (double)(pos.y - y)*(pos.y - y));
				if (this_dist<min_dist)
				{
					min_dist = this_dist;
					handle_id = handleIter;
				}


			}
		}

		selected_handle_id = handle_id;
		deform_mesh_flag = true;
	}

	if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
		deform_mesh_flag = false;

	last_x = x;
	last_y = y;
}

void motion(int x, int y)
{
	tbMotion(x, y);

	// if in deform mode and a handle is selected, deform the mesh
	if (current_mode == DEFORM_MODE && deform_mesh_flag == true)
	{
		edgeListOld = getEdge();
		matrix44 m1;
		vector4 vec = vector4((float)(x - last_x) / 1000.0f, (float)(y - last_y) / 1000.0f, 0.0, 1.0);

		gettbMatrix((float *)&m1);
		vec = m1 * vec;
		//--------------------存一份原始的，每次用这些原始点去作为原始mesh.
		vector<vector3>  oldList;
		vector3 haha; // let posList be  start from 1;

		oldList.push_back(haha);
		for (int i = 1; i < mesh->numvertices + 1; i++)                                         //give the solution to the new mesh
		{
			vector3 old;
			old.x = mesh->vertices[3 * i + 0]; old.y = mesh->vertices[3 * i + 1];  old.z = mesh->vertices[3 * i + 2];
			oldList.push_back(old);
		}
		//----------------------------------------------------------------------
		// deform handle points
		for (int vertIter = 0; vertIter<handles[selected_handle_id].size(); vertIter++)
		{
			int idx = handles[selected_handle_id][vertIter];
			vector3 pt(mesh->vertices[3 * idx + 0] + vec.x, mesh->vertices[3 * idx + 1] + vec.y, mesh->vertices[3 * idx + 2] + vec.z);
			mesh->vertices[3 * idx + 0] = pt[0];
			mesh->vertices[3 * idx + 1] = pt[1];
			mesh->vertices[3 * idx + 2] = pt[2];
		}
		//------------------------------开始用上次作业的方法作一个猜测一个解----------------------

		int n = mesh->numvertices;
		int m = handlesList.size();


		solver.Create(n + m, n, 1);      //set up  (n+m)*n solver matrix for x,y,z 
		solver1.Create(n + m, n, 1);
		solver2.Create(n + m, n, 1);

		int  numNeighbor;
		float factor;

		for (int i = 0; i < n; i++)       // make up Laplcian matrix of the solver
		{
			numNeighbor = neighborList[i + 1].size();
			factor = (float)-1 / numNeighbor;

			for (int j = 0; j < numNeighbor; j++)
			{
				solver.AddSysElement(i, neighborList[i + 1][j] - 1, factor);
				solver1.AddSysElement(i, neighborList[i + 1][j] - 1, factor);
				solver2.AddSysElement(i, neighborList[i + 1][j] - 1, factor);
			}


			solver.AddSysElement(i, i, 1.0f);
			solver1.AddSysElement(i, i, 1.0f);
			solver2.AddSysElement(i, i, 1.0f);
		}


		for (int i = 0; i < m; i++)                                 // set b for the solver ,the weight is Ws is 3 ,Ws^2 =9
		{

			solver.AddSysElement(i + n, handlesList[i] - 1, 1.0f);
			solver1.AddSysElement(i + n, handlesList[i] - 1, 1.0f);
			solver2.AddSysElement(i + n, handlesList[i] - 1, 1.0f);

		}
		float **b = new float*[1];  //x
		b[0] = new float[n + m];

		float **b1 = new float*[1];  //y
		b1[0] = new float[n + m];

		float **b2 = new float*[1];  //z
		b2[0] = new float[n + m];

		for (int i = 0; i < n + m; i++)
		{
			if (i < n)
			{
				b[0][i] = lap[i + 1][0];
				b1[0][i] = lap[i + 1][1];
				b2[0][i] = lap[i + 1][2];
			}
			else
			{
				b[0][i] = mesh->vertices[3 * handlesList[i - n] + 0];
				b1[0][i] = mesh->vertices[3 * handlesList[i - n] + 1];
				b2[0][i] = mesh->vertices[3 * handlesList[i - n] + 2];
			}
		}
		solver.SetRightHandSideMatrix(b);
		solver1.SetRightHandSideMatrix(b1);
		solver2.SetRightHandSideMatrix(b2);

		// direct solver
		solver.CholoskyFactorization();
		solver.CholoskySolve(0);

		solver1.CholoskyFactorization();
		solver1.CholoskySolve(0);

		solver2.CholoskyFactorization();
		solver2.CholoskySolve(0);

		// iterative solver
		/*	solver.SetInitialGuess(0 , 0 , 0.0f);
		solver.SetInitialGuess(0 , 1 , 0.0f);
		solver.SetInitialGuess(0 , 2 , 0.0f);
		solver.ConjugateGradientSolve();*/


		for (int i = 0; i < n; i++)                                         //give the solution to the new mesh
		{
			mesh->vertices[3 * (i + 1) + 0] = solver.GetSolution(0, i);

			mesh->vertices[3 * (i + 1) + 1] = solver1.GetSolution(0, i);

			mesh->vertices[3 * (i + 1) + 2] = solver2.GetSolution(0, i);
		}


		// release
		solver.ResetSolver(0, 0, 0);
		delete[] b[0];
		delete[] b;


		solver1.ResetSolver(0, 0, 0);
		delete[] b1[0];
		delete[] b1;

		solver2.ResetSolver(0, 0, 0);
		delete[] b2[0];
		delete[] b2;
		// getchar();

		//-----------------上面是用上次作业的结果给出一个解，然后，用这个解来求Ri----------------


		for (int q = 0;q <10;q++)
		{


			//获取b
			vector<Matrix3f> RotList = GetRList();

			float **bx = new float*[1];  //x
			bx[0] = new float[n + m];

			float **by = new float*[1];  //y
			by[0] = new float[n + m];

			float **bz = new float*[1];  //z
			bz[0] = new float[n + m];

			for (int i = 0;i < mesh->numvertices;i++)
			{
				int size = edgeListOld[i + 1].size();
				MatrixXf bi(3, 1);
				bi.setZero();
				for (int j = 0;j < size;j++)
				{
					int neighbor = neighborList[i + 1][j];
					MatrixXf p(3, 1);
					p(0, 0) = edgeListOld[i + 1][j].x;
					p(1, 0) = edgeListOld[i + 1][j].y;
					p(2, 0) = edgeListOld[i + 1][j].z;

					
						bi = bi + (RotList[i + 1] + RotList[neighbor])*p*0.5*neighW[i+1][j]; // Wij = 0.5*(cont(x1) + cot(x2)) ;  j-1,j+1,j,i构成的四边形的权重
				}
				bx[0][i] = bi(0, 0);
				by[0][i] = bi(1, 0);
				bz[0][i] = bi(2, 0);
			}
			for (int i = 0;i < m;i++)
			{
				bx[0][n + i] = mesh->vertices[3 * handlesList[i] + 0];
				by[0][n + i] = mesh->vertices[3 * handlesList[i] + 1];
				bz[0][n + i] = mesh->vertices[3 * handlesList[i] + 2];
			}

			//-----------------------------------------------------开始代入Ri 求解 Lp'=b----------------
			solverNew.Create(n + m, n, 1);      //set up  (n+m)*n solver matrix for x,y,z 
			solver1New.Create(n + m, n, 1);
			solver2New.Create(n + m, n, 1);
			for (int i = 0; i < n; i++)       // make up Laplcian matrix of the solver
			{
				numNeighbor = neighborList[i + 1].size();
				float Wii = 0.0;
				for (int j = 0; j < numNeighbor; j++)
				{
					solverNew.AddSysElement(i, neighborList[i + 1][j] - 1, -neighW[i + 1][j]);
					solver1New.AddSysElement(i, neighborList[i + 1][j] - 1, -neighW[i + 1][j]);
					solver2New.AddSysElement(i, neighborList[i + 1][j] - 1, -neighW[i + 1][j]);
					Wii = Wii + neighW[i+1][j];
				}
				solverNew.AddSysElement(i, i, Wii);
				solver1New.AddSysElement(i, i, Wii);
				solver2New.AddSysElement(i, i, Wii);
			}
			for (int i = 0; i < m; i++)                                 // set b for the solver ,the weight is Ws is 3 ,Ws^2 =9
			{
				solverNew.AddSysElement(i + n, handlesList[i] - 1, 1.0f);
				solver1New.AddSysElement(i + n, handlesList[i] - 1, 1.0f);
				solver2New.AddSysElement(i + n, handlesList[i] - 1, 1.0f);

			}
			solverNew.SetRightHandSideMatrix(bx);
			solver1New.SetRightHandSideMatrix(by);
			solver2New.SetRightHandSideMatrix(bz);

			// direct solver
			solverNew.CholoskyFactorization();
			solverNew.CholoskySolve(0);

			solver1New.CholoskyFactorization();
			solver1New.CholoskySolve(0);

			solver2New.CholoskyFactorization();
			solver2New.CholoskySolve(0);

			for (int i = 0; i < n; i++)                                         //give the solution to the new mesh
			{
				mesh->vertices[3 * (i + 1) + 0] = solverNew.GetSolution(0, i);

				mesh->vertices[3 * (i + 1) + 1] = solver1New.GetSolution(0, i);

				mesh->vertices[3 * (i + 1) + 2] = solver2New.GetSolution(0, i);
			}

			solverNew.ResetSolver(0, 0, 0);
			delete[] bx[0];
			delete[] bx;

			solver1New.ResetSolver(0, 0, 0);
			delete[] by[0];
			delete[] by;

			solver2New.ResetSolver(0, 0, 0);
			delete[] bz[0];
			delete[] bz;
		}
	}

	last_x = x;
	last_y = y;
}

// ----------------------------------------------------------------------------------------------------
// keyboard related functions

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'd':
		current_mode = DEFORM_MODE;
		break;
	default:
	case 's':
		current_mode = SELECT_MODE;
		break;
	}
}

// ----------------------------------------------------------------------------------------------------
// main function

void timf(int value)
{
	glutPostRedisplay();
	glutTimerFunc(1, timf, 0);
}

int main(int argc, char *argv[])
{
	// compute SVD decomposition of a matrix m
	// SVD: m = U * S * V^T
	//Eigen::MatrixXf m = Eigen::MatrixXf::Random(3,2);
	//cout << "Here is the matrix m:" << endl << m << endl;
	//Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
	//const Eigen::Matrix3f U = svd.matrixU();
	//// note that this is actually V^T!!
	//const Eigen::Matrix3f V = svd.matrixV();
	//const Eigen::VectorXf S = svd.singularValues();

	WindWidth = 800;
	WindHeight = 800;

	GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat light_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
	GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_position[] = { 0.0, 0.0, 1.0, 0.0 };

	// color list for rendering handles
	float red[] = { 1.0, 0.0, 0.0 };
	colors.push_back(red);
	float yellow[] = { 1.0, 1.0, 0.0 };
	colors.push_back(yellow);
	float blue[] = { 0.0, 1.0, 1.0 };
	colors.push_back(blue);
	float green[] = { 0.0, 1.0, 0.0 };
	colors.push_back(green);

	glutInit(&argc, argv);
	glutInitWindowSize(WindWidth, WindHeight);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow("ARAP");

	glutReshapeFunc(Reshape);
	glutDisplayFunc(Display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);
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
	mesh = glmReadOBJ("../data/cactus_small.obj");

	glmUnitize(mesh);
	glmFacetNormals(mesh);
	glmVertexNormals(mesh, 90.0);

	neighborList = getNeighborList();
	lap = getLap();

	
	neighW = getNeighW();

	
	//Matrix2f m;
	//m(0, 0) = 1;
	//m(0, 1) = 2;
	//m(1, 0) = 3;
	//m(1, 1) = 4;

	//cout << "Here is the matrix m:" << endl << m << endl;
	//Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
	//const Eigen::Matrix2f U = svd.matrixU();
	//// note that this is actually V^T!!
	//const Eigen::Matrix2f V = svd.matrixV();
	//const Eigen::Vector2f S = svd.singularValues();
	//Matrix2f e;
	//e(0, 0) = S(0);
	//e(0, 1) = 0;
	//e(1, 0) = 0;
	//e(1, 1) = S(1);
	//cout << "Here is the matrix m1:" << endl << U*e*V.transpose() << endl;

	glutMainLoop();

	return 0;

}
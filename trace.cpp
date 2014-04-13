#include <iostream>
#include <string>
#include <math.h>
#include <algorithm>

#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include "util.h"

#include <flann/flann.hpp>
#include "dual.cpp"

using namespace std;
using namespace Eigen;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfRM;

typedef Dualn<Dualn<float, 2>, 3> DVal;
typedef Dualn<float,2> Dual2;
Matrix<DVal,3,1> deform(const Matrix<DVal,3,1>& pos, DVal theta1, DVal theta2)
{
	Matrix<DVal,3,1> ret;
	ret(0,0)=pos(0,0);
	ret(1,0)=pos(1,0);
	ret(2,0)=pos(2,0)+theta1*pos(0,0)*pos(0,0) + theta2*pos(1,0)*pos(1,0);
	return ret;
}


Matrix<Dual2,3,1> transformednormal(const Matrix<DVal,3,1>& point, const Matrix<Dual2,3,1>& normal, DVal theta1, DVal theta2)
{
	Matrix<DVal,3,1> deformation = deform(point,theta1,theta2);
	//The deformation matrix; it has entries of the form a + b*dtheta1 + c*dtheta2
	Matrix<Dual2,3,3> defmatrix;
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			defmatrix(i,j)=deformation(j,0).dval[i];
		}
	}
	Matrix<Dual2,3,1> transformednormal = (normal.transpose()*defmatrix.inverse()).transpose();
	Dual2 transformednormalnorm = pow((transformednormal.transpose() * transformednormal)(0,0),0.5);
	for(int i=0; i<3; i++)
	{
		transformednormal(i,0) = transformednormal(i,0)/transformednormalnorm;
	}
	return transformednormal;
}

int main(int argc, char** argv)
{
	map<string,parameter> params = loadConfig("trace.config");
	string input = params["input"].s;
	string output = params["output"].s;
	float eps = params["eps"].f;
	float factor = params["factor"].f;
	int maxepoch = params["maxepoch"].i;
	int maxinnerepoch = params["maxinnerepoch"].i;

	int c;
	while((c = getopt (argc, argv, "i:o:")) !=-1)
	{
		switch(c)
		{
			case 'i':
				input = optarg;
				break;
			case 'o':
				output = optarg;
				break;
		}
	}

	stringstream f1;
	f1 << input << "sample/";
	vector<MatrixXfRM> samples = eigenFromFile<MatrixXfRM>(f1.str());
	stringstream f2;
	f2 << input << "samplenormal/";
	vector<MatrixXfRM> samplenormals = eigenFromFile<MatrixXfRM>(f2.str());

	vector<MatrixXfRM> rigtrans(samples.size());
	Dual2 dx[] = {Dual2(1),Dual2(0),Dual2(0)};
	Dual2 dy[] = {Dual2(0),Dual2(1),Dual2(0)};
	Dual2 dz[] = {Dual2(0),Dual2(0),Dual2(1)};
	Dual2 zero2[] = {Dual2(0),Dual2(0),Dual2(0)};

	float theta1init = -0;
	float theta2init = -0;

	float dtheta1[] = {1,0};
	float dtheta2[] = {0,1};
	float zero2f[] = {0,0};

	DVal theta1 = DVal(Dual2(theta1init,dtheta1), zero2);
	DVal theta2 = DVal(Dual2(theta2init,dtheta2), zero2);

	cout << "Promoting samples..." << endl;
	vector<Matrix<DVal,Dynamic,Dynamic> >samplespromoted;
	vector<Matrix<Dual2,Dynamic,Dynamic> >normalspromoted;

	vector<Matrix<Dual2,Dynamic,Dynamic> > deformednormals;
	for(int cl=0; cl<samples.size(); cl++)
	{
		Matrix<DVal,Dynamic,Dynamic> sample(samples[cl].rows(), 3);
		Matrix<Dual2,Dynamic,Dynamic> samplenormal(samples[cl].rows(), 3);
		Matrix<Dual2,Dynamic,Dynamic> deformednormal(samples[cl].rows(), 3);
		for(int i=0; i<samples[cl].rows(); i++)
		{
			sample(i,0) = DVal(Dual2(samples[cl](i,0),zero2f),dx);
			sample(i,1) = DVal(Dual2(samples[cl](i,1),zero2f),dy);
			sample(i,2) = DVal(Dual2(samples[cl](i,2),zero2f),dz);

			samplenormal(i,0) = Dual2(samplenormals[cl](i,0));
			samplenormal(i,1) = Dual2(samplenormals[cl](i,1));
			samplenormal(i,2) = Dual2(samplenormals[cl](i,2));
		}
		samplespromoted.push_back(sample);
		normalspromoted.push_back(samplenormal);
		deformednormals.push_back(deformednormal);
	}
	for(int epoch=0; epoch<maxepoch; epoch++)
	{
		cout << "Computing total error..." << endl;
		//accumulate error
		Dual2 deriv(0);
		for(int cl=0; cl<samples.size(); cl++)
		{
			for(int j=0; j<samples[cl].rows(); j++)
			{
				deformednormals[cl].row(j)=transformednormal(samplespromoted[cl].row(j), normalspromoted[cl].row(j), theta1, theta2);
			}
		}
		for(int cl=0; cl<samples.size(); cl++)
		{
			Matrix<Dual2,Dynamic,Dynamic> centred = deformednormals[cl].rowwise() - deformednormals[cl].colwise().mean();
			Matrix<Dual2,Dynamic,Dynamic> cov = centred.adjoint() * centred;
			for(int a=0;a<3;a++)
			{
				for(int b=0; b<3;b++)
				{
					deriv+=cov(a,b)*cov(a,b);
				}
			}
		}
		cout << deriv.val << " + " << deriv.dval[0] << "dtheta1 + " << deriv.dval[1] << "dtheta2" << endl;
		cout << "theta1: " << theta1.val.val << endl;
		cout << "theta2: " << theta2.val.val << endl;
		theta1.val.val -= eps*deriv.dval[0];
		theta2.val.val -= eps*deriv.dval[1];
	}
	stringstream straighten;
	straighten << "straighten.config";
	ofstream f(straighten.str().c_str());
	f << "float theta1=" << theta1.val.val << endl;
	f << "float theta2=" << theta2.val.val << endl;
	f.close();
}

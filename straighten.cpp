#include <iostream>
#include <string>
#include <math.h>
#include <vector>

#include <map>
#include <Eigen/Core>
#include "util.h"

#include <flann/flann.hpp>

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfRM;
Matrix<float,3,1> deform(const Matrix<float,3,1>& pos, float theta1, float theta2)
{
	Matrix<float,3,1> ret;
	ret(0,0)=pos(0,0);
	ret(1,0)=pos(1,0);
	ret(2,0)=pos(2,0)+theta1*pos(0,0)*pos(0,0) + theta2*pos(1,0)*pos(1,0);
	return ret;
}
int main(int argc, char** argv)
{
	map<string,parameter> params = loadConfig("straighten.config");
	string input;
	string output;
	float theta1 = params["theta1"].f;
	float theta2 = params["theta2"].f;

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
	vector<MatrixXfRM> clouds = eigenFromFile<MatrixXfRM>(input);
	for(int i=0; i< clouds.size(); i++)
	{
		for(int j=0; j<clouds[i].rows(); j++)
		{
			clouds[i].row(j)=deform(clouds[i].row(j),theta1,theta2);
		}
	}

	eigenToFile<MatrixXfRM>(clouds, output);
}

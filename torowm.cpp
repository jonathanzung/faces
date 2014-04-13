#include <iostream>
#include <string>
#include <math.h>

#include <map>
#include <Eigen/Core>
#include "util.h"

using namespace std;
using namespace Eigen;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfRM;

int main(int argc, char** argv)
{
	map<string,parameter> params = loadConfig("torowm.config");
	string input = params["input"].s;
	string output = params["output"].s;
	float dt = params["dt"].f;
	float xfov = params["xfov"].f;
	float yfov = params["yfov"].f;

	int c;
	while((c = getopt (argc, argv, "i:o:n:")) !=-1)
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
	vector<MatrixXf> scans = eigenFromFile<MatrixXf>(input);
	vector<MatrixXfRM> clouds;
	for(int i=0; i<scans.size(); i++)
	{
		clouds.push_back(project(scans[i],xfov,yfov));
	}

	eigenToFile<MatrixXfRM>(clouds, output);
}

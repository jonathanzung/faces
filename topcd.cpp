#include <iostream>
#include <string>
#include <math.h>
#include <complex>

#include <pcl/io/pcd_io.h>

#include <map>
//#include <fstream>
//#include <stdio.h>
//#include <ctype.h>
//#include <unistd.h>

#include <Eigen/Core>
#include "util.h"

using namespace std;
using namespace pcl;

using namespace std;
using namespace Eigen;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfRM;

PointCloud<PointXYZ> toPCD(MatrixXfRM scan)
{
	PointCloud<PointXYZ> cloud;
	cloud.width = scan.rows();
	cloud.height = 1;
	cloud.points.resize(cloud.height*cloud.width);
	cloud.is_dense=false;
	int pointcount = 0;
	for(int i = 0; i < scan.rows(); i++)
	{
		cloud.points[i].x = scan(i,0);
		cloud.points[i].y = scan(i,1);
		cloud.points[i].z = scan(i,2);
	}
	return cloud; 
}

PointCloud<PointNormal> toPCD(MatrixXfRM scan, MatrixXfRM scannormal)
{
	PointCloud<PointNormal> cloud;
	cloud.width = scan.rows();
	cloud.height = 1;
	cloud.points.resize(cloud.height*cloud.width);
	cloud.is_dense=false;
	int pointcount = 0;
	for(int i = 0; i < scan.rows(); i++)
	{
		cloud.points[i].x = scan(i,0);
		cloud.points[i].y = scan(i,1);
		cloud.points[i].z = scan(i,2);

		cloud.points[i].normal_x = scannormal(i,0);
		cloud.points[i].normal_y = scannormal(i,1);
		cloud.points[i].normal_z = scannormal(i,2);
	}
	return cloud; 
}

int main(int argc, char** argv)
{
	map<string,parameter> params = loadConfig("topcd.config");
	string input = params["input"].s;
	string output = params["output"].s;
	int normal = params["normal"].i;

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
			case 'n':
				normal = atoi(optarg);
				break;
		}
	}
	if(normal==1)
	{
		stringstream f1;
		f1 << input << "sample/";
		vector<MatrixXfRM> samples = eigenFromFile<MatrixXfRM>(f1.str());
		stringstream f2;
		f2 << input << "samplenormal/";
		vector<MatrixXfRM> samplenormals = eigenFromFile<MatrixXfRM>(f2.str());
		for(int i = 0; i < samples.size(); i++)
		{
			PointCloud<PointNormal> cloud = toPCD(samples[i],samplenormals[i]);
			stringstream filename;
			filename << output << i << ".pcd";
			io::savePCDFileBinary(filename.str(), cloud);
		}
	}
	else
	{
		vector<MatrixXfRM> clouds = eigenFromFile<MatrixXfRM>(input);
		cout << "Saving pcd clouds to " << output << endl;
		for(int i = 0; i < clouds.size(); i++)
		{
			PointCloud<PointXYZ> cloud = toPCD(clouds[i]);
			stringstream filename;
			filename << output << i << ".pcd";
			io::savePCDFileBinary(filename.str(), cloud);
		}
	}
}

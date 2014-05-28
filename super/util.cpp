#include <iostream>
#include <map>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <cmath>
#include <math.h>

#include <boost/algorithm/string.hpp>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;
typedef PointXYZ PointT;
typedef PointSurfel PointNT;

struct parameter
{
	int i;
	float f;
	string s;
};

map<string, parameter> loadConfig(const char* filename)
{
	ifstream f;
	f.open(filename);
	string t;
	string param;
	string val;
	map<string, parameter> config;
	while(getline(f,t,' '))
	{
		getline(f,param,'=');
		getline(f,val);
		boost::algorithm::trim(param);
		parameter x;
		if(t=="int")
		{
			x.i= atoi(val.c_str());
		}
		else if(t == "string")
		{
			x.s=val;
		}
		else if(t=="float")
		{
			x.f=atof(val.c_str());
		}
		config[param]=x;
	}
	return config;
}

void savePCDFiles(vector<PointCloud<PointT>::Ptr> clouds, string output)
{
	cout << "Writing clouds to " << output << endl;
	for(int i=0; i<clouds.size(); i++)
	{
		io::savePCDFileBinary(output + boost::to_string(i) + ".pcd", *(clouds[i]));
	}

	stringstream setmeta;
	setmeta << output << "set.meta";
	ofstream f(setmeta.str().c_str());
	f << clouds.size();
	f.close();
}

vector<PointCloud<PointNT>::Ptr> loadPCDFiles(string input)
{
	cout << "Loading from " << input << endl;
	stringstream setmeta;
	setmeta << input << "set.meta";
	ifstream rm(setmeta.str().c_str());
	int n;
	rm >> n;

	vector<PointCloud<PointNT>::Ptr> scans;
	for(int i=0; i<n; i++)
	{
		PointCloud<PointNT>::Ptr scan(new PointCloud<PointNT>);
		io::loadPCDFile<PointNT>(input + boost::to_string(i) + ".pcd", *scan);
		scans.push_back(scan);
	}
	return scans;
}

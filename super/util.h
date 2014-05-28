#ifndef __util_H_INCLUDED__
#define __util_H_INCLUDED__

#include <map>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <ctype.h>
#include <string>
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
map<string, parameter> loadConfig(const char* filename);
void savePCDFiles(vector<PointCloud<PointT>::Ptr> clouds, string output);
vector<PointCloud<PointNT>::Ptr> loadPCDFiles(string input);

#endif

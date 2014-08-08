#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/vtk_lib_io.h>

#include <math.h>
#include <boost/algorithm/string.hpp>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
#include <utility>
#include <algorithm>
using namespace pcl;
using namespace std;
typedef PointXYZ PointT;

int main (int argc, char** argv)
{
	int nbs=1;
	float rad = 10;
	vector<int> indices(nbs);//temp vector to hold indices of the nearest neighbours found
	vector<float> sqr_distances(nbs);//temp vector to hold the distances to the nearest neighbours
	int total_num_found=0;
	int total_sociable=0;

	PointCloud<PointT>::Ptr scan1(new PointCloud<PointT>);
	PointCloud<PointT>::Ptr scan2(new PointCloud<PointT>);
	io::loadPCDFile<PointT>(argv[1], *scan1);
	io::loadPCDFile<PointT>(argv[2], *scan2);

	KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(scan2);
	float distance=0;
	int num_sociable=0;

	for(int j=0; j<scan1->points.size(); j++)
	{
		int num_found = kdtree.nearestKSearch((scan1->points)[j], 1, indices, sqr_distances);
		if(num_found > 0)
		{
			num_sociable++;
			distance += sqr_distances[0];
		}
	}
	float rmsdistance = sqrt((float)distance/num_sociable);
	cout << "RMS distance: " << rmsdistance << endl;
}

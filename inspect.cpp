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
	float rad = 11;
	for(int n=1; n<argc; n++)
	{

		vector<int> indices(nbs);//temp vector to hold indices of the nearest neighbours found
		vector<float> sqr_distances(nbs);//temp vector to hold the distances to the nearest neighbours
		int total_num_found=0;
		int total_sociable=0;

		PointCloud<PointT>::Ptr scan(new PointCloud<PointT>);
		io::loadPCDFile<PointT>(argv[n], *scan);

		cout << "Point count: " << scan->points.size() << endl;

		KdTreeFLANN<PointT> kdtree;
		kdtree.setInputCloud(scan);
		float distance=0;
		int num_sociable=0;

		for(int j=0; j<scan->points.size(); j++)
		{
			int num_found = kdtree.nearestKSearch((scan->points)[j], 2, indices, sqr_distances);
			if(num_found > 1)
			{
				num_sociable++;
				distance += sqrt(sqr_distances[1]);
			}
		}
		cout << "Average spacing: " << (float)distance/num_sociable << endl;

		float minx=10e6;
		float maxx=-10e6;
		float miny=10e6;
		float maxy=-10e6;
		float minz=10e6;
		float maxz=-10e6;

		for(int i=0; i<scan->points.size(); i++)
		{
			minx = min(scan->points[i].x,minx);
			maxx = max(scan->points[i].x,maxx);
			miny = min(scan->points[i].y,miny);
			maxy = max(scan->points[i].y,maxy);
			minz = min(scan->points[i].z,minz);
			maxz = max(scan->points[i].z,maxz);
		}
		cout << "Bounding box: ";
		cout << "(" << minx << "," << maxx << ")x";
		cout << "(" << miny << "," << maxy << ")x";
		cout << "(" << minz << "," << maxz << ")" << endl;
	}
}

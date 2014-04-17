#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/vtk_lib_io.h>

#include <stdio.h>
#include <ctype.h>
#include <unistd.h>
#include <map>


using namespace pcl;
using namespace std;
typedef PointXYZ PointT;
typedef PointNormal PointNT;

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

void flipNormals(PointCloud<PointNT>::Ptr cloud, PointCloud<PointNT>::Ptr ref)
{
	vector<int> indices;
	getApproximateIndices<PointNT>(cloud,ref,indices);
	int j;
	for(int i = 0; i < cloud->points.size(); i++)
	{
		j = indices[i];
		if(cloud->points[i].normal_x * ref->points[j].normal_x + cloud->points[i].normal_y * ref->points[j].normal_y + cloud->points[i].normal_z * ref->points[j].normal_z < 0)
		{
			cloud->points[i].normal_x *= -1;
			cloud->points[i].normal_y *= -1;
			cloud->points[i].normal_z *= -1;
		}
	}
}
int main (int argc, char** argv)
{
	map<string,parameter> params = loadConfig("graft.config");

	string input = params["input"].s;
	string output = params["output"].s;

	int poissonDepth = params["poissonDepth"].i;
	float mlsSearchRadius = params["mlsSearchRadius"].f;
	const float leaf_size = params["leaf_size"].f;
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

	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<PointNT>::Ptr cloud_raw (new pcl::PointCloud<PointNT>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile (input, cloud_blob);
	pcl::fromPCLPointCloud2 (cloud_blob, *cloud_raw);

	if(leaf_size > 0)
	{
		cout << "Downsampling..." << endl;
		VoxelGrid<PointNT> grid;
		grid.setLeafSize(leaf_size,leaf_size,leaf_size);
		grid.setInputCloud(cloud_raw);
		grid.filter(*cloud_raw);
	}
	PointCloud<PointNT>::Ptr cloud(new PointCloud<PointNT>);
	PointCloud<PointXYZ>::Ptr cloudxyz(new PointCloud<PointXYZ>);
	copyPointCloud(*cloud_raw, *cloudxyz);
	if(mlsSearchRadius > 0)
	{
		cout << "MLS..." << endl;
		//* the data should be available in cloud
		MovingLeastSquares<PointXYZ, PointNT> mls;
		pcl::search::KdTree<PointXYZ>::Ptr tree1 (new pcl::search::KdTree<PointXYZ>);
		mls.setComputeNormals(true);
		mls.setInputCloud(cloudxyz);
		mls.setSearchRadius(mlsSearchRadius);
		mls.setPolynomialFit(true);
		mls.setPolynomialOrder(1);
		mls.setSearchMethod(tree1);
		mls.process(*cloud);
		flipNormals(cloud,cloud_raw);
	}
	else
	{
		cloud = cloud_raw;
	}

	// Create search tree*
	pcl::search::KdTree<PointNT>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud);

	cout << "Poisson reconstruction..." << endl;
	// Initialize objects
	pcl::PolygonMesh triangles;
	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(poissonDepth);
	poisson.setInputCloud(cloud);
	poisson.setSearchMethod(tree2);
	poisson.performReconstruction(triangles);
	pcl::io::savePolygonFileSTL(output+"mesh.stl",triangles);
	cout << "saved "<< output << "mesh.stl" << endl;
	return (0);
}

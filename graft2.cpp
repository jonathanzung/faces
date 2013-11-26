#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/vtk_lib_io.h>
using namespace pcl;
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal PointNT;
int main (int argc, char** argv)
{
		// Load input file into a PointCloud<T> with an appropriate type
		pcl::PointCloud<PointNT>::Ptr cloud (new pcl::PointCloud<PointNT>);
		pcl::io::loadPCDFile<PointNT> (argv[1], *cloud);

		int poissonDepth = atoi(argv[2]);
		float mlsSearchRadius = atof(argv[3]);
		const float leaf_size = atof(argv[4]);

		if(leaf_size > 0)
		{
				VoxelGrid<PointNT> grid;
				grid.setLeafSize(leaf_size,leaf_size,leaf_size);
				grid.setInputCloud(cloud);
				grid.filter(*cloud);
		}

		// Create search tree*
		pcl::search::KdTree<PointNT>::Ptr tree2 (new pcl::search::KdTree<PointNT>);
		tree2->setInputCloud (cloud);

		cout << "Poisson reconstruction..." << endl;
		// Initialize objects
		pcl::PolygonMesh triangles;
		pcl::Poisson<PointNT> poisson;
		poisson.setDepth(poissonDepth);
		poisson.setInputCloud(cloud);
		poisson.setSearchMethod(tree2);
		poisson.performReconstruction(triangles);
		pcl::io::savePolygonFileSTL("mesh.stl",triangles);
		cout << "saved mesh.stl" << endl;
		return (0);
}
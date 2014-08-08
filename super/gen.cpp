#include "dep.h"
#include <math.h>
#include <unsupported/Eigen/MatrixFunctions>
using namespace pcl;
using namespace std;
using namespace Eigen;
typedef PointXYZ PointT;

inline PointT f(float u, float v)
{
	return  PointT(u,v,0.06*sin(10*u)+0.06*cos(10*v));
}

void project_to_view(PointCloud<PointT>::Ptr in, PointCloud<PointT>::Ptr transformed, PointCloud<PointT>::Ptr out, Matrix4f* orientation)
{
	Matrix4f m = orientation->inverse();
	transformPointCloud (*in, *transformed, m);
	transformPointCloud (*in, *out, m);
	for(int i=0; i<out->points.size(); i++)
	{
		out->points[i].x /= out->points[i].z;
		out->points[i].y /= out->points[i].z;
		out->points[i].z /= out->points[i].z;
	}
}

int main (int argc, char** argv)
{
	map<string,parameter> params = loadConfig("gen.config");
	float spine_eps= params["spine_eps"].f;
	float scan_eps= params["scan_eps"].f;
	float dtheta= params["dtheta"].f;
	float dx= params["dx"].f;
	float orientation_noise=params["orientation_noise"].f;
	float view_rad= params["view_rad"].f;
	int search_nbs = params["search_nbs"].i;
	float search_rad = params["search_rad"].f;
	int nscans=params["nscans"].i;
	string output = params["output"].s;

	int opt;
	while((opt = getopt (argc, argv, "o:")) !=-1)
	{
		switch(opt)
		{
			case 'o':
				output = optarg;
				break;
		}
	}

	PointCloud<PointT>::Ptr spine (new pcl::PointCloud<PointT>); //The high res point cloud
	PointCloud<PointT>::Ptr spine_projected (new pcl::PointCloud<PointT>); //The high res point cloud
	PointCloud<PointT>::Ptr spine_transformed (new pcl::PointCloud<PointT>); //The high res point cloud
	PointCloud<PointT>::Ptr merged (new pcl::PointCloud<PointT>); //The high res point cloud
	vector<Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > orientations;
	vector<PointCloud<PointT>::Ptr> scans;

	for(float u=-1.0; u<1.0; u+=spine_eps)
	{
		for(float v=-1.0; v<1.0; v+=spine_eps)
		{
			spine->points.push_back(f(u,v));
		}
	}

	for(int i=0; i<nscans; i++)
	{
		Matrix4f m;
		Matrix4f randmtemp = Matrix4f::Random();
		Matrix4f randm = Matrix4f::Random();
		float theta=(i-nscans/2)*dtheta;
		m << 1, 0, 0, i*dx,
		  0,cos(theta),-sin(theta),view_rad*sin(theta),
		  0,sin(theta),cos(theta),-view_rad*cos(theta),
		  0,0,0,1;
		randm = randmtemp - randmtemp.transpose();
		for(int i=0; i<4; i++)
		{
			randm(i,3)=0;
			randm(3,i)=0;
		}
		m=(orientation_noise*randm).exp()*m;
		orientations.push_back(m);
	}
	for(int i=0; i<nscans; i++)
	{
		Matrix4f m;
		Matrix4f randmtemp = Matrix4f::Random();
		Matrix4f randm = Matrix4f::Random();
		float theta=(i-nscans/2)*dtheta;
		m << cos(theta),0,-sin(theta),view_rad*sin(theta),
		  0,1,0,i*dx,
		  sin(theta),0,cos(theta),-view_rad*cos(theta),
		  0,0,0,1;
		randm = randmtemp - randmtemp.transpose();
		for(int i=0; i<4; i++)
		{
			randm(i,3)=0;
			randm(3,i)=0;
		}
		m=(orientation_noise*randm).exp()*m;
		orientations.push_back(m);
	}

	PointCloud<PointT>::Ptr scan_projected(new pcl::PointCloud<PointT>);
	for(float u=-1.0; u<1.0; u+=scan_eps)
	{
		for(float v=-1.0; v<1.0; v+=scan_eps)
		{
			scan_projected->push_back(PointT(u,v,1));
		}
	}
	KdTreeFLANN<PointT> scan_projected_kdtree;
	scan_projected_kdtree.setInputCloud(scan_projected);

	for(int i=0; i<2*nscans; i++)
	{
		PointCloud<PointT>::Ptr scan(new pcl::PointCloud<PointT>);
		VectorXf depths(scan_projected->points.size());
		VectorXf counts(scan_projected->points.size());
		depths.setZero();
		counts.setZero();
		project_to_view(spine, spine_transformed, spine_projected, &(orientations[i]));
		vector<int> indices(search_nbs);
		vector<float> sqr_distances(search_nbs);

		for(int j=0; j<spine_projected->points.size(); j++)
		{
			int num_found = scan_projected_kdtree.radiusSearch(spine_projected->points[j], 
					search_rad, indices, sqr_distances, search_nbs);
			for(int k=0; k<num_found; k++)
			{
				depths(indices[k])+=spine_transformed->points[j].z;
				counts(indices[k])+=1.0;
			}
		}
		for(int j=0; j<scan_projected->points.size();j++)
		{
			if(counts(j)>0)
			{
				PointT p = scan_projected->points[j];
				p.x*=depths(j)/counts(j);
				p.y*=depths(j)/counts(j);
				p.z*=depths(j)/counts(j);
				scan->points.push_back(p);
			}
		}
		scans.push_back(scan);
		PointCloud<PointT>::Ptr scan_transformed(new pcl::PointCloud<PointT>);
		transformPointCloud (*scan, *scan_transformed, orientations[i]);
		*merged += *scan_transformed;
	}
	io::savePCDFileBinary(output+"spine.pcd", *spine);
	io::savePCDFileBinary(output+"spine_projected.pcd", *spine_projected);
	io::savePCDFileBinary(output+"spine_transformed.pcd", *spine_transformed);
	io::savePCDFileBinary(output+"spine_transformed.pcd", *spine_transformed);
	io::savePCDFileBinary(output+"merged.pcd", *merged);
	eigenToFile<Eigen::Matrix4f>(orientations, output+"orientations/");
	savePCDFiles(scans, output+"scans/");

	return (0);
}

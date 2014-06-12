#include "dep.h"
using namespace pcl;
using namespace std;
using namespace Eigen;
typedef PointXYZ PointT;
typedef Map<MatrixXf,Aligned,OuterStride<> > MatrixXfMap;

//Set row sums to 1, and return the row sums
VectorXf normalize(SparseMatrix<float>* A)
{
	VectorXf degrees(A->rows());
	degrees.setZero();
	for(int i=0; i<A->outerSize(); ++i)
	{
		for(SparseMatrix<float>::InnerIterator it(*A,i); it; ++it)
		{
			degrees(it.row())+= it.value();
		}
	}

	for(int i=0; i<A->outerSize(); ++i)
	{
		for(SparseMatrix<float>::InnerIterator it(*A,i); it; ++it)
		{
			it.valueRef()/=degrees(it.row());
		}
	}
	return degrees;
}

//Project the point cloud 'in' to 2d as viewed from 'viewpoint', and save the result to 'out'
void projectToView(PointCloud<PointT>::Ptr in, PointCloud<PointT>::Ptr out, Matrix4f* viewpoint)
{
	Matrix4f m = viewpoint->inverse();
	pcl::transformPointCloud (*in, *out, m);
	for(int i=0; i<out->points.size(); i++)
	{
		out->points[i].x /= out->points[i].z/50;
		out->points[i].y /= out->points[i].z/50;
		out->points[i].z /= out->points[i].z/50;
	}
}

void responsibilities(vector<Eigen::Triplet<float> >* tripletListi,KdTreeFLANN<PointT>* projected_scan_kdtree, PointCloud<PointT>::Ptr projected_scan, PointCloud<PointT>::Ptr projected_spine, int nbs, float rad, float var,int offset)
{
	int n = projected_spine->points.size();
	int m = projected_scan->points.size();

	vector<int> indices(nbs);//temp vector to hold indices of the nearest neighbours found
	vector<float> sqr_distances(nbs);//temp vector to hold the distances to the nearest neighbours

	for(int j=0; j<n; j++)//Iterate through high res points and find nearest neighbours
	{
		int num_found = projected_scan_kdtree->radiusSearch((projected_spine->points)[j], rad, indices, sqr_distances, nbs);//radiusSearch returns the number of neighbours found and writes to indices and sqr_distances
		for(int k=0; k<num_found; k++)
		{
			for(int l=0; l<3; l++)
			{
				tripletListi->push_back(Eigen::Triplet<float>(3*indices[k]+l+3*offset,3*j+l,1));//exp(-var*sqr_distances[k])));
			}
		}
	}
}
 
int main (int argc, char** argv)
{
	map<string,parameter> params = loadConfig("super.config");

	string input = params["input"].s; //Input folder. Make sure it ends with a slash.
	string output = params["output"].s; //Output folder. Make sure it ends with a slash.
	string orientationpath = params["orientations"].s;
	string stl = params["stl"].s; //location of the mesh guess in stl format
	int nbs = params["nbs"].i; //Number of nearest neighbours to look for
	float rad = params["rad"].f; //Radius to search for nearest neighbours
	float var = params["var"].f; //The variance to be used for the Gaussian around each high res point
	int max_epoch = params["max_epoch"].i; //Maximum number of outer iterations
	int maxit = params["maxit"].i; //Number of allowed ICP iterations
	int cgmaxit = params["cgmaxit"].i;


	int opt;
	while((opt = getopt (argc, argv, "i:o:s:r:")) !=-1)
	{
		switch(opt)
		{
			case 'i':
				input = optarg;
				break;
			case 'o':
				output = optarg;
				break;
			case 'r':
				orientationpath = optarg;
				break;
			case 's':
				stl = optarg;
				break;
		}
	}

	vector<PointCloud<PointT>::Ptr> scans;
	vector<PointCloud<PointT>::Ptr> scans_temp;
	vector<PointCloud<PointT>::Ptr> scans_projected;

	cout << "Loading " + input + "*.pcd..." << endl;
	scans = loadPCDFiles(input);
	scans_temp = loadPCDFiles(input);
	scans_projected = loadPCDFiles(input);

	cout << "Grabbing Eigen clouds..." << endl;
	vector<MatrixXfMap > matrix_scans; //The low res scans as 3xn eigen matrices
	vector<MatrixXfMap > matrix_scans_projected; //The low res scans as 3xn eigen matrices
	vector<KdTreeFLANN<PointT>* > projected_kdtrees;

	vector<Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > orientations = eigenFromFile<Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >(orientationpath); 
	vector<VectorXf> confidence;
	for(int i=0; i<scans.size(); i++)
	{
		matrix_scans.push_back(scans[i]->getMatrixXfMap(3,4,0));
		matrix_scans.push_back(scans_projected[i]->getMatrixXfMap(3,4,0));
		VectorXf t(scans[i]->points.size());
		for(int j=0; j<scans[i]->points.size(); j++)
		{
			t[j]=1;
		}
	}
	Matrix4f id=Eigen::Matrix4f::Identity();
	for(int i=0; i<scans.size(); i++)
	{
		projectToView(scans_projected[i], scans_projected[i], &(id));
		projected_kdtrees.push_back(new KdTreeFLANN<PointT>);
		projected_kdtrees[i]->setInputCloud(scans_projected[i]);
	}

	cout << "Reading the spine from " << stl << endl; //"spine" is the name I use for the high res point cloud/mesh
	PointCloud<PointT>::Ptr spine (new pcl::PointCloud<PointT>); //The high res point cloud
	PointCloud<PointT>::Ptr projected_spine (new pcl::PointCloud<PointT>); //The high res point cloud
	PolygonMesh spine_mesh;
	io::loadPolygonFileSTL(stl, spine_mesh);
	fromPCLPointCloud2(spine_mesh.cloud, *spine);
	MatrixXfMap matrix_spine = spine->getMatrixXfMap(3,4,0); //The high res cloud as an eigen matrix
	int n = spine->points.size();

	cout << "Spine size: " << n << " vertices, " << spine_mesh.polygons.size() << " polygons" << endl;

	for(int epoch=0; epoch<max_epoch; epoch++)
	{
		cout << "Epoch " << epoch << endl;

		/*
		   cout << "\tAligning point clouds..." << endl;
		   {
		   for(int i=0; i < scans.size(); i++)//Use ICP to align each low res scan to the spine
		   {
		   cout << "\t\tScan " + boost::to_string(i) + "..." << endl;
		   IterativeClosestPoint<PointT, PointT> icp;
		   icp.setTransformationEpsilon(1e-7);
		   icp.setMaxCorrespondenceDistance(0.05);
		   icp.setInputSource(scans[i]);
		   icp.setInputTarget(spine);
		   icp.setMaximumIterations(maxit);
		   icp.align(*(scans_temp[i]),orientations[i]);
		   orientations[i] = icp.getFinalTransformation();
		   cout << "\t\tScore: " << icp.getFitnessScore() << endl;
		   }
		   }
		   */
		vector<int> partial_sums(scans.size()+1);
		partial_sums[0]=0;
		for(int i=0; i<scans.size(); i++)
		{
			partial_sums[i+1]= scans[i]->points.size() + partial_sums[i];
		}
		vector<Eigen::Triplet<float> > tripletListi;//holds the triplets from which we construct A
		SparseMatrix<float> A(3*partial_sums[scans.size()],3*n);
		for(int i=0; i<scans.size(); i++)
		{
			projectToView(spine, projected_spine, &(orientations[i]));
			responsibilities(&tripletListi,projected_kdtrees[i],scans_projected[i],projected_spine,nbs,rad,var,partial_sums[i]);
		}
		A.setFromTriplets(tripletListi.begin(),tripletListi.end());

		VectorXf degrees = normalize(&A);

		VectorXf b(A.rows());
		VectorXf x(A.cols());
		for(int i=0; i<scans.size(); i++)
		{
			for(int k=0; k<scans[i]->points.size(); k++)
			{
				b(3*(partial_sums[i]+k))=scans[i]->points[k].x;
				b(3*(partial_sums[i]+k)+1)=scans[i]->points[k].y;
				b(3*(partial_sums[i]+k)+2)=scans[i]->points[k].z;
			}
		}
		for(int i=0; i<A.rows(); i++)
		{
			if(degrees(i)==0)
			b(i)=0;
		}
		for(int i=0; i<spine->points.size(); i++)
		{
			x(3*i)=spine->points[i].x;
			x(3*i+1)=spine->points[i].y;
			x(3*i+2)=spine->points[i].z;
		}

		BiCGSTAB<SparseMatrix<float> > solver;
		SparseMatrix<float> B = A.transpose() * A;
		cout << "Error: " << (A*x-b).squaredNorm() << endl;
		solver.compute(B);
		solver.setMaxIterations(cgmaxit);
		VectorXf c(A.cols());
		c=A.transpose() * b;
		x = solver.solveWithGuess(c,x);
		cout << "Error: " << (A*x-b).squaredNorm() << endl;
		std::cout << "Solver iterations: " << solver.iterations() << std::endl;

		for(int i=0; i<spine->points.size(); i++)
		{
			spine->points[i].x=x(3*i);
			spine->points[i].y=x(3*i+1);
			spine->points[i].z=x(3*i+2);
		}
		for(int i=0; i<scans.size(); i++)
		{
			for(int k=0; k<scans[i]->points.size(); k++)
			{
				scans[i]->points[k].x=b(3*(partial_sums[i]+k));
				scans[i]->points[k].y=b(3*(partial_sums[i]+k)+1);
				scans[i]->points[k].z=b(3*(partial_sums[i]+k)+2);
			}
		}
	}

	cout << "Saving point clouds..." << endl;
	savePCDFiles(scans, output);

	cout << "Writing to " << output << "mesh.stl" << endl;
	toPCLPointCloud2(*spine, spine_mesh.cloud);
	io::savePolygonFileSTL(output + "mesh.stl", spine_mesh);
	io::savePCDFileBinary(output + "spine.pcd", *(spine));

	return (0);
}

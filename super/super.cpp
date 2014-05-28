#include "dep.h"
using namespace pcl;
using namespace std;
using namespace Eigen;
typedef PointXYZ PointT;
typedef PointSurfel PointNT;

int main (int argc, char** argv)
{
	map<string,parameter> params = loadConfig("super.config");

	string input = params["input"].s; //Input folder. Make sure it ends with a slash.
	string output = params["output"].s; //Output folder. Make sure it ends with a slash.
	string stl = params["stl"].s; //location of the mesh guess in stl format
	int nbs = params["nbs"].i; //Number of nearest neighbours to look for
	float rad = params["rad"].f; //Radius to search for nearest neighbours
	float var = params["var"].f; //The variance to be used for the Gaussian around each high res point
	int relax_it = params["relax_it"].i; //Number of times to perform smoothing on each iteration
	float narc = params["narc"].f;
	int max_epoch = params["max_epoch"].i; //Maximum number of outer iterations
	int maxit = params["maxit"].i; //Number of allowed ICP iterations
	int align = params["align"].i; //Set 1 to run ICP on each epoch, 0 for no ICP


	int opt;
	while((opt = getopt (argc, argv, "i:o:s:")) !=-1)
	{
		switch(opt)
		{
			case 'i':
				input = optarg;
				break;
			case 'o':
				output = optarg;
				break;
			case 's':
				stl = optarg;
				break;
		}
	}

	vector<PointCloud<PointT>::Ptr> scans;
	vector<PointCloud<PointNT>::Ptr> scansNT;

	cout << "Loading " + input + "*.pcd..." << endl;
	scansNT = loadPCDFiles(input);

	cout << "Grabbing Eigen clouds..." << endl;
	vector<Map<MatrixXf, Aligned, OuterStride<> > > matrix_scans; //The low res scans as 3xn eigen matrices
	vector<VectorXf> confidence;
	for(int i=0; i<scans.size(); i++)
	{
		matrix_scans.push_back(scans[i]->getMatrixXfMap(3,4,0));
		VectorXf t(scans[i]->points.size());
		for(int j=0; j<scans[i]->points.size(); j++)
		{
			t[j]=scansNT[i]->points[j].confidence;
		}
	}

	cout << "Reading the spine from " << stl << endl; //"spine" is the name I use for the high res point cloud/mesh
	PointCloud<PointT>::Ptr spine (new pcl::PointCloud<PointT>); //The high res point cloud
	PolygonMesh spine_mesh;
	io::loadPolygonFileSTL(stl, spine_mesh);
	fromPCLPointCloud2(spine_mesh.cloud, *spine);
	Map<MatrixXf, Aligned, OuterStride<> > matrix_spine = spine->getMatrixXfMap(3,4,0); //The high res cloud as an eigen matrix
	int n = spine->points.size(); //Number of points in the high res cloud

	cout << "Spine size: " << n << " vertices, " << spine_mesh.polygons.size() << " polygons" << endl;

	cout << "Constructing adjacency matrix..." << endl;	
	SparseMatrix<float> adj(n,n); //The adjacency matrix for the high res mesh. 
	{
		vector<Eigen::Triplet<float> > tripletList;
		tripletList.reserve(8*n);
		vector<uint32_t>* face;
		for(int i=0; i<spine_mesh.polygons.size(); i++)
		{
			face = &(spine_mesh.polygons[i].vertices);
			for(int j=0; j<face->size(); j++)
			{
				tripletList.push_back(Eigen::Triplet<float>((*face)[j],(*face)[(j+1)%(face->size())],1));
				tripletList.push_back(Eigen::Triplet<float>((*face)[(j+1)%(face->size())],(*face)[j],1));
			}
		}
		adj.setFromTriplets(tripletList.begin(),tripletList.end());
	}

	cout << "Constructing normalized adjacency matrix..." << endl;
	SparseMatrix<float> nadj(n,n); //The adjacency matrix with columns normalized to 1;
	//Used for the smoothing operation
	{
		VectorXf degrees(n);
		degrees.setZero();
		for(int i=0; i<adj.outerSize(); ++i)
		{
			for(SparseMatrix<float>::InnerIterator it(adj,i); it; ++it)
			{
				degrees(i)+= it.value();
			}
		}

		nadj = adj;
		for(int i=0; i<nadj.outerSize(); ++i)
		{
			for(SparseMatrix<float>::InnerIterator it(nadj,i); it; ++it)
			{
				it.valueRef()/=degrees(i);
			}
		}
	}

	for(int epoch=0; epoch<max_epoch; epoch++)
	{
		cout << "Epoch " << epoch << endl;

		if(align == 1)
		{
			cout << "\tAligning point clouds..." << endl;
			for(int i=0; i < scans.size(); i++)//Use ICP to align each low res scan to the spine
			{
				cout << "\t\tScan " + boost::to_string(i) + "..." << endl;
				IterativeClosestPoint<PointT, PointT> icp;
				icp.setTransformationEpsilon(1e-7);
				icp.setMaxCorrespondenceDistance(0.05);
				icp.setInputSource(scans[i]);
				icp.setInputTarget(spine);
				icp.setMaximumIterations(maxit);
				icp.align(*(scans[i]));
				cout << "\t\tScore: " << icp.getFitnessScore() << endl;
			}
		}
		cout << "\tComputing responsibilities..." << endl;
		vector<SparseMatrix<float>* > resp;
		//resp[i] is a matrix assigning points of the i^{th} scan to points in the high res cloud
		//resp[i] is calculated using a Gaussian weight.
		//For the j^{th} point of the i^{th} scan, we find is nearest neighbours with indices {k_1,k_2,..,k_10} in the high res cloud
		//Then we for t in {1,...,10} we set resp[i](j,k_t)=exp(-d_t^2) where d_t is the distance from the low res points to the t^{th} nearest high res point.
		//Later we'll do the required normalization
		KdTreeFLANN<PointT> spine_kdtree;//kdtree holding the high res cloud
		spine_kdtree.setInputCloud(spine);

		vector<int> indices(nbs);//temp vector to hold indices of the nearest neighbours found
		vector<float> sqr_distances(nbs);//temp vector to hold the distances to the nearest neighbours

		for(int i=0; i<scans.size(); i++)
		{
			cout << "\t\tScan " + boost::to_string(i) + "..." << endl;
			resp.push_back(new SparseMatrix<float>(scans[i]->points.size(),n));

			vector<Eigen::Triplet<float> > tripletListi;//holds the triplets from which we construct resp
			for(int j=0; j<scans[i]->points.size(); j++)//Iterate through low res points and find nearest neighbours
			{
				int num_found = spine_kdtree.radiusSearch((scans[i]->points)[j], rad, indices, sqr_distances, nbs);//radiusSearch returns the number of neighbours found and writes to indices and sqr_discances
				for(int k=0; k<num_found; k++)
				{
					tripletListi.push_back(Eigen::Triplet<float>(j,indices[k],exp(-var*sqr_distances[k])));
				}
				if(j%10000 == 0)//sanity check: print some sample distances and responsibilities
				{
					cout << num_found << endl;
					for(int k=0; k<num_found; k++)
					{
						cout << sqr_distances[k] << " ";
					}
					cout << endl;
					for(int k=0; k<num_found; k++)
					{	
						cout << exp(-var*sqr_distances[k]) << " ";
					}
					cout << endl << endl;
				}

			}
			resp[i]->setFromTriplets(tripletListi.begin(),tripletListi.end());
		}
		cout << "\tNormalizing responsibilities..." << endl;
		VectorXf ZZ(spine->points.size());//nx1 vector holding the total responsibility assigned to each high res point
		ZZ.setZero();
		for(int i=0; i<scans.size(); i++)
		{
			cout << "\t\tScan " + boost::to_string(i) + "..." << endl;
			VectorXf Z(scans[i]->points.size());
			Z.setZero();

			for(int j=0; j<(*resp[i]).outerSize(); ++j)
			{
				for(SparseMatrix<float>::InnerIterator it(*resp[i],j); it; ++it)
				{
					Z(it.row()) += it.value();
				}
				for(SparseMatrix<float>::InnerIterator it(*resp[i],j); it; ++it)
				{
					if(Z(it.row()) > 0)
					{
						//First we have to make sure the total responsibility coming from each of the low res points is 1
						//That's the point of dividing by Z(it.row())
						//Next we add those add those responsibilities to ZZ
						it.valueRef() /= Z(it.row());
						ZZ(it.col()) += it.value();
					}
				}
			}
		}
		cout << "\tUpdating spine..." << endl;
		//Now we set each high res point to the weighted average of its low res neighbours
		//To slow down the movement, we add give each high res point a weight of narc
		//pulling it back towards its current position

		for(int i=0; i<n; i++)
		{
			ZZ(i) += narc;
		}
		for(int i=0; i<relax_it; i++)//Smooth the high res cloud
		{
			matrix_spine = matrix_spine * nadj;
		}
		matrix_spine*=narc;
		for(int i=0; i<scans.size(); i++)
		{
			matrix_spine += matrix_scans[i]*(*(resp[i]));
		}

		for(int i=0; i<n; i++)
		{
			for(int j=0; j<3; j++)
			{
				matrix_spine(j,i) /= ZZ(i);
			}
		}
		for(int i=0; i<scans.size(); i++)
		{
			delete resp[i];
		}
	}
	cout << "Saving point clouds..." << endl;
	savePCDFiles(scans, output);

	cout << "Writing to " << output << "mesh.stl" << endl;
	toPCLPointCloud2(*spine, spine_mesh.cloud);
	io::savePolygonFileSTL(output + "mesh.stl", spine_mesh);

	return (0);
}

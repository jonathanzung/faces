#include "dep.h"
using namespace pcl;
using namespace std;
using namespace Eigen;
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
	for(int i=0; i<clouds.size(); i++)
	{
		io::savePCDFileBinary(output + boost::to_string(i) + ".pcd", *(clouds[i]));
	}
}

vector<PointCloud<PointNT>::Ptr> loadPCDFiles(string input, int start,int end)
{
	vector<PointCloud<PointNT>::Ptr> scans;
	for(int i=start; i<=end; i++)
	{
		PointCloud<PointNT>::Ptr scan(new PointCloud<PointNT>);
		io::loadPCDFile<PointNT>(input + boost::to_string(i) + ".pcd", *scan);
		scans.push_back(scan);
	}
	return scans;
}

int main (int argc, char** argv)
{
	map<string,parameter> params = loadConfig("super.config");

	string folder = params["folder"].s;
	string output = params["output"].s;
	int start1 = params["start1"].i;
	int end1 = params["end1"].i;
	string stl = params["stl"].s;
	int nbs = params["nbs"].i;
	float var = params["var"].f;
	int relax_it = params["relax_it"].i;
	float narc = params["narc"].f;
	int max_epoch = params["max_epoch"].i;
	float rad = params["rad"].f;
	int maxit = params["maxit"].i;
	int vis = params["vis"].i;
	int align = params["align"].i;

	int c;

	while((c = getopt (argc, argv, "i:o:s:n:")) !=-1)
	{
		switch(c)
		{
			case 'i':
				folder = optarg;
				break;
			case 'o':
				output = optarg;
				break;
			case 's':
				stl = optarg;
				break;
			case 'n':
				start1 = 0;
				end1 = atoi(optarg)-1;
				break;
		}
	}

	vector<PointCloud<PointT>::Ptr> scans;
	vector<PointCloud<PointNT>::Ptr> scansNT;

	cout << "Loading " + folder + "*.pcd..." << endl;
	for(int i=start1; i<=end1; i++)
	{
		PointCloud<PointT>::Ptr scan(new PointCloud<PointT>);
		io::loadPCDFile<PointT>(folder + boost::to_string(i) + ".pcd", *scan);

		PointCloud<PointNT>::Ptr scanNT(new PointCloud<PointNT>);
		io::loadPCDFile<PointNT>(folder + boost::to_string(i) + ".pcd", *scanNT);

		scans.push_back(scan);
		scansNT.push_back(scanNT);
	}

	PolygonMesh spine_mesh;
	io::loadPolygonFileSTL(stl, spine_mesh);

	PointCloud<PointT>::Ptr spine (new pcl::PointCloud<PointT>);
	fromPCLPointCloud2(spine_mesh.cloud, *spine);
	//visualization::CloudViewer viewer1("spine");
	int n = spine->points.size();

	cout << "Spine size: " << n << " vertices, " << spine_mesh.polygons.size() << " polygons" << endl;

	cout << "Constructing adjacency matrix..." << endl;	
	SparseMatrix<float> adj(n,n);

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

	cout << "Constructing Laplacian..." << endl;
	SparseMatrix<float> lap(n,n);
	VectorXf degrees(n);
	degrees.setZero();
	for(int i=0; i<adj.outerSize(); ++i)
	{
		for(SparseMatrix<float>::InnerIterator it(adj,i); it; ++it)
		{
			degrees(i)+= it.value();
		}
	}

	lap = adj;
	for(int i=0; i<lap.outerSize(); ++i)
	{
		for(SparseMatrix<float>::InnerIterator it(lap,i); it; ++it)
		{
			it.valueRef()/=degrees(i);
		}
	}
	cout << "Grabbing Eigen clouds..." << endl;
	vector<Map<MatrixXf, Aligned, OuterStride<> > > matrix_scans;
	vector<VectorXf > confidence;
	for(int i=0; i<scans.size(); i++)
	{
		matrix_scans.push_back(scans[i]->getMatrixXfMap(3,4,0));
		VectorXf t(scans[i]->points.size());
		for(int j=0; j<scans[i]->points.size(); j++)
		{
			t[j]=scansNT[i]->points[j].confidence;
		}
	}

	Map<MatrixXf, Aligned, OuterStride<> > matrix_spine = spine->getMatrixXfMap(3,4,0);

	for(int epoch=0; epoch<max_epoch; epoch++)
	{
		if(vis == 1)
		{
			//	viewer1.showCloud(scans[0], "scan 0");
		}
		cout << "Epoch " << epoch << endl;

		if(align == 1)
		{
			cout << "\tAligning point clouds..." << endl;
			for(int i=0; i < scans.size(); i++)
			{
				cout << "\t\tScan " + boost::to_string(i) + "..." << endl;
				IterativeClosestPoint<PointT, PointT> icp;
				icp.setTransformationEpsilon(1e-7);
				icp.setMaxCorrespondenceDistance(0.05);
				icp.setInputCloud(scans[i]);
				icp.setInputTarget(spine);
				icp.setMaximumIterations(maxit);
				icp.align(*(scans[i]));
				cout << "\t\tScore: " << icp.getFitnessScore() << endl;
			}
		}
		cout << "\tComputing responsibilities..." << endl;
		vector<SparseMatrix<float>* > resp;
		KdTreeFLANN<PointT> spine_kdtree;
		spine_kdtree.setInputCloud(spine);

		VectorXf ZZ(spine->points.size());
		ZZ.setZero();
		vector<int> indices(nbs);
		vector<float> sqr_distances(nbs);

		for(int i=0; i<scans.size(); i++)
		{
			cout << "\t\tScan " + boost::to_string(i) + "..." << endl;
			resp.push_back(new SparseMatrix<float>(scans[i]->points.size(),n));
			vector<Eigen::Triplet<float> > tripletListi;
			for(int j=0; j<scans[i]->points.size(); j++)
			{
				int num_found = spine_kdtree.radiusSearch((scans[i]->points)[j], rad, indices, sqr_distances, nbs);
				for(int k=0; k<num_found; k++)
				{
					tripletListi.push_back(Eigen::Triplet<float>(j,indices[k],exp(-var*sqr_distances[k])));
				}
				if(j%10000 == 0)
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
						it.valueRef() /= Z(it.row());
						ZZ(it.col()) += it.value();
					}
				}
			}
		}
		cout << "\tUpdating spine..." << endl;

		for(int i=0; i<n; i++)
		{
			ZZ(i) += narc;
		}
		for(int i=0; i<relax_it; i++)
		{
			matrix_spine = matrix_spine * lap;
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

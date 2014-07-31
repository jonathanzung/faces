#include "dep.h"
using namespace pcl;
using namespace std;
using namespace Eigen;
typedef PointXYZ PointT;
struct point
{
	float x;
	float y;
	float z;
};

//Project the point cloud 'in' to 2d as viewed from 'orientation', and save the result to 'out'
void project_to_view(PointCloud<PointT>::Ptr in, PointCloud<PointT>::Ptr out, Matrix4f* orientation)
{
	Matrix4f m = orientation->inverse();
	transformPointCloud (*in, *out, m);
	for(int i=0; i<out->points.size(); i++)
	{
		out->points[i].x /= out->points[i].z;
		out->points[i].y /= out->points[i].z;
		out->points[i].z /= out->points[i].z;
	}
}

inline float euc_distance(PointT a, PointT b)
{
	return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z));
}

inline float blur_kernel(float sqr_dist, float var)
{
	return 1;//exp(-var*sqr_dist)
}

void responsibilities(vector<Eigen::Triplet<float> >* triplet_list,
		VectorXf* row_sums,
		VectorXf* x,
		KdTreeFLANN<PointT>* scan_projected_kdtree, 
		PointCloud<PointT>::Ptr scan_projected, 
		PointCloud<PointT>::Ptr spine_projected,
		PointCloud<PointT>::Ptr scan, 
		PointCloud<PointT>::Ptr spine, 
		Matrix4f* orientation,
		map<int,point>* boundary,
		int search_nbs, float search_rad, float var,float reject_rad, float xy_reg, int offset,int offset2)
{
	Matrix4f m = orientation->inverse();
	vector<int> indices(search_nbs);//temp vector to hold indices of the nearest neighbours found
	vector<float> sqr_distances(search_nbs);//temp vector to hold the distances to the nearest neighbours

	int total_num_found=0;
	int total_sociable=0;
	float weights[3]={xy_reg,xy_reg,1.0};

	for(int j=0; j<spine_projected->points.size(); j++)//Iterate through high res points and find nearest neighbours
	{
		if(boundary->find(j) == boundary->end())
		{
			int num_found = scan_projected_kdtree->radiusSearch(spine_projected->points[j], search_rad, indices, sqr_distances, search_nbs);
			//radiusSearch returns the number of neighbours found and writes to indices and sqr_distances

			if(num_found != 0)
			{
				total_num_found += num_found;
				total_sociable+= 1;
			}

			for(int k=0; k<num_found; k++)
			{
				//if(euc_distance(scan->points[indices[k]],spine->points[j]) < reject_rad)
				{
					float resp=blur_kernel(sqr_distances[k],var);
					for(int t=0; t<3; t++)
					{
						for(int l=0; l<3; l++)
						{
							triplet_list->push_back(Eigen::Triplet<float>(
										3*(indices[k]+offset)+t, 
										3*j+l,
										m(t,l)*weights[t]*resp
										));
							/*	
								triplet_list->push_back(Eigen::Triplet<float>(
								3*(indices[k]+offset)+t, 
								offset2 + 4*t+l,
								(*x)(3*j+l)*weights[t]*resp
								));
								*/

						}
						triplet_list->push_back(Eigen::Triplet<float>(
									3*(indices[k]+offset)+t,
									3*spine->points.size(),
									m(t,3)*weights[t]*resp
									));
						/*
						   triplet_list->push_back(Eigen::Triplet<float>(
						   3*(indices[k]+offset)+t, 
						   offset2 + 4*t + 3,
						   weights[t]*resp
						   ));
						   */
						(*row_sums)(3*(indices[k]+offset)+t)+=weights[t]*resp;
					}
				}
			}
		}
	}

	cout << "\t\tAverage #neighbours, high->low: " << (float)total_num_found/total_sociable << "/" << search_nbs << endl;
}

int main (int argc, char** argv)
{
	map<string,parameter> params = loadConfig("super.config");

	string input = params["input"].s; //Input folder. Make sure it ends with a slash.
	string output = params["output"].s; //Output folder. Make sure it ends with a slash.
	string orientations_path = params["orientations_path"].s;
	string stl = params["stl"].s; //location of the mesh guess in stl format
	int search_nbs = params["search_nbs"].i; //Number of nearest neighbours to look for
	float search_rad = params["search_rad"].f; //Radius to search for nearest neighbours
	float var = params["var"].f; //The variance to be used for the blur kernel around each high res point
	int max_epoch = params["max_epoch"].i; //Maximum number of outer iterations
	int icp_maxit = params["icp_maxit"].i; //Number of allowed ICP iterations
	float icp_max_correspondence_distance = params["icp_max_correspondence_distance"].f;
	int cg_maxit = params["cg_maxit"].i;
	float k = params["k"].f;
	float peg_k = params["peg_k"].f;
	float cam_k = params["cam_k"].f;

	int relax_maxit = params["relax_maxit"].i;
	float relax_ratio = params["relax_ratio"].f;

	int align = params["align"].i;
	float reject_rad = params["reject_rad"].f;
	float xy_reg= params["xy_reg"].f;


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
				orientations_path = optarg;
				break;
			case 's':
				stl = optarg;
				break;
		}
	}

	vector<PointCloud<PointT>::Ptr> scans;
	vector<PointCloud<PointT>::Ptr> scans_temp;
	vector<PointCloud<PointT>::Ptr> scans_projected;

	cout << "Loading scans..." << endl;
	scans = loadPCDFiles(input);
	scans_temp = loadPCDFiles(input);
	scans_projected = loadPCDFiles(input);

	//Contains offsets required for mapping the scans into their concatenation
	vector<int> offsets(scans.size()+1);
	offsets[0]=0;
	for(int i=0; i<scans.size(); i++)
	{
		offsets[i+1]= scans[i]->points.size() + offsets[i];
	}

	cout << "Constructing KD trees..." << endl;
	vector<KdTreeFLANN<PointT>* > scans_projected_kdtree;//holds 2d kd-trees
	Matrix4f id=Eigen::Matrix4f::Identity();

	for(int i=0; i<scans.size(); i++)
	{
		project_to_view(scans_projected[i], scans_projected[i], &(id));
		scans_projected_kdtree.push_back(new KdTreeFLANN<PointT>);
		scans_projected_kdtree[i]->setInputCloud(scans_projected[i]);
	}

	cout << "Loading orientations" << endl;
	vector<Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > orientations = 
		eigenFromFile<Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >(orientations_path); 
	//Orientations map scans to the spine

	cout << "Reading the spine from " << stl << endl; //"spine" is the name I use for the high res point cloud/mesh
	PointCloud<PointT>::Ptr spine (new pcl::PointCloud<PointT>); //The high res point cloud
	PointCloud<PointT>::Ptr spine_projected (new pcl::PointCloud<PointT>); //The high res point cloud
	PolygonMesh spine_mesh;
	io::loadPolygonFileSTL(stl, spine_mesh);
	fromPCLPointCloud2(spine_mesh.cloud, *spine);
	Map<MatrixXf, Aligned, OuterStride<> > spine_matrix = spine->getMatrixXfMap(3,4,0); //The high res cloud as an eigen matrix
	io::savePCDFileBinary(output + "spine_initial.pcd", *(spine));
	int n = spine->points.size();
	int m = offsets[scans.size()];

	cout << "Spine size: " << n << " vertices, " << spine_mesh.polygons.size() << " polygons" << endl;

	cout << "Constructing adjacency matrix..." << endl;	
	map<pair<int, int>, int> edge_counts;
	map<pair<int, int>, bool> edge_exists;
	map<int, point> mesh_boundary;
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
				int v1=(*face)[j];
				int v2=(*face)[(j+1)%(face->size())];
				tripletList.push_back(Eigen::Triplet<float>(v1,v2,1));
				tripletList.push_back(Eigen::Triplet<float>(v2,v1,1));
				if(edge_exists.find(make_pair(v1,v2)) == edge_exists.end())
				{
					edge_exists[make_pair(v1,v2)]=true;
					edge_exists[make_pair(v2,v1)]=true;
					edge_counts[make_pair(v1,v2)]=1;
					edge_counts[make_pair(v2,v1)]=1;
				}
				else
				{
					edge_counts[make_pair(v1,v2)]+=1;
					edge_counts[make_pair(v2,v1)]+=1;
				}
			}
		}
		adj.setFromTriplets(tripletList.begin(),tripletList.end());
	}

	for(map<pair<int, int>, int>::iterator it=edge_counts.begin(); it != edge_counts.end(); it++)
	{
		if(it->second % 2 == 1)
		{
			int v1= (it->first).first;
			int v2= (it->first).second;
			point t1;
			t1.x=spine->points[v1].x;
			t1.y=spine->points[v1].y;
			t1.z=spine->points[v1].z;
			point t2;
			t2.x=spine->points[v2].x;
			t2.y=spine->points[v2].y;
			t2.z=spine->points[v2].z;

			mesh_boundary[v1]=t1;
			mesh_boundary[v2]=t2;
		}
	}

	cout << "Constructing Laplacian..." << endl;
	SparseMatrix<float> lap(3*n+1+16*scans.size(),3*n+1+16*scans.size());
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
		vector<Eigen::Triplet<float> > tripletList;
		tripletList.reserve(8*n);
		for(int i=0; i<adj.outerSize(); ++i)
		{
			for(SparseMatrix<float>::InnerIterator it(adj, i); it; ++it)
			{
				tripletList.push_back(Eigen::Triplet<float>(3*it.row(),3*it.col(),-it.value()));
				tripletList.push_back(Eigen::Triplet<float>(3*it.row()+1,3*it.col()+1,-it.value()));
				tripletList.push_back(Eigen::Triplet<float>(3*it.row()+2,3*it.col()+2,-it.value()));
			}
		}
		for(int i=0; i<n; i++)
		{
			tripletList.push_back(Eigen::Triplet<float>(3*i,3*i,degrees(i)));
			tripletList.push_back(Eigen::Triplet<float>(3*i+1,3*i+1,degrees(i)));
			tripletList.push_back(Eigen::Triplet<float>(3*i+2,3*i+2,degrees(i)));
		}
		lap.setFromTriplets(tripletList.begin(),tripletList.end());
	}
	SparseMatrix<float> boundary_projection(3*n+1+16*scans.size(),3*n+1+16*scans.size());
	{
		vector<Eigen::Triplet<float> > tripletList;
		tripletList.reserve(3*6*sqrt(n));
		for(map<int, point>::iterator it=mesh_boundary.begin(); it != mesh_boundary.end(); it++)
		{
			tripletList.push_back(Eigen::Triplet<float>(3*(it->first),3*(it->first),1));
			tripletList.push_back(Eigen::Triplet<float>(3*(it->first)+1,3*(it->first)+1,1));
			tripletList.push_back(Eigen::Triplet<float>(3*(it->first)+2,3*(it->first)+2,1));
		}
		boundary_projection.setFromTriplets(tripletList.begin(), tripletList.end());
	}

	SparseMatrix<float> cam_projection(3*n+1+16*scans.size(), 3*n+1+16*scans.size());
	{
		vector<Eigen::Triplet<float> > tripletList;
		tripletList.reserve(3*6*sqrt(n));		
		for(int i=3*n+1; i<3*n+1+16*scans.size(); i++)
		{
			tripletList.push_back(Eigen::Triplet<float>(i,i,1));
		}
		cam_projection.setFromTriplets(tripletList.begin(), tripletList.end());
	}

	for(int epoch=0; epoch<max_epoch; epoch++)
	{
		cout << "Epoch " << epoch << endl;


		if(align == 1)
		{
			cout << "\tAligning point clouds..." << endl;
			for(int i=0; i < scans.size(); i++)//Use ICP to align each low res scan to the spine
			{
				IterativeClosestPoint<PointT, PointT> icp;
				icp.setTransformationEpsilon(1e-7);
				icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance);
				icp.setInputSource(scans[i]);
				icp.setInputTarget(spine);
				icp.setMaximumIterations(icp_maxit);
				icp.align(*(scans_temp[i]),orientations[i]);
				orientations[i] = icp.getFinalTransformation();
				cout << "\t\tICP Error: " << icp.getFitnessScore() << endl;
			}
		}


		cout << "\tConstructing responsibility matrix..." << endl;
		vector<Eigen::Triplet<float> > triplet_list;//holds the triplets from which we construct A
		VectorXf row_sums(3*m);
		row_sums.setZero();

		SparseMatrix<float> A(3*m,3*n+1 + 16*scans.size());
		cout << "\t\tAverage #neighbours, low->high: " << row_sums.mean() << endl;

		cout << "\tCG..." << endl;
		VectorXf b(A.rows());
		VectorXf x(A.cols());
		x.setZero();

		for(int i=0; i<n; i++)
		{
			x(3*i)=spine->points[i].x;
			x(3*i+1)=spine->points[i].y;
			x(3*i+2)=spine->points[i].z;
		}
		x(3*spine->points.size())=1;

		for(int i=0; i<scans.size(); i++)
		{
			project_to_view(spine, spine_projected, &(orientations[i]));
			responsibilities(&triplet_list,
					&row_sums,
					&x,
					scans_projected_kdtree[i],
					scans_projected[i],
					spine_projected,
					scans[i],
					spine,
					&(orientations[i]),
					&mesh_boundary,
					search_nbs,search_rad,var,reject_rad,xy_reg,offsets[i],16*i+3*n+1);
		}
		for(int i=0; i<scans.size(); i++)
		{
			for(int k=0; k<scans[i]->points.size(); k++)
			{
				b(3*(offsets[i]+k))=scans[i]->points[k].x;
				b(3*(offsets[i]+k)+1)=scans[i]->points[k].y;
				b(3*(offsets[i]+k)+2)=scans[i]->points[k].z;
			}
		}
		for(int i=0; i<A.rows(); i++)
		{
			if(row_sums(i)==0)
			{
				b(i)=0;
			}
		}
		A.setFromTriplets(triplet_list.begin(),triplet_list.end());
		for(int i=0; i<A.outerSize(); ++i)
		{
			for(SparseMatrix<float>::InnerIterator it(A,i); it; ++it)
			{
				it.valueRef()/=row_sums(it.row());
			}
		}
		SparseMatrix<float> B = A.transpose() * A + k*lap + peg_k*boundary_projection + cam_k*cam_projection;

		BiCGSTAB<SparseMatrix<float> > solver;
		cout << "\t\t||Ax-b||: " << (A*x-b).squaredNorm() << endl;
		//cout << "\t\t||Ax||" << (A*x).squaredNorm() << endl;
		//cout << "\t\t||b||" << (b).squaredNorm() << endl;
		solver.compute(B);
		solver.setMaxIterations(cg_maxit);
		x = solver.solveWithGuess(A.transpose() * b + peg_k*boundary_projection*x,x);
		cout << "\t\t||Ax-b||: " << (A*x-b).squaredNorm() << endl;
		//cout << "\t\t||Ax||" << (A*x).squaredNorm() << endl;
		//cout << "\t\t||b||" << (b).squaredNorm() << endl;
		cout << "\t\tSolver iterations: " << solver.iterations() << endl;

		for(int i=0; i<spine->points.size(); i++)
		{
			spine->points[i].x=x(3*i);
			spine->points[i].y=x(3*i+1);
			spine->points[i].z=x(3*i+2);
		}
	}

	//savePCDFiles(scans_projected, output);

	eigenToFile<Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >(orientations, output + "orientations/"); 
	cout << "Writing to " << output << "mesh.stl" << endl;
	toPCLPointCloud2(*spine, spine_mesh.cloud);
	io::savePolygonFileSTL(output + "mesh.stl", spine_mesh);
	io::savePCDFileBinary(output + "spine.pcd", *(spine));

	return (0);
}

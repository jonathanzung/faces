#include <iostream>
#include <string>
#include <math.h>
#include <vector>

#include <map>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "util.h"

#include <flann/flann.hpp>

using namespace std;
using namespace Eigen;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfRM;

int main(int argc, char** argv)
{
	map<string,parameter> params = loadConfig("normal.config");
	string input = params["input"].s;
	string output = params["output"].s;
	int nsample = params["nsample"].i;
	int nn = params["nn"].i;
	int nchecks = params["nchecks"].i;
	float outlier_cutoff = params["outlier_cutoff"].f;

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
	vector<MatrixXfRM> clouds = eigenFromFile<MatrixXfRM>(input);

	//concatenate all the clouds
	MatrixXfRM cloud=concat(clouds);

	cout << "Total points: " << cloud.rows() << "x" << cloud.cols() << endl;
	cout << "Building index..." << endl;

	vector<MatrixXfRM> samples;
	vector<MatrixXfRM> samplenormals;

	cout << "Find neighbours..." << endl;
	for(int cl=0; cl < clouds.size(); cl++)
	{
		MatrixXfRM cloud = clouds[cl];
		flann::Matrix<float> data(cloud.data(), cloud.rows(), cloud.cols());

		flann::Index<flann::L2<float> > index(data, flann::KDTreeIndexParams(3));
		index.buildIndex();

		cout << "Query..." << endl;
		vector<int> sampleindices;
		for(int i=0; i< nsample; i++)
		{
			sampleindices.push_back((clouds[cl].rows()*i)/nsample);
		}

		MatrixXfRM sample(nsample,3);
		MatrixXfRM samplenormal(nsample,3);
		for(int i=0; i<nsample; i++)
		{
			sample.row(i)=clouds[cl].row(sampleindices[i]);
		}

		flann::Matrix<float> query(sample.data(),nsample,3);
		vector<vector<int> > indices(nsample);
		vector<vector<float> > distances(nsample);
		index.knnSearch(query,indices,distances,nn,flann::SearchParams(nchecks));

		cout << "PCA..." << endl;

		int num_dropped=0;
		for(int i=0; i< nsample; i++)
		{
			float sdistances=0;
			for(int j=0; j<distances[i].size(); j++)
			{
				sdistances+=distances[i][j];
			}
			sdistances/= nsample;

			MatrixXf nbhd(nn,3);
			for(int j=0; j<nn; j++)
			{
				nbhd.row(j)=cloud.row(indices[i][j]);
			}


			if(false && i%1000 == 0)
			{
				for(int j=0; j<nn; j++)
				{
					cout << nbhd.row(j) << endl;
				}
				cout << endl;
			}
			MatrixXf centred = nbhd.rowwise() - nbhd.colwise().mean();

			MatrixXf cov = centred.adjoint() * centred;
			SelfAdjointEigenSolver<MatrixXf> eig(cov);
			sample.row(i-num_dropped) = sample.row(i);
			samplenormal.row(i-num_dropped) = eig.eigenvectors().col(0);
			if(samplenormal(i-num_dropped,2)>0)
			{
				samplenormal.row(i-num_dropped)*=-1;
			}
		}
		cout << "Rejected: " << num_dropped << endl;
		sample.conservativeResize(nsample-num_dropped, 3);
		samplenormal.conservativeResize(nsample-num_dropped,3);
		samples.push_back(sample);
		samplenormals.push_back(samplenormal);
	}
	stringstream f;
	f << output << "sample/";
	eigenToFile<MatrixXfRM>(samples, f.str());

	stringstream f2;
	f2 << output << "samplenormal/";
	eigenToFile<MatrixXfRM>(samplenormals, f2.str());
}

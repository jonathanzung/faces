#ifndef __util_H_INCLUDED__
#define __util_H_INCLUDED__

#include <map>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <ctype.h>
#include <string>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;
typedef PointXYZ PointT;

struct parameter
{
	int i;
	float f;
	string s;
};
map<string, parameter> loadConfig(const char* filename);
void savePCDFiles(vector<PointCloud<PointT>::Ptr> clouds, string output);
vector<PointCloud<PointT>::Ptr> loadPCDFiles(string input);

template <class T>
void eigenToFile(vector<T> matrices, string filename)
{
	cout << "Writing to matrices to " <<  filename << endl;
	for(int i=0; i<matrices.size(); i++)
	{
		stringstream fidat;
		stringstream fimeta;
		fidat << filename << i << ".dat";
		fimeta << filename << i << ".meta";
		ofstream f(fidat.str().c_str());
		f.write( (char *) matrices[i].data(), matrices[i].rows() * matrices[i].cols() * sizeof(float) );
		f.close();

		ofstream f2(fimeta.str().c_str());
		f2 << matrices[i].rows() << " " << matrices[i].cols();
		f2.close();
	}
	stringstream setmeta;
	setmeta << filename << "set.meta";
	ofstream f(setmeta.str().c_str());
	f << matrices.size();
	f.close();
}

template <class T>
vector<T> eigenFromFile(string filename)
{
	cout << "Reading from " << filename << endl;

	stringstream setmeta;
	setmeta << filename << "set.meta";
	ifstream rm(setmeta.str().c_str());
	int n;
	rm >> n;
	vector<T> matrices;
	for(int i=0; i<n; i++)
	{
		stringstream fidat;
		stringstream fimeta;
		fidat << filename << i << ".dat";
		fimeta << filename << i << ".meta";

		ifstream f(fidat.str().c_str());
		ifstream f2(fimeta.str().c_str());
		int rows;
		int cols;
		f2 >> rows;
		f2 >> cols;
		T m(rows,cols);
		f.read( (char *) m.data(), m.rows() * m.cols() * sizeof(float) );
		f.close();
		f2.close();
		matrices.push_back(m);
	}
	return matrices;
}
template <class T, class S>
void eigenToFile(vector<T,S> matrices, string filename)
{
	cout << "Writing to matrices to " <<  filename << endl;
	for(int i=0; i<matrices.size(); i++)
	{
		stringstream fidat;
		stringstream fimeta;
		fidat << filename << i << ".dat";
		fimeta << filename << i << ".meta";
		ofstream f(fidat.str().c_str());
		for(int j=0; j<matrices[i].rows(); j++)
		{
			for(int k=0; k<matrices[i].cols(); k++)
			{
				f << matrices[i](j,k) << endl;
			}
		}
		f.close();

		ofstream f2(fimeta.str().c_str());
		f2 << matrices[i].rows() << " " << matrices[i].cols();
		f2.close();
	}
	stringstream setmeta;
	setmeta << filename << "set.meta";
	ofstream f(setmeta.str().c_str());
	f << matrices.size();
	f.close();
}

template <class T, class S>
vector<T,S> eigenFromFile(string filename)
{
	cout << "Reading from " << filename << endl;

	stringstream setmeta;
	setmeta << filename << "set.meta";
	ifstream rm(setmeta.str().c_str());
	int n;
	rm >> n;
	vector<T,S> matrices;
	for(int i=0; i<n; i++)
	{
		stringstream fidat;
		stringstream fimeta;
		fidat << filename << i << ".dat";
		fimeta << filename << i << ".meta";

		ifstream f(fidat.str().c_str());
		ifstream f2(fimeta.str().c_str());
		int rows;
		int cols;
		f2 >> rows;
		f2 >> cols;

		T m;
		for(int j=0; j<rows; j++)
		{
			for(int k=0; k<cols; k++)
			{
				f >> m(j,k);
			}
		}
		f.close();
		f2.close();
		matrices.push_back(m);
	}
	return matrices;
}
#endif

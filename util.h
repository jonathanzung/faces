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

using namespace std;

struct parameter
{
	int i;
	float f;
	string s;
};
map<string, parameter> loadConfig(const char* filename);

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

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> project(Eigen::MatrixXf m, float xfov, float yfov);

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> concat(vector<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >);

map<string, parameter> loadConfig(const char* filename);

#endif

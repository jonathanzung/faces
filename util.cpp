#include <iostream>
#include <map>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <cmath>
#include <math.h>

#include <boost/algorithm/string.hpp>
#include <Eigen/Core>

using namespace std;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXfRM;

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

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> project(Eigen::MatrixXf m, float xfov, float yfov)
{
	int nonzero = 0;
	int index = 0;
	float avx = 0;
	float avy = 0;
	float avz = 0;

	float xmid=m.rows()/2;
	float ymid=m.cols()/2;

	for(int i=0; i<m.rows(); i++)
	{
		for(int j=0; j<m.cols(); j++)
		{
			if(m(i,j)>0 && m(i,j) < 80)
			{
				nonzero++;
				avx+=i;
				avy+=j;
				avz+=m(i,j);
			}
		}
	}
	avx /= nonzero;
	avy /= nonzero;
	avz /= nonzero;

	nonzero = 0;

	for(int i=0; i<m.rows(); i++)
	{
		for(int j=0; j<m.cols(); j++)
		{
			if(
					m(i,j)>avz-20 &&
					m(i,j)< avz + 20 &&
					i > avx - 200 &&
					i < avx + 200 &&
					j > avy - 200 &&
					j < avy + 200
			  )
			{
				nonzero++;
			}
		}
	}
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> ret(nonzero,3);
	for(int i=0; i<m.rows(); i++)
	{
		for(int j=0; j<m.cols(); j++)
		{
			if(
					m(i,j)>avz-20 &&
					m(i,j)< avz + 20 &&
					i > avx - 200 &&
					i < avx + 200 &&
					j > avy - 200 &&
					j < avy + 200
			  )
			{
				ret(index,0)=(i-xmid)/(xmid)*tan(xfov/2);
				ret(index,1)=(j-ymid)/(ymid)*tan(yfov/2);
				ret(index,2)=1;
				//ret.row(index)/= ret.row(index).norm();
				ret.row(index)*= m(i,j);
				index++;
			}
		}
	}
	return ret;
}

MatrixXfRM concat(vector<MatrixXfRM> clouds)
{
	int total_points = 0;
	for(int i=0; i< clouds.size(); i++)
	{
		total_points+=clouds[i].rows();
	}
	int n=clouds[0].cols();
	MatrixXfRM cloud(total_points,n);
	int curr=0;
	for(int i=0; i< clouds.size(); i++)
	{
		for(int j=0; j<clouds[i].rows(); j++)
		{
			for(int k=0; k<n; k++)
			{
				cloud(curr,k)=clouds[i](j,k);
			}
			curr++;
		}
	}
	return cloud;
}

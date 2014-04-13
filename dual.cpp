#include <cmath>
#include <math.h>
#include <iostream>
#include <Eigen/Core>
using namespace std;

template <typename T, int n>
struct Dualn
{
	T val;
	T dval[n];

	Dualn(T v, T dv[])
	{
		val=v;
		for(int i=0; i<n; i++)
		{
			dval[i]=dv[i];
		}
	}
	Dualn()
	{
		val=T();
		for(int i=0; i<n; i++)
		{
			dval[i]=T();
		}
	}
	Dualn(T v)
	{
		val=v;
		for(int i=0; i<n; i++)
		{
			dval[i]=T(0);
		}
	}

	Dualn& operator +=(const Dualn& b)
	{
		val += b.val;
		for(int i=0; i<n; i++)
		{
			dval[i] += b.dval[i];
		}
		return *this;
	}

	Dualn& operator *=(double other)
	{
		val = val * other;
		for(int i=0; i<n; i++)
		{
			dval[i] *= other;
		}
		return *this;
	}
};

	template <typename T, int n>
inline Dualn<T,n> operator+(Dualn<T,n> a, Dualn<T,n> b)
{
	T dval[n];
	for(int i=0; i<n; i++)
	{
		dval[i]=a.dval[i]+b.dval[i];
	}
	return Dualn<T,n>(a.val+b.val, dval);
}

	template <typename T, int n>
inline Dualn<T,n> operator*(Dualn<T,n> a, Dualn<T,n> b)
{
	T dval[n];
	for(int i=0; i<n; i++)
	{
		dval[i]=a.val*b.dval[i]+b.val*a.dval[i];
	}
	return Dualn<T,n>(a.val*b.val, dval);
}

	template <typename T, int n>
inline Dualn<T,n> operator+(float a, Dualn<T,n> b)
{
	T dval[n];
	for(int i=0; i<n; i++)
	{
		dval[i]=b.dval[i];
	}
	return Dualn<T,n>(a+b.val,dval);
}

	template <typename T, int n>
inline Dualn<T,n> operator*(float a, Dualn<T,n> b)
{
	T dval[n];
	for(int i=0; i<n; i++)
	{
		dval[i]=a*b.dval[i];
	}
	return Dualn<T,n>(a*b.val, dval);
}

	template <typename T, int n>
inline Dualn<T,n> operator*(Dualn<T,n> b,double a)
{
	T dval[n];
	for(int i=0; i<n; i++)
	{
		dval[i]=a*b.dval[i];
	}
	return Dualn<T,n>(a*b.val, dval);
}

	template <typename T, int n>
inline Dualn<T,n> operator*(double a, Dualn<T,n> b)
{
	T dval[n];
	for(int i=0; i<n; i++)
	{
		dval[i]=a*b.dval[i];
	}
	return Dualn<T,n>(a*b.val, dval);
}

template <typename T, int n>
inline Dualn<T,n> operator-(Dualn<T,n> b)
{
	return -1.0 * b;
}


template <typename T, int n>
Dualn<T,n> pow(Dualn<T,n> a, double exponent)
{
	for(int i=0; i<n; i++)
	{
		a.dval[i]*=exponent*pow(a.val,exponent-1);
	}
	a.val = pow(a.val,exponent);
	return a;
}

template <typename T, int n, int x, int y>
inline Eigen::Matrix<Dualn<T,n>, x, y> operator*(const Eigen::Matrix<Dualn<T,n>,x,y>& a, Dualn<T,n> s)
{
	Eigen::Matrix<Dualn<T,n>,x,y> m;
	for(int i=0; i<x; i++)
	{
		for(int j=0; j<y; j++)
		{
			m(i,j)=a(i,j)*s;
		}
	}
	return m;
}

	template <typename T, int n>
inline Dualn<T,n> operator/(Dualn<T,n> a, Dualn<T,n> b)
{
	T dval[n];
	for(int i=0; i<n; i++)
	{
		dval[i]=a.dval[i]/b.val - a.val*b.dval[i]/(b.val*b.val);
	}
	return Dualn<T,n>(a.val/b.val,dval);
}
template <typename T, int n>
inline Dualn<T,n> operator/(Dualn<T,n> a, double b)
{
	T dval[n];
	for(int i=0; i<n; i++)
	{
		dval[i]=a.dval[i]/b;
	}
	return Dualn<T,n>(a.val/b,dval);
}

	template <typename T, int n>
inline Dualn<T,n> operator-(Dualn<T,n> a, Dualn<T,n> b)
{
	T dval[n];
	for(int i=0; i<n; i++)
	{
		dval[i]=a.dval[i]-b.dval[i];
	}
	return Dualn<T,n>(a.val-b.val, dval);
}
namespace Eigen{
	template<typename T,int n> struct NumTraits<Dualn<T,n> >
		: NumTraits<double> // permits to get the epsilon, dummy_precision, lowest, highest functions
		{
			enum {
				IsComplex = 0,
				IsInteger = 0,
				IsSigned = 1,
				RequireInitialization = 1,
				ReadCost = 1,
				AddCost = 3,
				MulCost = 3
			};
		};
}

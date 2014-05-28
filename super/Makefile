all: super

dep.h.gch: dep.h
	g++ -x c++-header -O -c dep.h -lstdc++ -I/usr/lib -I/usr/include/ -I/usr/lib -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8

util.o: util.cpp
	g++ -c util.cpp

super.o: super.cpp
	g++ -O -c super.cpp -I/usr/include/vtk-5.8 -I/usr/include/eigen3
	
super: util.o dep.h.gch super.o
	g++ -w -O super.o util.o -lpcl_common -lpcl_io -lm -lstdc++ -lboost_system -lpcl_search -lpcl_kdtree -lpcl_surface -lpcl_filters -lpcl_registration -o super

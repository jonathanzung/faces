CC=g++ -I /dupa-filer/jzung/include -I /dupa-filer/jzung/include/vtk-6.1 -I /dupa-filer/jzung/include/eigen3 -L /dupa-filer/jzung/lib 

all: super gen

dep.h.pch: dep.h util.h
	$(CC) -O -x c++-header dep.h -o dep.h.pch -I/usr/lib -I/usr/include/ -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8 

util.o: util.cpp
	$(CC) -O -c util.cpp

super.o: super.cpp
	$(CC) -O -c -include dep.h super.cpp 
	#-I/usr/include/vtk-5.8 
	#-I/usr/include/eigen3

gen.o: gen.cpp
	$(CC) -O -c gen.cpp -I/usr/include/vtk-5.8 -I/usr/include/eigen3

super: util.o super.o
	$(CC) -w -O super.o util.o -lpcl_common -lpcl_io -lm -lstdc++ -lboost_system -lpcl_search -lpcl_kdtree -lpcl_surface -lpcl_filters -lpcl_registration -o super
	touch ../test/super.config

gen: util.o gen.o
	$(CC) -w -O gen.o util.o -lpcl_common -lpcl_io -lm -lstdc++ -lboost_system -lpcl_search -lpcl_kdtree -lpcl_surface -lpcl_filters -lpcl_registration -o gen
	touch ../test/gen.config

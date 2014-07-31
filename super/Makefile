all: super gen

dep.h.pch: dep.h util.h
	clang -O -x c++-header dep.h -o dep.h.pch -I/usr/lib -I/usr/include/ -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8 

util.o: util.cpp
	clang -O -c util.cpp

super.o: super.cpp dep.h.pch
	clang -O -c -include dep.h super.cpp -I/usr/include/vtk-5.8 -I/usr/include/eigen3

gen.o: gen.cpp
	clang -O -c gen.cpp -I/usr/include/vtk-5.8 -I/usr/include/eigen3

super: util.o super.o
	clang -w -O super.o util.o -lpcl_common -lpcl_io -lm -lstdc++ -lboost_system -lpcl_search -lpcl_kdtree -lpcl_surface -lpcl_filters -lpcl_registration -o super
	touch ../test/super.config

gen: util.o gen.o
	clang -w -O gen.o util.o -lpcl_common -lpcl_io -lm -lstdc++ -lboost_system -lpcl_search -lpcl_kdtree -lpcl_surface -lpcl_filters -lpcl_registration -o gen
	touch ../test/gen.config

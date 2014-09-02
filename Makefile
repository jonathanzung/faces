CC=g++ -march=native -msse3 -mfpmath=sse -isystem /dupa-filer/jzung/include -isystem /dupa-filer/jzung/include/eigen3 -I /dupa-filer/jzung/include/vtk-6.1 -L /dupa-filer/jzung/lib 
all: normal topcd trace torowm stitch straighten graft inspect compare

capture: capture.o util.o
	$(CC) -O capture.o util.o -L/home/jonathan/Documents/CS/faces/.lib/OpenNI/Redist -lOpenNI2 -lstdc++ -lm -o capture
	touch test/capture.config

normal: normal.o util.o
	$(CC) -O normal.o util.o -lstdc++ -lm -lflann -o normal 
	touch test/normal.config

torowm: torowm.o util.o
	$(CC) -O torowm.o util.o -lstdc++ -lm -o torowm
	touch test/torowm.config

straighten: straighten.o util.o
	$(CC) -O straighten.o util.o -lstdc++ -lm -o straighten
	touch test/straighten.config

trace: trace.o util.o
	$(CC) -O trace.o util.o -lstdc++ -lm -o trace
	touch test/trace.config

util.o: util.cpp
	$(CC) -c util.cpp

normal.o: normal.cpp util.h
	$(CC) -c normal.cpp

trace.o: trace.cpp util.h
	$(CC) -c trace.cpp

torowm.o: torowm.cpp util.h
	$(CC) -c torowm.cpp

topcd.o: topcd.cpp util.h
	$(CC) -c topcd.cpp

straighten.o: straighten.cpp util.h
	$(CC) -c straighten.cpp

capture.o: capture.cpp util.h
	$(CC) -std=gnu++11 -c capture.cpp -I/home/jonathan/Documents/CS/faces/.lib/OpenNI/Include

topcd: topcd.o util.o
	$(CC) -O topcd.o util.o -I/usr/include/pcl-1.7 -lpcl_common -lpcl_io -lstdc++ -lm -lboost_system -I/usr/include/eigen3 -o topcd

stitch: stitch.cpp util.h
	$(CC) -O stitch.cpp util.o  -lstdc++ -lm -lboost_system -lpcl_common -lpcl_io -lpcl_registration -lpcl_search -lpcl_visualization -lpcl_filters -lpcl_features -o stitch

graft: graft.cpp
	$(CC) -O graft.cpp -I/usr/include/pcl-1.7 -lpcl_common -lpcl_io -I/usr/lib -lstdc++ -lboost_system -lpcl_registration -lpcl_search -lpcl_kdtree -lpcl_filters -lpcl_surface -lpcl_features -I/usr/include/vtk-5.8 -I/usr/include/eigen3 -o graft

inspect.o: inspect.cpp
	$(CC) -O -c inspect.cpp -lstdc++ -I/usr/lib -I/usr/include/ -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8

inspect: inspect.o
	$(CC) -w -O inspect.o -lpcl_common -lpcl_io -lm -lstdc++ -lboost_system -lpcl_search -lpcl_kdtree -lpcl_surface -lpcl_filters -lpcl_registration -o inspect

compare.o: compare.cpp
	$(CC) -O -c compare.cpp -lstdc++ -I/usr/lib -I/usr/include/ -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/include/vtk-5.8

compare: compare.o
	$(CC) -w -O compare.o -lpcl_common -lpcl_io -lm -lstdc++ -lboost_system -lpcl_search -lpcl_kdtree -lpcl_surface -lpcl_filters -lpcl_registration -o compare

segment: segment.cpp
	$(CC) -O segment.cpp -I/usr/lib -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -lstdc++ -lm -lboost_system -lpcl_common -lpcl_io -lpcl_search -lpcl_visualization -lpcl_filters -lpcl_features -o segment

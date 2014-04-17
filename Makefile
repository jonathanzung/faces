all: capture normal topcd trace torowm stitch straighten graft

capture: capture.o util.o
	clang -O capture.o util.o -L/home/jonathan/faces/.lib/OpenNI/Redist -lOpenNI2 -lstdc++ -lm -o capture
	touch test/capture.config

normal: normal.o util.o
	clang -O normal.o util.o -lstdc++ -lm -o normal 
	touch test/normal.config

torowm: torowm.o util.o
	clang -O torowm.o util.o -lstdc++ -lm -o torowm
	touch test/torowm.config

straighten: straighten.o util.o
	clang -O straighten.o util.o -lstdc++ -lm -o straighten
	touch test/straighten.config

trace: trace.o util.o
	clang -O trace.o util.o -lstdc++ -lm -o trace
	touch test/trace.config

util.o: util.cpp
	clang -c util.cpp

normal.o: normal.cpp util.h
	clang -c normal.cpp

trace.o: trace.cpp util.h
	clang -c trace.cpp

torowm.o: torowm.cpp util.h
	clang -c torowm.cpp

topcd.o: topcd.cpp util.h
	clang -c topcd.cpp

straighten.o: straighten.cpp util.h
	clang -c straighten.cpp

capture.o: capture.cpp util.h
	clang -std=gnu++11 -c capture.cpp -I/home/jonathan/faces/.lib/OpenNI/Include

topcd: topcd.o util.o
	clang -O topcd.o util.o -I/usr/include/pcl-1.7 -lpcl_common -lpcl_io -lstdc++ -lm -lboost_system -I/usr/include/eigen3 -o topcd

stitch: stitch.cpp util.h
	clang -O stitch.cpp util.o  -I/usr/lib -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -lstdc++ -lm -lboost_system -lpcl_common -lpcl_io -lpcl_registration -lpcl_search -lpcl_visualization -lpcl_filters -lpcl_features -I/usr/include/vtk-5.8 -lvtkCommon -o stitch

graft: graft.cpp
	clang -O graft.cpp -I/usr/include/pcl-1.7 -lpcl_common -lpcl_io -I/usr/lib -lstdc++ -lboost_system -lpcl_registration -lpcl_search -lpcl_kdtree -lpcl_filters -lpcl_surface -lpcl_features -I/usr/include/vtk-5.8 -I/usr/include/eigen3 -o graft

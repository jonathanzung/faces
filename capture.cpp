//#ifdef __clang__
//typedef struct { long double x, y; } __float128;
//#endif

#include "OpenNI.h"
#include <PS1080.h>
#include <iostream>
#include <string>
#include <math.h>

#include <map>
#include <fstream>
#include <stdio.h>
#include <ctype.h>
#include <unistd.h>

#include <Eigen/Core>
#include "util.h"
#include <thread>

using namespace openni;
using namespace std;

bool a = true;
typedef Eigen::Matrix<short unsigned int, Eigen::Dynamic, Eigen::Dynamic> MatrixXsi;

void wait()
{
	cin.ignore();
	a=false;
}

int main(int argc, char** argv)
{
	map<string,parameter> params = loadConfig("capture.config");

	string output = params["output"].s;
	int delay = params["delay"].i;
	int num_frames = params["num_frames"].i;
	int mode_id = params["mode_id"].i;

	int c;
	while((c = getopt (argc, argv, "o:n:")) !=-1)
	{
		switch(c)
		{
			case 'o':
				output = optarg;
				break;
			case 'n':
				num_frames = atoi(optarg);
				break;
		}
	}

	vector<void*> data;
	OpenNI::initialize();
	Device* cam = new Device();
	Array<DeviceInfo>* deviceInfoList = new Array<DeviceInfo>;
	OpenNI::enumerateDevices(deviceInfoList);
	cam->open(deviceInfoList->operator[](0).getUri());
	VideoStream* vid = new VideoStream();
	vid->create(*cam, SENSOR_DEPTH);

	VideoMode mode;
	mode = vid->getSensorInfo().getSupportedVideoModes()[mode_id];
	int width = mode.getResolutionX();
	int height = mode.getResolutionY();
	cam->setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	cam->setDepthColorSyncEnabled(true);

	cout << "Registration Mode: " << cam->getImageRegistrationMode() << endl;
	cout << "Depth Pixel Format: " << mode.getPixelFormat() << endl;
	cout << "Resolution: " << mode.getResolutionX() << "x" << mode.getResolutionY() << "x" << mode.getFps() << endl;
	cout << "Field of View: " << vid->getHorizontalFieldOfView() << "x" << vid->getVerticalFieldOfView() << endl;

	vid->setVideoMode(mode);

	float xfov = vid->getHorizontalFieldOfView();
	float yfov = vid->getVerticalFieldOfView();
	vid->start();
	vid->setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, true);

	VideoFrameRef* frame = new VideoFrameRef();
	vid->readFrame(frame);
	int datasize = frame->getDataSize();
	cout << "Depth Datasize: " << datasize << endl;
	frame->release();
	cin.ignore();

	thread t1(wait);
	time_t start;
	time_t end;

	time(&start);
	for(int i = 0; i<num_frames && a; i++)
	{
		data.push_back(new short unsigned int* [datasize]);
		cout << "*" << flush;
		if(i%10 == 9)
		{
			cout << endl;
		}
		vid->readFrame(frame);
		memcpy(data[i], frame->getData(), datasize);
		frame->release();
		usleep(delay*1000);
	}
	time(&end);
	t1.join();
	vid->stop();
	OpenNI::shutdown();
	cout << difftime(end, start) << endl;

	cout << "Framerate: " << data.size()/(double(difftime(end,start))) << endl;

	vector<Eigen::MatrixXf> dataXf;
	for(int i=0; i<data.size(); i++)
	{
		Eigen::Map<MatrixXsi> m((short unsigned int*)(data[i]), width, height);
		Eigen::MatrixXf n(width, height);	
		for(int j=0; j<width; j++)
		{
			for(int k=0; k<height; k++)
			{
				n(j,k)=((float)m(j,k))/100;
			}
		}
		dataXf.push_back(n);
	}


	eigenToFile<Eigen::MatrixXf>(dataXf, output);
}

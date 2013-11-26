#include "OpenNI.h"
#include <PS1080.h>
#include <iostream>
#include <string>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/filters/filter.h>

using namespace openni;
using namespace std;
using namespace pcl;

PointCloud<PointXYZ> toPCD(const void* a, int width, int height, float xfov, float yfov)
{
		RangeImagePlanar range_im;
		PointCloud<PointXYZ> cloud;
		range_im.setDepthImage(((const short unsigned int*) a), width, height, width/2, height/2, (float)(width/2)/tan(xfov/2), (float)(height/2)/tan(yfov/2));
		cloud.width = height*width;
		cloud.height = 1;
		cloud.points.resize(height*width);
		cloud.is_dense=false;
		int pointcount = 0;
		for(int i = 0; i < width; i++)
		{
				for(int j = 0; j < height; j++)
				{
						if(true)
						{
								cloud.points[pointcount].x = range_im.at(i,j).x;
								cloud.points[pointcount].y = range_im.at(i,j).y;
								cloud.points[pointcount].z = range_im.at(i,j).z;
								pointcount++;
						}
				}
		}
		cloud.points.resize(pointcount);
		vector<int> indices;
		removeNaNFromPointCloud(cloud,cloud,indices);
		cloud.width = cloud.points.size();

		return cloud; 
}

int main(int argc, char** argv)
{
		int mode_id = 5;
		string folder = argv[1];
		int num_frames = atoi(argv[2]);
		vector<void*> data;
		OpenNI::initialize();
		Device* cam = new Device();
		cam->open(ANY_DEVICE);
		VideoStream* vid = new VideoStream();
		vid->create(*cam, SENSOR_DEPTH);
		VideoMode mode;
		mode = vid->getSensorInfo().getSupportedVideoModes()[mode_id];
		int width = mode.getResolutionX();
		int height = mode.getResolutionY();
		cout << "Pixel Format: " << mode.getPixelFormat() << endl;
		cout << "Resolution: " << mode.getResolutionX() << "x" << mode.getResolutionY() << endl;

		vid->setVideoMode(mode);

		cout << "Field of View: " << vid->getHorizontalFieldOfView() << "x" << vid->getVerticalFieldOfView() << endl;
		float xfov = vid->getHorizontalFieldOfView();
		float yfov = vid->getVerticalFieldOfView();
		vid->start();
		vid->setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, true);

		VideoFrameRef* frame = new VideoFrameRef();
		vid->readFrame(frame);
		int datasize = frame->getDataSize();
		frame->release();

		for(int i = 0; i < num_frames; i++)
		{
				data.push_back(new void* [datasize]);
		}
		cin.ignore();
		for(int i = 0; i < num_frames; i++)
		{
				cout << i << endl;
				vid->setEmitterEnabled(true);
				vid->readFrame(frame);
				vid->setEmitterEnabled(false);
				memcpy(data[i], frame->getData(), datasize);
				frame->release();
		}
		vid->stop();
		cam->close();
		for(int i = 0; i < num_frames; i++)
		{
				PointCloud<PointXYZ> cloud = toPCD(data[i],width,height,xfov,yfov);
				stringstream filename;
				filename << folder << i << ".pcd";
				io::savePCDFileBinary(filename.str(), cloud);
				cout << "saved cloud of " << cloud.points.size() << " to " << filename.str()  << endl;
		}
}
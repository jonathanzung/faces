#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/registration/icp_nl.h>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <boost/algorithm/string.hpp>
#include <map>
#include <iostream>
#include <fstream>
#include <string>
#include <utility>

#include "util.h"

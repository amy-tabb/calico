

#ifndef INCLUDES_HPP_
#define INCLUDES_HPP_

#include <iostream>

#include <getopt.h>
#include <string>
#include <fstream>
#include <sstream>
#include <map>
#include <iomanip>
#include <list>
#include <vector>
#include <stdlib.h>
#include <algorithm>
#include <inttypes.h>
#include <parallel/algorithm>
#include <algorithm>
#include <iterator>
#include <vector>
#include <set>
#include <cmath>
#include <cstdint>
#include <omp.h>
#include <chrono>
#include <random>
#include <sys/stat.h>
#include "glog/logging.h"
#include "ceres/ceres.h"

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <apriltags/TagDetector.h>
#include <apriltags/Tag16h5.h>
#include <apriltags/Tag25h7.h>
#include <apriltags/Tag25h9.h>
#include <apriltags/Tag36h9.h>
#include <apriltags/Tag36h11.h>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
#include "apriltag-file.hpp"


using namespace Eigen;
using namespace cv;
using std::vector;
using std::ifstream;
using std::ofstream;
using std::string;


using std::set;
using std::max;
using std::min;
using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::list;
using std::vector;
using std::map;
using std::pair;

#include <sys/stat.h>
#include <sys/time.h>
#include <chrono>

template<class T>
T FromString(const string& s)
{
	std::istringstream stream (s);
	T t;
	stream >> t;
	return t;
}

template<class T>
string ToString(T arg)
{
	std::ostringstream s;

	s << arg;

	return s.str();

}


inline void MultiplyMatrixVector(const vector< vector<double> >& M, int rows, int cols,
        const vector<double>& v, vector<double>& X){

	int c;
	// don't test to save time
	for (int r = 0; r < rows; r++){
		X[r] = 0;

		for (c = 0; c < cols; c++){
			X[r] += M[r][c]*v[c];

		}
	}


}


void PrintMatrix(vector< vector<double> >& p);



#endif /* INCLUDES_HPP_ */

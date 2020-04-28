/*
 * local_charuco.cpp
 *
 *  Created on: Apr 9, 2020
 *      Author: atabb
 *
 *     This file alters some OpenCV functions to fix a bug, and also to add some functionality.
 */



/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

 * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
 */

#include "Includes.hpp"
#include "local_charuco.hpp"

namespace cv
{
namespace aruco
{
namespace charucoLocal {

/**
 * Remove charuco corners if any of their minMarkers closest markers has not been detected
 */
static int _filterCornersWithoutMinMarkers(const Ptr<CharucoBoard> &_board,
		InputArray _allCharucoCorners,
		InputArray _allCharucoIds,
		InputArray _allArucoIds, int minMarkers,
		OutputArray _filteredCharucoCorners,
		OutputArray _filteredCharucoIds) {

	CV_Assert(minMarkers >= 0 && minMarkers <= 2);

	vector< Point2f > filteredCharucoCorners;
	vector< int > filteredCharucoIds;
	// for each charuco corner
	for(unsigned int i = 0; i < _allCharucoIds.getMat().total(); i++) {
		int currentCharucoId = _allCharucoIds.getMat().at< int >(i);
		int totalMarkers = 0; // number of closest marker detected
		// look for closest markers
		for(unsigned int m = 0; m < _board->nearestMarkerIdx[currentCharucoId].size(); m++) {
			int markerId = _board->ids[_board->nearestMarkerIdx[currentCharucoId][m]];
			bool found = false;
			for(unsigned int k = 0; k < _allArucoIds.getMat().total(); k++) {
				if(_allArucoIds.getMat().at< int >(k) == markerId) {
					found = true;
					break;
				}
			}
			if(found) totalMarkers++;
		}
		// if enough markers detected, add the charuco corner to the final list
		if(totalMarkers >= minMarkers) {
			filteredCharucoIds.push_back(currentCharucoId);
			filteredCharucoCorners.push_back(_allCharucoCorners.getMat().at< Point2f >(i));
		}
	}

	// parse output
	Mat(filteredCharucoCorners).copyTo(_filteredCharucoCorners);
	Mat(filteredCharucoIds).copyTo(_filteredCharucoIds);
	return (int)_filteredCharucoIds.total();
}

/**
 * @brief From all projected chessboard corners, select those inside the image and apply subpixel
 * refinement. Returns number of valid corners.
 */
static int _selectAndRefineChessboardCorners(InputArray _allCorners, InputArray _image,
		OutputArray _selectedCorners,
		OutputArray _selectedIds,
		const vector< Size > &winSizes) {

	const int minDistToBorder = 2; // minimum distance of the corner to the image border
	// remaining corners, ids and window refinement sizes after removing corners outside the image
	vector< Point2f > filteredChessboardImgPoints;
	vector< Size > filteredWinSizes;
	vector< int > filteredIds;

	// filter corners outside the image
	Rect innerRect(minDistToBorder, minDistToBorder, _image.getMat().cols - 2 * minDistToBorder,
			_image.getMat().rows - 2 * minDistToBorder);
	for(unsigned int i = 0; i < _allCorners.getMat().total(); i++) {
		if(innerRect.contains(_allCorners.getMat().at< Point2f >(i))) {
			filteredChessboardImgPoints.push_back(_allCorners.getMat().at< Point2f >(i));
			filteredIds.push_back(i);
			filteredWinSizes.push_back(winSizes[i]);
		}
	}

	// if none valid, return 0
	if(filteredChessboardImgPoints.size() == 0) return 0;

	// corner refinement, first convert input image to grey
	Mat grey;
	if(_image.type() == CV_8UC3)
		cvtColor(_image, grey, COLOR_BGR2GRAY);
	else
		_image.copyTo(grey);

	const Ptr<DetectorParameters> params = DetectorParameters::create(); // use default params for corner refinement

	// For each of the charuco corners, apply subpixel refinement using its corresponding winSize
	parallel_for_(Range(0, (int)filteredChessboardImgPoints.size()), [&](const Range& range) {
		const int begin = range.start;
		const int end = range.end;

		for (int i = begin; i < end; i++) {
			vector<Point2f> in;
			in.push_back(filteredChessboardImgPoints[i]);
			Size winSize = filteredWinSizes[i];
			if (winSize.height == -1 || winSize.width == -1)
				winSize = Size(params->cornerRefinementWinSize, params->cornerRefinementWinSize);

			cornerSubPix(grey, in, winSize, Size(),
					TermCriteria(TermCriteria::MAX_ITER | TermCriteria::EPS,
							params->cornerRefinementMaxIterations,
							params->cornerRefinementMinAccuracy));

			filteredChessboardImgPoints[i] = in[0];
		}
	});

	// parse output
	Mat(filteredChessboardImgPoints).copyTo(_selectedCorners);
	Mat(filteredIds).copyTo(_selectedIds);
	return (int)filteredChessboardImgPoints.size();
}


/**
 * Calculate the maximum window sizes for corner refinement for each charuco corner based on the
 * distance to their closest markers
 */
static void _getMaximumSubPixWindowSizes(InputArrayOfArrays markerCorners, InputArray markerIds,
		InputArray charucoCorners, const Ptr<CharucoBoard> &board,
		vector< Size > &sizes) {

	unsigned int nCharucoCorners = (unsigned int)charucoCorners.getMat().total();
	sizes.resize(nCharucoCorners, Size(-1, -1));

	for(unsigned int i = 0; i < nCharucoCorners; i++) {
		if(charucoCorners.getMat().at< Point2f >(i) == Point2f(-1, -1)) continue;
		if(board->nearestMarkerIdx[i].size() == 0) continue;

		double minDist = -1;
		int counter = 0;

		// calculate the distance to each of the closest corner of each closest marker
		for(unsigned int j = 0; j < board->nearestMarkerIdx[i].size(); j++) {
			// find marker
			int markerId = board->ids[board->nearestMarkerIdx[i][j]];
			int markerIdx = -1;
			for(unsigned int k = 0; k < markerIds.getMat().total(); k++) {
				if(markerIds.getMat().at< int >(k) == markerId) {
					markerIdx = k;
					break;
				}
			}
			if(markerIdx == -1) continue;
			Point2f markerCorner =
					markerCorners.getMat(markerIdx).at< Point2f >(board->nearestMarkerCorners[i][j]);
			Point2f charucoCorner = charucoCorners.getMat().at< Point2f >(i);
			double dist = norm(markerCorner - charucoCorner);
			if(minDist == -1) minDist = dist; // if first distance, just assign it
			minDist = min(dist, minDist);
			counter++;
		}

		// if this is the first closest marker, dont do anything
		if(counter == 0)
			continue;
		else {
			// else, calculate the maximum window size
			int winSizeInt = int(minDist - 2); // remove 2 pixels for safety
			if(winSizeInt < 1) winSizeInt = 1; // minimum size is 1
			if(winSizeInt > 10) winSizeInt = 10; // maximum size is 10
			sizes[i] = Size(winSizeInt, winSizeInt);
		}
	}
}

/**
 * Interpolate charuco corners using local homography
 */
static int _interpolateCornersCharucoLocalHom(InputArrayOfArrays _markerCorners,
		InputArray _markerIds, InputArray _image,
		const Ptr<CharucoBoard> &_board,
		OutputArray _charucoCorners,
		OutputArray _charucoIds, double tolerance) {

	CV_Assert(_image.getMat().channels() == 1 || _image.getMat().channels() == 3);
	CV_Assert(_markerCorners.total() == _markerIds.getMat().total() &&
			_markerIds.getMat().total() > 0);

	unsigned int nMarkers = (unsigned int)_markerIds.getMat().total();

	// calculate local homographies for each marker
	vector< Mat > transformations;
	transformations.resize(nMarkers);


	vector<bool> validTransform(nMarkers, true);

	double det = 0;

	for(unsigned int i = 0; i < nMarkers; i++) {
		vector< Point2f > markerObjPoints2D;
		int markerId = _markerIds.getMat().at< int >(i);
		vector< int >::const_iterator it = find(_board->ids.begin(), _board->ids.end(), markerId);
		if(it == _board->ids.end()) continue;
		int boardIdx = (int)std::distance<std::vector<int>::const_iterator>(_board->ids.begin(), it);
		markerObjPoints2D.resize(4);
		for(unsigned int j = 0; j < 4; j++)
			markerObjPoints2D[j] =
					Point2f(_board->objPoints[boardIdx][j].x, _board->objPoints[boardIdx][j].y);

		transformations[i] = getPerspectiveTransform(markerObjPoints2D, _markerCorners.getMat(i));
		det = determinant(transformations[i]);

		validTransform[i] = fabs(det) > tolerance;

	}

	unsigned int nCharucoCorners = (unsigned int)_board->chessboardCorners.size();
	vector< Point2f > allChessboardImgPoints(nCharucoCorners, Point2f(-1, -1));

	// for each charuco corner, calculate its interpolation position based on the closest marker's
	// homographies
	for(unsigned int i = 0; i < nCharucoCorners; i++) {
		Point2f objPoint2D = Point2f(_board->chessboardCorners[i].x, _board->chessboardCorners[i].y);

		vector< Point2f > interpolatedPositions;
		for(unsigned int j = 0; j < _board->nearestMarkerIdx[i].size(); j++) {
			int markerId = _board->ids[_board->nearestMarkerIdx[i][j]];
			int markerIdx = -1;
			for(unsigned int k = 0; k < _markerIds.getMat().total(); k++) {
				if(_markerIds.getMat().at< int >(k) == markerId) {
					markerIdx = k;
					break;
				}
			}
			if(markerIdx != -1 && validTransform[markerIdx] == true ) {
					vector< Point2f > in, out;
					in.push_back(objPoint2D);
					perspectiveTransform(in, out, transformations[markerIdx]);
					interpolatedPositions.push_back(out[0]);
			}
		}

		// none of the closest markers detected
		if(interpolatedPositions.size() == 0) continue;

		// more than one closest marker detected, take middle point
		if(interpolatedPositions.size() > 1) {
			allChessboardImgPoints[i] = (interpolatedPositions[0] + interpolatedPositions[1]) / 2.;
		}
		// a single closest marker detected
		else allChessboardImgPoints[i] = interpolatedPositions[0];
	}

	// calculate maximum window sizes for subpixel refinement. The size is limited by the distance
	// to the closes marker corner to avoid erroneous displacements to marker corners
	vector< Size > subPixWinSizes;
	_getMaximumSubPixWindowSizes(_markerCorners, _markerIds, allChessboardImgPoints, _board,
			subPixWinSizes);


	// filter corners outside the image and subpixel-refine charuco corners
	return _selectAndRefineChessboardCorners(allChessboardImgPoints, _image, _charucoCorners,
			_charucoIds, subPixWinSizes);
}


/**
 */
int interpolateCornersCharucoHomographyLocal(InputArrayOfArrays _markerCorners, InputArray _markerIds,
		InputArray _image, const Ptr<CharucoBoard> &_board,
		OutputArray _charucoCorners, OutputArray _charucoIds, double tolerance,
		int minMarkers) {

	// else use local homography
	_interpolateCornersCharucoLocalHom(_markerCorners, _markerIds, _image, _board,
			_charucoCorners, _charucoIds, tolerance);

	// to return a charuco corner, its closest aruco markers should have been detected
	return _filterCornersWithoutMinMarkers(_board, _charucoCorners, _charucoIds, _markerIds,
			minMarkers, _charucoCorners, _charucoIds);
}


/**
   @param board layout of ChArUco board.
 * @param image charucoIds list of identifiers for each corner in charucoCorners.
 * @return bool value, 1 (true) for detected corners form a line, 0 for non-linear.
      solvePnP will fail if the corners are collinear (true).
  * Check that the set of charuco markers in _charucoIds does not identify a straight line on
    the charuco board.  Axis parallel, as well as diagonal and other straight lines detected.
  */
bool testCharucoCornersCollinear(const Ptr<CharucoBoard> &_board, InputArray _charucoIds){

    unsigned int nCharucoCorners = (unsigned int)_charucoIds.getMat().total();

    // only test if there are 3 or more corners
    if (nCharucoCorners > 2){

    CV_Assert( _board->chessboardCorners.size() >= _charucoIds.getMat().total());

    Vec<double, 3> point0( _board->chessboardCorners[_charucoIds.getMat().at< int >(0)].x,
                           _board->chessboardCorners[_charucoIds.getMat().at< int >(0)].y,
                           1);

    Vec<double, 3> point1( _board->chessboardCorners[_charucoIds.getMat().at< int >(1)].x,
                           _board->chessboardCorners[_charucoIds.getMat().at< int >(1)].y,
                           1);

    // create a line from the first two points.
    Vec<double, 3> testLine = point0.cross(point1);

    Vec<double, 3> testPoint(0, 0, 1);

    double divisor = sqrt(testLine[0]*testLine[0] + testLine[1]*testLine[1]);

    CV_Assert( divisor != 0);

     // normalize the line with normal
     testLine /= divisor;

     double dotProduct;
     for (unsigned int i = 2; i < nCharucoCorners; i++){
         testPoint(0) = _board->chessboardCorners[_charucoIds.getMat().at< int >(i)].x;
         testPoint(1) = _board->chessboardCorners[_charucoIds.getMat().at< int >(i)].y;

         // if testPoint is on testLine, dotProduct will be zero (or very, very close)
         dotProduct = testPoint.dot(testLine);

         if (std::abs(dotProduct) > 1e-6){
             return false;
         }
     }

    // no points found that were off of testLine, return true that all points collinear.
    return true;
    }

    return true;
}

/**
 * @brief test whether the ChArUco markers are collinear
 *
 * @param _board layout of ChArUco board.
 * @param _charucoIds list of identifiers for each corner in charucoCorners per frame.
 * @return bool value, 1 (true) if detected corners form a line, 0 (false) if they do not.
      solvePnP, calibration functions will fail if the corners are collinear (true).
 *
 * The number of ids in charucoIDs should be <= the number of chessboard corners in the board.  This functions checks whether the charuco corners are on a straight line (returns true, if so), or not (false).  Axis parallel, as well as diagonal and other straight lines detected.  Degenerate cases: for number of charucoIDs <= 2, the function returns true.
 */
CV_EXPORTS_W bool testCharucoCornersCollinear(const Ptr<CharucoBoard> &_board,
                                              InputArray _charucoIds);


}
}
}

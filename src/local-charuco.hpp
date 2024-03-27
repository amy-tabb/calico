/*
 * local_charuco.hpp
 *
 *  Created on: Apr 9, 2020
 *      Author: atabb
 */

#ifndef LOCAL_CHARUCO_HPP_
#define LOCAL_CHARUCO_HPP_


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

namespace cv
{
namespace aruco
{
namespace charucoLocal
{



///**
// * @brief Interpolate position of ChArUco board corners
// * @param markerCorners vector of already detected markers corners. For each marker, its four
// * corners are provided, (e.g std::vector<std::vector<cv::Point2f> > ). For N detected markers, the
// * dimensions of this array should be Nx4. The order of the corners should be clockwise.
// * @param markerIds list of identifiers for each marker in corners
// * @param image input image necesary for corner refinement. Note that markers are not detected and
// * should be sent in corners and ids parameters.
// * @param board layout of ChArUco board.
// * @param charucoCorners interpolated chessboard corners
// * @param charucoIds interpolated chessboard corners identifiers
// * @param cameraMatrix optional 3x3 floating-point camera matrix
// * \f$A = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\f$
// * @param distCoeffs optional vector of distortion coefficients
// * \f$(k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6],[s_1, s_2, s_3, s_4]])\f$ of 4, 5, 8 or 12 elements
// * @param minMarkers number of adjacent markers that must be detected to return a charuco corner
// *
// * This function receives the detected markers and returns the 2D position of the chessboard corners
// * from a ChArUco board using the detected Aruco markers. If camera parameters are provided,
// * the process is based in an approximated pose estimation, else it is based on local homography.
// * Only visible corners are returned. For each corner, its corresponding identifier is
// * also returned in charucoIds.
// * The function returns the number of interpolated corners.
// */


CV_EXPORTS_W int interpolateCornersCharucoHomographyLocal(InputArrayOfArrays markerCorners, InputArray markerIds,
                                           InputArray image, const Ptr<CharucoBoard> &board,
                                           OutputArray charucoCorners, OutputArray charucoIds,
										   double tolerance = 1e-6, int minMarkers = 2);

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



#endif


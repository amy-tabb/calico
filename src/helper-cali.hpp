/*
 * helper-cali.hpp
 *
 *  Created on: Nov 4, 2022
 *      Author: atabb
 */

#ifndef HELPER_CALI_HPP_
#define HELPER_CALI_HPP_

bool readDetectorParameters(const string& filename, Ptr<aruco::DetectorParameters> &params);

int ConvertAprilMarkerIdIndexToGridPointIndex(int squaresX, int markerID, int index_in_marker, int starting_marker_id_group);

#endif /* HELPER_CALI_HPP_ */

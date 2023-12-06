/*
 * patterns.hpp
 *
 *  Created on: Nov 4, 2022
 *      Author: atabb
 */

#ifndef PATTERNS_HPP_
#define PATTERNS_HPP_

#include "Includes.hpp"
#include "pattern-parameters.hpp"

class PatternsCreated{
public:
    vector<vector<int> > pattern_id_marker_indexes_to_vector; // old double_to_single
    vector< cv::Point3f> three_d_points;
    vector< cv::Point3f> three_d_points_internal; // for strawberry case
    Ptr<aruco::Dictionary> dictionary;
    vector<cv::Ptr<cv::aruco::CharucoBoard> > boards; /// for refining the estimate of corner locations
    Ptr<aruco::DetectorParameters> detectorParams;
    vector< vector<int> > display_colors;

    vector<int> pattern_start_marker_indexes;

    bool rotate_case;
    vector<int> single_aruco_ids;
    int max_internal_patterns;
    int internalx, internaly;
    patternParameters pp;
    vector<Mat> april_images;
    bool is_charuco;
    bool is_april;


    PatternsCreated(const string& read_dir, const string& write_dir, const string& src_file,
            bool rotate, const bool is_charuco_local, const bool generate_only);

    void ConstructAprilTagVersion(const string& read_dir, const string& write_dir,
            const string& src_file, bool generate_only);

    void ConstructCharucoVersionNoRotate(const string& read_dir, const string& write_dir,
                const string& src_file, bool generate_only);

    void DetermineBoardsPresentFromMarkerList(const vector<int>& markers, vector<bool>& boards_seen);

    void DetermineBoardsPresentFromMarkerList(const vector<AprilTags::TagDetection>& markers, vector<bool>& boards_seen);

    Scalar Color(int index); //used

    int NumberPatterns() const;
    int NumberMarkersPerPattern() const; //used
    int NumberCornersPerPattern() const;


protected:
    int number_total_squares;
    int number_total_markers;
    int number_markers_per_pattern;
    int number_corners_per_pattern; // this will be different for april versus charuco.

};



#endif /* PATTERNS_HPP_ */

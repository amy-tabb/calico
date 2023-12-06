/*
 * apriltag-file.hpp
 *
 *  Created on: Sep 1, 2020
 *      Author: atabb
 */

#ifndef APRILTAG_FILE_HPP_
#define APRILTAG_FILE_HPP_

#include "Includes.hpp"

class AprilTagObject {

public:
    AprilTags::TagDetector* m_tagDetector;
    AprilTags::TagCodes m_tagCodes;

    bool m_draw; // draw image and April tag detections?
    bool m_timing; // print timing information for each tag extraction call

    int m_width; // image size in pixels
    int m_height;
    double m_tagSize; // April tag side length in meters of square black frame
    double m_fx; // camera focal length in pixels
    double m_fy;
    double m_px; // camera principal point
    double m_py;

    int m_deviceId; // camera id (in case of multiple cameras)

    // U Mich apriltag lib, alternate representation.
    apriltag_family_t *m_apriltag_lib_tag_family;

    vector<string> m_tag_strings;
    vector<AprilTags::TagCodes>  m_vectorTagCodes;
    // default constructor
    AprilTagObject();

    // changing the tag family
    void setTagCodes(string s);

    void setup() ;

};

cv::Mat apriltag_to_image_local_black_border(apriltag_family_t *fam, int idx);

#endif /* APRILTAG_FILE_HPP_ */

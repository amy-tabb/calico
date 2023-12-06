/*
 * apriltag-file.cpp
 *
 *  Created on: Sep 1, 2020
 *      Author: atabb
 */


#include "apriltag-file.hpp"
#include "helper.hpp"


AprilTagObject::AprilTagObject() :
// default settings, most can be modified through command line options (see below)
m_tagDetector(NULL),
m_tagCodes(AprilTags::tagCodes36h11),

m_draw(true),
m_timing(false),

m_width(640),
m_height(480),
m_tagSize(0.166),
m_fx(600),
m_fy(600),
m_px(m_width/2),
m_py(m_height/2),

m_deviceId(0),
m_apriltag_lib_tag_family(0)
{

    m_tag_strings =  vector<string>{"tag16h5", "tag25h7", "tag25h9", "tag36h9", "tag36h11" };
    m_vectorTagCodes = vector<AprilTags::TagCodes>{AprilTags::tagCodes16h5, AprilTags::tagCodes25h7, AprilTags::tagCodes25h9,
        AprilTags::tagCodes36h9, AprilTags::tagCodes36h11};

}

// changing the tag family
void AprilTagObject::setTagCodes(string s) {


    int tag_index = -1;
    uint number_types = uint(m_tag_strings.size());

    for (uint i = 0; i < number_types; i++){
        if (s.compare(m_tag_strings[i]) == 0){
            tag_index = i;
            m_tagCodes = m_vectorTagCodes[tag_index];
            i = number_types; // exit the loop
        }
    }

    cout << "Tag family " << s << endl;

    if (tag_index == -1){
        cout << "Invalid tag family specified" << endl;
        exit(1);
    }

    // the last member.

    // Initialize tag detector with options


    vector<string> tag_string{"tag36h11", "tag25h9", "tag16h5" };
    vector<std::function<apriltag_family_t *()> > tag_create_functions{tag36h11_create, tag25h9_create, tag16h5_create};

    vector<std::function<void(apriltag_family_t *)> > tag_destroy_functions{tag36h11_destroy, tag25h9_destroy, tag16h5_destroy};

    tag_index = -1;
    number_types = uint(tag_string.size());

    for (uint i = 0; i < number_types; i++){
        if (s.compare(tag_string[i]) == 0){
            tag_index = i;
            m_apriltag_lib_tag_family = tag_create_functions[tag_index]();
            i = number_types; // exit the loop
        }
    }

    if (tag_index == -1){
        cout << "not able to generate this tag family at the moment." << endl;
    }

}

void AprilTagObject::setup() {

    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

}


Mat apriltag_to_image_local_black_border(apriltag_family_t *fam, int idx)
{

    uint64_t APRILTAG_U64_ONE = 1;

    assert(fam != NULL);
    assert(idx >= 0 && idx < int(fam->ncodes));

    uint64_t code = fam->codes[idx];

    int width = fam->total_width;

    Mat marker_mat = Mat::zeros(width, width, CV_8UC1);

    // make the border.

    int border_start = (fam->total_width - fam->width_at_border)/2;

    for (uint i = 0; i < fam->nbits; i++) {
        if (code & (APRILTAG_U64_ONE << (fam->nbits - i - 1))) {
            marker_mat.data[(fam->bit_y[i] + border_start)*width + fam->bit_x[i] + border_start] = 255;
        }
    }

    return marker_mat;
}



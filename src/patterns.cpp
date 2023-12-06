/*
 * patterns.cpp
 *
 *  Created on: Nov 4, 2022
 *      Author: atabb
 */

#include "patterns.hpp"
#include "helper-cali.hpp"
#include "helper.hpp"

// used 11/27
Scalar PatternsCreated::Color(int index){

    if (index >= int(display_colors.size()) ){
        index = display_colors.size() % index;
    }
    return Scalar(display_colors[index][0], display_colors[index][1],display_colors[index][2]);
}

// used 11/27
void PatternsCreated::ConstructAprilTagVersion(const string& read_dir, const string& write_dir, const string& src_file, bool generate_only){

    // this version is for april tag version, have a check here just in case.
    assert(is_april == true);

    // cannot have rotate case with april tags.
    assert(is_april && !rotate_case);

    int count = 0;

    // read from yaml., no generate.
    string spec_file_path = read_dir + "network_specification_file.yaml";
    bool valid = readAprilTagSpecificationFile(spec_file_path, pp);

    if (pp.numberBoards > 1){
        cout << "Note: at this time, the code is only implemented for 1 AprilTag board at a time for comparison to Kalibr." << endl;
        cout << "your network_specification_file.yaml has " << pp.numberBoards << endl;
        cout << "Quitting." << endl;
        exit(1);
    }

    string filename_write = write_dir + "network_specification_file.yaml";

    CopyFile(spec_file_path, filename_write);

    if (!valid){
        cout << "Parameter read did not work. " << spec_file_path << endl;
        exit(1);
    }

    Size imageSize;
    int margins = pp.margins;

    imageSize.width = pp.squaresX * (pp.squareLength + pp.tagSpace) - pp.tagSpace + 2*margins;
    imageSize.height = pp.squaresY * (pp.squareLength + pp.tagSpace) - pp.tagSpace + 2*margins;

    number_corners_per_pattern = pp.squaresX*pp.squaresY*4;
    number_markers_per_pattern = pp.squaresX*pp.squaresY;

    // charuco's squarelength -> this one's marker length.
    int start_x_coordinate = 0;
    int start_y_coordinate = 0;


    start_x_coordinate = margins;
    start_y_coordinate = imageSize.height - (pp.squareLength) - margins;

    ofstream out;

    string command;

    pp.ATObject.setTagCodes(pp.april_family);
    pp.ATObject.setup();

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    vector<string> tag_string{"tag36h11", "tag25h9", "tag16h5" };
    vector<std::function<apriltag_family_t *()> > tag_create_functions{tag36h11_create, tag25h9_create, tag16h5_create};

    vector<std::function<void(apriltag_family_t *)> > tag_destroy_functions{tag36h11_destroy, tag25h9_destroy, tag16h5_destroy};

    int tag_index = -1;
    int number_types = int(tag_string.size());

    for (int i = 0; i < number_types; i++){
        if (pp.april_family.compare(tag_string[i]) == 0){
            tag_index = i;
            tf = tag_create_functions[tag_index]();
            i = number_types; // exit the loop
        }
    }

    if (tag_index == -1){
        cout << "not able to generate this tag family at the moment." << endl;
    }

    int total_corners = 0;

    vector<int> start_marker_counts;

    detectorParams = aruco::DetectorParameters::create();

    bool readOk = readDetectorParameters(src_file, detectorParams);
    if(!readOk) {
        cout << "Invalid detector parameters file, quitting " << src_file << endl;
        cout << __LINE__ << " in " << __FILE__ << endl;
        exit(1);
    }

    max_internal_patterns = 0;
    internalx = 0;  internaly = 0;

    string returnString;

    Mat boardCopy;
    Mat markerImg;
    Mat markedImage;
    Mat imageCopy;

    int x0, y0;

    string filename;

    for (int i = 0, s = 0; i < pp.numberBoards; i++){

        april_images.push_back(Mat::zeros(imageSize.height, imageSize.width, CV_8UC1));
        april_images[i].setTo(255);

        pattern_start_marker_indexes.push_back(s);
        for (int y = 0; y < pp.squaresY; y++){
            for (int x = 0; x < pp.squaresX; x++){

                Mat aprils = apriltag_to_image_local_black_border(tf, count);

                // michael kaess' lib needs a black border for detection.

                string filename0 = "raw.png";
                string filename1 = "larger.png";

                imwrite(filename0.c_str(), aprils);

                string command = "convert " + filename0 + " -sample "
                        + ToString<int>(pp.squareLength)+"x"+ ToString<int>(pp.squareLength)+ " " + filename1;
                int conv = system(command.c_str());

                markerImg = imread(filename1.c_str(), IMREAD_GRAYSCALE);

                command  = "convert " + filename1 + " -border 10x10 " + filename1;

                conv =system(command.c_str());

                x0 = start_x_coordinate + x*(pp.squareLength + pp.tagSpace);
                y0 = start_y_coordinate - y*(pp.squareLength + pp.tagSpace);

                Rect R = Rect(x0, y0, pp.squareLength, pp.squareLength);

                markerImg.copyTo(april_images[i](R));

                count++;

                total_corners += 4;

            }
        }


        filename = write_dir + "pattern" + ToString<int>(i) + ".png";
        imwrite(filename.c_str(), april_images[i]);

        markedImage = april_images[i].clone();
        cvtColor(markedImage, imageCopy, CV_GRAY2BGR);

        cout << "Before detections " << i << endl;


        vector<AprilTags::TagDetection> detections = pp.ATObject.m_tagDetector->extractTags(april_images[i]);

        // print out each detection
        cout << detections.size() << " tags detected:" << endl;


        /////////////// Fields below are filled in by TagDetector ///////////////
        //! Position (in fractional pixel coordinates) of the detection.
        /*  The points travel counter-clockwise around the target, always
         *  starting from the same corner of the tag.
         *  Starts at lower left corner.
         */

        for (uint j=0, dn = detections.size(); j < dn; j++) {
            // also highlight in the image
            detections[j].draw(imageCopy);

            for (uint k = 0; k < 4; k++){

                Point2f p(detections[j].p[k].first, detections[j].p[k].second);

                int grid_index =  ConvertAprilMarkerIdIndexToGridPointIndex(pp.squaresX, detections[j].id,
                        k, pattern_start_marker_indexes[i]);
                circle(imageCopy, p, 2, Scalar(255, 0, 0), 1);
                putText(imageCopy, ToString<int>(grid_index), p, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 255),1);


            }
        }

        filename = write_dir + "BoardWithAprilLabels" + ToString<int>(i) + ".png";
        cv::imwrite(filename, imageCopy);

    }

    tag_destroy_functions[tag_index](tf);
    tf = 0;


    number_total_markers = pp.squaresX * pp.squaresY * pp.numberBoards;

    // so for april versus aruco these mean different things.
    number_total_squares = number_total_markers;


    for (int i = 0; i < pp.numberBoards; i++){

        if (!generate_only){
            filename = read_dir + "pattern_square_mm" + ToString<int>(i) + ".txt";

            returnString = FindValueOfFieldInFile(filename, "squareLength_mm", false, true);
            pp.squareLength_mm = FromString<float>(returnString);

            double margin_ratio = double(pp.tagSpace)/double(pp.squareLength);
            pp.tagSpace_mm = pp.squareLength_mm*margin_ratio;

            filename_write = write_dir + "pattern_square_mm" + ToString<int>(i) + ".txt";

            CopyFile(filename, filename_write);

            three_d_points = vector< cv::Point3f >(pp.numberBoards*number_corners_per_pattern, cv::Point3f());
            int sm = 0;
            for (int i = 0, sc = 0; i < pp.numberBoards; i++){

                // we create map from pattern, # of marker relative to the pattern, to index of the marker in the full vector.
                // this may not be necessary.
                vector<int> current_index(pp.squaresX*pp.squaresY, 0);

                for (int m = 0; m < pp.squaresX*pp.squaresY; m++, sc++){
                    current_index[m] = sc;
                }

                pattern_id_marker_indexes_to_vector.push_back(current_index);


                for (int r = 0; r < pp.squaresY; r++){
                    for (int c = 0; c < pp.squaresX; c++){

                        //ccw from bottom left.
                        // index 0
                        Point3f p((pp.squareLength_mm+pp.tagSpace_mm)*float(c), float(r)*(pp.squareLength_mm+pp.tagSpace_mm), 0);
                        three_d_points[sm] = p;
                        sm++;

                        // index 1, right edhe of the marker
                        p = Point3f((pp.squareLength_mm+pp.tagSpace_mm)*float(c) + pp.squareLength, float(r)*(pp.squareLength_mm+pp.tagSpace_mm), 0);
                        three_d_points[sm] = p;
                        sm++;
                    }

                    for (int c = 0; c < pp.squaresX; c++){

                        // next row.
                        // index 0
                        Point3f p((pp.squareLength_mm+pp.tagSpace_mm)*float(c), float(r)*(pp.squareLength_mm+pp.tagSpace_mm) + pp.squareLength_mm, 0);
                        three_d_points[sm] = p;
                        sm++;

                        // index 1, right edge of the marker
                        p = Point3f((pp.squareLength_mm+pp.tagSpace_mm)*float(c) + pp.squareLength, float(r)*(pp.squareLength_mm+pp.tagSpace_mm) + + pp.squareLength_mm, 0);
                        three_d_points[sm] = p;
                        sm++;

                    }
                }
            }


        }   else {
            // create this template file to fill in.
            filename_write = write_dir + "pattern_square_mm" + ToString<int>(i) + ".txt";

            out.open(filename_write.c_str());

            out << "squareLength_mm  XX" << endl;

            out.close();
        }
    }

}

// used 11/27
void PatternsCreated::ConstructCharucoVersionNoRotate(const string& read_dir, const string& write_dir, const string& src_file, bool generate_only){

    // this version is for the charuco tag version, have a check here just in case.
    assert(is_charuco == true);

    // cannot have rotate case with april tags.
    assert(is_charuco && !rotate_case);


    // read from yaml., no generate.
    string spec_file_path = read_dir + "network_specification_file.yaml";
    bool valid = readCharucoSpecificationFile(spec_file_path, pp);

    string filename_write = write_dir + "network_specification_file.yaml";

    CopyFile(spec_file_path, filename_write);

    if (!valid){
        cout << "ReadCharucoSpecificationsFile did not work. " << spec_file_path << endl;
        exit(1);
    }

    // set everything up
    dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(pp.arc_code));

    Size imageSize;

    imageSize.width = pp.squaresX * (pp.squareLength) + 2*pp.margins;
    imageSize.height = pp.squaresY * (pp.squareLength) + 2*pp.margins;

    number_corners_per_pattern = (pp.squaresX - 1)*(pp.squaresY - 1);


    detectorParams = aruco::DetectorParameters::create();

    bool readOk = readDetectorParameters(src_file, detectorParams);
    if(!readOk) {
        cout << "Invalid detector parameters file, quitting " << src_file << endl;
        cout << __LINE__ << " in " << __FILE__ << endl;
        exit(1);
    }

    string returnString;

    Mat boardCopy;

    string filename;
    ofstream out;

    for (int i = 0, m = 0; i < pp.numberBoards; i++){
        /// the dimensions of the board are linked in here ....
        boards.push_back(cv::aruco::CharucoBoard::create(pp.squaresX, pp.squaresY, pp.squareLength, pp.markerLength, dictionary));

        pattern_start_marker_indexes.push_back(m);

        number_markers_per_pattern = boards[i]->ids.size();

        for (int j = 0; j < number_markers_per_pattern; j++, m++){
            boards[i]->ids[j] = m;
        }

        Mat boardImage(imageSize.height, imageSize.width, CV_8UC1, 255);

        boards[i]->draw( imageSize, boardImage, pp.margins, 1 );
        filename = write_dir + "Board" + ToString<int>(i) + ".png";
        imwrite(filename.c_str(), boardImage);

        // write one with the markers.
        cvtColor(boardImage, boardCopy, cv::COLOR_GRAY2RGB);

        vector< vector< Point2f > > corners, rejected;
        vector< int > ids;
        // detect markers and estimate pose
        aruco::detectMarkers(boardImage, dictionary, corners, ids, detectorParams, rejected);

        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;

        cv::aruco::interpolateCornersCharuco(corners, ids, boardImage, boards[i], charucoCorners, charucoIds);

        aruco::drawDetectedMarkers(boardCopy, corners, ids, Scalar(255, 255, 0));

        cv::aruco::drawDetectedCornersCharuco(boardCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 255));

        filename = write_dir + "BoardWithArucoLabels" + ToString<int>(i) + ".png";
        cv::imwrite(filename, boardCopy);
    }
    number_total_markers = number_markers_per_pattern*pp.numberBoards;


    //pendatic, but want to avoid re-computing it and making a mistake later.
    number_total_squares = pp.squaresX*pp.squaresY*pp.numberBoards;

    // then open up all of the physical measurement files, add to square length.
    for (int i = 0; i < pp.numberBoards; i++){
        if (!generate_only){
            filename = read_dir + "pattern_square_mm" + ToString<int>(i) + ".txt";

            returnString = FindValueOfFieldInFile(filename, "squareLength_mm", false, true);
            pp.squareLength_mm = FromString<float>(returnString);


            filename_write = write_dir + "pattern_square_mm" + ToString<int>(i) + ".txt";

            CopyFile(filename, filename_write);
        } else {

            filename_write = write_dir + "pattern_square_mm" + ToString<int>(i) + ".txt";

            out.open(filename_write.c_str());

            out << "squareLength_mm  XX" << endl;

            out.close();
        }
    }

    // convert everything to the class members.
    //////////////////////// double to single.//////////////////////////////
    if (!generate_only){

        three_d_points = vector< cv::Point3f >(number_corners_per_pattern*pp.numberBoards, cv::Point3f());

        int sm = 0;
        ofstream out;
        string filen = "test.txt";
        out.open(filen.c_str());

        for (int i = 0, sc = 0; i < pp.numberBoards; i++){

            // we create map from pattern, # of marker relative to the pattern, to index of the marker in the full vector.
            // this may not be necessary.
            vector<int> current_index(number_markers_per_pattern, 0);

            for (int m = 0; m < number_markers_per_pattern; m++, sc++){
                current_index[m] = sc;
            }

            pattern_id_marker_indexes_to_vector.push_back(current_index);

            // check that this gets created.
            out << "BOARD NUMBER " << i << endl;
            for (int r = 0; r < pp.squaresY - 1; r++){
                for (int c = 0; c < pp.squaresX - 1; c++, sm++){
                    Point3f p(pp.squareLength_mm*float(c), float(r)*pp.squareLength_mm, 0);
                    three_d_points[sm] = p;
                    out << three_d_points[sm].x << ", "<< three_d_points[sm].y << "," << three_d_points[sm].z << endl;
                }
            }
            out << "INDEX : " << sm << endl;


        }
        out.close();
    }

}

// used 11/27
void PatternsCreated::DetermineBoardsPresentFromMarkerList(const vector<int>& markers,
        vector<bool>& boards_seen){

    //int np = NumberPatterns();
    int board_index =0;
    int markers_per_board = NumberMarkersPerPattern();

    for (int i = 0, in = markers.size(); i < in; i++){
        board_index = markers[i]/markers_per_board;
        boards_seen[board_index] = true;
    }

}

// used 11/27
void PatternsCreated::DetermineBoardsPresentFromMarkerList(const vector<AprilTags::TagDetection>& markers,
        vector<bool>& boards_seen){

    int board_index =0;
    int markers_per_board = NumberMarkersPerPattern();

    for (int i = 0, in = markers.size(); i < in; i++){
        board_index = markers[i].id/markers_per_board;
        boards_seen[board_index] = true;
    }

}

// used 11/27
PatternsCreated::PatternsCreated(const string& read_dir,
        const string& write_dir,
        const string& src_file,
        bool rotate,
        const bool is_charuco_local,
        const bool generate_only):
        rotate_case(rotate),
        max_internal_patterns(0),
        internalx(0), internaly(0),
        is_charuco(is_charuco_local), is_april(!is_charuco_local),
        number_total_squares(0), number_total_markers(0),
        number_markers_per_pattern(0), number_corners_per_pattern(0) {


    cout << "Is charuco?"  << is_charuco << endl;
    cout << "Is april?"  << is_april << endl;

    if (is_april){
        ConstructAprilTagVersion(read_dir, write_dir, src_file, generate_only);
    }   else {
        if (!rotate_case){

            ConstructCharucoVersionNoRotate(read_dir, write_dir, src_file, generate_only);
        }   else {
            cout << "This version of the code does not deal with the rotation case. " << endl;
            cout << __FILE__ << ", line number " << __LINE__ << endl;
            exit(1);

        }
    }

    // GET 8 COLORS
    vector<int> color0(3, 20);
    vector<int> color1(3, 20);
    vector<int> color2(3, 20);
    vector<int> color3(3, 20);
    vector<int> color4(3, 20);
    vector<int> color5(3, 20);
    vector<int> color6(3, 20);
    vector<int> color7(3, 20);

    // r
    color0[2] = 255;

    // g
    color1[1] = 255;

    // r
    color2[0] = 255;

    // mix 0
    color3[0] = 255;
    color3[1] = 255;

    // mix1
    color4[1] = 255;
    color4[2] = 255;

    // mix2
    color5[0] = 255;
    color5[2] = 255;

    // dark g
    color6[0] = 0;
    color6[1] = 180;
    color6[2] = 0;


    //dark r
    color7[0] = 0;
    color7[1] = 0;
    color7[2] = 180;


    display_colors.push_back(color0);
    display_colors.push_back(color1);
    display_colors.push_back(color2);
    display_colors.push_back(color3);
    display_colors.push_back(color4);
    display_colors.push_back(color5);
    display_colors.push_back(color6);
    display_colors.push_back(color7);
}

// used 11/27
int PatternsCreated::NumberMarkersPerPattern() const{

    return number_markers_per_pattern;
}

// used 11/27
int PatternsCreated::NumberCornersPerPattern() const{

    return number_corners_per_pattern;
}

// used 11.27
int PatternsCreated::NumberPatterns() const{

    return pp.numberBoards;
}

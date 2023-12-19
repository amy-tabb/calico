/*
 * camera_calibration.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: atabb
 */

#include "camera-calibration.hpp"
#include "DirectoryFunctions.hpp"
#include "helper.hpp"
#include "local-charuco.hpp"
#include "helper-cali.hpp"


CameraCali::CameraCali(const string& read_dir, PatternsCreated* P, int max_ext_images, int max_int_images_read,
        int max_int_images_use){

    pixel_width = 0;
    count_internal_ids_present = 0;
    start_time_this_camera = 0;


    P_class = P; // have a ptr to all the full information about the patterns.

    // load the images ...
    string filename;
    string txt_ext = "txt";
    Mat im;

    string ext_dir = read_dir + "/external/";
    cout << "Ext dir! " << ext_dir << endl;

    // read the external images first, then read the internal ones.
    bool dir_exists = IsDirectory(ext_dir);
    cout << "Does exist? " << dir_exists << endl;

    vector<string> new_names;


    if (dir_exists){
        ReadDirectory(ext_dir, im_names);
    }	else {
        ext_dir = read_dir;
        ReadDirectory(read_dir, im_names);
        cout << "Number of images " << im_names.size() << endl;
    }

    if (max_ext_images < 0){
        max_ext_images = im_names.size();
    }

    int(im_names.size()) < max_ext_images ? number_external_images_max = im_names.size() : number_external_images_max = max_ext_images;

    for (int i = 0; i < number_external_images_max; i++){
        filename = ext_dir + "/" + im_names[i];

        if (filename.size() > 3 && filename.substr(filename.size() - 3, filename.size()) != txt_ext){
            new_names.push_back(filename);
        }
    }

    if (new_names.size() > 0){
        cout << "before read " << endl;
        im = imread(new_names[0].c_str(), IMREAD_COLOR);
        rows = im.rows;
        cols = im.cols;
        images.resize(new_names.size());

#pragma omp parallel for
        for (int i = 0; i < number_external_images_max; i++){
            //cout << "reading ... " << new_names[i] << endl;
            images[i] = imread(new_names[i].c_str(), IMREAD_COLOR);

        }

        new_names.swap(im_names);
    }

    /// read the internal dir.
    string int_dir = read_dir + "/internal/";
    dir_exists = IsDirectory(int_dir);

    vector<string> aux_names;
    vector<string> new_names2;

    cout << "int_dir : " << int_dir << endl;
    cout << "Does exist? " << dir_exists << endl;

    if (dir_exists){
        max_internals_use = 0;

        ReadDirectory(int_dir, aux_names);

        if (max_int_images_use == -1){
            max_int_images_use = aux_names.size();
        }

        if (max_int_images_read == -1){
            max_int_images_read = aux_names.size();
        }

        int number_images_to_read = aux_names.size();

        if (max_int_images_read < number_images_to_read){
            number_images_to_read = max_int_images_read;
        }

        for (int i = 0; i < number_images_to_read; i++){
            filename = int_dir + aux_names[i];

            if (filename.size() > 3 && filename.substr(filename.size() - 3, filename.size()) != txt_ext){
                new_names2.push_back(int_dir + aux_names[i]);
            }
        }

        if (new_names2.size() > 0){
            im = imread(new_names2[0].c_str(), IMREAD_COLOR);
            rows = im.rows;
            cols = im.cols;
            images.resize(images.size() + new_names2.size());
        }

#pragma omp parallel for
        for (int i = 0; i < number_images_to_read; i++){

            images[i + number_external_images_max] = imread(new_names2[i].c_str(), IMREAD_COLOR);

        }

    }	else {
        max_internals_use = 0;
    }

    cout << "Number max images " << number_external_images_max << endl;
    cout << "Number of internals to use " << max_internals_use << endl;

}

CameraCali::~CameraCali(){

    images.clear();

}

// used 11/24
void CameraCali::FindCornersCharuco(const string& write_dir, bool write_internal_images ){

    assert(P_class->is_charuco && P_class->rotate_case == false);


    int number_images = images.size();
    int number_patterns = P_class->NumberPatterns();
    string filename;

    Mat imageCopy;
    Scalar b_color;

    int number_corners_per_pattern = P_class->NumberCornersPerPattern();

    MatrixXd twod(number_corners_per_pattern*number_patterns, 2);
    twod.setConstant(0);
    int global_index;

    boards_detected.resize(number_images, vector<bool>(P_class->NumberPatterns(), false));

    std::ofstream out;
    filename = write_dir + "points.txt";

    cout << "File opened " << filename << endl;

    out.open(filename.c_str());

    two_d_point_coordinates_dense.resize(number_images, twod);
    points_present.resize(number_images, vector<bool>(number_corners_per_pattern*number_patterns, false));


    int internal_found_counter = 0;

#pragma omp parallel for private(b_color, global_index)
    for (int i = 0; i < number_images; i++){

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector< Vec3d > rvecs, tvecs;

        vector< vector< vector< Point2f > > > corners_sorted;
        vector< vector<int> > markers_sorted;

        bool collinear_markers = false;

        if ((max_internals_use == 0) || (internal_found_counter < max_internals_use) )
        {
            // detect markers and estimate pose
            aruco::detectMarkers(images[i], P_class->dictionary, corners, ids, P_class->detectorParams, rejected);

            // draw results -- dealing with the possibility of more than one board per image.
            if(ids.size() > 0) {
                //we id the board by the aruco tag.
                P_class->DetermineBoardsPresentFromMarkerList(ids, boards_detected.at(i));

                // per board refinement and localization of the corners.
                for (int p = 0; p < number_patterns; p++){
                    if (boards_detected.at(i).at(p) == true){
                        cout << "board detected " << i << ", " << p << endl;

                        b_color = P_class->Color(p);
                        // want this to be fresh each iter.
                        std::vector<cv::Point2f> charucoCorners;
                        std::vector<int> charucoIds;


                        cv::aruco::charucoLocal::interpolateCornersCharucoHomographyLocal(corners, ids, images.at(i),
                                P_class->boards.at(p), charucoCorners, charucoIds);


                        collinear_markers = cv::aruco::charucoLocal::testCharucoCornersCollinear(P_class->boards.at(p),
                                charucoIds);

                        assert(charucoCorners.size() >= charucoIds.size());

                        if (charucoIds.size() > 0 && collinear_markers){
                            for (int j = 0, jn = charucoIds.size(); j < jn; j++){
                                circle(images.at(i), charucoCorners.at(j), 10, b_color, 2);
                            }

                            boards_detected.at(i).at(p) = false;

                        }

                        if (charucoIds.size() == 0){
                            boards_detected.at(i).at(p) = false;
                        }


                        // if at least one charuco corner detected
                        if((charucoIds.size() > 0) && (collinear_markers == false))
                        {
                            if (i > number_external_images_max){
#pragma omp critical
                                {
                                    internal_found_counter++;
                                }
                            }

                            /// corners (2d) are linked to the Ids
                            cv::aruco::drawDetectedCornersCharuco(images.at(i), charucoCorners, charucoIds, b_color);

                            // the corner closest to a particular marker is added.
                            for (int j  = 0, jn = charucoIds.size(); j < jn; j++){

                                global_index = number_corners_per_pattern*p + charucoIds.at(j);

                                points_present.at(i).at(global_index) = true;

                                assert(two_d_point_coordinates_dense.at(i).rows() >= global_index);

                                two_d_point_coordinates_dense.at(i)(global_index, 0) = charucoCorners[j].x;
                                two_d_point_coordinates_dense[i](global_index, 1) = charucoCorners[j].y;

                            }
                        }
                    }
                }
            }
        }


    }


#pragma omp parallel for private(filename)
    for (int i = 0; i < number_images; i++){
        if (i < number_external_images_max || write_internal_images){
            filename = write_dir + "initial_detect" + ToString<int>(i) + ".png";

            imwrite(filename.c_str(), images[i]);

        }
    }



    for (int i = 0; i < number_images; i++){
        out << "image " << i << endl;

        for (int j = 0; j < number_corners_per_pattern*number_patterns; j++){
            if (points_present[i][j] == true){
                out << j << " " << std::setprecision(9) << two_d_point_coordinates_dense[i](j, 0)
                                                                                                                                                                                                                                                                                  << " " << std::setprecision(9) << two_d_point_coordinates_dense[i](j, 1) << " ";
            }
        }

        out << endl << "end " << i << endl;
    }


    // fill in everything computed in this function.
    out << "Image-board truth table " << endl;
    for (int i = 0; i < number_images; i++){
        out << "Image " << i << " : ";
        for (int p = 0; p < number_patterns; p++){
            out <<  PointsAtImagePattern(i, p) << " " ;
        }
        out << endl;
    }

    out.close();
}

// used 11/24
void CameraCali::FindCornersApril(const string& write_dir, bool write_internal_images ){

    assert(P_class->is_april && P_class->rotate_case == false);

    int number_images = images.size();
    int number_corners_per = P_class->NumberCornersPerPattern();
    int number_patterns = P_class->NumberPatterns();
    int pattern_index;
    string filename;

    Mat imageCopy;
    Scalar b_color;


    int number_markers_per_pattern = P_class->NumberMarkersPerPattern();
    int number_corners_per_pattern = P_class->NumberCornersPerPattern();


    MatrixXd twod(number_corners_per_pattern*number_patterns, 2);
    twod.setConstant(0);
    int global_index;

    boards_detected.resize(number_images, vector<bool>(P_class->NumberPatterns(), false));

    std::ofstream out;
    filename = write_dir + "points.txt";


    out.open(filename.c_str());

    two_d_point_coordinates_dense.resize(number_images, twod);
    points_present.resize(number_images, vector<bool>(number_corners_per_pattern*number_patterns, false));


    int internal_found_counter = 0;


    Mat grayCopy;

    cout << "Number of images " << images.size() << endl;


    for (int i = 0; i < number_images; i++){



        if ((max_internals_use == 0) || (internal_found_counter < max_internals_use) )
        {
            // convert
            cv::cvtColor(images[i], grayCopy, cv::COLOR_BGR2GRAY);

            // detect markers and estimate pose
            vector<AprilTags::TagDetection> detections = P_class->pp.ATObject.m_tagDetector->extractTags(grayCopy);

            // print out each detection
            cout << detections.size() << " ID tags detected for image " << i << endl;

            // draw results -- dealing with the possibility of more than one board per image.
            if(detections.size() > 0) {


                //we id the board by the aruco tag.
                P_class->DetermineBoardsPresentFromMarkerList(detections, boards_detected[i]);

                // per board refinement and localization of the corners.
                for (int p = 0; p < number_patterns; p++){

                    if (boards_detected[i][p] == true){

                        //cout << "board detected " << i << ", " << p << endl;

                        // housekeeping.
                        if (i > number_external_images_max){
                            internal_found_counter++;
                        }
                    }
                }

                for (uint j=0, dn = detections.size(); j < dn; j++) {
                    detections[j].draw(images[i]);

                    pattern_index = detections[j].id/number_markers_per_pattern;
                    b_color = P_class->Color(pattern_index);

                    for (uint k = 0; k < 4; k++){

                        Point2f p(detections[j].p[k].first, detections[j].p[k].second);

                        int grid_index =  ConvertAprilMarkerIdIndexToGridPointIndex(P_class->pp.squaresX, detections[j].id,
                                k, P_class->pattern_start_marker_indexes[pattern_index]);
                        circle(images[i], p, 3, b_color, 2);
                        putText(images[i], ToString<int>(grid_index), p, FONT_HERSHEY_SIMPLEX, 0.4, b_color,1);

                        global_index = number_corners_per*pattern_index + grid_index;
                        points_present[i][global_index] = true;
                        two_d_point_coordinates_dense[i](global_index, 0) = detections[j].p[k].first;
                        two_d_point_coordinates_dense[i](global_index, 1) = detections[j].p[k].second;

                    }
                }
            }
        }

        if (i < number_external_images_max || write_internal_images){

            filename = write_dir + "initial_detect" + ToString<int>(i) + ".png";
            imwrite(filename.c_str(), images[i]);
        }
    }


    for (int i = 0; i < number_images; i++){
        out << "image " << i << endl;

        for (int j = 0; j < number_corners_per_pattern*number_patterns; j++){
            if (points_present[i][j] == true){
                out << j << " " << std::setprecision(9) << two_d_point_coordinates_dense[i](j, 0)
                                                                                                                                                                                                                                                                                  << " " << std::setprecision(9) << two_d_point_coordinates_dense[i](j, 1) << " ";
            }
        }


        out << endl << "end " << i << endl;
    }


    // fill in everything computed in this function.
    out << "Image-board truth table " << endl;
    for (int i = 0; i < number_images; i++){
        out << "Image " << i << " : ";
        for (int p = 0; p < number_patterns; p++){
            out <<  PointsAtImagePattern(i, p) << " " ;
        }
        out << endl;
    }

    out.close();

}

//used --11/24
void CameraCali::FindCorners(const string& write_dir, bool write_internal_images ){


    if (P_class->is_charuco){
        if (!P_class->rotate_case){
            FindCornersCharuco(write_dir, write_internal_images);
        }   else {
            cout << "This release does not have the rotate case!" << __LINE__ << " in " << __FILE__ << endl;
            exit(1);
        }
    }   else {
        if (P_class->is_april){
            FindCornersApril(write_dir, write_internal_images);
        }
    }
}

// used 11/24
void CameraCali::CalibrateBasic(float initial_focal_px, int zero_tangent_dist,
        int zero_k3, int fix_principal_point, const string& write_dir, int number_points_needed_to_count,
        bool write_internal_images){


    int number_corners_per = P_class->NumberCornersPerPattern();


    int start_index = 0;
    int end_index = 0;


    /// want to recover pose after calibration ...need a map.
    vector<int> mapping_from_limited_to_full_images;
    vector<int> mapping_from_limited_to_full_patterns;

    // create a collection of the points for each image -- hopefully this will work. map -- .
    vector< vector< cv::Point2f> > twod_points_wo_blanks;
    vector< vector< cv::Point3f> > threed_points_wo_blanks;

    int number_images = images.size();
    int number_patterns = P_class->NumberPatterns();

    cout << "Number images " << number_images << endl;

    vector<vector<int> > points_per_board;

    for (int i = 0; i < number_images; i++){
        points_per_board.push_back(vector<int>(number_patterns, 3));
        for (int j = 0; j < number_patterns; j++){
            points_per_board[i][j] = PointsAtImagePattern(i, j);
        }
    }

    int s;
    cv::Size image_size;
    int last_added;
    has_calibration_estimate.resize(number_images, vector<bool>(number_patterns, false));

    int rows = 0; int cols = 0;


    for (int i = 0; i < number_images; i++){
        cout << "Image "  << i << endl;
        rows = images[i].rows;
        cols = images[i].cols;
        for (int p = 0; p < number_patterns; p++){
            if (points_per_board[i][p] >= number_points_needed_to_count){ // opencv calibration, may change for own calibration later.
                //   For this stage, only use high-quality poses.
                {
                    has_calibration_estimate[i][p] = true;

                    cout << "Points found per image " << points_per_board[i][p] << endl;

                    mapping_from_limited_to_full_images.push_back(i);
                    mapping_from_limited_to_full_patterns.push_back(p);

                    /// each board is a new observation, as is each image.
                    twod_points_wo_blanks.push_back(vector< cv::Point2f>(points_per_board[i][p]));
                    threed_points_wo_blanks.push_back(vector< cv::Point3f>(points_per_board[i][p]));

                    /// then, walk through all of the possibles ONLY AT THIS PATTERN/BOARD.
                    s = 0;
                    last_added = twod_points_wo_blanks.size();
                    last_added--;

                    start_index = number_corners_per*p;
                    end_index = number_corners_per*(p + 1);
                    //for (int j = P_class->min_max_id_squares[p].first; j <= P_class->min_max_id_squares[p].second; j++){

                    for (int j = start_index; j < end_index; j++){
                        if (points_present[i].at(j) == true){

                            twod_points_wo_blanks[last_added][s].x = two_d_point_coordinates_dense[i](j, 0);    /// twod points w/o blanks is NOT per image to make internal cali work.
                            twod_points_wo_blanks[last_added][s].y = two_d_point_coordinates_dense[i](j, 1);

                            threed_points_wo_blanks[last_added][s].x = P_class->three_d_points[j].x;
                            threed_points_wo_blanks[last_added][s].y = P_class->three_d_points[j].y;
                            threed_points_wo_blanks[last_added][s].z = P_class->three_d_points[j].z;

                            s++;
                        }
                    }
                }
            }
        }
    }

    if (twod_points_wo_blanks.size() == 0){
        cout << "Could not calibrate this camera, because there are no detected points. Suggestions -- increase the number of images, --max-external, etc." << endl;
        exit(1);
    }

    image_size = Size(cols, rows);


    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs = cv::Mat::zeros(5, 1, CV_64F);


    vector<cv::Mat> rvecs, tvecs;
    double max_dim = cols;
    max_dim < rows ? max_dim = rows : 0;

    double focal_length_px = max_dim*1.2;

    if (initial_focal_px > 0){

        focal_length_px = initial_focal_px;
    }

    cameraMatrix.at<double>(0, 0) = focal_length_px;
    cameraMatrix.at<double>(1, 1) = focal_length_px;

    cameraMatrix.at<double>(0, 2) = cols/2;
    cameraMatrix.at<double>(1, 2) = rows/2;


    cout << "Running calibration " << endl;

    double rms = 0;

    int flags = cv::CALIB_USE_INTRINSIC_GUESS;


    if (zero_k3){
        flags = flags |  cv::CALIB_FIX_K3;
    }

    if (fix_principal_point){
        flags = flags | cv::CALIB_FIX_PRINCIPAL_POINT;
    }

    if (zero_tangent_dist){
        flags = flags | cv::CALIB_ZERO_TANGENT_DIST;
    }

    rms = cv::calibrateCamera(threed_points_wo_blanks, twod_points_wo_blanks, image_size, cameraMatrix, distCoeffs, rvecs, tvecs, flags);

    cout << "rms " << rms << endl;

    /// write calibration details now.  Also, transfer to the Eigen format.

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            internal_parameters(i, j) = cameraMatrix.at<double>(i, j);
        }
    }

    distortion.resize(distCoeffs.rows);
    for (int i = 0; i < distCoeffs.rows; i++){
        distortion(i) = distCoeffs.at<double>(i, 0);

    }

    cout << "distortion " << distortion << endl;
    cout << "Camera matrix " << endl << cameraMatrix << endl;

    cv::Mat rotMatrix = cv::Mat::eye(3, 3, CV_64F);
    vector< vector <double> > tempRt(3, vector<double>(4, 0));


    double reproj_error = 0;
    vector<cv::Point2f> imagePoints2;
    double err;
    int correct_image;
    int correct_pattern;

    for (int i = 0; i < number_images; i++){
        reproject_cam_cali_images.push_back(images[i].clone());
    }


    vector<double> temp_repro(number_patterns, 0);
    reproj_error_per_board.resize(number_images, temp_repro);

    for (int m = 0; m < int(twod_points_wo_blanks.size()); m++){

        cv::projectPoints( cv::Mat(threed_points_wo_blanks[m]), rvecs[m], tvecs[m], cameraMatrix,  // project
                distCoeffs, imagePoints2);
        err = cv::norm(cv::Mat(twod_points_wo_blanks[m]), cv::Mat(imagePoints2), CV_L2);              // difference
        reproj_error        += err*err;

        //reproj_image_points.push_back(imagePoints2);
        correct_image = mapping_from_limited_to_full_images[m];
        correct_pattern = mapping_from_limited_to_full_patterns[m];
        reproj_error_per_board[correct_image][correct_pattern] = err*err;

        for (int j = 0, jn = imagePoints2.size(); j < jn; j++){
            line(reproject_cam_cali_images[correct_image], twod_points_wo_blanks[m][j],imagePoints2[j], Scalar(255, 0, 255), 2 );
        }
    }

    ////////////////////// External -- write into class variables ///////////////////////////////////////////

    Matrix4d I;  I.setIdentity();

    // initialize
    vector<Matrix4d> patterns_base(number_patterns, I);
    // whether or not the board is present tells us whether to look at the value there.
    external_parameters.resize(number_images, patterns_base);

    int image_index;
    int pattern_index;

    /// convert from the calibration to saved values.
    for (int stre = 0, stre_total = int(twod_points_wo_blanks.size()); stre < stre_total; stre++){

        cv::Rodrigues(rvecs[stre], rotMatrix);

        image_index = mapping_from_limited_to_full_images[stre];
        pattern_index = mapping_from_limited_to_full_patterns[stre];


        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                external_parameters[image_index][pattern_index](i, j) = rotMatrix.at<double>(i, j);
            }

            external_parameters[image_index][pattern_index](i, 3) = tvecs[stre].at<double>(i);
        }
    }


    /////////////////////////////// UNDISTORT, WRITE REPROJECTION ////////////////////////////////////
    cv::Mat view, rview, map1, map2;
    //	cv::Mat gray;
    string filename;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
            cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, image_size, 1, image_size, 0),
            image_size, CV_16SC2, map1, map2);


    int number_to_write = 0;

    write_internal_images == false ? number_to_write = number_external_images_max: number_to_write = number_images;

    assert(int(reproject_cam_cali_images.size()) >= number_to_write);

#pragma omp parallel for private(filename)
    for (int i = 0; i < number_to_write; i++){
        Mat remapped;
#pragma omp critical
        {
            if (i% 10 == 0){
                cout << "Writing external " << i << endl;
            }
        }
        cv::remap(reproject_cam_cali_images.at(i), remapped, map1, map2, cv::INTER_LINEAR);

        filename  = write_dir + "/ext" + ToString<int>(i) + ".png";

        {
            cv::imwrite(filename.c_str(), remapped);
        }
    }


    cout << "internal parameters in function : " << internal_parameters << endl;
    ///////////////////////////////// WRITE CALIBRATION INFORMATION ///////////////////////////
    std::ofstream out;
    filename = write_dir + "cali_results.txt";
    out.open(filename.c_str());

    out << "Number_patterns " << twod_points_wo_blanks.size() << endl;
    out << "rms " << rms << endl;
    out << "internal_matrix " << endl;
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            out << internal_parameters(i, j) << " ";
        }
        out << endl;
    }

    out << "distortion_size " << distortion.rows() << endl;
    out << "distortion_vector " << endl << distortion << endl;

    for (int stre = 0, stre_total = int(twod_points_wo_blanks.size()); stre < stre_total; stre++){


        image_index = mapping_from_limited_to_full_images.at(stre);
        pattern_index = mapping_from_limited_to_full_patterns.at(stre);

        out << "EXTERNAL image, pattern, reproj " << image_index << " " << pattern_index << " " << reproj_error_per_board.at(image_index).at(pattern_index) << endl;
        out << external_parameters.at(image_index).at(pattern_index) << endl;
    }


    out << "has-calibration-estimate " << endl;
    for (int i = 0; i < number_images; i++){
        for (int j = 0; j < number_patterns; j++){
            //out << has_calibration_estimate[i][j] << " ";
            out << has_calibration_estimate.at(i).at(j) << " ";
        }
        out << endl;
    }
    out.close();

    filename = write_dir + "two_d_data.txt";
    out.open(filename.c_str());
    for (int m = 0; m < int(twod_points_wo_blanks.size()); m++){
        out << "New-board " << twod_points_wo_blanks[m].size() << endl;
        for (int s = 0; s < int(twod_points_wo_blanks[m].size()); s++){
            out << twod_points_wo_blanks[m][s].x << " " << twod_points_wo_blanks[m][s].y  << endl;
        }
    }
    out.close();

    cout << "At the end of basic calibrate " << __LINE__ << "in " << __FILE__  << endl;
    filename = write_dir + "three_d_data.txt";
    out.open(filename.c_str());
    for (int m = 0; m < int(twod_points_wo_blanks.size()); m++){
        out << "New-board " << twod_points_wo_blanks[m].size() << endl;
        for (int s = 0; s < int(twod_points_wo_blanks[m].size()); s++){
            out << threed_points_wo_blanks[m][s].x << " " << threed_points_wo_blanks[m][s].y  << " " << threed_points_wo_blanks[m][s].z << endl;
        }
    }
    out.close();

    cout << "At the end of basic calibrate " << __LINE__ << "in " << __FILE__  << endl;
    images.clear(); /// we don't need to keep this hanging about .... too much memory.
}


//used. 11/24
int CameraCali::PointsAtImagePattern(int image_number, int pattern_number){


    int number_points = 0;

    int number_corners_per = P_class->NumberCornersPerPattern();

    int start_index = pattern_number*number_corners_per;
    int end_index = (pattern_number + 1)*number_corners_per;

    for (int j = start_index; j < end_index; j++){
        if (points_present[image_number][j] == true){
            number_points++;
        }
    }

    return number_points;
}

//used
void CameraCali::SetUpSelectPointsForMinimization(){

    points_used_min = points_present;  //then points used min will be altered in the multicamera.cpp files.
}

// used. 11/24
double CameraCali::ComputeReprojectionErrorOneImagePattern(const Matrix4d& ExtParameters, int image_number,
        int pattern_number,
        const string& write_directory, bool write, int equation_number, bool rotating,
        const Matrix3d* IntParameters){


    bool write_now = false;
    int number_corners_per = P_class->NumberCornersPerPattern();
    int start_j = pattern_number*number_corners_per;
    int end_j = (pattern_number + 1)*number_corners_per;


    if (write_now){
        cout << "This is image " << image_number << endl;

        cout << "Aprime " << endl << ExtParameters << endl;
    }


    int number_points = PointsAtImagePattern(image_number, pattern_number);

    if (number_points == 0){
        cout << "Number points is zero! " << number_points << endl;
        exit(1);
    }

    vector< cv::Point2f>  twod_points_wo_blanks(number_points);
    vector< cv::Point3f>  threed_points_wo_blanks(number_points);
    vector<cv::Point2f> imagePoints2;
    vector<bool> used_in_min(number_points, false);

    bool select_min_case = points_used_min.size() > 0;
    double err;

    int s = 0;

    for (int j = start_j; j < end_j; j++){
        if (points_present[image_number].at(j) == true){
            twod_points_wo_blanks[s].x = two_d_point_coordinates_dense[image_number](j, 0);    /// twod points w/o blanks is NOT per image to make internal cali work.
            twod_points_wo_blanks[s].y = two_d_point_coordinates_dense[image_number](j, 1);

            threed_points_wo_blanks[s].x = P_class->three_d_points[j].x;
            threed_points_wo_blanks[s].y = P_class->three_d_points[j].y;
            threed_points_wo_blanks[s].z = P_class->three_d_points[j].z;

            if (select_min_case){
                if (points_used_min[image_number].at(j) == true){
                    used_in_min[s] = true;
                }
            }
            s++;
        }
    }

    if (write_now){
        cout << "first 2d point " << twod_points_wo_blanks[0] << endl;
        cout << "first 3d point " << threed_points_wo_blanks[0] << endl;
    }

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

    int n_dist = distortion.rows();
    cv::Mat rotMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat rvec;
    cv::Mat tvec = cv::Mat(3, 1, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(n_dist, 1, CV_64F);

    // convert this matrix to the OpenCV format ....
    if (IntParameters == 0){
        for (int r = 0; r < 3; r++){
            for (int c = 0; c < 3; c++){
                rotMatrix.at<double>(r, c) = ExtParameters(r, c);
                cameraMatrix.at<double>(r, c) = internal_parameters(r, c);
            }
            tvec.at<double>(r, 0) = ExtParameters(r, 3);
        }

    }	else {
        Matrix3d IP = *IntParameters;

        for (int r = 0; r < 3; r++){
            for (int c = 0; c < 3; c++){
                rotMatrix.at<double>(r, c) = ExtParameters(r, c);
                cameraMatrix.at<double>(r, c) = IP(r, c);
            }
            tvec.at<double>(r, 0) = ExtParameters(r, 3);
        }

    }

    if (write_now){
        cout << "internal matrix " << endl << cameraMatrix << endl;
        cout << "rotation matrix " << endl << rotMatrix << endl;
        cout << "translation    " << endl << tvec << endl;
    }
    cv::Rodrigues(rotMatrix, rvec);

    for (int i = 0; i < n_dist; i++){
        distCoeffs.at<double>(i, 0) = distortion(i);
    }


    double reproj_error = 0;


    cv::projectPoints( cv::Mat(threed_points_wo_blanks), rvec, tvec, cameraMatrix,  // project
            distCoeffs, imagePoints2);
    err = cv::norm(cv::Mat(twod_points_wo_blanks), cv::Mat(imagePoints2), CV_L2);              // difference
    reproj_error        += err*err;


    // scale by the number of points ....
    reproj_error = reproj_error / double(number_points);

    if (write){

        // different colors for the different types.
        Scalar line_color(0, 0, 0);

        line_color = P_class->Color(pattern_number);

        Mat im_copy = imread(im_names[image_number].c_str(), IMREAD_COLOR);
        int number_p = imagePoints2.size();

        for (int j = 0, jn = imagePoints2.size(); j < jn; j++){
            line(im_copy, twod_points_wo_blanks[j],imagePoints2[j], line_color, 2 );

            if (select_min_case){
                if (used_in_min[j] == true){
                    circle(im_copy, twod_points_wo_blanks[j], 30, line_color, 2);
                }
            }
        }

        if (rotating){
            for (int j = 0, jn = imagePoints2.size() - 1; j < jn; j++){
                line(im_copy, imagePoints2[j],imagePoints2[j + 1], line_color, 10 );
            }

            line(im_copy, imagePoints2[0],imagePoints2[number_p - 1], line_color, 10 );
        }


        string filename;

        filename = write_directory + "Equation" + ToString<int>(equation_number);

        if (rotating){
            filename = filename + ".jpg";

            // resize
            Mat im_small;
            Size si;

            si.width = im_copy.cols/8;
            si.height = im_copy.rows/8;

            resize(im_copy, im_small, si);

            imwrite(filename.c_str(), im_small);

        }	else {
            filename = filename + ".png";
            imwrite(filename.c_str(), im_copy);
        }
    }

    return reproj_error;

}

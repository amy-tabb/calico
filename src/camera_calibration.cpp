/*
 * camera_calibration.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: atabb
 */

#include "camera_calibration.hpp"
#include "DirectoryFunctions.hpp"
#include "helper.hpp"
#include "local_charuco.hpp"

bool readDetectorParameters(const string& filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

Matrix3d CopyRotationMatrixFromExternal(Matrix4d& M4){

    Matrix3d M3;
    for (int r = 0; r < 3; r++){
        for (int c = 0; c < 3; c++){
            M3(r, c) = M4(r, c);
        }
    }

    return M3;

}

string CreatePaddedNumberString(int number, int length){

    string temp_string = ToString<int>(number);

    int n = length - temp_string.size();

    for (int i = 0; i < n; i++){
        temp_string = "0" + temp_string;
    }

    return temp_string;
}

int CreateRotateCaseImagesCharuco(vector<Mat>& images, int squaresX, int squaresY, int squareLength, int markerLength,
        int margins, int id_start_number, int dictionary_version, Ptr<aruco::DetectorParameters>& detectorParams){

    Ptr<aruco::Dictionary> dictionary =
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_version));

    Size imageSize;
    int number_markers = (squaresX/2)*((squaresY + 1)/2) + ((squaresX + 1)/2)*(squaresY/2);

    imageSize.width = squaresX * squareLength;
    imageSize.height = squaresY * squareLength;


    //////////// this creates the aruco board.
    Mat markerImg;
    Mat boardImage = Mat::zeros(imageSize.height, imageSize.width, CV_8UC1);
    boardImage.setTo(255);
    int x0, y0;

    for (int x = 0, count = 0; x < squaresX; x++){
        for (int y = 0; y < squaresY; y++, count++){
            aruco::drawMarker(dictionary, count, markerLength, markerImg, 1);

            /// where to place?
            x0 = x*squareLength + margins/2;
            y0 = y*squareLength + margins/2;

            Rect R = Rect(x0, y0, markerLength, markerLength);

            markerImg.copyTo(boardImage(R));
        }

    }
    images.push_back(boardImage);


    markerLength = squareLength*.9;
    int ss = 3;
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(ss, ss, 0.04, 0.9*0.04, dictionary);

    vector<cv::Ptr<cv::aruco::CharucoBoard> > boards;
    boards.push_back(cv::aruco::CharucoBoard::create(ss, ss, 0.04, 0.02, dictionary));
    imageSize.width = 1 * squareLength;
    imageSize.height = 1 * squareLength;
    boardImage = Mat::zeros(imageSize.height, imageSize.width, CV_8UC1);
    boardImage.setTo(255);

    boards[0]->ids[0] = id_start_number;
    board->ids[0] = id_start_number;
    board->draw( imageSize, boardImage, 10, 1 );

    images.push_back(boardImage.clone());

    for (int j  = 1; j < 8; j++){

        boards.push_back(cv::aruco::CharucoBoard::create(ss, ss, 0.04, 0.02, dictionary));
        //for
        boards[j]->ids[0] = id_start_number + j*4;

        for (int k = 0; k < 4; k++){
            board->ids[k] += 4;
        }
        board->draw( imageSize, boardImage, 10, 1 );

        images.push_back(boardImage.clone());
    }


    int board_selected;

    for (int i = 1, in = images.size(); i < in; i++){
        Mat image, imageCopy;

        image = images[i].clone();

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector< Vec3d > rvecs, tvecs;

        // detect markers and estimate pose
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) {
            cout << ids[0] << endl;
            aruco::drawDetectedMarkers(imageCopy, corners, ids);

            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            /// can choose which board to use here ...
            board_selected = ids[0]/number_markers;
            cv::aruco::interpolateCornersCharuco(corners, ids, image, boards[board_selected], charucoCorners, charucoIds);

            // if at least one charuco corner detected
            if(charucoIds.size() > 0)
            {
                /// corners (2d) are linked to the Ids
                cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
            }

            cout << "board " << board_selected << endl;

            images.push_back(imageCopy.clone());
        }
    }

    return 0;
}

PatternsCreated::PatternsCreated(const string& read_dir, const string& write_dir, bool rotate,
        const string& src_file, bool generate_only){

    vector<int> squaresX;
    vector<int> squaresY;
    vector<int> pixelSquareLength;
    vector<int> pixelMarkerLength;
    vector<double> squareLength;
    vector<int> number_markers;
    int number_boards;
    int number_markers_this_board;
    rotate_case = rotate;

    ofstream out;
    string filename_write;
    string command;

    detectorParams = aruco::DetectorParameters::create();

    bool readOk = readDetectorParameters(src_file, detectorParams);
    if(!readOk) {
        cout << "Invalid detector parameters file, quitting " << src_file << endl;
        cout << __LINE__ << " in " << __FILE__ << endl;
        exit(1);
    }

    if (rotate_case == false){
        max_internal_patterns = 0;
        internalx = 0;  internaly = 0;

        string returnString;
        int dictionary_version;

        Mat boardCopy;

        /// read spec file,
        string filename = read_dir + "network_specification_file.txt";
        ifstream in;

        returnString = FindValueOfFieldInFile(filename, "aruco_dict", false, true);
        dictionary_version = FromString<int>(returnString);


        returnString = FindValueOfFieldInFile(filename, "number_boards", false, true);
        number_boards = FromString<int>(returnString);

        dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_version));

        in.open(filename.c_str());

        // This particular read has to be in order.
        string temp;
        for (int i = 0; i < 4; i++){
            in >> temp;
        }

        for (int i = 0; i < number_boards; i++){
            in >> temp >> temp; squaresX.push_back(FromString<int>(temp));
            in >> temp >> temp; squaresY.push_back(FromString<int>(temp));
            in >> temp >> temp; pixelSquareLength.push_back(FromString<int>(temp));
            in >> temp >> temp; pixelMarkerLength.push_back(FromString<int>(temp));
        }
        in.close();

        string filename_write = write_dir + "network_specification_file.txt";

        CopyFile(filename, filename_write);


        for (int i = 0, m = 0, s = 0; i < number_boards; i++){
            /// the dimensions of the board are linked in here ....
            boards.push_back(cv::aruco::CharucoBoard::create(squaresX[i], squaresY[i], pixelSquareLength[i], pixelMarkerLength[i], dictionary));

            square_h_w.push_back(pair<int, int>(squaresY[i], squaresX[i]));

            min_max_id_pattern.push_back(pair<int, int>(m, m));
            number_markers_this_board = boards[i]->ids.size();
            number_markers.push_back(number_markers_this_board);

            for (int j = 0; j < number_markers_this_board; j++, m++){
                boards[i]->ids[j] = m;
            }

            min_max_id_pattern[i].second = min_max_id_pattern[i].first + number_markers[i] - 1;
            min_max_id_squares.push_back(pair<int, int>(s, s));
            s = s + (squaresX[i] - 1)*(squaresY[i] - 1) - 1;
            min_max_id_squares[i].second = s;
            s = s + 1;


            cv::Size imageSize;
            imageSize.width = 20 + 1 * pixelSquareLength[i]*squaresX[i];
            imageSize.height = 20 + 1 * pixelSquareLength[i] *squaresY[i];
            Mat boardImage(imageSize.height, imageSize.width, CV_8UC1, 255);

            boards[i]->draw( imageSize, boardImage, 10, 1 );
            filename = write_dir + "Board" + ToString<int>(i) + ".png";
            imwrite(filename.c_str(), boardImage);

            // write one with the markers.
            cvtColor(boardImage, boardCopy, cv::COLOR_GRAY2RGB);

            vector< vector< Point2f > > corners, rejected;
            vector< int > ids;
            // detect markers and estimate pose
            aruco::detectMarkers(boardImage, dictionary, corners, ids, detectorParams, rejected);

            aruco::drawDetectedMarkers(boardCopy, corners, ids, Scalar(255, 255, 0));
            filename = write_dir + "BoardWithArucoLabels" + ToString<int>(i) + ".png";
            cv::imwrite(filename, boardCopy);

        }

        //pendatic, but want to avoid re-computing it and making a mistake later.
        number_total_squares = min_max_id_squares[number_boards - 1].second  + 1;
        int_number_markers = min_max_id_pattern[number_boards - 1].second + 1;

        float mm = 0;
        // then open up all of the physical measurement files, add to square length.
        for (int i = 0; i < number_boards; i++){
            if (!generate_only){
                filename = read_dir + "pattern_square_mm" + ToString<int>(i) + ".txt";

                returnString = FindValueOfFieldInFile(filename, "squareLength_mm", false, true);
                mm = FromString<float>(returnString);

                squareLength.push_back(mm);

                in.close();

                filename_write = write_dir + "pattern_square_mm" + ToString<int>(i) + ".txt";

                CopyFile(filename, filename_write);
            }	else {

                filename_write = write_dir + "pattern_square_mm" + ToString<int>(i) + ".txt";

                out.open(filename_write.c_str());

                out << "squareLength_mm  XX" << endl;

                out.close();
            }
        }

        // convert everything to the class members.
        //////////////////////// double to single.//////////////////////////////
        if (!generate_only){
            three_d_points = vector< cv::Point3f >(number_total_squares, cv::Point3f());
            int sm = 0;
            for (int i = 0, sc = 0; i < number_boards; i++){
                mm = squareLength[i];

                vector<int> current_index(number_markers[i], 0);

                for (int m = 0; m < number_markers[i]; m++, sc++){
                    current_index[m] = sc;
                }

                for (int r = 0; r < squaresY[i] - 1; r++){
                    for (int c = 0; c < squaresX[i] - 1; c++, sm++){
                        Point3f p(mm*float(c), float(r)*mm, 0);
                        three_d_points[sm] = p;
                    }
                }

                double_to_single.push_back(current_index);
            }
        }


        number_patterns = number_boards;


        // a little hack, to be improved in the next version
        //square_h_w.push_back(pair<int, int>(squaresY[i], squaresX[i]));
        number_corners_per_pattern =squaresX[0]* squaresY[0]*4;
    }	else {
        ////////////////////////////////////////////////////////
        //////// Rotate case, possible internal parameters ///////
        /////////////////////////////////////////////////////////

        vector<Mat> images;
        string temp;
        int sLin, mLin, margin, id_start, squaresXext, squaresYext, num_markExt,
        sLext, mLext, dictionary_version;
        num_markExt = 0;
        ifstream in;
        string returnString;
        double mm;
        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        Mat imageCopy;
        double mm_internal = 0;
        double mm_external = 0;

        /// read specification file,
        string filename = read_dir + "rotate_specification_file.txt";;

        returnString = FindValueOfFieldInFile(filename, "aruco_dict", false, true);
        dictionary_version = FromString<int>(returnString);

        returnString = FindValueOfFieldInFile(filename, "internal_squaresX", false, true);
        internalx = FromString<int>(returnString);

        returnString = FindValueOfFieldInFile(filename, "internal_squaresY", false, true);
        internaly = FromString<int>(returnString);

        returnString = FindValueOfFieldInFile(filename, "internal_squareLength", false, true);
        sLin = FromString<int>(returnString);

        returnString = FindValueOfFieldInFile(filename, "internal_markerLength", false, true);
        mLin = FromString<int>(returnString);

        returnString = FindValueOfFieldInFile(filename, "internal_margin", false, true);
        margin = FromString<int>(returnString);

        returnString = FindValueOfFieldInFile(filename, "number_boards", false, true);
        number_boards = FromString<int>(returnString);

        returnString = FindValueOfFieldInFile(filename, "squaresX", false, true);
        squaresXext = FromString<int>(returnString);

        returnString = FindValueOfFieldInFile(filename, "squaresY", false, true);
        squaresYext = FromString<int>(returnString);

        returnString = FindValueOfFieldInFile(filename, "squareLength", false, true);
        sLext = FromString<int>(returnString);

        returnString = FindValueOfFieldInFile(filename, "markerLength", false, true);
        mLext = FromString<int>(returnString);

        id_start = internalx*internaly;

        /// now, read in mm for external
        // now, read in mm for internal

        cout << "After read " << number_boards << endl;

        filename_write = write_dir + "rotate_specification_file.txt";

//        string command = "cp " + filename + " " + filename_write;
//
//        system(command.c_str());

        CopyFile(filename, filename_write);

        dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionary_version));

        for (int i = 0, m = id_start, s = 0; i < number_boards; i++){
            /// the dimensions of the board are linked in here ....
            boards.push_back(cv::aruco::CharucoBoard::create(squaresXext, squaresYext, sLext, mLext, dictionary));
            square_h_w.push_back(pair<int, int>(squaresYext, squaresXext));


            min_max_id_pattern.push_back(pair<int, int>(m, m));

            num_markExt = boards[i]->ids.size();

            // relabel so that the aruco and charuco labels are different.
            for (int j = 0; j < num_markExt; j++, m++){
                boards[i]->ids[j] = m;
            }

            min_max_id_pattern[i].second = min_max_id_pattern[i].first + num_markExt - 1;
            min_max_id_squares.push_back(pair<int, int>(s, s));
            s = s + (squaresXext - 1)*(squaresYext - 1) - 1;
            min_max_id_squares[i].second = s;
            s = s + 1;

            cv::Size imageSize;
            imageSize.width = 20 + 1 * sLext * squaresXext;
            imageSize.height = 20 + 1 * sLext * squaresYext;
            Mat boardImage = Mat::zeros(imageSize.height, imageSize.width, CV_8UC1);
            boardImage.setTo(255);

            boards[i]->draw( imageSize, boardImage, 10, 1 );
            filename = write_dir + "Marker" + ToString<int>(i) + ".png";
            imwrite(filename.c_str(), boardImage);
        }

        // this is all inane, but want to avoid re-computing it and making a mistake later.
        number_total_squares = min_max_id_squares[number_boards - 1].second  + 1;
        int_number_markers = min_max_id_pattern[number_boards - 1].second + 1;

        filename = read_dir + "pattern_square_mm_external.txt";

        if (!generate_only){
            returnString = FindValueOfFieldInFile(filename, "squareLength_mm", false, true);
            mm_external = FromString<float>(returnString);

            for (int i = 0; i < number_boards; i++){
                squareLength.push_back(mm_external);
            }


            //////////////////////// double to single.//////////////////////////////
            three_d_points = vector< cv::Point3f >(number_total_squares, cv::Point3f());
            int sm = 0;
            for (int i = 0, sc = 0; i < number_boards; i++){
                mm = squareLength[i];

                vector<int> current_index(num_markExt, 0);

                for (int m = 0; m < num_markExt; m++, sc++){
                    current_index[m] = sc;
                }

                for (int r = 0; r < squaresYext - 1; r++){
                    for (int c = 0; c < squaresXext - 1; c++, sm++){
                        Point3f p(mm*float(c), float(r)*mm, 0);
                        three_d_points[sm] = p;
                    }
                }

                double_to_single.push_back(current_index);
            }

            filename = read_dir + "pattern_square_mm_external.txt";
            filename_write = write_dir + "pattern_square_mm_external.txt";
//            command = "cp " + filename + " " + filename_write;
//
//            system(command.c_str());

            CopyFile(filename, filename_write);


        }	else {
            filename_write = write_dir +  "pattern_square_mm_external.txt";

            out.open(filename_write.c_str());

            out << "squareLength_mm  XX" << endl;

            out.close();
        }


        // pendantic, to avoid re-computing it and making a mistake later.
        number_total_squares = min_max_id_squares[number_boards - 1].second  + 1;
        int_number_markers = min_max_id_pattern[number_boards - 1].second + 1;

        cout << "Number boards " << number_boards << endl;
        cout << "Number total squares " << number_total_squares << endl;

        cout << "Before 3d points " << endl;

        /// internal markers.
        filename = read_dir + "pattern_square_mm_internal.txt";

        if (!generate_only){

            returnString = FindValueOfFieldInFile(filename, "squareLength_mm", false, true);
            mm_internal = FromString<float>(returnString);


            filename = read_dir + "pattern_square_mm_internal.txt";
            filename_write = write_dir + "pattern_square_mm_internal.txt";

            CopyFile(filename, filename_write);

        }	else {
            filename_write = write_dir +  "pattern_square_mm_internal.txt";

            out.open(filename_write.c_str());

            out << "squareLength_mm  XX" << endl;

            out.close();
        }

        cout << "Before create images " << endl;
        // detect pattern locations on the image..

        CreateRotateCaseImagesCharuco(images, internalx, internaly, sLin, mLin, margin, id_start, dictionary_version, detectorParams);
        filename = write_dir + "created_backstop.png";
        imwrite(filename.c_str(), images[0]);

        // for each tag, generate where it should be in 3D space.  so indices are tag0 corner0-3 tag1 corner0-3 etc., .etc.  not on a grid necessarily.

        aruco::detectMarkers(images[0], dictionary, corners, ids, detectorParams, rejected);

        cvtColor(images[0], imageCopy, cv::COLOR_GRAY2RGB);

        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids, Scalar(255, 255, 0));
            filename = write_dir + "internal_with_labels.png";
            cv::imwrite(filename, imageCopy);
        }

        ///////////// FILL IN THE INTERAL PARAMETERS AND WRITE ///////////////////

        // internal_mm is the measurement per square edge.
        // how far apart are the squares?

        if (!generate_only){
            max_internal_patterns = internalx*internaly;
            if (ids.size() > 0 && int(ids.size()) == max_internal_patterns){
                vector<int> mapping_to_id(max_internal_patterns, 0);
                // are the ids in order?
                for (int i = 0, in = ids.size(); i < in; i++){
                    mapping_to_id[ids[i]] = i;
                }

                double distance_between_squares = mm_internal*double(sLin)/double(mLin);

                cout << "Current settings, square length, distance bewteen squares froms start to finish " << mm_internal << ", " << distance_between_squares << endl;

                int current_index;
                int x_value, y_value;
                vector< Point2f > current_corners;

                int internal_squares = 4*max_internal_patterns;
                three_d_points_internal = vector< cv::Point3f >(internal_squares, cv::Point3f());

                for (int i = 0, in = ids.size(); i < in; i++){
                    cout << "Pattern creation, in this order: " << ids[i] << endl;
                    current_index = ids[i];
                    current_corners = corners[i];

                    x_value = current_index/internaly;
                    y_value = internaly - current_index%internaly - 1;

                    // assign the value -- clockwise
                    Point3f p0(distance_between_squares*double(x_value), distance_between_squares*double(y_value) + mm_internal, 0);
                    Point3f p1(distance_between_squares*double(x_value) + mm_internal, distance_between_squares*double(y_value) + mm_internal, 0);
                    Point3f p2(distance_between_squares*double(x_value) + mm_internal, distance_between_squares*double(y_value), 0);
                    Point3f p3(distance_between_squares*double(x_value), distance_between_squares*double(y_value), 0);

                    three_d_points_internal[4*current_index] = p0; // would correspond to current corners[0]
                    three_d_points_internal[4*current_index + 1] = p1;
                    three_d_points_internal[4*current_index + 2] = p2;
                    three_d_points_internal[4*current_index + 3] = p3;


                    string coords = ToString<float>(x_value) + ", " + ToString<float>(y_value);
                    putText(imageCopy, coords, Point(current_corners[0].x,current_corners[0].y), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 0, 0), 1);
                    for (int j = 1; j < 4; j++){
                        string coords = ToString<float>(three_d_points_internal[4*current_index + j].x) + ", " + ToString<float>(three_d_points_internal[4*current_index + j].y);
                        putText(imageCopy, coords, Point(current_corners[j].x,current_corners[j].y), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 0, 0), 1);
                    }
                }

                filename = write_dir + "internal_with_morelabels.png";
                cv::imwrite(filename, imageCopy);
            }	else {
                cout << "not finding all of the elements .... squareX, Y is wrong?  Double check specification file" << endl;
                exit(1);
            }
        }

        number_patterns = number_boards;
        number_corners_per_pattern =squaresXext* squaresYext*4;
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


int PatternsCreated::NumberMarkers() const{
    return int_number_markers;
}

int PatternsCreated::NumberPatterns() const{
    return number_patterns;
}

int PatternsCreated::NumberSquares() const{
    return number_total_squares;
}


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

    vector<string> new_names;


    if (dir_exists){
        ReadDirectory(ext_dir, im_names);
    }	else {
        ext_dir = read_dir;
        ReadDirectory(read_dir, im_names);
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
        im = imread(new_names[0].c_str(), IMREAD_COLOR);
        rows = im.rows;
        cols = im.cols;
        images.resize(new_names.size());

#pragma omp parallel for
        for (int i = 0; i < number_external_images_max; i++){

            images[i] = imread(new_names[i].c_str(), IMREAD_COLOR);

        }

        new_names.swap(im_names);
    }

    /// read the internal dir.
    string int_dir = read_dir + "/internal/";
    dir_exists = IsDirectory(int_dir);

    vector<string> aux_names;
    vector<string> new_names2;

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


void CameraCali::ReadExifInformationRotatingSet(const string& input_dir, const string& read_dir){

    vector<string> im_names;
    string filename;
    ReadDirectory(read_dir, im_names);


    filename = read_dir + "/" + im_names[0];

    string command;
    string exif_filename = "../exif.txt";

    command = "exiftool " + filename + " > " + exif_filename;
    int ret_val = system(command.c_str());

    if (ret_val != 0){
        cout << "Error running " << command << endl;
        cout << " at " << __LINE__ << " in file " << __FILE__ << endl;
        exit(1);
    }

    string sensor_characteristics_filename = input_dir + "rotate_specification_file.txt";

    double sensor_width = 23.5;
    double focal_length = 16.0;
    string returnString;
    string sensorName_specs;
    string sensorName_exif;
    int image_width = 0;

    returnString = FindValueOfFieldInFile(sensor_characteristics_filename, "sensorWidth", false, true);
    sensor_width = FromString<double>(returnString);

    returnString = FindMultipleStringValueOfFieldInFile(exif_filename, "Focal Length", true, true, 2);
    cout << "focal length .... testing ... " << returnString << endl;
    focal_length = FromString<double>(returnString);

    returnString = FindMultipleStringValueOfFieldInFile(exif_filename, "Exif Image Width", true, true, 3);
    image_width = FromString<int>(returnString);

    pixel_width = double(image_width)*focal_length/sensor_width;

}

void PatternsCreated::DetermineBoardsPresentFromMarkerList(const vector<int>& markers, vector<bool>& boards_seen){

    int np = NumberPatterns();

    for (int i = 0, in = markers.size(); i < in; i++){
        for (int j = 0;  j < np; j++){ // don't walk this one if it is already occupied.
            if (markers[i] >= min_max_id_pattern[j].first && markers[i] <= min_max_id_pattern[j].second){
                boards_seen[j] = true;
            }
        }
    }
}

void PatternsCreated::DetermineBoardsPresentFromMarkerList(const vector<int>& markers, vector<bool>& boards_seen,
        const vector< vector< Point2f > >& corners, vector< vector< vector< Point2f > > >& corners_sorted,
        vector< vector<int> >& markers_sorted){

    int np = NumberPatterns();

    corners_sorted.resize(np);
    markers_sorted.resize(np);

    for (int i = 0, in = markers.size(); i < in; i++){
        for (int j = 0;  j < np; j++){ // don't walk this one if it is already occupied.
            if (markers[i] >= min_max_id_pattern[j].first && markers[i] <= min_max_id_pattern[j].second){
                boards_seen[j] = true;
                markers_sorted[j].push_back(markers[i]);
                corners_sorted[j].push_back(corners[i]);
            }
        }
    }
}


int PatternsCreated::MappingArucoIDToPatternNumber(int id){
    int np = NumberPatterns();

    for (int j = 0;  j < np; j++){ // don't walk this one if it is already occupied.
        if (id >= min_max_id_pattern[j].first && id <= min_max_id_pattern[j].second){
            return j;
        }
    }

    return -1;
}


Scalar PatternsCreated::Color(int index){

    if (index >= int(display_colors.size()) ){
        index = display_colors.size() % index;
    }
    return Scalar(display_colors[index][0], display_colors[index][1],display_colors[index][2]);
}


float euclideanDist(Point2f& p, Point2f& q) {
    Point2f diff = p - q;
    return sqrt(diff.x*diff.x + diff.y*diff.y);
}

void ComputeStats(vector<double>& ds, double& mu, double& var){

    mu = 0;
    double n = ds.size();
    var = 0;

    for (int i = 0, nn = ds.size(); i < nn; i++){
        mu += ds[i];
    }

    mu /= n;

    for (int i = 0, nn = ds.size(); i < nn; i++){
        var += pow(mu - ds[i], 2);
    }

    var /= n;
}

double ComputePDF(double mu, double var, double x){

    double pdf = (1.0/sqrt(2.0*3.14*var))*exp(-0.5*(pow(x - mu, 2)/(var)));

    return pdf;
}

void FilterLargerDuplicates( vector< int >& ids, vector< vector< Point2f > >& corners, int max_internal){

    int n = ids.size();
    vector<bool> keep(n, false);

    vector< Point2f > current_cornersi;
    vector< Point2f > current_corners0;

    vector<int> ids_new;
    vector< vector< Point2f > > corners_new;

    // don't keep until verify that is 1) unique number or 2) is smallest.
    bool found;
    int other_index;

    for (int i = 0; i < n; i++){

        if (keep[i] == false){
            found = false; other_index = -1;

            for (int j  = i + 1; j < n && found == false; j++){
                if (ids[j] == ids[i]){
                    other_index = j;  found = true;
                }
            }

            if (found == false){
                keep[i] = true;
            }	else {
                // need to decide which one to keep.
                current_cornersi = corners[i];
                current_corners0 = corners[other_index];

                /// diagonal
                int di = euclideanDist(current_cornersi[0], current_cornersi[2]);
                int d0 = euclideanDist(current_corners0[0], current_corners0[2]);

                if (di < d0){
                    keep[i] = true;
                }	else {
                    keep[other_index] = true;
                }

            }
        }
    }

    /// now create new list ....
    for (int i = 0; i < n; i++){
        if (keep[i] == true){
            ids_new.push_back(ids[i]);
            corners_new.push_back(corners[i]);
        }
    }

    corners_new.swap(corners);
    ids_new.swap(ids);

    corners_new.clear();
    ids_new.clear();

    vector<double> distances;
    double mu, var;
    //double pdf;


    /// find out the size of the internals ....
    for (int i = 0; i < n; i++){
        if (ids[i] < max_internal){
            current_cornersi = corners[i];
            int di = euclideanDist(current_cornersi[0], current_cornersi[2]);
            distances.push_back(di);
        }
    }

    ComputeStats( distances, mu, var);

    for (int i = 0; i < n; i++){
        if (ids[i] < max_internal){
            current_cornersi = corners[i];
            int di = euclideanDist(current_cornersi[0], current_cornersi[2]);

            if (fabs(mu - di) < 0.20*mu ){
                ids_new.push_back(ids[i]);
                corners_new.push_back(corners[i]);
            }

        }	else {
            ids_new.push_back(ids[i]);
            corners_new.push_back(corners[i]);
        }
    }

    corners_new.swap(corners);
    ids_new.swap(ids);

}


void CameraCali::FindCornersArucoCharuco(const string& write_dir, bool verbose, string verbose_write_dir ){


    int number_patterns = P_class->NumberPatterns();
    int number_markers = P_class->NumberMarkers();
    int number_squares = P_class->NumberSquares();

    if (verbose){
        cout << "number patterns : " << number_patterns << endl;
        cout << "number markers : " << number_markers << endl;
        cout << "number squares : " << number_squares << endl;
    }

    string filename;
    int number_images = images.size();


    Mat imageCopy;
    Scalar b_color;
    MatrixXd twod(number_squares, 2);
    twod.setConstant(0);
    int global_index;

    boards_detected.resize(number_images, vector<bool>(P_class->NumberPatterns(), false));

    std::ofstream out;
    filename = write_dir + "points.txt";
    out.open(filename.c_str());

    /// also, read exif.
    int internal_pattern_max_squares = P_class->max_internal_patterns*4;
    MatrixXd twod_internal(internal_pattern_max_squares, 2);


    if (number_images == 0){
        cout << "the number of images is 0, quitting " << endl;
        exit(1);
    }	else {
        cout << "Number of images " << number_images << endl;
    }


    int current_index = 0;

    /////////// ARUCO PATTERNS ONLY ///////////
    // the non-synch-ness of the rotating set comes into play here, too.
    for (int i = 0; i < 1; i++){  // collecting the internal characteristics
        imageCopy = images[i].clone();

        /// already know the number of points per board.
        internal_two_d_point_coordinates_dense.push_back(twod_internal);

        count_internal_ids_present = 0;

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;

        id_bool.resize(P_class->max_internal_patterns, false);

        // detect markers and estimate pose
        aruco::detectMarkers(images[i], P_class->dictionary, corners, ids, P_class->detectorParams, rejected);

        if(ids.size() > 0) {

            // need to make sure that the list is unique ....can take out later.
            FilterLargerDuplicates(ids, corners, P_class->max_internal_patterns);

            aruco::drawDetectedMarkers(imageCopy, corners, ids, Scalar(0, 255, 255));

            /// walk through the ids -- only process those that are in the range we're interested in.
            for (int id_count = 0, idn = ids.size(); id_count < idn; id_count++){
                // if this particular arUco pattern is within the internal range ...
                if (ids[id_count] < P_class->max_internal_patterns){
                    vector< Point2f > current_corners;
                    current_index = ids[id_count];

                    count_internal_ids_present++;
                    id_bool[current_index] = true;

                    /// not all corners were found, so use id_count for this index.
                    current_corners = corners[id_count];

                    for (int j = 0; j < 3; j++){
                        line(imageCopy, Point(current_corners[j].x,current_corners[j].y), Point(current_corners[j+1].x,current_corners[j+1].y), Scalar(255, 255, 0), 10);
                    }

                    string coords = ToString<int>(current_index);
                    putText(imageCopy, coords, Point(current_corners[0].x,current_corners[0].y), FONT_HERSHEY_DUPLEX, 2, Scalar(255, 0, 0), 2);

                    for (int j = 0; j < 4; j++){
                        internal_two_d_point_coordinates_dense[0](4*current_index + j, 0) = current_corners[j].x;
                        internal_two_d_point_coordinates_dense[0](4*current_index + j, 1) = current_corners[j].y;
                        string coords = ToString<float>(P_class->three_d_points_internal[4*current_index + j].x) + ", " + ToString<float>(P_class->three_d_points_internal[4*current_index + j].y);
                        putText(imageCopy, coords, Point(current_corners[j].x,current_corners[j].y), FONT_HERSHEY_DUPLEX, 2, Scalar(255, 0, 0), 2);
                    }
                }
            }
        }

        filename = write_dir + "internal_initial_detect" + ToString<int>(i) + ".jpg";
        imwrite(filename.c_str(), imageCopy);

    }

    //////////////////////// EXTERNAL PART ... sub in the charuco part from the regular cali files /////////////////////

    two_d_point_coordinates_dense.resize(number_images, twod);
    points_present.resize(number_images, vector<bool>(number_squares, false));

#pragma omp parallel for private(b_color, global_index, filename)
    for (int i = 0; i < number_images; i++){

        vector< vector<cv::Point2f> > charuco_corners_all;
        vector< vector<int> > charuco_ids_all;
        vector<Scalar> colors;

        if (verbose){
#pragma omp critical
            {
                cout << "image " << i << endl;
            }
        }

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;

        // detect markers and estimate pose
        // want this to be done in parallel.
        aruco::detectMarkers(images[i], P_class->dictionary, corners, ids, P_class->detectorParams, rejected);

        // draw results -- dealing with the possibility of more than one board per image.
        if(ids.size() > 0) {
            //we id the board by the aruco tag.
            P_class->DetermineBoardsPresentFromMarkerList(ids, boards_detected[i]);

            // per board refinement and localization of the corners.
            for (int p = 0; p < number_patterns; p++){
                if (boards_detected[i][p] == true){

                    b_color = P_class->Color(p);

                    // want this to be fresh each iter.
                    vector<cv::Point2f> charucoCorners;
                    vector<int> charucoIds;

                    cv::aruco::charucoLocal::interpolateCornersCharucoHomographyLocal(corners, ids, images[i],
                            P_class->boards[p], charucoCorners, charucoIds);

                    // if at least one charuco corner detected
                    if(charucoIds.size() > 0)
                    {
                        if (verbose == true){
#pragma omp critical
                            {
                                cout << "Board " << p << " number " << charucoIds.size() << endl;
                            }
                        }

                        charuco_corners_all.push_back(charucoCorners);
                        charuco_ids_all.push_back(charucoIds);
                        colors.push_back(b_color);

                        for (int j  = 0, jn = charucoIds.size(); j < jn; j++){

                            global_index = P_class->min_max_id_squares[p].first + charucoIds[j];

                            points_present[i][global_index] = true;
                            two_d_point_coordinates_dense[i](global_index, 0) = charucoCorners[j].x;
                            two_d_point_coordinates_dense[i](global_index, 1) = charucoCorners[j].y;

                        }
                    }	else {
                        boards_detected[i][p] = false;  // sometimes there's a marker detected, but we can't grab good corners.
                    }
                }
            }

            for (int j = 0, jn = colors.size(); j < jn; j++){
                for (int p = 0, pn  = charuco_corners_all[j].size(); p < pn; p++){
                    cv::circle(images[i],charuco_corners_all[j][p], 40, colors[j], 6);
                }
            }

        }

        if (verbose){



            filename = verbose_write_dir + "external_initial_detect" + ToString<int>(i) + ".jpg";

            Mat im_small;
            Size si;

            si.width = images[i].cols/2;
            si.height = images[i].rows/2;

            resize(images[i], im_small, si);

            imwrite(filename.c_str(), im_small);
        }

    }

    for (int i = 0; i < number_images; i++){
        out << "image " << i << endl;

        for (int j = 0; j < number_squares; j++){
            if (points_present[i][j] == true){
                out << j << " " << std::setprecision(9) << two_d_point_coordinates_dense[i](j, 0)
														                        << " " << std::setprecision(9) << two_d_point_coordinates_dense[i](j, 1) << " ";
            }
        }


        out << endl << "end " << i << endl;
    }

    // make it easy for self ... fill in everything computed in this function.
    out << "Image-board truth table " << endl;
    for (int i = 0; i < number_images; i++){
        out << "Image " << i << " : ";
        for (int p = 0; p < number_patterns; p++){
            out << 	PointsAtImagePattern(i, p) << " " ;
        }
        out << endl;
    }


    out.close();
}

void convertToMat( InputArray _points, Mat& outArr) {

    outArr = _points.getMat();
}




void CameraCali::FindCornersCharuco(const string& write_dir, bool write_internal_images ){


    int number_patterns = P_class->NumberPatterns();
    int number_squares = P_class->NumberSquares();

    string filename;
    int number_images = images.size();
    Mat imageCopy;
    Scalar b_color;
    MatrixXd twod(number_squares, 2);
    twod.setConstant(0);
    int global_index;

    boards_detected.resize(number_images, vector<bool>(P_class->NumberPatterns(), false));

    std::ofstream out;
    filename = write_dir + "points.txt";
    out.open(filename.c_str());

    two_d_point_coordinates_dense.resize(number_images, twod);
    points_present.resize(number_images, vector<bool>(number_squares, false));


    int internal_found_counter = 0;

#pragma omp parallel for private(b_color, global_index, filename)
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
                P_class->DetermineBoardsPresentFromMarkerList(ids, boards_detected[i]);

                // per board refinement and localization of the corners.
                for (int p = 0; p < number_patterns; p++){
                    if (boards_detected[i][p] == true){

                        b_color = P_class->Color(p);

                        // want this to be fresh each iter.
                        std::vector<cv::Point2f> charucoCorners;
                        std::vector<int> charucoIds;

                        cv::aruco::charucoLocal::interpolateCornersCharucoHomographyLocal(corners, ids, images[i],
                                P_class->boards[p], charucoCorners, charucoIds);

                        // check for degenerate configs ....
                        collinear_markers = cv::aruco::charucoLocal::testCharucoCornersCollinear(P_class->boards[p],
                                charucoIds);

                        if (charucoIds.size() > 0 && collinear_markers){
                            for (int j = 0, jn = charucoIds.size(); j < jn; j++){
                                circle(images[i], charucoCorners[j], 10, b_color, 2);
                            }
                        }

                        // if at least one charuco corner detected
                        if((charucoIds.size() > 0) && (collinear_markers == false))
                        {
                            if (i > number_external_images_max){
                                internal_found_counter++;
                            }

                            /// corners (2d) are linked to the Ids
                            cv::aruco::drawDetectedCornersCharuco(images[i], charucoCorners, charucoIds, b_color);

                            for (int j  = 0, jn = charucoIds.size(); j < jn; j++){
                                global_index = P_class->min_max_id_squares[p].first + charucoIds[j];

                                points_present[i][global_index] = true;
                                two_d_point_coordinates_dense[i](global_index, 0) = charucoCorners[j].x;
                                two_d_point_coordinates_dense[i](global_index, 1) = charucoCorners[j].y;

                            }
                        }	else {
                            boards_detected[i][p] = false;  // sometimes there's a marker detected, but we can't grab good corners.
                        }
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

        for (int j = 0; j < number_squares; j++){
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
            out << 	PointsAtImagePattern(i, p) << " " ;
        }
        out << endl;
    }

    out.close();
}




void CameraCali::ReadCorners(const string& read_dir){

    string filename = read_dir + "points.txt";

    ifstream in;
    in.open(filename.c_str());

    if(!in.good()) {
        cout << "Cannot read in camera calibration file " << filename << endl;
        cout << "And the argument specified read camera calibration parameters. Exiting " << endl;
        exit(1);
    }

    int number_patterns = P_class->NumberPatterns();
    int number_squares = P_class->NumberSquares();
    int number_images = images.size();
    Mat imageCopy;
    Scalar b_color;
    MatrixXd twod(number_squares, 2);
    twod.setConstant(0);
    int global_index;
    float x, y;

    boards_detected.resize(number_images, vector<bool>(P_class->NumberPatterns(), false));

    string temp_string0;
    string temp_string1;
    string temp_string2;

    string s_end = "end";

    bool in_image = false;

    // output something about the images channel.
    for (int i = 0; i < number_images; i++){

        in >> temp_string0 >> temp_string1;

        // initialize per image.
        points_present.push_back(vector<bool>(number_squares, false));
        two_d_point_coordinates_dense.push_back(twod);

        // while we don't have 'end' as the first of a triple ....
        in_image = true;
        do {
            in >> temp_string0;

            // is this end?
            if (temp_string0.compare(s_end) == 0){
                in_image = false;
            }	else {
                in >> x >> y;

                // convert to int, double, double.
                global_index = FromString<int>(temp_string0);
                points_present[i][global_index] = true;
                two_d_point_coordinates_dense[i](global_index, 0) = x;
                two_d_point_coordinates_dense[i](global_index, 1) = y;
            }
        } while (in_image == true);

        in >> temp_string1; // reads in the number of image.

    }

    // make it easy for self ... fill in everything computed in this function.
    in >> temp_string0 >> temp_string1 >> temp_string2;

    for (int i = 0; i < number_images; i++){

        in >> temp_string0 >> temp_string1 >> temp_string2;

        for (int p = 0; p < number_patterns; p++){
            in >> temp_string0;

            int temp_number = FromString<int>(temp_string0);

            boards_detected[i][p] = temp_number > 0;
        }
    }

    in.close();

}

int ReturnOption(string entry, vector<string>& comps){

    for (int i = 0, in = comps.size(); i < in; i++){
        if (entry.compare(comps[i]) == 0){
            return i;
        }
    }

    return -1;
}

void CameraCali::ReadCalibration(const string& read_dir){

    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

    string filename = read_dir + "cali_results.txt";
    std::ifstream in;
    in.open(filename.c_str());


    int number_images = images.size();
    int number_patterns = P_class->NumberPatterns();



    Matrix4d I;  I.setIdentity();

    // initialize
    vector<Matrix4d> patterns_base(number_patterns, I);
    // whether or not the board is present tells us whether to look at the value there.
    external_parameters.resize(number_images, patterns_base);
    vector<double> temp_repro(number_patterns, 0);
    double reproj_read;
    reproj_error_per_board.resize(number_images, temp_repro);

    string temp_string0, temp_string1, temp_string2;
    int number_points_wo_blanks;
    int dist_rows;
    int image_index, pattern_index;
    in >> temp_string0 >> number_points_wo_blanks;

    in >> temp_string0 >> temp_string1;

    in >> temp_string0;

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            in >> internal_parameters(i, j);
        }
    }

    in >> temp_string0 >> dist_rows;
    in >> temp_string0;
    distortion.resize(dist_rows);
    distCoeffs = cv::Mat::zeros(dist_rows, 1, CV_64F);

    for (int i = 0; i < dist_rows; i++){
        in >> distortion(i);

    }


    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            cameraMatrix.at<double>(i, j) = internal_parameters(i, j);
        }
    }

    for (int i = 0; i < distCoeffs.rows; i++){
        distCoeffs.at<double>(i, 0) = distortion(i);

    }

    for (int i = 0; i < number_points_wo_blanks; i++){

        in >> temp_string0 >> temp_string1 >> temp_string2 >> temp_string0 >> image_index >> pattern_index >> reproj_read;
        cout << temp_string0 << temp_string1 << temp_string2 << temp_string0 << image_index << pattern_index << reproj_read << endl;

        reproj_error_per_board.at(image_index).at(pattern_index) = reproj_read;
        // read matrix.
        for (int i = 0; i < 4; i++){
            for (int j = 0; j < 4; j++){
                in >> external_parameters[image_index][pattern_index](i, j);
            }
        }
    }

    has_calibration_estimate.resize(number_images, vector<bool>(number_patterns, false));
    in >> temp_string0;

    cout << "should be has-calibration-estimate  " << temp_string0 << endl;

    for (int i = 0; i < number_images; i++){
        for (int j = 0; j < number_patterns; j++){

            //in >> read_int;
            in >> temp_string0;
            cout << temp_string0 << " ";
            has_calibration_estimate[i][j] = FromString<int>(temp_string0);
        }
        cout << endl;
    }

    for (int i = 0; i < number_images; i++){
        for (int j = 0; j < number_patterns; j++){
            cout << has_calibration_estimate[i][j] << " ";
        }
        cout << endl;
    }


    in.close();

    cout << "Read in the calibration file -- Success!" << endl << internal_parameters << endl;
}

void CameraCali::CalibrateBasic(float initial_focal_px, int zero_tangent_dist,
        int zero_k3, int fix_principal_point, const string& write_dir, int number_points_needed_to_count,
        bool write_internal_images){


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

                    /// each board is a new observation, as is each image.  Get to finer resolution-figuring or transformations later if there are two on the same image.
                    twod_points_wo_blanks.push_back(vector< cv::Point2f>(points_per_board[i][p]));
                    threed_points_wo_blanks.push_back(vector< cv::Point3f>(points_per_board[i][p]));

                    /// then, walk through all of the possibles ONLY AT THIS PATTERN/BOARD.
                    s = 0;
                    last_added = twod_points_wo_blanks.size();
                    last_added--;

                    for (int j = P_class->min_max_id_squares[p].first; j <= P_class->min_max_id_squares[p].second; j++){
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
    /// need to prep the matrix of rotations ...
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

#pragma omp parallel for private(filename, rview)
    for (int i = 0; i < number_to_write; i++){
#pragma omp critical
        {
            if (i% 10 == 0){
                cout << "Writing external " << i << endl;
            }
        }
        cv::remap(reproject_cam_cali_images[i], rview, map1, map2, cv::INTER_LINEAR);

        filename  = write_dir + "/ext" + ToString<int>(i) + ".png";
        cv::imwrite(filename.c_str(), rview);
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
        image_index = mapping_from_limited_to_full_images[stre];
        pattern_index = mapping_from_limited_to_full_patterns[stre];

        out << "EXTERNAL image, pattern, reproj " << image_index << " " << pattern_index << " " << reproj_error_per_board[image_index][pattern_index] << endl;
        out << external_parameters[image_index][pattern_index] << endl;
    }

    out << "has-calibration-estimate " << endl;
    for (int i = 0; i < number_images; i++){
        for (int j = 0; j < number_patterns; j++){
            out << has_calibration_estimate[i][j] << " ";
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

    filename = write_dir + "three_d_data.txt";
    out.open(filename.c_str());
    for (int m = 0; m < int(twod_points_wo_blanks.size()); m++){
        out << "New-board " << twod_points_wo_blanks[m].size() << endl;
        for (int s = 0; s < int(twod_points_wo_blanks[m].size()); s++){
            out << threed_points_wo_blanks[m][s].x << " " << threed_points_wo_blanks[m][s].y  << " " << threed_points_wo_blanks[m][s].z << endl;
        }
    }
    out.close();


    images.clear(); /// we don't need to keep this hanging about .... too much memory.
}

void CameraCali::CalibrateRotatingSet(const string& write_dir, int number_points_needed_to_count){


    /// want to recover pose after calibration ...need a map.
    vector<int> mapping_from_limited_to_full_images;
    vector<int> mapping_from_limited_to_full_patterns;

    // create a collection of the points for each image -- hopefully this will work. map -- .
    vector< vector< cv::Point2f> > twod_points_wo_blanks;
    vector< vector< cv::Point3f> > threed_points_wo_blanks;

    int number_images = images.size();
    int number_patterns = P_class->NumberPatterns();

    vector<vector<int> > points_per_board;

    for (int i = 0; i < number_images; i++){
        points_per_board.push_back(vector<int>(number_patterns, 3));
        for (int j = 0; j < number_patterns; j++){
            points_per_board[i][j] = PointsAtImagePattern(i, j);
        }
    }

    int s = 0;
    cv::Size image_size;
    int last_added = 0;
    has_calibration_estimate.resize(number_images, vector<bool>(number_patterns, false));

    int rows = 0; int cols;
    for (int i = 0; i < number_images; i++){
        cout << "i "  << i << endl;
        rows = images[i].rows;
        cols = images[i].cols;
        for (int p = 0; p < number_patterns; p++){
            if (points_per_board[i][p] >= number_points_needed_to_count){

                has_calibration_estimate[i][p] = true;
                //cout << "Points per " << points_per_board[i][p] << endl;

                mapping_from_limited_to_full_images.push_back(i);
                mapping_from_limited_to_full_patterns.push_back(p);

                twod_points_wo_blanks.push_back(vector< cv::Point2f>(points_per_board[i][p]));
                threed_points_wo_blanks.push_back(vector< cv::Point3f>(points_per_board[i][p]));

                /// then, walk through all of the possibles ONLY AT THIS PATTERN/BOARD.
                s = 0;
                last_added = twod_points_wo_blanks.size();
                last_added--;
                for (int j = P_class->min_max_id_squares[p].first; j <= P_class->min_max_id_squares[p].second; j++){
                    if (points_present[i].at(j) == true){
                        //cout << "And added " << endl;
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


    // create separate structure for the internal items ....
    vector<vector< cv::Point2f> > twod_points_wo_blanks_internal;
    vector< vector< cv::Point3f> > threed_points_wo_blanks_internal;

    twod_points_wo_blanks_internal.push_back(vector< cv::Point2f>(count_internal_ids_present*4));
    threed_points_wo_blanks_internal.push_back(vector< cv::Point3f>(count_internal_ids_present*4));

    cout << "Filled in ... " << count_internal_ids_present*4 << endl;

    last_added = 0;
    s = 0;
    for (int i = 0; i < P_class->max_internal_patterns; i++){
        if (id_bool[i] == true){
            for (int j =0 ; j < 4; j++){
                twod_points_wo_blanks_internal[last_added][s].x = internal_two_d_point_coordinates_dense[0](i*4 + j, 0);
                twod_points_wo_blanks_internal[last_added][s].y = internal_two_d_point_coordinates_dense[0](i*4 + j, 1);

                threed_points_wo_blanks_internal[last_added][s].x = P_class->three_d_points_internal[i*4 + j].x;
                threed_points_wo_blanks_internal[last_added][s].y = P_class->three_d_points_internal[i*4 + j].y;
                threed_points_wo_blanks_internal[last_added][s].z = P_class->three_d_points_internal[i*4 + j].z;
                s++;
            }
        }
    }

    image_size = Size(cols, rows);
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    vector<cv::Mat> rvecs, tvecs;
    vector<cv::Mat> rvecs_internal, tvecs_internal;

    cameraMatrix.at<double>(0, 0) = pixel_width;
    cameraMatrix.at<double>(1, 1) = pixel_width;;

    cameraMatrix.at<double>(0, 2) = cols/2;
    cameraMatrix.at<double>(1, 2) = rows/2;

    cout << "Running calibration " << endl;

    double rms = 0;


    rms = calibrateCamera(threed_points_wo_blanks_internal, twod_points_wo_blanks_internal, image_size, cameraMatrix, distCoeffs, rvecs_internal, tvecs_internal,
            CALIB_USE_INTRINSIC_GUESS| CALIB_ZERO_TANGENT_DIST| CALIB_FIX_PRINCIPAL_POINT | CALIB_FIX_K3 | CALIB_FIX_FOCAL_LENGTH );

    cout << "rms " << rms << endl;

    int number_from_external = twod_points_wo_blanks.size();

    cout << "Before external" << endl;
    for (int i = 0; i < number_from_external; i++){
        cv::Mat rv; cv::Mat tv;
        solvePnP(threed_points_wo_blanks[i], twod_points_wo_blanks[i], cameraMatrix, distCoeffs, rv, tv, false);

        rvecs.push_back(rv);
        tvecs.push_back(tv);
    }

    cout << "After external" << endl;

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

    /// internal only
    int number_cali_items = twod_points_wo_blanks.size();
    {

        cv::projectPoints( cv::Mat(threed_points_wo_blanks_internal[0]), rvecs_internal[0], tvecs_internal[0], cameraMatrix,  // project
                distCoeffs, imagePoints2);
        err = cv::norm(cv::Mat(twod_points_wo_blanks_internal[0]), cv::Mat(imagePoints2), CV_L2);              // difference
        reproj_error        += err*err;

        for (int j = 0, jn = imagePoints2.size(); j < jn; j++){
            line(reproject_cam_cali_images[0], twod_points_wo_blanks_internal[0][j],imagePoints2[j], Scalar(255, 0, 255), 6 );
        }

    }

    vector<double> temp_repro(number_patterns, 0);
    reproj_error_per_board.resize(number_images, temp_repro);

    /// EXTERNAL
    for (int m = 0; m < number_cali_items; m++){

        cv::projectPoints( cv::Mat(threed_points_wo_blanks[m]), rvecs[m], tvecs[m], cameraMatrix,  // project
                distCoeffs, imagePoints2);
        err = cv::norm(cv::Mat(twod_points_wo_blanks[m]), cv::Mat(imagePoints2), CV_L2);              // difference
        reproj_error        += err*err;

        correct_image = mapping_from_limited_to_full_images[m];
        correct_pattern = mapping_from_limited_to_full_patterns[m];
        reproj_error_per_board[correct_image][correct_pattern] = err*err;

        for (int j = 0, jn = imagePoints2.size(); j < jn; j++){
            line(reproject_cam_cali_images[correct_image], twod_points_wo_blanks[m][j],imagePoints2[j], Scalar(255, 0, 255), 10 );
        }

    }

    ////////////////////// External -- write into class variables ///////////////////////////////////////////
    /// need to prep the matrix of rotations ...
    Matrix4d I;  I.setIdentity();

    // initialize
    vector<Matrix4d> patterns_base(number_patterns, I);

    // whether or not the board is present tells us whether to look at the value there.
    external_parameters.resize(number_images, patterns_base);

    int image_index;
    int pattern_index;

    /// convert from the calibration to saved values.
    for (int stre = 0, stre_total = number_cali_items; stre < stre_total; stre++){

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


    //	/////////////////////////////// UNDISTORT, WRITE REPROJECTION ////////////////////////////////////
    cv::Mat view, rview, map1, map2;
    Mat im_small;
    Size si;
    string filename;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
            cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, image_size, 1, image_size, 0),
            image_size, CV_16SC2, map1, map2);

#pragma omp parallel for private(filename, rview, im_small, si)
    for (int i = 0; i < number_images; i++){
        if (i % 10 == 0){
#pragma omp critical
            {
                cout << "Writing external " << i << endl;
            }
        }
        cv::remap(reproject_cam_cali_images[i], rview, map1, map2, cv::INTER_LINEAR);

        {
            filename  = write_dir + "/ext" + ToString<int>(i) + ".jpg";
            cv::imwrite(filename.c_str(), rview);

            si.width = rview.cols/8;
            si.height = rview.rows/8;

            resize(rview, im_small, si);

            imwrite(filename.c_str(), im_small);
        }
    }


    cout << "internal parameters in function : " << internal_parameters << endl;
    ///////////////////////////////// WRITE CALIBRATION INFORMATION ///////////////////////////
    std::ofstream out;
    filename = write_dir + "cali_results.txt";
    out.open(filename.c_str());

    out << "Number_patterns " << number_cali_items << endl;
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


    for (int stre = 0; stre < number_cali_items; stre++){
        image_index = mapping_from_limited_to_full_images[stre];
        pattern_index = mapping_from_limited_to_full_patterns[stre];

        out << "EXTERNAL image, pattern, reproj " << image_index << " " << pattern_index << " " << reproj_error_per_board[image_index][pattern_index] << endl;
        out << external_parameters[image_index][pattern_index] << endl;
    }

    out << "has-calibration-estimate " << endl;
    for (int i = 0; i < number_images; i++){
        for (int j = 0; j < number_patterns; j++){
            out << has_calibration_estimate[i][j] << " ";
        }
        out << endl;
    }
    out.close();

    filename = write_dir + "two_d_data.txt";
    out.open(filename.c_str());
    for (int m = 0; m < number_cali_items; m++){
        out << "New-board " << twod_points_wo_blanks[m].size() << endl;
        for (int s = 0; s < int(twod_points_wo_blanks[m].size()); s++){
            out << twod_points_wo_blanks[m][s].x << " " << twod_points_wo_blanks[m][s].y  << endl;
        }
    }
    out.close();

    filename = write_dir + "three_d_data.txt";
    out.open(filename.c_str());
    for (int m = 0; m < number_cali_items; m++){
        out << "New-board " << twod_points_wo_blanks[m].size() << endl;
        for (int s = 0; s < int(twod_points_wo_blanks[m].size()); s++){
            out << threed_points_wo_blanks[m][s].x << " " << threed_points_wo_blanks[m][s].y  << " " << threed_points_wo_blanks[m][s].z << endl;
        }
    }
    out.close();


    filename = write_dir + "image_points_first_image.txt";
    out.open(filename.c_str());

    /// external points.
    for (int i = 0; i < 1; i++){
        cout << "i "  << i << endl;

        for (int p = 0; p < number_patterns; p++){
            if (points_per_board[i][p] == 4){
                cout << "Image, Pattern " << i << ", " << p << endl;

                out << "EXTERNAL " << endl;

                for (int j = 0; j < 4; j++){
                    out << two_d_point_coordinates_dense[i](p*4 + j, 0) << " " << two_d_point_coordinates_dense[i](p*4 + j, 1) << endl;
                }
            }
        }
    }

    for (int i = 0; i < P_class->max_internal_patterns; i++){
        if (id_bool[i] == true){
            out << "INTERNAL " << i << endl;
            for (int j =0 ; j < 4; j++){
                out << internal_two_d_point_coordinates_dense[0](i*4 + j, 0) << " " << internal_two_d_point_coordinates_dense[0](i*4 + j, 1) << endl;
            }
        }
    }

    out << -1 << endl;
    out.close();

    images.clear();
}

int CameraCali::PointsAtImagePattern(int image_number, int pattern_number){
    int number_points = 0;
    for (int j = P_class->min_max_id_squares[pattern_number].first; j <= P_class->min_max_id_squares[pattern_number].second; j++){
        if (points_present[image_number].at(j) == true){
            number_points++;
        }
    }

    return number_points;
}

void CameraCali::SetUpSelectPointsForMinimization(){

    points_used_min = points_present;  //then points used min will be altered in the multicamera.cpp files.
}

int CameraCali::PointsForMinimization(int image_number, int pattern_number){
    int number_points = 0;
    for (int j = P_class->min_max_id_squares[pattern_number].first; j <= P_class->min_max_id_squares[pattern_number].second; j++){
        if (points_used_min[image_number].at(j) == true){
            number_points++;
        }
    }

    return number_points;
}

double CameraCali::ComputeReprojectionErrorOneImagePattern(const Matrix4d& ExtParameters, int image_number,
        int pattern_number,
        const string& write_directory, bool write, int equation_number, bool rotating,
        const Matrix3d* IntParameters){

    bool write_now = false;


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

    for (int j = P_class->min_max_id_squares[pattern_number].first; j <= P_class->min_max_id_squares[pattern_number].second; j++){
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


double CameraCali::ComputeReprojectionErrorOneImagePatternGTDRel(const Matrix4d& ExtParameters, int image_number,
        int pattern_number,
        const string& write_directory, bool write, int equation_number, bool rotating, const vector<Vector3d>& points_from_gtd){

    bool write_now = false;


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

    for (int j = P_class->min_max_id_squares[pattern_number].first; j <= P_class->min_max_id_squares[pattern_number].second; j++){
        if (points_present[image_number].at(j) == true){
            twod_points_wo_blanks[s].x = two_d_point_coordinates_dense[image_number](j, 0);    /// twod points w/o blanks is NOT per image to make internal cali work.
            twod_points_wo_blanks[s].y = two_d_point_coordinates_dense[image_number](j, 1);

            threed_points_wo_blanks[s].x = points_from_gtd[j](0);
            threed_points_wo_blanks[s].y = points_from_gtd[j](1);
            threed_points_wo_blanks[s].z = points_from_gtd[j](2);

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
    for (int r = 0; r < 3; r++){
        for (int c = 0; c < 3; c++){
            rotMatrix.at<double>(r, c) = ExtParameters(r, c);
            cameraMatrix.at<double>(r, c) = internal_parameters(r, c);
        }
        tvec.at<double>(r, 0) = ExtParameters(r, 3);
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
            cout << "Resize!!!!" << endl;

        }	else {
            filename = filename + ".png";
            imwrite(filename.c_str(), im_copy);
        }
    }

    return reproj_error;
}

//============================================================================
// Name        : compute-error.cpp
// Author      : Amy Tabb
// Version     :
// Copyright   : MIT
// Description :
//============================================================================

#include "Includes.hpp"

#include "helper.hpp"
#include "DirectoryFunctions.hpp"
#include <Eigen/Geometry>
#include "camera-visualization.hpp"

void ReadAndComputeErrorKalibr2(string input_dir, string output_dir, string comparison_file, string numCamerasFile, double camera_size);

void ReadAndComputeErrorCalico(string input_dir, string output_dir, string comparison_file, double camera_size);

int main(int argc, char **argv){

    int print_help = 0;
    int kalibr = 0;
    int calico = 0;
    int doProfile = 0;

    string input_dir = "";
    string output_dir = "";
    string comparison_file = ""; // this will be a text file.
    string numCamerasFile = ""; //required for kalibr

    if (argc == 1){
        print_help = 1;
    }

    while (1)
    {
        static struct option long_options[] =
        {
                {"help",   no_argument,       &print_help, 1},
                {"kalibr", no_argument,       &kalibr, 1},
                {"calico", no_argument,       &calico, 1},
                {"profiling", no_argument, &doProfile, 1},
                /* These options don’t set a flag.
	             We distinguish them by their indices. */
                {"input",   required_argument, 0, 'a'},
                {"output",   required_argument, 0, 'b'},
                {"comparison-file",  required_argument, 0, 'c'},
                {"num-cameras-file", required_argument, 0, 'd'},
        };


        if (print_help == 1){
            // todo
            cout << "Printing help for TODO."<< endl;
            // The comparison file for Kalibr will be the camchain-*bagfile name*-.yaml

            // The comparison file for calico will be the camera_cali_incremental.txt
            exit(1);
        }


        // The comparison file for Kalibr will be the camchain-*bagfile name*-.yaml
        /* getopt_long stores the option index here. */
        int option_index = 0;
        int opt_argument;

        opt_argument = getopt_long (argc, argv, "abcd",
                long_options, &option_index);

        /* Detect the end of the options. */
        if (opt_argument == -1)
            break;

        switch (opt_argument)
        {
        case 0:
            if (long_options[option_index].flag != 0)
                break;
            printf ("option %s", long_options[option_index].name);
            if (optarg)
                printf (" with arg %s", optarg);
            printf ("\n");
            break;

        case 'a':
            input_dir = optarg;
            break;
        case 'b':
            output_dir = optarg;
            break;
        case 'c':
            comparison_file = optarg;
            break;
        case 'd':
            numCamerasFile = optarg;
            break;
        default:{
            cout << "Argument not found " << optarg << endl;
            exit(1);
        }

        }

    }





    bool dirExists = CheckExistenceOfDirectory(input_dir);
    if (dirExists == false){
        cout << "input_dir does not exist, quitting. " << input_dir << endl;
        exit(1);
    }
    dirExists = CheckExistenceOfDirectory(output_dir);
    if (dirExists == false){
        cout << "output_dir does not exist, quitting. " << output_dir << endl;
        exit(1);
    }


    ifstream in;
    in.open(comparison_file.c_str());
    TestStream(in, comparison_file);
    in.close();

    EnsureDirHasTrailingBackslash(input_dir);
    EnsureDirHasTrailingBackslash(output_dir);

    string compDir = output_dir + "comparison-ground-truth/";
    mkdir(compDir.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

    ofstream out;
    string filename;
    if (kalibr == 1){
        filename = compDir + "arguments-compute-error-kalibr.txt";
    }   else {
        filename = compDir + "arguments-compute-error-calico.txt";
    }

    out.open(filename.c_str());
    out << "arguments: " << endl << argv[0] << " ";

    // need to have a gt-versus-kalibr setting.
    if (kalibr == 1){
        out << "--kalibr \\" << endl;
    }

    if (calico == 1){
        out << "--calico \\" << endl;
    }


    out << " --input " << input_dir << "\\" << endl;
    out << " --output " << output_dir <<  "\\" << endl;
    out << " --comparison-file " << comparison_file;
    if (numCamerasFile.size() > 0){
        out <<  "\\" << endl << " --num-cameras-file " << numCamerasFile << endl;
    }      else {
        out << endl;
    }


    out.close();

    if (kalibr == true){

        cout << "Inner Kalibr loop " << __LINE__ << " is kalibr true? " << kalibr << endl;
        if (numCamerasFile.size() == 0){
            cout << "you need to specify a text file for the number of cameras with the kalibr option. Killing." << endl;
            exit(1);
        }

        // this is really gt-versus-kalibr.
        ReadAndComputeErrorKalibr2( input_dir, compDir, comparison_file, numCamerasFile, 80);
    }

    if (calico == true){

        ReadAndComputeErrorCalico( input_dir, compDir, comparison_file, 80);

    }


    return 0;



}


void ReadAndComputeErrorKalibr2(string input_dir, string output_dir, string comparison_file, string numCamerasFile, double camera_size){


    // camera transformations has the number of cameras.
    string ground_truth_filename = input_dir + "CameraTransformationsOpenGL.txt";

    string filename;

    //    // read file.
    ifstream in;
    in.open(numCamerasFile.c_str());
    TestStream(in, numCamerasFile);

    int number_cameras = 0;

    in >> number_cameras;

    cout << "Number of cameras " << number_cameras << endl;
    in.close();

    in.open(ground_truth_filename.c_str());
    TestStream(in, ground_truth_filename);

    string temp_string;

    Matrix3d internal;
    Matrix4d external;

    internal.setIdentity();
    external.setIdentity();

    vector<Matrix3d> internals_gt(number_cameras);
    vector<Matrix4d> externals_gt(number_cameras);
    vector<int> rows_cols(number_cameras*2, 0);

    vector<Matrix3d> internals_kr(number_cameras);
    vector<Vector4d> distortion_coefficients(number_cameras);
    vector<Matrix4d> externals_kr(number_cameras);
    vector<Vector3d> centers_gt_relative(number_cameras);
    vector<Vector3d> centers_kr(number_cameras);

    for (int i = 0; i < number_cameras; i++){
        in >> temp_string;

        //        in >> temp_string;  in >> rows_cols[i*2];
        //        in >> temp_string;  in >> rows_cols[i*2 + 1];

        for (int r = 0; r < 3; r++){
            for (int c = 0; c < 3; c++){
                in >> internal(r, c);
            }
        }

        for (int r = 0; r < 4; r++){
            for (int c = 0; c < 4; c++){
                in >> external(r, c);
            }
        }

        rows_cols[i*2] = internal(1,2)*2;
        rows_cols[i*2 + 1] = internal(0, 2)*2;


        internals_gt[i] = internal;
        externals_gt[i] = external;
    }

    in.close();

    vector<Matrix4d> externals_gt_relative(number_cameras);


    string gtRelativeFile = output_dir + "ground-truth-relative-cameras.txt";

    ofstream gtOut;
    gtOut.open(gtRelativeFile.c_str());




    // will need to check this.  visualize.  ok, this works.
    for (int i = 0; i < number_cameras; i++){
        externals_gt_relative[i] = externals_gt[i]*externals_gt[0].inverse();

        cout << "GT relative " << i << endl;

        cout << externals_gt_relative[i] << endl;

        centers_gt_relative[i] = ReturnCenter(externals_gt_relative[i]);
        gtOut << "GT-rel " << i << " rows, cols " << rows_cols[i*2] << ", " << rows_cols[i*2 + 1] << endl;
        gtOut << internals_gt[i] << endl;
        gtOut << externals_gt_relative[i] << endl;
    }
    gtOut.close();

    string ground_truth_relative_dir = output_dir + "ground-truth-cameras-relative/";

    mkdir(ground_truth_relative_dir.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

    Vector3d cam_color_gt;
    cam_color_gt.setZero();
    cam_color_gt(0) = 255;

    Vector3d cam_color_kr;
    cam_color_kr.setZero();
    cam_color_kr(2) = 255;



    /// todo look at this

    string writeErrorFile = output_dir + "KalibrErrorFile.txt";

    cout << "write file " << writeErrorFile << endl;
    ofstream writeError;
    writeError.open(writeErrorFile.c_str());


    for (int i = 0; i < number_cameras; i++){

        filename = ground_truth_relative_dir + "/c"+ ToString<int>(i) + ".ply";

        create_camera(internals_gt[i], externals_gt_relative[i], camera_size, cam_color_gt(0), cam_color_gt(1),
                cam_color_gt(2), rows_cols[i*2], rows_cols[i*2 + 1], filename);

    }

    filename = ground_truth_relative_dir + "/all-gt.ply";

    if (number_cameras > 0){
        create_cameras(internals_gt, externals_gt_relative, cam_color_gt(0), cam_color_gt(1),
                cam_color_gt(2), rows_cols[0], rows_cols[1], filename, camera_size);
    }


    bool token_found = false;
    string camera_name = "";

    Vector4d distortion;
    string dist = "distortion_coeffs:";
    string intrin = "intrinsics:";
    string base_extrin = "T_cn_cnm";
    string extrin_string;
    string num_string;

    internal.setIdentity();
    vector<bool> alreadyFound(number_cameras, false);
    // question is if everytime the cameras are computed relative to camera 0.
    //for (int i = 0; i < 1; i++){

    for (int camIndex = 0; camIndex < number_cameras; camIndex++){

        in.open(comparison_file.c_str());

        // walk through the whole file until we find it.
        {
            //for (int i = 0; i < 1; i++){
            // advance until you find the camera number.
            camera_name = "cam" + ToString<int>(camIndex) + ":";

            token_found = false;

            while (token_found == false && in.good()){
                in >> temp_string;
                if (temp_string.compare(camera_name) == 0){
                    token_found = true;
                }
            }

            // after finding the camera name, if camera number is, find next the distortion coeficients.
            if (camIndex > 0){
                //extrin_string = base_extrin + ToString<int>(i) + ":";
                extrin_string = base_extrin + "1:";

                token_found = false;
                while (token_found == false && in.good()){
                    in >> temp_string;
                    if (temp_string.compare(extrin_string) == 0){
                        token_found = true;
                        //cout << "found string" << endl;
                    }
                }

                for (int r = 0; r < 4; r++){
                    in >> temp_string; // dash.
                    //cout << "Should be dash " << temp_string << endl;

                    for (int j = 0; j < 4; j++){
                        in >> temp_string;
                        //cout << "loop, temp string |" << temp_string << " " << j << endl;

                        if (j == 0){
                            num_string = temp_string.substr(1, temp_string.size() - 2);

                        }else {
                            num_string = temp_string.substr(0, temp_string.size() -1);
                        }

                        cout << "External num_string " << num_string << endl;

                        external(r, j) = FromString<double>(num_string);

                        if (j == 3 && r != 3){
                            external(r, j) *= 1000; // convert to mm
                        }

                    }
                }
                externals_kr[camIndex] = external;
                cout << "external found " << camIndex << endl << externals_kr[camIndex] << endl;

                writeError << "externals for " << camIndex << endl << externals_kr[camIndex] << endl;


            }   else {
                external.setIdentity();
                externals_kr[camIndex] = external;
            }

            ////////////////// distortion
            token_found = false;
            while (token_found == false && in.good()){
                in >> temp_string;
                if (temp_string.compare(dist) == 0){
                    token_found = true;
                }
            }

            //cout << "Line " << __LINE__ << endl;

            for (int j = 0; j < 4; j++){
                in >> temp_string;

                if (j == 0){
                    num_string = temp_string.substr(1, temp_string.size() - 2);

                }else {
                    num_string = temp_string.substr(0, temp_string.size() -1);
                }

                //cout << "num_string " << num_string << endl;

                distortion(j) = FromString<double>(num_string);

            }

            //cout << "distortion " << distortion << endl;

            distortion_coefficients[camIndex] = distortion;

            token_found = false;
            while (token_found == false){
                in >> temp_string;
                if (temp_string.compare(intrin) == 0){
                    token_found = true;
                }
            }

            for (int j = 0; j < 4; j++){
                in >> temp_string;

                if (j == 0){
                    num_string = temp_string.substr(1, temp_string.size() - 2);

                }else {
                    num_string = temp_string.substr(0, temp_string.size() -1);
                }

                //cout << "num_string " << num_string << endl;

                switch(j){
                case 0:{
                    internal(0,0) = FromString<double>(num_string);
                } break;
                case 1:{
                    internal(1,1) = FromString<double>(num_string);
                } break;
                case 2: {
                    internal(0,2) = FromString<double>(num_string);
                } break;
                case 3: {
                    internal(1,2) = FromString<double>(num_string);
                }
                }
            }

            internals_kr[camIndex] = internal;

            writeError << "internal for " << camIndex << endl << internals_kr[camIndex] << endl;

            // cout << "Line " << __LINE__ << endl;

            //cout << "internals " << endl << internal << endl;


            // if (token.compare(fieldTag) == 0){

            centers_kr[camIndex] = ReturnCenter(externals_kr[camIndex]);

            alreadyFound[camIndex] = true;



            //cout << "Line " << __LINE__ << endl;

        }
        in.close();
    }

    //in.close();

    //cout << "Line " << __LINE__ << endl;

    string kalibr_dir = output_dir + "kalibr-cameras-relative/";

    mkdir(kalibr_dir.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

    string kalibrRelativeFile = output_dir + "kalibr-relative-cameras.txt";

    ofstream kaOut;
    kaOut.open(kalibrRelativeFile .c_str());

    for (int i = 0; i < number_cameras; i++){

        filename = kalibr_dir + "/c"+ ToString<int>(i) + ".ply";

        create_camera(internals_kr[i], externals_kr[i], camera_size, cam_color_kr(0), cam_color_kr(1),
                cam_color_kr(2), rows_cols[i*2], rows_cols[i*2 + 1], filename);

        kaOut << "kalibr-rel " << i << " rows, cols " << rows_cols[i*2] << ", " << rows_cols[i*2 + 1] << endl;
        kaOut << internals_kr[i] << endl;
        kaOut << externals_kr[i] << endl;

    }
    kaOut.close();

    filename = kalibr_dir + "/all-kalibr.ply";

    if (number_cameras > 0){
        create_cameras(internals_kr, externals_kr, cam_color_kr(0), cam_color_kr(1),
                cam_color_kr(2), rows_cols[0], rows_cols[1], filename, camera_size);
    }

    double translation_error = 0;

    Vector3d transA, transB;

    Matrix3d Ra, Rb, Rc;

    double degrees_per_rad = 180.0/M_PI;

    double rot_error_degrees = 0;

    for (int i = 0; i < number_cameras; i++){

        for (int j = 0; j < 3; j++){
            transA(j) = externals_gt_relative[i](j, 3);
            transB(j) = externals_kr[i](j, 3);

            for (int k = 0; k < 3; k++){
                Ra(j, k) = externals_gt_relative[i](j, k);
                Rb(j, k) = externals_kr[i](j, k);
            }
        }

        translation_error += (transA - transB).norm();

        Rc = Rb.transpose()*Ra;

        Eigen::AngleAxisd newAngleAxis(Rc);

        cout << "angle " << newAngleAxis.angle()*degrees_per_rad << endl;

        rot_error_degrees += newAngleAxis.angle()*degrees_per_rad;

    }

    cout << "translation error, average: " << translation_error/double(number_cameras - 1) << endl;

    cout << "rotation error, degrees, average: " << rot_error_degrees/double(number_cameras - 1) << endl;



    writeError << "translation error, average: " << translation_error/double(number_cameras - 1) << endl;

    writeError << "rotation error, degrees, average: " << rot_error_degrees/double(number_cameras - 1) << endl;

    writeError.close();


}


void ReadAndComputeErrorCalico(string input_dir, string output_dir, string comparison_file, double camera_size){

    string ground_truth_filename = input_dir + "CameraTransformationsOpenGL.txt";
    cout << "ground truth filename " << ground_truth_filename << endl;
    cout << "comparison filename " << comparison_file << endl;

    string filename;

    // first, read in the number of cameras from the comparison file, which has the number.  CameraTransformationsOpenGL does not.
    ifstream in;
    in.open(comparison_file.c_str());
    TestStream(in, comparison_file);

    int number_cameras = 0;

    in >> number_cameras;

    cout << "Number of cameras " << number_cameras << endl;

    in.close();

    //    // read file.
    in.open(ground_truth_filename.c_str());
    TestStream(in, ground_truth_filename);


    string temp_string;

    Matrix3d internal;
    Matrix4d external;

    internal.setIdentity();
    external.setIdentity();

    vector<Matrix3d> internals_gt(number_cameras);
    vector<Matrix4d> externals_gt(number_cameras);
    vector<int> rows_cols(number_cameras*2, 0);

    vector<Matrix3d> internals_kr(number_cameras);
    vector<Vector4d> distortion_coefficients(number_cameras);
    vector<Matrix4d> externals_kr(number_cameras);
    vector<Vector3d> centers_gt_relative(number_cameras);
    vector<Vector3d> centers_calico_relative(number_cameras);
    vector<Vector3d> centers_kr(number_cameras);


    /////////// read the simulation dataset
    for (int i = 0; i < number_cameras; i++){
        in >> temp_string;

        //in >> temp_string;  in >> rows_cols[i*2];
        //in >> temp_string;  in >> rows_cols[i*2 + 1];

        for (int r = 0; r < 3; r++){
            for (int c = 0; c < 3; c++){
                in >> internal(r, c);
            }
        }

        cout << "internal read " << i << endl << internal << endl;

        // here is the problem -- dependent on dataset creation knowledge.
        rows_cols[i*2] = internal(1,2)*2;
        rows_cols[i*2 + 1] = internal(0, 2)*2;


        for (int r = 0; r < 4; r++){
            for (int c = 0; c < 4; c++){
                in >> external(r, c);
            }
        }

        internals_gt[i] = internal;
        externals_gt[i] = external;
    }

    in.close();

    vector<Matrix4d> externals_gt_relative(number_cameras);


    //M = CameraTransformations[i]*Minv;

    // will need to check this.  visualize.  ok, this works.
    for (int i = 0; i < number_cameras; i++){
        externals_gt_relative[i] = externals_gt[i]*externals_gt[0].inverse();
        // todo take out if working.
        // testing
        // externals_gt_relative[i].setIdentity();

        cout << "GT relative " << i << endl;

        cout << externals_gt_relative[i] << endl;

        centers_gt_relative[i] = ReturnCenter(externals_gt_relative[i]);
    }

    string ground_truth_relative_dir = output_dir + "ground-truth-cameras-relative/";

    mkdir(ground_truth_relative_dir.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

    string gtRelativeFile = output_dir + "ground-truth-relative-cameras.txt";

    ofstream gtOut;
    gtOut.open(gtRelativeFile.c_str());

    Vector3d cam_color_gt;
    cam_color_gt.setZero();
    cam_color_gt(0) = 230;
    cam_color_gt(1) = 159;


    Vector3d cam_color_kr;
    cam_color_kr.setZero();
    cam_color_kr(1) = 158;
    cam_color_kr(2) = 115;

    for (int i = 0; i < number_cameras; i++){

        filename = ground_truth_relative_dir + "/gt-camera"+ ToString<int>(i) + ".ply";

        create_camera(internals_gt[i], externals_gt_relative[i], camera_size, cam_color_gt(0), cam_color_gt(1),
                cam_color_gt(2), rows_cols[i*2], rows_cols[i*2 + 1], filename);
        gtOut << "GT-rel " << i << " rows, cols " << rows_cols[i*2] << ", " << rows_cols[i*2 + 1] << endl;
        gtOut << internals_gt[i] << endl;
        gtOut << externals_gt_relative[i] << endl;

    }
    gtOut.close();

    filename = ground_truth_relative_dir + "/all-gt.ply";

    if (number_cameras > 0){
        create_cameras(internals_gt, externals_gt_relative, cam_color_gt(0), cam_color_gt(1),
                cam_color_gt(2), rows_cols[0], rows_cols[1], filename, camera_size);
    }

    //        filename = cam_dir + "/all.ply";
    //
    //        if (internals.size() > 0){
    //            create_cameras(internals, externals, cam_color(0), cam_color(1), cam_color(2), CCV[0]->rows, CCV[0]->cols, filename, camera_size);
    //        }


    /////////////////// calico cameras //////////////////////////////////////

    in.open(comparison_file.c_str());


    //FindValueOfFieldInFile(filename, fieldTag, separator, kill_if_not_found)
    //bool token_found = false;
    string camera_name = "";

    Vector4d distortion;
    string extrin_string;
    string num_string;

    internal.setIdentity();

    string tempString;
    in >> number_cameras;

    // question is if everytime the cameras are computed relative to camera 0.
    //for (int i = 0; i < 1; i++){
    for (int i = 0; i < number_cameras; i++){

        in >> tempString;


        // read in the internal matrix.
        internal.setIdentity();

        for (int r = 0; r < 3; r++){
            for (int c  = 0; c < 3; c++){
                in >> internal(r, c);
            }
        }

        internals_kr[i] = internal;

        external.setIdentity();

        for (int r = 0; r < 3; r++){
            for (int c  = 0; c < 3; c++){
                in >> external(r, c);
            }
        }

        for (int r = 0; r < 3; r++){
            in >> external(r, 3);
        }

        externals_kr[i] = external;

        for (int r = 0; r < 4; r++){
            in >> distortion(r);
        }

        in >> tempString;


        distortion_coefficients[i] = distortion;

        // if (token.compare(fieldTag) == 0){

        centers_kr[i] = ReturnCenter(externals_kr[i]);


        cout << "External " << endl << external << endl;
        //cout << "Line " << __LINE__ << endl;

    }

    in.close();


    vector<Matrix4d> externals_calico_relative(number_cameras);



    //M = CameraTransformations[i]*Minv;

    // will need to check this.  visualize.  ok, this works.
    for (int i = 0; i < number_cameras; i++){
        externals_calico_relative[i] = externals_kr[i]*externals_kr[0].inverse();



        cout << "GT relative " << i << endl;

        cout << externals_calico_relative[i] << endl;

        centers_calico_relative[i] = ReturnCenter(externals_calico_relative[i]);
    }




    //cout << "Line " << __LINE__ << endl;

    string calico_dir = output_dir + "calico-cameras-relative/";

    mkdir(calico_dir.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

    string calicoRelativeFile = output_dir + "calico-relative-cameras.txt";

    ofstream caOut;
    caOut.open(calicoRelativeFile .c_str());



    for (int i = 0; i < number_cameras; i++){

        filename = calico_dir + "/calico"+ ToString<int>(i) + ".ply";

        create_camera(internals_kr[i], externals_calico_relative[i], camera_size, cam_color_kr(0), cam_color_kr(1),
                cam_color_kr(2), rows_cols[i*2], rows_cols[i*2 + 1], filename);

        caOut << "calico-rel " << i << " rows, cols " << rows_cols[i*2] << ", " << rows_cols[i*2 + 1] << endl;
        caOut << internals_kr[i] << endl;
        caOut << externals_calico_relative[i] << endl;

    }
    caOut.close();

    filename = calico_dir + "/all-calico.ply";

    if (number_cameras > 0){
        create_cameras(internals_kr, externals_calico_relative, cam_color_kr(0), cam_color_kr(1),
                cam_color_kr(2), rows_cols[0], rows_cols[1], filename, camera_size);
    }



    double translation_error = 0;

    Vector3d transA, transB;

    Matrix3d Ra, Rb, Rc;

    double degrees_per_rad = 180.0/M_PI;

    double rot_error_degrees = 0;

    for (int i = 0; i < number_cameras; i++){

        for (int j = 0; j < 3; j++){
            transA(j) = externals_gt_relative[i](j, 3);
            transB(j) = externals_calico_relative[i](j, 3);

            for (int k = 0; k < 3; k++){
                Ra(j, k) = externals_gt_relative[i](j, k);
                Rb(j, k) = externals_calico_relative[i](j, k);
            }
        }

        translation_error += (transA - transB).norm();

        Rc = Rb.transpose()*Ra;

        Eigen::AngleAxisd newAngleAxis(Rc);

        cout << "angle " << newAngleAxis.angle()*degrees_per_rad << endl;

        rot_error_degrees += newAngleAxis.angle()*degrees_per_rad;

    }

    cout << "translation error, average: " << translation_error/double(number_cameras - 1) << endl;

    cout << "rotation error, degrees, average: " << rot_error_degrees/double(number_cameras - 1) << endl;

    string writeErrorFile = output_dir + "calicoErrorFile.txt";

    cout << "write file " << writeErrorFile << endl;
    ofstream writeError;
    writeError.open(writeErrorFile.c_str());

    writeError << "translation error, average: " << translation_error/double(number_cameras - 1) << endl;

    writeError << "rotation error, degrees, average: " << rot_error_degrees/double(number_cameras - 1) << endl;

    writeError.close();


}

//void ReadAndComputeErrorCalico(string input_dir, string output_dir, string comparison_file, double camera_size){
//
//    string ground_truth_filename = input_dir + "CameraTransformationsOpenGL.txt";
//
//    string filename;
//
//    //    // read file.
//    ifstream in;
//    in.open(ground_truth_filename.c_str());
//    TestStream(in, ground_truth_filename);
//
//    int number_cameras = 0;
//
//    in >> number_cameras;
//    string temp_string;
//
//    Matrix3d internal;
//    Matrix4d external;
//
//    internal.setIdentity();
//    external.setIdentity();
//
//    vector<Matrix3d> internals_gt(number_cameras);
//    vector<Matrix4d> externals_gt(number_cameras);
//    vector<int> rows_cols(number_cameras*2, 0);
//
//    vector<Matrix3d> internals_kr(number_cameras);
//    vector<Vector4d> distortion_coefficients(number_cameras);
//    vector<Matrix4d> externals_kr(number_cameras);
//    vector<Vector3d> centers_gt_relative(number_cameras);
//    vector<Vector3d> centers_calico_relative(number_cameras);
//    vector<Vector3d> centers_kr(number_cameras);
//
//
//    /////////// read the simulation dataset
//    for (int i = 0; i < number_cameras; i++){
//        in >> temp_string;
//
//        in >> temp_string;  in >> rows_cols[i*2];
//        in >> temp_string;  in >> rows_cols[i*2 + 1];
//
//        for (int r = 0; r < 3; r++){
//            for (int c = 0; c < 3; c++){
//                in >> internal(r, c);
//            }
//        }
//
//        for (int r = 0; r < 4; r++){
//            for (int c = 0; c < 4; c++){
//                in >> external(r, c);
//            }
//        }
//
//        internals_gt[i] = internal;
//        externals_gt[i] = external;
//    }
//
//    in.close();
//
//    vector<Matrix4d> externals_gt_relative(number_cameras);
//
//
//    //M = CameraTransformations[i]*Minv;
//
//    // will need to check this.  visualize.  ok, this works.
//    for (int i = 0; i < number_cameras; i++){
//        externals_gt_relative[i] = externals_gt[i]*externals_gt[0].inverse();
//
//        cout << "GT relative " << i << endl;
//
//        cout << externals_gt_relative[i] << endl;
//
//        centers_gt_relative[i] = ReturnCenter(externals_gt_relative[i]);
//    }
//
//    string ground_truth_relative_dir = output_dir + "ground-truth-cameras-relative/";
//
//    mkdir(ground_truth_relative_dir.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
//
//    Vector3d cam_color_gt;
//    cam_color_gt.setZero();
//    cam_color_gt(0) = 230;
//    cam_color_gt(1) = 159;
//
//
//    Vector3d cam_color_kr;
//    cam_color_kr.setZero();
//    cam_color_kr(1) = 158;
//    cam_color_kr(2) = 115;
//
//    for (int i = 0; i < number_cameras; i++){
//
//        filename = ground_truth_relative_dir + "/gt-camera"+ ToString<int>(i) + ".ply";
//
//        create_camera(internals_gt[i], externals_gt_relative[i], camera_size, cam_color_gt(0), cam_color_gt(1),
//                cam_color_gt(2), rows_cols[i*2], rows_cols[i*2 + 1], filename);
//
//    }
//
//    //        filename = cam_dir + "/all.ply";
//    //
//    //        if (internals.size() > 0){
//    //            create_cameras(internals, externals, cam_color(0), cam_color(1), cam_color(2), CCV[0]->rows, CCV[0]->cols, filename, camera_size);
//    //        }
//
//
//    /////////////////// calico cameras //////////////////////////////////////
//
//    in.open(comparison_file.c_str());
//
//
//    //FindValueOfFieldInFile(filename, fieldTag, separator, kill_if_not_found)
//    bool token_found = false;
//    string camera_name = "";
//
//    Vector4d distortion;
//    string extrin_string;
//    string num_string;
//
//    internal.setIdentity();
//
//    string tempString;
//    in >> number_cameras;
//
//    // question is if everytime the cameras are computed relative to camera 0.
//    //for (int i = 0; i < 1; i++){
//    for (int i = 0; i < number_cameras; i++){
//
//        in >> tempString;
//
//
//        // read in the internal matrix.
//        internal.setIdentity();
//
//        for (int r = 0; r < 3; r++){
//            for (int c  = 0; c < 3; c++){
//                in >> internal(r, c);
//            }
//        }
//
//        internals_kr[i] = internal;
//
//        external.setIdentity();
//
//        for (int r = 0; r < 3; r++){
//            for (int c  = 0; c < 3; c++){
//                in >> external(r, c);
//            }
//        }
//
//        for (int r = 0; r < 3; r++){
//            in >> external(r, 3);
//        }
//
//        externals_kr[i] = external;
//
//        for (int r = 0; r < 4; r++){
//            in >> distortion(r);
//        }
//
//        in >> tempString;
//
//
//        distortion_coefficients[i] = distortion;
//
//        // if (token.compare(fieldTag) == 0){
//
//        centers_kr[i] = ReturnCenter(externals_kr[i]);
//
//
//        cout << "External " << endl << external << endl;
//        //cout << "Line " << __LINE__ << endl;
//
//    }
//
//    in.close();
//
//
//    vector<Matrix4d> externals_calico_relative(number_cameras);
//
//
//    //M = CameraTransformations[i]*Minv;
//
//    // will need to check this.  visualize.  ok, this works.
//    for (int i = 0; i < number_cameras; i++){
//        externals_calico_relative[i] = externals_kr[i]*externals_kr[0].inverse();
//
//        cout << "GT relative " << i << endl;
//
//        cout << externals_calico_relative[i] << endl;
//
//        centers_calico_relative[i] = ReturnCenter(externals_calico_relative[i]);
//    }
//
//
//
//
//    //cout << "Line " << __LINE__ << endl;
//
//    string kalibr_dir = output_dir + "calico-cameras-relative/";
//
//    mkdir(kalibr_dir.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
//
//    for (int i = 0; i < number_cameras; i++){
//
//        filename = kalibr_dir + "/calico"+ ToString<int>(i) + ".ply";
//
//        create_camera(internals_kr[i], externals_calico_relative[i], camera_size, cam_color_kr(0), cam_color_kr(1),
//                cam_color_kr(2), rows_cols[i*2], rows_cols[i*2 + 1], filename);
//
//    }
//
//    double translation_error = 0;
//
//    Vector3d transA, transB;
//
//    Matrix3d Ra, Rb, Rc;
//
//    double degrees_per_rad = 180.0/M_PI;
//
//    double rot_error_degrees = 0;
//
//    for (int i = 0; i < number_cameras; i++){
//
//        for (int j = 0; j < 3; j++){
//            transA(j) = externals_gt_relative[i](j, 3);
//            transB(j) = externals_calico_relative[i](j, 3);
//
//            for (int k = 0; k < 3; k++){
//                Ra(j, k) = externals_gt_relative[i](j, k);
//                Rb(j, k) = externals_calico_relative[i](j, k);
//            }
//        }
//
//        translation_error += (transA - transB).norm();
//
//        Rc = Rb.transpose()*Ra;
//
//        Eigen::AngleAxisd newAngleAxis(Rc);
//
//        cout << "angle " << newAngleAxis.angle()*degrees_per_rad << endl;
//
//        rot_error_degrees += newAngleAxis.angle()*degrees_per_rad;
//
//    }
//
//    cout << "translation error, average: " << translation_error/double(number_cameras - 1) << endl;
//
//    cout << "rotation error, degrees, average: " << rot_error_degrees/double(number_cameras - 1) << endl;
//
//    string writeErrorFile = output_dir + "calicoErrorFile.txt";
//
//    cout << "write file " << writeErrorFile << endl;
//    ofstream writeError;
//    writeError.open(writeErrorFile.c_str());
//
//    writeError << "translation error, average: " << translation_error/double(number_cameras - 1) << endl;
//
//    writeError << "rotation error, degrees, average: " << rot_error_degrees/double(number_cameras - 1) << endl;
//
//    writeError.close();
//
//
//}


//============================================================================
// Name        : calico.cpp
// Author      : Amy Tabb
// Version     :
// Copyright   : MIT
// Description :
//============================================================================

#include "calico.hpp"

#include "helper.hpp"
#include "DirectoryFunctions.hpp"
#include "camera-calibration.hpp"
#include "multicamera.hpp"
#include "solving-structure.hpp"


int main(int argc, char **argv){

    int print_help = 0;
    //int rotating = 0;
    int calibrateGroupCameras = 0;
    int verbose = 0;
    int is_charuco = 0;
    int is_april = 0;
    int zero_tangent_dist = 1;
    int zero_k3 = 1;
    int read_camera_calibration = 0;
    int only_camera_calibration = 0;
    int max_internal_read  = -1; // not initialized;
    int max_internal_use = -1; // not initialized.
    int max_external_positions = -1; // not initialized
    int selectedk = -1;
    int create_patterns_only = 0;
    int fix_principal_point = 0;
    int number_points_needed_to_count_pattern = -1;
    int synchronized_rotating_option = 0;
    int number_threads = omp_get_max_threads();
    float percentage_global_alg = 0.2;
    float percentage_global_rp = 0.5;
    int interleaved_solve = 1;
    int doProfile = 0;

    string input_dir = "";
    string output_dir = "";
    float initial_focal_px = -1;
    double camera_size = 40.0;
    float track_size = 0.5;
    string src_file = "../src/detector_params.yml";


    if (argc == 1){
        print_help = 1;
    }


    while (1)
    {
        static struct option long_options[] =
        {
                {"help",   no_argument,       &print_help, 1},
                {"calibrate", no_argument,       &calibrateGroupCameras, 1}, //check
                {"create-patterns", no_argument, &create_patterns_only, 1}, //check
                {"non-zero-tangent", no_argument,       &zero_tangent_dist, 0}, //check
                {"non-zero-k3", no_argument,            &zero_k3, 0}, //check
                {"fix-pp", no_argument, &fix_principal_point, 1}, //check
                {"verbose", no_argument, &verbose, 1}, //check
                {"charuco",   no_argument,       &is_charuco, 1}, //check
                {"april",   no_argument,       &is_april, 1}, //check
                /* These options don’t set a flag.
                 We distinguish them by their indices. */
                {"input",   required_argument, 0, 'a'}, //check
                {"output",  required_argument, 0, 'b'}, //check
                {"camera-size", required_argument, 0, 'c'}, //check
                {"focal-px",  required_argument, 0, 'd'},
                {"src-dir", required_argument, 0, 'e'}, //check
                {"max-internal-read", required_argument, 0, 'f'}, //check
                {"max-internal-use", required_argument, 0, 'g'}, //check
                {"max-external", required_argument, 0, 'h'}, //check
                {"k", required_argument, 0, 'i'}, //check
                {"track-size", required_argument, 0, 'j'}, //check
                {"num-pattern", required_argument, 0, 'k'}, //check
                {"num-threads", required_argument, 0, 'l'}, //check
                {"perc-ae", required_argument, 0, 'm'}, //check
                {"perc-rp", required_argument, 0, 'n'}, //check
        };


        if (argc == 1){ print_help = 1; }

        if (print_help == 1){
            cout << "Printing help for calico, Dec. 2023."<< endl;

            cout << "ESSENTIAL FUNCTIONALITY -------------------" << endl;
            cout << std::left << setw(30) << "--verbose" << "No arguments.  Writes additional information during the run." << endl;
            cout << std::left << setw(30) << "--create-patterns" << "No arguments, write charuco or april image patterns from a specification file." << endl;
            cout << std::left << setw(30) << "--calibrate" << "No arguments, indicates that this dataset is a camera network or ";
                                cout << "multicamera system." << endl;
            cout << std::left << setw(30) << "--num-threads "<< "Number of threads to use.  Default is # returned by ";
            cout << "omp_get_max_threads();, currently = " << number_threads << endl;
            cout << std::left << setw(30) << "--charuco" << "Using charuco patterns." << endl;
            cout << std::left << setw(30) << "--april" << "Using AprilTag patterns." << endl;


            cout << endl;
            cout << "DIRECTORIES AND PATHS ----------------------- " << endl;
            cout << std::left << setw(30) << "--input=[STRING] "<< "Mandatory, has to be a directory." << endl;
            cout << std::left << setw(30) << "--output=[STRING] " << "Mandatory, has to be a directory." << endl;
            cout << std::left << setw(30) << "--src-dir=[STRING] ";
            cout << "Directory where the source code resides relative to where the executable is being run. " << endl;
            cout << " Specifically, the location of 'detector_params.yml'  Default is ../src/ . " << endl;

            cout << endl;
            cout << "CAMERA CALIBRATION OPTIONS ---------------------------" << endl;
            cout << std::left << setw(30) << "--non-zero-tangent " << "No arguments. In the camera calibration part, sets the tangential components of radial distortion (p1, p2) to non-zero." << endl;
            cout << std::left << setw(30) << "--non-zero-k3 " << "No arguments. In the camera calibration part, sets the 3rd radial distortion k value to non-zero." << endl;
            cout << std::left << setw(30) << "--fix-pp " << "No arguments. In the camera calibration part, sets the principal point to the image center. " << endl;
            cout << std::left << setw(30) << "--focal-px=[float] " << "Initial focal length in pixels for the camera.  Default is max dimension * 1.2 " << endl;

            cout << endl;

            cout << endl;
            cout <<"OPTIONS ON NUMBER OF IMAGES READ/USED; NUMBER OF POINTS USED FOR NETWORK -----------" << endl;
            cout << std::left << setw(30) << "--max-internal-read=[int] " << "Integer argument. Sets the number of internal camera calibration images to read.";
            cout << " Default is the number of images in the directory 'internal'." << endl;
            cout << std::left << setw(30) << "--max-internal-use=[int] " << "Integer argument. Sets the number of images to use where the pattern is detected in the calibration, from the 'internal' directory. ";
            cout << "The default is the maximum number of patterns found in . " << endl;
            cout << std::left << setw(30) << "--max-external=[int] " << "Integer argument. Sets the number of images /time instants to read from ";
            cout << " the 'external' directory for each camera." << endl;
            cout << std::left << setw(30) << "--k=[int] " << "Specifies the number of points to use for the minimization of reprojection error, relevant only ";
            cout << " for the network case.  The default is 8." << endl;
            cout << std::left << setw(30) << "--num-pattern=[int] " << "Integer argument. Sets the number of points required to estimate the  ";
            cout << " pose for a pattern.  Default is >=10 for network, >= 4 for rotating." << endl;

            cout <<"OPTIONS HOW OFTEN TO RUN GLOBAL MINIMIZATION OF VARIABLES -----------" << endl;
            cout << std::left << setw(30) << "--perc-ae=[float] " << "float argument from (0,1]. Global LM optimization of variables for algebraic error occurs after this percentage ";
            cout << "of variables are solved for. Unspecified, the default is 0.2, so this step will be run 5 times." << endl;
            cout << std::left << setw(30) << "--perc-rp=[float] " << "float argument from (0,1]. Global LM optimization of variables for reprojection error is triggered when this percentage ";
            cout << "of constraints/equations are added to the model. Unspecified, the default is 0.5, so this step will be run 2 times." << endl;

            cout << endl;
            cout <<"DISPLAY -----------" << endl;
            cout << std::left << setw(30) << "--camera-size=[float] " << "Float argument.  Specifies the size of the cameras written, default is 40." << endl;
            cout << std::left << setw(30) << "--track-size=[float] " << "Float argument.  Specifies the size of the track size written, default is 0.5 ." << endl;

            cout << "All other arguments are ignored." << endl;
            cout << endl << endl;

            exit(1);
        }


        /* getopt_long stores the option index here. */
        int option_index = 0;
        int opt_argument;

        opt_argument = getopt_long (argc, argv, "abcdefghijklmn",
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
            camera_size = FromString<float>(optarg);
            break;
        case 'd':
            initial_focal_px = FromString<float>(optarg);
            break;
        case 'e':
            src_file = optarg;
            EnsureDirHasTrailingBackslash(src_file);
            src_file = src_file + "detector_params.yml";
            break;
        case 'f':
            max_internal_read  = FromString<int>(optarg);
            break;
        case 'g':
            max_internal_use  = FromString<int>(optarg);
            break;
        case 'h':
            max_external_positions  = FromString<int>(optarg);
            break;
        case 'i':
            selectedk  = FromString<int>(optarg);
            break;
        case 'j':
            track_size  = FromString<int>(optarg);
            break;
        case 'k':
            number_points_needed_to_count_pattern  = FromString<int>(optarg);
            break;
        case 'l':
            number_threads  = FromString<int>(optarg);
            omp_set_num_threads(number_threads);
            break;
        case 'm':{
            float temp = FromString<float>(optarg);
            if (temp <= 0 || temp > 1){
                cout << "Value for --perc-ae is bad.  Needs to be greater than 0 and less than or equal to 1." << endl;
                exit(1);
            }

            percentage_global_alg = temp;
        }
        break;
        case 'n':
        {
            float temp = FromString<float>(optarg);
            if (temp <= 0 || temp > 1){
                cout << "Value for --perc-rp is bad.  Needs to be greater than 0 and less than or equal to 1." << endl;
                exit(1);
            }

            percentage_global_rp = temp;
        }
        break;
        default:{
            cout << "Argument not found " << optarg << endl;
            exit(1);
        }

        }

    }

    ///////// CHECK THAT DIRECTORIES EXIST /////////////////
    bool dir_exists = CheckExistenceOfDirectory(input_dir);

    if (dir_exists == false){
        exit(1);
    }

    dir_exists = CheckExistenceOfDirectory(output_dir);
    if (dir_exists == false){
        exit(1);
    }


    EnsureDirHasTrailingBackslash(input_dir);
    EnsureDirHasTrailingBackslash(output_dir);


    ofstream out;
    string filename = output_dir + "arguments-calico.txt";
    out.open(filename.c_str());
    out << "arguments: " << endl << argv[0] << " ";



    if ( (is_charuco && is_april) ){
        cout << "You cannot select both --charuco and --april.  It is exclusive or.  Quitting ... " << endl;
        exit(1);
    }

    if ((!is_april && !is_charuco)){
        cout << "You need to select either --charuco or --april.  Quitting ... " << endl;
        exit(1);
    }

    if (is_charuco){
        out << "--charuco " ;
    }

    if (is_april){
        out << "--april ";
    }

    if (calibrateGroupCameras){
        out << "--calibrate ";


        if (selectedk == -1){
            // then set to the default, which is 8.
            selectedk = 8;
        }

        if (selectedk != -1){
            out << "--k=" << selectedk << " "; // same, only relevant for the network case.
        }

        if (selectedk < 4){
            cout << "argument --k has to be greater than or equal to 4. Currently is it set to " << selectedk << endl;
            cout << "Quitting " << selectedk << endl;
            exit(1);
        }

        if (number_points_needed_to_count_pattern == -1){
            number_points_needed_to_count_pattern = 10;
        }

        synchronized_rotating_option = 1; // downstream, need this to be true for the network case.
    }

    if (create_patterns_only){
        out << "--create-patterns " << endl;

        if (input_dir.size() == 0|| output_dir.size() == 0 ){
            cout << "Error!  --create-patterns flag  used, but either :" << endl;
            cout << "1) input directory is empty" << endl;
            cout << "2) output directory is empty" << endl;
            cout << "none of these can occur with --create-patterns.  Fix and run again." << endl;
            exit(1);
        }
    }

    if (zero_tangent_dist == 0){
        out << "--non-zero-tangent ";
    }
    if (zero_k3 == 0){
        out << "--non-zero-k3  ";
    }

    out << "--perc-ae=" << percentage_global_alg << " --perc-rp=" << percentage_global_rp << " ";

    out << "--num-threads "<< number_threads << " ";

    out << "--input=" << input_dir << " " ;
    out << "--output=" << output_dir << " ";

    if (max_internal_use > max_internal_read){
        cout << "You entered --max-internal-read values < --max-internal-use values ... " << max_internal_read << ", "
                << max_internal_use << endl;
        cout << "I am correcting this error, check the argument file --   " << endl;

        max_internal_use = max_internal_read;
    }

    out << "--camera-size="<< camera_size << " " ;
    out << "--track-size="<< track_size << " " ;

    out << "--num-pattern=" << number_points_needed_to_count_pattern << " ";

    if (fix_principal_point){
        out << "--fix-pp ";
    }

    if (initial_focal_px > 0){
        out << "--focal-px="<< initial_focal_px << " ";
    }

    if (max_internal_read >= 0){
        out << "--max-internal-read=" << max_internal_read << " ";
    }

    if (max_internal_use >= 0) {
        out << "--max-internal-use=" << max_internal_use << " ";
    }

    if (max_external_positions >=  0){
        out << "--max-external=" << max_external_positions << " ";
    }


    if (verbose > 0 ){
        out << "--verbose  ";
    }


    if ((!calibrateGroupCameras && !create_patterns_only)){
        cout << "You need to select either --calibrate or --create-patterns.  Quitting ... " << endl;
        exit(1);
    }

    if (!interleaved_solve){
        out << "--non-incremental ";
    }


    out << endl << endl << "OpenCV Version " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "." << CV_SUBMINOR_VERSION << endl;

    out.close();


    if (!create_patterns_only){
        MultipleCameraCalibration(input_dir, output_dir, src_file, camera_size, track_size,
                initial_focal_px, zero_tangent_dist,
                zero_k3, fix_principal_point, verbose, read_camera_calibration, only_camera_calibration, max_internal_read, max_internal_use,
                max_external_positions, selectedk, number_points_needed_to_count_pattern, synchronized_rotating_option,
                percentage_global_alg, percentage_global_rp, is_charuco, is_april, interleaved_solve);
    }   else {
        CreatePatterns(input_dir, output_dir, src_file, is_charuco);
    }


    return 0;

}



void CreatePatterns(const string& input_dir, const string& output_dir,
        const string& src_file, bool is_charuco){

    string pattern_dir = output_dir + "patterns/";

    mkdir(pattern_dir.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

    PatternsCreated P_Class(input_dir, pattern_dir, src_file, false, is_charuco, true);

}

void MultipleCameraCalibration(const string& input_dir, const string& output_dir,
        const string& src_file, float camera_size, float track_size,  float initial_focal_px,
        int zero_tangent_dist, int zero_k3, int fix_principal_point,
        bool verbose, bool read_camera_calibration, bool only_camera_calibration,
        int max_internal_read, int max_internal_use, int max_external_positions, int selectedk,
        int number_points_needed_to_count_pattern, int synchronized_rotating_option, float percentage_global_alg,
        float percentage_global_rp,
        const bool is_charuco, const bool is_april, const bool interleavedSolve){

    cout << "is chaurco, is april " << is_charuco << ", " << is_april << endl;


    int number_not_in = 0;

    string logfile = output_dir + "logfile.txt";
    google::InitGoogleLogging(logfile.c_str());

    bool write_internals = false;

    auto start_timer = std::chrono::high_resolution_clock::now();


    vector<string> camera_write_dirs;
    string command;


    string filename = output_dir + "trace.txt";
    ofstream out_trace; out_trace.open(filename.c_str());

    out_trace << "Max external " << max_external_positions << endl;
    out_trace << "Max internal read " << max_internal_read << endl;
    out_trace << "Max internal use " << max_internal_use << endl;
    out_trace << "Write internal " << write_internals << endl;

    string pattern_dir = output_dir + "patterns/";

    mkdir(pattern_dir.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

    PatternsCreated* P_Class = 0;

    P_Class = new PatternsCreated(input_dir, pattern_dir, src_file, false, is_charuco, false);


    string temp;  int number_cameras;
    string id_camera_dir;

    ifstream in;
    ofstream out;  ofstream ceres_out;

    vector<string> camera_directories;
    ReadDirectory(string(input_dir + "data"), camera_directories);

    if (camera_directories.size() == 0){
        cout << "No camera directories, abort." << endl;
        exit(1);
    }

    if (!read_camera_calibration){

        mkdir((output_dir + "data/").c_str(), S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
    }

    string verbose_dir = output_dir + "verbose_output/";
    if (verbose){

        mkdir(verbose_dir.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
    }


    string reconstructed_patterns_dir = output_dir + "reconstructed-patterns/";

    mkdir(reconstructed_patterns_dir.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

    ///////////////////////////////////////////////////////////////////////////////////////////
    ////// Stage 1. Calibrate cameras wrt visible cameras at all time instants,
    // todo change numbers around after paper is solid
    ///////////////////////////////////////////////////////////////////////////////////////////


    number_cameras = camera_directories.size();
    out_trace << "After read " << number_cameras << endl;

    vector<CameraCali*> CCV(number_cameras, 0);

    for (int i = 0; i < number_cameras; i++){

        id_camera_dir = input_dir + "data/" + camera_directories[i];
        CameraCali* C = new CameraCali(id_camera_dir, P_Class,
                max_external_positions, max_internal_read,
                max_internal_use);
        out_trace << "After create new camera " << i << " " << camera_directories[i] << endl;

        CCV[i] = C;

        id_camera_dir = output_dir + "data/" + camera_directories[i] + "/";
        camera_write_dirs.push_back(id_camera_dir);

        cout << id_camera_dir << endl;


        if (!read_camera_calibration){

            mkdir(id_camera_dir.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

            CCV[i]->FindCorners(id_camera_dir);

            CCV[i]->CalibrateBasic(initial_focal_px, zero_tangent_dist, zero_k3,
                    fix_principal_point, id_camera_dir, number_points_needed_to_count_pattern);

        }

    }

    auto end_find_id_calibrate = std::chrono::high_resolution_clock::now();

    if (only_camera_calibration){
        // quit.
        out << "Times, seconds" << endl;
        out << "    Load, internal/external cali ... ";
        out << std::chrono::duration_cast<std::chrono::seconds>(end_find_id_calibrate - start_timer).count() << endl;
        out.close();

        exit(1);
    }

    ////////////////////////// END CAMERA CALIBRATION ////////////////////////////////////////////

    double* camera_params = new double[CCV.size()*12];



    CopyFromCalibration(CCV, camera_params);


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////// Stage 2. Generation of single and double foundational relationships; simplify using common factors.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////

    filename = output_dir + "multicamera_detail.txt";
    out.open(filename.c_str());
    filename = output_dir + "multicamera_detail_ceres.txt";
    ceres_out.open(filename.c_str());

    filename = output_dir + "rae_detail_ceres.txt";
    ofstream rae_ceres_out;
    rae_ceres_out.open(filename.c_str());

    for (uint i = 0; i < CCV.size(); i++){
        ceres_out << "Camera " << i << endl;
        for (int j = 0; j < 12; j++){
            ceres_out <<  camera_params[12*i + j] << " ";
        }
        ceres_out << endl;
    }

    /// MCcali chooses the exemplar pattern and time ////
    MCcali MC(CCV, *P_Class, max_external_positions, out, synchronized_rotating_option);

    number_not_in = MC.BuildCostFunctionAndGraphWithTestAndDegenerateInitialize(CCV, out, verbose_dir, verbose);

    out << "Number not in the connected component = " << number_not_in << endl;
    out << "Number in the connected component = " << MC.NumberSingles() - number_not_in << endl;


    if (number_not_in > 0){
        cout << "Cannot solve, cc component number greater than 1 " << endl;
        cout << "Quitting " << endl;

        exit(1);
    }



    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////// Stage 3. Substitute exemplar pattern and time in single and double
    /// foundational relationships, determine the C values that can be computed.
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    vector<int> variable_order;

    MC.SubstitutePTstar(variable_order, verbose_dir, verbose);



    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////// Stage 4. Iteratively find the best solutions for individual variables (C, P, T) given solutions found at prior iterations
    ///////////////////////////////////////////////////////////////////////////////////////////////////////

    bool has_some_to_solve  = true;

    auto start_stage_4 = std::chrono::high_resolution_clock::now();
    auto end_stage_4 = std::chrono::high_resolution_clock::now();

    //// THIS IS DONE FOR BOTH INCREMENTAL AND PHASED SOLVE /////////


    for (uint i = 0; i < CCV.size(); i++){
        CCV[i]->SetUpSelectPointsForMinimization();
    }


    // strategy: use k-means to estimate clusters in the images, use only k points per image for minimization.
    MC.SelectKPointsForMinimization(CCV, selectedk);

    out_trace << "Selected k " << selectedk << endl;

    int number_iters_local = 10;
    vector<int> equation_order;
    vector<int> equations_per_iter;
    int var_accumulator = 0;
    int num_equations = 0;
    int mod_solve = int(round(percentage_global_alg*double(MC.NumberVariables())));

    if (mod_solve == 0){mod_solve = 1;}



    CeresProblemClass CPC(Cali_Quaternion, MC, ceres_out);

    if (variable_order.size() > 0){
        num_equations = CPC.AddToProblemAlgebraicError(MC, variable_order, equation_order, out, variable_order.size());
        equations_per_iter.push_back(num_equations);
    }


    for (int i = 0, vn = MC.NumberVariables(); i < vn && has_some_to_solve == true; i++){

        cout << "counting " << i << endl;


        has_some_to_solve = MC.IterativelySolveForVariables2(out, false, variable_order,interleavedSolve,
                number_iters_local);


        if (has_some_to_solve){

            var_accumulator++;
        }


        if (!has_some_to_solve){

            bool can_solve = MC.CanSolveSystem();
            out_trace << "Can solve this system??? " << can_solve << endl;

            if (can_solve == false){


                MC.TestSolutionHasIdentityPstarTstar(MC.V_initial);

                bool used_closed_form = MC.SolveClique(out, variable_order, number_iters_local);

                var_accumulator+= 2;

                has_some_to_solve = true;

                }
        }

        if (i % mod_solve == 0){
            num_equations = CPC.AddToProblemAlgebraicError(MC, variable_order, equation_order, out, var_accumulator);
            equations_per_iter.push_back(num_equations);

            CPC.SolveWriteBackToMCAlgebraicError(MC, ceres_out, number_iters_local, true);

            var_accumulator = 0;
        }


    }

    // this is the cleanup section

    num_equations = CPC.AddToProblemAlgebraicError(MC, variable_order, equation_order, out, var_accumulator);
    equations_per_iter.push_back(num_equations);

    CPC.SolveWriteBackToMCAlgebraicError(MC, ceres_out, number_iters_local, true);

    cout << "break in between algebraic error and reprojection ... " << endl;

    ceres_out << "--------------- reprojection error ----------------------------" << endl;

    ///////////////////////////////////// Now, stage 5. ///////////////////////////////////////////
    ////////////////////////////// reprojection error. ////////////////////////////////////////////


    int number_equations = equation_order.size();
    int number_equations_before_solve = int(round(percentage_global_rp*float(number_equations)));
    int number_iters_rp = 10;

    // ceres class already has all of the variables.  But we need them in the format for the reprojection problem.
    CPC.SetUpXForReprojectionError(MC);

    int end_index = 0;

    cout << "Number of equations " << number_equations << endl;

    for (int start_index = 0; start_index < number_equations; start_index+=number_equations_before_solve){
        // call

        end_index = min(start_index + number_equations_before_solve, number_equations);

        cout << "Adding equations  " << start_index << ", " << end_index << endl;

        CPC.AddEqsToProblemReprojectionError(MC, CCV, camera_params, start_index, end_index, equation_order);

        CPC.SolveWriteBackToMCRP(MC, ceres_out, number_iters_rp, variable_order, true);

    }

    MC.V_progressive_solutions.push_back(MC.V_initial);

    auto end_stage_5 = std::chrono::high_resolution_clock::now();


    ////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////// OUTPUT SECTION ////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    MC.ReconstructionAccuracyErrorAndWriteII(reconstructed_patterns_dir, 2, CCV, camera_params, rae_ceres_out);

    rae_ceres_out.close();


    cout << "After iteratively solve " << endl;
    cout << "Number uninitialized " << MC.NumberUninitialized() << endl;
    cout << "Number to fill in " << MC.RemainingToFillIn() << endl;


    out_trace  << "After iteratively solve " << endl;
    out_trace << "Number uninitialized " << MC.NumberUninitialized() << endl;
    out_trace << "Number to fill in " << MC.RemainingToFillIn() << endl;


    out << "Times, seconds" << endl;
    out << "    Load, internal/external cali ... " << std::chrono::duration_cast<std::chrono::seconds>(end_find_id_calibrate - start_timer).count()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            << " seconds "<< endl;
    out << " Set up cali structure           ... " << std::chrono::duration_cast<std::chrono::seconds>(start_stage_4 - end_find_id_calibrate).count()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            << " seconds "<< endl;
    out << "Stage 4                           ... " << std::chrono::duration_cast<std::chrono::seconds>(end_stage_4 - start_stage_4).count()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            << " seconds "<< endl;
    out << "Stage 5                           ... " << std::chrono::duration_cast<std::chrono::seconds>(end_stage_5 - end_stage_4).count()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            << " seconds "<< endl;
    out << "Total, load and everything ... " <<   std::chrono::duration_cast<std::chrono::seconds>(end_stage_5 - start_timer).count()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    << " seconds "<< endl;
    out << "MetaTotal, without load+internal/external cali, just network cali ... " <<   std::chrono::duration_cast<std::chrono::seconds>(end_stage_5 - end_find_id_calibrate).count()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            << " seconds "<< endl;


    out << "Times,milliseconds" << endl;
    out << "    Load, internal/external cali ... ";
    out << std::chrono::duration_cast<std::chrono::milliseconds>(end_find_id_calibrate - start_timer).count() << " ms"<< endl;
    out << " Set up cali structure           ... ";
    out << std::chrono::duration_cast<std::chrono::milliseconds>(start_stage_4 - end_find_id_calibrate).count() << " ms "<< endl;
    out << "Stage 4                           ... ";
    out << std::chrono::duration_cast<std::chrono::milliseconds>(end_stage_4 - start_stage_4).count() << " ms "<< endl;
    out << "Stage 5                           ... ";
    out << std::chrono::duration_cast<std::chrono::milliseconds>(end_stage_5 - end_stage_4).count() << " ms "<< endl;
    out << "Total, load and everything ... ";
    out <<   std::chrono::duration_cast<std::chrono::milliseconds>(end_stage_5 - start_timer).count() << " ms "<< endl;
    out << "MetaTotal, without load+internal/external cali, just network cali ... ";
    out <<   std::chrono::duration_cast<std::chrono::milliseconds>(end_stage_5 - end_find_id_calibrate).count() << " milliseconds "<< endl;

    out << "Times, minutes" << endl;
    out << "Total, load and everything ... " <<   std::chrono::duration_cast<std::chrono::minutes>(end_stage_5 - start_timer).count()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                << " minutes "<< endl;
    out << "MetaTotal, without load+internal/external cali, just network cali ... " <<   std::chrono::duration_cast<std::chrono::minutes>(end_stage_5 - end_find_id_calibrate).count()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    << " minutes "<< endl;


    MC.WriteSolutionAssessErrorII(output_dir, camera_directories, CCV, 2, true, camera_size, track_size);

    out.close();
    ceres_out.close();

    filename = output_dir + "total_results.txt";

    // MC.OutputRunResults(filename);

     MC.OutputRunResultsII(filename);

    //////////////////////////////////-- DEALLOC -----------////////////////////////////////////////

    // dealloc
    for (int i = 0; i < number_cameras; i++){
        delete CCV[i];
    }

    CCV.clear();

    delete [] camera_params;

    delete P_Class;

}

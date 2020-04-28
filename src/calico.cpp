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
#include "camera_calibration.hpp"
#include "multicamera.hpp"
#include "solving_structure.hpp"

int main(int argc, char **argv){

	int print_help = 0;
	int rotating = 0;
	int network = 0;
	int verbose = 0;
	int zero_tangent_dist = 1; // default is to have these set to zero
	int zero_k3 = 1;           // default is to have these set to zero
	int read_camera_calibration = 0;
	int only_camera_calibration = 0;
	int max_internal_read  = -1; // not initialized;
	int max_internal_use = -1; // not initialized.
	int max_external_positions = -1; // not initialized
	int selectedk = -1;
	int ground_truth = 0;
	int create_patterns_only = 0;
	int fix_principal_point = 0;
	int number_points_needed_to_count_pattern = -1;
	int synchronized_rotating_option = 0;
	int interleaved_solve = 1; // set as default, since the best option.
	int number_threads = omp_get_max_threads();

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
				{"rotating", no_argument,       &rotating, 1},
				{"network", no_argument,       &network, 1},
				{"create-patterns", no_argument, &create_patterns_only, 1},
				{"read-camera-cali", no_argument, &read_camera_calibration, 1},
				{"only-camera-cali", no_argument, &only_camera_calibration, 1},
				{"non-zero-tangent", no_argument,       &zero_tangent_dist, 0},
				{"non-zero-k3", no_argument,            &zero_k3, 0},
				{"fix-pp", no_argument, &fix_principal_point, 1},
				{"verbose", no_argument, &verbose, 1},
				{"ground-truth", no_argument, &ground_truth, 1},
				{"synch-rotate", no_argument, &synchronized_rotating_option, 1},
				{"non-incremental", no_argument, &interleaved_solve, 0},
				/* These options don’t set a flag.
	             We distinguish them by their indices. */
				{"input",   required_argument, 0, 'a'},
				{"output",  required_argument, 0, 'b'},
				{"camera-size", required_argument, 0, 'c'},
				{"focal-px",  required_argument, 0, 'd'},
				{"src-dir", required_argument, 0, 'e'},
				{"max-internal-read", required_argument, 0, 'f'},
				{"max-internal-use", required_argument, 0, 'g'},
				{"max-external", required_argument, 0, 'h'},
				{"k", required_argument, 0, 'i'},
				{"track-size", required_argument, 0, 'j'},
				{"num-pattern", required_argument, 0, 'k'},
				{"num-threads", required_argument, 0, 'l'},
		};


		if (print_help == 1){
			cout << "Printing help for calico"<< endl;

			cout << "ESSENTIAL FUNCTIONALITY -------------------" << endl;
			cout << std::left << setw(30) << "--verbose" << "No arguments.  Writes additional information during the run." << endl;
			cout << std::left << setw(30) << "--create-patterns" << "No arguments, write aruco image patterns from a specification file." << endl;
			cout << std::left << setw(30) << "--network" << "No arguments, indicates that this dataset is a camera network or ";
			cout << "multicamera system." << endl;
			cout << std::left << setw(30) << "--rotating" << "No arguments, indicates that this dataset is the rotating type ";
			cout << "with background." << endl;
			cout << std::left << setw(30) << "--synch-rotate" << "No arguments, indicates the where there are multiple ";
			cout << "cameras in the rotating case, that they are synchronized.  Default is false. (not synchronized)." << endl;
			cout << std::left << setw(30) << "--ground-truth " << "No arguments. The ground truth calibration information is available." << endl;
			cout << std::left << setw(30) << "--num-threads "<< "Number of threads to use.  Default is # returned by ";
			cout << "omp_get_max_threads();, currently = " << number_threads << endl;
			cout << std::left << setw(30) << "--non-incremental " << "No arguments.  Set the solving method to use the linear the ";
			cout << "non-linear match minimization, instead of the default incremental linear/non-linear reprojection error algo." << endl;

			cout << endl;
			cout << "DIRECTORIES AND PATHS ----------------------- " << endl;
			cout << std::left << setw(30) << "--input=[STRING] "<< "Mandatory, has to be a directory." << endl;
			cout << std::left << setw(30) << "--output=[STRING] " << "Mandatory, has to be a directory." << endl;
			cout << std::left << setw(30) << "--src-dir=[STRING] ";
			cout << "Directory where the source code resides relative to where the executable is being run. " << endl;
			cout << " Specifically, the location of 'detector_params.yml'  Default is ../src/ . " << endl;

			cout << endl;
			cout << "CAMERA CALIBRATION OPTIONS ---------------------------" << endl;
			cout << std::left << setw(30) << "--only-camera-cali" << "No arguments, only perform individual camera calibration, but not network calibration." << endl;
			cout << std::left << setw(30) << "--read-camera-cali" << "No arguments, read-previously-computed camera calibration from file.  It should be in the output directory."   << endl;
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

		opt_argument = getopt_long (argc, argv, "abcdefghijkl",
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
			break;
		default:{
			cout << "Argument not found " << optarg << endl;
			exit(1);
		}

		}

	}

	struct stat info;

	///////// CHECK THAT DIRECTORIES EXIST /////////////////
	if (input_dir.size() > 0 && output_dir.size() > 0){
		if( stat( output_dir.c_str(), &info ) != 0 ){
			cout << "cannot access " << output_dir << endl;
			exit(1);
		}
		else if( info.st_mode & S_IFDIR ){  // S_ISDIR()
		}

		if( stat( input_dir.c_str(), &info ) != 0 ){
			cout << "cannot access " << input_dir << endl;
			exit(1);
		}
		else if( info.st_mode & S_IFDIR ){  // S_ISDIR()

		}
	}	else {
		if (output_dir.size() == 0){
			cout << "output directory was empty" << endl; exit(1);
		}

		if (input_dir.size() == 0){
			cout << "input directory was empty" << endl; exit(1);
		}
	}


	EnsureDirHasTrailingBackslash(input_dir);
	EnsureDirHasTrailingBackslash(output_dir);


	ofstream out;
	string filename = output_dir + "arguments-calico.txt";
	out.open(filename.c_str());
	out << "arguments: " << endl << argv[0] << " ";
	if (rotating){
		out << "--rotating " ;

		if (number_points_needed_to_count_pattern == -1){
			number_points_needed_to_count_pattern = 4;
		}

		if (synchronized_rotating_option){
			out << "--synch-rotate ";
		}
	}

	if (network){
		out << "--network ";


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

	if (ground_truth == 1){
		out << "--ground-truth ";
	}

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

	if (read_camera_calibration > 0){
		out << "--read-camera-cali ";
	}

	if (interleaved_solve == 0){
		out << "--non-incremental " << endl;
	}

	if (verbose > 0 ){
		out << "--verbose  ";
	}

	if ( (rotating && network) ){
		cout << "You cannot select both --rotating and --network.  It is exclusive or.  Quitting ... " << endl;
		exit(1);
	}

	if ((!rotating && !network && !create_patterns_only)){
		cout << "You need to select either --rotating or --network or --create-patterns.  Quitting ... " << endl;
		exit(1);
	}

	out << endl << endl << "OpenCV Version " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "." << CV_SUBMINOR_VERSION << endl;

	out.close();

	if (only_camera_calibration && ground_truth){
		cout << "You entered ground truth and only camera calibration.  These two options not allowed at once.  Quitting." << endl;
		exit(1);
	}

	// if need to read the ground truth items from file ... leave this here.
	// have the read file be in the same format as I do the results for this function.
	int calibration_context = 0;
	network == 1 ? calibration_context = 1 : 0;

	if (create_patterns_only){
		CreatePatterns(calibration_context, input_dir, output_dir, src_file);
	}	else {
		MultipleCameraCalibration(calibration_context, input_dir, output_dir, src_file, camera_size, track_size,
				initial_focal_px, zero_tangent_dist,
				zero_k3, fix_principal_point,
				ground_truth, verbose, read_camera_calibration, only_camera_calibration, max_internal_read, max_internal_use,
				max_external_positions, selectedk, number_points_needed_to_count_pattern, synchronized_rotating_option,
				interleaved_solve);
	}


	return 0;


}


void CreatePatterns(int calibration_context, string input_dir, string output_dir, string src_file){

	// calibration context -- 0 for rotating, 1 for network version.
	// these variables technically not needed, but added for readability.
	bool rotating = calibration_context == 0;

	string command;

	string pattern_dir = output_dir + "patterns/";
	command = "mkdir " + pattern_dir;
	cout << "command ... " << command << endl;
	system(command.c_str());

	PatternsCreated P_Class(input_dir, pattern_dir, rotating, src_file, true);

}

void MultipleCameraCalibration(int calibration_context, string input_dir, string output_dir, string src_file,
		float camera_size, float track_size,  float initial_focal_px, int zero_tangent_dist, int zero_k3, int fix_principal_point,
		bool has_ground_truth, bool verbose, bool read_camera_calibration, bool only_camera_calibration,
		int max_internal_read, int max_internal_use, int max_external_positions, int selectedk,
		int number_points_needed_to_count_pattern, int synchronized_rotating_option, int interleaved_solve){

	// calibration context -- 0 for rotating, 1 for network version.
	// these variables technically not needed, but added for readability.
	bool rotating = calibration_context == 0;
	bool network = calibration_context == 1;
	int number_not_in = 0;


	string logfile = output_dir + "logfile.txt";
	google::InitGoogleLogging(logfile.c_str());

	bool write_internals = false;

	auto start_timer = std::chrono::high_resolution_clock::now();

	GroundTruthData* GTD = 0;
	if (has_ground_truth){
		GTD = new GroundTruthData(input_dir);
	}

	vector<string> camera_write_dirs;
	string command;

	string filename = output_dir + "trace.txt";
	ofstream out_trace; out_trace.open(filename.c_str());

	out_trace << "Max external " << max_external_positions << endl;
	out_trace << "Max internal read " << max_internal_read << endl;
	out_trace << "Max internal use " << max_internal_use << endl;
	out_trace << "Write internal " << write_internals << endl;

	string pattern_dir = output_dir + "patterns/";
	command = "mkdir " + pattern_dir;
	cout << "command ... " << command << endl;
	system(command.c_str());

	PatternsCreated P_Class(input_dir, pattern_dir, rotating, src_file, false);
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
		command = "mkdir " + output_dir + "data/";
		cout << "command ... " << command << endl;
		system(command.c_str());
	}

	string verbose_dir = output_dir + "verbose_output/";
	if (verbose){
		command = "mkdir " + verbose_dir;
		cout << "command ... " << command << endl;
		system(command.c_str());
	}


	string reconstructed_patterns_dir = output_dir + "reconstructed-patterns/";
	command = "mkdir " + reconstructed_patterns_dir;
	cout << "command ... " << command << endl;
	system(command.c_str());


	///////////////////////////////////////////////////////////////////////////////////////////
	////// Stage 1. Calibrate cameras wrt visible cameras at all time instants,
	///////////////////////////////////////////////////////////////////////////////////////////


	number_cameras = camera_directories.size();
	out_trace << "After read " << number_cameras << endl;


	vector<CameraCali*> CCV(number_cameras, 0);

	for (int i = 0; i < number_cameras; i++){

		id_camera_dir = input_dir + "data/" + camera_directories[i];
		CameraCali* C = new CameraCali(id_camera_dir, &P_Class,
				max_external_positions, max_internal_read,
				max_internal_use);
		out_trace << "After create new camera " << i << " " << camera_directories[i] << endl;


		/// read exif information.
		if (rotating){
			C->ReadExifInformationRotatingSet(input_dir, id_camera_dir);
		}

		CCV[i] = C;

		id_camera_dir = output_dir + "data/" + camera_directories[i] + "/";
		camera_write_dirs.push_back(id_camera_dir);

		cout << id_camera_dir << endl;
		if (rotating){
			if (!read_camera_calibration){
				command = "mkdir " + id_camera_dir;
				system(command.c_str());

				CCV[i]->FindCornersArucoCharuco(id_camera_dir, verbose, verbose_dir);
				CCV[i]->CalibrateRotatingSet(id_camera_dir, number_points_needed_to_count_pattern);

			}	else {

				CCV[i]->ReadCorners(id_camera_dir);
				CCV[i]->ReadCalibration(id_camera_dir);

			}
		}	else {

			if (!read_camera_calibration){
				command = "mkdir " + id_camera_dir;
				system(command.c_str());
				CCV[i]->FindCornersCharuco(id_camera_dir);
				CCV[i]->CalibrateBasic(initial_focal_px, zero_tangent_dist, zero_k3,
						fix_principal_point, id_camera_dir, number_points_needed_to_count_pattern);
			}	else {

				CCV[i]->ReadCorners(id_camera_dir);
				CCV[i]->ReadCalibration(id_camera_dir);
			}
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


	MCcali MC(CCV, P_Class, max_external_positions, out, synchronized_rotating_option);

	number_not_in = MC.BuildCostFunctionAndGraphWithTestAndDegenerateInitialize(CCV, out, verbose_dir, verbose);

	out << "Number not in the connected component = " << number_not_in << endl;
	out << "Number in the connected component = " << MC.NumberSingles() - number_not_in << endl;


	if (number_not_in > 0){
		cout << "Cannot solve, cc component number greater than 1 " << endl;
		cout << "Quitting " << endl;

		exit(1);
	}

	if (verbose){
		filename = verbose_dir + "variables_after_2.txt";
		MC.OutputVariablesWithInitialization( filename, 0);
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////// Stage 3. Choose exemplar pattern and time; substitute exemplar pattern and time in single and double
	/// foundational relationships, determine the C values that can be computed.
	///////////////////////////////////////////////////////////////////////////////////////////////////////

	MC.SubstitutePTstar(verbose_dir, verbose, 0);

	if (verbose){
		filename = verbose_dir + "variables_after_3.txt";
		MC.OutputVariablesWithInitialization( filename, 0);
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////// Stage 4. Iteratively find the best solutions for individual variables (C, P, T) given solutions found at prior iterations
	///////////////////////////////////////////////////////////////////////////////////////////////////////

	int stage_4_counter = 0;

	bool has_some_to_solve  = true;

	int solved_var;



	auto start_stage_4 = std::chrono::high_resolution_clock::now();
	auto end_stage_4 = std::chrono::high_resolution_clock::now();

	//// THIS IS DONE FOR BOTH INCREMENTAL AND PHASED SOLVE /////////


	for (uint i = 0; i < CCV.size(); i++){
		CCV[i]->SetUpSelectPointsForMinimization();
	}

	if (network){
		// strategy: use k-means to estimate clusters in the images, use only k points per image for minimization.
		MC.SelectKPointsForMinimization(CCV, selectedk);
	}

	out_trace << "Selected k " << selectedk << endl;


	if (interleaved_solve == 0){

		CeresProblemClass CPC(Cali_Quaternion, MC, ceres_out);


		for (int i = 0, vn = MC.NumberVariables(); i < vn && has_some_to_solve == true; i++){
			has_some_to_solve = MC.IterativelySolveForVariables1(out, solved_var, false);
			// the initializated vars list should keep a record of which vars are solved (or not).

			CPC.AddToProblem(MC, CCV, camera_params, out);

			if (has_some_to_solve && verbose){
				filename = verbose_dir + "variables_after_4_" +  ToString<int>(i) + ".txt";
				MC.OutputVariablesWithInitialization( filename, stage_4_counter);
			}

			if (!has_some_to_solve){
				bool can_solve = MC.CanSolveSystem();
				out_trace << "Can solve this system??? " << can_solve << endl;

				if (can_solve == false){
					cout << "Finding pairs to solve " << endl;

					bool solved_pair = MC.SolveClique(out);
					CPC.AddToProblem(MC, CCV, camera_params, out);

					if (solved_pair){
						has_some_to_solve = true;

					}

					if (solved_pair && verbose){

						filename = verbose_dir + "variables_after_4_" +  ToString<int>(i) + "_pair.txt";
						MC.OutputVariablesWithInitialization( filename, stage_4_counter);

					}
				}
			}
		}

		MC.V_progressive_solutions.push_back(MC.V_initial);

		// now solve, read back into vars.
		CPC.SolveWriteBackToMC(MC, ceres_out, 50, true);
		// don't have one for linear, then not, so just push both.

		MC.V_progressive_solutions.push_back(MC.V_initial);

	}	else {

		CeresProblemClass CPC(Cali_Quaternion, MC, ceres_out);


		for (int i = 0, vn = MC.NumberVariables(); i < vn && has_some_to_solve == true; i++){
			has_some_to_solve = MC.IterativelySolveForVariables1(out, solved_var, false);
			// the initialized vars list should keep a record of which vars are solved (or not).

			CPC.AddToProblem(MC, CCV, camera_params, out);
			CPC.SolveWriteBackToMC(MC, ceres_out, 4, false);

			if (has_some_to_solve && verbose){
				filename = verbose_dir + "variables_after_4_" +  ToString<int>(i) + ".txt";
				MC.OutputVariablesWithInitialization( filename, stage_4_counter);
			}

			if (!has_some_to_solve){
				bool can_solve = MC.CanSolveSystem();
				out_trace << "Can solve this system??? " << can_solve << endl;

				if (can_solve == false){
					cout << "Finding pairs to solve " << endl;

					bool solved_pair = MC.SolveClique(out);
					CPC.AddToProblem(MC, CCV, camera_params, out);
					CPC.SolveWriteBackToMC(MC, ceres_out, 4, false);

					if (solved_pair ){
						has_some_to_solve = true;
					}

					if (solved_pair && verbose){
						filename = verbose_dir + "variables_after_4_" +  ToString<int>(i) + "_pair.txt";
						MC.OutputVariablesWithInitialization( filename, stage_4_counter);
					}
				}
			}
		}

		// don't have one for linear, then not, so just push both.
		//MC.V_progressive_solutions.push_back(MC.V_initial);

		// now solve, read back into vars.
		CPC.SolveWriteBackToMC(MC, ceres_out, 40, true);

		MC.V_progressive_solutions.push_back(MC.V_initial);

	}

	auto end_stage_5 = std::chrono::high_resolution_clock::now();


	////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////// OUTPUT SECTION ////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////

	if (has_ground_truth){
		GTD->ComputeRelativeToExemplar(MC.p_star(), MC.t_star(), output_dir, CCV[0]->P_class);

		MC.WriteSolutionAssessError(output_dir, camera_directories, CCV, -2, GTD, rotating, true, camera_size, track_size);
		MC.WriteSolutionAssessError(output_dir, camera_directories, CCV, -1, GTD, rotating, true, camera_size, track_size);

		MC.ReconstructionAccuracyErrorAndWriteI(reconstructed_patterns_dir, -2, CCV, camera_params, rae_ceres_out, GTD);
		MC.ReconstructionAccuracyErrorAndWriteI(reconstructed_patterns_dir, -1, CCV, camera_params, rae_ceres_out, GTD);

		if (!interleaved_solve){

			MC.AssessCamerasWRTGroundTruth(GTD, 0);
			MC.AssessCamerasWRTGroundTruth(GTD, 1);
		}	else {
			MC.AssessCamerasWRTGroundTruth(GTD, 0);
		}

	}


	if (!interleaved_solve){
		// initial solution
		MC.ReconstructionAccuracyErrorAndWriteI(reconstructed_patterns_dir, 0, CCV, camera_params, rae_ceres_out, GTD);

		// solution from solving reprojection error.
		MC.ReconstructionAccuracyErrorAndWriteI(reconstructed_patterns_dir, 1, CCV, camera_params, rae_ceres_out, GTD);

	}	else {

		MC.ReconstructionAccuracyErrorAndWriteI(reconstructed_patterns_dir, 2, CCV, camera_params, rae_ceres_out, GTD);

	}

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
	out << std::chrono::duration_cast<std::chrono::milliseconds>(end_stage_5 - end_stage_4).count()	<< " ms "<< endl;
	out << "Total, load and everything ... ";
	out <<   std::chrono::duration_cast<std::chrono::milliseconds>(end_stage_5 - start_timer).count() << " ms "<< endl;
	out << "MetaTotal, without load+internal/external cali, just network cali ... ";
	out <<   std::chrono::duration_cast<std::chrono::milliseconds>(end_stage_5 - end_find_id_calibrate).count() << " milliseconds "<< endl;

	out << "Times, minutes" << endl;
	out << "Total, load and everything ... " <<   std::chrono::duration_cast<std::chrono::minutes>(end_stage_5 - start_timer).count()
																																																																																																<< " minutes "<< endl;
	out << "MetaTotal, without load+internal/external cali, just network cali ... " <<   std::chrono::duration_cast<std::chrono::minutes>(end_stage_5 - end_find_id_calibrate).count()
																																																																																																	<< " minutes "<< endl;

	if (!interleaved_solve){
		MC.WriteSolutionAssessError(output_dir, camera_directories, CCV, 0, GTD, rotating, true, camera_size, track_size);
		MC.WriteSolutionAssessError(output_dir, camera_directories, CCV, 1, GTD, rotating, true, camera_size, track_size);
	}	else {
		MC.WriteSolutionAssessError(output_dir, camera_directories, CCV, 2, GTD, rotating, true, camera_size, track_size);
	}


	out.close();
	ceres_out.close();

	filename = output_dir + "total_results.txt";

	MC.OutputRunResults(filename);


	if (rotating){
		// write the calibration information.
		MC.WriteCalibrationFileForSimulatedCamerasAtAllTimes( output_dir,  CCV);
	}


	//////////////////////////////////-- DEALLOC -----------////////////////////////////////////////

	// dealloc
	for (int i = 0; i < number_cameras; i++){
		delete CCV[i];
	}

	CCV.clear();

	delete [] camera_params;

	if (GTD != 0){
		delete GTD;
	}

}


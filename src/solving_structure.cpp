/*
 * solving_structure.cpp
 *
 *  Created on: Jul 16, 2018
 *      Author: Amy Tabb
 */

#include "solving_structure.hpp"
#include "multicamera.hpp"
#include "camera_visualization.hpp"

void SolveWithShahsMethod(Matrix4d& Result, vector<Matrix4d>& LHS, vector<Matrix4d>& RHS, bool verbose){
	//Format is VAR Known_Constant_LHS = Known_Constant_RHS
	// Code from Shah .... YB = A, LHS = B, RHS = A.
	/// Matrices are stacked side by side [horizontally].

	int n = LHS.size();
	MatrixXd X(3, n*3);   // LHS
	MatrixXd Xprime(3, n*3);  /// RHS
	Vector3d t; //LHS    // t from Shah
	Vector3d tprime; //RHS // t-hat from Shah
	// find average lhs t, .

	t.setZero();
	tprime.setZero();

	//LHS = B, RHS = A
	for (int i = 0; i < n; i++){
		for (int j = 0; j < 3; j++){
			tprime(j) += RHS[i](j, 3);
			t(j) += LHS[i](j, 3);
		}
	}
	for (int j = 0; j < 3; j++){
		t(j) /= double(n);
		tprime(j) /= double(n);
	}


	if (verbose){
		cout << "Average lhs (t) " << t << endl << "Average rhs (tprime) " << tprime << endl;
	}

	/// build the A and B mats ...
	for (int i = 0; i < n; i++){
		for (int r = 0; r < 3; r++){
			for (int c = 0; c < 3; c++){
				Xprime(r, i*3 + c) = RHS[i](r, c);
			}
		}

		for (int r = 0; r < 3; r++){
			for (int c = 0; c < 3; c++){
				X(r, i*3 + c) = LHS[i](r, c);
			}
		}

	}

	if (verbose){
		cout << "To solve: HX = Xprime" << endl << "Y" << endl;
		for (int i = 0; i < n; i++){
			cout << i << endl << LHS[i] << endl << RHS[i] << endl;
		}

		ofstream out;

		string filename = "X.txt";
		out.open(filename.c_str());
		out << X << endl;
		out.close();

		filename = "Xprime.txt";
		out.open(filename.c_str());
		out << Xprime << endl;
		out.close();

		char ch; cin >> ch;
	}

	// svd with eigen
	MatrixXd XXprimet = X*Xprime.transpose();

	JacobiSVD<MatrixXd> svd(XXprimet, ComputeThinU | ComputeThinV);
	if (verbose){
		cout << "Its singular values are:" << endl << svd.singularValues() << endl;
		cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;

		cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
	}

	Matrix3d R = svd.matrixV()*svd.matrixU().transpose();

	if (verbose){
		cout << "R " << endl << R << endl;
		cout << "R's determinant " << R.determinant() << endl;
	}
	if (R.determinant() < 0){
		cout << "warning, R is a reflection " << endl;
		Matrix3d tempI;  tempI.setIdentity();  tempI(2, 2) = -1;
		R = svd.matrixV()*tempI*svd.matrixU().transpose();
	}

	//Vector3d t = tB - R*tA;
	Vector3d tau = tprime - R*t;

	Result.setIdentity();
	for (int r = 0; r < 3; r++){
		for (int c = 0; c < 3; c++){
			Result(r, c) = R(r, c);
		}
		Result(r, 3) = tau(r);
	}
	if (verbose){
		cout << "Result " << endl << Result << endl;

		// assess error ....
		Matrix4d E;

		cout << "Error " << endl;
		for (int i = 0; i < n; i++){
			// Y B - A
			E = Result*LHS[i] - RHS[i];
			cout << E << endl;

		}
	}
}
// camera params should be 12*number_cameras.
void CopyFromCalibration(vector<CameraCali*>& CCV, double* camera_params){
	int number_cameras = CCV.size();

	for (int i = 0; i < number_cameras; i++){
		for (int j = 0; j < 12; j++){
			camera_params[12*i + j] = 0;
		}
	}

	for (int i = 0; i < number_cameras; i++){
		camera_params[12*i] = CCV[i]->internal_parameters(0, 0);
		camera_params[12*i + 1] = CCV[i]->internal_parameters(0, 2);
		camera_params[12*i + 2] = CCV[i]->internal_parameters(1, 1);
		camera_params[12*i + 3] = CCV[i]->internal_parameters(1, 2);


		for (int j = 0; j < 8; j++){
			camera_params[12*i + 4 + j] = 0;
		}

		for (int j = 0; j < CCV[i]->distortion.rows(); j++){
			camera_params[12*i + 4 + j] = CCV[i]->distortion(j);
		}
	}
}

void ReconstructXFunctionIDsMC(MCcali& MC, vector<Matrix4d>& vector_variables, vector<CameraCali*>& CCV, double* camera_params,
		vector< Vector3d >& estimated_threed_points, vector< bool >& has_values, std::ofstream& out){
	// This function solves the optimization problem of estimating the 3D points from 2D points and cameras.

	/// camera params is initialized and static, b/c we're not going to be altering the camera characteristics.
	int max_number_obs = CCV[0]->P_class->three_d_points.size();

	// checked -- this is dealloc'd at the end of the function.
	double* est_threed_vars = new double[max_number_obs*3];

	// need to initialize these ....
	for (int i = 0; i < max_number_obs*3; i++){
		est_threed_vars[i] = 0;
	}

	/// find out where there are at least two observations ...
	vector< bool > found_one(max_number_obs, false);
	has_values.clear();
	has_values.resize(max_number_obs, false);
	int camera_index, pattern_graph_index, pattern_superscript, time_graph_index, time_superscript;

	// will need to dealloc, but this structure will keep everything where I need it while the optimization runs.
	vector<double*> A_hats(MC.NumberSingles(), 0);

	Matrix4d C, P, T, A_hat;

	// The vector of variables has one temporary variable at the end.
	if (MC.NumberVariables() + 1 != int(vector_variables.size())){
		cout << "The size of the vector sent to ReconstructXFunctionIDsMC is " << vector_variables.size() << endl;
		cout << "the number of variables according to MC.NumberVariables() + 1 is " << MC.NumberVariables() + 1<< endl;
		cout << "They should be the same, quitting." << endl;
		exit(1);
	}

	// initialize matrix, so we're not doing that all of the time.
	for (int i  = 0; i < MC.NumberSingles(); i++){
		if (MC.singles_open[i] == false){ /// means we have an initial value for all of the variables.
			// compute an A-hat for this one.
			camera_index = MC.singles[i].lhs;

			pattern_graph_index = MC.singles[i].rhs[1];
			time_graph_index = MC.singles[i].rhs[2];

			time_superscript = MC.V_index.at(time_graph_index);
			pattern_superscript = MC.V_index.at(pattern_graph_index);

			C = vector_variables.at(camera_index);
			P = vector_variables.at(pattern_graph_index);
			T = vector_variables.at(time_graph_index);

			A_hat = C*T.inverse()*P.inverse();

			A_hats[i]= new double[16];

			for (int r = 0, in = 0; r < 4; r++){
				for (int c = 0; c < 4; c++, in++){
					A_hats[i][in] = A_hat(r, c);
				}
			}
		}

	}
	// walk through by number points, estimate the three D points.

	double x_coord, y_coord;

	int two_d_counter = 0;
	int local_time_index = 0;

	for (int k = 0; k < max_number_obs; k++){
		Problem problem;

		two_d_counter = 0;
		for (int i  = 0; i < MC.NumberSingles(); i++){
			if (MC.singles_open[i] == false){ /// means we have an initial value for all of the variables.
				// compute an A-hat for this one.

				camera_index = MC.singles[i].lhs;

				pattern_graph_index = MC.singles[i].rhs[1];
				time_graph_index = MC.singles[i].rhs[2];

				time_superscript = MC.V_index.at(time_graph_index);
				pattern_superscript = MC.V_index.at(pattern_graph_index);

				local_time_index = time_superscript - CCV[camera_index]->start_time_this_camera;

				// is point k here?
				if (k >= CCV[camera_index]->P_class->min_max_id_squares[pattern_superscript].first &&
						k <= CCV[camera_index]->P_class->min_max_id_squares[pattern_superscript].second){ // i.e., is k on this pattern/FR
					if (CCV[camera_index]->points_present[local_time_index].at(k) == true){ // is k detected.

						out << "Camera " << camera_index << " pattern " << pattern_superscript << ", " << time_superscript << endl;

						x_coord = CCV[camera_index]->two_d_point_coordinates_dense[local_time_index](k, 0);
						y_coord = CCV[camera_index]->two_d_point_coordinates_dense[local_time_index](k, 1);

						ceres::CostFunction* cost_function =
								ReconstructXStruct::Create(
										&camera_params[12*camera_index], &A_hats[i][0],
										x_coord, y_coord);
						problem.AddResidualBlock(cost_function,
								NULL /* squared loss */,
								&est_threed_vars[k*3]);

						two_d_counter++;
					}
				}
			}
		}


		if (two_d_counter >= 2){

			has_values[k] = true;
			Solver::Options options;
			options.linear_solver_type = ceres::DENSE_QR;
			options.minimizer_progress_to_stdout = false;
			Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);

			out << endl << k << endl;
			out << summary.FullReport() << endl;
		}
	}

	estimated_threed_points.clear();
	estimated_threed_points.resize(max_number_obs, Vector3d());
	for (int i = 0; i < max_number_obs; i++){
		estimated_threed_points[i](0) = est_threed_vars[3*i];
		estimated_threed_points[i](1) = est_threed_vars[3*i + 1];
		estimated_threed_points[i](2) = est_threed_vars[3*i + 2];
	}

	delete [] est_threed_vars;

	// delete all of the transformations.
	for (int i = 0; i < MC.NumberSingles(); i++){
		if (A_hats[i] != 0){
			delete [] A_hats[i];
		}
	}

	A_hats.clear();
}

void MinimizeReprojectionError(MCcali& MC, vector<CameraCali*>& CCV, double* camera_params,
		vector<bool>& bit_vector_true_min, std::ofstream& out, int start_id, int end_id, bool use_all_points_present){

	/// camera params is initialized and static, b/c we're not going to be altering the camera characteristics.
	Problem problem;

	if (start_id == -1){
		start_id = 0;
		end_id = MC.NumberSingles();
	}

	PARAM_TYPE param_type = Cali_Quaternion;

	int number_variables = MC.NumberVariables();
	double* x = new double[7*number_variables];
	double* vars_initial = new double[16*number_variables];
	bool* bit_vector_bool = new bool[number_variables];


	out << "Minimizing for these variables: " << endl;
	cout << "Minimizing for these variables: " << endl;

	int camera_index, pattern_graph_index, pattern_superscript, time_graph_index, time_superscript;

	for (int i = 0; i < number_variables; i++){
		bit_vector_bool[i] = bit_vector_true_min[i];
	}

	// Will only minimize for these variables....
	for (int i = 0; i < number_variables; i++){
		if (bit_vector_true_min[i] == true){
			out << i << " ";
			cout << i << " ";
		}
	}


	out << endl;
	cout << endl;


	/// quaternion representation
	for (int i = 0; i < number_variables*7; i++){
		x[i] = 0;
	}

	/// matrix rep
	for (int i = 0; i < number_variables*16; i++){
		vars_initial[i] = 0;
	}

	Matrix4d M; Matrix4d Minv;

	// convert everything to relevant parameter type
	for (int i = 0; i < MC.NumberCameras(); i++ ){
		M = MC.V_initial[i];

		switch (param_type){
		case Cali_Quaternion:{
			ConvertMatrixTo7QuaternionRepresentationx(M, &x[7*i]);
		} break;
		default:
		{
			cout << "Option not dealt with," << __LINE__ << " of " << __FILE__ << endl;
			exit(1);
		}
		}


		// copy over
		for (int r = 0, v = 0;  r < 4; r++){
			for (int c = 0; c < 4; c++, v++){
				vars_initial[16*i + v] = M(r, c);
			}
		}

	}

	for (int i = MC.NumberCameras(); i < number_variables; i++ ){
		M = MC.V_initial[i];
		Minv = M.inverse();


		switch (param_type){
		case Cali_Quaternion:{
			ConvertMatrixTo7QuaternionRepresentationx(Minv, &x[7*i]);
		} break;
		default:
			cout << "Option not dealt with, " << __LINE__ << " of " __FILE__ << endl;
			exit(1);
		}

		// copy over
		for (int r = 0, v = 0;  r < 4; r++){
			for (int c = 0; c < 4; c++, v++){
				vars_initial[16*i + v] = Minv(r, c);
			}
		}
	}

	vector<double*> threeDp;
	vector<double*> twoDp;

	double* weight = new double[MC.NumberCameras()];


	for (int i = 0, cn = MC.NumberCameras(); i < cn; i++){
		weight[i] = 1;
	}


	MC.UpdateSinglesOpenFlag();

	int single_counter = 0;

	int local_time_index;

	{

		for (int i  = start_id; i < end_id; i++){
			if (MC.singles_open[i] == false){ /// means we have an initial value for all of the variables.

				if (i %10 == 0){
					cout << "Every 10 output ... Adding equation number.... " << i << endl;
				}

				camera_index = MC.singles[i].lhs;

				pattern_graph_index = MC.singles[i].rhs[1];
				time_graph_index = MC.singles[i].rhs[2];

				time_superscript = MC.V_index.at(time_graph_index);
				pattern_superscript = MC.V_index.at(pattern_graph_index);

				if (use_all_points_present == true){
					local_time_index = time_superscript - CCV[camera_index]->start_time_this_camera;

					int n_points = CCV[camera_index]->PointsAtImagePattern(local_time_index, pattern_superscript); /// image = time
					threeDp.push_back(new double[n_points*3]);
					twoDp.push_back(new double[n_points*2]);

					int s = 0;
					for (int j = CCV[camera_index]->P_class->min_max_id_squares[pattern_superscript].first;
							j <= CCV[camera_index]->P_class->min_max_id_squares[pattern_superscript].second; j++){

						if (CCV[camera_index]->points_present[local_time_index].at(j) == true){

							twoDp[single_counter][2*s] = 0;

							twoDp[single_counter][2*s] = CCV[camera_index]->two_d_point_coordinates_dense[local_time_index](j, 0);    /// twod points w/o blanks is NOT per image to make internal cali work.
							twoDp[single_counter][2*s + 1] = CCV[camera_index]->two_d_point_coordinates_dense[local_time_index](j, 1);

							threeDp[single_counter][3*s] = CCV[camera_index]->P_class->three_d_points[j].x;
							threeDp[single_counter][3*s + 1] = CCV[camera_index]->P_class->three_d_points[j].y;
							threeDp[single_counter][3*s + 2] = CCV[camera_index]->P_class->three_d_points[j].z;

							ceres::CostFunction* cost_function =
									MultiCameraReprojectionError::Create(
											&camera_params[12*camera_index], &twoDp[single_counter][s*2], &threeDp[single_counter][s*3],
											&vars_initial[0], &bit_vector_bool[0],
											param_type, number_variables, camera_index, pattern_graph_index, time_graph_index, &weight[camera_index]);
							double* c_ = x + camera_index*7;
							double* p_ = x + pattern_graph_index*7;
							double* t_ = x + time_graph_index*7;

							problem.AddResidualBlock(cost_function,
									NULL /* squared loss */,
									c_,
									p_,
									t_);

							s++;
						}
					}

				}	else {

					local_time_index = time_superscript - CCV[camera_index]->start_time_this_camera;

					int k_points = CCV[camera_index]->PointsForMinimization(local_time_index, pattern_superscript); /// image = time

					threeDp.push_back(new double[k_points*3]);
					twoDp.push_back(new double[k_points*2]);

					int s = 0;
					for (int j = CCV[camera_index]->P_class->min_max_id_squares[pattern_superscript].first;
							j <= CCV[camera_index]->P_class->min_max_id_squares[pattern_superscript].second; j++){

						if (CCV[camera_index]->points_used_min[local_time_index].at(j) == true){
							twoDp[single_counter][2*s] = CCV[camera_index]->two_d_point_coordinates_dense[local_time_index](j, 0);    /// twod points w/o blanks is NOT per image to make internal cali work.
							twoDp[single_counter][2*s + 1] = CCV[camera_index]->two_d_point_coordinates_dense[local_time_index](j, 1);

							threeDp[single_counter][3*s] = CCV[camera_index]->P_class->three_d_points[j].x;
							threeDp[single_counter][3*s + 1] = CCV[camera_index]->P_class->three_d_points[j].y;
							threeDp[single_counter][3*s + 2] = CCV[camera_index]->P_class->three_d_points[j].z;

							ceres::CostFunction* cost_function =
									MultiCameraReprojectionError::Create(
											&camera_params[12*camera_index], &twoDp[single_counter][s*2], &threeDp[single_counter][s*3],
											&vars_initial[0], &bit_vector_bool[0],
											param_type, number_variables, camera_index, pattern_graph_index, time_graph_index, &weight[camera_index]);

							double* c_ = x + camera_index*7;
							double* p_ = x + pattern_graph_index*7;
							double* t_ = x + time_graph_index*7;

							problem.AddResidualBlock(cost_function,
									NULL /* squared loss */,
									c_,
									p_,
									t_);

							s++;
						}
					}
				}
				single_counter++;
			}
		}
	}


	cout << "Before solver " << endl;
	// Run the solver
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.minimizer_progress_to_stdout = true;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	out << summary.FullReport() << endl;
	std::cout << summary.BriefReport() << "\n";

	cout << "After running solver " << endl;


	/// copy over changed items to variable list.
	for (int i = 0; i < number_variables; i++){
		if (bit_vector_bool[i] == true){ // IOW this was a varable we solved for this time.

			switch (param_type){
			case Cali_Quaternion:{
				Convert7ParameterQuaternionRepresentationIntoMatrix(&x[7*i], M);
			} break;
			}

			if (i < MC.NumberCameras()){
				MC.V_initial[i] = M;
			}	else {
				// T and P
				Minv = M.inverse();

				MC.V_initial[i] = Minv;
			}
		}

	}

	for (int i = 0, in = threeDp.size(); i < in; i++){
		delete [] threeDp[i];
		delete [] twoDp[i];
	}


	delete [] weight;
	delete [] bit_vector_bool;
	delete [] vars_initial;
	delete [] x;

}



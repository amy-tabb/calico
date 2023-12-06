/*
 * solving_structure.cpp
 *
 *  Created on: Jul 16, 2018
 *      Author: Amy Tabb
 */

#include "multicamera.hpp"
#include "camera-visualization.hpp"
#include "helper.hpp"
#include "solving-structure.hpp"
// used 11/27
void SolveWithShahsMethod(Matrix4d& Result, const vector<Matrix4d>& LHS, const vector<Matrix4d>& RHS, bool verbose){

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

// used 11/27
// camera params should be 12*number_cameras.
void CopyFromCalibration(const vector<CameraCali*>& CCV, double* camera_params){

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

// used 11/27
void ReconstructXFunctionIDsMCwithDLTs(MCcali& MC, vector<Matrix4d>& vector_variables,
        const vector<CameraCali*>& CCV, double* camera_params,
        vector< Vector3d >& estimated_threed_points, vector< bool >& has_values, std::ofstream& out){
    // This function solves the optimization problem of estimating the 3D points from 2D points and cameras.

     /// camera params is initialized and static, b/c we're not going to be altering the camera characteristics.
    int max_number_obs = CCV[0]->P_class->three_d_points.size();

    // checked -- this is dealloc'd at the end of the function.
    double* est_threed_vars = new double[max_number_obs*3];

    // need to initialize these .... use the DLT to initialize.
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
    Matrix3d K;

    vector<Matrix4d> KtimesRT(MC.NumberSingles());
    vector<MatrixXd> AhatVec(MC.NumberSingles());

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

            K.setIdentity();
            K = CCV[camera_index]->internal_parameters;

            AhatVec[i] = A_hat.block<3, 4>(0, 0);
            MatrixXd temp = K*A_hat.block<3, 4>(0, 0); // should be a 3x4.
            A_hat.block<3, 4>(0,0) = temp;

            KtimesRT[i] = A_hat;

        }

    }
    // walk through by number points, estimate the three D points.

    double x_coord, y_coord;

    int two_d_counter = 0;
    int local_time_index = 0;
    int number_corners_per = CCV[0]->P_class->NumberCornersPerPattern();
    int start_j, end_j;

    for (int k = 0; k < max_number_obs; k++){
        Problem problem;

        two_d_counter = 0;

        vector<Matrix3d> Kvec;
        vector<MatrixXd> AhatMatrices;
        vector<Matrix4d> PMatrices;
        vector<double> imagePts;
        vector<int> cameraIdxs;
        vector<int> iIdxs;
        for (int i  = 0; i < MC.NumberSingles(); i++){
            if (MC.singles_open[i] == false){ /// means we have an initial value for all of the variables.
                // compute an A-hat for this one.

                camera_index = MC.singles[i].lhs;

                pattern_graph_index = MC.singles[i].rhs[1];
                time_graph_index = MC.singles[i].rhs[2];

                time_superscript = MC.V_index.at(time_graph_index);
                pattern_superscript = MC.V_index.at(pattern_graph_index);

                local_time_index = time_superscript - CCV[camera_index]->start_time_this_camera;

                start_j = number_corners_per*pattern_superscript;
                end_j = number_corners_per*(pattern_superscript + 1);

                if (k >= start_j && k < end_j){
                    if (CCV[camera_index]->points_present[local_time_index].at(k) == true){ // is k detected.

                        out << "Camera " << camera_index << " pattern " << pattern_superscript << ", " << time_superscript << endl;


                        x_coord = CCV[camera_index]->two_d_point_coordinates_dense[local_time_index](k, 0);
                        y_coord = CCV[camera_index]->two_d_point_coordinates_dense[local_time_index](k, 1);

                        imagePts.push_back(x_coord);
                        imagePts.push_back(y_coord);

                        PMatrices.push_back(KtimesRT[i]);
                        AhatMatrices.push_back(AhatVec[i]);
                        Kvec.push_back(CCV[camera_index]->internal_parameters);
                        cameraIdxs.push_back(camera_index);
                        iIdxs.push_back(i);

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


            int m = two_d_counter*2;
            int n = 3;

            //numCamerasThisPoint = 10;
            Matrix3d T(3, 3); T.setIdentity();
            MatrixXd LHS(m, n);
            MatrixXd RHS(m, 1);
            MatrixXd r1(1, 4);
            MatrixXd r2(1, 4);
            LHS.setZero();
            MatrixXd P;
            MatrixXd xcam(3, 1); xcam(2) = 1;
            for (int c = 0; c < two_d_counter; c++){

                T(0,0) = 2.0/float(CCV[cameraIdxs[c]]->cols);
                T(1,1) = 2.0/float(CCV[cameraIdxs[c]]->rows);
                T(0, 2) = -1;
                T(1, 2) = -1;

                P = T*PMatrices[c].block<3, 4>(0,0);
                xcam(0) = imagePts[c*2];
                xcam(1) = imagePts[c*2 + 1];

                xcam = T*xcam;

                r1 = xcam(0)*P.row(2) - P.row(0);
                r2 = xcam(1)*P.row(2) - P.row(1);


                LHS.row(c*2) = r1.block<1, 3>(0, 0);
                LHS.row(c*2 + 1) = r2.block<1, 3>(0,0);

                RHS(c*2, 0) = -r1(3);
                RHS(c*2 + 1, 0) = -r2(3);
            }


            MatrixXd xstar(4, 1); xstar(3) = 1;

            xstar.block<3, 1>(0,0) = LHS.colPivHouseholderQr().solve(RHS);

            est_threed_vars[k*3] = xstar(0);
            est_threed_vars[k*3 + 1] = xstar(1);
            est_threed_vars[k*3 + 2] = xstar(2);

            has_values[k] = true;
            Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = false;
            Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            out << endl << k << endl;
            out << summary.BriefReport() << endl;


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


// used 11/27
CeresProblemClass::CeresProblemClass(PARAM_TYPE parameter_type, MCcali& MC,	std::ofstream& out){

    out << "Setting up the problem in CeresProblemClass. " << endl;
    param_type = parameter_type;

    number_variables = MC.NumberVariables();
    number_equations = MC.NumberSingles();

    x = new double[7*number_variables]; // vars to solve for via LM

    bit_vector_bool = new bool[number_variables]; // bit vector which vars we are solving for, versus not.
    bit_vector_equations = new bool[number_equations];

    for (int i = 0; i < number_variables; i++){
        bit_vector_bool[i] = false;
    }

    for (int i = 0; i < number_equations; i++){
        bit_vector_equations[i] = false;
    }

    /// quaternion representation
    for (int i = 0; i < number_variables*7; i++){
        x[i] = 0;
    }
}

// used 11/27
void CeresProblemClass::SetUpXForReprojectionError(MCcali& MC){

    Matrix4d M;
    Matrix4d Minv;

    for (int v_index = 0, vn = MC.NumberVariables(); v_index < vn; v_index++){

        if (v_index < MC.NumberCameras()){
            M = MC.V_initial[v_index];

            switch (param_type){
            case Cali_Quaternion:{
                ConvertMatrixTo7QuaternionRepresentationx(M, &x[7*v_index]);
            } break;
            default:
            {
                cout << "Option not dealt with," << __LINE__ << " of " << __FILE__ << endl;
                exit(1);
            }
            }
        }   else {
            // need to use the inverses for these variables.
            M = MC.V_initial[v_index];
            Minv = M.inverse();


            switch (param_type){
            case Cali_Quaternion:{
                ConvertMatrixTo7QuaternionRepresentationx(Minv, &x[7*v_index]);
            } break;
            default:
                cout << "Option not dealt with, " << __LINE__ << " of " __FILE__ << endl;
                exit(1);
            }
        }
    }


}

// used 11/27
void CeresProblemClass::AddEqsToProblemReprojectionError(MCcali& MC, vector<CameraCali*>& CCV, double* camera_params, int start_eq_index,
        int end_eq_index, const vector<int>& equation_ordering){


    int camera_index, pattern_graph_index, pattern_superscript, time_graph_index, time_superscript;
    int number_corners_per, start_j, end_j;

    // convert over those that have been updated in between the bool vectors; update bool vector.  Add these equations with these vars to
    // keep track of which equations have already been added to the problem so they are not added twice.

    vector<int> new_vars;

    int e = -1;

    int local_time_index;

    double im_x, im_y;
    cv::Point3f threeDpt;
    Matrix4d M; Matrix4d Minv;

    const int p_star_graph = MC.pattern_a_vars[MC.p_star()];
    const int t_star_graph = MC.time_f_vars[MC.t_star()];

    int function_case = -1;

    vector<ceres::CostFunction*> case_0_cost_functions;
    vector< double* > case_0_c;
    vector< double* > case_0_p;
    vector< double* > case_0_t;

    vector<ceres::CostFunction*> case_1_cost_functions;
    vector< double* > case_1_c;
    vector< double* > case_1_t;

    vector<ceres::CostFunction*> case_2_cost_functions;
    vector< double* > case_2_c;
    vector< double* > case_2_p;

    vector<ceres::CostFunction*> case_3_cost_functions;
    vector< double* > case_3_c;

    // this will rarely be exploited to run in parallel.  Can we unwrap it?

#pragma omp parallel for private(e, camera_index, pattern_graph_index, time_graph_index, time_superscript, pattern_superscript, number_corners_per, function_case, start_j, end_j, local_time_index, im_x, im_y, threeDpt)
    for (int eq_index  = start_eq_index; eq_index < end_eq_index; eq_index++)
    {
        e = equation_ordering[eq_index];

        /// means we have an initial value for all of the variables.

        camera_index = MC.singles[e].lhs;

        pattern_graph_index = MC.singles[e].rhs[1];
        time_graph_index = MC.singles[e].rhs[2];

        time_superscript = MC.V_index.at(time_graph_index);
        pattern_superscript = MC.V_index.at(pattern_graph_index);

        local_time_index = time_superscript - CCV[camera_index]->start_time_this_camera;

        number_corners_per = CCV[camera_index]->P_class->NumberCornersPerPattern();

        start_j = number_corners_per*pattern_superscript;
        end_j = number_corners_per*(pattern_superscript + 1);

        function_case = -1;

        if (( pattern_graph_index != p_star_graph) && (time_graph_index != t_star_graph)){
            function_case = 0;
        }

        if (( pattern_graph_index == p_star_graph) && (time_graph_index != t_star_graph)){
            function_case = 1;
        }

        if (( pattern_graph_index != p_star_graph) && (time_graph_index == t_star_graph)){
            function_case = 2;
        }

        if (( pattern_graph_index == p_star_graph) && (time_graph_index == t_star_graph)){
            function_case = 3;
        }

        double* c_ = x + camera_index*7;
        double* p_ = x + pattern_graph_index*7;
        double* t_ = x + time_graph_index*7;

        for (int j = start_j; j < end_j; j++){


            // and add those that are used on *this* pattern
            if (CCV[camera_index]->points_used_min[local_time_index].at(j) == true){ // present b/c always used selected k

                im_x = CCV[camera_index]->two_d_point_coordinates_dense[local_time_index](j, 0);    /// twod points w/o blanks is NOT per image to make internal cali work.
                im_y = CCV[camera_index]->two_d_point_coordinates_dense[local_time_index](j, 1);

                threeDpt = CCV[camera_index]->P_class->three_d_points[j];


                switch (function_case){
                case 0 : {
                    ceres::CostFunction* cost_function = MultiCameraReprojectionError::Create(
                            &camera_params[12*camera_index], im_x, im_y, threeDpt,
                            param_type);

#pragma omp critical
                    {
                        case_0_cost_functions.push_back(cost_function);
                        case_0_c.push_back(c_);
                        case_0_t.push_back(t_);
                        case_0_p.push_back(p_);
                    }


                } break;
                case 1: {
                    ceres::CostFunction* cost_function =
                            MultiCameraReprojectionErrorCamTime::Create(
                                    &camera_params[12*camera_index], im_x, im_y, threeDpt,
                                    param_type);

#pragma omp critical
                    {
                        case_1_cost_functions.push_back(cost_function);
                        case_1_c.push_back(c_);
                        case_1_t.push_back(t_);
                    }

                } break;
                case 2: {
                    ceres::CostFunction* cost_function =
                            MultiCameraReprojectionErrorCamPattern::Create(
                                    &camera_params[12*camera_index], im_x, im_y, threeDpt,
                                    param_type);

#pragma omp critical
                    {
                        case_2_cost_functions.push_back(cost_function);
                        case_2_c.push_back(c_);
                        case_2_p.push_back(p_);
                    }

                } break;
                case 3: {
                    ceres::CostFunction* cost_function =
                            MultiCameraReprojectionErrorCam::Create(
                                    &camera_params[12*camera_index], im_x, im_y, threeDpt,
                                    param_type);


#pragma omp critical
                    {
                        case_3_cost_functions.push_back(cost_function);
                        case_3_c.push_back(c_);
                    }


                } break;
                default: {
                    cout << "We don't have this implemented." << __LINE__ << " in " <<  __FILE__ << endl;
                    exit(1);
                }



                }
            }
        }
    }

    int number_case_0 = case_0_cost_functions.size();
    int number_case_1 = case_1_cost_functions.size();
    int number_case_2 = case_2_cost_functions.size();
    int number_case_3 = case_3_cost_functions.size();

    for (int i = 0; i < number_case_0; i++){
        problemRP.AddResidualBlock(case_0_cost_functions[i],
                NULL /* squared loss */,
                case_0_c[i],
                case_0_p[i],
                case_0_t[i]);

    }

    cout << "add case 1s RP " << endl;
    for (int i = 0; i < number_case_1; i++){
        problemRP.AddResidualBlock(case_1_cost_functions[i],
                NULL /* squared loss */,
                case_1_c[i],
                case_1_t[i]);
    }

    cout << "add case 2s RP " << endl;
    for (int i = 0; i < number_case_2; i++){
        problemRP.AddResidualBlock(case_2_cost_functions[i],
                NULL /* squared loss */,
                case_2_c[i],
                case_2_p[i]);
    }

    cout << "add case 3s RP " << endl;
    for (int i = 0; i < number_case_3; i++){
        problemRP.AddResidualBlock(case_3_cost_functions[i],
                NULL /* squared loss */,
                case_3_c[i]);
    }
}

// used 11/27
// return int is the number of equations added.
int CeresProblemClass::AddToProblemAlgebraicError(MCcali& MC, vector<int>& solved_vars, vector<int>& equations_added,
        std::ofstream& out, int vars_to_process){


    if (a_vectors.size() == 0){
        // create.
        for (int i = 0; i < number_equations; i++){
            a_vectors.push_back(new double[7]);

            switch (param_type){
            case Cali_Quaternion:{
                ConvertMatrixTo7QuaternionRepresentationx(MC.A[i], a_vectors[i]);
            } break;
            default:
            {
                cout << "Option not dealt with," << __LINE__ << " of " << __FILE__ << endl;
                exit(1);
            }
            }
        }


    }

    int camera_index, pattern_graph_index, time_graph_index;

    // convert over those that have been updated in between the bool vectors; update bool vector.  Add these equations with these vars to
    // keep track of which equations have already been added to the problem so they are not added twice.

    vector<int> new_vars;
    vector<int> new_eqs;
    int num_eqs_add = 0;
    int num_vars_add = 0;
    int e = -1;  int v_index = -1;

    cv::Point3f threeDpt;
    Matrix4d M; Matrix4d Minv;

    const int p_star_graph = MC.pattern_a_vars[MC.p_star()];
    const int t_star_graph = MC.time_f_vars[MC.t_star()];

    out << "New vars in algebraic error " ; /// DO NOT CHANGE THE FLAGS OF P-STAR AND T-STAR

    // walk through using solved vars.
    int start_index = solved_vars.size() - vars_to_process;
    int end_index = solved_vars.size();
    for (int i = start_index; i < end_index; i++){
        bit_vector_bool[solved_vars[i]] = true;
        new_vars.push_back(solved_vars[i]);
        out << solved_vars[i] << " ";
        cout << "Adding " << solved_vars[i] << endl;
    }

    out << endl;


    MC.UpdateSinglesOpenFlag();

    /// compare singles open with the current equation list.
    for (int i = 0; i < number_equations; i++){
        if (MC.singles_open[i] == false && bit_vector_equations[i] == false){
            // for MC, false means everything is initialized.  for bit_vector_equations, false means we can't add to the cost
            // function b/c not everything is initialized.

            bit_vector_equations[i] = true;
            new_eqs.push_back(i);
            equations_added.push_back(i);
            out << "Will add eq " << i << endl;
        }
    }

    num_vars_add = new_vars.size();


    for (int var_index = 0; var_index < num_vars_add; var_index++){
        v_index = new_vars[var_index];

        // nothing is inverted in this function.
        M = MC.V_initial[v_index];

        switch (param_type){
        case Cali_Quaternion:{
            ConvertMatrixTo7QuaternionRepresentationx(M, &x[7*v_index]);
        } break;
        default:
        {
            cout << "Option not dealt with," << __LINE__ << " of " << __FILE__ << endl;
            exit(1);
        }
        }
    }

    int function_case = -1;

    vector<ceres::CostFunction*> case_0_cost_functions;
    vector< double* > case_0_c;
    vector< double* > case_0_p;
    vector< double* > case_0_t;

    vector<ceres::CostFunction*> case_1_cost_functions;
    vector< double* > case_1_c;
    vector< double* > case_1_t;

    vector<ceres::CostFunction*> case_2_cost_functions;
    vector< double* > case_2_c;
    vector< double* > case_2_p;

    vector<ceres::CostFunction*> case_3_cost_functions;
    vector< double* > case_3_c;

    num_eqs_add = new_eqs.size();


#pragma omp parallel for private(e, camera_index, pattern_graph_index, time_graph_index, function_case)
    for (int eq_index  = 0; eq_index < num_eqs_add; eq_index++){
        e = new_eqs[eq_index];

        /// means we have an initial value for all of the variables.

        camera_index = MC.singles[e].lhs;

        pattern_graph_index = MC.singles[e].rhs[1];
        time_graph_index = MC.singles[e].rhs[2];

        function_case = -1;

        if (( pattern_graph_index != p_star_graph) && (time_graph_index != t_star_graph)){
            function_case = 0;
        }

        if (( pattern_graph_index == p_star_graph) && (time_graph_index != t_star_graph)){
            function_case = 1;
        }

        if (( pattern_graph_index != p_star_graph) && (time_graph_index == t_star_graph)){
            function_case = 2;
        }

        if (( pattern_graph_index == p_star_graph) && (time_graph_index == t_star_graph)){
            function_case = 3;
        }

        double* c_ = x + camera_index*7;
        double* p_ = x + pattern_graph_index*7;
        double* t_ = x + time_graph_index*7;

        switch (function_case){
        case 0 : {

            ceres::CostFunction* cost_function = AlgebraicErrorCamPatternTime::Create(
                    a_vectors[e], param_type);

#pragma omp critical
            {
                case_0_cost_functions.push_back(cost_function);
                case_0_c.push_back(c_);
                case_0_t.push_back(t_);
                case_0_p.push_back(p_);
            }

        } break;
        case 1 : {
            ceres::CostFunction* cost_function = AlgebraicErrorCamTime::Create(
                    a_vectors[e], param_type);


#pragma omp critical
            {
                case_1_cost_functions.push_back(cost_function);
                case_1_c.push_back(c_);
                case_1_t.push_back(t_);
            }


        } break;
        case 2 : {
            ceres::CostFunction* cost_function = AlgebraicErrorCamPattern::Create(
                    a_vectors[e], param_type);

#pragma omp critical
            {
                case_2_cost_functions.push_back(cost_function);
                case_2_c.push_back(c_);
                case_2_p.push_back(p_);
            }

        } break;
        case 3 : {
            ceres::CostFunction* cost_function = AlgebraicErrorCam::Create(
                    a_vectors[e], param_type);

#pragma omp critical
            {
                case_3_cost_functions.push_back(cost_function);
                case_3_c.push_back(c_);
            }

        } break;
        default: {
            cout << "We don't have this implemented." << __LINE__ << endl;
            exit(1);
        }
        }
    }

    int number_case_0 = case_0_cost_functions.size();
    int number_case_1 = case_1_cost_functions.size();
    int number_case_2 = case_2_cost_functions.size();
    int number_case_3 = case_3_cost_functions.size();

    for (int i = 0; i < number_case_0; i++){
        problem.AddResidualBlock(case_0_cost_functions[i],
                NULL /* squared loss */,
                case_0_c[i],
                case_0_p[i],
                case_0_t[i]);

    }

    for (int i = 0; i < number_case_1; i++){
        problem.AddResidualBlock(case_1_cost_functions[i],
                NULL /* squared loss */,
                case_1_c[i],
                case_1_t[i]);
    }

    for (int i = 0; i < number_case_2; i++){
        problem.AddResidualBlock(case_2_cost_functions[i],
                NULL /* squared loss */,
                case_2_c[i],
                case_2_p[i]);
    }

    for (int i = 0; i < number_case_3; i++){
        problem.AddResidualBlock(case_3_cost_functions[i],
                NULL /* squared loss */,
                case_3_c[i]);
    }



    return new_eqs.size();
}

// used 11/27
void CeresProblemClass::SolveWriteBackToMCAlgebraicError(MCcali& MC, std::ofstream& out, int iterations, bool output_to_terminal){

    Matrix4d M; Matrix4d Minv;

    MC.TestSolutionHasIdentityPstarTstar(MC.V_initial);


    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    options.minimizer_progress_to_stdout = output_to_terminal;
    options.num_threads = omp_get_max_threads();
    options.max_num_iterations = iterations;

    cout << "Number threads .... " << options.num_threads << endl;

    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (output_to_terminal){
        out << summary.BriefReport() << endl;
        std::cout << summary.BriefReport() << "\n";
        cout << "After running solver " << endl;
    }   else {
        out << summary.BriefReport() << endl;

    }


    /// copy over changed items to variable list.
    for (int i = 0; i < number_variables; i++){
        if (bit_vector_bool[i] == true){ // IOW this was a varable we solved for this time.  could also do this with the list of variables.

            switch (param_type){
            case Cali_Quaternion:{
                Convert7ParameterQuaternionRepresentationIntoMatrix(&x[7*i], M);
            } break;
            }

            // no inverses.
            MC.V_initial[i] = M;
        }
    }

    MC.TestSolutionHasIdentityPstarTstar(MC.V_initial);

}

// used 11/27
void CeresProblemClass::SolveWriteBackToMCRP(MCcali& MC, std::ofstream& out, int iterations, vector<int>& solved_vars, bool output_to_terminal){

    Matrix4d M; Matrix4d Minv;

    MC.TestSolutionHasIdentityPstarTstar(MC.V_initial);

    Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;

    options.minimizer_progress_to_stdout = output_to_terminal;
    options.num_threads = omp_get_max_threads();
    options.max_num_iterations = iterations;

    Solver::Summary summary;
    ceres::Solve(options, &problemRP, &summary);

    if (output_to_terminal){
        out << summary.BriefReport() << endl;
        std::cout << summary.BriefReport() << "\n";
        cout << "After running solver " << endl;
    }   else {
        out << summary.BriefReport() << endl;

    }


    int nsv = solved_vars.size();

    int current_var = 0;
    /// copy over changed items to variable list.
    for (int i = 0; i <nsv; i++){

        current_var = solved_vars[i];


        switch (param_type){
        case Cali_Quaternion:{
            Convert7ParameterQuaternionRepresentationIntoMatrix(&x[7*current_var], M);
        } break;
        }

        if (current_var < MC.NumberCameras()){
            MC.V_initial[current_var] = M;
        }   else {
            // T and P
            Minv = M.inverse();

            MC.V_initial[current_var] = Minv;
        }


    }

    MC.TestSolutionHasIdentityPstarTstar(MC.V_initial);

}

//used 11/27
CeresProblemClass::~CeresProblemClass(){

    if (x != 0){
        delete [] x;
        x = 0;
    }

    if (bit_vector_bool != 0){
        delete [] bit_vector_bool;
        bit_vector_bool = 0;
    }

    if (bit_vector_equations != 0){
        delete [] bit_vector_equations;
        bit_vector_equations = 0;
    }


    if (a_vectors.size() != 0){
        for (uint i = 0; i < a_vectors.size(); i++){
            delete [] a_vectors[i];
        }

        a_vectors.clear();
    }
}

// used 11/27
void XASolveIteratively(vector<Matrix4d>& As, Matrix4d& X, PARAM_TYPE param_type,
        int max_iterations){


    // assume that X is initialized


    Problem problem;
    int number_eqs = As.size();

    double* x = new double[7];

    double* a = new double[7*number_eqs];

    switch (param_type){
    case Cali_Quaternion:{
        ConvertMatrixTo7QuaternionRepresentationx(X, x);
        for (int i = 0; i < number_eqs; i++){
            ConvertMatrixTo7QuaternionRepresentationx(As[i], &a[7*i]);
        }
    } break;
    default: {
        cout << "Other parameterizations not implemented " << __LINE__ << " in " << __FILE__ << endl;
        exit(1);
    }

    }

    vector<double*> parameter_vector;
    vector<ceres::CostFunction*> cost_function_vector;


    for (int i = 0; i < number_eqs; i++){

        double* currenta_ = a + i*7;

        ceres::CostFunction* cost_function = AXAlgebraicError::Create(
                currenta_, param_type);

#pragma omp critical
        {
            cost_function_vector.push_back(cost_function);
        }

    }

    for (int i = 0; i < number_eqs; i++){

        problem.AddResidualBlock(cost_function_vector[i],
                NULL /* squared loss */,
                x);

    }

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;

    options.minimizer_progress_to_stdout = false;
    options.num_threads = omp_get_max_threads();
    options.max_num_iterations = max_iterations;

    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    switch (param_type){
    case Cali_Quaternion:{
        Convert7ParameterQuaternionRepresentationIntoMatrix(x, X);
    } break;
    }

    delete [] x;
    delete [] a;
}

// used 11/27
void AXZBSolveIteratively(vector<Matrix4d>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z,  PARAM_TYPE param_type,
        int max_iterations){

    // assume that X and Z are initialized.

    Problem problem;
    int number_eqs = As.size();

    double* x = new double[7];
    double* z = new double[7];

    double* a = new double[7*number_eqs];
    double* b = new double[7*number_eqs];

    switch (param_type){
    case Cali_Quaternion:{
        ConvertMatrixTo7QuaternionRepresentationx(X, x);
        ConvertMatrixTo7QuaternionRepresentationx(Z, z);
        for (int i = 0; i < number_eqs; i++){
            ConvertMatrixTo7QuaternionRepresentationx(As[i], &a[7*i]);
            ConvertMatrixTo7QuaternionRepresentationx(Bs[i], &b[7*i]);
        }
    } break;
    default: {
        cout << "Other parameterizations not implemented " << __LINE__ << " in " << __FILE__ << endl;
        exit(1);
    }

    }

    for (int i = 0; i < number_eqs; i++){

        double* currenta_ = a + i*7;
        double* currentb_ = b + i*7;

        ceres::CostFunction* cost_function = AXZBAlgebraicError::Create(
                currenta_, currentb_, param_type);

        problem.AddResidualBlock(cost_function,
                NULL /* squared loss */,
                x,
                z);

    }

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;

    options.minimizer_progress_to_stdout = true;
    options.num_threads = omp_get_max_threads();
    options.max_num_iterations = max_iterations;

    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    switch (param_type){
    case Cali_Quaternion:{
        Convert7ParameterQuaternionRepresentationIntoMatrix(x, X);
        Convert7ParameterQuaternionRepresentationIntoMatrix(z, Z);
    } break;
    }

    delete [] x;
    delete [] z;
    delete [] a;
    delete [] b;
}

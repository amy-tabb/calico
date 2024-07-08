/*
 * multicamera.cpp
 *
 *  Created on: Jul 6, 2018
 *      Author: Amy Tabb
 */



#include "multicamera.hpp"
#include "camera-visualization.hpp"
#include "helper.hpp"
#include "solving-structure.hpp"
//used
single_relationship_container::single_relationship_container(int l){

    lhs = l;
    rhs.resize(3, -1); /// initialize to Identity;
}

//used
// CCV objects are updated, though not added to.
MCcali::MCcali(vector<CameraCali*>& CCV, const PatternsCreated& P_class, int max_number_images_to_use,
        ofstream& out, int synchronized_rotating_option){

    synch_rot_option = synchronized_rotating_option;
    number_images = max_number_images_to_use;

    pn = P_class.pp.numberBoards;
    vector<int> p_count(pn, 0);
    cn = CCV.size();

    /// first, find out if number images is correct.
    if (synch_rot_option == true){
        for (int c_count = 0; c_count < cn; c_count++){
            if ((number_images == -1) ||  (CCV[c_count]->number_external_images_max < number_images)){
                number_images = CCV[c_count]->number_external_images_max;
            }
        }

        tn = number_images;
    }   else {

        int start_time = 0;

        for (int i = 0, nc = CCV.size(); i < nc; i++){
            CCV[i]->start_time_this_camera = start_time;
            start_time += CCV[i]->number_external_images_max;
        }

        tn = start_time;
    }

    vector<vector<int> > t_count;
    int max_value = 0; int max_index = -1;


    if (CCV.size() >= 1){

        t_count.resize(tn, vector<int>(pn, 0));
    }   else {
        cout << "Number of cameras is empty!" << endl;
        exit(1);
    }


    if (synch_rot_option == true){ // default case
        for (int c_count = 0; c_count < cn; c_count++){
            for (int i = 0, in = tn; i < in; i++){
                for (int p = 0; p < pn; p++){
                    if ( CCV[c_count]->has_calibration_estimate[i][p] == true){
                        p_count[p]++;
                        t_count[i][p]++;
                    }
                }
            }
        }

    }   else {
        for (int c_count = 0; c_count < cn; c_count++){
            // iterate over the number of images
            for (int i = 0, in = CCV[c_count]->number_external_images_max; i < in; i++){
                for (int p = 0; p < pn; p++){
                    if ( CCV[c_count]->has_calibration_estimate[i][p] == true){
                        p_count[p]++;
                        // locate the index wrt time.
                        t_count[i + CCV[c_count]->start_time_this_camera][p]++;
                    }
                }
            }
        }
    }

    // which one has the most observations?
    out << "Pattern frequencies : " << endl;
    for (int p = 0; p < pn; p++){
        out << p_count[p] <<  " ";
        if (p_count[p] > max_value){
            max_value = p_count[p];  max_index = p;
        }
    }

    out << endl;

    p_s = max_index;


    max_value = 0;
    max_index = -1;

    for (int i = 0; i < tn; i++){
        if (t_count[i][p_s] > max_value){
            max_value = t_count[i][p_s];
            max_index = i;
        }
    }

    // fastest convergence when the protocol is selected by max, but this is not necessary.
    t_s = max_index;


    out << "Max number of occurrences for t " << max_value << endl << endl;

    out << "p_star and t_star " << p_s << " " << t_s << endl;
    out << "cn " << cn << endl;
    out << "pn " << pn << endl;
    out << "tn " << tn << endl;


    an = 0; vn = 0;  uninitialized = 0;
    number_valid_points_rae_multi = -1;
    number_valid_points_rae_strict = -1;

}
//used
int MCcali::NumberPatterns() const{

    return pn;
}
int MCcali::NumberVariables() const {

    return vn;
}

int MCcali::NumberSingles() const{

    return an;
}

int MCcali::NumberCameras() const{

    return cn;
}

//used
int MCcali::NumberUninitialized(){

    uninitialized = 0;
    for (int i =0; i < vn; i++){
        uninitialized += !V_has_initialization[i];
    }

    return uninitialized;
}

// used 11/27
int MCcali::RemainingToFillIn(){

    int count = 0;
    for (int i =0; i < vn; i++){
        count += (!V_has_initialization[i]) && (V_in_foundational_relationships[i]);
    }

    return count;
}
// used 11/27
int MCcali::p_star() const{

    return p_s;
}
// used 11/27
int MCcali::t_star() const{

    return t_s;
}

// used 11/27
bool MCcali::AllInitialized(const vector<int>& side){

    for (int j = 1; j < 3; j++){
        if (V_has_initialization[side[j]] == false){
            return false;
        }
    }

    return true;
}

// used 11/27
int MCcali::BuildCostFunctionAndGraphWithTestAndDegenerateInitialize(const vector<CameraCali*>& CCV,
        ofstream& out, const string& write_dir, bool write_docs){

    if (synch_rot_option == true){

        // network option.
        /// A_camera_indices here
        for (int c_count = 0; c_count < cn; c_count++){
            for (int i = 0, in = CCV[c_count]->number_external_images_max; i < in; i++){
                for (int p = 0; p < pn; p++){
                    if (CCV[c_count]->has_calibration_estimate[i][p] == true){
                        A_camera_indices.push_back(c_count);
                        A_pattern_indices.push_back(p);
                        A_time_indices.push_back(i);
                        A_number_occurrences.push_back(0); /// this is added later .... when we assemble the cost function.
                        A.push_back(CCV[c_count]->external_parameters[i][p]);
                    }
                }
            }
        }



    }   else {
        // the cameras are not synchronized.

        /// A_camera_indices here
        int A_counter = 0;
        for (int c_count = 0; c_count < cn; c_count++){
            for (int i = 0, in = CCV[c_count]->number_external_images_max; i < in; i++){
                for (int p = 0; p < pn; p++){
                    if (CCV[c_count]->has_calibration_estimate[i][p] == true){
                        A_camera_indices.push_back(c_count);
                        A_pattern_indices.push_back(p);
                        A_time_indices.push_back(i + CCV[c_count]->start_time_this_camera);
                        A_number_occurrences.push_back(0); /// this is added later .... when we assemble the cost function.
                        A.push_back(CCV[c_count]->external_parameters[i][p]);
                        A_counter++;
                    }
                }
            }
        }
    }

    an = A.size();
    out << "The total number of A variables is: " << an << endl;

    // write the As.
    string Afile = write_dir + "/A.txt";
    ofstream Awrite;
    Awrite.open(Afile.c_str());
    for (int i = 0; i < an; i++){
        Awrite << A[i] << endl;
    }
    Awrite.close();


    // Now build out the variable indices.
    /// cn Cs
    //   pn Ps
    //   tn Ts
    for (int i  = 0; i < cn; i++){
        camera_i_vars.push_back(V_type.size());
        V_type.push_back(camera_var);
        V_index.push_back(i);

    }

    for (int i  = 0; i < pn; i++){
        pattern_a_vars.push_back(V_type.size());
        V_type.push_back(pattern_var);
        V_index.push_back(i);
    }

    for (int i  = 0; i < tn; i++){
        time_f_vars.push_back(V_type.size());
        V_type.push_back(time_var);
        V_index.push_back(i);
    }


    V_index.push_back(-1);  // if we ever try to get the superscript out of the special placeholder var, we'll get -1 out, because this is just a placeholder.

    Matrix4d I;  I.setIdentity();
    vn = V_type.size();
    V_has_initialization.resize(vn + 1, false);
    V_in_foundational_relationships.resize(vn, false);
    V_number_occurrences.resize(vn, 0);
    V_initial.resize(vn + 1, I);

    V_has_initialization[vn] = true;
    V_type.push_back(special_var);

    out << "The total number of V variables is " << vn << endl;

    /// build the singles, for each A.

    for (int a_count = 0; a_count < an; a_count++){
        /// camera var index;
        singles.push_back(single_relationship_container(A_camera_indices[a_count]));
        // A var index
        singles[a_count].rhs[0]= a_count;
        /// P var index
        singles[a_count].rhs[1] = pattern_a_vars[A_pattern_indices[a_count]];
        /// T var index
        singles[a_count].rhs[2] = time_f_vars[A_time_indices[a_count]];
        A_number_occurrences[a_count]++;



        // camera var
        V_in_foundational_relationships[A_camera_indices[a_count]] = true;
        // pattern var
        V_in_foundational_relationships[pattern_a_vars[A_pattern_indices[a_count]]] = true;
        // time var
        V_in_foundational_relationships[time_f_vars[A_time_indices[a_count]]] = true;

    }

    InitializeNumberOccurrencesAndInitialization();
    SumOccurrencesSingles();



    vector<int> connected_component_labels(NumberVariables(), 0);

    int cc_label = 1;
    int number_in, number_out;
    vector<bool> still_check(an, true);
    if (an > 0){
        connected_component_labels[singles[0].lhs] = cc_label;

        connected_component_labels[singles[0].rhs[1]] = cc_label;

        connected_component_labels[singles[0].rhs[2]] = cc_label;

        still_check[0] = false;

        bool change = true;

        while (change){

            change = false;

            // walk through the FRs until there is no change.
            for (int i = 0; i < an; i++){

                if (still_check[i]){

                    // change is one of the vars is in component, and one out.
                    number_in = 0; number_out = 0;

                    connected_component_labels[singles[i].lhs] == cc_label ? number_in++ : number_out++;
                    connected_component_labels[singles[i].rhs[1]] == cc_label ? number_in++ : number_out++;
                    connected_component_labels[singles[i].rhs[2]] == cc_label ? number_in++ : number_out++;

                    if (number_in > 0 ){
                        // set all to in component.
                        connected_component_labels[singles[i].lhs] = cc_label;

                        connected_component_labels[singles[i].rhs[1]] = cc_label;

                        connected_component_labels[singles[i].rhs[2]] = cc_label;

                        still_check[i] = false;
                        change = true;
                    }
                }

            }

        }
    }

    // need to check that all of the cameras are in the same connected component.
    int sum_not_in = 0;

    for (int c = 0; c < cn; c++){
        if (connected_component_labels[c] == 0){
            sum_not_in++;
        }
    }

    uninitialized = 0;
    for (int i = 0; i < NumberVariables(); i++){
        if (!V_has_initialization[i]){
            uninitialized++;
        }
    }

    return sum_not_in;
}
// used 11/27
bool MCcali::CanSolveSystem(){

    int count = 0;
    for (int i =0; i < cn; i++){
        count += (!V_has_initialization[i]) && (V_in_foundational_relationships[i]);
    }

    return count == 0;
}
// used 11/27
void MCcali::CreateTAequalsBproblem(int var_to_solve, const vector<int>& rel_equations, vector<Matrix4d>& LHS, vector<Matrix4d>& RHS, bool verbose){

    // at the conclusion, can solve this with Shah or ceres method or whatever else.
    //Format is VAR Known_Constant_LHS = Known_Constant_RHS

    LHS.clear();
    RHS.clear();

    int var_type  = -1;

    Matrix4d lhs; Matrix4d rhs;
    Matrix4d temp_lhs; Matrix4d temp_rhs;
    lhs.setIdentity();
    int n_equations = rel_equations.size();

    LHS.resize(n_equations, lhs);
    RHS.resize(n_equations, lhs);

    /// we can tell where the var will be by its type.
    if (var_to_solve < cn){
        var_type = 0;
    }   else {
        if (var_to_solve < cn + pn){
            var_type = 1;
        }   else {
            var_type = 2;
        }
    }

    if (verbose){
        cout << "Var type " << var_type << endl;
    }

    int equation_number;
    for (int en = 0, enn = rel_equations.size(); en < enn; en++){

        equation_number = rel_equations[en];

        if (equation_number < an){
            // singles

            switch (var_type){
            case 0 : {
                rhs = A[equation_number]*V_initial[singles[equation_number].rhs[1]] *V_initial[singles[equation_number].rhs[2]];
                lhs.setIdentity();
            } break;
            case 1 : {
                rhs = A[equation_number].inverse().eval()*V_initial[singles[equation_number].lhs];
                lhs = V_initial[singles[equation_number].rhs[2]];
            } break;
            case 2 : {
                lhs.setIdentity();
                rhs = (A[equation_number]*V_initial[singles[equation_number].rhs[1]]).inverse().eval()*V_initial[singles[equation_number].lhs];
                if (verbose){
                    cout << "single rhs " << equation_number << endl << rhs << endl;
                }
            } break;
            }
        }   else {
            cout << "ERROR shouldn't be getting doubles in Create TA equals B problem" << endl;
            exit(1);
        }

        LHS[en] = lhs;
        RHS[en] = rhs;
    }

    if (verbose){
        cout << "LHS, RHS " << endl;
        for (int i = 0; i < n_equations; i++){
            cout << "Eq number " << rel_equations[i] << endl;
            cout << LHS[i] << endl;
            cout << RHS[i] << endl;
        }
    }

}
// used 11/27
void MCcali::InitializeNumberOccurrences(){

    TestSolutionHasIdentityPstarTstar(V_initial);

    for (int i = 0; i < vn; i++){
        V_number_occurrences[i] = 0;
    }

    TestSolutionHasIdentityPstarTstar(V_initial);
}

// used 11/27
void MCcali::InitializeNumberOccurrencesAndInitialization(){

    for (int i = 0; i < an; i++){
        A_number_occurrences[i] = 0;
    }

    // here!
    for (int i = 0; i < vn; i++){
        V_number_occurrences[i] = 0;
        V_has_initialization[i] = false;
    }
}

bool MCcali::IsPstar(int p) const{

    return p == p_s;
}

bool MCcali::IsTstar(int t) const{

    return t == t_s;
}
// used 11/27
bool MCcali::IterativelySolveForVariables2(ofstream& out, bool verbose, vector<int>& solved_vars,
        const bool interleavedSolve, int max_iterations){

    InitializeNumberOccurrences();

    int number_singles_open = 0;
    int number_doubles_open = 0;
    int number_equations_to_solve = 0;
    int number_uninitialized = 0;

    vector<int> position_init_map(4, 0);
    vector<int> current_variables(4, 0); // for singles and doubles

    vector<vector<int> > equations_per_var(vn, vector<int>()); // rebuild every iteration.

    // need to reset this every time -- can shift with every solve.
    for (int i = 0; i < an; i++){
        equation_to_solve[i] = false;
    }

    vector<double> average_var_uncertainity_score(vn, 0);


    // prior approach using frequency
    for (int i = 0; i < an; i++){
        if (singles_open[i] == true){
            number_uninitialized = 0;
            // see how many vars are left -- have all been initialized? -- then set singles open = false and equations to solve = false.

            for (int j =0 ; j < 3; j++){
                position_init_map[j] = 0;
            }

            position_init_map[0] = V_has_initialization[singles[i].lhs];
            position_init_map[1] = V_has_initialization[singles[i].rhs[1]];
            position_init_map[2] = V_has_initialization[singles[i].rhs[2]];

            current_variables[0] = singles[i].lhs;
            current_variables[1] = singles[i].rhs[1];
            current_variables[2] = singles[i].rhs[2];

            for (int j =0 ; j < 3; j++){
                number_uninitialized += 1 - position_init_map[j];
            }

            if (number_uninitialized == 0){
                singles_open[i] = false;
            }   else {
                number_singles_open++;
                if (number_uninitialized == 1){
                    equation_to_solve[i] = true;
                    number_equations_to_solve++;
                    for (int j =0 ; j < 3; j++){
                        if (position_init_map[j] == false){

                            V_number_occurrences[current_variables[j]]++;

                            equations_per_var[current_variables[j]].push_back(i);
                        }
                    }
                }
            }
        }
    }

    if (verbose){
        cout << "Is the special var initialized? " << V_has_initialization[vn] << endl;
        cout << "singles open, doubles open " << number_singles_open << ", " << number_doubles_open << endl;
        cout << "Number of equations that can be solved " << number_equations_to_solve << endl;

        out << "Equations to solve " << endl;
        for (int i = 0; i < an; i++){
            if (equation_to_solve[i] == true){
                out << i << endl;
            }
        }

        out << "Var counts " << endl;
        for (int i= 0; i < vn; i++){
            if (V_number_occurrences[i] > 0){
                out << "V " << i << " and number " << V_number_occurrences[i] << endl;
            }
        }
    }

    if (number_equations_to_solve > 0){
        // we can solve for one of these variables ...

        int var_to_solve = -1;

        var_to_solve = SelectVarToIterativelySolve();


        TestSolutionHasIdentityPstarTstar(V_initial);

        VAR_TYPE VT = ReturnVarType(var_to_solve);
        if (verbose){
            cout << "We're solving for " << var_to_solve << endl;
        }

        solved_vars.push_back(var_to_solve);
        out << "Iterative solving: solving for var " << var_to_solve << ", type " << VT <<  " b/c it has " << equations_per_var[var_to_solve].size() << " current equations " << endl;

        vector<Matrix4d> LHS;
        vector<Matrix4d> RHS;

        CreateTAequalsBproblem(var_to_solve, equations_per_var[var_to_solve], LHS, RHS, verbose);

        TestSolutionHasIdentityPstarTstar(V_initial);

        /// Solve with Shah's method ....
        Matrix4d Result;

        if (LHS.size() == 1){
            // TA = B
            Result = RHS[0]*LHS[0].inverse();
            //cout << "least squares " << endl;
        }   else {
            // solve with a closed-form method
            SolveWithShahsMethod(Result, LHS, RHS, verbose);

        }

        V_initial[var_to_solve] = Result;
        V_has_initialization[var_to_solve] = true;

        if (interleavedSolve){

            // iterative solve.
            int number_equations = LHS.size();
            Matrix4d eye4; eye4.setIdentity();
            vector<Matrix4d> newRHS(number_equations, eye4);

            for (int i = 0; i < number_equations; i++){
                newRHS[i] = RHS[i]*LHS[i].inverse();
            }

            XASolveIteratively(newRHS, Result, Cali_Quaternion, max_iterations);

            V_initial[var_to_solve] = Result;
            V_has_initialization[var_to_solve] = true;
        }

        return true;
    }   else {
        return false;

    }

}

// used 11/27
Matrix4d MCcali::MultiplyInitializedValuesOnSideUseThisSolution(const vector<int>& side, const vector<Matrix4d>& this_solution){

    Matrix4d AM;
    Matrix4d RM;

    // start with A -- will always be initialized.
    int a_index = side[0];
    AM = A[a_index];
    RM = AM; // in case there's nothing on this side/

    // everything on this side should be initialized or empty ....
    for (int i = 1; i < 3; i++){

        if (!V_has_initialization[side[i]]){

            cout << "Error in MultiplyInitializedValuesOnSideUseThisSolution -- everything should be initialized already or empty ...." << endl; exit(1);
        }

        // assumed that has initialization ....
        RM = RM*this_solution[side[i]];
    }
    return  RM;
}

void MCcali::OutputRunResultsII(const string& filename){

    ofstream out;
    out.open(filename.c_str());

    /// make this a MC function, and record the types being submitted.
    int number_types = type_recorder.size();

    out << "Algebraic error cost function error, summed over all FR" << endl;
    for (int i = 0; i < number_types; i++){
        out << std::left << setw(20) << descriptor_recorder[i] << ": " << summed_c1_cost_function_error_by_type[i] << endl;
    }

    out << endl << endl;
    out << "---------------------------------" << endl;

    out << "Algebraic error cost function error, averaged by number of FRs (equation 16) " << endl;
    for (int i = 0; i < number_types; i++){
        out << std::left << setw(20) << descriptor_recorder[i] << ": " << summed_c1_cost_function_error_by_type[i]/double(an) << endl;
    }

    out << endl << endl;
    out << "---------------------------------" << endl;
    double sum_err = 0;

    vector<double> reproj_sums;
    for (int i = 0; i <number_types; i++){
        sum_err = 0;
        for (int j = 0; j < NumberSingles(); j++){
            sum_err += unsquared_reprojection_error_by_term_and_type[i][j];
        }
        reproj_sums.push_back(sum_err);
    }

    out << "Reprojection error " << endl;
    for (int i = 0; i <number_types; i++){
        out << descriptor_recorder[i] << ": ";
        out << "Reprojection error, average: " << (reproj_sums[i]/double(NumberSingles())) << endl;
    }



    // reconstruction error, averages without square
    out << endl << endl;
    out << "Reconstruction accuracy error (RAE), unsquared (equation 19)" << endl;
    out << "Number valid image points  " << number_valid_points_rae_strict << endl;


        for (int i = 0; i <number_types; i++){
            out << descriptor_recorder[i] << ": ";

            out << "RAE w/ BA, average, stddev, median : " << average_rae_ba[i];
            out << ", " << stddev_rae_ba[i] << ", " << median_rae_ba[i] << endl;
        }





    out << endl <<  endl;
    out << "These are the values of the error per term, sometimes useful to find sources of error in the data." << endl;

    out << endl <<  "---------------------------------" << endl;
    out << "Algebraic error, per foundational relationship. " << endl;

    for (int i = 0; i < number_types; i++){
        out << descriptor_recorder[i] << ": ";
        for (int j = 0; j < NumberSingles(); j++){
            out << cost_function_error_by_type_c1[i][j] << " ";
        }
        out << endl;
    }

    out << endl <<  "---------------------------------" << endl;
    out << "Reprojection error, per foundational relationship. " << endl;

    for (int i = 0; i <number_types; i++){
        out << descriptor_recorder[i] << ": ";

        for (int j = 0; j < NumberSingles(); j++){
            out << unsquared_reprojection_error_by_term_and_type[i][j] << " ";
        }
        out <<  " Total " << sum_err << endl;

    }


    out.close();

}

// used 11/27
void MCcali::ReconstructionAccuracyErrorAndWriteII(const string& write_dir, int current_item, const vector<CameraCali*>& CCV,
        double* camera_params, ofstream& out){

    // input checking
    string descriptor = "";
    string filename = "";

    switch (current_item){
    case 0: {
        descriptor = "initial";
    }   break;
    case 1: {
        descriptor = "minimization1";
    } break;
    case 2: {
        descriptor = "incremental";
    } break;
    }


    vector<Matrix4d> vector_to_use;
    switch (current_item){
    case 0: {
        vector_to_use = V_progressive_solutions[0];
    } break;
    case 1: {
        vector_to_use = V_progressive_solutions[1];
    } break;
    case 2: {
        vector_to_use = V_progressive_solutions[0];
    } break;
    default: {
        cout << "A type in Write Solutions chosen that is not implemented .... " << current_item << endl;
        cout << "Line " << __LINE__ << " in " << __FILE__ << endl;
        cout << "Exiting " << endl;
        exit(1);
    }
    }


    ReconstructXFunctionIDsMCwithDLTs(*this, vector_to_use, CCV, camera_params,
            reconstructed_points, valid_reconstructed_points, out);


    // then process...
    double summed_error = 0;
    int n = 0;

    int number_pts = valid_reconstructed_points.size();

    Vector3d gt;

    string filename1 = write_dir + "EstimatedVsGroundTruth" + ToString<int>(current_item) + ".txt";
    ofstream ot; ot.open(filename1.c_str());


    double single_norm = 0;
    double squared_norm = 0;

    vector<double> norms;

    summed_error = 0;
    for (int i = 0; i < number_pts; i++){
        if (valid_reconstructed_points[i] == true){
            n++;

            gt(0) = CCV[0]->P_class->three_d_points[i].x;
            gt(1) = CCV[0]->P_class->three_d_points[i].y;
            gt(2) = CCV[0]->P_class->three_d_points[i].z;

            squared_norm = 0;
            for (int j =0; j < 3; j++){
                squared_norm += pow(reconstructed_points[i](j) - gt(j), 2);
            }

            single_norm = sqrt(squared_norm);
            summed_error+= single_norm;

            norms.push_back(single_norm);

            ot << i << endl;
            ot << gt.transpose() << endl;
            ot << reconstructed_points[i].transpose() << endl;
            ot << "norm " << sqrt(squared_norm) << endl;
            ot << "summed error " << summed_error << endl;
        }
    }

    cout << "Number of reconstructed points ... " << n << endl;
    ot.close();

    int sample_n = n;

    if (n == 0){
        sample_n++;
        norms.push_back(0);
        // just to deal w/ the non-existence of valid points.
    }
    cout << "Sample n ... " << sample_n << endl;

    double mu = summed_error/double(sample_n);

    number_valid_points_rae_strict = sample_n;
    average_rae_ba.push_back(mu);

    double stddev = StdDeviation(norms, mu);
    stddev_rae_ba.push_back(stddev);

    std::sort(norms.begin(), norms.end());
    double median = norms[norms.size()/2];
    median_rae_ba.push_back(median);

    out << "summed error is " << summed_error << endl;
    out << "Number valid points " << n << " out of " << number_pts << endl;
    out << "average error " << summed_error/double(sample_n) << endl;


    // assuming all patterns have the same number of points, for now.
    PatternsCreated* P = CCV[0]->P_class;

    for (int k = 0; k< NumberPatterns(); k++){

        int number_corners_this_pattern = P->NumberCornersPerPattern();

        int start_j = number_corners_this_pattern*k;
        int end_j = number_corners_this_pattern*(k + 1);



        vector<Vector3d> ideal(number_corners_this_pattern);
        for (int i = 0, j = start_j; j < end_j; j++, i++){
            //              for (int i = 0, j = P->min_max_id_squares[k].first;
            //                      j <= P->min_max_id_squares[k].second; j++, i++){

            ideal[i](0) = P->three_d_points[j].x;
            ideal[i](1) = P->three_d_points[j].y;
            ideal[i](2) = P->three_d_points[j].z;
        }

        filename = write_dir + "world-ideal_pattern" + ToString<int>(k) + ".ply";
        if (P->is_charuco){

            WritePatternsCharuco(ideal, P->pp.squaresY - 1, P->pp.squaresX - 1, k, filename);

        }   else {

            WritePatternsApril(ideal, P->pp.squaresY, P->pp.squaresX, k, filename);

        }


        filename = write_dir + "pattern"+ ToString<int>(k) + "reconstruction-of-id-pattern-points.ply";

        vector<Vector3d> current_points;


        /// need to make new vectors without skips ...
        for (int j = start_j; j < end_j; j++){
            if (valid_reconstructed_points[j]){
                current_points.push_back(reconstructed_points[j]);
            }

        }

        // Debugging, really
        out << "total this pattern, current size " << number_corners_this_pattern << ", " << current_points.size() << endl;

        if (P->is_charuco){
            if (int(current_points.size()) == number_corners_this_pattern){

                WritePatternsCharuco(current_points, P->pp.squaresY - 1, P->pp.squaresX - 1, k + 1, filename);

            }   else {
                WritePatternsSkips(current_points, k + 1, filename);
            }
        }   else {
            if (int(current_points.size()) == number_corners_this_pattern){

                WritePatternsApril(current_points, P->pp.squaresY, P->pp.squaresX, k + 1, filename);

            }   else {
                WritePatternsSkips(current_points, k + 1, filename);
            }
        }

        current_points.clear();

    }



    // write patterns as computed by the optimization
    if (current_item == 0 || current_item == 1 || current_item == 2){
        // write the pattern as computed w/ the pattern vars.
        PatternsCreated* P = CCV[0]->P_class;
        int number_corners_this_pattern = P->NumberCornersPerPattern();


        Vector4d xprior;  xprior.setConstant(1);
        Vector4d xpost;  Vector3d x;
        int pattern_graph_index;


        for (int p = 0; p< NumberPatterns(); p++){

            filename = write_dir + "pattern"+ ToString<int>(p) + "using-pattern-transformations.ply";

            int start_j = number_corners_this_pattern*p;
            int end_j = number_corners_this_pattern*(p + 1);

            vector<Vector3d> current_points;

            pattern_graph_index = p + cn;


            for (int j = start_j; j < end_j; j++){

                xprior(0) = P->three_d_points[j].x;
                xprior(1) = P->three_d_points[j].y;
                xprior(2) = P->three_d_points[j].z;

                xpost  = vector_to_use[pattern_graph_index].inverse()*xprior;

                for (int k= 0; k < 3; k++){
                    x(k) = xpost(k);
                }

                current_points.push_back(x);
            }

            out << "total this pattern, current size " << number_corners_this_pattern << ", " << current_points.size() << endl;

            if (P->is_charuco){
                if (int(current_points.size()) == number_corners_this_pattern){
                    WritePatternsCharuco(current_points, P->pp.squaresY - 1, P->pp.squaresX - 1, p + 1, filename);

                }   else {
                    WritePatternsSkips(current_points, p + 1, filename);
                }
            }   else {
                if (int(current_points.size()) == number_corners_this_pattern){

                    WritePatternsApril(current_points, P->pp.squaresY, P->pp.squaresX, p + 1, filename);

                }   else {
                    WritePatternsSkips(current_points, p + 1, filename);
                }
            }

            current_points.clear();
        }
    }


    // update
    Points_progressive_solutions_strict.push_back(reconstructed_points);
    Valid_progressive_solutions_strict.push_back(valid_reconstructed_points);
}

// used 11/27
VAR_TYPE MCcali::ReturnVarType(int index) const {

    if (index < 0 || index > vn + 1){
        cout << "Index out of bounds in Return Var Type! " << endl;
        exit(1);
    }

    return V_type[index];
}

// used 11/27
void MCcali::SelectKPointsForMinimization(const vector<CameraCali*>& CCV, int selectedk){


    // this is for an individual.
    int camera_index, pattern_graph_index, pattern_superscript, time_graph_index, time_superscript, number_corners_per;
    int start_j = 0; int end_j =0;

    for (int i = 0; i < NumberSingles(); i++){
        camera_index = singles[i].lhs;

        pattern_graph_index = singles[i].rhs[1];
        time_graph_index = singles[i].rhs[2];

        camera_index = singles[i].lhs;

        pattern_graph_index = singles[i].rhs[1];
        time_graph_index = singles[i].rhs[2];

        time_superscript = V_index.at(time_graph_index);
        pattern_superscript = V_index.at(pattern_graph_index);

        int n_points = CCV[camera_index]->PointsAtImagePattern(time_superscript, pattern_superscript); /// image = time

        if (n_points > selectedk){

            vector<cv::Point2f> sample_points(n_points);
            vector<int> best_labels;
            vector<cv::Point2f> centers;
            // select a smaller number of clusters. otherwise, leave this the same, because we are at the target number.

            int s = 0;

            number_corners_per = CCV[camera_index]->P_class->NumberCornersPerPattern();

            start_j = number_corners_per*pattern_superscript;
            end_j = number_corners_per*(pattern_superscript + 1);

            for (int j = start_j; j < end_j; j++){

                if (CCV[camera_index]->points_present[time_superscript].at(j) == true){

                    sample_points[s].x = CCV[camera_index]->two_d_point_coordinates_dense[time_superscript](j, 0);
                    sample_points[s].y = CCV[camera_index]->two_d_point_coordinates_dense[time_superscript](j, 1);


                    s++;
                }
            }

            // will be a run-time error in OpenCV 3.3.0, not in OpenCV 3.4.8
            kmeans(sample_points, selectedk, best_labels,
                    cv::TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 0.001), 10,
                    KMEANS_PP_CENTERS, centers);

            /// now, find the assignment of the points to the centers.
            /// find the smallest distance to the center, for each label.  Then need to map that to the dense labels ....
            vector<int> indices_of_best_in_sparse(selectedk, -1);
            vector<double> best_sq_distances(selectedk, 0);
            vector<bool> bool_map_sparse(n_points, false);

            double current_distance = 0;
            int current_label = 0;
            Point2f current_center;

            for (int j = 0; j < n_points; j++){
                current_label = best_labels[j];
                current_center = centers[current_label];
                current_distance = pow(current_center.x - sample_points[j].x, 2) + pow(current_center.y - sample_points[j].y, 2);
                // now go through the centers, and update indices.
                if ((indices_of_best_in_sparse[current_label] == -1) || (current_distance < best_sq_distances[current_label])){
                    best_sq_distances[current_label] = current_distance;
                    indices_of_best_in_sparse[current_label] = j;
                }
            }

            for (int j = 0; j < selectedk; j++){
                bool_map_sparse[indices_of_best_in_sparse[j]] = true;
            }

            s = 0;

            number_corners_per = CCV[camera_index]->P_class->NumberCornersPerPattern();

            start_j = number_corners_per*pattern_superscript;
            end_j = number_corners_per*(pattern_superscript + 1);

            for (int j = start_j; j < end_j; j++){

                if (CCV[camera_index]->points_present[time_superscript].at(j) == true){

                    if (bool_map_sparse[s] == false){
                        CCV[camera_index]->points_used_min[time_superscript].at(j) = false;
                    }

                    s++;
                }
            }


        }   else {

            for (int j = start_j; j < end_j; j++){

                if (CCV[camera_index]->points_present[time_superscript].at(j) == true){
                    CCV[camera_index]->points_used_min[time_superscript].at(j) = true;

                }
            }

        }
    }

}

// used 11/27
int MCcali::SelectVarToIterativelySolve(){

    // select largest frequency by V_number_occurrences

    // then C over P over T

    // then by lowest index.

    int max_value = 0;

    for (int i = 0; i < vn; i++){
        if (max_value < V_number_occurrences[i]){
            max_value = V_number_occurrences[i];
        }
    }

    /// then, determine relevant vars within categories.

    /// are there any C variables? pick the first one.
    int c_var = -1; int p_var = -1; int t_var = -1;
    for (int i = 0; i < cn; i++){
        if (max_value == V_number_occurrences[i] && c_var == -1){
            c_var = i;
            return c_var;
        }
    }

    /// Are there any P vars?  pick the first one.
    for (int i = cn; i < cn+pn; i++){
        // second part not really needed....
        if (max_value == V_number_occurrences[i] && p_var == -1){
            p_var = i;
            return p_var;
        }
    }

    /// are there any T vars? pick the first one.
    for (int i = cn + pn; i < vn; i++){
        // second part not really needed....
        if (max_value == V_number_occurrences[i] && t_var == -1){
            t_var = i;
            return t_var;
        }
    }

    return -1;
}

// used 11/27
bool MCcali::SolveClique(std::ofstream& out, vector<int>& solved_vars,
        int number_iterations ){
    // find the best pair

    pair<int, int> best_pair(-1, -1);
    int best_numberfrs = 0;

    vector<int> uninitialized_items(3, 0);
    int n_uninit = 0;
    Matrix4d I4; I4.setIdentity();

    int number_open0 = 0;
    MatrixXd M(NumberVariables(), NumberVariables());  M.setZero();

    for (int i = 0, number_frs = NumberSingles(); i < number_frs; i++){

        if (singles_open[i] == true){ // i.e. at least one is uninitialized REQUIRE that one of these be a camera.
            number_open0++;
            // start from the left, work to the right.  work w/ two, and only two uninitialized.
            n_uninit = 0;

            if (V_has_initialization[singles[i].lhs] == false){  /// the camera has to be uninitialized.
                uninitialized_items[n_uninit] = singles[i].lhs;
                n_uninit++;


                if (V_has_initialization[singles[i].rhs[1]] == false){
                    uninitialized_items[n_uninit] = singles[i].rhs[1];
                    n_uninit++;
                }

                if (V_has_initialization[singles[i].rhs[2]] == false){
                    uninitialized_items[n_uninit] = singles[i].rhs[2];
                    n_uninit++;
                }

                if (n_uninit == 2){
                    M(uninitialized_items[0], uninitialized_items[1])++;

                    if (M(uninitialized_items[0], uninitialized_items[1]) > best_numberfrs){
                        best_numberfrs = M(uninitialized_items[0], uninitialized_items[1]);
                        best_pair = pair<int, int>(uninitialized_items[0], uninitialized_items[1]);
                    }
                }

            }
        }

    }

    out << "The best pair is " << best_pair.first << " " << best_pair.second << " with " << best_numberfrs << " occurrences " << endl;
    solved_vars.push_back(best_pair.first);
    solved_vars.push_back(best_pair.second);


    // then find where this is happening ... create the optimization problem.

    // what is the case?  1st var has to be a camera
    bool second_var = 0;

    if (best_pair.second >= cn + pn){
        second_var = 1;
    }

    if (second_var == 0){
        out << "second var case is a pattern variable." << endl;
    }   else {
        out << "second var case is a time variable." << endl;
    }

    vector<Matrix4d> Arwhec;
    vector<Matrix4d> Brwhec;

    for (int i = 0, number_frs = NumberSingles(); i < number_frs; i++){

        if (singles_open[i] == true){
            if (singles[i].lhs == best_pair.first){

                // AX = ZB

                switch (second_var){
                case 0:{
                    if (singles[i].rhs[1] == best_pair.second){

                        Arwhec.push_back(A[i].inverse()); // A.inverse() * C = P * T
                        Brwhec.push_back(V_initial[singles[i].rhs[2]]);  // T
                    }
                } break;
                case 1: {
                    if (singles[i].rhs[2] == best_pair.second){
                        Arwhec.push_back( (A[i]*V_initial[singles[i].rhs[1]]).inverse()); // (A*P).inverse() * C = T*I
                        Brwhec.push_back(I4);
                    }
                } break;
                }
            }
        }
    }


    Matrix4d X; Matrix4d Z;


    if (best_numberfrs >= 3){ // can't solve well w/ just one observation.

        ShahKroneckerProduct(Arwhec, Brwhec, X, Z, out);

        out << "X, Z, before iterative solve " << endl << X << endl << "++++" << Z << endl;

        AXZBSolveIteratively(Arwhec, Brwhec, X, Z, Cali_Quaternion, number_iterations);
        out << "X, Z, after iterative solve " << endl << X << endl << "++++" << Z << endl;


    } else { // this is likely not set up just right . Shah's was slight different ...
        X.setIdentity();
        Z.setIdentity();
        AXZBSolveIteratively(Arwhec, Brwhec, X, Z, Cali_Quaternion, number_iterations);
    }

    double error = 0;

    Matrix4d diff;
    for (int i = 0, n = int(Arwhec.size()); i < n; i++){

        diff = Arwhec[i]*X - Z*Brwhec[i];

        for (int r = 0; r < 4; r++){
            for (int c = 0; c < 4; c++){
                error += diff(r, c)*diff(r, c);
            }
        }

        out << "diff matrix " << i << " " << error << endl << diff << endl;
    }

    out << "Algebraic error for RWHEC for this problem " << sqrt(error/double((Arwhec.size()))) << endl;

    V_initial[best_pair.first] = X;
    V_has_initialization[best_pair.first] = true;

    V_initial[best_pair.second] = Z;
    V_has_initialization[best_pair.second] = true;

    UpdateSinglesOpenFlag();

    int number_open1 = 0;
    for (int i = 0, number_frs = NumberSingles(); i < number_frs; i++){

        if (singles_open[i] == true){
            number_open1++;
        }
    }

    out << "Number open before, and after " << number_open0 << ", " << number_open1 << endl;


    if (best_numberfrs >= 3){ // can't solve well w/ just one observation.
        return true;
    }   else {
        return false;
    }

}

// used 11/27
void MCcali::SubstitutePTstar(vector<int>& variable_order, const string& write_dir, bool write_docs){

    int p_graph_index;
    int t_graph_index;
    int p_superscript;
    int t_superscript;
    int c_graph_index;

    V_has_initialization[pattern_a_vars[p_star()]] = true; /// identity
    V_has_initialization[time_f_vars[t_star()]] = true; // identity

    singles_open.clear();
    singles_open.resize(an, true);

    equation_to_solve.clear();
    equation_to_solve.resize(an, false);


    for (int a_count = 0; a_count < an; a_count++){
        // what's all of these variables, can they be reduced?
        c_graph_index = singles[a_count].lhs;
        if (c_graph_index >= 0){


        }   else {
            cout << "Major error!  camera matrix on a single is empty." << endl;
            exit(1);
        }

        // what are they?
        p_graph_index = singles[a_count].rhs[1];
        p_superscript = V_index[p_graph_index];


        t_graph_index = singles[a_count].rhs[2];
        t_superscript = V_index[t_graph_index];


        if (t_superscript == t_star() && p_superscript == p_star()){

            V_has_initialization[c_graph_index] = true;
            V_initial[c_graph_index] = A[a_count];
            singles_open[a_count] = false;
            variable_order.push_back(c_graph_index);
        }
    }


}

// used 11/27
void MCcali::SumOccurrencesSingles(){

    bool has_p;
    bool has_t;
    int p_graph_index = -1;
    int t_graph_index = -1;
    int c_graph_index = -1;

    for (int i = 0; i < vn; i++){
        V_number_occurrences[i] = 0;
    }


    for (int a_count = 0; a_count < an; a_count++){

        c_graph_index = singles[a_count].lhs;

        has_p = (singles[a_count].rhs[1] >= 0);
        has_t = (singles[a_count].rhs[2] >= 0);

        if (has_p){
            // what are they?
            p_graph_index = singles[a_count].rhs[1];
        }   else {
            //p_superscript = p_star();
        }

        if (has_t){
            t_graph_index = singles[a_count].rhs[2];
        }

        A_number_occurrences[a_count]++;
        // c_graph is in the variable space.
        V_number_occurrences[c_graph_index]++;

        if (has_p){
            V_number_occurrences[p_graph_index]++;
        }   else {
            cout << "Bad pattern variable case, " << __LINE__ << " in " << __FILE__ << endl;
            exit(1);
        }

        if (has_t){
            V_number_occurrences[t_graph_index]++;
        }   else {
            cout << "Bad time variable case, " << __LINE__ << " in " << __FILE__ << endl;
            exit(1);
        }
    }

}

// used 11/27
void MCcali::TestSolutionHasIdentityPstarTstar(const vector<Matrix4d>& vector_to_use){
    // last two are empty if the dataset is not simulated.

    int p_star_graph = pattern_a_vars[p_star()];
    int t_star_graph = time_f_vars[t_star()];

    Matrix4d I;
    I.setIdentity();

    double tol = 1e-20;
    double mat_norm0 = 0;
    double mat_norm1 = 0;

    mat_norm0 = (vector_to_use[p_star_graph] - I).norm();

    assert(mat_norm0 < tol);

    mat_norm1 = (vector_to_use[t_star_graph] - I).norm();

    assert(mat_norm1 < tol);

}



// used 11/27
void MCcali::WriteCameraCalibrationResult(const vector<CameraCali*>& CCV, const vector<string>& camera_names,
        const vector<Matrix4d>& ext_to_use, const string& filename){

    ofstream out;
    out.open(filename.c_str());

    if (!out.good()){
        cout << "filename for write is not good, exiting." << endl;
        exit(1);

    }   else {

        out << cn << endl;

        for (int i = 0; i < cn; i++){

            out << camera_names[i] << " ";
            for (int r = 0; r < 3; r++){
                for (int c = 0; c < 3; c++){
                    out << CCV[i]->internal_parameters(r, c) << " ";
                }
            }

            for (int r = 0; r < 3; r++){
                for (int c = 0; c < 3; c++){
                    out << ext_to_use[i](r,c) << " ";
                }
            }

            for (int r = 0; r < 3; r++){
                out <<ext_to_use[i](r,3) << " ";
            }

            for (int r = 0, rn = CCV[i]->distortion.rows(); r < rn; r++){
                out << CCV[i]->distortion(r) << " ";
            }

            out << endl;

        }
    }
}


// used 11/27
void MCcali::WriteSimulatedCamerasAtAllTimes(const string& write_directory, const string& current_dir,
        const vector<CameraCali*>& CCV,
        float camera_size, float track_size, const vector<Matrix4d>& vector_to_use, int r, int g, int b){


    Matrix4d ProposedMat;
    Matrix4d TimeMat;

    vector<Matrix3d> internals;
    vector<Matrix4d> externals;

    Vector3d center;

    string filename;

    Vector3d offset;  offset.setZero();  offset(1) = -1;

    for (int i = 0; i < NumberCameras(); i++){
        Matrix4d Cam;
        vector<Vector3d> one_track;
        if (V_has_initialization[i]){
            Cam = vector_to_use[i];
            for (int t = 0; t < tn; t++){

                if (V_has_initialization[cn + pn + t] == true){

                    TimeMat = vector_to_use[cn + pn + t];

                    ProposedMat = Cam*TimeMat.inverse();

                    internals.push_back(CCV[i]->internal_parameters);
                    externals.push_back(ProposedMat);

                    filename  =  write_directory + current_dir + "/c"+
                            ToString<int>(i) + "_time" + ToString<int>(t) + ".ply";


                    create_camera(CCV[i]->internal_parameters, ProposedMat, camera_size, r, g, b,
                            CCV[i]->rows, CCV[i]->cols, filename);

                    center = ReturnCenter(ProposedMat);

                    one_track.push_back(center);
                }
            }

            filename  =  write_directory + current_dir + "/track"+
                    ToString<int>(i) + ".ply";

            create_tracks(one_track, 0, 0, 0, track_size, offset, filename);
        }
    }


    filename  =  write_directory + current_dir + "/all.ply";

    create_cameras(internals, externals, r, g, b,
            CCV[0]->rows, CCV[0]->cols, filename, camera_size);

}
// used 11/27
void MCcali::WriteSolutionAssessErrorII(const string& write_directory, const vector<string>& camera_names,
        const vector<CameraCali*>& CCV, int type,
        bool write, float camera_size, float track_size ){
    // last two are empty if the dataset is not simulated.

    type_recorder.push_back(type);

    string filename, command;
    string descriptor;

    vector<Matrix4d> vector_to_use;
    switch (type){
    case 0: {
        vector_to_use = V_progressive_solutions[0];
    } break;
    case 1: {
        vector_to_use = V_progressive_solutions[1];
    } break;
    case 2: {
        vector_to_use = V_progressive_solutions[0];
    } break;
    default: {
        cout << "A type in Write Solutions chosen that is not implemented .... " << type << endl;
        cout << "Line " << __LINE__ << " in " << __FILE__ << endl;
        cout << "Exiting " << endl;
        exit(1);
    }
    }

    switch (type){
    case 0: {
        descriptor = "initial";
    }   break;
    case 1: {
        descriptor = "minimization1";
    } break;
    case 2: {
        descriptor = "incremental";
    } break;
    }


    descriptor_recorder.push_back(descriptor);

    /// before solve, write initial solution ....
    filename = write_directory + "camera_cali_" + descriptor + ".txt";

    switch (type){
    case 0: {
        WriteCameraCalibrationResult(CCV, camera_names, vector_to_use, filename);
    }   break;
    case 1: {
        WriteCameraCalibrationResult(CCV, camera_names, vector_to_use, filename);
    } break;
    case 2: {
        WriteCameraCalibrationResult(CCV, camera_names, vector_to_use, filename);
    } break;
    }


    string current_dir = "cameras-" + descriptor;

    mkdir((write_directory + current_dir).c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

    string cam_dir = write_directory + current_dir + "/single_cameras";

    mkdir(cam_dir.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

    vector<Matrix3d> internals;
    vector<Matrix4d> externals;

    vector<int> camColor(3, 0);

    switch (type){
    case -2: {
        //cam_color << 230, 159, 0;
        camColor[0] = 230;
        camColor[1] = 159; // [2]  = 0
    } break;
    case -1:{
        //cam_color << 230, 159, 0;
        camColor[0] = 230;
        camColor[1] = 159;
    }break;
    case 0:{
        //cam_color << 0, 114, 178;
        camColor[1] = 114;
        camColor[2] = 178;
    } break;
    case 1:{

        camColor[1] = 158;
        camColor[2] = 115;
    } break;
    case 2:{
        // cam_color << 0, 158, 0;
        camColor[1] = 158;
        camColor[1] = 158;
        camColor[2] = 115;
    } break;
    }

    for (int i = 0; i < NumberCameras(); i++){

        filename = cam_dir + "/c"+ ToString<int>(i) + ".ply";

        switch (type){
        default: {
            if (V_has_initialization[i]){
                // rows and cols are empty
                create_camera(CCV[i]->internal_parameters, vector_to_use[i], camera_size,  camColor[0], camColor[1], camColor[2], CCV[i]->rows, CCV[i]->cols, filename);
                internals.push_back(CCV[i]->internal_parameters);
                externals.push_back(vector_to_use[i]);
            }
        } break;

        }
    }

    filename = cam_dir + "/all.ply";

    // not sure this gets called.
    if (internals.size() > 0){
        create_cameras(internals, externals, camColor[0], camColor[1], camColor[2], CCV[0]->rows, CCV[0]->cols, filename, camera_size);
    }

    string write_file = write_directory + current_dir +  "/variables.txt";
    ofstream out;
    out.open(write_file.c_str());

    {

        int v = 0;
        for (int i = 0; i < cn; i++, v++){
            out << "C " << i << endl;
            out << vector_to_use[v] << endl;
        }

        for (int i = 0; i < pn; i++, v++){
            out << "P " << i << ", " << v <<  endl;
            out << vector_to_use[v] << endl;
        }

        for (int i = 0; i < tn; i++, v++){
            out << "T " << i << ", " << v <<  endl;
            out << vector_to_use[v] << endl;
        }
    }

    out.close();

    // work on error.  sum up singles ....
    Matrix4d SummedSquaredError;
    Matrix4d RHS;
    Matrix4d modRHS;
    Matrix4d LHS, Aprime;
    Matrix4d SummedSquaredError2;
    double c1_acc = 0;
    double c2_acc = 0;

    vector<double> id_error_c1(NumberSingles(), 0);
    vector<double> id_error_c2(NumberSingles(), 0);



    for (int i = 0; i < NumberSingles(); i++){
        /// if everyone is initialized ....

        // test that all initialized ....
        if (AllInitialized(singles[i].rhs)){


            RHS = MultiplyInitializedValuesOnSideUseThisSolution(singles[i].rhs, vector_to_use);
            SummedSquaredError = vector_to_use[singles[i].lhs]  - RHS;

            modRHS = A[i].inverse()*RHS;
            SummedSquaredError2 = vector_to_use[singles[i].lhs]*modRHS.inverse() -A[i];


            id_error_c1[i] = SummedSquaredError.squaredNorm();
            c1_acc += id_error_c1[i];

            id_error_c2[i] = SummedSquaredError2.squaredNorm();

            c2_acc += id_error_c2[i];
        }
    }


    cost_function_error_by_type_c1.push_back(id_error_c1);
    cost_function_error_by_type_c2.push_back(id_error_c2);

    // ceres answer will be 1/2 by construction
    summed_c1_cost_function_error_by_type.push_back(c1_acc);
    summed_c2_cost_function_error_by_type.push_back(c2_acc);


    string current_write_dir= write_directory + current_dir + "/";

    vector<double> unsquared_reprojection_error(NumberSingles(), 0);



    write_file = write_directory + current_dir + "/A_est.txt";
    out.open(write_file.c_str());



    ////////////////////////////////// Reprojection error //////////////////////////////////////////////////////////////
    int local_time_index = 0;
    {
        int camera_index, pattern_graph_index, pattern_superscript, time_graph_index, time_superscript;
        int num_singles = NumberSingles();
        vector<Matrix4d> vec_Aprime(num_singles);
        vector<bool> aprime_bit_vector(num_singles, false);



#pragma omp parallel for private(camera_index, pattern_graph_index, time_graph_index, LHS, time_superscript, pattern_superscript, local_time_index, Aprime )
        for (int i  = 0; i < num_singles; i++){

            if (AllInitialized(singles[i].rhs) && V_has_initialization[singles[i].lhs]){
                if (i % 20 == 0){
#pragma omp critical
                    {
                        cout << "Computing and writing reprojection for " << i << " of  " << NumberSingles() << endl;
                    }
                }

                camera_index = singles[i].lhs;

                pattern_graph_index = singles[i].rhs[1];
                time_graph_index = singles[i].rhs[2];

                LHS = vector_to_use[camera_index];

                time_superscript = V_index.at(time_graph_index);
                pattern_superscript = V_index.at(pattern_graph_index);

                local_time_index = time_superscript - CCV[camera_index]->start_time_this_camera;

                Aprime = LHS* vector_to_use[time_graph_index].inverse()*vector_to_use[pattern_graph_index].inverse();
                aprime_bit_vector[i] = true;
                vec_Aprime[i] = Aprime;

                // reprojection error is squared, then divided by number of points, need to sum and divide by number of foundational relationships.
                if (type < 0) {

                }   else {
                    unsquared_reprojection_error[i] = CCV[camera_index]->ComputeReprojectionErrorOneImagePattern(Aprime, local_time_index,
                            pattern_superscript, current_write_dir, write, i, false, 0);
                }



            }
        }

        for (int i  = 0; i < num_singles; i++){
            if (aprime_bit_vector[i] == true){
                out << "Equation " << i << endl << vec_Aprime[i] << endl;
            }
        }

    }

    out.close();
    unsquared_reprojection_error_by_term_and_type.push_back(unsquared_reprojection_error);


    WriteSimulatedCamerasAtAllTimes(write_directory, current_dir, CCV, camera_size, track_size, vector_to_use, camColor[0], camColor[1], camColor[2]);


}

//used 11/27
void MCcali::UpdateSinglesOpenFlag(){

    vector<int> position_init_map(4, 0);

    int number_uninitialized = 0;
    for (int i = 0; i < an; i++){
        if (singles_open[i] == true){
            number_uninitialized = 0;
            // see how many vars are left -- have all been initialized? -- then set singles open = false and equations to solve = false.

            for (int j =0 ; j < 3; j++){
                position_init_map[j] = 0;
            }

            position_init_map[0] = V_has_initialization[singles[i].lhs];
            position_init_map[1] = V_has_initialization[singles[i].rhs[1]];
            position_init_map[2] = V_has_initialization[singles[i].rhs[2]];

            for (int j =0 ; j < 3; j++){
                number_uninitialized += 1 - position_init_map[j];
            }

            if (number_uninitialized == 0){
                singles_open[i] = false;
            }

        }
    }

}

// used 11/27
Matrix3d ExtractRotationMatrix(Matrix4d& M){

    Matrix3d R;
    for (int r = 0; r < 3; r++){
        for (int c = 0; c < 3; c++){
            R(r, c) = M(r, c);
        }
    }

    return R;
}

//used 11/27
void ExtractRotationTranslationMatrix(Matrix4d& M, Matrix3d& R, MatrixXd& t){


    t.resize(3, 1);
    for (int r = 0; r < 3; r++){
        for (int c = 0; c < 3; c++){
            R(r, c) = M(r, c);
        }
        t(r, 0) = M(r, 3);
    }
}

//used 11/27
MatrixXd KroneckerProduct(Matrix3d& M1, Matrix3d& M2){

    int rowsa = M1.rows(); int rowsb = M2.rows();
    int colsa = M1.cols(); int colsb = M2.cols();

    MatrixXd M3(rowsa*rowsb, colsa*colsb);

    M3.setZero();

    double item = 0;
    Matrix3d B;
    for (int r0 = 0; r0 < 3; r0++){
        for (int c0 = 0; c0 < 3; c0++){

            item = M1(r0, c0);

            B = item*M2;

            for (int r1 = 0; r1 < 3; r1++){
                for (int c1 = 0; c1 < 3; c1++){
                    M3(r0*3 + r1, c0*3 + c1) = B(r1, c1);
                }
            }
        }
    }


    return M3;

}

//used 11/27
MatrixXd KroneckerProduct(MatrixXd& M1, MatrixXd& M2){


    int rowsa = M1.rows(); int rowsb = M2.rows();
    int colsa = M1.cols(); int colsb = M2.cols();

    MatrixXd M3(rowsa*rowsb, colsa*colsb);

    M3.setZero();

    for (int r0 = 0; r0 < rowsa; r0++){
        for (int c0 = 0; c0 < colsa; c0++){
            for (int r1 = 0; r1 < rowsb; r1++){
                for (int c1 = 0; c1 < colsb; c1++){
                    M3(r0*rowsb + r1, c0*colsb + c1 ) = M1(r0, c0)*M2(r1, c1);
                }
            }
        }
    }

    return M3;

}
//used 11/27
void ShahKroneckerProduct(vector<Matrix4d>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z, std::ofstream& out){


    X.setZero();  Z.setZero();

    Matrix3d Rtest(3, 3);
    int N = Bs.size();

    MatrixXd A(9*N, 18);
    MatrixXd T(9, 9);
    MatrixXd I9(9, 9);
    Matrix3d Ra, Rb;

    MatrixXd KP;
    A.setZero();
    T.setZero();

    I9.setIdentity();

    MatrixXd kp_minus_i9(9, 9);

    ofstream tempout;
    string filename;

    for (int i = 0; i < N; i++){

        Ra = ExtractRotationMatrix(As[i]);
        Rb = ExtractRotationMatrix(Bs[i]);

        KP = KroneckerProduct(Rb, Ra);
        kp_minus_i9 = KP - I9;

        for (int r = i*9, r0 = 0; r < i*9 + 9; r++, r0++ ){
            for (int c = 0; c < 9; c++){
                A(r, c) = kp_minus_i9(r0, c);

            }
        }

        T = T + KP;
    }

    JacobiSVD<MatrixXd> svd(T, ComputeThinU | ComputeThinV);

    MatrixXd V = svd.matrixV();  MatrixXd U = svd.matrixU();
    for (int r0 = 0; r0 < 3; r0++){
        X(r0, 0) = V(r0, 0);
    }

    for (int r0 = 0; r0 < 3; r0++){
        X(r0, 1) = V(r0 + 3, 0);
    }

    for (int r0 = 0; r0 < 3; r0++){
        X(r0, 2) = V(r0 + 6, 0);
    }


    for (int r0 = 0; r0 < 3; r0++){
        Z(r0, 0) = U(r0, 0);
    }

    for (int r0 = 0; r0 < 3; r0++){
        Z(r0, 1) = U(r0 + 3, 0);
    }

    for (int r0 = 0; r0 < 3; r0++){
        Z(r0, 2) = U(r0 + 6, 0);
    }

    Matrix3d Rx; Matrix3d Rz;
    Rx = ExtractRotationMatrix(X);
    Rz = ExtractRotationMatrix(Z);

    double detx;

    detx = Rx.determinant();

    out << "detx " << detx << endl;

    double det_sign = copysign(1.0, detx);
    double multiplier = det_sign/(pow(fabs(detx), (1.0/3.0)));
    Rx = multiplier * Rx;

    JacobiSVD<MatrixXd> svdRx(Rx, ComputeThinU | ComputeThinV);

    Rx = svdRx.matrixU()*svdRx.matrixV().transpose();

    double detz = Rz.determinant();
    det_sign = copysign(1.0, detz);
    multiplier = det_sign/(pow(fabs(detz), (1.0/3.0)));
    Rz = multiplier * Rz;

    out << "detz " << detz << endl;

    JacobiSVD<MatrixXd> svdRz(Rz, ComputeThinU | ComputeThinV);

    Rz = svdRz.matrixU()*svdRz.matrixV().transpose();

    MatrixXd Atransl(3*N, 6);
    MatrixXd btransl(3*N, 1);

    MatrixXd I3(3, 3); I3.setZero();
    for (int i = 0; i < 3; i++){
        I3(i, i) = 1;
    }

    // remember that everything is still in Rx, Rz.
    MatrixXd zvec(9, 1);

    for (int r0 = 0; r0 < 3; r0++){
        zvec(r0, 0) = Rz(r0, 0);
    }

    for (int r0 = 0; r0 < 3; r0++){
        zvec(r0 + 3, 0) = Rz(r0, 1);
    }

    for (int r0 = 0; r0 < 3; r0++){
        zvec(r0 + 6, 0) = Rz(r0, 2);
    }


    MatrixXd ta(3, 1); MatrixXd tb(3, 1);

    MatrixXd kp_result; MatrixXd tb_trans;
    MatrixXd inter_result;
    for (int i = 0; i < N; i++){

        ExtractRotationTranslationMatrix(As[i], Ra, ta);
        ExtractRotationTranslationMatrix(Bs[i], Rb, tb);

        for (int r = i*3, r0 = 0; r < i*3 + 3; r++, r0++ ){
            for (int c = 0; c < 3; c++){
                Atransl(r, c) = -Ra(r0, c);

            }
        }

        for (int r = i*3, r0 = 0; r < i*3 + 3; r++, r0++ ){
            for (int c = 0; c < 3; c++){
                Atransl(r, c + 3) = I3(r0, c);

            }
        }

        tb_trans = tb.transpose();
        kp_result = KroneckerProduct(tb_trans, I3);
        inter_result = ta - kp_result*zvec;


        for (int r = i*3, r0 = 0; r < i*3 + 3; r++, r0++ ){
            btransl(r, 0) = inter_result(r0, 0);

        }

    }

    MatrixXd xstar;

    xstar = (Atransl.transpose()*Atransl).inverse()*Atransl.transpose()*btransl;
    out << "xstar " << endl << xstar << endl;
    for (int r0 = 0; r0 < 3;  r0++ ){
        for (int c = 0; c < 3; c++){
            X(r0, c) = Rx(r0, c);
            Z(r0, c) = Rz(r0, c);
        }

        X(r0, 3) = xstar(r0, 0);
        Z(r0, 3) = xstar(r0 + 3, 0);
    }

    X(3, 3) = 1;  Z(3, 3) = 1;

}

// used 11/27
double StdDeviation(vector<double>& v, double m)
{

    double E=0; double n = v.size();
    for(int i=0; i<int(v.size()); i++){
        E+=(v[i] - m)*(v[i] - m);
    }
    return sqrt(E/n);
}


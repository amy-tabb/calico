/*
 * multicamera.hpp
 *
 *  Created on: Jul 6, 2018
 *      Author: atabb
 */

#ifndef MULTICAMERA_HPP_
#define MULTICAMERA_HPP_

#include "Includes.hpp"
#include "camera-calibration.hpp"


using namespace std;

enum VAR_TYPE { camera_var, pattern_var, time_var, special_var };

bool VarEmpty(int var);

class single_relationship_container{
public:
    int lhs; // ^iC variable.
    vector<int> rhs; /// -1 indicates that the matrix is identity has been reduced.
    // standard is A matrix, P matrix, T matrix.

    single_relationship_container(int l);

};

class MCcali {
public:

    int number_images;

    vector<int> A_camera_indices;
    vector<int> A_pattern_indices;
    vector<int> A_time_indices;
    vector<int> A_number_occurrences;
    vector<Matrix4d> A;

    /// all of the vars in a list .... for putting in ceres.
    vector<VAR_TYPE> V_type; // 0 1 2 = C P T
    vector<int> V_index; // superscript -- destination coordinate system
    vector<bool> V_has_initialization;
    vector<bool> V_in_foundational_relationships;
    vector<Matrix4d> V_initial;
    vector<int> V_number_occurrences;
    vector<bool> V_edge_written_graph;
    vector<Matrix4d> V_first_solutiona;
    vector<Matrix4d> V_first_solutionb;
    vector<Matrix4d> V_ground_truth;
    vector<Matrix4d> V_ground_truth_relative_original;
    vector<Matrix4d> V_ground_truth_only_tstar;
    vector<Matrix4d> V_ground_truth_only_pstar;
    vector<bool> is_degenerate;

    vector<Vector3d> reconstructed_points;
    vector<bool> valid_reconstructed_points;

    vector<vector<Matrix4d> > V_progressive_solutions; /// as we need to add a solution, we increment this vector so there is a record.
    vector<vector< Vector3d> > Points_progressive_solutions_strict;
    vector<vector< bool> > Valid_progressive_solutions_strict;

    vector<double> average_rae_ba;
    vector<double> median_rae_ba;
    vector<double> stddev_rae_ba;

    vector<vector< Vector3d> > Points_progressive_solutions_multi;
    vector<vector< bool> > Valid_progressive_solutions_multi;

    vector<double> average_rae_multi;
    vector<double> median_rae_multi;
    vector<double> stddev_rae_multi;

    int number_valid_points_rae_strict;
    int number_valid_points_rae_multi;

    vector<bool> singles_open;
    vector<bool> equation_to_solve;

    vector<int> camera_i_vars;
    vector<int> pattern_a_vars;
    vector<int> time_f_vars;

    vector<single_relationship_container> singles;
    vector<double> camera_uncertainity_score;


    vector<double> summed_c1_cost_function_error_by_type;
    vector<double> summed_c2_cost_function_error_by_type;
    vector< vector< double> > cost_function_error_by_type_c1;
    vector< vector< double> > cost_function_error_by_type_c2;
    vector< vector< double> > reprojection_error_by_term_and_type;

    vector<int> type_recorder;
    vector<string> descriptor_recorder;

    vector< vector<bool> > selected_points_for_min;

    vector< double > average_rot_error;
    vector<double> std_rot_error;
    vector<double> median_rot_error;

    vector< double > average_rot_angle_error;
    vector<double> std_rot_angle_error;
    vector<double> median_rot_angle_error;


    vector< double > average_translation_error;
    vector<double> std_translation_error;
    vector<double> median_translation_error;


    vector< double > average_whole_error;
    vector<double> std_whole_error;
    vector<double> median_whole_error;


    MCcali(vector<CameraCali*>& CCV, const PatternsCreated& P_class, int max_number_images_to_use, ofstream& out,
            int synchronized_rotating_option);

    int p_star() const;
    int t_star() const;

    bool AllInitialized(const vector<int>& side);

    int BuildCostFunctionAndGraphWithTestAndDegenerateInitialize(const vector<CameraCali*>& CCV,
            ofstream& out, const string& write_dir, bool write_docs);

    bool CanSolveSystem();

    void CreateTAequalsBproblem(int var_to_solve, const vector<int>& rel_equations, vector<Matrix4d>& LHS,
            vector<Matrix4d>& RHS, bool verbose);

    void InitializeNumberOccurrences();

    void InitializeNumberOccurrencesAndInitialization();

    bool IsPstar(int p) const;

    bool IsTstar(int t) const;

    bool IterativelySolveForVariables2(ofstream& out, bool verbose, vector<int>& solved_vars, const bool interleavedSolve,
            int max_iterations = 10);

    Matrix4d MultiplyInitializedValuesOnSideUseThisSolution(const vector<int>& side, const vector<Matrix4d>& this_solution);

    int NumberPatterns() const;
    int NumberVariables() const;
    int NumberSingles() const;
    int NumberCameras() const;
    int NumberUninitialized();

    void OutputRunResults(const string& filename);

    void ReconstructionAccuracyErrorAndWriteII(const string& write_dir, int current_item, const vector<CameraCali*>& CCV,
            double* camera_params, ofstream& out);

    int RemainingToFillIn();

    VAR_TYPE ReturnVarType(int index) const;

    void SelectKPointsForMinimization(const vector<CameraCali*>& CCV, int selectedk);

    int SelectVarToIterativelySolve();

    bool SolveClique(std::ofstream& out, vector<int>& solved_vars,
            int number_iterations = 20);

    void SubstitutePTstar(vector<int>& variable_order, const string& write_dir, bool write_docs);

    void SumOccurrencesSingles();

    void TestSolutionHasIdentityPstarTstar(const vector<Matrix4d>& vector_to_use);

    void UpdateSinglesOpenFlag();

    void WriteCameraCalibrationResult(const vector<CameraCali*>& CCV, const vector<string>& camera_names,
            const vector<Matrix4d>& ext_to_use, const string& filename);

    void WriteSimulatedCamerasAtAllTimes(const string& write_directory,  const string& current_dir,
            const vector<CameraCali*>& CCV,
            float camera_size, float track_size, const vector<Matrix4d>& vector_to_use, int r, int g, int b);

    void WriteSolutionAssessErrorII(const string& write_directory, const vector<string>& camera_names,
            const vector<CameraCali*>& CCV, int type,
            bool write, float camera_size, float track_size );

private:
    int p_s;
    int t_s;
    int tn;
    int pn;
    int cn;
    int an;
    int vn;
    int uninitialized;
    bool synch_rot_option;
};

void ExtractRotationTranslationMatrix(Matrix4d& M, Matrix3d& R, MatrixXd& t);

Matrix3d ExtractRotationMatrix(Matrix4d& M);

MatrixXd KroneckerProduct(Matrix3d& M1, Matrix3d& M2);

MatrixXd KroneckerProduct(MatrixXd& M1, MatrixXd& M2);

void ShahKroneckerProduct(vector<Matrix4d>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z, std::ofstream& out);

double StdDeviation(vector<double>& v, double m);



#endif /* MULTICAMERA_HPP_ */

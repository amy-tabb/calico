/*
 * multicamera.hpp
 *
 *  Created on: Jul 6, 2018
 *      Author: atabb
 */

#ifndef MULTICAMERA_HPP_
#define MULTICAMERA_HPP_

#include "Includes.hpp"
#include "camera_calibration.hpp"


using namespace std;

enum VAR_TYPE { camera_var, pattern_var, time_var, special_var };

bool VarEmpty(int var);

class single_relationship_container{
public:
	int lhs; // ^iC variable.
	vector<int> rhs; /// -1 indicates that the matrix is identity has been reduced.
	// standard is A matrix, P matrix, T matrix.

	single_relationship_container(int l);

	int count_var_number_rhs(vector<int>& V_index, int p_star, int t_star);
};


void SolveWithShahsMethod(Matrix4d& Result, vector<Matrix4d>& LHS, vector<Matrix4d>& RHS, bool verbose);

class GroundTruthData{

public:
	vector<Matrix3d> InternalMatrices;
	vector<Matrix4d> CameraTransformations;
	vector<Matrix4d> PatternTransformations;
	vector<Matrix4d> TimeTransformations;

	vector<Matrix4d> CamerasRel;
	vector<Matrix4d> PatternsRel;
	vector<Matrix4d> TimesRel;

	vector<Matrix4d> variables_raw;
	vector<Matrix4d> variables_rel;

	vector<Vector3d> points_rel; // points computed wrt to the pattern transformations.

	GroundTruthData(string input);

	void ComputeRelativeToExemplar(int p_star, int t_star, string outputdir, PatternsCreated* P_class);
};


class MCcali {
public:

	int number_images;

	int p_star();
	int t_star();

	int NumberTimes();
	int NumberPatterns();
	int NumberVariables();
	int NumberSingles();
	int NumberCameras();
	int NumberUninitialized();
	int RemainingToFillIn();

	MCcali(vector<CameraCali*>& CCV, PatternsCreated& P_class, int max_number_images_to_use, ofstream& out,
			int synchronized_rotating_option);
	MCcali(MCcali& parent, int pstar, int tstar);

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

	vector<double> average_rae_strict;
	vector<double> median_rae_strict;
	vector<double> stddev_rae_strict;

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


	bool AllInitialized(vector<int>& side);

	bool CanSolveSystem();

	pair<int, int> SelectNextPstarTstar();

	void InitializeNumberOccurrences();

	int SelectVarToIterativelySolve();

	VAR_TYPE ReturnVarType(int index);

	bool IterativelySolveForVariables1(ofstream& out, int& solved_var, bool verbose);

	void UpdateSinglesOpenFlag();

	void MinimizeReprojectionErrorMC(vector<CameraCali*>& CCV, double* camera_params, ofstream& out, int start_id,
			int end_id, bool use_all_points_present);

	void CreateTAequalsBproblem(int var_to_solve, vector<int>& rel_equations, vector<Matrix4d>& LHS, vector<Matrix4d>& RHS, bool verbose);

	void WriteCalibrationFileForSimulatedCamerasAtAllTimes(string write_directory,  vector<CameraCali*>& CCV);

	void SubstitutePTstar(string write_dir, bool write_docs, int iter);

	int BuildCostFunctionAndGraphWithTestAndDegenerateInitialize(vector<CameraCali*>& CCV,
			ofstream& out, string write_dir, bool write_docs);

	void InitializeNumberOccurrencesAndInitialization();

	void WriteLatex(string write_dir, string descriptor, bool write_exemplars = true);

	void WriteASide(vector<int>& side, ofstream& out, bool write_exemplars = true);

	Matrix4d MultiplyInitializedValuesOnSideUseThisSolution(vector<int>& side, vector<Matrix4d>& this_solution);

	void SumOccurrencesSingles();

	void WriteCameraCalibrationResult(vector<CameraCali*>& CCV, vector<string>& camera_names,
			vector<Matrix4d>& ext_to_use, string filename);

	void WriteCameraCalibrationResult(GroundTruthData* GTD, vector<string>& camera_names,
			vector<Matrix4d>& ext_to_use, string filename, int rows);

	void ReconstructionAccuracyErrorAndWrite(string write_dir, int current_item, vector<CameraCali*>& CCV,
			double* camera_params, ofstream& out, GroundTruthData* GTD);

	void WriteSolutionAssessError(string write_directory, vector<string>& camera_names, vector<CameraCali*>& CCV, int type,
			GroundTruthData* GTD,
			bool rotating, bool write, float camera_size, float track_size );

	void WriteSimulatedCamerasAtAllTimes(string write_directory,  string current_dir, vector<CameraCali*>& CCV,
			float camera_size, float track_size, vector<Matrix4d>& vector_to_use);

	void WriteSimulatedCamerasForRotatingCase(string write_directory, string current_dir, vector<CameraCali*>& CCV,
			float camera_size, float track_size, vector<Matrix4d>& vector_to_use);

	void OutputRunResults(string filename);

	void OutputVariablesWithInitialization(string filename, int type);

	bool IsPstar(int p);

	bool IsTstar(int t);

	void SelectKPointsForMinimization(vector<CameraCali*>& CCV, int selectedk);

	void AssessCamerasWRTGroundTruth(GroundTruthData* GTD, int soln_number);

	bool SolveClique(std::ofstream& out);
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

double AssessRotationError(vector<Matrix4d>& Cgts, vector<Matrix4d>& Cnews, int number_Cs, double& stddev, double& median);

double AssessRotationErrorAxisAngle(vector<Matrix4d>& Cgts, vector<Matrix4d>& Cnews, int number_Cs, double& stddev, double& median);

double AssessTranslationError(vector<Matrix4d>& Cgts, vector<Matrix4d>& Cnews, int number_Cs, double& stddev, double& median);

double AssessErrorWhole(vector<Matrix4d>& Cgts, vector<Matrix4d>& Cnews, int number_Cs, double& stddev, double& median);


#endif /* MULTICAMERA_HPP_ */

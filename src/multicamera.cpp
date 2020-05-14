/*
 * multicamera.cpp
 *
 *  Created on: Jul 6, 2018
 *      Author: Amy Tabb
 */



#include "multicamera.hpp"
#include "camera_visualization.hpp"
#include "solving_structure.hpp"
#include "helper.hpp"


bool IsValue(int exemplar, int newvalue){
	return exemplar == newvalue;
}


bool MCcali::IsPstar(int p) const{
	return p == p_s;
}

bool MCcali::IsTstar(int t) const{
	return t == t_s;
}


int MCcali::p_star() const{
	return p_s;
}

int MCcali::t_star() const{
	return t_s;
}

int MCcali::NumberTimes() const{
	return tn;
}

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


int MCcali::NumberUninitialized(){
	uninitialized = 0;
	for (int i =0; i < vn; i++){
		uninitialized += !V_has_initialization[i];
	}

	return uninitialized;
}


int MCcali::RemainingToFillIn(){
	int count = 0;
	for (int i =0; i < vn; i++){
		count += (!V_has_initialization[i]) && (V_in_foundational_relationships[i]);
	}

	return count;
}

bool MCcali::CanSolveSystem(){

	int count = 0;
	for (int i =0; i < cn; i++){
		count += (!V_has_initialization[i]) && (V_in_foundational_relationships[i]);
	}

	return count == 0;
}

// CCV objects are updated, though not added to.
MCcali::MCcali(vector<CameraCali*>& CCV, const PatternsCreated& P_class, int max_number_images_to_use,
		ofstream& out, int synchronized_rotating_option){

	// need to select the exemplar pattern and time using CCV.
	// pattern -- find the pattern viewed by the most cameras.
	// synchronized rot option is just a more straightforward way of representing the 'network' versus 'rotating' cases,
	// b/c there is some ambiguity there.
	synch_rot_option = synchronized_rotating_option;
	number_images = max_number_images_to_use;

	pn = P_class.NumberPatterns();
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
	}	else {
		//synch_rot_option == false

		int start_time = 0;

		for (int i = 0, nc = CCV.size(); i < nc; i++){
			CCV[i]->start_time_this_camera = start_time;
			start_time += CCV[i]->number_external_images_max;

			cout << "Start time " << start_time << endl;
		}

		tn = start_time;
	}

	vector<vector<int> > t_count;
	int max_value = 0; int max_index = -1;
	// depends on what the type is -- rotating or network.


	if (CCV.size() >= 1){

		t_count.resize(tn, vector<int>(pn, 0));
	}	else {
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

	}	else {
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

void MCcali::SubstitutePTstar(const string& write_dir, bool write_docs, int iter){


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


		}	else {
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
			//cout << "Initialized single, camera " << c_graph_index << endl;
			V_initial[c_graph_index] = A[a_count];
			singles_open[a_count] = false;


		}
	}


	cout << "Exit initialize with Pstar, Tstar" << iter << endl;

	if (write_docs) {WriteLatex(write_dir, "doc-after-stage3"); }
}



void MCcali::InitializeNumberOccurrences(){

	for (int i = 0; i < vn; i++){
		V_number_occurrences[i] = 0;
	}
}

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

VAR_TYPE MCcali::ReturnVarType(int index) const {


	if (index < 0 || index > vn + 1){
		cout << "Index out of bounds in Return Var Type! " << endl;
		exit(1);
	}

	return V_type[index];
}


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
	}	else {
		if (var_to_solve < cn + pn){
			var_type = 1;
		}	else {
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
		}	else {
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

Matrix3d ExtractRotationMatrix(Matrix4d& M){

	Matrix3d R;
	for (int r = 0; r < 3; r++){
		for (int c = 0; c < 3; c++){
			R(r, c) = M(r, c);
		}
	}

	return R;
}

void ExtractRotationTranslationMatrix(Matrix4d& M, Matrix3d& R, MatrixXd& t){

	t.resize(3, 1);
	for (int r = 0; r < 3; r++){
		for (int c = 0; c < 3; c++){
			R(r, c) = M(r, c);
		}
		t(r, 0) = M(r, 3);
	}
}


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

	cout << "X current " << endl << X << endl;

	for (int r0 = 0; r0 < 3; r0++){
		Z(r0, 0) = U(r0, 0);
	}

	for (int r0 = 0; r0 < 3; r0++){
		Z(r0, 1) = U(r0 + 3, 0);
	}

	for (int r0 = 0; r0 < 3; r0++){
		Z(r0, 2) = U(r0 + 6, 0);
	}

	cout << "Z current " << endl <<  Z << endl;

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


bool MCcali::IterativelySolveForVariables1(ofstream& out, int& solved_var, bool verbose){

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
			}	else {
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

		VAR_TYPE VT = ReturnVarType(var_to_solve);
		if (verbose){
			cout << "We're solving for " << var_to_solve << endl;
		}

		out << "Iterative solving: solving for var " << var_to_solve << ", type " << VT <<  " b/c it has " << equations_per_var[var_to_solve].size() << " current equations " << endl;

		vector<Matrix4d> LHS;
		vector<Matrix4d> RHS;

		solved_var = var_to_solve;

		CreateTAequalsBproblem(var_to_solve, equations_per_var[var_to_solve], LHS, RHS, verbose);

		/// Solve with Shah's method ....
		Matrix4d Result;

		if (LHS.size() == 1){
			// TA = B
			Result = RHS[0]*LHS[0].inverse();
		}	else {
			// solve with a closed-form method.
			SolveWithShahsMethod(Result, LHS, RHS, verbose);
		}

		V_initial[var_to_solve] = Result;
		V_has_initialization[var_to_solve] = true;

		return true;
	}	else {
		return false;

	}

}


void MCcali::SelectKPointsForMinimization(const vector<CameraCali*>& CCV, int selectedk){

	// this is for an individual.
	int camera_index, pattern_graph_index, pattern_superscript, time_graph_index, time_superscript;

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

			for (int j = CCV[camera_index]->P_class->min_max_id_squares[pattern_superscript].first;
					j <= CCV[camera_index]->P_class->min_max_id_squares[pattern_superscript].second; j++){

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


			for (int j = CCV[camera_index]->P_class->min_max_id_squares[pattern_superscript].first;
					j <= CCV[camera_index]->P_class->min_max_id_squares[pattern_superscript].second; j++){

				if (CCV[camera_index]->points_present[time_superscript].at(j) == true){

					if (bool_map_sparse[s] == false){
						CCV[camera_index]->points_used_min[time_superscript].at(j) = false;
					}

					s++;
				}
			}


		}	else {
			for (int j = CCV[camera_index]->P_class->min_max_id_squares[pattern_superscript].first;
					j <= CCV[camera_index]->P_class->min_max_id_squares[pattern_superscript].second; j++){

				if (CCV[camera_index]->points_present[time_superscript].at(j) == true){
					CCV[camera_index]->points_used_min[time_superscript].at(j) = true;

				}
			}

		}
	}

}

bool MCcali::SolveClique(std::ofstream& out){
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

	if (best_numberfrs == 1){ // can't solve well w/ just one observation.
		cout << "There is only one observation, this solution is bound to be ill-conditioned.  " << endl;
	}

	// then find where this is happening ... create the optimization problem.

	// what is the case?  1st var has to be a camera
	bool second_var = 0;

	if (best_pair.second >= cn + pn){
		second_var = 1;
	}

	if (second_var == 0){
		out << "second var case is a pattern variable." << endl;
	}	else {
		out << "second var case is a time variable." << endl;
	}

	vector<Matrix4d> Arwhec;
	vector<Matrix4d> Brwhec;

	for (int i = 0, number_frs = NumberSingles(); i < number_frs; i++){

		if (singles_open[i] == true){
			if (singles[i].lhs == best_pair.first){


				switch (second_var){
				case 0:{
					if (singles[i].rhs[1] == best_pair.second){
						// pattern variable.
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
	ShahKroneckerProduct(Arwhec, Brwhec, X, Z, out);

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

	return true;

}

void MCcali::MinimizeReprojectionErrorMC(const vector<CameraCali*>& CCV, double* camera_params, ofstream& out,
		int start_id, int end_id,
		bool use_all_points_present){
	vector<bool> minimize_bit_vector(NumberVariables(), true);

	/// do not change the two exemplars, or anything that is not initialized.
	int t_ex_graph = t_star() + cn + pn;
	int p_ex_graph = p_star() + cn;

	for (int i = 0; i < NumberVariables(); i++){
		if (V_has_initialization[i] == true){
			if (i == t_ex_graph || i == p_ex_graph){
				minimize_bit_vector[i] = false;
			}
		}
	}

	MinimizeReprojectionError(*this, CCV, camera_params, minimize_bit_vector,  out,
			start_id, end_id, use_all_points_present);

}


double StdDeviation(vector<double>& v, double m)
{
	double E=0; double n = v.size();
	for(int i=0; i<int(v.size()); i++){
		E+=(v[i] - m)*(v[i] - m);
	}
	return sqrt(E/n);
}

void MCcali::ReconstructionAccuracyErrorAndWriteI(const string& write_dir, int current_item, const vector<CameraCali*>& CCV,
		double* camera_params, ofstream& out, const GroundTruthData* GTD){

	// input checking
	if (current_item < 0 && GTD == 0){
		cout << "Reconstruct accuracy error function, GTD has to be allocated and non-zero." << endl;
		exit(1);
	}

	string descriptor = "";
	string filename = "";

	switch (current_item){
	case -2: {
		descriptor = "ground-truth-relative";
	} break;
	case -1: {
		descriptor = "ground-truth-raw";
	} break;
	case 0: {
		descriptor = "initial";
	}	break;
	case 1: {
		descriptor = "minimization1";
	} break;
	case 2: {
		descriptor = "incremental";
	} break;
	}


	vector<Matrix4d> vector_to_use;
	switch (current_item){
	case -2: {
		vector_to_use = GTD->variables_rel;
	}
	break;
	case -1 : {
		vector_to_use = GTD->variables_raw;
	} break;
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

	ReconstructXFunctionIDsMC(*this, vector_to_use, CCV, camera_params,
			reconstructed_points, valid_reconstructed_points, out);


	// then process...
	double summed_error = 0;
	int n = 0;

	int number_pts = valid_reconstructed_points.size();

	Vector3d gt;

	string filename1 = write_dir + "EstimatedVsGroundTruth" + ToString<int>(current_item) + ".txt";
	ofstream ot; ot.open(filename1.c_str());

	double squared_norm;

	vector<double> norms;


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

			summed_error+= squared_norm;

			norms.push_back(squared_norm);

			ot << i << endl;
			ot << gt.transpose() << endl;
			ot << reconstructed_points[i].transpose() << endl;
			ot << "squared norm " << squared_norm << endl;
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

	int total_this_pattern;
	{
		string filename;

		// assuming all patterns have the same number of points, for now.
		PatternsCreated* P = CCV[0]->P_class;

		for (int k = 0; k< NumberPatterns(); k++){

			int number_this_pattern = P->min_max_id_squares[k].second - P->min_max_id_squares[k].first + 1;


			vector<Vector3d> ideal(number_this_pattern);
			for (int i = 0, j = P->min_max_id_squares[k].first;
					j <= P->min_max_id_squares[k].second; j++, i++){

				ideal[i](0) = P->three_d_points[j].x;
				ideal[i](1) = P->three_d_points[j].y;
				ideal[i](2) = P->three_d_points[j].z;
			}

			filename = write_dir + "world-ideal_pattern" + ToString<int>(k) + ".ply";
			WritePatterns(ideal, P->square_h_w[k].first - 1, P->square_h_w[k].second - 1, k, filename);

			filename = write_dir + "world-"+ descriptor + "-pattern"+ ToString<int>(k) + ".ply";

			vector<Vector3d> current_points;
			/// need to make new vectors without skips ...
			for (int j = P->min_max_id_squares[k].first;
					j <= P->min_max_id_squares[k].second; j++){
				if (valid_reconstructed_points[j]){
					current_points.push_back(reconstructed_points[j]);
				}

			}

			total_this_pattern = P->min_max_id_squares[k].second - P->min_max_id_squares[k].first + 1;

			out << "total this pattern, current size " << total_this_pattern << ", " << current_points.size() << endl;

			if (int(current_points.size()) == total_this_pattern){
				WritePatterns(current_points, P->square_h_w[k].first - 1, P->square_h_w[k].second - 1, k + 1, filename);
			}	else {
				WritePatternsSkips(current_points, k + 1, filename);
			}


			current_points.clear();

		}
	}


	// Ground truth case.
	if (current_item == -2){
		PatternsCreated* P = CCV[0]->P_class;

		if (GTD->points_rel.size() == P->three_d_points.size()){

			for (int k = 0; k< NumberPatterns(); k++){
				filename = write_dir + "at-P-" +  descriptor + "-pattern"+ ToString<int>(k) + ".ply";

				vector<Vector3d> current_points;

				/// need to make new vectors without skips ...
				for (int j = P->min_max_id_squares[k].first;
						j <= P->min_max_id_squares[k].second; j++){
					{
						current_points.push_back(GTD->points_rel[j]);
					}
				}

				total_this_pattern = P->min_max_id_squares[k].second - P->min_max_id_squares[k].first + 1;

				out << "total this pattern, current size " << total_this_pattern << ", " << current_points.size() << endl;

				if (int(current_points.size()) == total_this_pattern){
					WritePatterns(current_points, P->square_h_w[k].first - 1, P->square_h_w[k].second - 1, k + 1, filename);
				}	else {
					WritePatternsSkips(current_points, k + 1, filename);
				}

				current_points.clear();
			}
		}

	}

	// write patterns as computed by the optimization
	if (current_item == 0 || current_item == 1 || current_item == 2){
		// write the pattern as computed w/ the pattern vars.
		PatternsCreated* P = CCV[0]->P_class;
		Vector4d xprior;  xprior.setConstant(1);
		Vector4d xpost;  Vector3d x;
		int pattern_graph_index;

		{
			for (int p = 0; p< NumberPatterns(); p++){
				filename = write_dir + "at-P-" + descriptor + "-pattern"+ ToString<int>(p) + ".ply";

				vector<Vector3d> current_points;

				pattern_graph_index = p + cn;

				/// need to make new vectors without skips ...
				for (int j = P->min_max_id_squares[p].first;
						j <= P->min_max_id_squares[p].second; j++){

					xprior(0) = P->three_d_points[j].x;
					xprior(1) = P->three_d_points[j].y;
					xprior(2) = P->three_d_points[j].z;

					xpost  = vector_to_use[pattern_graph_index].inverse()*xprior;

					for (int k= 0; k < 3; k++){
						x(k) = xpost(k);
					}

					current_points.push_back(x);
				}

				total_this_pattern = P->min_max_id_squares[p].second - P->min_max_id_squares[p].first + 1;

				out << "total this pattern, current size " << total_this_pattern << ", " << current_points.size() << endl;

				if (int(current_points.size()) == total_this_pattern){
					WritePatterns(current_points, P->square_h_w[p].first - 1, P->square_h_w[p].second - 1, p + 1, filename);
				}	else {
					WritePatternsSkips(current_points, p + 1, filename);
				}

				current_points.clear();
			}
		}
	}

	// update
	Points_progressive_solutions_strict.push_back(reconstructed_points);
	Valid_progressive_solutions_strict.push_back(valid_reconstructed_points);
}

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



	}	else {
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

	// write the As.  For some reason this is now missing.
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

	if (write_docs) {WriteLatex(write_dir, "doc-first-unreduced-singles", false); }

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

	cout << "Number cameras not in the connected component = " << sum_not_in << endl;
	cout << "Number in the connected component = " << cn - sum_not_in << endl;

	uninitialized = 0;
	for (int i = 0; i < NumberVariables(); i++){
		if (!V_has_initialization[i]){
			uninitialized++;
		}
	}

	return sum_not_in;
}

void MCcali::WriteASide(const vector<int>& side, ofstream& out, bool write_exemplars){

	bool has_p;
	bool has_t;
	int a_count;
	int p_graph_index;
	int t_graph_index;
	int p_superscript;
	int t_superscript;
	int c_superscript;


	a_count = side[0];
	c_superscript = A_camera_indices[a_count];
	int a_pattern_superscript = A_pattern_indices[a_count];
	int a_time_superscipt = A_time_indices[a_count];

	p_graph_index = side[1];
	p_superscript = V_index[p_graph_index];

	bool is_exemplar_p = IsPstar(p_superscript);

	if (is_exemplar_p && !write_exemplars){
		is_exemplar_p = !is_exemplar_p;
	}

	has_p = !is_exemplar_p && (p_superscript >= 0); // p_star is in p-space OR this has been factored out..

	t_graph_index = side[2];
	t_superscript = V_index[t_graph_index];

	bool is_exemplar_t = IsTstar(t_superscript);
	if (is_exemplar_t && !write_exemplars){
		is_exemplar_t = !is_exemplar_t;
	}

	has_t = !is_exemplar_t && (t_superscript >= 0); // tstar is in t-space OR this has been factored out.

	// A
	if (!is_exemplar_p && !is_exemplar_t){
		out << "^{c_{"<< c_superscript<< "} }\\mathbf{A}_{p_{"<<a_pattern_superscript<<"} @ t_{"<< a_time_superscipt << "}}";
	}	else {

		if (!is_exemplar_p){
			out << "^{c_{"<< c_superscript<< "} }\\mathbf{A}_{p_{"<<a_pattern_superscript<<"} @ t_{"<<a_time_superscipt << "}^*}";
		}	else {

			if (!is_exemplar_t){
				out << "^{c_{"<< c_superscript<< "} }\\mathbf{A}_{p_{"<<a_pattern_superscript<<"}^* @ t_{"<< a_time_superscipt << "}}";
			}	else {
				out << "^{c_{"<< c_superscript<< "} }\\mathbf{A}_{p_{"<<a_pattern_superscript<<"}^* @ t_{"<< a_time_superscipt << "}^*}";
			}
		}
	}

	// p and t
	if (has_p){
		if (V_has_initialization[p_graph_index]){
			out << "{^{" << p_superscript << "}\\mathds{P}}";
		}	else {
			out << "{^{" << p_superscript << "}\\mathbf{P}}";
		}
	}

	if (has_t){
		if (V_has_initialization[t_graph_index]){
			out << "{^{" << t_superscript << "}\\mathds{T}}";
		}	else {
			out << "{^{" << t_superscript << "}\\mathbf{T}}";
		}
	}

}


void MCcali::WriteCameraCalibrationResult(const vector<CameraCali*>& CCV, const vector<string>& camera_names,
		const vector<Matrix4d>& ext_to_use, const string& filename){

	ofstream out;
	out.open(filename.c_str());

	if (!out.good()){
		cout << "filename for write is not good, exiting." << endl;
		exit(1);

	}	else {

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


void MCcali::WriteCameraCalibrationResult(const GroundTruthData* GTD, const vector<string>& camera_names,
		const vector<Matrix4d>& ext_to_use, const string& filename, int rows){

	// we will choose different versions to write ... intial, first cost, second cost, etc.
	ofstream out;
	out.open(filename.c_str());

	if (!out.good()){
		cout << "filename for write is not good, exiting." << endl;
		exit(1);

	}	else {

		out << cn << endl;

		for (int i = 0; i < cn; i++){

			out << camera_names[i] << " ";
			for (int r = 0; r < 3; r++){
				for (int c = 0; c < 3; c++){
					out << GTD->InternalMatrices[i](r, c) << " ";
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

			for (int r = 0, rn = rows; r < rn; r++){
				out << 0 << " ";
			}

			out << endl;

		}
	}

}

void MCcali::WriteLatex(const string& write_dir, const string& descriptor, bool write_exemplars){


	/// if we have a var with graph index vn, do not draw. Just a placeholder.
	// \mathds for initilialized
	cout << "in write latex " << descriptor << endl;
	int c_superscript;
	int c_graph_index;

	int max_number_per_column = 30;

	string filename = write_dir + descriptor + ".tex";


	ofstream out;
	out.open(filename.c_str());

	out << "\\documentclass[10pt,twocolumn,letterpaper]{article}" << endl;
	out << "\\usepackage{times}" << endl;
	out << "\\usepackage{epsfig}" << endl;
	out << "\\usepackage{dsfont}" << endl;
	out << "\\usepackage{graphicx}" << endl;
	out << "\\usepackage{amsmath}" << endl;
	out << "\\usepackage{amssymb}" << endl;
	out << "\\usepackage{color}" << endl;
	out << "\\usepackage{algorithm}" << endl;
	out << "\\usepackage[noend]{algpseudocode}" << endl;
	out << "\\usepackage{mathtools}" << endl;
	out << "\\begin{document}" << endl;

	out << "\\section{ Section " << descriptor << "}" << endl;

	out << "p-star is " << p_star() << " and t-star is " << t_star() << endl;

	for (int a_count = 0; a_count < an; a_count++){
		if (a_count % max_number_per_column == 0){
			if (a_count > 0){
				out << "\\end{align}" << endl;
			}

			out << "\\begin{align} " << endl;
		}

		// go through this -- is there a pattern AND a time?
		c_graph_index = singles[a_count].lhs;
		if (c_graph_index >= 0){

			c_superscript = V_index[c_graph_index];
		}	else {
			cout << "Major error!  camera matrix on a single is empty." << endl;
			exit(1);
		}

		if (V_has_initialization[c_graph_index]){
			out << "^{" << c_superscript << "}\\mathds{C} =& ";
		}	else {
			out << "^{" << c_superscript << "}\\mathbf{C} =& ";
		}

		WriteASide(singles[a_count].rhs, out, write_exemplars);


		if (a_count < an - 1){
			out << "\\\\" << endl;
		}	else {
			out << endl;
		}

	}
	out << "\\end{align}" << endl;

	out << "Initialized variables: " << endl;
	for (int i = 0; i < vn; i++){
		if (V_has_initialization[i] == true){
			switch (V_type[i]){
			case camera_var:{
				out << "$^{" << i << "}\\mathbf{C}$ ";
			} break;
			case pattern_var:{
				out << "${^{" << V_index[i] << "}\\mathbf{P}}$";
			} break;
			case time_var: {
				out << "${^{" << V_index[i] << "}\\mathbf{T}}$";
			} break;
			case special_var: {
				// do nothing.
			} break;
			}

			out << endl;
		}
	}

	out << endl << endl << "Uninitialized variables: "<< endl;
	for (int i = 0; i < vn; i++){
		if (V_has_initialization[i] == false){
			switch (V_type[i]){
			case camera_var:{
				out << "$^{" << i << "}\\mathbf{C}$ ";
			} break;
			case pattern_var:{
				out << "${^{" << V_index[i] << "}\\mathbf{P}}$";
			} break;
			case time_var: {
				out << "${^{" << V_index[i] << "}\\mathbf{T}}$";
			} break;
			case special_var: {
				// do nothing.
			} break;
			}
			out << endl;
		}
	}

	out << "\\end{document}" << endl;
	out.close();

	string command = string("pdflatex ") +  " -output-directory=" + write_dir + " " + filename + " > pdflatexlog.out";
	int ret_val = system(command.c_str());

	if (ret_val != 0){
	    cout << "error with " << command << endl;
	    cout << "at " << __LINE__ << " of " << __FILE__ << endl;
	}

}

void MCcali::InitializeNumberOccurrencesAndInitialization(){
	for (int i = 0; i < an; i++){
		A_number_occurrences[i] = 0;
	}

	for (int i = 0; i < vn; i++){
		V_number_occurrences[i] = 0;
		V_has_initialization[i] = false;
	}
}

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
		}	else {
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
		}	else {
			cout << "Bad pattern variable case, " << __LINE__ << " in " << __FILE__ << endl;
			exit(1);
		}

		if (has_t){
			V_number_occurrences[t_graph_index]++;
		}	else {
			cout << "Bad time variable case, " << __LINE__ << " in " << __FILE__ << endl;
			exit(1);
		}
	}

}


single_relationship_container::single_relationship_container(int l){
	lhs = l;
	rhs.resize(3, -1); /// initialize to Identity;
}

int single_relationship_container::count_var_number_rhs(const vector<int>& V_index, int p_star, int t_star) const{

	int number_present = 0;

	//P
	number_present += !IsValue(p_star, V_index[rhs[1]]);

	//T
	number_present += !IsValue(t_star, V_index[rhs[2]]);

	return number_present;

}


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

void MCcali::WriteSimulatedCamerasAtAllTimes(const string& write_directory, const string& current_dir,
        const vector<CameraCali*>& CCV,
		float camera_size, float track_size, const vector<Matrix4d>& vector_to_use){

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


					create_camera(CCV[i]->internal_parameters, ProposedMat, camera_size, 0, 255, 255,
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

	create_cameras(internals, externals, 0, 255, 255,
			CCV[0]->rows, CCV[0]->cols, filename, camera_size);

}

void MCcali::WriteSimulatedCamerasForRotatingCase(const string& write_directory, const string& current_dir,
        const vector<CameraCali*>& CCV,
		float camera_size, float track_size, const vector<Matrix4d>& vector_to_use){

	Matrix4d ProposedMat;
	Matrix4d TimeMat;



	Vector3d center;

	string filename;

	Vector3d offset;  offset.setZero();  offset(1) = -1;

	for (int i = 0; i < NumberCameras(); i++){
		Matrix4d Cam;
		vector<Vector3d> one_track;

		vector<Matrix3d> internals;
		vector<Matrix4d> externals;

		int local_time_start = CCV[i]->start_time_this_camera;

		int local_time_stop = 0;
		if ((i + 1) == NumberCameras()){
			local_time_stop = tn;
		}	else {
			local_time_stop = CCV[i + 1]->start_time_this_camera;
		}

		cout << "Writing camera ply file for camera " << i << endl;


		if (V_has_initialization[i] == true){
			Cam = vector_to_use[i];

			for (int t = local_time_start; t < local_time_stop; t++){

				if (V_has_initialization[cn + pn + t] == true){
					TimeMat = vector_to_use[cn + pn + t];

					ProposedMat = Cam*TimeMat.inverse();

					internals.push_back(CCV[i]->internal_parameters);
					externals.push_back(ProposedMat);

					filename  =  write_directory + current_dir + "/c"+
							ToString<int>(i) + "_time" + ToString<int>(t) + ".ply";


					create_camera(CCV[i]->internal_parameters, ProposedMat, camera_size, 0, 255, 255,
							CCV[i]->rows, CCV[i]->cols, filename);

					center = ReturnCenter(ProposedMat);

					one_track.push_back(center);
				}
			}

			filename  =  write_directory + current_dir + "/rotating_track"+
					ToString<int>(i) + ".ply";

			create_tracks(one_track, 0, 0, 0, track_size, offset, filename);

			filename  =  write_directory + current_dir + "/rotating_cameras" +  ToString<int>(i) + ".ply";

			create_cameras(internals, externals, 0, 255, 255,
					CCV[0]->rows, CCV[0]->cols, filename, camera_size);
		}
	}
}


void MCcali::WriteCalibrationFileForSimulatedCamerasAtAllTimes(const string& write_directory,  const vector<CameraCali*>& CCV){

	string descriptor = "initialization";

	Matrix4d ProposedMat;
	Matrix4d TimeMat;

	string filename;
	ofstream out;
	string pair_filename;

	string image_name;

	Matrix4d ProposedMatprior;

	Matrix4d currentPair;

	// don't reduce size, testing.
	double size_reduction  = 1;

	filename = write_directory + "cali_camera_all.txt";
	out.open(filename.c_str());

	int total_cameras_combined = 0;

	for (int i = 0; i < NumberCameras(); i++){
		if (V_has_initialization[i]){
			int local_time_start = CCV[i]->start_time_this_camera;

			int local_time_stop = 0;
			if ((i + 1) == NumberCameras()){
				local_time_stop = tn;
			}	else {
				local_time_stop = CCV[i + 1]->start_time_this_camera;
			}

			for (int t = local_time_start; t < local_time_stop; t++){
				if (V_has_initialization[cn + pn + t] == true){
					total_cameras_combined++;
				}
			}
		}
	}

	out << total_cameras_combined << endl;

	for (int i = 0; i < NumberCameras(); i++){
		Matrix4d Cam;

		int local_time_start = CCV[i]->start_time_this_camera;

		int local_time_stop = 0;
		if ((i + 1) == NumberCameras()){
			local_time_stop = tn;
		}	else {
			local_time_stop = CCV[i + 1]->start_time_this_camera;
		}

		if (V_has_initialization[i]){
			Cam = V_initial[i];

			cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

			cv::Mat distCoeffs = cv::Mat::zeros(CCV[i]->distortion.rows(), 1, CV_64F);

			cv::Mat rotMatrix = cv::Mat::eye(3, 3, CV_64F);

			cv::Mat tMatrix = cv::Mat::zeros(3, 1, CV_64F);

			for (int r = 0; r < 3; r++){
				for (int c = 0; c < 3; c++){
					cameraMatrix.at<double>(r, c) = CCV[i]->internal_parameters(r, c)/size_reduction;
				}
			}

			cameraMatrix.at<double>(2, 2) = 1;

			for (int r = 0; r < CCV[i]->distortion.rows(); r++){
				distCoeffs.at<double>(r, 0) = CCV[i]->distortion(r);
			}


			for (int t = local_time_start; t < local_time_stop; t++){

				if (V_has_initialization[cn + pn + t] == true){

					TimeMat = V_initial[cn + pn + t];

					ProposedMat = Cam*TimeMat.inverse();

					out << CCV[i]->im_names[t - CCV[i]->start_time_this_camera] << " ";

					// internal K
					for (int r = 0; r < 3; r++){
						for (int c = 0; c < 3; c++){
							out << CCV[i]->internal_parameters(r, c) << " ";
						}
					}

					/// R
					for (int r = 0; r < 3; r++){
						for (int c = 0; c < 3; c++){
							out << ProposedMat(r, c) << " ";
						}
					}

					// t
					for (int r = 0; r < 3; r++){
						for (int c = 3; c < 4; c++){
							out << ProposedMat(r, c) << " ";
						}
					}

					// distortion --
					for (int r = 0, rn = CCV[i]->distortion.rows(); r < rn; r++){
						out << CCV[i]->distortion(r) << " ";
					}
					out << endl;
				}

			}
		}
	}
	out.close();
}


bool MCcali::AllInitialized(const vector<int>& side){
	for (int j = 1; j < 3; j++){
		if (V_has_initialization[side[j]] == false){
			return false;
		}
	}

	return true;
}

GroundTruthData::GroundTruthData(const string& input){

	string filename = input + "CameraTransformations.txt";
	ifstream in(filename.c_str());

	TestStream(in, filename);

	// read in.

	int cn;
	in >> cn;

	Matrix4d M;

	for (int i = 0; i < cn; i++){
		for (int r = 0; r < 4; r++){
			for (int c = 0; c < 4; c++){

				in >> M(r, c);

			}
		}
		CameraTransformations.push_back(M);
		variables_raw.push_back(M);
	}

	in.close();

	///////////////// INTERNALS ///////////////////////

	filename = input + "CameraTransformationsOpenGL.txt";

	Matrix3d M3;

	string temp;

	in.open(filename.c_str());
	TestStream(in, filename);

	for (int i = 0; i < cn; i++){

		in >> temp;
		// read the internal matrices.
		for (int r = 0; r < 3; r++){
			for (int c = 0; c < 3; c++){
				in >> M3(r, c);
			}
		}

		InternalMatrices.push_back(M3);

		for (int r = 0; r < 4; r++){
			for (int c = 0; c < 4; c++){

				in >> M(r, c);

			}
		}
	}

	in.close();

	////////////////////// Patterns //////////////////////////
	filename = input + "PatternTransformations.txt";

	int pn = -1;
	in.open(filename.c_str());
	TestStream(in, filename);

	in >> pn;
	for (int i = 0; i < pn; i++){

		for (int r = 0; r < 4; r++){
			for (int c = 0; c < 4; c++){

				in >> M(r, c);

			}
		}
		Matrix4d temp = M.inverse();
		M = temp;

		PatternTransformations.push_back(M);
		variables_raw.push_back(M);
	}

	in.close();

	///////////////// Time ////////////////////////

	filename = input + "TimeTransformations.txt";

	int tn = -1;
	in.open(filename.c_str());
	TestStream(in, filename);

	in >> tn;
	for (int i = 0; i < tn; i++){

		for (int r = 0; r < 4; r++){
			for (int c = 0; c < 4; c++){

				in >> M(r, c);

			}
		}

		Matrix4d temp = M.inverse();
		M = temp;

		TimeTransformations.push_back(M);
		variables_raw.push_back(M);
	}

	in.close();
}

void GroundTruthData::ComputeRelativeToExemplar(int p_star, int t_star, const string& outputdir, PatternsCreated* P_class){

	string filename;

	ofstream out;

	Matrix4d M;
	Matrix4d Minv;
	// need to create relative C, P, T.
	// all the stars are in subscript coords.

	M = PatternTransformations[p_star]*TimeTransformations[t_star];
	Minv= M.inverse();
	filename = outputdir + "CamerasRelativeComputed.txt";
	out.open(filename.c_str());

	TestStream(out, filename);

	out << CameraTransformations.size() << endl;
	for (int i = 0, cn  = CameraTransformations.size(); i < cn; i++){
		M = CameraTransformations[i]*Minv;
		CamerasRel.push_back(M);
		variables_rel.push_back(M);
		out << M << endl;
	}
	out.close();

	//////// Patterns /////////////////
	filename = outputdir + "PatternsRelativeComputed.txt";

	out.open(filename.c_str());

	TestStream(out, filename);


	out << PatternTransformations.size() << endl;
	Minv = PatternTransformations[p_star].inverse();
	for (int i = 0, pn  = PatternTransformations.size(); i < pn; i++){
		M = Minv*PatternTransformations[i];
		M = PatternTransformations[i];
		PatternsRel.push_back(M);
		variables_rel.push_back(M);
		out << M << endl;
	}

	out.close();

	///////////// Times ////////////////
	filename = outputdir + "TimeRelativeComputed.txt";

	out.open(filename.c_str());

	TestStream(out, filename);

	M = PatternTransformations[p_star]*TimeTransformations[t_star];
	Minv= M.inverse();

	out << "Test1 " << endl << M << endl << "Test2 " << Minv << endl;
	out << TimeTransformations.size() << endl;

	for (int i = 0, tn  = TimeTransformations.size(); i < tn; i++){
		M = TimeTransformations[i]*Minv;
		TimesRel.push_back(M);
		variables_rel.push_back(M);
		out << M << endl;
	}

	out.close();

	// Then, for RAE need to add an identity matrix ...
	M.setIdentity();
	variables_raw.push_back(M);
	variables_rel.push_back(M);

	Vector4d xprior;
	Vector4d xpost;
	Vector3d x;
	xprior.setConstant(1);

	points_rel.resize(P_class->three_d_points.size());

	// create threed points that correspond to the pattern transformations ...
	for (int p = 0, pn = PatternTransformations.size(); p < pn; p++ ){
		for (int j = P_class->min_max_id_squares[p].first;
				j <= P_class->min_max_id_squares[p].second; j++){

			xprior(0) = P_class->three_d_points[j].x;
			xprior(1) = P_class->three_d_points[j].y;
			xprior(2) = P_class->three_d_points[j].z;

			xpost = PatternsRel[p].inverse()*xprior;

			for (int k = 0; k < 3; k++){
				x(k) = xpost(k);
			}

			points_rel[j] = x;
		}
	}
}

void MCcali::AssessCamerasWRTGroundTruth(const GroundTruthData* GTD, int soln_number){

	int current_n = average_rot_angle_error.size();
	std_rot_angle_error.push_back(0);
	median_rot_angle_error.push_back(0);

	std_rot_error.push_back(0);
	median_rot_error.push_back(0);

	std_translation_error.push_back(0);
	median_translation_error.push_back(0);

	std_whole_error.push_back(0);
	median_whole_error.push_back(0);

	double er = AssessRotationErrorAxisAngle(GTD->variables_rel, V_progressive_solutions[soln_number],
			cn, std_rot_angle_error[current_n], median_rot_angle_error[current_n]);

	average_rot_angle_error.push_back(er);

	er = AssessRotationError(GTD->variables_rel, V_progressive_solutions[soln_number], cn,
			std_rot_error[current_n], median_rot_error[current_n]);

	average_rot_error.push_back(er);

	double et = AssessTranslationError(GTD->variables_rel, V_progressive_solutions[soln_number], cn,
			std_translation_error[current_n], median_translation_error[current_n]);

	average_translation_error.push_back(et);

	double ew = AssessErrorWhole(GTD->variables_rel, V_progressive_solutions[soln_number], cn,
			std_whole_error[current_n], median_whole_error[current_n]);

	average_whole_error.push_back(ew);
}


void MCcali::WriteSolutionAssessError(const string& write_directory, const vector<string>& camera_names,
        const vector<CameraCali*>& CCV, int type,
		const GroundTruthData* GTD, bool rotating, bool write, float camera_size, float track_size ){
	// last two are empty if the dataset is not simulated.

	type_recorder.push_back(type);

	string filename, command;
	string descriptor;

	vector<Matrix4d> vector_to_use;
	switch (type){
	case -2: {
		vector_to_use = GTD->variables_rel;
	}
	break;
	case -1 : {
		vector_to_use = GTD->variables_raw;
	} break;
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
	case -2: {
		descriptor = "ground-truth-rel";
	} break;
	case -1: {
		descriptor = "ground-truth-raw";
	} break;
	case 0: {
		descriptor = "initial";
	}	break;
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
	case -2: {
		WriteCameraCalibrationResult(GTD, camera_names, vector_to_use, filename, CCV[0]->distortion.rows());
	}	break;
	case -1: {
		WriteCameraCalibrationResult(GTD, camera_names, vector_to_use, filename, CCV[0]->distortion.rows());
	}	break;
	case 0: {
		WriteCameraCalibrationResult(CCV, camera_names, vector_to_use, filename);
	}	break;
	case 1: {
		WriteCameraCalibrationResult(CCV, camera_names, vector_to_use, filename);
	} break;
	}

	string current_dir = "cameras-" + descriptor;

	mkdir((write_directory + current_dir).c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

	string cam_dir = write_directory + current_dir + "/single_cameras";

	mkdir(cam_dir.c_str(),  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

	vector<Matrix3d> internals;
	vector<Matrix4d> externals;

	Vector3f cam_color;

	switch (type){
	case -2: {
		cam_color << 230, 159, 0;
	} break;
	case -1:{
		cam_color << 230, 159, 0;
	}break;
	case 0:{
		cam_color << 0, 114, 178;
	} break;
	case 1:{
		cam_color << 0, 158, 115;
	} break;
	case 2:{
		cam_color << 0, 158, 0;
	} break;
	}

	for (int i = 0; i < NumberCameras(); i++){

		filename = cam_dir + "/c"+ ToString<int>(i) + ".ply";

		switch (type){
		case -2: {
			create_camera(GTD->InternalMatrices[i], vector_to_use[i], camera_size, cam_color(0), cam_color(1),
					cam_color(2), CCV[i]->rows, CCV[i]->cols, filename);
		} break;
		case -1: {
			create_camera(GTD->InternalMatrices[i], vector_to_use[i], camera_size, cam_color(0), cam_color(1),
					cam_color(2), CCV[i]->rows, CCV[i]->cols, filename);
		} break;
		default: {
			if (V_has_initialization[i]){
				// rows and cols are empty
				create_camera(CCV[i]->internal_parameters, vector_to_use[i], camera_size, cam_color(0), cam_color(1),
						cam_color(2), CCV[i]->rows, CCV[i]->cols, filename);
				internals.push_back(CCV[i]->internal_parameters);
				externals.push_back(vector_to_use[i]);
			}
		} break;

		}

	}

	filename = cam_dir + "/all.ply";

	if (internals.size() > 0){
		create_cameras(internals, externals, cam_color(0), cam_color(1), cam_color(2), CCV[0]->rows, CCV[0]->cols, filename, camera_size);
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


	{

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
	}

	cost_function_error_by_type_c1.push_back(id_error_c1);
	cost_function_error_by_type_c2.push_back(id_error_c2);

	// ceres answer will be 1/2 by construction
	summed_c1_cost_function_error_by_type.push_back(c1_acc);
	summed_c2_cost_function_error_by_type.push_back(c2_acc);


	string current_write_dir= write_directory + current_dir + "/";
	vector<double> reprojection_error(NumberSingles(), 0);



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
					reprojection_error[i] = CCV[camera_index]->ComputeReprojectionErrorOneImagePattern(Aprime, local_time_index,
							pattern_superscript, current_write_dir, write, i, rotating, &GTD->InternalMatrices[camera_index]);
				}	else {
					reprojection_error[i] = CCV[camera_index]->ComputeReprojectionErrorOneImagePattern(Aprime, local_time_index,
							pattern_superscript, current_write_dir, write, i, rotating, 0);
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
	reprojection_error_by_term_and_type.push_back(reprojection_error);

	if (rotating){
		WriteSimulatedCamerasForRotatingCase(write_directory, current_dir, CCV, camera_size, track_size, vector_to_use);
	}	else {
		WriteSimulatedCamerasAtAllTimes(write_directory, current_dir, CCV, camera_size, track_size, vector_to_use);
	}

}

void MCcali::OutputVariablesWithInitialization(const string& filename, int type){

	ofstream out;
	out.open(filename.c_str());

	vector<Matrix4d> vector_to_use;
	switch (type){
	case -4: {
		vector_to_use = V_ground_truth_relative_original;
	} break;
	case -1 : {
		vector_to_use = V_ground_truth;
	} break;
	case 0: {
		vector_to_use = V_initial;
	} break;
	case 1: {
		vector_to_use = V_first_solutiona;
	} break;
	case 2: {
		vector_to_use = V_first_solutionb;
	} break;
	}

	int v = 0;
	for (int i = 0; i < cn; i++, v++){
		if (V_has_initialization[v] == true){
			out << "C " << i << endl;
			out << vector_to_use[v] << endl;
		}	else {
			out << "C " << i << " not initialized yet " <<  endl;
		}

	}

	for (int i = 0; i < pn; i++, v++){
		if (V_has_initialization[v] == true){
			out << "P " << i << ", " << v <<  endl;
			out << vector_to_use[v] << endl;
		}	else {
			out << "P " << i << ", " << v <<  " does not have initialization yet " << endl;
		}
	}

	for (int i = 0; i < tn; i++, v++){
		if (V_has_initialization[v] == true){
			out << "T " << i << ", " << v <<  endl;
			out << vector_to_use[v] << endl;
		}	else {
			out << "T " << i << ", " << v <<  " does not have initialization yet " << endl;
		}
	}

	out.close();

}


void MCcali::OutputRunResults(const string& filename){

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
	out << "Reprojection error " << endl;
	double sum_err = 0;

	vector<double> reproj_sums;
	for (int i = 0; i <number_types; i++){
		sum_err = 0;
		for (int j = 0; j < NumberSingles(); j++){
			sum_err += reprojection_error_by_term_and_type[i][j];
		}
		reproj_sums.push_back(sum_err);
	}

	for (int i = 0; i <number_types; i++){
		out << descriptor_recorder[i] << ": ";
		out << "Reprojection error, rrmse: " << sqrt(reproj_sums[i]/double(NumberSingles())) << endl;
	}

	out << endl << endl;
	out << "Reconstruction accuracy error (RAE).  Note that these values are squared (equations 18, 19)" << endl;
	out << "Number valid image points  " << number_valid_points_rae_strict << endl;

	for (int i = 0; i <number_types; i++){
		out << descriptor_recorder[i] << ": ";

		out << "RAE w/ BA, average, stddev, median : " << average_rae_ba[i];
		out << ", " << stddev_rae_ba[i] << ", " << median_rae_ba[i] << endl;

		out << "SQRT -- RAE w/ BA, average, stddev, median : " << sqrt(average_rae_ba[i]);
		out << ", " << sqrt(stddev_rae_ba[i]) << ", " << sqrt(median_rae_ba[i]) << endl;
	}


	out << endl << endl;
	if (average_rae_multi.size() > 0){

		out << "Number valid image points multi " << number_valid_points_rae_multi << endl;
		for (int i = 0; i <number_types; i++){
			out << descriptor_recorder[i] << ": ";
			out << "RAE multi, average, stddev, median : " << average_rae_multi[i] << ", " << stddev_rae_multi[i] << ", " << median_rae_multi[i] << endl;
		}
	}

	if (average_rot_angle_error.size() > 0){
		// off by 2 in this case.
		out << "Comparison to camera ground truth. " << endl;


		for (int i = 0, in = average_rot_angle_error.size(); i < in; i++){
			out << descriptor_recorder[i + 2] << ": ";
			out << "Average rotation angle, Average rotation error, Average translation error, average whole error : " << endl;
			out << "rotation angle, average, stddev, median: " << average_rot_angle_error[i] << ", "
					<< std_rot_angle_error[i] << ", " << median_rot_angle_error[i] << endl;
			out << "rotation, average, stddev, median: " << average_rot_error[i] << ", "
					<< std_rot_error[i] << ", " << median_rot_error[i] << endl;
			out << "translation, average, stddev, median: " << average_translation_error[i] << ", "
					<< std_translation_error[i] << ", " << median_translation_error[i] << endl;
			out << "whole, average, stddev, median: " << average_whole_error[i] << ", "
					<< std_whole_error[i] << ", " << median_whole_error[i] << endl;
			out << "----" << endl;
		}

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
			out << reprojection_error_by_term_and_type[i][j] << " ";
		}
		out <<  " Total " << sum_err << endl;

	}


	out.close();

}

double AssessRotationError(const vector<Matrix4d>& Cgts, const vector<Matrix4d>& Cnews, int number_Cs,
        double& stddev, double& median){
	double error = 0;
	double local_error = 0;

	Matrix4d H;
	vector<double> error_vector;

	double number_stops = number_Cs;



	for (int i = 0; i < number_Cs; i++){

		H = Cgts[i] - Cnews[i];

		local_error = 0;
		for (int r = 0; r < 3; r++){
			for (int c = 0; c < 3; c++){
				local_error += H(r, c)*H(r, c);
			}
		}

		error_vector.push_back(local_error);

		error += local_error;
	}


	double mu = error/number_stops;

	stddev = StdDeviation(error_vector, mu);

	std::sort(error_vector.begin(), error_vector.end());

	median = error_vector[int(number_stops/2)];

	return mu;
}


double AssessRotationErrorAxisAngle(const vector<Matrix4d>& Cgts, const vector<Matrix4d>& Cnews,
		int number_Cs, double& stddev, double& median){
	double error = 0;

	Matrix4d H;

	double number_stops = number_Cs;
	double local_error = 0;

	Matrix3d R0;
	Matrix3d R1;
	Matrix3d R_relative;


	double RV[9];
	double aa[3];

	vector<double> error_vector;

	for (int i = 0; i < number_Cs; i++){

		// try once for now .....

		for (int r = 0; r < 3; r++){
			for (int c = 0; c < 3; c++){
				R0(r, c) = Cgts[i](r, c);
				R1(r, c) = Cnews[i](r, c);
			}
		}

		R_relative = R1.transpose()*R0;
		for (int r = 0, index = 0; r < 3; r++){
			for (int c = 0; c < 3; c++, index++){
				RV[index] = R_relative(r, c);
			}
		}

		ceres::RotationMatrixToAngleAxis(RV, aa);

		local_error = 57.2958 * sqrt(pow(aa[0], 2) + pow(aa[1], 2) + pow(aa[2], 2));

		error_vector.push_back(local_error);
		error += local_error;

	}

	double mu = error/number_stops;

	stddev = StdDeviation(error_vector, mu);

	std::sort(error_vector.begin(), error_vector.end());

	median = error_vector[int(number_stops/2)];

	// remember to average amoung cameras when there is more than one camera.
	return mu;
}

double AssessTranslationError(const vector<Matrix4d>& Cgts, const vector<Matrix4d>& Cnews, int number_Cs,
		double& stddev, double& median){
	double error = 0;

	Matrix4d H;

	double number_stops = number_Cs;

	double local_error;
	vector<double> error_vector;

	for (int i = 0; i < number_Cs; i++){

		H = Cgts[i] - Cnews[i];

		local_error = 0;

		for (int r = 0; r < 3; r++){
			local_error += H(r, 3)*H(r,3);
		}

		error_vector.push_back(local_error);

		error += local_error;

	}
	double mu = error/number_stops;

	stddev = StdDeviation(error_vector, mu);

	std::sort(error_vector.begin(), error_vector.end());

	median = error_vector[int(number_stops/2)];

	// this error is squared within the parentheses
	// Within the main loop, average amoung cameras for multi-camera datasets.
	return mu;
}

double AssessErrorWhole(const vector<Matrix4d>& Cgts, const vector<Matrix4d>& Cnews, int number_Cs, double& stddev, double& median){
	double error = 0;
	Matrix4d H;

	double local_error;
	vector<double> error_vector;

	double number_stops = number_Cs;

	for (int i = 0; i < number_Cs; i++){

		H = Cgts[i] - Cnews[i];

		local_error = 0;
		for (int r = 0; r < 4; r++){
			for (int c = 0; c < 4; c++){
				local_error +=  H(r, c)*H(r, c);
			}
		}
		error_vector.push_back(local_error);

		error += local_error;
	}

	double mu = error/number_stops;

	stddev = StdDeviation(error_vector, mu);

	std::sort(error_vector.begin(), error_vector.end());

	median = error_vector[int(number_stops/2)];

	return mu;
}




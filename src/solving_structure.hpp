/*
 * solving_structure.hpp
 *
 *  Created on: Jul 16, 2018
 *      Author: atabb
 */

#ifndef SOLVING_STRUCTURE_HPP_
#define SOLVING_STRUCTURE_HPP_

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "glog/logging.h"
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "multicamera.hpp"


using namespace Eigen;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace std;

enum PARAM_TYPE { Cali_Quaternion };

template <typename T>
void ConvertMatrixTo7QuaternionRepresentationx(Matrix4d& M, T* X){

	// read the R component into RX...RX has to be column major
	T RX[9];
	RX[0] = T(M(0,0));
	RX[1] = T(M(1,0));
	RX[2] = T(M(2,0));
	RX[3] = T(M(0,1));
	RX[4] = T(M(1,1));
	RX[5] = T(M(2,1));
	RX[6] = T(M(0,2));
	RX[7] = T(M(1,2));
	RX[8] = T(M(2,2));

	ceres::RotationMatrixToQuaternion(RX, X);

	// then the translation --
	X[4] = T(M(0, 3));
	X[5] = T(M(1, 3));
	X[6] = T(M(2, 3));
}



template <typename T>
void Convert7ParameterQuaternionRepresentationIntoMatrix(const T* X, T* XM){
	T RX[9];

	ceres::QuaternionToRotation(X, RX);

	XM[0] = RX[0];
	XM[1] = RX[1];
	XM[2] = RX[2];
	XM[3] = X[4];

	XM[4] = RX[3];
	XM[5] = RX[4];
	XM[6] = RX[5];
	XM[7] = X[5];

	XM[8] = RX[6];
	XM[9] = RX[7];
	XM[10] = RX[8];
	XM[11] = X[6];

	XM[12] = T(0);
	XM[13] = T(0);
	XM[14] = T(0);
	XM[15] = T(1);

}

template <typename T>
void Convert7ParameterQuaternionRepresentationIntoMatrix(const T* X, Matrix4d& M){
	T RX[9];

	ceres::QuaternionToRotation(X, RX);


	M(0,0) = RX[0];
	M(0,1) = RX[1];
	M(0,2) = RX[2];
	M(0,3) = X[4];

	M(1,0) = RX[3];
	M(1,1) = RX[4];
	M(1,2) = RX[5];
	M(1,3) = X[5];

	M(2,0) = RX[6];
	M(2,1) = RX[7];
	M(2,2) = RX[8];
	M(2,3) = X[6];

	M(3,0) = T(0);
	M(3,1) = T(0);
	M(3,2) = T(0);
	M(3,3) = T(1);

}


template <typename T>
void MatrixMultiply(const T* X, const T* Y, T* M, int row_col){

	// assuming square matrices
	int xi, yi;
	T r;
	for (int i = 0; i < row_col; i++){
		for (int j = 0; j < row_col; j++){
			// dot product the ith row of X by the jth column of Y

			r= T(0);
			for (int index = 0; index < row_col; index++){
				xi = i*row_col + index;
				yi = index*row_col + j;

				r += X[xi]*Y[yi];
			}
			M[i*row_col + j] = r;
		}
	}
}

template <typename T>
void AssignTo(const T* X, T* Y, int size){

	for (int i = 0; i < size; i++){
		Y[i] = X[i];
	}
}

template <typename T>
void MatrixMultiply(const T* X, const T* Y, T* M, int row0, int col0, int row1, int col1){

	// not assuming square matrices
	if (col0 != row1){
		cout << "Wrong args sent to Matrix Multiply: " << endl;
		cout << row0 << ", " << col0 << ", " << row1 << ", " << col1 << endl;
	}

	int xi, yi;
	T r;
	for (int i = 0; i < row0; i++){
		for (int j = 0; j < col1; j++){
			// dot product the ith row of X by the jth column of Y, results in the


			r= T(0);
			for (int index = 0; index < col0; index++){
				xi = i*col0 + index; // walk across the row
				yi = index*col1 + j; // walk down the columm

				r += X[xi]*Y[yi];
			}

			M[i*col1 + j] = r;
		}
	}
}

template <typename T>
void PrintMatrix(const T* X, int row0, int col0){

	for (int i = 0, index = 0; i < row0; i++){
		for (int j = 0; j < col0; j++, index++){
			cout << X[index] << " ";
		}
		cout << endl;
	}

}



void CopyFromCalibration(vector<CameraCali*>& CCV, double* camera_params);

void BundleAdjustmentCaliCeres(MCcali& MC, vector<CameraCali*>& CCV, double* camera_params,
		vector<bool>& bit_vector_true_min, std::ofstream& out, int start_id, int end_id, bool use_all_points);

void ReconstructXFunctionIDsMC(MCcali& MC, vector<Matrix4d>& vector_variables, vector<CameraCali*>& CCV, double* camera_params,
		vector< Vector3d >& estimated_threed_points, vector< bool >& has_values, std::ofstream& out);

void MinimizeReprojectionError(MCcali& MC, vector<CameraCali*>& CCV, double* camera_params,
		vector<bool>& bit_vector_true_min, std::ofstream& out, int start_id, int end_id, bool use_all_points_present);

void SolveWithShahsMethod(Matrix4d& Result, vector<Matrix4d>& LHS, vector<Matrix4d>& RHS, bool verbose);


struct MultiCameraReprojectionError {
	MultiCameraReprojectionError(double* camera_parameters, double* twoDpoints, double* threeDpoints, double* vars_initial,
			bool* bitvector_vars_min, PARAM_TYPE param_type,
			int number_variables, int c_index, int p_index, int t_index, double* weighting):
				camera_parameters(camera_parameters), twoDpoints(twoDpoints), threeDpoints(threeDpoints), vars_initial(vars_initial), bitvector_vars_min(bitvector_vars_min),
				param_type(param_type), number_variables(number_variables), c_index(c_index), p_index(p_index), t_index(t_index), weighting(weighting){
	}

	template <typename T>

	bool operator()(const T* const camera_var,
			const T* const pattern_var, const T* const time_var,
			T* residuals) const {


		// A = C * Tinv * Pinv T and P are already in inverse

		T CM[16];
		T PM[16];
		T tM[16];

		T C[7];
		T P[7];
		T t[7];



		T w = T(sqrt(*weighting));


		for (int i = 0; i < 7; i++){
			C[i] = T(camera_var[i]);
			P[i] =  T(pattern_var[i]);
			t[i] =  T(time_var[i]);

		}

		switch (param_type){
		case Cali_Quaternion: {
			if (bitvector_vars_min[c_index]){
				// copy the var
				Convert7ParameterQuaternionRepresentationIntoMatrix(C, CM);
			}	else {
				// copy the constant
				for (int i = 0; i < 16; i++){
					CM[i] = T(vars_initial[16*c_index + i]);
				}
			}

			if (bitvector_vars_min[p_index]){
				// copy the var
				Convert7ParameterQuaternionRepresentationIntoMatrix(P, PM);
			}	else {
				// copy the constant
				for (int i = 0; i < 16; i++){
					PM[i] = T(vars_initial[16*p_index + i]);
				}
			}

			if (bitvector_vars_min[t_index]){
				// copy the var
				Convert7ParameterQuaternionRepresentationIntoMatrix(t, tM);
			}	else {
				// copy the constant
				for (int i = 0; i < 16; i++){
					tM[i] = T(vars_initial[16*t_index + i]);
				}
			}

		}	break;
		default:
		{
			cout << "Other options not currently handled, quitting" << endl;
			exit(1);
		}
		}

		T term0[16];


		// have to convert to T
		T A_hat_T[16];
		T K[9];
		T k1, k2, p1, p2, k3, k4, k5, k6;
		T r_sqr;

		T Xp[4];
		T xp[3];
		T xpp[3];



		for (int i = 0; i < 9; i++){
			K[i] = T(0);
		}

		K[8] = T(1);

		// set up A_hat
		MatrixMultiply(CM, tM, term0, 4);
		MatrixMultiply(term0, PM, A_hat_T, 4);


		// copy over parameters

		// don't use the parameter version for camera cali -- this amtrix is sent all set up for use ....
		K[0] = T(camera_parameters[0]);
		K[2] = T(camera_parameters[1]);
		K[4] = T(camera_parameters[2]);
		K[5] = T(camera_parameters[3]);
		k1 = T(camera_parameters[4]);
		k2 = T(camera_parameters[5]);
		p1 = T(camera_parameters[6]);
		p2 = T(camera_parameters[7]);
		k3 = T(camera_parameters[8]);
		k4 = T(camera_parameters[9]);
		k5 = T(camera_parameters[10]);
		k6 = T(camera_parameters[11]);

		for (int j = 0; j < 3; j++){
			Xp[j] = T(threeDpoints[j]);
		}
		Xp[3] = T(1);

		MatrixMultiply(A_hat_T, Xp, xp, 3, 4, 4, 1);

		xp[0] = xp[0]/xp[2];
		xp[1] = xp[1]/xp[2];

		r_sqr= xp[0]*xp[0] + xp[1]*xp[1];

		T numer = (T(1) + k1*r_sqr + k2*r_sqr*r_sqr + k3*r_sqr*r_sqr*r_sqr);
		T denom = (T(1) + k4*r_sqr + k5*r_sqr*r_sqr + k6*r_sqr*r_sqr*r_sqr);

		xpp[0] = xp[0]*numer/denom + T(2)*p1*xp[0]*xp[1] + p2*(r_sqr + T(2)*xp[0]*xp[0]);
		xpp[1] = xp[1]*numer/denom + T(2)*p2*xp[0]*xp[1] + p1*(r_sqr + T(2)*xp[1]*xp[1]);

		T predicted_x = (xpp[0]*K[0] + K[2]);
		T predicted_y = (xpp[1]*K[4] + K[5]);

		residuals[0] = w*(predicted_x - T(twoDpoints[0]));
		residuals[1] = w*(predicted_y - T(twoDpoints[1]));

		return true;
	}

	static ceres::CostFunction* Create(double* camera_parameters, double* twoDpoints, double* threeDpoints, double* vars_initial,
			bool* bitvector_vars_min, PARAM_TYPE param_type,
			int number_variables, int c_index, int p_index, int t_index, double* weighting) {

		return (new ceres::AutoDiffCostFunction<MultiCameraReprojectionError, 2, 7, 7, 7>(
				new MultiCameraReprojectionError(camera_parameters, twoDpoints, threeDpoints, vars_initial,
						bitvector_vars_min,
						param_type,  number_variables, c_index, p_index, t_index, weighting)));
	}


	double* camera_parameters;
	double* twoDpoints;
	double* threeDpoints;
	double* vars_initial;
	bool* bitvector_vars_min;

	PARAM_TYPE param_type;
	int number_variables;
	int c_index; // within var space
	int p_index; // within var space inverted
	int t_index; /// within var space, inverted
	double* weighting;
};


struct ReconstructXStruct {
	ReconstructXStruct(double* camera_parameters, double* transformation_matrix, double twoDpoints_x, double twoDpoints_y,
			int number_points, bool individual, int this_point):
				camera_parameters(camera_parameters), transformation_matrix(transformation_matrix), twoDpoints_x(twoDpoints_x),
				twoDpoints_y(twoDpoints_y),
				number_points(number_points), individual(individual), this_point(this_point){
	}

	template <typename T>

	bool operator()(const T* const parameters,
			T* residuals) const {
		// the parameters are the threed points

		// have to convert to T
		T A_hat_T[16];

		T K[9];
		T k1, k2, p1, p2, k3, k4, k5, k6;
		T r_sqr;
		T Xp[4];
		T xp[3];
		T xpp[3];


		// read in transformation matric
		for (int i = 0; i < 16; i++){
			A_hat_T[i] = T(transformation_matrix[i]);
		}

		// Read in camera calibration matrix
		for (int i = 0; i < 9; i++){
			K[i] = T(0);
		}
		K[8] = T(1);

		K[0] = T(camera_parameters[0]);
		K[2] = T(camera_parameters[1]);
		K[4] = T(camera_parameters[2]);
		K[5] = T(camera_parameters[3]);
		k1 = T(camera_parameters[4]);
		k2 = T(camera_parameters[5]);
		p1 = T(camera_parameters[6]);
		p2 = T(camera_parameters[7]);
		k3 = T(camera_parameters[8]);
		k4 = T(camera_parameters[9]);
		k5 = T(camera_parameters[10]);
		k6 = T(camera_parameters[11]);


		for (int j = 0; j < 3; j++){
			Xp[j] = T(parameters[j]);
		}
		Xp[3] = T(1);

		MatrixMultiply(A_hat_T, Xp, xp, 3, 4, 4, 1);

		xp[0] = xp[0]/xp[2];
		xp[1] = xp[1]/xp[2];

		r_sqr= xp[0]*xp[0] + xp[1]*xp[1];

		T numer = (T(1) + k1*r_sqr + k2*r_sqr*r_sqr + k3*r_sqr*r_sqr*r_sqr);
		T denom = (T(1) + k4*r_sqr + k5*r_sqr*r_sqr + k6*r_sqr*r_sqr*r_sqr);

		xpp[0] = xp[0]*numer/denom + T(2)*p1*xp[0]*xp[1] + p2*(r_sqr + T(2)*xp[0]*xp[0]);
		xpp[1] = xp[1]*numer/denom + T(2)*p2*xp[0]*xp[1] + p1*(r_sqr + T(2)*xp[1]*xp[1]);

		T predicted_x = (xpp[0]*K[0] + K[2]);
		T predicted_y = (xpp[1]*K[4] + K[5]);

		residuals[0] = (predicted_x - T(twoDpoints_x));
		residuals[1] = (predicted_y - T(twoDpoints_y));


		return true;
	}

	static ceres::CostFunction* Create(double* camera_parameters, double* transformation_matrix, double twoDpoint_x, double twoDpoint_y) {
		return (new	ceres::AutoDiffCostFunction<ReconstructXStruct, 2, 3>(new
				ReconstructXStruct(camera_parameters, transformation_matrix, twoDpoint_x, twoDpoint_y, 1, true, 0)));
	}


	// we need the camera parameters, transformation matrices (one per camera), and twoDpoints for each camera that can see the pattern.
	// for now, solve all together as one big cost function
	// result is the three d points for the pattern.
	double* camera_parameters;
	double* transformation_matrix;
	double twoDpoints_x;
	double twoDpoints_y;

	int number_points;
	bool individual;
	int this_point;

};


#endif /* SOLVING_STRUCTURE_HPP_ */

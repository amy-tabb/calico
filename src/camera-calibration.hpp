/*
 * camera_calibration.hpp
 *
 *  Created on: Jun 26, 2018
 *      Author: atabb
 */

#ifndef CAMERA_CALIBRATION_HPP_
#define CAMERA_CALIBRATION_HPP_

#include "Includes.hpp"
#include "patterns.hpp"

class CameraCali{
public:
	// so the patterns create an ordering of the points ... 0 ... n-1. #1 n to 2n.
	// We have a map in PatternsCreated in case there is a pattern with an unusual number of squares.
	vector<vector<bool> > points_present; //charuco
	vector<vector<bool> > points_used_min; //charuco

	vector<MatrixXd> two_d_point_coordinates_dense; /// this is initialized to 0, 0.  points present tell us whether a point is there or not.
	vector<vector<bool> > boards_detected; /// theoretically, more than one visible in each frame.
	vector< vector<double> > reproj_error_per_board;

	vector<vector<bool> > has_calibration_estimate;
	vector<string> im_names;

	vector<MatrixXd> internal_two_d_point_coordinates_dense;
	int count_internal_ids_present;

	int max_internals_use;
	vector<bool> id_bool;
	double pixel_width;
	int rows;
	int cols;
	int start_time_this_camera;

	vector<Mat> images; // the calibration images in the set
	int number_external_images_max;
	vector<Mat> reproject_cam_cali_images;

	Matrix3d internal_parameters;
	VectorXd distortion;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	vector<vector< Matrix4d> > external_parameters;

	PatternsCreated* P_class;

	vector<int> internal_pattern_indices;

	CameraCali(const string& read_dir, PatternsCreated* P, int max_ext_images,
	        int max_int_images_read, int max_int_images_use);

	~CameraCali();

	void FindCorners(const string& write_dir, bool write_internal_images = false);

	void FindCornersCharuco(const string& write_dir, bool write_internal_images );

    void FindCornersApril(const string& write_dir, bool write_internal_images );

	void SetUpSelectPointsForMinimization();

	void CalibrateBasic(float initial_focal_px, int zero_tangent_dist, int zero_k3, int fix_principal_point,
			const string& write_dir, int number_points_needed_to_count, bool write_internal_images = false);

	double ComputeReprojectionErrorOneImagePattern(const Matrix4d& ExtParameters, int image_number,
			int pattern_number,  const string& write_directory, bool write, int equation_number,
			bool rotating, const Matrix3d* IntParameters);

	int PointsAtImagePattern(int image_number, int pattern_number);

};

#endif /* CAMERA_CALIBRATION_HPP_ */

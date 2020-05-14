/*
 * camera_calibration.hpp
 *
 *  Created on: Jun 26, 2018
 *      Author: atabb
 */

#ifndef CAMERA_CALIBRATION_HPP_
#define CAMERA_CALIBRATION_HPP_

#include "Includes.hpp"

class PatternsCreated{
public:
	vector<vector<int> > double_to_single;
	vector< cv::Point3f> three_d_points;
	vector< cv::Point3f> three_d_points_internal; // for strawberry case
	Ptr<aruco::Dictionary> dictionary;
	vector<cv::Ptr<cv::aruco::CharucoBoard> > boards; /// for refining the estimate of corner locations
	Ptr<aruco::DetectorParameters> detectorParams;
	vector< vector<int> > display_colors;
	vector<pair<int, int> > min_max_id_pattern;
	vector<pair<int, int> > min_max_id_squares;
	vector<pair<int, int> > square_h_w;
	bool rotate_case;
	vector<int> single_aruco_ids;
	int max_internal_patterns;
	int internalx, internaly;

	PatternsCreated(const string& read_dir, const string& write_dir, bool rotate,
	        const string& src_file, bool generate_only);

	void DetermineBoardsPresentFromMarkerList(const vector<int>& markers, vector<bool>& boards_seen);
	void DetermineBoardsPresentFromMarkerList(const vector<int>& markers, vector<bool>& boards_seen,
			const vector< vector< Point2f > >& corners, vector< vector< vector< Point2f > > >& corners_sorted,
			vector< vector<int> >& markers_sorted);
	int MappingArucoIDToPatternNumber(int id);

	Scalar Color(int index);

	int NumberMarkers() const;
	int NumberSquares() const;
	int NumberPatterns() const;

	void SetNumberMarkers(int n);
	void SetNumberPatterns(int n);

protected:
	int number_patterns;
	int number_total_squares;
	int int_number_markers;

};

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

	void FindCornersCharuco(const string& write_dir, bool write_internal_images = false);

	void FindCornersArucoCharuco(const string& write_dir, bool verbose = true, string verbose_write_dir = "");

	void ReadCorners(const string& read_dir);

	void ReadCalibration(const string& read_dir);

	void SetUpSelectPointsForMinimization();

	void CalibrateBasic(float initial_focal_px, int zero_tangent_dist, int zero_k3, int fix_principal_point,
			const string& write_dir, int number_points_needed_to_count, bool write_internal_images = false);

	void CalibrateRotatingSet(const string& write_dir, int number_points_needed_to_count);

	double ComputeReprojectionErrorOneImagePattern(const Matrix4d& ExtParameters, int image_number,
			int pattern_number,  const string& write_directory, bool write, int equation_number,
			bool rotating, const Matrix3d* IntParameters);

	double ComputeReprojectionErrorOneImagePatternGTDRel(const Matrix4d& ExtParameters, int image_number,
			int pattern_number,
			const string& write_directory, bool write, int equation_number, bool rotating,
			const vector<Vector3d>& points_from_gtd);

	void ReadExifInformationRotatingSet(const string& input_dir, const string& read_dir);

	int PointsAtImagePattern(int image_number, int pattern_number);

	int PointsForMinimization(int image_number, int pattern_number);
};

bool readDetectorParameters(const string& filename, Ptr<aruco::DetectorParameters> &params);

void WritePatterns(double* pattern_points, int chess_h, int chess_w, int index_number, string outfile);

string CreatePaddedNumberString(int number, int length);

int CreateStrawberryImagesCharucoExp(vector<Mat>& images, int squaresX, int squaresY, int squareLength, int markerLength,
		int margins, int id_start_number);

int CreateRotateCaseImagesCharuco(vector<Mat>& images, int squaresX, int squaresY, int squareLength, int markerLength,
		int margins, int id_start_number, int dictionary_version, string src_file);

Matrix3d CopyRotationMatrixFromExternal(const Matrix4d& M);

#endif /* CAMERA_CALIBRATION_HPP_ */

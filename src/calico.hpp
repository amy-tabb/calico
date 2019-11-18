#include "Includes.hpp"

void CreatePatterns(int calibration_context, string input_dir, string output_dir, string src_file);

void MultipleCameraCalibration(int calibration_context, string input_dir, string output_dir, string src_file,
		float camera_size, float track_size,  float initial_focal_px, int zero_tangent_dist, int zero_k3, int fix_principal_point,
		bool has_ground_truth, bool verbose, bool read_camera_calibration, bool only_camera_calibration,
		int max_internal_read, int max_internal_use, int max_external_positions, int selectedk,
		int number_points_needed_to_count_pattern, int synchronized_rotating_option);

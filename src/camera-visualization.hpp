/*
 * camera_visualization.hpp
 *
 *  Created on: Jun 18, 2018
 *      Author: atabb
 */

#ifndef CAMERA_VISUALIZATION_HPP_
#define CAMERA_VISUALIZATION_HPP_

#include "Includes.hpp"

int create_camera(const Matrix3d& internal, const Matrix4d& external, const Vector3d& C, int r, int g, int b,
        int rows, int cols,
		const string& ply_file, float camera_size);

int create_camera(const Matrix3d& internal, const Matrix4d& external, float camera_size, int r, int g, int b,
        int rows, int cols,
		const string& ply_file);

int create_camera( vector< Vector3d >& vertex_coordinates, vector< vector<int> >& face_indices,
		Matrix3d& internal, Matrix4d& external, Vector3d& C, int rows, int cols, float camera_size);

int create_cameras(const vector<Matrix3d>& internal, const vector<Matrix4d>& external, int r, int g, int b, int rows,
		int cols, const string& ply_file, float camera_size);

void WritePatternsSkips(const vector<Vector3d>& pattern_points, int index_number, const string& outfile);

void WritePatternsCharuco(const vector<Vector3d>& pattern_points, int chess_h, int chess_w, int index_number, const string& outfile);

void WritePatternsApril(const vector<Vector3d>& pattern_points, int chess_h, int chess_w,
        int index_number, const string& outfile);

Vector3d ReturnCenter(const Matrix4d& external);

int create_tracks(vector<Vector3d>& one_track, int r, int g, int b, float offset,
        const Vector3d& offset_vector, const string& ply_file);



#endif /* CAMERA_VISUALIZATION_HPP_ */

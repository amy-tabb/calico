/*
 *
 *
 *  Created on: Jul 9, 2009
 *      Author: atabb
 */

#include "camera-visualization.hpp"
#include "helper.hpp"

// used 11/24
void camera(const Matrix3d& Kinv, float max_u, float max_v, float mag, vector< Vector3d >& vertex_coordinates ) {

    Vector3d a;
    Vector3d b;
    Vector3d c;
    Vector3d d;

    Vector3d x;
    x << 0, 0, 1;

    a = mag*Kinv*x;

    x << max_u, 0, 1;
    b = mag*Kinv*x;

    x << max_u, max_v, 1;
    c = mag*Kinv*x;

    x << 0, max_v, 1;
    d = mag*Kinv*x;

    vertex_coordinates.push_back(a);
    vertex_coordinates.push_back(d);
    vertex_coordinates.push_back(c);
    vertex_coordinates.push_back(b);

}

//used 11/24
int create_camera(vector< Vector3d >& vertex_coordinates, vector< vector<int> >& face_indices,
        const Matrix3d& internal, const Matrix4d& external, const Vector3d& C, int rows, int cols, float camera_size)
{
    //////////////////////////roi box///////////////////////////////////

    Matrix3d R;
    Vector3d t;

    Matrix3d Kinv = internal.inverse();

    for (int r = 0; r < 3; r++){
        for (int c = 0; c < 3; c++){
            R(r, c) = external(r, c);
        }
        t(r) = external(r, 3);
    }

    Matrix3d Rinv = R.transpose();

    camera(Kinv, cols, rows, camera_size, vertex_coordinates);

    Vector3d tempV;

    for (int i = 0; i < 4; i++){
        tempV = vertex_coordinates[i];


        vertex_coordinates[i] = Rinv*(vertex_coordinates[i] - t);

    }

    Vector3d diff_vector;


    vertex_coordinates.push_back(C);


    Vector3d cp;

    int vertex_number = 0;

    // front face.
    vector<int> face;
    face.push_back(vertex_number);
    face.push_back(vertex_number + 3);
    face.push_back(vertex_number + 1);

    face_indices.push_back(face);


    face.clear();

    face.push_back(vertex_number + 2);
    face.push_back(vertex_number + 1);
    face.push_back(vertex_number + 3);

    face_indices.push_back(face);


    face.clear();
    // a side.

    face.push_back(vertex_number);
    face.push_back(vertex_number + 4);
    face.push_back(vertex_number + 3);


    face_indices.push_back(face);

    face.clear();

    face.push_back(vertex_number);
    face.push_back(vertex_number + 1);
    face.push_back(vertex_number + 4);


    face_indices.push_back(face);

    face.clear();

    face.push_back(vertex_number + 1);
    face.push_back(vertex_number + 2);
    face.push_back(vertex_number + 4);


    face_indices.push_back(face);

    face.clear();

    face.push_back(vertex_number + 2);
    face.push_back(vertex_number + 3);
    face.push_back(vertex_number + 4);


    face_indices.push_back(face);

    face.clear();


    vertex_number += 5;

    return 0;
}
// used 11/24
int create_camera(const Matrix3d& internal, const Matrix4d& external, const Vector3d& C, int r, int g, int b,
        int rows, int cols,
        const string& ply_file, float camera_size)
{

    vector< Vector3d > vertex_coordinates;
    vector< Vector3d > vertex_normals;
    vector< vector<int> > face_indices;
    vector< vector<int> > edge_indices;
    vector<Vector3d> colors;

    //vector< Vector3d >& vertex_coordinates_external;
    //////////////////////////roi box///////////////////////////////////

    Matrix3d R;
    Vector3d t;

    Matrix3d Kinv = internal.inverse();

    for (int r = 0; r < 3; r++){
        for (int c = 0; c < 3; c++){
            R(r, c) = external(r, c);
        }
        t(r) = external(r, 3);
    }

    Matrix3d Rinv = R.transpose();


    camera(Kinv, cols, rows, camera_size, vertex_coordinates);

    Vector3d tempV;

    for (int i = 0; i < 4; i++){
        tempV = vertex_coordinates[i];

        //vertex_coordinates[i] = Rinv*(vertex_coordinates[i] - t);
        vertex_coordinates[i] = Rinv*(tempV - t);

        if (isnan(vertex_coordinates[i](0)) || isnan(vertex_coordinates[i](1)) || isnan(vertex_coordinates[i](2))){
            cout << "vertex is NAN! " << i << endl;
            cout << __LINE__ << " line in " << __FILE__ << endl;
            cout << "Rinv" << endl << Rinv << endl;
            cout << "tempV" << endl << tempV << endl;
            cout << "t " << endl << t << endl;
            cout << "vertex coord " << endl << vertex_coordinates[i] << endl;
            cout << "size of vertex coords " << vertex_coordinates.size() << endl;
            cout << "Kinv " << endl << Kinv << endl;
            cout << "internal " << endl << internal << endl;
            cout << "external " << endl << external << endl;
            exit(1);
        }

    }

    Vector3d diff_vector;

    vertex_coordinates.push_back(C);


    Vector3d cp;

    int vertex_number = 0;

    // front face.
    vector<int> face;
    face.push_back(vertex_number);
    face.push_back(vertex_number + 3);
    face.push_back(vertex_number + 1);

    face_indices.push_back(face);


    face.clear();

    face.push_back(vertex_number + 2);
    face.push_back(vertex_number + 1);
    face.push_back(vertex_number + 3);

    face_indices.push_back(face);


    face.clear();
    // a side.

    face.push_back(vertex_number);
    face.push_back(vertex_number + 4);
    face.push_back(vertex_number + 3);


    face_indices.push_back(face);

    face.clear();

    face.push_back(vertex_number);
    face.push_back(vertex_number + 1);
    face.push_back(vertex_number + 4);


    face_indices.push_back(face);

    face.clear();

    face.push_back(vertex_number + 1);
    face.push_back(vertex_number + 2);
    face.push_back(vertex_number + 4);


    face_indices.push_back(face);

    face.clear();

    face.push_back(vertex_number + 2);
    face.push_back(vertex_number + 3);
    face.push_back(vertex_number + 4);


    face_indices.push_back(face);

    face.clear();


    vertex_number += 5;


    std::ofstream out;
    out.open(ply_file.c_str());


    out << "ply" << endl;
    out << "format ascii 1.0" << endl;
    out << "element vertex " << vertex_coordinates.size() << endl;
    out << "property float x" << endl;
    out << "property float y" << endl;
    out << "property float z" << endl;
    out << "property uchar red" << endl;
    out << "property uchar green" << endl;
    out << "property uchar blue" << endl;
    out << "property uchar alpha" << endl;
    out << "element face " << face_indices.size() << endl;
    out << "property list uchar int vertex_indices"<< endl;

    out << "end_header" << endl;

    for (int i = 0, nc = vertex_coordinates.size(); i < nc; i++){
        out << vertex_coordinates[i](0) << " " << vertex_coordinates[i](1) << " " << vertex_coordinates[i](2) << " ";

        if (i == 0){
            out << " 255 255 255 255" << endl;
        }	else {
            out << r << " " << g << " " << b << " 255 " << endl;
        }


    }

    for (int i = 0; i < int(face_indices.size()); i++){

        out << face_indices[i].size() << " ";


        for (int j = 0; j < int(face_indices[i].size()); j++){
            out << face_indices[i].at(j) << " ";
        }


        out << endl;

    }


    out.close();

    return(0);

}

//used 11/24
Vector3d ReturnCenter(const Matrix4d& external){

    Vector3d C;
    Vector3d t;
    Matrix3d R;
    // t = -RC, so C = -R'*t
    for (int r0 = 0; r0 < 3; r0++){
        for (int c = 0; c < 3; c++){
            R(r0, c) = external(r0, c);
        }

        t(r0) = external(r0, 3);
    }


    C = -R.transpose()*t;

    return C;

}
//used 11/24
int create_camera(const Matrix3d& internal, const Matrix4d& external, float camera_size, int r, int g, int b,
        int rows, int cols, const string& ply_file){

    Vector3d C;
    Vector3d t;
    Matrix3d R;
    // t = -RC, so C = -inv(R)*t
    for (int r0 = 0; r0 < 3; r0++){
        for (int c = 0; c < 3; c++){
            R(r0, c) = external(r0, c);
        }

        t(r0) = external(r0, 3);
    }


    C = -R.transpose()*t;

    create_camera(internal, external, C, r, g, b, rows, cols, ply_file, camera_size);

    return 0;
}

//used 11/24
int create_cameras(const vector<Matrix3d>& internal, const vector<Matrix4d>& external, int r, int g, int b, int rows,
        int cols, const string& ply_file, float camera_size){

    vector< Vector3d > vertex_coordinates;
    vector< vector<int> > face_indices;
    vector< vector<int> > edge_indices;
    Vector3d C;
    Vector3d t;
    Matrix3d R;

    int number_cameras = external.size();

    for (int i = 0; i < number_cameras; i++){

        for (int r0 = 0; r0 < 3; r0++){
            for (int c = 0; c < 3; c++){
                R(r0, c) = external[i](r0, c);
            }

            t(r0) = external[i](r0, 3);
        }

        C = -R.inverse()*t;


        vector< Vector3d > local_vertex_coordinates;
        vector< vector<int> > local_face_indices;

        create_camera( local_vertex_coordinates, local_face_indices, internal[i], external[i], C, rows, cols, camera_size);

        // now, translation to the global vertex indices.
        int vertex_count = vertex_coordinates.size();

        for (int j = 0, jn = local_vertex_coordinates.size(); j < jn; j++){
            vertex_coordinates.push_back(local_vertex_coordinates[j]);
        }

        for (int j = 0, jn = local_face_indices.size(); j < jn; j++){
            for (int k = 0, kn = local_face_indices[j].size(); k < kn; k++){
                local_face_indices[j][k] += vertex_count;
            }

            face_indices.push_back(local_face_indices[j]);
        }

    }

    std::ofstream out;
    out.open(ply_file.c_str());


    out << "ply" << endl;
    out << "format ascii 1.0" << endl;
    out << "element vertex " << vertex_coordinates.size() << endl;
    out << "property float x" << endl;
    out << "property float y" << endl;
    out << "property float z" << endl;
    out << "property uchar red" << endl;
    out << "property uchar green" << endl;
    out << "property uchar blue" << endl;
    out << "property uchar alpha" << endl;
    out << "element face " << face_indices.size() << endl;
    out << "property list uchar int vertex_indices"<< endl;

    out << "end_header" << endl;

    for (int i = 0, nc = vertex_coordinates.size(); i < nc; i++){
        out << vertex_coordinates[i](0) << " " << vertex_coordinates[i](1) << " " << vertex_coordinates[i](2) << " ";

        if (i == 0){
            out << " 255 255 255 255" << endl;
        }	else {
            out << r << " " << g << " " << b << " 255 " << endl;
        }


    }

    for (int i = 0; i < int(face_indices.size()); i++){

        out << face_indices[i].size() << " ";


        for (int j = 0; j < int(face_indices[i].size()); j++){
            out << face_indices[i].at(j) << " ";
        }


        out << endl;

    }


    out.close();


    return 0;
}
//used 11/24
int create_tracks(vector<Vector3d>& one_track, int r, int g, int b, float offset,
        const Vector3d& offset_vector, const string& ply_file){

    vector< Vector3d > vertex_coordinates;
    vector< vector<int> > face_indices;
    vector< vector<int> > edge_indices;
    Vector3d C;
    Vector3d t;
    Matrix3d R;

    int number_cameras = one_track.size();

    for (int i = 0; i < number_cameras; i++){

        C = one_track[i];
        t = C + offset*offset_vector;

        vertex_coordinates.push_back(C);
        vertex_coordinates.push_back(t);

        if (i > 0){
            // create faces ... at this iter, created i*2 and i*2 + 1
            // want to connect
            // i*2 with
            vector<int> tri1;
            tri1.push_back((i-1)*2);
            tri1.push_back((i)*2);
            tri1.push_back((i)*2 + 1);

            vector<int> tri2;
            tri2.push_back((i-1)*2);
            tri2.push_back((i - 1)*2 + 1);
            tri2.push_back((i)*2 + 1);


            face_indices.push_back(tri1);
            face_indices.push_back(tri2);
        }

    }

    std::ofstream out;
    out.open(ply_file.c_str());


    out << "ply" << endl;
    out << "format ascii 1.0" << endl;
    out << "element vertex " << vertex_coordinates.size() << endl;
    out << "property float x" << endl;
    out << "property float y" << endl;
    out << "property float z" << endl;
    out << "property uchar red" << endl;
    out << "property uchar green" << endl;
    out << "property uchar blue" << endl;
    out << "property uchar alpha" << endl;
    out << "element face " << face_indices.size() << endl;
    out << "property list uchar int vertex_indices"<< endl;

    out << "end_header" << endl;

    for (int i = 0, nc = vertex_coordinates.size(); i < nc; i++){
        out << vertex_coordinates[i](0) << " " << vertex_coordinates[i](1) << " " << vertex_coordinates[i](2) << " ";

        if (i == 0){
            out << " 255 255 255 255" << endl;
        }	else {
            out << r << " " << g << " " << b << " 255 " << endl;
        }


    }

    for (int i = 0; i < int(face_indices.size()); i++){

        out << face_indices[i].size() << " ";


        for (int j = 0; j < int(face_indices[i].size()); j++){
            out << face_indices[i].at(j) << " ";
        }

        out << endl;
    }


    out.close();


    return 0;
}


//used 11/24
void WritePatternsCharuco(const vector<Vector3d>& pattern_points, int chess_h, int chess_w,
        int index_number, const string& outfile){


    // each vertex needs a color ....
    // first, find the range ....

    vector<vector<int> > colors;

    vector<int> c(3);
    // lowest value is grey
    c[0] = 0;
    c[1] = 0;
    c[2] = 0;

    colors.push_back(c);

    // next lowest value is purple
    c[0] = 128;
    c[1] = 0;
    c[2] = 128;
    colors.push_back(c);

    // next lowest value is blue
    c[0] = 0;
    c[1] = 0;
    c[2] = 200;
    colors.push_back(c);

    // next lowest value is cyan
    c[0] = 0;
    c[1] = 255;
    c[2] = 255;
    colors.push_back(c);

    // next lowest value is green
    c[0] = 0;
    c[1] = 255;
    c[2] = 0;
    colors.push_back(c);

    // next lowest value is yellow
    c[0] = 255;
    c[1] = 255;
    c[2] = 0;
    colors.push_back(c);

    // next lowest value is red
    c[0] = 255;
    c[1] = 0;
    c[2] = 0;
    colors.push_back(c);

    c = colors[index_number % colors.size()];


    cout << "Writing to " << outfile << endl;
    std::ofstream out;
    out.open(outfile.c_str());

    // first row has chess_h - 2 faces ?
    int number_faces = (chess_h/2)*(chess_w/2) + (chess_h/2 - 1)*(chess_w/2 - 1);


    out << "ply" << endl;
    out << "format ascii 1.0" << endl;
    out << "element vertex " << chess_h*chess_w << endl;
    out << "property float x" << endl;
    out << "property float y" << endl;
    out << "property float z" << endl;
    out << "property uchar red" << endl;
    out << "property uchar green" << endl;
    out << "property uchar blue" << endl;
    out << "property uchar alpha" << endl;
    out << "element face " << number_faces << endl;
    out << "property list uchar int vertex_indices"<< endl;
    out << "end_header" << endl;

    for (int i = 0; i < chess_h*chess_w; i++){
        out << pattern_points[i](0) << " " << pattern_points[i](1) << " " << pattern_points[i](2) << " " << c[0] << " " << c[1] << " " << c[2] << " 175" << endl;
    }


    int p0, p1, p2, p3;
    for (int i = 0; i < chess_h; i++){

        for (int j = 0; j < chess_w/2; j++){
            if (i % 2  == 0){
                p0 = i*chess_w + 2*j;
                p1 = i*chess_w + 2*j + 1;
                p2 = (i + 1)*chess_w + 2*j + 1;
                p3 = (i + 1)*chess_w + 2*j;
                out << "4 " << p0 << " " << p1 << " " << p2 << " " << p3 << endl;

            }	else {
                if (j < chess_w/2 - 1){
                    p0 = i*chess_w + 2*j + 1;
                    p1 = i*chess_w + 2*j + 2;
                    p2 = (i + 1)*chess_w + 2*j + 2;
                    p3 = (i + 1)*chess_w + 2*j + 1;
                    out << "4 " << p0 << " " << p1 << " " << p2 << " " << p3 << endl;

                }
            }


        }
    }


    out << endl;

    out.close();
}
//used 11/24
void WritePatternsApril(const vector<Vector3d>& pattern_points, int chess_h, int chess_w,
        int index_number, const string& outfile){


    // each vertex needs a color ....
    // first, find the range ....

    vector<vector<int> > colors;

    vector<int> c(3);
    // lowest value is grey
    c[0] = 0;
    c[1] = 0;
    c[2] = 0;

    colors.push_back(c);

    // next lowest value is purple
    c[0] = 128;
    c[1] = 0;
    c[2] = 128;
    colors.push_back(c);

    // next lowest value is blue
    c[0] = 0;
    c[1] = 0;
    c[2] = 200;
    colors.push_back(c);

    // next lowest value is cyan
    c[0] = 0;
    c[1] = 255;
    c[2] = 255;
    colors.push_back(c);

    // next lowest value is green
    c[0] = 0;
    c[1] = 255;
    c[2] = 0;
    colors.push_back(c);

    // next lowest value is yellow
    c[0] = 255;
    c[1] = 255;
    c[2] = 0;
    colors.push_back(c);

    // next lowest value is red
    c[0] = 255;
    c[1] = 0;
    c[2] = 0;
    colors.push_back(c);

    c = colors[index_number % colors.size()];


    cout << "Writing to " << outfile << endl;
    std::ofstream out;
    out.open(outfile.c_str());


    int number_faces = chess_h*chess_w;


    out << "ply" << endl;
    out << "format ascii 1.0" << endl;
    out << "element vertex " << chess_h*chess_w*4 << endl;
    out << "property float x" << endl;
    out << "property float y" << endl;
    out << "property float z" << endl;
    out << "property uchar red" << endl;
    out << "property uchar green" << endl;
    out << "property uchar blue" << endl;
    out << "property uchar alpha" << endl;
    out << "element face " << number_faces << endl;
    out << "property list uchar int vertex_indices"<< endl;
    out << "end_header" << endl;

    for (int i = 0; i < chess_h*chess_w*4; i++){
        out << pattern_points[i](0) << " " << pattern_points[i](1) << " " << pattern_points[i](2) << " " << c[0] << " " << c[1] << " " << c[2] << " 175" << endl;
    }


    int p0, p1, p2, p3;

    for (int r = 0; r < chess_h*2; r= r + 2){
        for (int c = 0; c < chess_w; c++){
            p0 = 2*r*chess_w + 2*c;    // 1st row
            p1 = 2*r*chess_w + 2*c + 1;
            p2 = 2*(r + 1)*chess_w + 2*c +1; // 2nd row
            p3 = 2*(r + 1)*chess_w + 2*c;
            out << "4 " << p0 << " " << p1 << " " << p2 << " " << p3 << endl;
        }
    }


    out << endl;

    out.close();
}


// used 1//24
void WritePatternsSkips(const vector<Vector3d>& pattern_points, int index_number, const string& outfile){

    // each vertex needs a color ....
    // first, find the range ....

    vector<vector<int> > colors;

    vector<int> c(3);
    // lowest value is grey
    c[0] = 0;
    c[1] = 0;
    c[2] = 0;

    colors.push_back(c);

    // next lowest value is purple
    c[0] = 128;
    c[1] = 0;
    c[2] = 128;
    colors.push_back(c);

    // next lowest value is blue
    c[0] = 255;
    c[1] = 0;
    c[2] = 0;
    colors.push_back(c);

    // next lowest value is cyan
    c[0] = 0;
    c[1] = 255;
    c[2] = 255;
    colors.push_back(c);

    // next lowest value is green
    c[0] = 0;
    c[1] = 255;
    c[2] = 0;
    colors.push_back(c);

    // next lowest value is yellow
    c[0] = 255;
    c[1] = 255;
    c[2] = 0;
    colors.push_back(c);

    // next lowest value is red
    c[0] = 255;
    c[1] = 0;
    c[2] = 0;
    colors.push_back(c);

    c = colors[index_number % colors.size()];

    if (pattern_points.size() > 0){

        cout << "Writing to " << outfile << endl;
        std::ofstream out;
        out.open(outfile.c_str());

        int number_faces = 0; //(chess_h/2)*(chess_w/2) + (chess_h/2 - 1)*(chess_w/2 - 1);


        out << "ply" << endl;
        out << "format ascii 1.0" << endl;
        out << "element vertex " << pattern_points.size() << endl;
        out << "property float x" << endl;
        out << "property float y" << endl;
        out << "property float z" << endl;
        out << "property uchar red" << endl;
        out << "property uchar green" << endl;
        out << "property uchar blue" << endl;
        out << "property uchar alpha" << endl;
        out << "element face " << number_faces << endl;
        out << "property list uchar int vertex_indices"<< endl;
        out << "end_header" << endl;

        for (uint i = 0; i < pattern_points.size(); i++){
            out << pattern_points[i](0) << " " << pattern_points[i](1) << " " << pattern_points[i](2) << " " << c[0] << " " << c[1] << " " << c[2] << " 175" << endl;
        }


        out << endl;

        out.close();

    }
}




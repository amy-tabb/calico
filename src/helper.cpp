/*
 * helper.cpp
 *
 *  Created on: Jul 31, 2019
 *      Author: Amy Tabb
 */

#include "helper.hpp"

// cleared Sat

profileInfo::profileInfo(){

    stream.open(file.c_str());

    TestStream(stream, file);
}

profileInfo::profileInfo(string f){
    if (stream.good()){
        stream.close();
    }
    doProfile = 1;
    file = f;
    stream.open(file.c_str());

    TestStream(stream, file);
}

//used
// this function quits if we encounter a bad file.
void TestStream(const ifstream& in, const string& filename){

    if (!in.good()){
        cout << "Bad filestream with filename " << filename << ", quitting" << endl;
        exit(1);
    }
}

void TestStream(const ofstream& out, const string& filename){

    if (!out.good()){
        cout << "Bad filestream with filename " << filename << ", quitting" << endl;
        exit(1);
    }
}
//used
bool CheckExistenceOfDirectory(string write_directory){

    bool exists= true;
    struct stat info;
    if( stat( write_directory.c_str(), &info ) != 0 ){
        cout << "Path to directory is wrong and/or cannot access " << write_directory << endl;
        exists = false;
    }

    return exists;

}
// used
string FindValueOfFieldInFile(const string& filename, const string& fieldTag, bool separator, bool kill_if_not_found){

    /// reopen file each time, in case things get switched around.  Assume that these are very small files, not the most efficient.

    ifstream in(filename.c_str());

    if (!in.good()){
        cout << "Filename to find " << fieldTag << " is bad " << filename << " quitting !" << endl;
        exit(1);
    }

    string cmp_str;
    string read_str;


    vector<string> tokens;
    string token;
    string return_str = "";
    bool found = false;


    while (in  && found == false){

        in >> token;

        if (token.compare(fieldTag) == 0){
            found = true;

            if (separator == true && in){
                in >> token;
            }

            if (in){
                in >> return_str;
            }

        }
    }


    cout << "Found! " << found << " field " << fieldTag << " and result " << return_str << endl;

    if (kill_if_not_found && !found){
        cout << "Not found, and kill if not found activated. Killing ... " << endl;
        exit(1);
    }
    in.close();

    return return_str;

}

//used
void EnsureDirHasTrailingBackslash(string& write_directory){

    int n_letters = write_directory.size();
    bool eval =  (write_directory[n_letters - 1] == '/');
    cout << "Last character compare " << write_directory << " " <<  eval << endl;
    if (eval == false){
        write_directory = write_directory + "/";
    }

}

//used
void CopyFile(const string& source, const string& destination){

    ifstream src(source.c_str(), std::ios::binary);
    ofstream dest(destination.c_str(), std::ios::binary);

    std::istreambuf_iterator<char> begin_source(src);
    std::istreambuf_iterator<char> end_source;
    std::ostreambuf_iterator<char> begin_dest(dest);
    std::copy(begin_source, end_source, begin_dest);

    src.close();
    dest.close();
}



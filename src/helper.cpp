/*
 * helper.cpp
 *
 *  Created on: Jul 31, 2019
 *      Author: Amy Tabb
 */

#include "helper.hpp"

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


string FindMultipleStringValueOfFieldInFile(const string& filename, const string& fieldTag,
        bool separator, bool kill_if_not_found, int number_items){

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
    int current_size = 0;
    string test_string;


    while (in  && found == false){

        in >> token;


        tokens.push_back(token);
        current_size++;

        if (current_size >= number_items){
            test_string = "";
            for (int j = current_size - number_items; j < current_size - 1; j++ ){
                test_string = test_string + tokens[j] + " ";
            }

            test_string = test_string + tokens[current_size - 1];

            if (test_string.compare(fieldTag) == 0){
                found = true;

                if (separator == true && in){
                    in >> token;
                }

                if (in){
                    in >> return_str;
                }

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


void EnsureDirHasTrailingBackslash(string& write_directory){
    int n_letters = write_directory.size();
    bool eval =  (write_directory[n_letters - 1] == '/');
    cout << "Last character compare " << write_directory << " " <<  eval << endl;
    if (eval == false){
        write_directory = write_directory + "/";
    }

}

bool sort_by_x(const pair<Point2f, Point2f>& p0, const pair<Point2f, Point2f>& p1){

    if (p0.first.x < p1.first.x){
        return true;
    }	else {
        return false;
    }
}

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



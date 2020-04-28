#include "DirectoryFunctions.hpp"
#include "Includes.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

bool SortbyLength(string s0, string s1){
	if (s0.length() < s1.length()){
		return true;
	}	else {
		if (s0.length() > s1.length()){
			return false;
		}	else {
			if (s0.compare(s1) < 0){
				return true;
			}	else {
				return false;
			}
		}
	}
}

bool SortAlphabetically(string s0, string s1){
	if (s0.compare(s1) <= 0){
		return true;
	}	else {
		return false;
	}
}


bool IsDirectory(string dir_name){

	struct stat info;
	bool is_dir = false;
	if( stat( dir_name.c_str(), &info ) != 0 ){

	}
	else if( info.st_mode & S_IFDIR ){  // S_ISDIR()
		is_dir = true;
	}

	return is_dir;

}

void ReadDirectory(string dir_name, vector<string>& content_names){

	struct dirent * dp;

	// Enter path to directory existing below

	DIR * dir = opendir (dir_name.c_str());
	string s;
	dp = readdir(dir);
	while ( dp != NULL) {

		if (dp->d_name[0] != '.'){
			s= dp->d_name;
			if (s.at(s.size() - 1) != '~'){
				content_names.push_back(dp->d_name);
			}
		}

		dp = readdir(dir);
	}
	closedir (dir);

	std::sort(content_names.begin(), content_names.end(), SortbyLength);

}
